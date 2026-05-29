// Copyright (c) 2026 jzw

#include "m_common/rtsp_interface/video_file_gst.hpp"

#include "rtsp_device_detect.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <mlogger/mlogger.hpp>

#if defined(RTSP_IF_HAS_GSTREAMER)
#include <glib.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#endif

namespace m_common
{

namespace
{
std::uint64_t vf_now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

#if defined(RTSP_IF_HAS_GSTREAMER)

bool gst_has_factory_nm(const char * name)
{
  GstElementFactory * f = gst_element_factory_find(name);
  if (f == nullptr) {
    return false;
  }
  gst_object_unref(f);
  return true;
}

bool vf_use_matroska_demux(const std::string & path)
{
  std::string lower;
  lower.reserve(path.size());
  for (unsigned char u : path) {
    lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(u))));
  }
  const auto dot = lower.rfind('.');
  const std::string ext = dot == std::string::npos ? std::string{} : lower.substr(dot);
  return ext == ".mkv" || ext == ".mka" || ext == ".webm";
}

std::uint32_t vf_read_be32(const std::uint8_t * p)
{
  return (static_cast<std::uint32_t>(p[0]) << 24U) | (static_cast<std::uint32_t>(p[1]) << 16U) |
         (static_cast<std::uint32_t>(p[2]) << 8U) | static_cast<std::uint32_t>(p[3]);
}

bool vf_box_type_is(const std::uint8_t * p, const char * fourcc)
{
  return std::memcmp(p, fourcc, 4) == 0;
}

/// ISO BMFF 分片 MP4（fMP4）：moov 内含 mvex 或顶层 moof。此类文件 query_duration 会分段增长，
/// 不可用 position/duration 判 EOF；循环宜整管 reopen 而非 qtdemux seek。
bool vf_probe_fragmented_qt(const std::string & path)
{
  if (vf_use_matroska_demux(path)) {
    return false;
  }
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return false;
  }
  std::vector<std::uint8_t> buf(262144U);
  in.read(reinterpret_cast<char *>(buf.data()), static_cast<std::streamsize>(buf.size()));
  const std::size_t n = static_cast<std::size_t>(in.gcount());
  if (n < 8U) {
    return false;
  }

  std::size_t pos = 0U;
  while (pos + 8U <= n) {
    const std::uint32_t sz = vf_read_be32(buf.data() + pos);
    const std::uint8_t * type = buf.data() + pos + 4U;
    if (sz < 8U) {
      break;
    }
    if (pos + static_cast<std::size_t>(sz) > n) {
      break;
    }
    if (vf_box_type_is(type, "moof") || vf_box_type_is(type, "mvex")) {
      return true;
    }
    if (vf_box_type_is(type, "moov")) {
      const std::size_t moov_end = pos + static_cast<std::size_t>(sz);
      for (std::size_t i = pos + 8U; i + 4U <= moov_end; ++i) {
        if (vf_box_type_is(buf.data() + i, "mvex")) {
          return true;
        }
      }
    }
    pos += static_cast<std::size_t>(sz);
  }
  return false;
}

namespace std_fs = std::filesystem;

std::uint64_t vf_file_mtime_ns(const std::string & path)
{
  std::error_code ec;
  const auto t = std_fs::last_write_time(std_fs::path(path), ec);
  if (ec) {
    return 0U;
  }
  return static_cast<std::uint64_t>(t.time_since_epoch().count());
}

std::string vf_shell_single_quote(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2U);
  out.push_back('\'');
  for (char c : s) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out.push_back(c);
    }
  }
  out.push_back('\'');
  return out;
}

std_fs::path vf_fmp4_remux_cache_dir()
{
  if (const char * home = std::getenv("HOME"); home != nullptr && home[0] != '\0') {
    return std_fs::path(home) / ".cache" / "usv" / "video_file_gst";
  }
  return std_fs::temp_directory_path() / "usv_video_file_gst";
}

std_fs::path vf_fmp4_remux_cache_path(const std::string & src)
{
  const std_fs::path sp(src);
  const std::size_t h =
    std::hash<std::string>{}(src) ^
    (std::hash<std::uint64_t>{}(vf_file_mtime_ns(src)) << 1U);
  std::ostringstream name;
  name << sp.stem().string() << '_' << std::hex << (h & 0xffffffffU) << ".faststart.mp4";
  return vf_fmp4_remux_cache_dir() / name.str();
}

bool vf_cache_is_fresh(const std::string & src, const std_fs::path & cache)
{
  if (!std_fs::exists(cache)) {
    return false;
  }
  std::error_code ec;
  const auto src_t = std_fs::last_write_time(std_fs::path(src), ec);
  if (ec) {
    return true;
  }
  const auto cache_t = std_fs::last_write_time(cache, ec);
  if (ec) {
    return false;
  }
  return cache_t >= src_t;
}

/// qtdemux 对无 mfra 的 fMP4 常只解首段（~数秒）即 EOS；ffmpeg copy+faststart 重封装后可正常 PLAYING。
std::string vf_ensure_fmp4_remux_cache(const std::string & src)
{
  const std_fs::path cache = vf_fmp4_remux_cache_path(src);
  if (vf_cache_is_fresh(src, cache)) {
    return cache.string();
  }
  std::error_code ec;
  std_fs::create_directories(cache.parent_path(), ec);

  const std::string cmd =
    "ffmpeg -y -v error -nostdin -i " + vf_shell_single_quote(src) +
    " -c copy -movflags +faststart " + vf_shell_single_quote(cache.string());
  if (std::system(cmd.c_str()) != 0) {
    throw std::runtime_error(
      "VideoFileGst: fMP4 remux failed (need ffmpeg in PATH). "
      "Manual fix: ffmpeg -i <in> -c copy -movflags +faststart <out> "
      "or set file_decode=opencv");
  }
  if (!std_fs::exists(cache)) {
    throw std::runtime_error("VideoFileGst: fMP4 remux produced no output file");
  }
  return cache.string();
}

GstElement * make_first_decoder(const std::vector<std::string> & names, std::string & picked)
{
  for (const auto & n : names) {
    GstElement * e = gst_element_factory_make(n.c_str(), nullptr);
    if (e != nullptr) {
      picked = n;
      return e;
    }
  }
  picked.clear();
  return nullptr;
}

std::vector<std::string> h264_dec_order()
{
  if (rtsp_device_is_jetson()) {
    return {
      "nvv4l2decoder",
      "vah264dec",
      "vaapih264dec",
      "v4l2h264dec",
      "openh264dec",
      "avdec_h264",
    };
  }
  std::vector<std::string> v;
  if (gst_has_factory_nm("openh264dec")) {
    v.push_back("openh264dec");
  }
  v.insert(v.end(), {"avdec_h264", "vah264dec", "vaapih264dec", "v4l2h264dec"});
  return v;
}

std::vector<std::string> h265_dec_order()
{
  if (rtsp_device_is_jetson()) {
    return {"nvv4l2decoder", "vah265dec", "vaapih265dec", "v4l2h265dec", "avdec_h265"};
  }
  return {"vah265dec", "vaapih265dec", "v4l2h265dec", "avdec_h265"};
}

static void gst_msg_to_err(GstMessage * msg, std::string * out)
{
  if (out == nullptr || msg == nullptr) return;
  GError * e = nullptr;
  gchar * d = nullptr;
  gst_message_parse_error(msg, &e, &d);
  if (d != nullptr) {
    g_free(d);
  }
  if (e != nullptr) {
    *out = e->message;
    g_error_free(e);
  } else {
    *out = "VideoFileGst pipeline error";
  }
}

#endif
}  // namespace

#if defined(RTSP_IF_HAS_GSTREAMER)

namespace
{
const char * gst_state_abbr(GstState s)
{
  switch (s) {
  case GST_STATE_VOID_PENDING:
    return "-";
  case GST_STATE_NULL:
    return "NULL";
  case GST_STATE_READY:
    return "READY";
  case GST_STATE_PAUSED:
    return "PAUSED";
  case GST_STATE_PLAYING:
    return "PLAYING";
  default:
    return "?";
  }
}

std::string fmt_time_ms(gint64 t)
{
  const gint64 none = static_cast<gint64>(GST_CLOCK_TIME_NONE);
  if (t == none || t < 0)
    return "?";
  return std::to_string(t / static_cast<gint64>(GST_MSECOND));
}

/// 在未持 Impl::mtx_ 时可调用（read() 为防 Gst 流线程回调与 try_pull 死锁会先 unlock）。
bool pipeline_element_time_near_eof(GstElement * pipeline)
{
  if (pipeline == nullptr) return false;
  gint64 dur = GST_CLOCK_TIME_NONE;
  gint64 cur = GST_CLOCK_TIME_NONE;
  const gint64 none = static_cast<gint64>(GST_CLOCK_TIME_NONE);
  if (!gst_element_query_duration(pipeline, GST_FORMAT_TIME, &dur))
    return false;
  if (!gst_element_query_position(pipeline, GST_FORMAT_TIME, &cur))
    return false;
  if (dur <= 0 || dur == none || cur == none)
    return false;
  const gint64 slack = static_cast<gint64>(300 * GST_MSECOND);
  return cur + slack >= dur;
}
}  // namespace

class VideoFileGst::Impl
{
public:
  void open(const VideoFileGstConfig & cfg)
  {
    std::unique_lock<std::mutex> lk(mtx_);
    if (cfg.file_path.empty()) {
      throw std::runtime_error("VideoFileGst: cfg.file_path is empty");
    }
    cfg_ = cfg;
    open_throw_locked(lk);
  }

  void close()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    destroy_locked();
  }

  bool is_open() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return sink_ != nullptr;
  }

  bool read(cv::Mat & frame)
  {
    std::unique_lock<std::mutex> lk(mtx_);
    if (sink_ == nullptr || pipeline_ == nullptr) return false;

    // pull/seek/state 若在持 mtx_ 时阻塞，可能与 Gst streaming 线程里 notify::caps 等回调抢同一把锁而卡死整机。
    const int max_read_spin = 160;
    int spin_guard = 0;

    while (true) {
      GstElement * snk_was = sink_;
      GstElement * pipe_was = pipeline_;

      if (snk_was == nullptr || pipe_was == nullptr)
        return false;

      if (++spin_guard > max_read_spin) {
        if (!cfg_.loop)
          return false;
        diag_warn_ln(
          "read: spin_guard exceeded -> reopen_pipeline_fallback (eos/seek path likely stuck)");
        reopen_pipeline_fallback_locked(lk);
        spin_guard = 0;
        continue;
      }

      const GstClockTime pull_to =
        static_cast<GstClockTime>(std::max(1, cfg_.read_timeout_ms)) * GST_MSECOND;

      lk.unlock();
      GstSample * sample =
        gst_app_sink_try_pull_sample(GST_APP_SINK(snk_was), pull_to);
      lk.lock();

      if (sink_ == nullptr || pipeline_ == nullptr || sink_ != snk_was ||
          pipeline_ != pipe_was)
      {
        if (sample != nullptr) gst_sample_unref(sample);
        return false;
      }

      if (sample != nullptr) {
        bool ok = sample_to_bgr(sample, frame);
        gst_sample_unref(sample);
        if (ok) {
          last_ok_ms_ = vf_now_ms();
          return true;
        }
        continue;
      }

      GstAppSink * appsink = GST_APP_SINK(snk_was);
      if (gst_app_sink_is_eos(appsink)) {
        if (!cfg_.loop)
          return false;
        loop_playback_locked(lk, "appsink EOS");
        if (sink_ == nullptr || pipeline_ == nullptr)
          return false;
        continue;
      }

      const bool wants_loop = cfg_.loop;

      lk.unlock();
      const bool eof_by_clock =
        wants_loop && !fragmented_mp4_ && pipeline_element_time_near_eof(pipe_was);
      lk.lock();

      if (sink_ == nullptr || pipeline_ == nullptr || sink_ != snk_was || pipeline_ != pipe_was)
        return false;

      if (eof_by_clock) {
        loop_playback_locked(lk, "time_near_eof");
        if (sink_ == nullptr || pipeline_ == nullptr)
          return false;
        continue;
      }

      GstBus * bus = gst_element_get_bus(pipe_was);
      if (bus == nullptr) {
        if (!cfg_.loop)
          return false;
        continue;
      }

      lk.unlock();

      GstMessage * bus_msg = nullptr;
      bool saw_bus_eos = false;
      bool saw_bus_error = false;
      std::string err_txt;
      while (
        (bus_msg = gst_bus_pop_filtered(
          bus,
          static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR))) != nullptr)
      {
        const GstMessageType t = GST_MESSAGE_TYPE(bus_msg);
        if (t == GST_MESSAGE_ERROR) {
          saw_bus_error = true;
          gst_msg_to_err(bus_msg, &err_txt);
          gst_message_unref(bus_msg);
          break;
        }
        gst_message_unref(bus_msg);
        saw_bus_eos = true;
      }
      gst_object_unref(bus);

      lk.lock();

      if (sink_ == nullptr || pipeline_ == nullptr || sink_ != snk_was || pipeline_ != pipe_was)
        return false;

      if (saw_bus_error) {
        destroy_locked();
        throw std::runtime_error(err_txt.empty() ? "VideoFileGst pipeline error" : err_txt);
      }

      if (saw_bus_eos) {
        if (!cfg_.loop)
          return false;
        loop_playback_locked(lk, "bus EOS");
        if (sink_ == nullptr || pipeline_ == nullptr)
          return false;
        continue;
      }

      if (!cfg_.loop) {
        return false;
      }
      continue;
    }
  }

  std::uint64_t last_frame_time_ms() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_ok_ms_;
  }

private:
  void diag_ln(const std::string & msg) const
  {
    if (!cfg_.diag_log) {
      return;
    }
    if (!cfg_.diag_tag.empty())
      MLOGGER_INFO("camera={} VideoFileGst {}", cfg_.diag_tag, msg);
    else
      MLOGGER_INFO("VideoFileGst {}", msg);
  }

  void diag_warn_ln(const std::string & msg) const
  {
    if (!cfg_.diag_log) {
      return;
    }
    if (!cfg_.diag_tag.empty())
      MLOGGER_WARN("camera={} VideoFileGst {}", cfg_.diag_tag, msg);
    else
      MLOGGER_WARN("VideoFileGst {}", msg);
  }

  std::string pipeline_playback_diag_locked() const
  {
    if (pipeline_ == nullptr) {
      return "no-pipeline";
    }
    GstState cs = GST_STATE_NULL;
    GstState pend = GST_STATE_VOID_PENDING;
    (void)gst_element_get_state(pipeline_, &cs, &pend, GstClockTime{0});
    gint64 dur = GST_CLOCK_TIME_NONE;
    gint64 cur = GST_CLOCK_TIME_NONE;
    gboolean qd = gst_element_query_duration(pipeline_, GST_FORMAT_TIME, &dur);
    gboolean qp = gst_element_query_position(pipeline_, GST_FORMAT_TIME, &cur);

    std::ostringstream o;
    o << "state=" << gst_state_abbr(cs);
    if (pend != GST_STATE_VOID_PENDING && pend != cs)
      o << '/' << gst_state_abbr(pend);
    o << " pos_ms=" << (qp ? fmt_time_ms(cur) : "?");
    o << " dur_ms=" << (qd ? fmt_time_ms(dur) : "?");
    o << " dec=" << decoder_name_;
    if (fragmented_mp4_) {
      o << " fMP4=1 backpressure=1";
    }
    return o.str();
  }

  /// fMP4：EOS 后整管 reopen（qtdemux seek 不可靠）；普通 MP4：FLUSH seek / READY 重绕。
  void loop_playback_locked(std::unique_lock<std::mutex> & lk, const char * reason)
  {
    if (fragmented_mp4_) {
      diag_ln(
        std::string("read: ") + reason + " -> fMP4 reopen | " +
        pipeline_playback_diag_locked());
      reopen_pipeline_fallback_locked(lk);
      return;
    }
    diag_ln(std::string("read: ") + reason + " -> rewind | " + pipeline_playback_diag_locked());
    lk.unlock();
    rewind_after_eos_without_mtx();
    lk.lock();
  }

  static void pad_added_trampoline(GstElement * elem, GstPad * pad, gpointer user_data);

  struct VideoTrackCand
  {
    GstPad * pad{nullptr};
    gboolean hevc{FALSE};
    gint w{0};
    gint h{0};
    gulong caps_notify{0};
  };

  static void video_pad_caps_notify(GObject * obj, GParamSpec *, gpointer user_data);

  static void no_more_pads_trampoline(GstElement *, gpointer user_data);

  void refresh_candidate_dims_locked(GstPad * pad)
  {
    for (auto & c : video_track_cands_) {
      if (c.pad != pad) continue;
      GstCaps * ccaps = gst_pad_get_current_caps(pad);
      gint nw = 0;
      gint nh = 0;
      if (ccaps != nullptr) {
        GstStructure * sst = gst_caps_get_structure(ccaps, 0);
        if (sst != nullptr) {
          gst_structure_get_int(sst, "width", &nw);
          gst_structure_get_int(sst, "height", &nh);
        }
        gst_caps_unref(ccaps);
      }
      c.w = nw;
      c.h = nh;
      return;
    }
  }

  void clear_video_candidates_locked()
  {
    for (auto & c : video_track_cands_) {
      if (c.pad != nullptr && c.caps_notify != 0U) {
        g_signal_handler_disconnect(c.pad, c.caps_notify);
      }
      if (c.pad != nullptr) {
        gst_object_unref(c.pad);
      }
    }
    video_track_cands_.clear();
  }

  bool link_pad_to_drain_queue_fakesink(GstPad * src_pad)
  {
    if (pipeline_ == nullptr || src_pad == nullptr) return false;
    GstElement * q = gst_element_factory_make("queue", nullptr);
    GstElement * fk = gst_element_factory_make("fakesink", nullptr);
    if (q == nullptr || fk == nullptr) {
      if (q) gst_object_unref(q);
      if (fk) gst_object_unref(fk);
      return false;
    }
    g_object_set(
      G_OBJECT(q), "max-size-buffers", gint(32), "max-size-bytes", gint(0), "max-size-time",
      GstClockTime{0}, nullptr);
    g_object_set(G_OBJECT(fk), "sync", gboolean(FALSE), nullptr);
    gst_bin_add_many(GST_BIN(pipeline_), q, fk, nullptr);
    if (!gst_element_link_many(q, fk, nullptr)) {
      gst_bin_remove_many(GST_BIN(pipeline_), q, fk, nullptr);
      gst_object_unref(q);
      gst_object_unref(fk);
      return false;
    }
    GstPad * qsink = gst_element_get_static_pad(q, "sink");
    if (gst_pad_link(src_pad, qsink) != GST_PAD_LINK_OK) {
      gst_object_unref(qsink);
      gst_bin_remove_many(GST_BIN(pipeline_), q, fk, nullptr);
      gst_element_set_state(q, GST_STATE_NULL);
      gst_element_set_state(fk, GST_STATE_NULL);
      gst_object_unref(q);
      gst_object_unref(fk);
      return false;
    }
    gst_object_unref(qsink);
    gst_element_sync_state_with_parent(q);
    gst_element_sync_state_with_parent(fk);
    return true;
  }

  void finalize_video_track_choice_gst()
  {
    std::vector<VideoTrackCand> work;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (linked_video_ || fatal_open_flag_) return;
      work = video_track_cands_;
    }
    if (work.empty()) return;

    std::size_t best_i = 0;
    gint best_area = -1;
    for (std::size_t i = 0; i < work.size(); ++i) {
      gint area = -1;
      if (work[i].w > 0 && work[i].h > 0) {
        area = work[i].w * work[i].h;
      }
      if (area > best_area) {
        best_area = area;
        best_i = i;
      }
    }

    GstPad * chosen = work[best_i].pad;
    const gboolean hevc = work[best_i].hevc;

    if (work.size() > 1U) {
      g_print(
        "VideoFileGst: \"%s\" 多路视频轨，选用解码 pad=%dx%d（候选数=%zu）\n",
        cfg_.file_path.c_str(), static_cast<int>(work[best_i].w), static_cast<int>(work[best_i].h),
        work.size());
    }

    for (std::size_t i = 0; i < work.size(); ++i) {
      if (work[i].pad == chosen) continue;
      if (!link_pad_to_drain_queue_fakesink(work[i].pad)) {
        g_printerr(
          "VideoFileGst: 未能排空次要视频轨 demux pad（多轨 MP4 可能阻塞），请检查插件 queue/fakesink\n");
      }
    }

    link_video_locked(chosen, hevc);

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (linked_video_) {
        clear_video_candidates_locked();
      }
    }
  }

  bool sample_to_bgr(GstSample * sample, cv::Mat & frame)
  {
    GstBuffer * buf = gst_sample_get_buffer(sample);
    GstCaps * caps = gst_sample_get_caps(sample);
    if (buf == nullptr || caps == nullptr) return false;
    GstStructure * s = gst_caps_get_structure(caps, 0);
    gint w = 0;
    gint h = 0;
    if (!gst_structure_get_int(s, "width", &w) ||
        !gst_structure_get_int(s, "height", &h))
      return false;

    GstMapInfo map{};
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) return false;

    bool ok = false;
    const gsize need = static_cast<gsize>(w) * static_cast<gsize>(h) * 3U;
    if (map.size >= need) {
      cv::Mat wrapped(h, w, CV_8UC3, map.data);
      frame = wrapped.clone();
      ok = true;
    }

    gst_buffer_unmap(buf, &map);
    return ok;
  }

  /// 仅为常见 MP4 AAC 音轨排空；其余格式暂不链接（避免 pad 协商阻塞）。
  void link_audio_aac_drainer(GstPad * pad)
  {
    if (audio_linked_) return;

    GstCaps * cap = gst_pad_get_current_caps(pad);
    if (cap == nullptr) cap = gst_pad_query_caps(pad, nullptr);
    if (cap == nullptr) return;
    GstStructure * st = gst_caps_get_structure(cap, 0);
    if (st == nullptr) {
      gst_caps_unref(cap);
      return;
    }
    gboolean is_aac = gst_structure_has_name(st, "audio/mpeg");
    gint mv = 0;
    if (!is_aac || !gst_structure_get_int(st, "mpegversion", &mv) || mv != 4) {
      gst_caps_unref(cap);
      return;
    }
    gst_caps_unref(cap);

    GstElement * aq = gst_element_factory_make("queue", nullptr);
    GstElement * ap = gst_element_factory_make("aacparse", nullptr);
    GstElement * ad = gst_element_factory_make("avdec_aac", nullptr);
    GstElement * ac = gst_element_factory_make("audioconvert", nullptr);
    GstElement * fs = gst_element_factory_make("fakesink", nullptr);
    if (aq == nullptr || ap == nullptr || ad == nullptr || ac == nullptr || fs == nullptr) {
      if (aq) gst_object_unref(aq);
      if (ap) gst_object_unref(ap);
      if (ad) gst_object_unref(ad);
      if (ac) gst_object_unref(ac);
      if (fs) gst_object_unref(fs);
      return;
    }

    g_object_set(
      G_OBJECT(aq), "max-size-buffers", gint(24), "max-size-bytes", gint(0), "max-size-time",
      GstClockTime{0}, nullptr);
    g_object_set(G_OBJECT(fs), "sync", gboolean(FALSE), nullptr);

    gst_bin_add_many(GST_BIN(pipeline_), aq, ap, ad, ac, fs, nullptr);
    if (!gst_element_link_many(aq, ap, ad, ac, fs, nullptr)) {
      gst_bin_remove_many(GST_BIN(pipeline_), aq, ap, ad, ac, fs, nullptr);
      gst_object_unref(aq);
      gst_object_unref(ap);
      gst_object_unref(ad);
      gst_object_unref(ac);
      gst_object_unref(fs);
      return;
    }

    GstPad * qs = gst_element_get_static_pad(aq, "sink");
    const gboolean lr = (gst_pad_link(pad, qs) == GST_PAD_LINK_OK);
    gst_object_unref(qs);
    if (!lr) {
      gst_bin_remove_many(GST_BIN(pipeline_), aq, ap, ad, ac, fs, nullptr);
      gst_element_set_state(aq, GST_STATE_NULL);
      gst_element_set_state(ap, GST_STATE_NULL);
      gst_element_set_state(ad, GST_STATE_NULL);
      gst_element_set_state(ac, GST_STATE_NULL);
      gst_element_set_state(fs, GST_STATE_NULL);
      gst_object_unref(aq);
      gst_object_unref(ap);
      gst_object_unref(ad);
      gst_object_unref(ac);
      gst_object_unref(fs);
      return;
    }

    gst_element_sync_state_with_parent(aq);
    gst_element_sync_state_with_parent(ap);
    gst_element_sync_state_with_parent(ad);
    gst_element_sync_state_with_parent(ac);
    gst_element_sync_state_with_parent(fs);
    audio_linked_ = TRUE;
  }

  void signal_open_fail(const std::string & msg)
  {
    last_err_ = msg;
    fatal_open_flag_ = TRUE;
    // 禁止在此对 pipeline 做 set_state：本函数常从 demuxer 的 pad-added（流线程）调用，
    // 会触发 “cannot change state from streaming thread” 与连锁 CRITICAL。
    // teardown 由 open_throw_locked 主循环检测到 fatal_open_flag_ 后 destroy_locked()。
  }

  void link_video_locked(GstPad * video_pad, gboolean is_hevc)
  {
    if (fatal_open_flag_) return;
    if (linked_video_) return;

    GstElement * vq_el = gst_element_factory_make("queue", nullptr);
    GstElement * parse_el =
      gst_element_factory_make(is_hevc ? "h265parse" : "h264parse", nullptr);
    std::vector<std::string> cands = is_hevc ? h265_dec_order() : h264_dec_order();
    GstElement * dec = make_first_decoder(cands, decoder_name_);

    GstElement * nvcvt = nullptr;
    const bool use_nv = rtsp_device_is_jetson() && decoder_name_ == "nvv4l2decoder" &&
      gst_has_factory_nm("nvvidconv");
    if (use_nv) nvcvt = gst_element_factory_make("nvvidconv", nullptr);

    GstElement * cvt_el = gst_element_factory_make("videoconvert", nullptr);
    GstElement * cap_el = gst_element_factory_make("capsfilter", nullptr);
    GstElement * snk_el = gst_element_factory_make("appsink", nullptr);

    if (
      vq_el == nullptr || parse_el == nullptr || dec == nullptr ||
      (use_nv && nvcvt == nullptr) ||
      cvt_el == nullptr || cap_el == nullptr || snk_el == nullptr)
    {
      std::string hint =
        "VideoFileGst: decoder chain unavailable"
        "（无法创建解码链；H264 需 openh264dec 或 avdec_h264，请安装 "
        "gstreamer1.0-plugins-bad / gstreamer1.0-libav）";
      if (dec == nullptr && !cands.empty()) {
        hint += " 尝试顺序: ";
        for (size_t i = 0; i < cands.size(); ++i) {
          if (i) hint += ", ";
          hint += cands[i];
        }
      }
      signal_open_fail(hint);
      if (vq_el) gst_object_unref(vq_el);
      if (parse_el) gst_object_unref(parse_el);
      if (dec) gst_object_unref(dec);
      if (nvcvt) gst_object_unref(nvcvt);
      if (cvt_el) gst_object_unref(cvt_el);
      if (cap_el) gst_object_unref(cap_el);
      if (snk_el) gst_object_unref(snk_el);
      return;
    }

    GstCaps * bgr = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", nullptr);
    g_object_set(G_OBJECT(cap_el), "caps", bgr, nullptr);
    gst_caps_unref(bgr);

    const int sink_bufs =
      fragmented_mp4_ ? 2 : std::max(1, cfg_.appsink_max_buffers);
    const gboolean sink_drop =
      fragmented_mp4_ ? TRUE : (cfg_.drop_old_frames ? TRUE : FALSE);
    const guint vq_bufs =
      fragmented_mp4_ ? guint(2) : guint(std::max(8, cfg_.appsink_max_buffers));
    g_object_set(
      G_OBJECT(vq_el),
      "max-size-buffers", vq_bufs, "max-size-bytes", guint(0), "max-size-time",
      GstClockTime{0}, nullptr);
    g_object_set(
      G_OBJECT(snk_el), "emit-signals", gboolean(FALSE), "sync", gboolean(FALSE),
      "max-buffers", gint(sink_bufs), "drop", sink_drop, nullptr);

    gst_bin_add_many(GST_BIN(pipeline_), vq_el, parse_el, dec, nullptr);
    if (nvcvt != nullptr) gst_bin_add(GST_BIN(pipeline_), nvcvt);
    gst_bin_add_many(GST_BIN(pipeline_), cvt_el, cap_el, snk_el, nullptr);

    if (!gst_element_link_many(vq_el, parse_el, dec, nullptr)) {
      signal_open_fail("VideoFileGst: link queue-parse-decoder");
      return;
    }

    GstElement * before_cvt = dec;
    if (nvcvt != nullptr) {
      if (!gst_element_link(dec, nvcvt)) {
        signal_open_fail("VideoFileGst: link decoder→nvvidconv");
        return;
      }
      before_cvt = nvcvt;
    }
    if (!gst_element_link_many(before_cvt, cvt_el, cap_el, snk_el, nullptr)) {
      signal_open_fail("VideoFileGst: link to appsink");
      return;
    }

    GstPad * qsink = gst_element_get_static_pad(vq_el, "sink");
    if (gst_pad_link(video_pad, qsink) != GST_PAD_LINK_OK) {
      gst_object_unref(qsink);
      signal_open_fail("VideoFileGst: demux→video link failed");
      return;
    }
    gst_object_unref(qsink);

    gst_element_sync_state_with_parent(vq_el);
    gst_element_sync_state_with_parent(parse_el);
    gst_element_sync_state_with_parent(dec);
    if (nvcvt != nullptr) gst_element_sync_state_with_parent(nvcvt);
    gst_element_sync_state_with_parent(cvt_el);
    gst_element_sync_state_with_parent(cap_el);
    gst_element_sync_state_with_parent(snk_el);

    sink_ = snk_el;
    linked_video_ = TRUE;
    g_print(
      "VideoFileGst: \"%s\" %s decoder=%s nvvidconv=%s\n",
      cfg_.file_path.c_str(),
      is_hevc ? "HEVC" : "H264",
      decoder_name_.c_str(),
      nvcvt != nullptr ? "yes" : "no");
  }

  /// 文件循环 rewind：先发 PAUSED + FLUSH seek，再 READY→PLAYING，仍无效则由调用方 reopen。
  void drain_pipeline_bus_eos_only_locked()
  {
    GstBus * bd = gst_element_get_bus(pipeline_);
    if (bd == nullptr) return;
    GstMessage * dust = nullptr;
    while ((dust = gst_bus_pop_filtered(bd, GST_MESSAGE_EOS)) != nullptr) {
      gst_message_unref(dust);
    }
    gst_object_unref(bd);
  }

  /// EOS 尾部循环：尽量不整管 PAUSED（qtdemux+parse+avdec_h265 eos 时常让 PAUSED settle 卡住数秒）。
  /// 先做 PLAYING + FLUSH seek；失败或解码器不认时再短等 PAUSED 后 seek。
  bool rewind_pause_seek_play_locked()
  {
    if (pipeline_ == nullptr) return false;

    auto flush_seek_zero = [&]() -> gboolean {
      gboolean sk = gst_element_seek(
          pipeline_,
          1.0,
          GST_FORMAT_TIME,
          static_cast<GstSeekFlags>(GST_SEEK_FLAG_FLUSH | GST_SEEK_FLAG_KEY_UNIT),
          GST_SEEK_TYPE_SET,
          static_cast<gint64>(0),
          GST_SEEK_TYPE_NONE,
          GST_CLOCK_TIME_NONE);
      if (!sk)
        sk = gst_element_seek_simple(
          pipeline_,
          GST_FORMAT_TIME,
          static_cast<GstSeekFlags>(GST_SEEK_FLAG_FLUSH),
          static_cast<gint64>(0));
      (void)gst_element_get_state(pipeline_, nullptr, nullptr, GstClockTime{0});
      return sk;
    };

    gboolean ok_first = flush_seek_zero();
    if (
      ok_first != FALSE && sink_ != nullptr &&
      gst_app_sink_is_eos(GST_APP_SINK(sink_)) == FALSE) {
      return true;
    }

    GstStateChangeReturn sr = gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (sr == GST_STATE_CHANGE_FAILURE) return false;

    (void)gst_element_get_state(
      pipeline_,
      nullptr,
      nullptr,
      GstClockTime(400 * GST_MSECOND));  // 不再用秒级填满 —— eos 管线常拖满整条 read()

    const gboolean ok_after_pause = flush_seek_zero();

    sr = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (sr == GST_STATE_CHANGE_FAILURE)
      return false;

    (void)gst_element_get_state(pipeline_, nullptr, nullptr, GstClockTime{0});
    return ok_after_pause != FALSE;
  }

  bool rewind_ready_then_play_locked()
  {
    if (pipeline_ == nullptr)
      return false;

    GstStateChangeReturn rr = gst_element_set_state(pipeline_, GST_STATE_READY);
    if (rr == GST_STATE_CHANGE_FAILURE)
      return false;

    rr = gst_element_get_state(pipeline_, nullptr, nullptr, GstClockTime(2500 * GST_MSECOND));
    if (rr == GST_STATE_CHANGE_FAILURE)
      return false;

    rr = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (rr == GST_STATE_CHANGE_FAILURE)
      return false;

    (void)gst_element_get_state(pipeline_, nullptr, nullptr, GstClockTime{0});
    return rr != GST_STATE_CHANGE_FAILURE;
  }

  void rewind_after_eos_without_mtx()
  {
    if (!cfg_.loop || pipeline_ == nullptr || sink_ == nullptr)
      return;

    GstAppSink * appsink = GST_APP_SINK(sink_);
    const gboolean eos_before_bus_drain = gst_app_sink_is_eos(appsink);

    drain_pipeline_bus_eos_only_locked();

    const std::uint64_t t_wall0_ms = vf_now_ms();
    const bool pause_ok = rewind_pause_seek_play_locked();
    const gboolean eos_after_pause = gst_app_sink_is_eos(appsink);

    const bool used_ready = (!pause_ok || eos_after_pause);
    if (used_ready)
      (void)rewind_ready_then_play_locked();

    const std::uint64_t t_wall1_ms = vf_now_ms();

    if (cfg_.diag_log) {
      std::ostringstream o;
      o << "rewind eos_before_drain=" << (eos_before_bus_drain ? 1 : 0) << " pause_seek_ok="
        << pause_ok << " eos_after_pause_seek=" << (eos_after_pause ? 1 : 0)
        << " used_READY_FALLBACK=" << used_ready << " wall_ms=" << (t_wall1_ms - t_wall0_ms)
        << " | ";
      o << pipeline_playback_diag_locked();
      diag_ln(o.str());
    }
  }

  void reopen_pipeline_fallback_locked(std::unique_lock<std::mutex> & lk)
  {
    diag_warn_ln(
      std::string("reopen_pipeline_fallback: tearing down and rebuilding | ") +
      pipeline_playback_diag_locked());
    destroy_locked();
    open_throw_locked(lk);
  }

  /// pull 超时且无 is_eos 时：凭 position/duration 判断是否已到片尾需要 loop。
  bool time_position_at_eof_locked()
  {
    if (fragmented_mp4_) {
      return false;
    }
    return pipeline_element_time_near_eof(pipeline_);
  }

  void destroy_locked()
  {
    clear_video_candidates_locked();
    fatal_open_flag_ = FALSE;
    linked_video_ = FALSE;
    audio_linked_ = FALSE;
    decoder_name_.clear();
    last_err_.clear();
    playback_path_.clear();
    sink_ = nullptr;

    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  void open_throw_locked(std::unique_lock<std::mutex> & lk)
  {
    destroy_locked();
    playback_path_ = cfg_.file_path;
    fragmented_mp4_ = vf_probe_fragmented_qt(cfg_.file_path);
    if (fragmented_mp4_) {
      playback_path_ = vf_ensure_fmp4_remux_cache(cfg_.file_path);
      fragmented_mp4_ = false;
      if (cfg_.diag_log) {
        diag_ln(
          std::string("fMP4 remux cache: ") + playback_path_ +
          std::string(" (source=") + cfg_.file_path + ")");
      }
    }

    static std::once_flag gst_once;
    std::call_once(gst_once, []() {
      GError * er = nullptr;
      if (!gst_init_check(nullptr, nullptr, &er) && er != nullptr) g_error_free(er);
    });

    pipeline_ = gst_pipeline_new("video_file_pipeline");
    GstElement * fs = gst_element_factory_make("filesrc", nullptr);
    GstElement * qin = gst_element_factory_make("queue", nullptr);
    GstElement * dmx =
      vf_use_matroska_demux(playback_path_) ?
                            gst_element_factory_make("matroskademux", nullptr) :
                            gst_element_factory_make("qtdemux", nullptr);

    if (pipeline_ == nullptr || fs == nullptr || qin == nullptr || dmx == nullptr) {
      if (pipeline_) gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      if (fs) gst_object_unref(fs);
      if (qin) gst_object_unref(qin);
      if (dmx) gst_object_unref(dmx);
      throw std::runtime_error("VideoFileGst: create base elements failed");
    }

    std::vector<char> loc(playback_path_.begin(), playback_path_.end());
    loc.push_back('\0');
    g_object_set(G_OBJECT(fs), "location", loc.data(), nullptr);
    if (fragmented_mp4_) {
      g_object_set(
        G_OBJECT(qin), "max-size-buffers", guint(8), "max-size-bytes", guint(0),
        "max-size-time", GstClockTime{0}, nullptr);
    } else {
      g_object_set(
        G_OBJECT(qin), "max-size-buffers", guint(64), "max-size-bytes", guint(0),
        "max-size-time", GstClockTime{0}, nullptr);
    }

    gst_bin_add_many(GST_BIN(pipeline_), fs, qin, dmx, nullptr);
    if (!gst_element_link_many(fs, qin, dmx, nullptr)) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      throw std::runtime_error("VideoFileGst: filesrc→demux link failed");
    }

    g_signal_connect(dmx, "pad-added", G_CALLBACK(pad_added_trampoline), this);
    g_signal_connect(dmx, "no-more-pads", G_CALLBACK(no_more_pads_trampoline), this);

    lk.unlock();

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      lk.lock();
      destroy_locked();
      throw std::runtime_error("VideoFileGst: PLAYING failed");
    }

    const std::uint64_t t_end = vf_now_ms() + 14000ULL;
    while (vf_now_ms() < t_end) {
      if (fatal_open_flag_) {
        std::string e = last_err_.empty() ? "VideoFileGst: open aborted" : last_err_;
        lk.lock();
        destroy_locked();
        throw std::runtime_error(e);
      }
      GstBus * bus = gst_element_get_bus(pipeline_);
      while (GstMessage * m =
               gst_bus_timed_pop_filtered(bus, 50 * GST_MSECOND, GST_MESSAGE_ERROR))
      {
        std::string em;
        gst_msg_to_err(m, &em);
        gst_message_unref(m);
        gst_object_unref(bus);
        lk.lock();
        destroy_locked();
        throw std::runtime_error(em);
      }
      gst_object_unref(bus);

      if (linked_video_ && sink_ != nullptr) {
        lk.lock();
        last_ok_ms_ = 0;
        if (cfg_.diag_log) {
          diag_ln(
            std::string("open_ok ") + pipeline_playback_diag_locked() +
            std::string(" loop=") + (cfg_.loop ? "true" : "false") +
            std::string(" read_timeout_ms=") + std::to_string(cfg_.read_timeout_ms) +
            std::string(" fragmented=") + (fragmented_mp4_ ? "true" : "false") +
            std::string(" file=") + cfg_.file_path);
        }
        return;
      }
      g_usleep(12000);
    }

    if (!linked_video_) {
      finalize_video_track_choice_gst();
    }
    if (linked_video_ && sink_ != nullptr) {
      lk.lock();
      last_ok_ms_ = 0;
      if (cfg_.diag_log) {
        diag_ln(
          std::string("open_ok ") + pipeline_playback_diag_locked() +
          std::string(" loop=") + (cfg_.loop ? "true" : "false") +
          std::string(" read_timeout_ms=") + std::to_string(cfg_.read_timeout_ms) +
          std::string(" fragmented=") + (fragmented_mp4_ ? "true" : "false") +
          std::string(" file=") + cfg_.file_path);
      }
      return;
    }

    lk.lock();
    destroy_locked();
    throw std::runtime_error("VideoFileGst: timeout waiting video track");
  }

  GstElement * pipeline_{nullptr};
  GstElement * sink_{nullptr};

  gboolean linked_video_{FALSE};
  gboolean audio_linked_{FALSE};
  gboolean fatal_open_flag_{FALSE};

  VideoFileGstConfig cfg_;
  std::string playback_path_;
  std::string decoder_name_;
  std::string last_err_;
  std::uint64_t last_ok_ms_{0};
  bool fragmented_mp4_{false};

  std::vector<VideoTrackCand> video_track_cands_;

  mutable std::mutex mtx_;
};

void VideoFileGst::Impl::video_pad_caps_notify(GObject * obj, GParamSpec *, gpointer user_data)
{
  auto * self = static_cast<Impl *>(user_data);
  if (self == nullptr) return;
  GstPad * pad = GST_PAD(obj);
  std::lock_guard<std::mutex> lk(self->mtx_);
  self->refresh_candidate_dims_locked(pad);
}

void VideoFileGst::Impl::no_more_pads_trampoline(GstElement *, gpointer user_data)
{
  auto * self = static_cast<Impl *>(user_data);
  if (self == nullptr) return;
  self->finalize_video_track_choice_gst();
}

void VideoFileGst::Impl::pad_added_trampoline(GstElement * /*elem*/, GstPad * pad, gpointer user_data)
{
  auto * self = static_cast<Impl *>(user_data);
  if (
    self == nullptr || pad == nullptr || GST_PAD_DIRECTION(pad) != GST_PAD_SRC ||
    self->fatal_open_flag_)
    return;

  GstCaps * caps = gst_pad_get_current_caps(pad);
  if (caps == nullptr) caps = gst_pad_query_caps(pad, nullptr);
  if (caps == nullptr) return;
  GstStructure * st = gst_caps_get_structure(caps, 0);
  const gchar * nm = st != nullptr ? gst_structure_get_name(st) : nullptr;

  gboolean is_h264 = nm != nullptr &&
    (g_str_equal(nm, "video/x-h264") || g_str_equal(nm, "video/x-avc") ||
     g_str_equal(nm, "video/avc"));
  gboolean is_h265 = nm != nullptr &&
    (g_str_equal(nm, "video/x-h265") || g_str_equal(nm, "video/x-hevc") ||
     g_str_equal(nm, "video/hevc") || g_str_equal(nm, "video/x-hev"));

  if (is_h264 || is_h265) {
    gint vw = 0;
    gint vh = 0;
    if (st != nullptr) {
      gst_structure_get_int(st, "width", &vw);
      gst_structure_get_int(st, "height", &vh);
    }
    gst_caps_unref(caps);
    caps = nullptr;

    {
      std::lock_guard<std::mutex> lk(self->mtx_);
      if (self->linked_video_) return;
      for (const auto & ec : self->video_track_cands_) {
        if (ec.pad == pad) return;
      }
      gst_object_ref(pad);
      VideoTrackCand vc;
      vc.pad = pad;
      vc.hevc = is_h265 ? TRUE : FALSE;
      vc.w = vw;
      vc.h = vh;
      vc.caps_notify = g_signal_connect(pad, "notify::caps", G_CALLBACK(video_pad_caps_notify), self);
      self->video_track_cands_.push_back(vc);
    }
    return;
  }

  self->link_audio_aac_drainer(pad);
  gst_caps_unref(caps);
}

#endif

VideoFileGst::VideoFileGst()
#if defined(RTSP_IF_HAS_GSTREAMER)
: impl_(std::make_unique<Impl>())
#else
:
  impl_(nullptr)
#endif
{
}

VideoFileGst::~VideoFileGst()
{
#if defined(RTSP_IF_HAS_GSTREAMER)
  if (impl_) impl_->close();
#endif
}

void VideoFileGst::open(const VideoFileGstConfig & cfg)
{
#if defined(RTSP_IF_HAS_GSTREAMER)
  impl_->open(cfg);
#else
  (void)cfg;
  throw std::runtime_error("VideoFileGst: built without RTSP_IF_HAS_GSTREAMER");
#endif
}

void VideoFileGst::close()
{
#if defined(RTSP_IF_HAS_GSTREAMER)
  if (impl_) impl_->close();
#endif
}

bool VideoFileGst::is_open() const
{
#if defined(RTSP_IF_HAS_GSTREAMER)
  return impl_ && impl_->is_open();
#else
  return false;
#endif
}

bool VideoFileGst::read(cv::Mat & frame)
{
#if defined(RTSP_IF_HAS_GSTREAMER)
  return impl_ && impl_->read(frame);
#else
  (void)frame;
  return false;
#endif
}

std::uint64_t VideoFileGst::last_frame_time_ms() const
{
#if defined(RTSP_IF_HAS_GSTREAMER)
  return impl_ ? impl_->last_frame_time_ms() : 0;
#else
  return 0;
#endif
}

}  // namespace m_common
