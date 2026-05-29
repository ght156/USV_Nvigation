// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// m_common::rtsp_interface 的 GStreamer 后端实现：
//   - GstRTSPServer + appsrc + … + (H.264: x264enc/nvv4l2h264enc + rtph264pay)
//     （H.265: x265enc/nvv4l2h265enc + rtph265pay）
//   - 每路一个 mount，appsrc max-buffers=1 + leaky downstream，低端到端延迟
//   - push() 把 cv::Mat 拷贝到 GstBuffer，invoke 到 GLib MainLoop 线程上 push_buffer
//   - gst_rtsp_server_attach 必须在运行 g_main_loop 的同一线程上调用（Jetson/aarch64 上跨线程 attach 会失败）
//   - 暂停时 push() 直接丢帧；mount/factory/appsrc 不变，客户端连接保留
//
// GStreamer RTSP 推流后端（m_common::rtsp_interface）
// rtsp 包（rtsp 包不 export 库）。

#include "backend_iface.hpp"
#include "rtsp_device_detect.hpp"

#include <atomic>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <glib-object.h>
#include <glib.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>

namespace m_common
{
namespace rtsp_internal
{
namespace
{

GQuark unprepared_hook_quark()
{
  static GQuark q = 0;
  if (q == 0) q = g_quark_from_static_string("m_common.rtsp_interface.unprepared_hook");
  return q;
}

enum class H264Impl { kX264, kNvV4L2, kOpenH264 };
enum class H265Impl { kX265, kNvV4L2 };

bool gst_has_factory(const char * name)
{
  GstElementFactory * f = gst_element_factory_find(name);
  if (!f) return false;
  gst_object_unref(f);
  return true;
}

bool jetson_hw_h264_available()
{
  return gst_has_factory("nvv4l2h264enc") && gst_has_factory("nvvidconv");
}

bool jetson_hw_h265_available()
{
  return gst_has_factory("nvv4l2h265enc") && gst_has_factory("nvvidconv");
}

H264Impl resolve_h264_impl(const std::string & raw)
{
  std::string c = raw;
  for (char & ch : c) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

  if (c == "nvv4l2" || c == "nvv4l2h264" || c == "nvv4l2h264enc" || c == "jetson" || c == "hw" ||
    c == "nvenc") {
    if (!jetson_hw_h264_available()) {
      throw std::runtime_error("rtsp_interface[gst]: h264_encoder=hw 但缺少 nvv4l2h264enc");
    }
    return H264Impl::kNvV4L2;
  }
  if (c == "x264" || c == "libx264" || c == "x264enc") {
    if (!gst_has_factory("x264enc")) {
      throw std::runtime_error("rtsp_interface[gst]: h264_encoder=x264 但系统无 x264enc");
    }
    return H264Impl::kX264;
  }
  if (c == "openh264" || c == "openh264enc" || c == "cisco-openh264" || c == "cisco_openh264") {
    if (!gst_has_factory("openh264enc")) {
      throw std::runtime_error(
        "rtsp_interface[gst]: h264_encoder=openh264 但系统无 openh264enc（gst-plugins-bad）");
    }
    return H264Impl::kOpenH264;
  }
  // auto（含空）：Jetson + nvv4l2 → 硬件；否则优先 openh264enc，再无则 x264enc
  if (c == "auto" || c.empty()) {
    if (m_common::rtsp_device_is_jetson() && jetson_hw_h264_available()) {
      return H264Impl::kNvV4L2;
    }
    if (gst_has_factory("openh264enc")) {
      return H264Impl::kOpenH264;
    }
    if (gst_has_factory("x264enc")) {
      return H264Impl::kX264;
    }
    throw std::runtime_error(
      "rtsp_interface[gst]: h264_encoder=auto 但未找到编码器 "
      "(Jetson 需 nvv4l2h264enc+nvvidconv；其它平台需 openh264enc 或 x264enc)");
  }
  throw std::runtime_error(
    "rtsp_interface[gst]: unknown h264_encoder \"" + raw +
    "\" （可用 auto | x264 | x264enc | openh264 | openh264enc | nvv4l2h264enc | jetson | hw）");
}

H265Impl resolve_h265_impl(const std::string & raw)
{
  std::string c = raw;
  for (char & ch : c) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

  if (c == "nvv4l2" || c == "nvv4l2h265" || c == "nvv4l2h265enc" || c == "jetson" || c == "hw" ||
    c == "nvenc") {
    if (!jetson_hw_h265_available()) {
      throw std::runtime_error(
        "rtsp_interface[gst]: h265_encoder=hw 但缺少 nvv4l2h265enc 或 nvvidconv");
    }
    return H265Impl::kNvV4L2;
  }
  if (c == "x265" || c == "libx265" || c == "x265enc") {
    if (!gst_has_factory("x265enc")) {
      throw std::runtime_error("rtsp_interface[gst]: h265_encoder=x265 但系统无 x265enc");
    }
    return H265Impl::kX265;
  }
  if (c == "auto" || c.empty()) {
    if (m_common::rtsp_device_is_jetson() && jetson_hw_h265_available()) {
      return H265Impl::kNvV4L2;
    }
    if (gst_has_factory("x265enc")) {
      if (m_common::rtsp_device_is_jetson() && !jetson_hw_h265_available()) {
        std::fprintf(stderr,
          "[rtsp_interface][gst] WARN: Jetson 上 h265_encoder=auto 无 nvv4l2h265enc，回退 CPU x265enc "
          "（较慢）；请安装 nvidia-l4t-gstreamer 或改用 H.264\n");
      }
      return H265Impl::kX265;
    }
    throw std::runtime_error(
      "rtsp_interface[gst]: h265_encoder=auto 但未找到编码器 "
      "(Jetson 需 nvv4l2h265enc+nvvidconv；其它平台需 x265enc)");
  }
  throw std::runtime_error(
    "rtsp_interface[gst]: unknown h265_encoder \"" + raw +
    "\" （可用 auto | x265 | x265enc | nvv4l2h265enc | jetson | hw）");
}

std::string h264_branch(int kbps, H264Impl impl)
{
  if (kbps < 200) kbps = 200;
  if (impl == H264Impl::kX264) {
    // CAVLC、单参考、关 slice 线程，减轻 RTP/ffplay 花屏；默认 x264 配置在弱解码器上更易出块/拖影。
    return std::string(
      "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=10 threads=0 bframes=0 b-adapt=false "
      "ref=1 cabac=false dct8x8=false mb-tree=false rc-lookahead=0 sync-lookahead=0 sliced-threads=false "
      "bitrate=") +
      std::to_string(kbps) + " ! h264parse";
  }
  if (impl == H264Impl::kNvV4L2) {
    // 编码器后加 queue，减轻与 rtph264pay 之间的瞬时反压（单缓冲时 Jetson 上易整链卡顿）
    return std::string("nvv4l2h264enc bitrate=") + std::to_string(kbps * 1000) +
      " iframeinterval=10 insert-sps-pps=true "
      "! queue max-size-buffers=4 max-size-time=0 max-size-bytes=0 ! h264parse";
  }
  if (impl == H264Impl::kOpenH264) {
    return std::string("openh264enc bitrate=") + std::to_string(static_cast<unsigned>(kbps) * 1000U) +
      " rate-control=bitrate complexity=low ! h264parse";
  }
  throw std::logic_error("rtsp_interface[gst]: h264_branch: unexpected H264Impl");
}

std::string h265_branch(int kbps, H265Impl impl)
{
  if (kbps < 200) kbps = 200;
  if (impl == H265Impl::kNvV4L2) {
    return std::string("nvv4l2h265enc bitrate=") + std::to_string(kbps * 1000) +
      " iframeinterval=10 insert-sps-pps=true "
      "! queue max-size-buffers=4 max-size-time=0 max-size-bytes=0 ! h265parse";
  }
  return std::string(
           "x265enc speed-preset=ultrafast tune=zerolatency key-int-max=10 bitrate=") +
    std::to_string(kbps) + " ! h265parse";
}

std::string build_h264_full_pipeline(H264Impl impl, int kbps)
{
  const std::string pay = " ! rtph264pay name=pay0 pt=96 config-interval=-1 aggregate-mode=none )";
  const std::string q = "queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 ! ";
  // Jetson NVMM：单缓冲易在 nvvidconv<->编码器间反压卡顿，略加大队列仍保持低延迟
  const std::string q_nvmm = "queue max-size-buffers=4 max-size-time=0 max-size-bytes=0 ! ";
  if (impl == H264Impl::kNvV4L2) {
    return std::string("( appsrc name=ros_src is-live=true format=time ! videoconvert ! "
                       "video/x-raw,format=NV12 ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! ") +
      q_nvmm + h264_branch(kbps, impl) + pay;
  }
  return std::string("( appsrc name=ros_src is-live=true format=time ! videoconvert ! "
                     "video/x-raw,format=I420 ! ") +
    q + h264_branch(kbps, impl) + pay;
}

std::string build_h265_full_pipeline(H265Impl impl, int kbps)
{
  const std::string pay = " ! rtph265pay name=pay0 pt=97 config-interval=-1 aggregate-mode=none )";
  const std::string q = "queue max-size-buffers=1 max-size-time=0 max-size-bytes=0 ! ";
  const std::string q_nvmm = "queue max-size-buffers=4 max-size-time=0 max-size-bytes=0 ! ";
  if (impl == H265Impl::kNvV4L2) {
    return std::string("( appsrc name=ros_src is-live=true format=time ! videoconvert ! "
                       "video/x-raw,format=NV12 ! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! ") +
      q_nvmm + h265_branch(kbps, impl) + pay;
  }
  return std::string("( appsrc name=ros_src is-live=true format=time ! videoconvert ! "
                     "video/x-raw,format=I420 ! ") +
    q + h265_branch(kbps, impl) + pay;
}

const char * gst_h264_primary_element(H264Impl impl)
{
  switch (impl) {
    case H264Impl::kX264: return "x264enc";
    case H264Impl::kNvV4L2: return "nvv4l2h264enc";
    case H264Impl::kOpenH264: return "openh264enc";
  }
  return "?";
}

const char * gst_h265_primary_element(H265Impl impl)
{
  return impl == H265Impl::kNvV4L2 ? "nvv4l2h265enc" : "x265enc";
}

void log_gstreamer_rtsp_encoder_summary(const RtspPublisherConfig & cfg)
{
  gchar * ver = gst_version_string();
  std::fprintf(stderr, "[rtsp_interface][gst] RTSP listening bind=%s port=%u | %s\n",
    cfg.bind_address.c_str(), static_cast<unsigned>(cfg.port), ver ? ver : "?");
  if (ver != nullptr) g_free(ver);

  for (std::size_t i = 0; i < cfg.streams.size(); ++i) {
    const auto & s = cfg.streams[i];
    if (s.codec == RtspCodec::kH265) {
      const H265Impl hi = resolve_h265_impl(cfg.h265_encoder);
      std::fprintf(stderr,
        "[rtsp_interface][gst] stream #%zu mount=\"%s\" HEVC pay=rtph265pay primary=%s "
        "(h265_encoder=\"%s\")%s\n",
        i, s.mount_path.c_str(), gst_h265_primary_element(hi), cfg.h265_encoder.c_str(),
        hi == H265Impl::kNvV4L2 ? " +nvvidconv NVMM" : " +I420");
    } else {
      const H264Impl hi = resolve_h264_impl(cfg.h264_encoder);
      std::fprintf(stderr,
        "[rtsp_interface][gst] stream #%zu mount=\"%s\" AVC pay=rtph264pay primary=%s "
        "(h264_encoder=\"%s\")%s\n",
        i, s.mount_path.c_str(), gst_h264_primary_element(hi), cfg.h264_encoder.c_str(),
        hi == H264Impl::kNvV4L2 ? " +nvvidconv NVMM" : " +I420");
    }
  }
}

struct Channel;

struct PushCtx
{
  Channel * ch{nullptr};
  GstBuffer * buf{nullptr};
  int fw{0};
  int fh{0};
};

struct Channel
{
  Channel() { g_weak_ref_init(&appsrc_weak, nullptr); }
  ~Channel() { g_weak_ref_clear(&appsrc_weak); }
  Channel(const Channel &) = delete;
  Channel & operator=(const Channel &) = delete;

  std::string mount_path;
  std::string rtsp_app;
  std::string rtsp_stream;
  std::string mount_path_rtsp;     // "/<app>/<stream>"
  std::string pipeline_launch;

  GWeakRef appsrc_weak;
  std::mutex mu;

  int pending_w{0}, pending_h{0};
  int locked_w{0}, locked_h{0};
  int fps_num{30}, fps_den{1};
  int bitrate_kbps{4000};
  std::uint64_t frame_index{0};   // 仅在成功 push_buffer 后递增，与 PTS 一致（避免无客户端时丢 PTS 序号）
  std::uint64_t pushed_total{0};  // 对外统计

  std::atomic<bool> paused{false};
  std::uint64_t paused_drop_counter = 0;
};

gboolean idle_push_buffer(gpointer user_data)
{
  auto * w = static_cast<PushCtx *>(user_data);
  if (!w->buf || !w->ch) {
    if (w->buf) gst_buffer_unref(w->buf);
    delete w;
    return G_SOURCE_REMOVE;
  }
  GstElement * src_el = nullptr;
  std::uint64_t fi = 0;
  int fps_num = 25;
  int fps_den = 1;
  {
    std::lock_guard<std::mutex> lk(w->ch->mu);
    gpointer p = g_weak_ref_get(&w->ch->appsrc_weak);
    if (!p) {
      gst_buffer_unref(w->buf);
      delete w;
      return G_SOURCE_REMOVE;
    }
    src_el = GST_ELEMENT(p);

    if (w->ch->locked_w <= 0 || w->ch->locked_h <= 0) {
      if (w->fw <= 0 || w->fh <= 0) {
        gst_object_unref(src_el);
        gst_buffer_unref(w->buf);
        delete w;
        return G_SOURCE_REMOVE;
      }
      w->ch->locked_w = w->fw;
      w->ch->locked_h = w->fh;
      GstCaps * caps = gst_caps_new_simple(
        "video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT, w->fw,
        "height", G_TYPE_INT, w->fh, "framerate", GST_TYPE_FRACTION, w->ch->fps_num,
        w->ch->fps_den, nullptr);
      gst_app_src_set_caps(GST_APP_SRC(src_el), caps);
      gst_caps_unref(caps);
    }

    fi = w->ch->frame_index;
    fps_num = w->ch->fps_num;
    fps_den = w->ch->fps_den;
  }

  GST_BUFFER_PTS(w->buf) = gst_util_uint64_scale(fi, GST_SECOND * fps_den, fps_num);
  GST_BUFFER_DURATION(w->buf) = gst_util_uint64_scale(1, GST_SECOND * fps_den, fps_num);
  GST_BUFFER_DTS(w->buf) = GST_BUFFER_PTS(w->buf);
  GST_BUFFER_FLAG_SET(w->buf, GST_BUFFER_FLAG_LIVE);
  const GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(src_el), w->buf);
  gst_object_unref(src_el);

  if (flow >= GST_FLOW_OK) {
    std::lock_guard<std::mutex> lk(w->ch->mu);
    ++w->ch->pushed_total;
    ++w->ch->frame_index;
  } else {
    gst_buffer_unref(w->buf);
  }
  delete w;
  return G_SOURCE_REMOVE;
}

void on_media_unprepared(GstRTSPMedia *, gpointer user_data)
{
  auto * ch = static_cast<Channel *>(user_data);
  std::lock_guard<std::mutex> lk(ch->mu);
  g_weak_ref_clear(&ch->appsrc_weak);
  g_weak_ref_init(&ch->appsrc_weak, nullptr);
  ch->frame_index = 0;
  ch->locked_w = 0;
  ch->locked_h = 0;
}

void on_media_configure(GstRTSPMediaFactory *, GstRTSPMedia * media, gpointer user_data)
{
  auto * ch = static_cast<Channel *>(user_data);
  GstElement * element = gst_rtsp_media_get_element(media);
  if (!element) return;
  if (g_object_get_qdata(G_OBJECT(media), unprepared_hook_quark()) == nullptr) {
    g_object_set_qdata(G_OBJECT(media), unprepared_hook_quark(), gpointer(1));
    g_signal_connect(media, "unprepared", G_CALLBACK(on_media_unprepared), ch);
  }
  GstElement * src = gst_bin_get_by_name_recurse_up(GST_BIN(element), "ros_src");
  if (!src) {
    gst_object_unref(element);
    return;
  }
  g_object_set(src, "is-live", TRUE, "format", GST_FORMAT_TIME, "do-timestamp", FALSE,
               "block", FALSE, "max-buffers", guint64{1},
               "leaky-type", GST_APP_LEAKY_TYPE_DOWNSTREAM, nullptr);
  {
    std::lock_guard<std::mutex> lk(ch->mu);
    ch->locked_w = 0;
    ch->locked_h = 0;
  }
  gst_app_src_set_caps(GST_APP_SRC(src), nullptr);
  {
    std::lock_guard<std::mutex> lk(ch->mu);
    g_weak_ref_set(&ch->appsrc_weak, G_OBJECT(src));
    ch->frame_index = 0;
  }
  gst_object_unref(src);
  gst_object_unref(element);
}

class GstreamerBackend final : public IBackend
{
public:
  ~GstreamerBackend() override { teardown(); }

  const char * name() const override { return "gstreamer"; }

  void start(const RtspPublisherConfig & cfg) override
  {
    if (started_) throw std::runtime_error("rtsp_interface[gst]: already started");
    if (cfg.streams.empty()) throw std::runtime_error("rtsp_interface[gst]: streams empty");
    cfg_ = cfg;

    static std::once_flag s_gst_init;
    std::call_once(s_gst_init, []() { gst_init(nullptr, nullptr); });

    server_ = gst_rtsp_server_new();
    if (!server_) throw std::runtime_error("rtsp_interface[gst]: gst_rtsp_server_new failed");
    gst_rtsp_server_set_address(server_, cfg_.bind_address.c_str());
    gst_rtsp_server_set_service(server_, std::to_string(static_cast<int>(cfg_.port)).c_str());
    setup_auth_if_needed();

    main_ctx_ = g_main_context_new();
    if (!main_ctx_) throw std::runtime_error("rtsp_interface[gst]: g_main_context_new failed");
    loop_ = g_main_loop_new(main_ctx_, FALSE);
    if (!loop_) throw std::runtime_error("rtsp_interface[gst]: g_main_loop_new failed");
    glib_thread_ = std::thread([this]() { g_main_loop_run(loop_); });

    try {
      channels_.reserve(cfg_.streams.size());
      for (const auto & s : cfg_.streams) {
        if (s.codec == RtspCodec::kMjpeg) {
          throw std::runtime_error(
            "rtsp_interface[gst]: MJPEG 当前未支持（请使用 H.264/H.265）");
        }
        auto ch = std::make_unique<Channel>();
        ch->mount_path = s.mount_path;
        if (!ch->mount_path.empty() && ch->mount_path[0] == '/') ch->mount_path.erase(0, 1);
        const auto slash = ch->mount_path.find('/');
        if (slash == std::string::npos) {
          ch->rtsp_app = "live";
          ch->rtsp_stream = ch->mount_path;
        } else {
          ch->rtsp_app = ch->mount_path.substr(0, slash);
          ch->rtsp_stream = ch->mount_path.substr(slash + 1);
        }
        if (ch->rtsp_app.empty() || ch->rtsp_stream.empty()) {
          throw std::runtime_error("rtsp_interface[gst]: invalid mount_path: " + s.mount_path);
        }
        ch->mount_path_rtsp = "/" + ch->rtsp_app + "/" + ch->rtsp_stream;
        ch->fps_num = (s.fps > 0) ? s.fps : 25;
        ch->fps_den = 1;
        ch->bitrate_kbps = (s.bitrate_kbps >= 200) ? s.bitrate_kbps : 200;
        if (s.codec == RtspCodec::kH265) {
          ch->pipeline_launch =
            build_h265_full_pipeline(resolve_h265_impl(cfg_.h265_encoder), ch->bitrate_kbps);
        } else {
          ch->pipeline_launch =
            build_h264_full_pipeline(resolve_h264_impl(cfg_.h264_encoder), ch->bitrate_kbps);
        }
        channels_.push_back(std::move(ch));
      }

      wait_register_mounts_on_main();
      // 须在运行 g_main_loop 的同一线程上 attach（与注册 factory 一致）；在 ROS 主线程直接 attach
      // 在部分 aarch64/Jetson GLib 上会返回 0，x86 上偶发仍能通过。
      wait_attach_rtsp_server_on_main();
    } catch (...) {
      teardown();
      throw;
    }
    started_ = true;
    log_gstreamer_rtsp_encoder_summary(cfg_);
  }

  void stop() override { teardown(); }

  bool push(std::size_t idx, const cv::Mat & bgr_in) override
  {
    if (!started_ || idx >= channels_.size()) return false;
    auto & ch = channels_[idx];
    if (!ch || bgr_in.empty()) return false;
    if (global_paused_.load(std::memory_order_acquire) || ch->paused.load()) {
      ++ch->paused_drop_counter;
      return false;
    }
    cv::Mat img = bgr_in;
    if (!img.isContinuous()) img = img.clone();
    {
      std::lock_guard<std::mutex> lk(ch->mu);
      ch->pending_w = img.cols;
      ch->pending_h = img.rows;
    }
    int locked_w = 0, locked_h = 0;
    {
      std::lock_guard<std::mutex> lk(ch->mu);
      locked_w = ch->locked_w;
      locked_h = ch->locked_h;
    }
    if (locked_w > 0 && locked_h > 0 && (img.cols != locked_w || img.rows != locked_h)) {
      cv::Mat resized;
      cv::resize(img, resized, cv::Size(locked_w, locked_h), 0, 0, cv::INTER_LINEAR);
      img = std::move(resized);
    }

    const gsize nbytes = static_cast<gsize>(img.total() * img.elemSize());
    GstBuffer * buf = gst_buffer_new_allocate(nullptr, nbytes, nullptr);
    if (!buf) return false;
    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_WRITE)) { gst_buffer_unref(buf); return false; }
    std::memcpy(map.data, img.ptr(), nbytes);
    gst_buffer_unmap(buf, &map);

    auto * ctx = new PushCtx;
    ctx->ch = ch.get();
    ctx->buf = buf;
    ctx->fw = img.cols;
    ctx->fh = img.rows;
    g_main_context_invoke(g_main_loop_get_context(loop_), idle_push_buffer, ctx);
    return true;
  }

  void set_global_paused(bool paused) override
  {
    global_paused_.store(paused);
  }

  void set_stream_paused(const std::string & key, bool paused) override
  {
    for (auto & ch : channels_) {
      if (!ch) continue;
      const bool match = (key == ch->mount_path) ||
                         (key == ch->rtsp_app + "/" + ch->rtsp_stream);
      if (match) ch->paused.store(paused);
    }
  }

  std::string url(std::size_t idx) const override
  {
    if (idx >= channels_.size() || !channels_[idx]) return {};
    const auto & ch = channels_[idx];
    return "rtsp://" + cfg_.bind_address + ":" + std::to_string(cfg_.port) + ch->mount_path_rtsp;
  }

  std::optional<std::size_t> find_channel(const std::string & key) const override
  {
    for (std::size_t i = 0; i < channels_.size(); ++i) {
      const auto & ch = channels_[i];
      if (!ch) continue;
      if (ch->mount_path == key) return i;
      if (ch->rtsp_app + "/" + ch->rtsp_stream == key) return i;
    }
    return std::nullopt;
  }

  std::uint64_t pushed_frames(std::size_t idx) const override
  {
    if (idx >= channels_.size() || !channels_[idx]) return 0;
    return channels_[idx]->pushed_total;
  }

private:
  struct MountRegCtx
  {
    GstreamerBackend * be{nullptr};
    std::promise<void> done;
  };

  static gboolean invoke_register_all_mounts(gpointer user_data)
  {
    auto * ctx = static_cast<MountRegCtx *>(user_data);
    ctx->be->register_all_factories_on_main();
    ctx->done.set_value();
    delete ctx;
    return G_SOURCE_REMOVE;
  }

  void wait_register_mounts_on_main()
  {
    if (!loop_) return;
    auto * ctx = new MountRegCtx;
    ctx->be = this;
    auto fut = ctx->done.get_future();
    g_main_context_invoke(g_main_loop_get_context(loop_), invoke_register_all_mounts, ctx);
    fut.wait();
  }

  struct AttachCtx
  {
    GstreamerBackend * be{nullptr};
    std::promise<guint> attach_result;
  };

  static gboolean idle_attach_rtsp_server(gpointer user_data)
  {
    auto * ctx = static_cast<AttachCtx *>(user_data);
    const guint id = gst_rtsp_server_attach(ctx->be->server_, ctx->be->main_ctx_);
    ctx->attach_result.set_value(id);
    delete ctx;
    return G_SOURCE_REMOVE;
  }

  void wait_attach_rtsp_server_on_main()
  {
    if (!loop_) return;
    auto * ctx = new AttachCtx;
    ctx->be = this;
    auto fut = ctx->attach_result.get_future();
    g_main_context_invoke(g_main_loop_get_context(loop_), idle_attach_rtsp_server, ctx);
    const guint attach_id = fut.get();
    if (attach_id == 0) {
      const int err = errno;
      std::string msg = "rtsp_interface[gst]: gst_rtsp_server_attach failed bind=" + cfg_.bind_address +
        " port=" + std::to_string(static_cast<int>(cfg_.port));
      if (err != 0) {
        msg += " errno=" + std::to_string(err) + " (" + std::strerror(err) + ")";
      }
      msg += " | 请检查端口是否被占用(如 ss -tlnp|grep " + std::to_string(static_cast<int>(cfg_.port)) +
             ")、Docker/其它 RTSP 进程；或尝试修改 rtsp_multi_source.yaml 的 output.port";
      throw std::runtime_error(msg);
    }
  }

  void register_all_factories_on_main()
  {
    GstRTSPMountPoints * mp = gst_rtsp_server_get_mount_points(server_);
    if (!mp) return;
    for (auto & uch : channels_) {
      Channel * ch = uch.get();
      GstRTSPMediaFactory * factory = gst_rtsp_media_factory_new();
      gst_rtsp_media_factory_set_launch(factory, ch->pipeline_launch.c_str());
      gst_rtsp_media_factory_set_shared(factory, TRUE);
      gst_rtsp_media_factory_set_latency(factory, static_cast<guint>(cfg_.pipeline_latency_ms));
      if (auth_enabled_) {
        GstRTSPPermissions * perms = gst_rtsp_permissions_new();
        gst_rtsp_permissions_add_role(
          perms, auth_role_.c_str(),
          GST_RTSP_PERM_MEDIA_FACTORY_ACCESS, G_TYPE_BOOLEAN, TRUE,
          GST_RTSP_PERM_MEDIA_FACTORY_CONSTRUCT, G_TYPE_BOOLEAN, TRUE,
          nullptr);
        gst_rtsp_media_factory_set_permissions(factory, perms);
        gst_rtsp_permissions_unref(perms);
      }
      g_signal_connect(factory, "media-configure", G_CALLBACK(on_media_configure), ch);
      gst_rtsp_mount_points_add_factory(mp, ch->mount_path_rtsp.c_str(), factory);
    }
    g_object_unref(mp);
  }

  void setup_auth_if_needed()
  {
    const bool has_u = !cfg_.auth_username.empty();
    const bool has_p = !cfg_.auth_password.empty();
    if (has_u != has_p) {
      throw std::runtime_error(
        "rtsp_interface[gst]: auth_username/auth_password 必须同时设置或同时为空");
    }
    if (!has_u) {
      auth_enabled_ = false;
      return;
    }
    auth_enabled_ = true;
    auth_role_ = "user";
    auth_ = gst_rtsp_auth_new();
    if (!auth_) throw std::runtime_error("rtsp_interface[gst]: gst_rtsp_auth_new failed");
    const std::string realm = cfg_.auth_realm.empty() ? "rtsp" : cfg_.auth_realm;
    gst_rtsp_auth_set_realm(auth_, realm.c_str());

    gchar * basic = gst_rtsp_auth_make_basic(cfg_.auth_username.c_str(), cfg_.auth_password.c_str());
    GstRTSPToken * token = gst_rtsp_token_new(
      GST_RTSP_TOKEN_MEDIA_FACTORY_ROLE, G_TYPE_STRING, auth_role_.c_str(), nullptr);
    gst_rtsp_auth_add_basic(auth_, basic, token);
    g_free(basic);
    gst_rtsp_token_unref(token);
    gst_rtsp_server_set_auth(server_, auth_);
  }

  void teardown()
  {
    if (shutting_down_.exchange(true)) return;
    if (loop_) g_main_loop_quit(loop_);
    if (glib_thread_.joinable()) glib_thread_.join();
    if (loop_)     { g_main_loop_unref(loop_);  loop_ = nullptr; }
    if (auth_)     { g_object_unref(auth_); auth_ = nullptr; }
    if (server_)   { g_object_unref(server_);   server_ = nullptr; }
    if (main_ctx_) { g_main_context_unref(main_ctx_); main_ctx_ = nullptr; }
    channels_.clear();
    started_ = false;
    shutting_down_.store(false);
  }

  RtspPublisherConfig cfg_;
  GstRTSPServer * server_{nullptr};
  GstRTSPAuth * auth_{nullptr};
  bool auth_enabled_{false};
  std::string auth_role_{"user"};
  GMainContext * main_ctx_{nullptr};
  GMainLoop * loop_{nullptr};
  std::thread glib_thread_;
  std::vector<std::unique_ptr<Channel>> channels_;
  std::atomic<bool> global_paused_{false};
  std::atomic<bool> shutting_down_{false};
  bool started_ = false;
};

}  // namespace

std::unique_ptr<IBackend> make_gstreamer_backend()
{
  return std::make_unique<GstreamerBackend>();
}

}  // namespace rtsp_internal
}  // namespace m_common
