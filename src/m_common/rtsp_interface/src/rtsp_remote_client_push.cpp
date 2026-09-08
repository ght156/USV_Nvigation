// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT

#include "m_common/rtsp_interface/rtsp_remote_client_push.hpp"

#include <atomic>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <cerrno>
#include <csignal>
#include <stdlib.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <thread>
#endif

#include <opencv2/imgproc.hpp>

#include <glib-object.h>
#include <glib.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/rtsp/gstrtsptransport.h>

#include "rtsp_device_detect.hpp"

namespace m_common
{
namespace
{

bool gst_has_factory(const char * name)
{
  GstElementFactory * f = gst_element_factory_find(name);
  if (!f) return false;
  gst_object_unref(f);
  return true;
}

#if defined(__linux__)
bool spawn_child_pdeathsig(
  const char * file, char * const argv[], pid_t * out_pid, std::string * err_msg)
{
  if (file == nullptr || argv == nullptr || out_pid == nullptr) {
    return false;
  }
  *out_pid = -1;
  const pid_t pid = ::fork();
  if (pid < 0) {
    if (err_msg != nullptr) {
      *err_msg = std::string("fork(") + file + ") 失败 errno=" + std::to_string(errno);
    }
    return false;
  }
  if (pid == 0) {
    (void)::prctl(PR_SET_PDEATHSIG, SIGTERM);
    if (::getppid() == 1) {
      _exit(127);
    }
    (void)::setpgid(0, 0);
    ::execvp(file, argv);
    _exit(127);
  }
  (void)::setpgid(pid, pid);
  *out_pid = pid;
  return true;
}
#endif

bool jetson_hw_h264_available()
{
  return gst_has_factory("nvv4l2h264enc") && gst_has_factory("nvvidconv");
}

bool jetson_hw_h265_available()
{
  return gst_has_factory("nvv4l2h265enc") && gst_has_factory("nvvidconv");
}

enum class H264Impl { kX264, kNvV4L2, kOpenH264 };
enum class H265Impl { kX265, kNvV4L2 };

H264Impl resolve_h264_impl(const std::string & raw)
{
  std::string c = raw;
  for (char & ch : c) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

  if (c == "nvv4l2" || c == "nvv4l2h264" || c == "nvv4l2h264enc" || c == "jetson" || c == "hw" ||
    c == "nvenc") {
    if (!jetson_hw_h264_available()) {
      throw std::runtime_error("rtsp_remote_push[gst]: h264_encoder=hw 但缺少 nvv4l2h264enc");
    }
    return H264Impl::kNvV4L2;
  }
  if (c == "x264" || c == "libx264" || c == "x264enc") {
    if (!gst_has_factory("x264enc")) {
      throw std::runtime_error("rtsp_remote_push[gst]: h264_encoder=x264 但系统无 x264enc");
    }
    return H264Impl::kX264;
  }
  if (c == "openh264" || c == "openh264enc" || c == "cisco-openh264" || c == "cisco_openh264") {
    if (!gst_has_factory("openh264enc")) {
      throw std::runtime_error(
        "rtsp_remote_push[gst]: h264_encoder=openh264 但系统无 openh264enc（gst-plugins-bad）");
    }
    return H264Impl::kOpenH264;
  }
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
      "rtsp_remote_push[gst]: h264_encoder=auto 但未找到编码器 "
      "(Jetson 需 nvv4l2h264enc+nvvidconv；其它平台优先 openh264enc，否则 x264enc)");
  }
  throw std::runtime_error(
    "rtsp_remote_push[gst]: unknown h264_encoder \"" + raw +
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
        "rtsp_remote_push[gst]: h265_encoder=hw 但缺少 nvv4l2h265enc 或 nvvidconv");
    }
    return H265Impl::kNvV4L2;
  }
  if (c == "x265" || c == "libx265" || c == "x265enc") {
    if (!gst_has_factory("x265enc")) {
      throw std::runtime_error("rtsp_remote_push[gst]: h265_encoder=x265 但系统无 x265enc");
    }
    return H265Impl::kX265;
  }
  if (c == "auto" || c.empty()) {
    if (m_common::rtsp_device_is_jetson() && jetson_hw_h265_available()) {
      return H265Impl::kNvV4L2;
    }
    if (gst_has_factory("x265enc")) {
      return H265Impl::kX265;
    }
    throw std::runtime_error(
      "rtsp_remote_push[gst]: h265_encoder=auto 但未找到编码器 "
      "(Jetson 需 nvv4l2h265enc+nvvidconv；其它平台需 x265enc)");
  }
  throw std::runtime_error(
    "rtsp_remote_push[gst]: unknown h265_encoder \"" + raw +
    "\" （可用 auto | x265 | x265enc | nvv4l2h265enc | jetson | hw）");
}

int gop_frames_from_fps(int fps)
{
  return std::max(15, (fps > 0) ? fps : 25);
}

std::string h264_branch(int kbps, H264Impl impl, int fps)
{
  if (kbps < 200) kbps = 200;
  const int gop = gop_frames_from_fps(fps);
  if (impl == H264Impl::kX264) {
    return std::string(
      "x264enc tune=zerolatency speed-preset=veryfast key-int-max=") +
      std::to_string(gop) +
      " threads=0 bframes=0 b-adapt=false "
      "ref=1 cabac=false dct8x8=false mb-tree=false rc-lookahead=0 sync-lookahead=0 sliced-threads=false "
      "bitrate=" +
      std::to_string(kbps) + " ! h264parse";
  }
  if (impl == H264Impl::kOpenH264) {
    // openh264enc.bitrate 单位为 bit/s；低复杂度利于实时推流
    return std::string("openh264enc bitrate=") + std::to_string(static_cast<unsigned>(kbps) * 1000U) +
      " rate-control=bitrate complexity=low ! h264parse";
  }
  if (impl == H264Impl::kNvV4L2) {
    // GOP≈1s；编码后 queue 不 leaky（leaky 会丢 P 帧导致播放花屏）。preset-level=2 Medium。
    return std::string("nvv4l2h264enc bitrate=") + std::to_string(kbps * 1000) +
      " iframeinterval=" + std::to_string(gop) + " idrinterval=" + std::to_string(gop) +
      " insert-sps-pps=true num-B-Frames=0 "
      "preset-level=2 profile=0 control-rate=1 maxperf-enable=true "
      "! queue max-size-buffers=8 max-size-time=0 max-size-bytes=0 ! h264parse";
  }
  throw std::logic_error("rtsp_remote_push[gst]: h264_branch: unexpected H264Impl");
}

std::string h265_branch(int kbps, H265Impl impl, int fps)
{
  if (kbps < 200) kbps = 200;
  const int gop = gop_frames_from_fps(fps);
  if (impl == H265Impl::kNvV4L2) {
    return std::string("nvv4l2h265enc bitrate=") + std::to_string(kbps * 1000) +
      " iframeinterval=" + std::to_string(gop) + " idrinterval=" + std::to_string(gop) +
      " insert-sps-pps=true num-B-Frames=0 "
      "preset-level=2 "
      "! queue max-size-buffers=8 max-size-time=0 max-size-bytes=0 ! h265parse";
  }
  return std::string(
           "x265enc speed-preset=veryfast tune=zerolatency key-int-max=") +
    std::to_string(gop) + " bitrate=" + std::to_string(kbps) + " ! h265parse";
}

std::string build_h264_client_launch(H264Impl impl, int kbps, int fps)
{
  // rtspclientsink 使用 request pad sink_%u，不能与 rtph264pay 由 gst_parse 静态链接；
  // 直接接入 h264parse 输出的 video/x-h264，由 sink 内部完成 RTP/RECORD。
  const std::string tail = " ! rtspclientsink name=rsink";
  // 编码前可丢旧原帧；编码后不得 leaky。
  const std::string q =
    "queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! ";
  if (impl == H264Impl::kNvV4L2) {
    // appsrc 直接出 BGRx（CPU 侧已转），nvvidconv VIC 上传 NVMM；无 videoconvert
    return std::string(
             "( appsrc name=ros_src is-live=true format=time ! "
             "video/x-raw,format=BGRx ! "
             "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! ") +
      q + h264_branch(kbps, impl, fps) + tail + " )";
  }
  return std::string("( appsrc name=ros_src is-live=true format=time ! videoconvert ! "
                     "video/x-raw,format=I420 ! ") +
    q + h264_branch(kbps, impl, fps) + tail + " )";
}

std::string build_h265_client_launch(H265Impl impl, int kbps, int fps)
{
  const std::string tail = " ! rtspclientsink name=rsink";
  const std::string q =
    "queue max-size-buffers=2 max-size-time=0 max-size-bytes=0 leaky=downstream ! ";
  if (impl == H265Impl::kNvV4L2) {
    return std::string(
             "( appsrc name=ros_src is-live=true format=time ! "
             "video/x-raw,format=BGRx ! "
             "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! ") +
      q + h265_branch(kbps, impl, fps) + tail + " )";
  }
  return std::string("( appsrc name=ros_src is-live=true format=time ! videoconvert ! "
                     "video/x-raw,format=I420 ! ") +
    q + h265_branch(kbps, impl, fps) + tail + " )";
}

struct RPushChannel;

struct RPushCtx
{
  RPushChannel * ch{nullptr};
  GstBuffer * buf{nullptr};
  int fw{0};
  int fh{0};
};

struct RPushChannel
{
  RPushChannel() { g_weak_ref_init(&appsrc_weak, nullptr); }
  ~RPushChannel()
  {
    for (auto & s : bgrx_slots) {
      if (s.data != nullptr) {
        std::free(s.data);
        s.data = nullptr;
      }
    }
    g_weak_ref_clear(&appsrc_weak);
  }
  RPushChannel(const RPushChannel &) = delete;
  RPushChannel & operator=(const RPushChannel &) = delete;

  std::string mount_path;
  std::string rtsp_app;
  std::string rtsp_stream;
  std::string remote_url;

  GstElement * pipeline{nullptr};
  GWeakRef appsrc_weak;
  std::mutex mu;
  /// 与 relocate_stream_idle_cb 互斥：同在 GLib main_ctx 上串行化 appsrc push 与管线 NULL/重配，避免 gst_app_src 断言崩溃
  std::mutex stream_serial_mtx;

  int pending_w{0}, pending_h{0};
  int locked_w{0}, locked_h{0};
  int fps_num{30}, fps_den{1};
  RtspCodec codec{RtspCodec::kH264};
  int bitrate_kbps{2000};
  std::uint64_t frame_index{0};
  /// 首帧 wall-clock，用于实时 PTS（避免按 fps 虚拟时间戳把码流排进未来）
  GstClockTime pts_base{GST_CLOCK_TIME_NONE};
  std::uint64_t pushed_total{0};

  std::atomic<bool> paused{false};
  std::uint64_t paused_drop_counter = 0;
  std::atomic<std::uint64_t> appsrc_push_fail_count{0};
  std::atomic<bool> logged_first_push{false};
  /// bus ERROR/EOS 或 appsrc push 失败时置位；重建管线成功后清零
  std::atomic<bool> push_broken{false};

  /// Jetson 硬编：appsrc 喂 BGRx，跳过 videoconvert；池内 wrap 避免每帧 memcpy+invoke
  bool jetson_nvmm_input{false};
  struct BgrxSlot
  {
    std::uint8_t * data{nullptr};
    std::size_t bytes{0};
    std::atomic<int> busy{0};
  };
  BgrxSlot bgrx_slots[3]{};
};

struct HealthPack
{
  RPushChannel * ch{nullptr};
  std::shared_ptr<std::promise<bool>> prom;
};

struct StopStreamPack
{
  RPushChannel * ch{nullptr};
  std::shared_ptr<std::promise<bool>> prom;
};

gboolean health_check_idle_cb(gpointer user_data)
{
  auto * pack = static_cast<HealthPack *>(user_data);
  bool ok = false;
  if (pack->ch != nullptr && !pack->ch->paused.load(std::memory_order_acquire) &&
      !pack->ch->push_broken.load(std::memory_order_acquire) &&
      pack->ch->pipeline != nullptr) {
    GstState state = GST_STATE_VOID_PENDING;
    GstState pending = GST_STATE_VOID_PENDING;
    const GstStateChangeReturn ret =
      gst_element_get_state(pack->ch->pipeline, &state, &pending, 0);
    ok = (ret != GST_STATE_CHANGE_FAILURE && state == GST_STATE_PLAYING);
    if (!ok) {
      pack->ch->push_broken.store(true, std::memory_order_release);
    }
  }
  if (pack->prom) {
    pack->prom->set_value(ok);
  }
  delete pack;
  return G_SOURCE_REMOVE;
}

gboolean session_check_idle_cb(gpointer user_data)
{
  auto * pack = static_cast<HealthPack *>(user_data);
  bool ok = false;
  if (pack->ch != nullptr && !pack->ch->push_broken.load(std::memory_order_acquire) &&
      pack->ch->pipeline != nullptr) {
    GstState state = GST_STATE_VOID_PENDING;
    GstState pending = GST_STATE_VOID_PENDING;
    const GstStateChangeReturn ret =
      gst_element_get_state(pack->ch->pipeline, &state, &pending, 0);
    ok = (ret != GST_STATE_CHANGE_FAILURE &&
          (state == GST_STATE_PLAYING || state == GST_STATE_PAUSED));
  }
  if (pack->prom) {
    pack->prom->set_value(ok);
  }
  delete pack;
  return G_SOURCE_REMOVE;
}

gboolean stop_stream_push_idle_cb(gpointer user_data)
{
  auto * pack = static_cast<StopStreamPack *>(user_data);
  bool ok = false;
  if (pack->ch != nullptr) {
    RPushChannel * ch = pack->ch;
    std::lock_guard<std::mutex> serial_lk(ch->stream_serial_mtx);
    if (ch->pipeline != nullptr) {
      gst_element_set_state(ch->pipeline, GST_STATE_NULL);
      GstState st = GST_STATE_VOID_PENDING;
      GstState pending = GST_STATE_VOID_PENDING;
      gst_element_get_state(ch->pipeline, &st, &pending, 10 * GST_SECOND);
      gst_object_unref(ch->pipeline);
      ch->pipeline = nullptr;
    }
    {
      std::lock_guard<std::mutex> lk(ch->mu);
      g_weak_ref_clear(&ch->appsrc_weak);
      g_weak_ref_init(&ch->appsrc_weak, nullptr);
      ch->frame_index = 0;
      ch->pts_base = GST_CLOCK_TIME_NONE;
    }
    ch->paused.store(true, std::memory_order_release);
    ch->push_broken.store(false, std::memory_order_release);
    ch->logged_first_push.store(false, std::memory_order_release);
    ch->appsrc_push_fail_count.store(0, std::memory_order_relaxed);
    ok = true;
    std::fprintf(
      stderr, "[rtsp_remote_push][gst] mount=\"%s\" 已停止并销毁 rtspclientsink 管线\n",
      ch->mount_path.c_str());
  }
  if (pack->prom) {
    pack->prom->set_value(ok);
  }
  delete pack;
  return G_SOURCE_REMOVE;
}

gboolean idle_rpush_buffer(gpointer user_data)
{
  auto * w = static_cast<RPushCtx *>(user_data);
  if (!w->buf || !w->ch) {
    if (w->buf) gst_buffer_unref(w->buf);
    delete w;
    return G_SOURCE_REMOVE;
  }
  std::lock_guard<std::mutex> serial_lk(w->ch->stream_serial_mtx);
  GstElement * src_el = nullptr;
  int fps_num = 25;
  int fps_den = 1;
  GstClockTime pts = 0;
  GstClockTime dur = 0;
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

    fps_num = std::max(1, w->ch->fps_num);
    fps_den = std::max(1, w->ch->fps_den);
    const GstClockTime now = gst_util_get_timestamp();
    if (w->ch->pts_base == GST_CLOCK_TIME_NONE) {
      w->ch->pts_base = now;
    }
    pts = (now > w->ch->pts_base) ? (now - w->ch->pts_base) : 0;
    dur = gst_util_uint64_scale(1, GST_SECOND * static_cast<guint64>(fps_den), static_cast<guint64>(fps_num));
  }

  GST_BUFFER_PTS(w->buf) = pts;
  GST_BUFFER_DURATION(w->buf) = dur;
  GST_BUFFER_DTS(w->buf) = pts;
  GST_BUFFER_FLAG_SET(w->buf, GST_BUFFER_FLAG_LIVE);
  const GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(src_el), w->buf);
  gst_object_unref(src_el);

  if (flow >= GST_FLOW_OK) {
    std::lock_guard<std::mutex> lk(w->ch->mu);
    ++w->ch->pushed_total;
    ++w->ch->frame_index;
    if (!w->ch->logged_first_push.exchange(true, std::memory_order_relaxed)) {
      std::fprintf(
        stderr,
        "[rtsp_remote_push][gst] mount=\"%s\" 首帧已进入管线（远端一般会在此后才响应 PLAY/DESCRIBE）\n",
        w->ch->mount_path.c_str());
    }
  } else {
    if (flow != GST_FLOW_FLUSHING) {
      w->ch->push_broken.store(true, std::memory_order_release);
    }
    const std::uint64_t n = w->ch->appsrc_push_fail_count.fetch_add(1, std::memory_order_relaxed) + 1;
    if (n <= 8 || (n % 250 == 0)) {
      std::fprintf(
        stderr, "[rtsp_remote_push][gst] mount=\"%s\" appsrc push-buffer failed flow=%d (%s)\n",
        w->ch->mount_path.c_str(), static_cast<int>(flow),
        gst_flow_get_name(flow));
    }
    gst_buffer_unref(w->buf);
  }
  delete w;
  return G_SOURCE_REMOVE;
}

gboolean rpush_bus_cb(GstBus *, GstMessage * msg, gpointer user_data)
{
  auto * ch = static_cast<RPushChannel *>(user_data);
  switch (GST_MESSAGE_TYPE(msg)) {
    case GST_MESSAGE_ERROR: {
      ch->push_broken.store(true, std::memory_order_release);
      GError * err = nullptr;
      gchar * dbg = nullptr;
      gst_message_parse_error(msg, &err, &dbg);
      std::fprintf(
        stderr, "[rtsp_remote_push][gst] mount=\"%s\" ERROR: %s | dbg=%s\n",
        ch->mount_path.c_str(), err ? err->message : "?", dbg ? dbg : "");
      if (err) g_error_free(err);
      if (dbg) g_free(dbg);
      break;
    }
    case GST_MESSAGE_EOS: {
      ch->push_broken.store(true, std::memory_order_release);
      std::fprintf(
        stderr, "[rtsp_remote_push][gst] mount=\"%s\" EOS（远端可能已断连）\n",
        ch->mount_path.c_str());
      break;
    }
    case GST_MESSAGE_WARNING: {
      GError * err = nullptr;
      gchar * dbg = nullptr;
      gst_message_parse_warning(msg, &err, &dbg);
      std::fprintf(
        stderr, "[rtsp_remote_push][gst] mount=\"%s\" WARN: %s\n",
        ch->mount_path.c_str(), err ? err->message : "?");
      if (err) g_error_free(err);
      if (dbg) g_free(dbg);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

void rpush_bgrx_clear(RPushChannel * ch)
{
  for (auto & s : ch->bgrx_slots) {
    if (s.data != nullptr) {
      std::free(s.data);
      s.data = nullptr;
    }
    s.bytes = 0;
    s.busy.store(0, std::memory_order_relaxed);
  }
}

bool rpush_bgrx_ensure(RPushChannel * ch, int w, int h)
{
  const std::size_t need =
    static_cast<std::size_t>(std::max(1, w)) * static_cast<std::size_t>(std::max(1, h)) * 4U;
  if (ch->bgrx_slots[0].data != nullptr && ch->bgrx_slots[0].bytes == need) {
    return true;
  }
  rpush_bgrx_clear(ch);
  for (auto & s : ch->bgrx_slots) {
    void * p = nullptr;
    if (posix_memalign(&p, 4096, need) != 0 || p == nullptr) {
      rpush_bgrx_clear(ch);
      return false;
    }
    s.data = static_cast<std::uint8_t *>(p);
    s.bytes = need;
    s.busy.store(0, std::memory_order_relaxed);
  }
  return true;
}

void rpush_bgrx_wrap_notify(gpointer data)
{
  auto * busy = static_cast<std::atomic<int> *>(data);
  busy->store(0, std::memory_order_release);
}

bool rpush_push_jetson_bgrx(RPushChannel * ch, const cv::Mat & bgr_in)
{
  cv::Mat img = bgr_in;
  if (img.empty() || img.cols < 1 || img.rows < 1) return false;

  std::unique_lock<std::mutex> serial_lk(ch->stream_serial_mtx, std::try_to_lock);
  if (!serial_lk.owns_lock()) return false;

  int locked_w = 0;
  int locked_h = 0;
  {
    std::lock_guard<std::mutex> lk(ch->mu);
    ch->pending_w = img.cols;
    ch->pending_h = img.rows;
    locked_w = ch->locked_w;
    locked_h = ch->locked_h;
  }
  if (locked_w > 0 && locked_h > 0 && (img.cols != locked_w || img.rows != locked_h)) {
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(locked_w, locked_h), 0, 0, cv::INTER_LINEAR);
    img = std::move(resized);
  }
  const int fw = img.cols;
  const int fh = img.rows;
  if (!rpush_bgrx_ensure(ch, fw, fh)) return false;

  RPushChannel::BgrxSlot * slot = nullptr;
  for (auto & s : ch->bgrx_slots) {
    int expected = 0;
    if (s.busy.compare_exchange_strong(expected, 1, std::memory_order_acq_rel)) {
      slot = &s;
      break;
    }
  }
  if (slot == nullptr) return false;

  if (!img.isContinuous()) img = img.clone();
  cv::Mat bgrx(fh, fw, CV_8UC4, slot->data);
  cv::cvtColor(img, bgrx, cv::COLOR_BGR2BGRA);

  GstBuffer * buf = gst_buffer_new_wrapped_full(
    static_cast<GstMemoryFlags>(0), slot->data, slot->bytes, 0, slot->bytes, &slot->busy,
    rpush_bgrx_wrap_notify);
  if (buf == nullptr) {
    slot->busy.store(0, std::memory_order_release);
    return false;
  }

  gpointer p = nullptr;
  int fps_num = 25;
  int fps_den = 1;
  GstClockTime pts = 0;
  GstClockTime dur = 0;
  {
    std::lock_guard<std::mutex> lk(ch->mu);
    p = g_weak_ref_get(&ch->appsrc_weak);
    if (p == nullptr) {
      gst_buffer_unref(buf);
      return false;
    }
    if (ch->locked_w <= 0 || ch->locked_h <= 0) {
      ch->locked_w = fw;
      ch->locked_h = fh;
      GstCaps * caps = gst_caps_new_simple(
        "video/x-raw", "format", G_TYPE_STRING, "BGRx", "width", G_TYPE_INT, fw, "height",
        G_TYPE_INT, fh, "framerate", GST_TYPE_FRACTION, ch->fps_num, ch->fps_den, nullptr);
      gst_app_src_set_caps(GST_APP_SRC(p), caps);
      gst_caps_unref(caps);
    }
    fps_num = std::max(1, ch->fps_num);
    fps_den = std::max(1, ch->fps_den);
    const GstClockTime now = gst_util_get_timestamp();
    if (ch->pts_base == GST_CLOCK_TIME_NONE) {
      ch->pts_base = now;
    }
    pts = (now > ch->pts_base) ? (now - ch->pts_base) : 0;
    dur = gst_util_uint64_scale(
      1, GST_SECOND * static_cast<guint64>(fps_den), static_cast<guint64>(fps_num));
  }

  GST_BUFFER_PTS(buf) = pts;
  GST_BUFFER_DURATION(buf) = dur;
  GST_BUFFER_DTS(buf) = pts;
  GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);
  const GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(p), buf);
  gst_object_unref(GST_ELEMENT(p));

  if (flow >= GST_FLOW_OK) {
    std::lock_guard<std::mutex> lk(ch->mu);
    ++ch->pushed_total;
    ++ch->frame_index;
    if (!ch->logged_first_push.exchange(true, std::memory_order_relaxed)) {
      std::fprintf(
        stderr,
        "[rtsp_remote_push][gst] mount=\"%s\" 首帧已进入管线（BGRx wrap→nvvidconv NVMM，无 videoconvert/memcpy/invoke）\n",
        ch->mount_path.c_str());
    }
    return true;
  }
  if (flow != GST_FLOW_FLUSHING) {
    ch->push_broken.store(true, std::memory_order_release);
  }
  const std::uint64_t n = ch->appsrc_push_fail_count.fetch_add(1, std::memory_order_relaxed) + 1;
  if (n <= 8 || (n % 250 == 0)) {
    std::fprintf(
      stderr, "[rtsp_remote_push][gst] mount=\"%s\" appsrc push-buffer failed flow=%d (%s)\n",
      ch->mount_path.c_str(), static_cast<int>(flow), gst_flow_get_name(flow));
  }
  gst_buffer_unref(buf);
  return false;
}

void rpush_configure_appsrc(RPushChannel * ch, GstElement * pipeline)
{
  GstElement * src = gst_bin_get_by_name(GST_BIN(pipeline), "ros_src");
  if (!src) return;
  g_object_set(
    src, "is-live", TRUE, "format", GST_FORMAT_TIME, "do-timestamp", FALSE, "block", FALSE,
    "max-buffers", guint64{1}, "leaky-type", GST_APP_LEAKY_TYPE_DOWNSTREAM, nullptr);
  // 首帧到达前 pending 未知；勿默认 640×480，否则 locked 会把所有帧强行缩放至此分辨率。
  {
    std::lock_guard<std::mutex> lk(ch->mu);
    ch->locked_w = 0;
    ch->locked_h = 0;
  }
  rpush_bgrx_clear(ch);
  gst_app_src_set_caps(GST_APP_SRC(src), nullptr);
  {
    std::lock_guard<std::mutex> lk(ch->mu);
    g_weak_ref_set(&ch->appsrc_weak, G_OBJECT(src));
    ch->frame_index = 0;
    ch->pts_base = GST_CLOCK_TIME_NONE;
  }
  gst_object_unref(src);
}

struct RelocatePack
{
  RPushChannel * ch{nullptr};
  std::string new_url;
  std::shared_ptr<std::promise<std::pair<bool, std::string>>> prom;
  RtspRemoteClientPushConfig cfg;
};

gboolean relocate_stream_idle_cb(gpointer user_data)
{
  auto * pack = static_cast<RelocatePack *>(user_data);
  auto done = [&](bool ok, std::string err) {
    pack->prom->set_value({ok, std::move(err)});
    delete pack;
  };

  RPushChannel * ch = pack->ch;
  if (ch == nullptr) {
    done(false, "relocate_stream_remote_url: channel missing");
    return G_SOURCE_REMOVE;
  }

  const std::string & trimmed = pack->new_url;
  if (trimmed.empty()) {
    done(false, "relocate_stream_remote_url: new_rtsp_url empty");
    return G_SOURCE_REMOVE;
  }

  const RtspRemoteClientPushConfig & cfg = pack->cfg;

  std::lock_guard<std::mutex> serial_lk(ch->stream_serial_mtx);

  if (ch->remote_url == trimmed && !ch->push_broken.load(std::memory_order_acquire)) {
    if (ch->pipeline != nullptr) {
      GstState state = GST_STATE_VOID_PENDING;
      GstState pending = GST_STATE_VOID_PENDING;
      const GstStateChangeReturn gst_ret =
        gst_element_get_state(ch->pipeline, &state, &pending, 0);
      if (gst_ret != GST_STATE_CHANGE_FAILURE &&
          (state == GST_STATE_PLAYING || state == GST_STATE_PAUSED)) {
        const bool was_paused = ch->paused.load(std::memory_order_acquire);
        ch->paused.store(false, std::memory_order_release);
        std::fprintf(
          stderr,
          was_paused
            ? "[rtsp_remote_push][gst] mount=\"%s\" 已从 pause 恢复推流（无需重建）-> %s\n"
            : "[rtsp_remote_push][gst] mount=\"%s\" 会话仍有效，恢复推流（无需重建）-> %s\n",
          ch->mount_path.c_str(), trimmed.c_str());
        done(true, {});
        return G_SOURCE_REMOVE;
      }
    }
    // URL 未变但管线已断开/非 PLAYING：继续走下方 tear_down + 重建
  }

  if (ch->codec == RtspCodec::kMjpeg) {
    done(false, "relocate_stream_remote_url: MJPEG 未支持");
    return G_SOURCE_REMOVE;
  }

  auto tear_down_pipeline = [&]() {
    if (ch->pipeline == nullptr) return;
    gst_element_set_state(ch->pipeline, GST_STATE_NULL);
    GstState st = GST_STATE_VOID_PENDING;
    GstState pd = GST_STATE_VOID_PENDING;
    gst_element_get_state(ch->pipeline, &st, &pd, 10 * GST_SECOND);
    gst_object_unref(ch->pipeline);
    ch->pipeline = nullptr;
  };

  tear_down_pipeline();

  {
    std::lock_guard<std::mutex> lk(ch->mu);
    g_weak_ref_clear(&ch->appsrc_weak);
    g_weak_ref_init(&ch->appsrc_weak, nullptr);
    ch->logged_first_push.store(false, std::memory_order_relaxed);
  }

  const int kbps = (ch->bitrate_kbps >= 200) ? ch->bitrate_kbps : 200;
  const int fps = (ch->fps_num > 0) ? ch->fps_num : 25;
  std::string launch = (ch->codec == RtspCodec::kH265)
                           ? build_h265_client_launch(resolve_h265_impl(cfg.h265_encoder), kbps, fps)
                           : build_h264_client_launch(resolve_h264_impl(cfg.h264_encoder), kbps, fps);

  GError * perr = nullptr;
  ch->pipeline = gst_parse_launch(launch.c_str(), &perr);
  if (ch->pipeline == nullptr) {
    std::string msg = perr && perr->message ? perr->message : "gst_parse_launch failed";
    if (perr != nullptr) g_error_free(perr);
    done(false, "relocate_stream_remote_url: " + msg + " mount=" + ch->mount_path);
    return G_SOURCE_REMOVE;
  }
  if (perr != nullptr) g_error_free(perr);

  GstElement * sink = gst_bin_get_by_name(GST_BIN(ch->pipeline), "rsink");
  if (sink == nullptr) {
    tear_down_pipeline();
    done(false, "relocate_stream_remote_url: pipeline 缺少 rsink mount=" + ch->mount_path);
    return G_SOURCE_REMOVE;
  }
  g_object_set(
    sink, "location", trimmed.c_str(), "protocols",
    static_cast<GstRTSPLowerTrans>(GST_RTSP_LOWER_TRANS_TCP), nullptr);
  if (!cfg.auth_username.empty()) {
    g_object_set(
      sink, "user-id", cfg.auth_username.c_str(), "user-pw", cfg.auth_password.c_str(), nullptr);
  }
  if (std::getenv("RTSP_CLIENT_SINK_DEBUG") != nullptr) {
    g_object_set(sink, "debug", TRUE, nullptr);
  }
  gst_object_unref(sink);

  GstBus * bus = gst_element_get_bus(ch->pipeline);
  if (bus != nullptr) {
    gst_bus_add_watch(bus, rpush_bus_cb, ch);
    gst_object_unref(bus);
  }

  GstStateChangeReturn ret = gst_element_set_state(ch->pipeline, GST_STATE_READY);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    tear_down_pipeline();
    done(false, "relocate_stream_remote_url: READY failed mount=" + ch->mount_path);
    return G_SOURCE_REMOVE;
  }
  rpush_configure_appsrc(ch, ch->pipeline);
  ret = gst_element_set_state(ch->pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    tear_down_pipeline();
    done(false, "relocate_stream_remote_url: PLAYING failed mount=" + ch->mount_path);
    return G_SOURCE_REMOVE;
  }

  GstState state = GST_STATE_VOID_PENDING;
  GstState pending = GST_STATE_VOID_PENDING;
  ret = gst_element_get_state(ch->pipeline, &state, &pending, 20 * GST_SECOND);
  if (ret == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING) {
    std::string detail =
      "relocate_stream_remote_url: 未能稳定进入 PLAYING mount=" + ch->mount_path + " url=" + trimmed +
      " get_state_ret=" + std::to_string(static_cast<int>(ret)) +
      " state=" + std::to_string(static_cast<int>(state));
    GstBus * rbus = gst_element_get_bus(ch->pipeline);
    if (rbus != nullptr) {
      GstMessage * msg = gst_bus_timed_pop_filtered(
        rbus, static_cast<GstClockTime>(2 * GST_SECOND), GST_MESSAGE_ERROR);
      if (msg != nullptr) {
        GError * err = nullptr;
        gchar * dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        if (err != nullptr && err->message != nullptr) detail += std::string(" | ") + err->message;
        if (dbg != nullptr) detail += std::string(" | dbg=") + dbg;
        if (err != nullptr) g_error_free(err);
        if (dbg != nullptr) g_free(dbg);
        gst_message_unref(msg);
      }
      gst_object_unref(rbus);
    }
    tear_down_pipeline();
    done(false, detail);
    return G_SOURCE_REMOVE;
  }

  {
    std::lock_guard<std::mutex> lk(ch->mu);
    ch->remote_url = trimmed;
  }
  ch->push_broken.store(false, std::memory_order_release);
  ch->appsrc_push_fail_count.store(0, std::memory_order_relaxed);
  ch->paused.store(false, std::memory_order_release);

  std::fprintf(
    stderr, "[rtsp_remote_push][gst] mount=\"%s\" 已切换远端 ingest（重建管线）-> %s\n", ch->mount_path.c_str(),
    trimmed.c_str());
  done(true, {});
  return G_SOURCE_REMOVE;
}

}  // namespace

class RtspRemoteClientPush::Impl
{
public:
  ~Impl() { teardown(); }

  void start(const RtspRemoteClientPushConfig & cfg)
  {
    if (started_) throw std::runtime_error("RtspRemoteClientPush: already started");
    if (cfg.streams.empty()) throw std::runtime_error("RtspRemoteClientPush: streams empty");
    if (!gst_element_factory_find("rtspclientsink")) {
      throw std::runtime_error(
        "RtspRemoteClientPush: 无 rtspclientsink 元素（请安装 gstreamer1.0-plugins-bad）");
    }
    cfg_ = cfg;

    const bool auth_u = !cfg_.auth_username.empty();
    const bool auth_p = !cfg_.auth_password.empty();
    if (auth_u != auth_p) {
      throw std::runtime_error(
        "RtspRemoteClientPush: auth_username 与 auth_password 须同时设置或同时为空");
    }

    static std::once_flag s_gst_init;
    std::call_once(s_gst_init, []() { gst_init(nullptr, nullptr); });

    main_ctx_ = g_main_context_new();
    if (!main_ctx_) throw std::runtime_error("RtspRemoteClientPush: g_main_context_new failed");
    loop_ = g_main_loop_new(main_ctx_, FALSE);
    if (!loop_) throw std::runtime_error("RtspRemoteClientPush: g_main_loop_new failed");
    glib_thread_ = std::thread([this]() { g_main_loop_run(loop_); });

    try {
      channels_.reserve(cfg_.streams.size());
      for (const auto & e : cfg_.streams) {
        const auto & s = e.spec;
        if (s.codec == RtspCodec::kMjpeg) {
          throw std::runtime_error("RtspRemoteClientPush: MJPEG 未支持");
        }
        auto ch = std::make_unique<RPushChannel>();
        ch->mount_path = s.mount_path;
        while (!ch->mount_path.empty() && ch->mount_path[0] == '/') ch->mount_path.erase(0, 1);
        const auto slash = ch->mount_path.find('/');
        if (slash == std::string::npos) {
          ch->rtsp_app = "live";
          ch->rtsp_stream = ch->mount_path;
        } else {
          ch->rtsp_app = ch->mount_path.substr(0, slash);
          ch->rtsp_stream = ch->mount_path.substr(slash + 1);
        }
        if (ch->rtsp_app.empty() || ch->rtsp_stream.empty()) {
          throw std::runtime_error("RtspRemoteClientPush: invalid mount_path: " + s.mount_path);
        }
        ch->remote_url = e.remote_rtsp_url;
        ch->fps_num = (s.fps > 0) ? s.fps : 25;
        ch->fps_den = 1;
        ch->codec = s.codec;
        ch->bitrate_kbps = (s.bitrate_kbps >= 200) ? s.bitrate_kbps : 200;
        ch->jetson_nvmm_input = (s.codec == RtspCodec::kH265)
                                  ? (resolve_h265_impl(cfg_.h265_encoder) == H265Impl::kNvV4L2)
                                  : (resolve_h264_impl(cfg_.h264_encoder) == H264Impl::kNvV4L2);

        const int kbps = ch->bitrate_kbps;
        const int fps = ch->fps_num;
        std::string launch = (s.codec == RtspCodec::kH265)
                               ? build_h265_client_launch(resolve_h265_impl(cfg_.h265_encoder), kbps, fps)
                               : build_h264_client_launch(resolve_h264_impl(cfg_.h264_encoder), kbps, fps);

        GError * perr = nullptr;
        ch->pipeline = gst_parse_launch(launch.c_str(), &perr);
        if (!ch->pipeline) {
          std::string msg = perr && perr->message ? perr->message : "gst_parse_launch failed";
          if (perr) g_error_free(perr);
          throw std::runtime_error("RtspRemoteClientPush: " + msg);
        }
        if (perr) g_error_free(perr);

        GstElement * sink = gst_bin_get_by_name(GST_BIN(ch->pipeline), "rsink");
        if (!sink) {
          gst_object_unref(ch->pipeline);
          ch->pipeline = nullptr;
          throw std::runtime_error("RtspRemoteClientPush: pipeline 缺少 rsink");
        }
        g_object_set(
          sink, "location", ch->remote_url.c_str(), "protocols",
          static_cast<GstRTSPLowerTrans>(GST_RTSP_LOWER_TRANS_TCP), nullptr);
        if (!cfg_.auth_username.empty()) {
          g_object_set(
            sink, "user-id", cfg_.auth_username.c_str(), "user-pw", cfg_.auth_password.c_str(),
            nullptr);
        }
        if (std::getenv("RTSP_CLIENT_SINK_DEBUG") != nullptr) {
          g_object_set(sink, "debug", TRUE, nullptr);
        }
        gst_object_unref(sink);

        GstBus * bus = gst_element_get_bus(ch->pipeline);
        if (bus) {
          gst_bus_add_watch(bus, rpush_bus_cb, ch.get());
          gst_object_unref(bus);
        }

        channels_.push_back(std::move(ch));
      }

      for (auto & uch : channels_) {
        RPushChannel * ch = uch.get();
        GstStateChangeReturn ret = gst_element_set_state(ch->pipeline, GST_STATE_READY);
        if (ret == GST_STATE_CHANGE_FAILURE) {
          throw std::runtime_error("RtspRemoteClientPush: READY 失败 mount=" + ch->mount_path);
        }
        rpush_configure_appsrc(ch, ch->pipeline);
        ret = gst_element_set_state(ch->pipeline, GST_STATE_PLAYING);
        if (ret == GST_STATE_CHANGE_FAILURE) {
          throw std::runtime_error("RtspRemoteClientPush: PLAYING 失败 mount=" + ch->mount_path);
        }
        // RECORD/TCP 建链常为异步；仅检查 set_state 返回值会漏掉「稍后失败」。等待稳定 PLAYING 并抓取 ERROR。
        GstState state = GST_STATE_VOID_PENDING;
        GstState pending = GST_STATE_VOID_PENDING;
        ret = gst_element_get_state(ch->pipeline, &state, &pending, 20 * GST_SECOND);
        if (ret == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING) {
          std::string detail =
            "RtspRemoteClientPush: 未能稳定进入 PLAYING（常与远端 RECORD/路径/防火墙有关）mount=" +
            ch->mount_path + " url=" + ch->remote_url +
            " get_state_ret=" + std::to_string(static_cast<int>(ret)) +
            " state=" + std::to_string(static_cast<int>(state));
          GstBus * bus = gst_element_get_bus(ch->pipeline);
          if (bus) {
            GstMessage * msg = gst_bus_timed_pop_filtered(
              bus, static_cast<GstClockTime>(2 * GST_SECOND), GST_MESSAGE_ERROR);
            if (msg) {
              GError * err = nullptr;
              gchar * dbg = nullptr;
              gst_message_parse_error(msg, &err, &dbg);
              if (err && err->message) detail += std::string(" | ") + err->message;
              if (dbg) detail += std::string(" | dbg=") + dbg;
              if (err) g_error_free(err);
              if (dbg) g_free(dbg);
              gst_message_unref(msg);
            }
            gst_object_unref(bus);
          }
          throw std::runtime_error(detail);
        }
        ch->push_broken.store(false, std::memory_order_release);
      }
    } catch (...) {
      teardown();
      throw;
    }
    started_ = true;

    std::fprintf(stderr, "[rtsp_remote_push][gst] 已向远端 ingest 推流，路数=%zu\n", channels_.size());
    for (std::size_t i = 0; i < channels_.size(); ++i) {
      std::fprintf(
        stderr, "[rtsp_remote_push][gst]  #%zu mount=\"%s\" -> %s%s\n", i,
        channels_[i]->mount_path.c_str(), channels_[i]->remote_url.c_str(),
        channels_[i]->jetson_nvmm_input ? " (BGRx wrap→nvvidconv NVMM)" : "");
    }
  }

  bool append_stream(const RtspRemotePushEntry & ent, std::string * err_msg)
  {
    if (!started_) {
      if (err_msg != nullptr) *err_msg = "RtspRemoteClientPush 未 start，无法 append_stream";
      return false;
    }
    if (shutting_down_.load()) {
      if (err_msg != nullptr) *err_msg = "append_stream: 正在关闭";
      return false;
    }
    const auto & s = ent.spec;
    if (s.codec == RtspCodec::kMjpeg) {
      if (err_msg != nullptr) *err_msg = "append_stream: MJPEG 未支持";
      return false;
    }
    std::string lookup = s.mount_path;
    while (!lookup.empty() && lookup.front() == '/') lookup.erase(lookup.begin());
    if (find_channel(lookup).has_value()) {
      if (err_msg != nullptr) *err_msg = std::string("append_stream: mount 已存在 ") + lookup;
      return false;
    }

    auto ch = std::make_unique<RPushChannel>();
    ch->mount_path = s.mount_path;
    while (!ch->mount_path.empty() && ch->mount_path[0] == '/') ch->mount_path.erase(0, 1);
    const auto slash = ch->mount_path.find('/');
    if (slash == std::string::npos) {
      ch->rtsp_app = "live";
      ch->rtsp_stream = ch->mount_path;
    } else {
      ch->rtsp_app = ch->mount_path.substr(0, slash);
      ch->rtsp_stream = ch->mount_path.substr(slash + 1);
    }
    if (ch->rtsp_app.empty() || ch->rtsp_stream.empty()) {
      if (err_msg != nullptr) *err_msg = "append_stream: invalid mount_path: " + s.mount_path;
      return false;
    }
    ch->remote_url = ent.remote_rtsp_url;
    ch->fps_num = (s.fps > 0) ? s.fps : 25;
    ch->fps_den = 1;
    ch->codec = s.codec;
    ch->bitrate_kbps = (s.bitrate_kbps >= 200) ? s.bitrate_kbps : 200;
    ch->jetson_nvmm_input = (s.codec == RtspCodec::kH265)
                              ? (resolve_h265_impl(cfg_.h265_encoder) == H265Impl::kNvV4L2)
                              : (resolve_h264_impl(cfg_.h264_encoder) == H264Impl::kNvV4L2);

    const int kbps = ch->bitrate_kbps;
    const int fps = ch->fps_num;
    std::string launch = (s.codec == RtspCodec::kH265)
                           ? build_h265_client_launch(resolve_h265_impl(cfg_.h265_encoder), kbps, fps)
                           : build_h264_client_launch(resolve_h264_impl(cfg_.h264_encoder), kbps, fps);

    GError * perr = nullptr;
    ch->pipeline = gst_parse_launch(launch.c_str(), &perr);
    if (ch->pipeline == nullptr) {
      std::string msg = perr && perr->message ? perr->message : "gst_parse_launch failed";
      if (perr != nullptr) g_error_free(perr);
      if (err_msg != nullptr) *err_msg = "append_stream: " + msg;
      return false;
    }
    if (perr != nullptr) g_error_free(perr);

    GstElement * sink = gst_bin_get_by_name(GST_BIN(ch->pipeline), "rsink");
    if (sink == nullptr) {
      gst_object_unref(ch->pipeline);
      ch->pipeline = nullptr;
      if (err_msg != nullptr) *err_msg = "append_stream: pipeline 缺少 rsink";
      return false;
    }
    g_object_set(
      sink, "location", ch->remote_url.c_str(), "protocols",
      static_cast<GstRTSPLowerTrans>(GST_RTSP_LOWER_TRANS_TCP), nullptr);
    if (!cfg_.auth_username.empty()) {
      g_object_set(
        sink, "user-id", cfg_.auth_username.c_str(), "user-pw", cfg_.auth_password.c_str(), nullptr);
    }
    if (std::getenv("RTSP_CLIENT_SINK_DEBUG") != nullptr) {
      g_object_set(sink, "debug", TRUE, nullptr);
    }
    gst_object_unref(sink);

    GstBus * bus = gst_element_get_bus(ch->pipeline);
    if (bus != nullptr) {
      gst_bus_add_watch(bus, rpush_bus_cb, ch.get());
      gst_object_unref(bus);
    }

    RPushChannel * ch_raw = ch.get();
    channels_.push_back(std::move(ch));

    GstStateChangeReturn ret = gst_element_set_state(ch_raw->pipeline, GST_STATE_READY);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      const std::string mnt = ch_raw->mount_path;
      gst_element_set_state(ch_raw->pipeline, GST_STATE_NULL);
      gst_object_unref(ch_raw->pipeline);
      ch_raw->pipeline = nullptr;
      channels_.pop_back();
      if (err_msg != nullptr) *err_msg = "append_stream: READY 失败 mount=" + mnt;
      return false;
    }
    rpush_configure_appsrc(ch_raw, ch_raw->pipeline);
    ret = gst_element_set_state(ch_raw->pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      const std::string mnt = ch_raw->mount_path;
      gst_element_set_state(ch_raw->pipeline, GST_STATE_NULL);
      gst_object_unref(ch_raw->pipeline);
      ch_raw->pipeline = nullptr;
      channels_.pop_back();
      if (err_msg != nullptr) *err_msg = "append_stream: PLAYING 失败 mount=" + mnt;
      return false;
    }
    GstState state = GST_STATE_VOID_PENDING;
    GstState pending = GST_STATE_VOID_PENDING;
    ret = gst_element_get_state(ch_raw->pipeline, &state, &pending, 20 * GST_SECOND);
    if (ret == GST_STATE_CHANGE_FAILURE || state != GST_STATE_PLAYING) {
      std::string detail =
        "append_stream: 未能稳定进入 PLAYING mount=" + ch_raw->mount_path + " url=" + ch_raw->remote_url +
        " get_state_ret=" + std::to_string(static_cast<int>(ret)) +
        " state=" + std::to_string(static_cast<int>(state));
      GstBus * rbus = gst_element_get_bus(ch_raw->pipeline);
      if (rbus != nullptr) {
        GstMessage * msg = gst_bus_timed_pop_filtered(
          rbus, static_cast<GstClockTime>(2 * GST_SECOND), GST_MESSAGE_ERROR);
        if (msg != nullptr) {
          GError * gerr = nullptr;
          gchar * dbg = nullptr;
          gst_message_parse_error(msg, &gerr, &dbg);
          if (gerr != nullptr && gerr->message != nullptr) detail += std::string(" | ") + gerr->message;
          if (dbg != nullptr) detail += std::string(" | dbg=") + dbg;
          if (gerr != nullptr) g_error_free(gerr);
          if (dbg != nullptr) g_free(dbg);
          gst_message_unref(msg);
        }
        gst_object_unref(rbus);
      }
      gst_element_set_state(ch_raw->pipeline, GST_STATE_NULL);
      gst_object_unref(ch_raw->pipeline);
      ch_raw->pipeline = nullptr;
      channels_.pop_back();
      if (err_msg != nullptr) *err_msg = detail;
      return false;
    }
    ch_raw->push_broken.store(false, std::memory_order_release);

    std::fprintf(
      stderr, "[rtsp_remote_push][gst] append_stream mount=\"%s\" -> %s%s\n", ch_raw->mount_path.c_str(),
      ch_raw->remote_url.c_str(), ch_raw->jetson_nvmm_input ? " (BGRx wrap→nvvidconv NVMM)" : "");
    return true;
  }

  void stop() { teardown(); }

  bool push(std::size_t idx, const cv::Mat & bgr_in)
  {
    if (!started_ || idx >= channels_.size()) return false;
    auto & ch = channels_[idx];
    if (!ch || bgr_in.empty()) return false;
    if (global_paused_.load(std::memory_order_acquire) || ch->paused.load()) {
      ++ch->paused_drop_counter;
      return false;
    }
    if (ch->jetson_nvmm_input) {
      return rpush_push_jetson_bgrx(ch.get(), bgr_in);
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
    if (!gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
      gst_buffer_unref(buf);
      return false;
    }
    std::memcpy(map.data, img.ptr(), nbytes);
    gst_buffer_unmap(buf, &map);

    auto * ctx = new RPushCtx;
    ctx->ch = ch.get();
    ctx->buf = buf;
    ctx->fw = img.cols;
    ctx->fh = img.rows;
    g_main_context_invoke(g_main_loop_get_context(loop_), idle_rpush_buffer, ctx);
    return true;
  }

  void set_global_paused(bool paused) { global_paused_.store(paused); }

  void set_stream_paused(const std::string & key, bool paused)
  {
    for (auto & ch : channels_) {
      if (!ch) continue;
      const bool match =
        (key == ch->mount_path) || (key == ch->rtsp_app + "/" + ch->rtsp_stream);
      if (match) ch->paused.store(paused);
    }
  }

  std::string url(std::size_t idx) const
  {
    if (idx >= channels_.size() || !channels_[idx]) return {};
    return channels_[idx]->remote_url;
  }

  std::optional<std::size_t> find_channel(const std::string & key) const
  {
    for (std::size_t i = 0; i < channels_.size(); ++i) {
      const auto & ch = channels_[i];
      if (!ch) continue;
      if (ch->mount_path == key) return i;
      if (ch->rtsp_app + "/" + ch->rtsp_stream == key) return i;
    }
    return std::nullopt;
  }

  std::uint64_t pushed_frames(std::size_t idx) const
  {
    if (idx >= channels_.size() || !channels_[idx]) return 0;
    return channels_[idx]->pushed_total;
  }

  bool stream_push_healthy(const std::string & mount_key) const
  {
    if (!started_ || main_ctx_ == nullptr) return false;
    std::string key = mount_key;
    while (!key.empty() && key.front() == '/') key.erase(key.begin());
    const auto idx = find_channel(key);
    if (!idx.has_value() || !channels_[*idx]) return false;

    auto prom = std::make_shared<std::promise<bool>>();
    std::future<bool> fut = prom->get_future();
    auto * pack = new HealthPack{channels_[*idx].get(), prom};
    g_main_context_invoke(main_ctx_, health_check_idle_cb, pack);
    return fut.get();
  }

  bool stream_push_session_exists(const std::string & mount_key) const
  {
    if (!started_ || main_ctx_ == nullptr) return false;
    std::string key = mount_key;
    while (!key.empty() && key.front() == '/') key.erase(key.begin());
    const auto idx = find_channel(key);
    if (!idx.has_value() || !channels_[*idx]) return false;

    auto prom = std::make_shared<std::promise<bool>>();
    std::future<bool> fut = prom->get_future();
    auto * pack = new HealthPack{channels_[*idx].get(), prom};
    g_main_context_invoke(main_ctx_, session_check_idle_cb, pack);
    return fut.get();
  }

  bool stream_push_paused(const std::string & mount_key) const
  {
    std::string key = mount_key;
    while (!key.empty() && key.front() == '/') key.erase(key.begin());
    const auto idx = find_channel(key);
    if (!idx.has_value() || !channels_[*idx]) return false;
    return channels_[*idx]->paused.load(std::memory_order_acquire);
  }

  bool stop_stream_push(const std::string & mount_key)
  {
    if (!started_ || main_ctx_ == nullptr) return false;
    std::string key = mount_key;
    while (!key.empty() && key.front() == '/') key.erase(key.begin());
    const auto idx = find_channel(key);
    if (!idx.has_value() || !channels_[*idx]) return false;

    auto prom = std::make_shared<std::promise<bool>>();
    std::future<bool> fut = prom->get_future();
    auto * pack = new StopStreamPack{channels_[*idx].get(), prom};
    g_main_context_invoke(main_ctx_, stop_stream_push_idle_cb, pack);
    return fut.get();
  }

  bool relocate_stream_remote_url(const std::string & mount_key, const std::string & new_url, std::string * err_msg)
  {
    if (!started_) {
      if (err_msg != nullptr) *err_msg = "RtspRemoteClientPush 未 start";
      return false;
    }
    std::string key = mount_key;
    while (!key.empty() && key.front() == '/') key.erase(key.begin());
    const auto idx = find_channel(key);
    if (!idx.has_value()) {
      if (err_msg != nullptr) *err_msg = "mount_path 未找到: " + mount_key;
      return false;
    }
    std::string trimmed = new_url;
    while (!trimmed.empty() && std::isspace(static_cast<unsigned char>(trimmed.front()))) trimmed.erase(trimmed.begin());
    while (!trimmed.empty() && std::isspace(static_cast<unsigned char>(trimmed.back()))) trimmed.pop_back();
    if (trimmed.empty()) {
      if (err_msg != nullptr) *err_msg = "new_rtsp_url 为空";
      return false;
    }

    RPushChannel * ch = channels_[*idx].get();
    auto prom = std::make_shared<std::promise<std::pair<bool, std::string>>>();
    std::future<std::pair<bool, std::string>> fut = prom->get_future();
    auto * job = new RelocatePack{ch, trimmed, prom, cfg_};
    g_main_context_invoke(main_ctx_, relocate_stream_idle_cb, job);
    const std::pair<bool, std::string> result = fut.get();
    if (!result.first && err_msg != nullptr) *err_msg = result.second;
    return result.first;
  }

private:
  void teardown()
  {
    if (shutting_down_.exchange(true)) return;
    for (auto & ch : channels_) {
      if (ch && ch->pipeline) {
        gst_element_set_state(ch->pipeline, GST_STATE_NULL);
        gst_object_unref(ch->pipeline);
        ch->pipeline = nullptr;
      }
      if (ch) {
        std::lock_guard<std::mutex> lk(ch->mu);
        g_weak_ref_clear(&ch->appsrc_weak);
        g_weak_ref_init(&ch->appsrc_weak, nullptr);
      }
    }
    channels_.clear();

    if (loop_) g_main_loop_quit(loop_);
    if (glib_thread_.joinable()) glib_thread_.join();
    if (loop_) {
      g_main_loop_unref(loop_);
      loop_ = nullptr;
    }
    if (main_ctx_) {
      g_main_context_unref(main_ctx_);
      main_ctx_ = nullptr;
    }
    started_ = false;
    shutting_down_.store(false);
  }

  RtspRemoteClientPushConfig cfg_;
  std::vector<std::unique_ptr<RPushChannel>> channels_;
  GMainContext * main_ctx_{nullptr};
  GMainLoop * loop_{nullptr};
  std::thread glib_thread_;
  std::atomic<bool> global_paused_{false};
  std::atomic<bool> shutting_down_{false};
  bool started_ = false;
};

bool rtsp_client_sink_available()
{
  static std::once_flag once;
  std::call_once(once, []() { gst_init(nullptr, nullptr); });
  return gst_element_factory_find("rtspclientsink") != nullptr;
}

bool spawn_gstreamer_rtsp_url_relay(
  const std::string & pull_url, const std::string & push_url, bool use_h265, int * out_pid,
  std::string * err_msg)
{
  if (out_pid == nullptr) return false;
  *out_pid = -1;
#if !defined(__linux__)
  if (err_msg != nullptr) *err_msg = "spawn_gstreamer_rtsp_url_relay 仅 Linux";
  return false;
#else
  static std::once_flag once;
  std::call_once(once, []() { gst_init(nullptr, nullptr); });
  if (!gst_element_factory_find("rtspclientsink")) {
    if (err_msg != nullptr) {
      *err_msg = "无 rtspclientsink，请安装 gstreamer1.0-plugins-bad";
    }
    return false;
  }
  const std::string depay = use_h265 ? "rtph265depay" : "rtph264depay";
  const std::string parse = use_h265 ? "h265parse" : "h264parse";
  // GS 1.20：rtspclientsink 为 request pad，不能与 rtph26xpay 由 gst-launch 静态链接；
  // 用 depay+parse 送 ES 给 clientsink（由其内部打包 RECORD）。
  std::vector<std::string> store = {
    "gst-launch-1.0",
    "-e",
    "rtspsrc",
    "location=" + pull_url,
    "latency=100",
    "protocols=tcp",
    "!",
    depay,
    "!",
    parse,
    "!",
    "rtspclientsink",
    "location=" + push_url,
    "protocols=tcp",
  };
  std::vector<char *> argv;
  argv.reserve(store.size() + 1);
  for (auto & s : store) {
    argv.push_back(s.data());
  }
  argv.push_back(nullptr);
  pid_t pid = -1;
  if (!spawn_child_pdeathsig("gst-launch-1.0", argv.data(), &pid, err_msg)) {
    return false;
  }
  // 等管线建链/连远端；过短会把立刻失败的 gst-launch 误判为成功（僵尸仍占 pid）
  std::this_thread::sleep_for(std::chrono::milliseconds(1200));
  if (::kill(pid, 0) != 0) {
    int status = 0;
    (void)::waitpid(pid, &status, WNOHANG);
    if (err_msg != nullptr) {
      *err_msg = "gst-launch 子进程已退出（pipeline/连接失败，请检查 pull/push URL 与 H264/H265 是否匹配）";
    }
    return false;
  }
  // 僵尸：已死但未 wait，仍可能 kill==0；再试 waitpid
  {
    int status = 0;
    const pid_t w = ::waitpid(pid, &status, WNOHANG);
    if (w == pid) {
      if (err_msg != nullptr) {
        *err_msg = "gst-launch 子进程已退出（pipeline/连接失败，请检查 pull/push URL 与 H264/H265 是否匹配）";
      }
      return false;
    }
  }
  *out_pid = static_cast<int>(pid);
  return true;
#endif
}

bool jetson_nvmm_h265_to_h264_available()
{
  static std::once_flag once;
  std::call_once(once, []() { gst_init(nullptr, nullptr); });
  return gst_has_factory("nvv4l2decoder") && gst_has_factory("nvvidconv") &&
         gst_has_factory("nvv4l2h264enc") && gst_has_factory("rtspclientsink") &&
         gst_has_factory("rtph265depay") && gst_has_factory("h265parse") &&
         gst_has_factory("h264parse");
}

bool spawn_gstreamer_jetson_h265_to_h264_relay(
  const std::string & pull_url, const std::string & push_url, int bitrate_kbps, int fps,
  int * out_pid, std::string * err_msg)
{
  if (out_pid == nullptr) return false;
  *out_pid = -1;
#if !defined(__linux__)
  if (err_msg != nullptr) *err_msg = "spawn_gstreamer_jetson_h265_to_h264_relay 仅 Linux";
  return false;
#else
  if (!jetson_nvmm_h265_to_h264_available()) {
    if (err_msg != nullptr) {
      *err_msg =
        "Jetson NVMM 转码插件不齐（需 nvv4l2decoder+nvvidconv+nvv4l2h264enc+rtspclientsink；"
        "请安装 nvidia-l4t-gstreamer 并配置 NVIDIA apt 源）";
    }
    return false;
  }
  if (pull_url.empty() || push_url.empty()) {
    if (err_msg != nullptr) *err_msg = "pull_url/push_url 不能为空";
    return false;
  }

  int kbps = bitrate_kbps;
  if (kbps < 200) kbps = 200;
  int gop = fps > 0 ? fps : 25;
  if (gop < 5) gop = 5;
  if (gop > 60) gop = 60;
  const std::string bitrate_bps = std::to_string(kbps * 1000);
  const std::string iframe = std::to_string(gop);

  // 低延迟 NVMM：不解 CPU BGR；queue leaky 丢旧帧；rtspsrc latency=0
  std::vector<std::string> store = {
    "gst-launch-1.0",
    "-e",
    "rtspsrc",
    "location=" + pull_url,
    "latency=0",
    "protocols=tcp",
    "!",
    "rtph265depay",
    "!",
    "h265parse",
    "!",
    "nvv4l2decoder",
    "!",
    "queue",
    "max-size-buffers=1",
    "max-size-time=0",
    "max-size-bytes=0",
    "leaky=downstream",
    "!",
    "nvvidconv",
    "!",
    "video/x-raw(memory:NVMM),format=NV12",
    "!",
    "nvv4l2h264enc",
    "bitrate=" + bitrate_bps,
    "iframeinterval=" + iframe,
    "insert-sps-pps=true",
    "!",
    "queue",
    "max-size-buffers=2",
    "max-size-time=0",
    "max-size-bytes=0",
    "!",
    "h264parse",
    "config-interval=-1",
    "!",
    "rtspclientsink",
    "location=" + push_url,
    "protocols=tcp",
  };

  std::vector<char *> argv;
  argv.reserve(store.size() + 1);
  for (auto & s : store) {
    argv.push_back(s.data());
  }
  argv.push_back(nullptr);

  pid_t pid = -1;
  if (!spawn_child_pdeathsig("gst-launch-1.0", argv.data(), &pid, err_msg)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  if (::kill(pid, 0) != 0) {
    int status = 0;
    (void)::waitpid(pid, &status, WNOHANG);
    if (err_msg != nullptr) {
      *err_msg =
        "gst-launch NVMM H265→H264 子进程已退出（检查相机 H.265、远端 ingest、以及 "
        "gst-inspect-1.0 nvv4l2h264enc）";
    }
    return false;
  }
  *out_pid = static_cast<int>(pid);
  std::fprintf(
    stderr,
    "[rtsp_nvmm] H265→H264 relay started pid=%d bitrate_kbps=%d iframeinterval=%d\n"
    "  pull=%s\n  push=%s\n",
    *out_pid, kbps, gop, pull_url.c_str(), push_url.c_str());
  return true;
#endif
}

RtspRemoteClientPush::RtspRemoteClientPush() : impl_(std::make_unique<Impl>()) {}
RtspRemoteClientPush::~RtspRemoteClientPush() = default;

void RtspRemoteClientPush::start(const RtspRemoteClientPushConfig & cfg) { impl_->start(cfg); }
bool RtspRemoteClientPush::append_stream(const RtspRemotePushEntry & ent, std::string * err_msg)
{
  return impl_->append_stream(ent, err_msg);
}
void RtspRemoteClientPush::stop() { impl_->stop(); }

bool RtspRemoteClientPush::push(std::size_t stream_idx, const cv::Mat & bgr)
{
  return impl_->push(stream_idx, bgr);
}

bool RtspRemoteClientPush::push(const std::string & mount_path, const cv::Mat & bgr)
{
  const auto idx = impl_->find_channel(mount_path);
  return idx.has_value() ? impl_->push(*idx, bgr) : false;
}

void RtspRemoteClientPush::set_global_paused(bool paused) { impl_->set_global_paused(paused); }

void RtspRemoteClientPush::set_stream_paused(const std::string & mount_path, bool paused)
{
  impl_->set_stream_paused(mount_path, paused);
}

bool RtspRemoteClientPush::relocate_stream_remote_url(
  const std::string & mount_path, const std::string & new_rtsp_url, std::string * err_msg)
{
  return impl_->relocate_stream_remote_url(mount_path, new_rtsp_url, err_msg);
}

std::string RtspRemoteClientPush::url(std::size_t stream_idx) const { return impl_->url(stream_idx); }

std::optional<std::size_t> RtspRemoteClientPush::find_channel(const std::string & key) const
{
  return impl_->find_channel(key);
}

std::uint64_t RtspRemoteClientPush::pushed_frames(std::size_t stream_idx) const
{
  return impl_->pushed_frames(stream_idx);
}

bool RtspRemoteClientPush::stream_push_healthy(const std::string & mount_path) const
{
  return impl_->stream_push_healthy(mount_path);
}

bool RtspRemoteClientPush::stream_push_session_exists(const std::string & mount_path) const
{
  return impl_->stream_push_session_exists(mount_path);
}

bool RtspRemoteClientPush::stream_push_paused(const std::string & mount_path) const
{
  return impl_->stream_push_paused(mount_path);
}

bool RtspRemoteClientPush::stop_stream_push(const std::string & mount_path)
{
  return impl_->stop_stream_push(mount_path);
}

}  // namespace m_common
