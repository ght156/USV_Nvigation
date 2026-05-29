// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT

#include "m_common/rtsp_interface/rtsp_client_gst.hpp"

#include "rtsp_device_detect.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(RTSP_IF_HAS_GSTREAMER)
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/rtsp/gstrtsptransport.h>
#endif

namespace m_common
{
namespace
{

std::uint64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

#if defined(RTSP_IF_HAS_GSTREAMER)
bool gst_has_factory_name(const char * name)
{
  GstElementFactory * f = gst_element_factory_find(name);
  if (f == nullptr) {
    return false;
  }
  gst_object_unref(f);
  return true;
}

GstElement * make_first_available_decoder(
  const std::vector<std::string> & candidates, std::string & selected_name)
{
  for (const auto & name : candidates) {
    GstElement * ele = gst_element_factory_make(name.c_str(), "decoder");
    if (ele != nullptr) {
      selected_name = name;
      return ele;
    }
  }
  selected_name.clear();
  return nullptr;
}
#endif

}  // namespace

class RtspClientGst::Impl
{
public:
  void open(const RtspClientGstConfig & cfg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (cfg.url.empty()) {
      throw std::runtime_error("RtspClientGst: cfg.url is empty");
    }
    cfg_ = cfg;
    opened_url_ = cfg_.url;
    open_locked_or_throw();
  }

  void close()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    close_pipeline_locked();
  }

  bool is_open() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
#if defined(RTSP_IF_HAS_GSTREAMER)
    return pipeline_ != nullptr && sink_ != nullptr;
#else
    return false;
#endif
  }

  bool read(cv::Mat & frame)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!read_once_locked(frame)) {
      const int attempts = std::max(0, cfg_.max_reconnect_attempts);
      for (int i = 0; i < attempts; ++i) {
        if (!reopen_locked()) {
          continue;
        }
        if (read_once_locked(frame)) {
          return true;
        }
      }
      return false;
    }
    return true;
  }

  std::uint64_t last_frame_time_ms() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_frame_time_ms_;
  }

  std::string opened_url() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return opened_url_;
  }

private:
#if defined(RTSP_IF_HAS_GSTREAMER)
  static void on_rtspsrc_pad_added(GstElement * src, GstPad * new_pad, gpointer user_data)
  {
    (void)src;
    auto * self = static_cast<Impl *>(user_data);
    if (self == nullptr) return;
    self->link_dynamic_branch(new_pad);
  }

  void link_dynamic_branch(GstPad * new_pad)
  {
    if (new_pad == nullptr || pipeline_ == nullptr || queue_ == nullptr) return;
    if (branch_linked_) return;

    GstCaps * caps = gst_pad_get_current_caps(new_pad);
    if (caps == nullptr) {
      caps = gst_pad_query_caps(new_pad, nullptr);
    }
    if (caps == nullptr) return;

    std::string encoding_name;
    GstStructure * st = gst_caps_get_structure(caps, 0);
    if (st != nullptr) {
      const char * enc = gst_structure_get_string(st, "encoding-name");
      if (enc != nullptr) encoding_name = enc;
    }
    gst_caps_unref(caps);

    for (char & c : encoding_name) {
      c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    if (encoding_name.empty()) {
      last_link_error_ = "RtspClientGst: cannot determine encoding-name from RTSP caps";
      return;
    }

    std::string depay_name;
    std::string parse_name;
    std::vector<std::string> decoder_candidates;
    if (encoding_name == "H264") {
      depay_name = "rtph264depay";
      parse_name = "h264parse";
      if (m_common::rtsp_device_is_jetson()) {
        decoder_candidates = {
          "nvv4l2decoder", "vah264dec", "vaapih264dec", "v4l2h264dec", "openh264dec", "avdec_h264"};
      } else {
        if (gst_has_factory_name("openh264dec")) {
          decoder_candidates.push_back("openh264dec");
        }
        decoder_candidates.push_back("avdec_h264");
        decoder_candidates.push_back("vah264dec");
        decoder_candidates.push_back("vaapih264dec");
        decoder_candidates.push_back("v4l2h264dec");
      }
    } else if (encoding_name == "H265" || encoding_name == "HEVC") {
      depay_name = "rtph265depay";
      parse_name = "h265parse";
      if (m_common::rtsp_device_is_jetson()) {
        decoder_candidates = {
          "nvv4l2decoder", "vah265dec", "vaapih265dec", "v4l2h265dec", "avdec_h265"};
      } else {
        decoder_candidates = {"vah265dec", "vaapih265dec", "v4l2h265dec", "avdec_h265"};
      }
    } else if (encoding_name == "JPEG" || encoding_name == "MJPEG") {
      depay_name = "rtpjpegdepay";
      decoder_candidates = {"nvjpegdec", "vaapijpegdec", "v4l2jpegdec", "jpegdec"};
    } else {
      last_link_error_ = "RtspClientGst: unsupported encoding-name: " + encoding_name;
      return;
    }

    depay_ = gst_element_factory_make(depay_name.c_str(), "depay");
    decoder_ = make_first_available_decoder(decoder_candidates, active_decoder_);
    parse_ = parse_name.empty() ? nullptr : gst_element_factory_make(parse_name.c_str(), "parser");
    convert_ = gst_element_factory_make("videoconvert", "convert");
    capsfilter_ = gst_element_factory_make("capsfilter", "bgr_caps");
    if (depay_ == nullptr || decoder_ == nullptr || convert_ == nullptr || capsfilter_ == nullptr ||
        (!parse_name.empty() && parse_ == nullptr)) {
      last_link_error_ = "RtspClientGst: failed to create depay/decoder branch for " + encoding_name;
      return;
    }

    GstCaps * bgr_caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", nullptr);
    g_object_set(G_OBJECT(capsfilter_), "caps", bgr_caps, nullptr);
    gst_caps_unref(bgr_caps);

    if (parse_ != nullptr) {
      gst_bin_add_many(
        GST_BIN(pipeline_), depay_, parse_, decoder_, convert_, capsfilter_, sink_, nullptr);
      if (!gst_element_link_many(queue_, depay_, parse_, decoder_, convert_, capsfilter_, sink_, nullptr)) {
        last_link_error_ = "RtspClientGst: failed to link branch elements with parser";
        return;
      }
    } else {
      gst_bin_add_many(
        GST_BIN(pipeline_), depay_, decoder_, convert_, capsfilter_, sink_, nullptr);
      if (!gst_element_link_many(queue_, depay_, decoder_, convert_, capsfilter_, sink_, nullptr)) {
        last_link_error_ = "RtspClientGst: failed to link branch elements";
        return;
      }
    }

    GstPad * sink_pad = gst_element_get_static_pad(queue_, "sink");
    if (sink_pad == nullptr) {
      last_link_error_ = "RtspClientGst: queue sink pad not found";
      return;
    }
    const GstPadLinkReturn lret = gst_pad_link(new_pad, sink_pad);
    gst_object_unref(sink_pad);
    if (lret != GST_PAD_LINK_OK) {
      last_link_error_ = "RtspClientGst: failed to link rtspsrc dynamic pad";
      return;
    }

    gst_element_sync_state_with_parent(queue_);
    gst_element_sync_state_with_parent(depay_);
    if (parse_ != nullptr) gst_element_sync_state_with_parent(parse_);
    gst_element_sync_state_with_parent(decoder_);
    gst_element_sync_state_with_parent(convert_);
    gst_element_sync_state_with_parent(capsfilter_);
    gst_element_sync_state_with_parent(sink_);

    branch_linked_ = true;
    active_encoding_ = encoding_name;
    g_print(
      "RtspClientGst: encoding=%s, decoder=%s, url=%s\n",
      active_encoding_.c_str(),
      active_decoder_.empty() ? "unknown" : active_decoder_.c_str(),
      opened_url_.c_str());
  }
#endif

  void open_locked_or_throw()
  {
    if (!open_locked()) {
      throw std::runtime_error("RtspClientGst: failed to open stream: " + opened_url_);
    }
  }

  bool reopen_locked()
  {
    close_pipeline_locked();
    if (cfg_.reconnect_interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnect_interval_ms));
    }
    return open_locked();
  }

  bool read_once_locked(cv::Mat & frame)
  {
#if !defined(RTSP_IF_HAS_GSTREAMER)
    (void)frame;
    return false;
#else
    if (!last_link_error_.empty()) {
      throw std::runtime_error(last_link_error_);
    }
    if (sink_ == nullptr) {
      return false;
    }
    GstSample * sample = gst_app_sink_try_pull_sample(
      GST_APP_SINK(sink_), static_cast<GstClockTime>(std::max(1, cfg_.read_timeout_ms)) * GST_MSECOND);
    if (sample == nullptr) {
      return false;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstCaps * caps = gst_sample_get_caps(sample);
    if (buffer == nullptr || caps == nullptr) {
      gst_sample_unref(sample);
      return false;
    }

    GstStructure * s = gst_caps_get_structure(caps, 0);
    int width = 0;
    int height = 0;
    if (!gst_structure_get_int(s, "width", &width) ||
        !gst_structure_get_int(s, "height", &height) ||
        width <= 0 || height <= 0) {
      gst_sample_unref(sample);
      return false;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return false;
    }

    const std::size_t min_bytes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3U;
    bool ok = false;
    if (map.size >= min_bytes) {
      cv::Mat wrapped(height, width, CV_8UC3, map.data);
      frame = wrapped.clone();
      last_frame_time_ms_ = now_ms();
      ok = true;
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return ok;
#endif
  }

  bool open_locked()
  {
#if !defined(RTSP_IF_HAS_GSTREAMER)
    return false;
#else
    static std::once_flag gst_init_once;
    static bool gst_init_ok = false;
    std::call_once(gst_init_once, []() {
      GError * err = nullptr;
      gst_init_ok = gst_init_check(nullptr, nullptr, &err);
      if (!gst_init_ok && err != nullptr) {
        g_error_free(err);
      }
    });
    if (!gst_init_ok) {
      throw std::runtime_error("RtspClientGst: gstreamer init failed");
    }

    close_pipeline_locked();

    branch_linked_ = false;
    active_encoding_.clear();
    active_decoder_.clear();
    last_link_error_.clear();
    depay_ = nullptr;
    parse_ = nullptr;
    decoder_ = nullptr;
    convert_ = nullptr;
    capsfilter_ = nullptr;

    pipeline_ = gst_pipeline_new("rtsp_client_gst_pipeline");
    rtspsrc_ = gst_element_factory_make("rtspsrc", "src");
    queue_ = gst_element_factory_make("queue", "q0");
    sink_ = gst_element_factory_make("appsink", "sink");
    if (pipeline_ == nullptr || rtspsrc_ == nullptr || queue_ == nullptr || sink_ == nullptr) {
      close_pipeline_locked();
      throw std::runtime_error("RtspClientGst: failed to create base gstreamer elements");
    }

    g_object_set(
      G_OBJECT(rtspsrc_),
      "location", cfg_.url.c_str(),
      "latency", std::max(0, cfg_.latency_ms),
      "protocols", cfg_.prefer_tcp_transport ? static_cast<guint>(GST_RTSP_LOWER_TRANS_TCP)
                                             : static_cast<guint>(GST_RTSP_LOWER_TRANS_UDP),
      nullptr);
    g_object_set(
      G_OBJECT(queue_),
      "max-size-buffers", std::max(1, cfg_.appsink_max_buffers),
      "max-size-bytes", 0U,
      "max-size-time", 0ULL,
      "leaky", 2,  // downstream: 丢旧帧，保新帧
      nullptr);
    g_object_set(
      G_OBJECT(sink_),
      "emit-signals", FALSE,
      "sync", FALSE,
      "max-buffers", std::max(1, cfg_.appsink_max_buffers),
      "drop", cfg_.drop_old_frames ? TRUE : FALSE,
      nullptr);

    gst_bin_add_many(GST_BIN(pipeline_), rtspsrc_, queue_, nullptr);
    g_signal_connect(rtspsrc_, "pad-added", G_CALLBACK(on_rtspsrc_pad_added), this);

    const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      close_pipeline_locked();
      throw std::runtime_error("RtspClientGst: failed to set pipeline PLAYING");
    }

    return true;
#endif
  }

  void close_pipeline_locked()
  {
#if defined(RTSP_IF_HAS_GSTREAMER)
    branch_linked_ = false;
    active_encoding_.clear();
    active_decoder_.clear();
    last_link_error_.clear();
    depay_ = nullptr;
    parse_ = nullptr;
    decoder_ = nullptr;
    convert_ = nullptr;
    capsfilter_ = nullptr;
    sink_ = nullptr;
    queue_ = nullptr;
    rtspsrc_ = nullptr;
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
#endif
  }

  mutable std::mutex mtx_;
  RtspClientGstConfig cfg_;
  std::string opened_url_;
  std::uint64_t last_frame_time_ms_ = 0;

#if defined(RTSP_IF_HAS_GSTREAMER)
  GstElement * pipeline_ = nullptr;
  GstElement * rtspsrc_ = nullptr;
  GstElement * queue_ = nullptr;
  GstElement * depay_ = nullptr;
  GstElement * parse_ = nullptr;
  GstElement * decoder_ = nullptr;
  GstElement * convert_ = nullptr;
  GstElement * capsfilter_ = nullptr;
  GstElement * sink_ = nullptr;
  bool branch_linked_ = false;
  std::string active_encoding_;
  std::string active_decoder_;
  std::string last_link_error_;
#endif
};

RtspClientGst::RtspClientGst() : impl_(std::make_unique<Impl>()) {}
RtspClientGst::~RtspClientGst() = default;

void RtspClientGst::open(const RtspClientGstConfig & cfg)
{
#if !defined(RTSP_IF_HAS_GSTREAMER)
  (void)cfg;
  throw std::runtime_error("RtspClientGst: m_common was built without GStreamer support");
#else
  impl_->open(cfg);
#endif
}

void RtspClientGst::close() { impl_->close(); }
bool RtspClientGst::is_open() const { return impl_->is_open(); }
bool RtspClientGst::read(cv::Mat & frame) { return impl_->read(frame); }
std::uint64_t RtspClientGst::last_frame_time_ms() const { return impl_->last_frame_time_ms(); }
std::string RtspClientGst::opened_url() const { return impl_->opened_url(); }

}  // namespace m_common
