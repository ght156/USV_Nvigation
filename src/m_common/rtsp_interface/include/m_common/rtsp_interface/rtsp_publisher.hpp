// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// m_common::rtsp_interface - 简化的 RTSP 推流接口，供任何 ROS2 包 / 纯 C++ 程序快速调用。
//
// 特点：
//   - 三步用法：start(cfg) -> push(idx, cv::Mat) -> 析构自动 stop
//   - 多路并发：每路独立处理，互不阻塞
//   - 支持暂停（全局 / 单路），恢复时 H.264 会自动发 IDR 让客户端立即可解
//   - 仅 GStreamer 后端（gst-rtsp-server + H.264/H.265，含 Jetson nvv4l2* 与 x264/x265 等）
//
// 构建依赖：GStreamer（gstreamer-1.0 / app / rtsp-server）、OpenCV。
// 运行期：gstreamer1.0-plugins-base/-good/-ugly 等。

#ifndef M_COMMON__RTSP_INTERFACE__RTSP_PUBLISHER_HPP_
#define M_COMMON__RTSP_INTERFACE__RTSP_PUBLISHER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace m_common
{

enum class RtspCodec : int
{
  kH264 = 0,
  kMjpeg = 1,
  kH265 = 2,
};

/// 后端选择（仅 GStreamer；kAuto 与 kGStreamer 等价）。
enum class RtspBackend : int
{
  kAuto = 0,
  kGStreamer = 1,
};

struct RtspStreamSpec
{
  /// mount_path：RTSP URL 第二/第三段。
  ///   "cam_live"      -> rtsp://host:port/live/cam_live    (默认 app="live")
  ///   "live/cam_live" -> rtsp://host:port/live/cam_live    (显式写法，等价)
  ///   "myapp/foo"     -> rtsp://host:port/myapp/foo
  std::string mount_path = "cam";

  /// 视频帧率（推流端锁定，用于 encoder GOP 计算）
  int fps = 25;
  /// 比特率（kbps），H.264/H.265 有效
  int bitrate_kbps = 4000;
  /// 编码格式：kH264 / kH265；kMjpeg 当前后端不支持（start 将失败）
  RtspCodec codec = RtspCodec::kH264;
};

struct RtspPublisherConfig
{
  uint16_t port = 8554;
  std::string bind_address = "0.0.0.0";

  // 同时设置 username/password 才启用认证；任一为空 = 不认证。
  std::string auth_username{};
  std::string auth_password{};
  std::string auth_realm = "rtsp";

  RtspBackend backend = RtspBackend::kAuto;

  /// H.264：auto=Jetson 且存在 nvv4l2h264enc 时用硬件；否则优先 openh264enc，再无则用 x264enc。
  /// h264_encoder 可写逻辑名（x264、hw）或工厂名（x264enc、nvv4l2h264enc）。
  std::string h264_encoder = "auto";
  /// H.265：auto=Jetson 且存在 nvv4l2h265enc 时用硬件，否则 x265enc
  std::string h265_encoder = "auto";
  /// GstRTSPMediaFactory 同步缓冲（毫秒），默认 0
  int pipeline_latency_ms = 0;

  std::vector<RtspStreamSpec> streams;
};

/// 多路 RTSP 推流发布器。
class RtspPublisher
{
public:
  RtspPublisher();
  ~RtspPublisher();

  RtspPublisher(const RtspPublisher &) = delete;
  RtspPublisher & operator=(const RtspPublisher &) = delete;
  RtspPublisher(RtspPublisher &&) = delete;
  RtspPublisher & operator=(RtspPublisher &&) = delete;

  void start(const RtspPublisherConfig & cfg);
  void stop();

  bool push(std::size_t stream_idx, const cv::Mat & bgr);
  bool push(const std::string & mount_path, const cv::Mat & bgr);

  void set_global_paused(bool paused);
  void set_stream_paused(const std::string & mount_path, bool paused);

  std::string url(std::size_t stream_idx) const;
  std::string url(const std::string & mount_path) const;

  std::uint64_t pushed_frames(std::size_t stream_idx) const;

  /// 实际选用的后端名；未 start 返回空串
  std::string backend_name() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace m_common

#endif  // M_COMMON__RTSP_INTERFACE__RTSP_PUBLISHER_HPP_
