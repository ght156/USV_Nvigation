// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// m_common::rtsp_interface - GStreamer RTSP 客户端拉流接口。

#ifndef M_COMMON__RTSP_INTERFACE__RTSP_CLIENT_GST_HPP_
#define M_COMMON__RTSP_INTERFACE__RTSP_CLIENT_GST_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace m_common
{

struct RtspClientGstConfig
{
  /// RTSP 播放地址，例如 rtsp://127.0.0.1:8554/live/cam_live
  std::string url;

  /// rtspsrc latency（毫秒），越小实时性越高，抗抖动越差
  int latency_ms = 100;

  /// 每次 read() 的等待超时（毫秒）
  int read_timeout_ms = 1000;

  /// appsink 最大缓存帧数（配合 drop=true 可做最新帧策略）
  int appsink_max_buffers = 1;

  /// appsink 丢弃旧帧，仅保留新帧（建议低延迟场景保持 true）
  bool drop_old_frames = true;

  /// 拉流传输协议：true=tcp，false=udp
  bool prefer_tcp_transport = true;

  /// 读帧失败后的重连间隔（毫秒）
  int reconnect_interval_ms = 300;

  /// 单次 read() 最多重连次数；0 = 只尝试当前连接
  int max_reconnect_attempts = 3;
};

class RtspClientGst
{
public:
  RtspClientGst();
  ~RtspClientGst();

  RtspClientGst(const RtspClientGst &) = delete;
  RtspClientGst & operator=(const RtspClientGst &) = delete;
  RtspClientGst(RtspClientGst &&) = delete;
  RtspClientGst & operator=(RtspClientGst &&) = delete;

  /// 建立连接。失败抛 std::runtime_error。
  void open(const RtspClientGstConfig & cfg);

  /// 主动关闭连接。析构时自动调用。
  void close();

  /// 当前是否处于可读状态。
  bool is_open() const;

  /// 拉取一帧 BGR 图像；失败时按配置自动重连并重试。
  bool read(cv::Mat & frame);

  /// 最近一次成功读帧的时间戳（steady clock，ms）。
  std::uint64_t last_frame_time_ms() const;

  /// 当前实际使用 URL。
  std::string opened_url() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace m_common

#endif  // M_COMMON__RTSP_INTERFACE__RTSP_CLIENT_GST_HPP_
