// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// m_common::rtsp_interface - RTSP 客户端拉流接口。
// 设计目标：
//   - 统一 OpenCV 拉流入口，支持 RTSP URL 直连
//   - 提供可控重连策略，降低短时断流影响
//   - 保持和 RtspPublisher 同样的轻量 API 风格

#ifndef M_COMMON__RTSP_INTERFACE__RTSP_CLIENT_HPP_
#define M_COMMON__RTSP_INTERFACE__RTSP_CLIENT_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace m_common
{

struct RtspClientConfig
{
  /// RTSP 播放地址，例如 rtsp://127.0.0.1:8554/live/cam_live
  std::string url;

  /// OpenCV VideoCapture backend（默认 FFmpeg）
  int backend = 1900;  // cv::CAP_FFMPEG，避免头文件版本差异导致编译失败

  /// VideoCapture 内部缓冲帧数，越小实时性越好
  int buffer_size = 1;

  /// 读帧失败后的重连间隔（毫秒）
  int reconnect_interval_ms = 300;

  /// 单次 read() 最多重连次数；0 = 只尝试当前连接
  int max_reconnect_attempts = 3;

  /// 当 URL 未显式指定 rtsp_transport 时，自动追加 tcp 参数
  bool prefer_tcp_transport = true;
};

class RtspClient
{
public:
  RtspClient();
  ~RtspClient();

  RtspClient(const RtspClient &) = delete;
  RtspClient & operator=(const RtspClient &) = delete;
  RtspClient(RtspClient &&) = delete;
  RtspClient & operator=(RtspClient &&) = delete;

  /// 建立连接。失败抛 std::runtime_error。
  void open(const RtspClientConfig & cfg);

  /// 主动关闭连接。析构时自动调用。
  void close();

  /// 当前是否处于可读状态。
  bool is_open() const;

  /// 拉取一帧 BGR 图像；失败时按配置自动重连并重试。
  bool read(cv::Mat & frame);

  /// 最近一次成功读帧的时间戳（steady clock，ms）。
  std::uint64_t last_frame_time_ms() const;

  /// 当前实际使用 URL（可能包含追加参数）。
  std::string opened_url() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace m_common

#endif  // M_COMMON__RTSP_INTERFACE__RTSP_CLIENT_HPP_
