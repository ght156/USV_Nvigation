// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// m_common::rtsp_interface 的内部 backend 抽象。RtspPublisher 通过 PIMPL 持有
// 一个 IBackend 实例，实际后端在 start() 时根据 cfg.backend + 编译期开关确定。
//
// 这是私有头，仅供 rtsp_interface 内部 .cpp 互相 include；不安装、不公开。

#ifndef M_COMMON__RTSP_INTERFACE__BACKEND_IFACE_HPP_
#define M_COMMON__RTSP_INTERFACE__BACKEND_IFACE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "m_common/rtsp_interface/rtsp_publisher.hpp"

namespace m_common
{
namespace rtsp_internal
{

class IBackend
{
public:
  virtual ~IBackend() = default;

  /// 启动后端：创建 RTSP server、为每路准备 channel/encoder/media。失败抛 std::runtime_error。
  virtual void start(const RtspPublisherConfig & cfg) = 0;
  /// 停止：joint workers、release server。析构时也调。
  virtual void stop() = 0;

  /// 推送 BGR8 帧；非阻塞，leaky 队列。
  virtual bool push(std::size_t stream_idx, const cv::Mat & bgr) = 0;

  virtual void set_global_paused(bool paused) = 0;
  virtual void set_stream_paused(const std::string & mount, bool paused) = 0;

  virtual std::string url(std::size_t stream_idx) const = 0;
  virtual std::optional<std::size_t> find_channel(const std::string & key) const = 0;

  virtual std::uint64_t pushed_frames(std::size_t stream_idx) const = 0;

  virtual const char * name() const = 0;
};

#if defined(RTSP_IF_HAS_GSTREAMER)
std::unique_ptr<IBackend> make_gstreamer_backend();
#endif

}  // namespace rtsp_internal
}  // namespace m_common

#endif  // M_COMMON__RTSP_INTERFACE__BACKEND_IFACE_HPP_
