// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT

#include "m_common/rtsp_interface/rtsp_publisher.hpp"

#include <memory>
#include <stdexcept>
#include <string>

#include "backend_iface.hpp"

namespace m_common
{

namespace
{
std::unique_ptr<rtsp_internal::IBackend> create_backend(RtspBackend choice)
{
#if !defined(RTSP_IF_HAS_GSTREAMER)
  static_assert(false, "rtsp_interface 需要 GStreamer（-DRTSP_IF_ENABLE_GSTREAMER=ON）");
#endif
  (void)choice;
  return rtsp_internal::make_gstreamer_backend();
}
}  // namespace

class RtspPublisher::Impl
{
public:
  void start(const RtspPublisherConfig & cfg)
  {
    if (backend_) throw std::runtime_error("RtspPublisher: already started");
    if (cfg.backend != RtspBackend::kAuto && cfg.backend != RtspBackend::kGStreamer) {
      throw std::runtime_error("rtsp_interface: 仅支持 backend=auto|gstreamer");
    }
    backend_ = create_backend(cfg.backend);
    backend_->start(cfg);
  }
  void stop()
  {
    if (backend_) {
      backend_->stop();
      backend_.reset();
    }
  }
  bool push(std::size_t idx, const cv::Mat & bgr)
  {
    return backend_ ? backend_->push(idx, bgr) : false;
  }
  bool push(const std::string & key, const cv::Mat & bgr)
  {
    if (!backend_) return false;
    auto idx = backend_->find_channel(key);
    return idx.has_value() ? backend_->push(*idx, bgr) : false;
  }
  void set_global_paused(bool paused) { if (backend_) backend_->set_global_paused(paused); }
  void set_stream_paused(const std::string & key, bool paused)
  {
    if (backend_) backend_->set_stream_paused(key, paused);
  }
  std::string url(std::size_t idx) const { return backend_ ? backend_->url(idx) : std::string{}; }
  std::string url(const std::string & key) const
  {
    if (!backend_) return {};
    auto idx = backend_->find_channel(key);
    return idx.has_value() ? backend_->url(*idx) : std::string{};
  }
  std::uint64_t pushed_frames(std::size_t idx) const
  {
    return backend_ ? backend_->pushed_frames(idx) : 0;
  }
  std::string backend_name() const
  {
    return backend_ ? std::string(backend_->name()) : std::string{};
  }

private:
  std::unique_ptr<rtsp_internal::IBackend> backend_;
};

RtspPublisher::RtspPublisher() : impl_(std::make_unique<Impl>()) {}
RtspPublisher::~RtspPublisher() = default;

void RtspPublisher::start(const RtspPublisherConfig & cfg) { impl_->start(cfg); }
void RtspPublisher::stop() { impl_->stop(); }
bool RtspPublisher::push(std::size_t idx, const cv::Mat & bgr) { return impl_->push(idx, bgr); }
bool RtspPublisher::push(const std::string & m, const cv::Mat & bgr) { return impl_->push(m, bgr); }
void RtspPublisher::set_global_paused(bool p) { impl_->set_global_paused(p); }
void RtspPublisher::set_stream_paused(const std::string & m, bool p)
{
  impl_->set_stream_paused(m, p);
}
std::string RtspPublisher::url(std::size_t idx) const { return impl_->url(idx); }
std::string RtspPublisher::url(const std::string & m) const { return impl_->url(m); }
std::uint64_t RtspPublisher::pushed_frames(std::size_t idx) const
{
  return impl_->pushed_frames(idx);
}
std::string RtspPublisher::backend_name() const { return impl_->backend_name(); }

}  // namespace m_common
