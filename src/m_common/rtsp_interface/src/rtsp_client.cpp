// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT

#include "m_common/rtsp_interface/rtsp_client.hpp"

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <opencv2/videoio.hpp>

namespace m_common
{
namespace
{

std::string add_tcp_transport_if_needed(const std::string & url, bool prefer_tcp)
{
  if (!prefer_tcp || url.empty()) {
    return url;
  }
  if (url.find("rtsp_transport=") != std::string::npos) {
    return url;
  }
  const char delimiter = (url.find('?') == std::string::npos) ? '?' : '&';
  return url + delimiter + "rtsp_transport=tcp";
}

std::uint64_t now_ms()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

}  // namespace

class RtspClient::Impl
{
public:
  void open(const RtspClientConfig & cfg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (cfg.url.empty()) {
      throw std::runtime_error("RtspClient: cfg.url is empty");
    }
    cfg_ = cfg;
    opened_url_ = add_tcp_transport_if_needed(cfg_.url, cfg_.prefer_tcp_transport);
    open_locked_or_throw();
  }

  void close()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cap_.release();
  }

  bool is_open() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return cap_.isOpened();
  }

  bool read(cv::Mat & frame)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!cap_.isOpened() && !reopen_locked()) {
      return false;
    }
    if (cap_.read(frame)) {
      last_frame_time_ms_ = now_ms();
      return true;
    }
    const int attempts = cfg_.max_reconnect_attempts < 0 ? 0 : cfg_.max_reconnect_attempts;
    for (int i = 0; i < attempts; ++i) {
      if (!reopen_locked()) {
        continue;
      }
      if (cap_.read(frame)) {
        last_frame_time_ms_ = now_ms();
        return true;
      }
    }
    return false;
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
  bool open_locked()
  {
    cap_.release();
    const bool ok = cap_.open(opened_url_, cfg_.backend);
    if (!ok) {
      return false;
    }
    if (cfg_.buffer_size > 0) {
      cap_.set(cv::CAP_PROP_BUFFERSIZE, static_cast<double>(cfg_.buffer_size));
    }
    return true;
  }

  void open_locked_or_throw()
  {
    if (!open_locked()) {
      throw std::runtime_error("RtspClient: failed to open stream: " + opened_url_);
    }
  }

  bool reopen_locked()
  {
    cap_.release();
    if (cfg_.reconnect_interval_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.reconnect_interval_ms));
    }
    return open_locked();
  }

  mutable std::mutex mtx_;
  RtspClientConfig cfg_;
  cv::VideoCapture cap_;
  std::string opened_url_;
  std::uint64_t last_frame_time_ms_ = 0;
};

RtspClient::RtspClient() : impl_(std::make_unique<Impl>()) {}
RtspClient::~RtspClient() = default;

void RtspClient::open(const RtspClientConfig & cfg) { impl_->open(cfg); }
void RtspClient::close() { impl_->close(); }
bool RtspClient::is_open() const { return impl_->is_open(); }
bool RtspClient::read(cv::Mat & frame) { return impl_->read(frame); }
std::uint64_t RtspClient::last_frame_time_ms() const { return impl_->last_frame_time_ms(); }
std::string RtspClient::opened_url() const { return impl_->opened_url(); }

}  // namespace m_common
