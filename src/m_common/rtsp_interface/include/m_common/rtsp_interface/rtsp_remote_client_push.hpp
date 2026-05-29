// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// 使用 GStreamer rtspclientsink：编码后经 h264parse/h265parse 送入 sink，由其内部打包并完成远端 RECORD（ingest），
// 与本机 GstRTSPServer 被拉流模式互补。运行期需 gst-plugins-bad 提供 rtspclientsink。

#ifndef M_COMMON__RTSP_INTERFACE__RTSP_REMOTE_CLIENT_PUSH_HPP_
#define M_COMMON__RTSP_INTERFACE__RTSP_REMOTE_CLIENT_PUSH_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "m_common/rtsp_interface/rtsp_publisher.hpp"

namespace m_common
{

struct RtspRemotePushEntry
{
  RtspStreamSpec spec;
  /// 完整远端 URL，例如 rtsp://10.0.0.5:8554/live/cam0
  std::string remote_rtsp_url;
};

struct RtspRemoteClientPushConfig
{
  std::string h264_encoder = "auto";
  std::string h265_encoder = "auto";
  /// 与 RtspPublisher 一致：须同时为空或同时非空；非空时设置 rtspclientsink 的 user-id / user-pw
  std::string auth_username;
  std::string auth_password;
  std::vector<RtspRemotePushEntry> streams;
};

/// 多路 BGR8 -> 编码 -> rtspclientsink，每路独立 pipeline。
class RtspRemoteClientPush
{
public:
  RtspRemoteClientPush();
  ~RtspRemoteClientPush();

  RtspRemoteClientPush(const RtspRemoteClientPush &) = delete;
  RtspRemoteClientPush & operator=(const RtspRemoteClientPush &) = delete;
  RtspRemoteClientPush(RtspRemoteClientPush &&) = delete;
  RtspRemoteClientPush & operator=(RtspRemoteClientPush &&) = delete;

  void start(const RtspRemoteClientPushConfig & cfg);
  void stop();

  /// 在已成功 start() 后追加一路独立 pipeline（mount 须尚未存在）
  bool append_stream(const RtspRemotePushEntry & ent, std::string * err_msg = nullptr);

  bool push(std::size_t stream_idx, const cv::Mat & bgr);
  bool push(const std::string & mount_path, const cv::Mat & bgr);

  void set_global_paused(bool paused);
  void set_stream_paused(const std::string & mount_path, bool paused);

  /// 运行中将该路的 rtspclientsink 切到新 URL（先 NULL 再改 location 再 PLAYING）。仅适用于已由 start() 创建的通路。
  /// @param err_msg 失败时可写入原因
  bool relocate_stream_remote_url(
    const std::string & mount_path, const std::string & new_rtsp_url, std::string * err_msg = nullptr);

  /// 返回配置的远端 ingest URL
  std::string url(std::size_t stream_idx) const;

  std::optional<std::size_t> find_channel(const std::string & key) const;
  std::uint64_t pushed_frames(std::size_t stream_idx) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

/// 当前进程/插件是否能够使用 rtspclientsink（须已 gst_init 或本函数内部会 init）。
bool rtsp_client_sink_available();

/// 通过 gst-launch-1.0 启动一路 RTSP 拉流转推（不重新编码，仅 RTP 解封装/封装）。
/// @param use_h265 为 true 时使用 rtph265depay/pay，否则 H.264。
/// @return 子进程 PID，失败时 *out_pid=-1
bool spawn_gstreamer_rtsp_url_relay(
  const std::string & pull_url, const std::string & push_url, bool use_h265, int * out_pid,
  std::string * err_msg);

}  // namespace m_common

#endif  // M_COMMON__RTSP_INTERFACE__RTSP_REMOTE_CLIENT_PUSH_HPP_
