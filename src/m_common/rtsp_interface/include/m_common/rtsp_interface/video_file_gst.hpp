// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// GStreamer 本地视频文件解码为 BGR（appsink），可与 RtspClientGst 同库使用，
// Jetson 上默认 nvv4l2decoder + nvvidconv，其它平台 H.264 优先 openh264dec。

#ifndef M_COMMON__RTSP_INTERFACE__VIDEO_FILE_GST_HPP_
#define M_COMMON__RTSP_INTERFACE__VIDEO_FILE_GST_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace m_common
{

struct VideoFileGstConfig
{
  /// 本地容器路径（.mp4/.mov/.m4v/.3gp → qtdemux；.mkv/.webm → matroskademux）
  std::string file_path;

  /// 末尾读完是否 seek 到头循环（对齐 input.file_loop）
  bool loop = true;

  /// 每次 read() 拉取超时（毫秒）
  int read_timeout_ms = 1000;

  /// 调试：EOS/seek/READY 重绕/整管重开等打点（可由 rtsp2 input.file_gst_diag_log 打开）
  bool diag_log = false;
  /// 诊断日志前缀（如 camera_id）；空则无前缀。
  std::string diag_tag;

  int appsink_max_buffers = 1;
  bool drop_old_frames = true;
};

/// GStreamer 本地文件解码（qtdemux/mat）：多轨容器时在「不再新增 pad」后选取宽高最大的视频轨，其余轨排空。
/// fMP4（moov+mvex/moof）会先 ffmpeg copy+faststart 重封装到 ~/.cache/usv/video_file_gst/ 再交给 qtdemux（规避 GS 1.20 qtdemux 只解首段问题）。
/// 轻量用法：open(cfg) → 循环 read(mat) → close / 析构自动停止。
/// 仅在编译启用 GStreamer 时可用（与 RtspPublisher / RtspClientGst 同源）。
/// 线程安全约定与 RtspClientGst 一致：单线程 read 或外部串行化。
class VideoFileGst
{
public:
  VideoFileGst();
  ~VideoFileGst();

  VideoFileGst(const VideoFileGst &) = delete;
  VideoFileGst & operator=(const VideoFileGst &) = delete;
  VideoFileGst(VideoFileGst &&) = delete;
  VideoFileGst & operator=(VideoFileGst &&) = delete;

  void open(const VideoFileGstConfig & cfg);
  void close();
  bool is_open() const;
  bool read(cv::Mat & frame);
  std::uint64_t last_frame_time_ms() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace m_common

#endif  // M_COMMON__RTSP_INTERFACE__VIDEO_FILE_GST_HPP_
