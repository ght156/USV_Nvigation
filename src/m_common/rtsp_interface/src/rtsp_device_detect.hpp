// Copyright (c) 2026 jzw
// SPDX-License-Identifier: MIT
//
// 探测是否为 NVIDIA Jetson/Tegra 设备（供 GStreamer 编码器 auto 与拉流解码器排序共用）。

#ifndef M_COMMON__RTSP_INTERFACE__RTSP_DEVICE_DETECT_HPP_
#define M_COMMON__RTSP_INTERFACE__RTSP_DEVICE_DETECT_HPP_

#include <cctype>
#include <fstream>
#include <string>

namespace m_common
{

/// 用于 rtsp_interface：device-tree model 关键字 + /etc/nv_tegra_release 兜底。
inline bool rtsp_device_is_jetson()
{
  std::ifstream model("/proc/device-tree/model", std::ios::binary);
  if (model) {
    std::string m;
    char ch{};
    while (model.get(ch) && m.size() < 256) {
      if (ch == '\0') {
        break;
      }
      m.push_back(ch);
    }
    for (char & c : m) {
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (m.find("jetson") != std::string::npos) {
      return true;
    }
    if (m.find("nvidia") != std::string::npos && m.find("orin") != std::string::npos) {
      return true;
    }
    if (m.find("xavier") != std::string::npos || m.find("tegra") != std::string::npos) {
      return true;
    }
  }

  std::ifstream tegra("/etc/nv_tegra_release");
  return tegra.good();
}

}  // namespace m_common

#endif  // M_COMMON__RTSP_INTERFACE__RTSP_DEVICE_DETECT_HPP_
