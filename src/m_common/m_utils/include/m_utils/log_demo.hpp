#pragma once

#include <string>

namespace m_utils {

// 演示：m_utils 内部使用同包子库 m_common::mlogger 打日志。
// 注意：调用前需要先在你的进程入口处初始化 mlogger，例如：
//   MLOGGER_MODULE_INIT("/tmp", "demo", "smoke");
// 否则 spdlog 后端尚未创建，会触发空指针调用。
void log_demo(const std::string &msg);

}  // namespace m_utils
