#include "m_utils/log_demo.hpp"

// 同包子库相互依赖：
//   m_utils 在编译期通过 build-time 的 INTERFACE 桥目标
//   m_common_mlogger_iface 拿到 mlogger 头与 spdlog 链接。
//   下游消费 m_common::m_utils 时不需要显式 include "mlogger/..."。
#include "mlogger/mlogger.hpp"

namespace m_utils {

void log_demo(const std::string &msg)
{
  MLOGGER_INFO("[m_utils::log_demo] {}", msg);
}

}  // namespace m_utils
