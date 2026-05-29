#pragma once

namespace m_common {
namespace result_code {

constexpr int kSuccess       = 0;    // 成功
constexpr int kInvalidParam  = 1001; // 参数错误
constexpr int kInvalidUrl    = 1002; // URL 非法
constexpr int kConnectFailed = 1003; // 连接或启动失败
constexpr int kUnknown       = 1999; // 未知错误

} // namespace result_code
} // namespace m_common
