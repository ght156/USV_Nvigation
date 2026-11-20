#ifndef WORKSPACE_NAV__NAVIGATION_LOGGING_HPP_
#define WORKSPACE_NAV__NAVIGATION_LOGGING_HPP_

#include <mlogger/mlogger.hpp>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace workspace_nav::navigation_logging
{

inline std::string log_root()
{
  const char * home = std::getenv("USV_HOME");
  return home && *home ? std::string(home) + "/logs" : "logs";
}

inline std::string format(const char * text)
{
  return text;
}

template<typename... Args>
std::string format(const char * format_string, Args... args)
{
  std::vector<char> buffer(2048);
  const int length = std::snprintf(buffer.data(), buffer.size(), format_string, args...);
  if (length < 0) return "log formatting error";
  if (static_cast<std::size_t>(length) >= buffer.size()) {
    buffer.resize(static_cast<std::size_t>(length) + 1);
    std::snprintf(buffer.data(), buffer.size(), format_string, args...);
  }
  return std::string(buffer.data(), static_cast<std::size_t>(length));
}

}  // namespace workspace_nav::navigation_logging

#define NAV_LOGGER_INIT(node_name) \
  MLOGGER_MODULE_INIT( \
    workspace_nav::navigation_logging::log_root(), "navigation", node_name)
#define NAV_LOG_INFO(...) \
  MLOGGER_INFO("{}", workspace_nav::navigation_logging::format(__VA_ARGS__))
#define NAV_LOG_WARN(...) \
  MLOGGER_WARN("{}", workspace_nav::navigation_logging::format(__VA_ARGS__))
#define NAV_LOG_ERROR(...) \
  MLOGGER_ERROR("{}", workspace_nav::navigation_logging::format(__VA_ARGS__))
#define NAV_LOG_WARN_THROTTLE(interval_ms, ...) \
  MLOGGER_THROTTLE_WARN( \
    interval_ms, "{}", workspace_nav::navigation_logging::format(__VA_ARGS__))
#define NAV_LOG_INFO_THROTTLE(interval_ms, ...) \
  do { \
    static std::atomic<int64_t> nav_log_last_info_ms{0}; \
    const auto nav_log_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>( \
      std::chrono::steady_clock::now().time_since_epoch()).count(); \
    auto nav_log_previous_ms = nav_log_last_info_ms.load(std::memory_order_relaxed); \
    if (nav_log_now_ms - nav_log_previous_ms >= static_cast<int64_t>(interval_ms) && \
      nav_log_last_info_ms.compare_exchange_strong( \
        nav_log_previous_ms, nav_log_now_ms, std::memory_order_relaxed)) \
    { \
      NAV_LOG_INFO(__VA_ARGS__); \
    } \
  } while (0)
#define NAV_LOG_ERROR_THROTTLE(interval_ms, ...) \
  do { \
    static std::atomic<int64_t> nav_log_last_error_ms{0}; \
    const auto nav_log_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>( \
      std::chrono::steady_clock::now().time_since_epoch()).count(); \
    auto nav_log_previous_ms = nav_log_last_error_ms.load(std::memory_order_relaxed); \
    if (nav_log_now_ms - nav_log_previous_ms >= static_cast<int64_t>(interval_ms) && \
      nav_log_last_error_ms.compare_exchange_strong( \
        nav_log_previous_ms, nav_log_now_ms, std::memory_order_relaxed)) \
    { \
      NAV_LOG_ERROR(__VA_ARGS__); \
    } \
  } while (0)

#endif  // WORKSPACE_NAV__NAVIGATION_LOGGING_HPP_
