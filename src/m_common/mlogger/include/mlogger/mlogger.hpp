/* Modified from logger.hpp to mlogger.hpp - glog removed, only spdlog remains */
/* Fix: include <unistd.h> for getpid(), and use std_fs::is_regular_file(entry.path()) for
 * experimental fs */
/* Added: InitWithModule() and MLOGGER_MODULE_INIT macro */
/* Added: InitWithModule() with also_log_to_stderr parameter and MLOGGER_MODULE_INIT_EX macro */

#pragma once

#if __cplusplus >= 201703L
#include <filesystem>
namespace std_fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace std_fs = std::experimental::filesystem;
#endif

#include <unistd.h> // for getpid()

#include <mutex>

#include <fmt/format.h>
#include <fmt/ranges.h>
namespace fmt {
template <typename T, typename Char>
struct formatter<T, Char, typename std::enable_if<std::is_enum<T>::value>::type>
    : formatter<typename std::underlying_type<T>::type, Char> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx)
  {
    return formatter<typename std::underlying_type<T>::type, Char>::parse(ctx);
  }

  template <typename FormatContext>
  auto format(const T &value, FormatContext &ctx)
  {
    return formatter<typename std::underlying_type<T>::type, Char>::format(
        static_cast<typename std::underlying_type<T>::type>(value), ctx);
  }

  template <typename FormatContext>
  auto format(const T &value, FormatContext &ctx) const
  {
    return formatter<typename std::underlying_type<T>::type, Char>::format(
        static_cast<typename std::underlying_type<T>::type>(value), ctx);
  }
};
} // namespace fmt

#include <spdlog/async.h>
#include <spdlog/sinks/daily_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#define SPDLOG_USE_ROLLING_LOG

namespace mlogger {

#define logger_likely(x) (__builtin_expect((x), 1))
#define logger_unlikely(x) (__builtin_expect((x), 0))

constexpr int keep_days = 1;

class Logger;

namespace detail {
template <typename... Args>
void LogInfo(const char *file, int line, const char *fmt, Args &&...args);
template <typename... Args>
void LogWarning(const char *file, int line, const char *fmt, Args &&...args);
template <typename... Args>
void LogError(const char *file, int line, const char *fmt, Args &&...args);
template <typename... Args>
void LogFatal(const char *file, int line, const char *fmt, Args &&...args);
} // namespace detail

class Logger {
public:
  static Logger &Instance()
  {
    static Logger instance;
    return instance;
  }

  // 简化初始化：不再需要 backend 参数，固定使用 spdlog
  void Init(const std::string &log_dir)
  {
    auto env                = std::getenv("ALSO_LOG_TO_STDERR");
    bool also_log_to_stderr = (env == nullptr || std::string(env) != "false");
    Init(log_dir, also_log_to_stderr);
  }

  void Init(const std::string &log_dir, bool also_log_to_stderr)
  {
    std::lock_guard<std::mutex> lk(mu_);
    log_name_ = "app"; // 默认名称，可由后续注册覆盖
    log_dir_  = log_dir;
    InitSpdlog(also_log_to_stderr);
  }

  // 兼容旧接口：从 argv[0] 提取程序名，并拼接默认日志路径
  void Init(int argc, char *argv[], bool also_log_to_stderr)
  {
    (void)argc;
    std::lock_guard<std::mutex> lk(mu_);
    log_name_ = std_fs::path(argv[0]).filename().string();
    log_dir_  = std::string(std::getenv("HOME")) + "/.cyber/log/uqilog/" + log_name_;
    InitSpdlog(also_log_to_stderr);
  }

  // 组件模式初始化（使用 HOME 路径）
  void Init(const std::string &module, const std::string &component)
  {
    std::lock_guard<std::mutex> lk(mu_);
    log_name_ = component;
    log_dir_  = std::string(std::getenv("HOME")) + "/.cyber/log/uqilog/" + module + "/";
    auto env  = std::getenv("ALSO_LOG_TO_STDERR");
    bool also_log_to_stderr = (env == nullptr || std::string(env) != "false");
    if (spdlog_loggers_.count(component) == 0)
    {
      InitComponentLogger(module, component, also_log_to_stderr);
    }
  }

  // 新增：支持传入基础目录和模块名的组件模式初始化（从环境变量读取控制台标志）
  void InitWithModule(const std::string &base_dir,
                      const std::string &module,
                      const std::string &component)
  {
    auto env                = std::getenv("ALSO_LOG_TO_STDERR");
    bool also_log_to_stderr = (env == nullptr || std::string(env) != "false");
    InitWithModule(base_dir, module, component, also_log_to_stderr);
  }

  // 新增：支持传入基础目录、模块名和自定义控制台标志的组件模式初始化
  void InitWithModule(const std::string &base_dir,
                      const std::string &module,
                      const std::string &component,
                      bool               also_log_to_stderr)
  {
    std::lock_guard<std::mutex> lk(mu_);
    log_name_ = component;
    log_dir_  = base_dir + "/" + module + "/";
    if (spdlog_loggers_.count(component) == 0)
    {
      InitComponentLogger(module, component, also_log_to_stderr);
    }
  }

  void Register(const std::string &base_dir, const std::string &module, const std::string &component)
  {
    InitWithModule(base_dir, module, component);
  }

protected:
  // 兜底：若用户未显式 Init（composable 常见），首次日志调用时自动初始化一个 stderr logger，
  // 保证 MLOGGER_* 不会因空指针/未就绪 async 队列而崩溃或“假死”。
  // 注意：该兜底仅 stderr，不写文件；需要落盘请在进程入口或各库首次使用前调用 Init/InitWithModule
  //（与 rtsp2_multi_source_node_main 中 MLOGGER_MODULE_INIT 一致），或自行封装统一初始化。
  void EnsureInitialized()
  {
    std::call_once(fallback_init_flag_, [this]() {
      // 不创建文件 sink，避免依赖目录权限；也避免 init_thread_pool 带来的全局副作用。
      auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
      console_sink->set_color(spdlog::level::info, "\033[1;32m");
      console_sink->set_color(spdlog::level::warn, "\033[1;33m");
      console_sink->set_color(spdlog::level::err, "\033[1;31m");
      console_sink->set_level(spdlog::level::info);

      auto logger = std::make_shared<spdlog::logger>(
          std::string("mlogger_fallback.") + std::to_string(getpid()),
          spdlog::sinks_init_list{console_sink});
      logger->set_pattern("[%Y-%m-%d %T.%f][%t][%^%l%$][%s:%#] %v");
      logger->set_level(spdlog::level::info);
      logger->flush_on(spdlog::level::err);

      std::lock_guard<std::mutex> lk(mu_);
      if (!spdlog_logger_) {
        spdlog_logger_ = logger;
        try {
          spdlog::register_logger(logger);
        } catch (...) {
          // ignore: register_logger may throw if name collides; fallback still usable
        }
      }
    });
  }

  void InitSpdlog(bool also_log_to_stderr)
  {
    // 显式 Init：覆盖 fallback logger，并启用文件+async。
    static std::once_flag spdlog_thread_pool_flag;
    std::call_once(spdlog_thread_pool_flag, []() -> void {
      spdlog::init_thread_pool(8192, 1);
      spdlog::flush_every(std::chrono::seconds(3));
    });

    std::string spd_dir = log_dir_ + "/spdlog/";
    std_fs::create_directories(spd_dir);

#ifndef SPDLOG_USE_ROLLING_LOG
    CleanupOldLogs(spd_dir);
#endif

#ifdef SPDLOG_USE_ROLLING_LOG
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        spd_dir + log_name_ + ".log", 10 * 1024 * 1024, 10);
#else
    auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
        spd_dir + log_name_ + ".log", 0, 0, false, keep_days);
#endif

    file_sink->set_level(spdlog::level::info);

    auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    console_sink->set_color(spdlog::level::info, "\033[1;32m");
    console_sink->set_color(spdlog::level::warn, "\033[1;33m");
    console_sink->set_color(spdlog::level::err, "\033[1;31m");
    console_sink->set_level(also_log_to_stderr ? spdlog::level::info : spdlog::level::off);

    spdlog_logger_ = std::make_shared<spdlog::async_logger>(
        "async_multi_sink_logger_" + log_name_ + "." + std::to_string(getpid()),
        spdlog::sinks_init_list{file_sink, console_sink}, spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
    spdlog_logger_->set_pattern("[%Y-%m-%d %T.%f][%t][%^%l%$][%s:%#] %v");
    spdlog_logger_->set_level(spdlog::level::info);
    spdlog_logger_->flush_on(spdlog::level::err);

    spdlog::register_logger(spdlog_logger_);
  }

  void InitComponentLogger(const std::string &module, const std::string &component, bool also_log_to_stderr)
  {
    std_fs::create_directories(log_dir_);

#ifndef SPDLOG_USE_ROLLING_LOG
    CleanupOldLogs(log_dir_);
#endif

    static std::once_flag spdlog_thread_pool_flag;
    std::call_once(spdlog_thread_pool_flag, []() -> void {
      spdlog::init_thread_pool(8192, 1);
      spdlog::flush_every(std::chrono::seconds(3));
    });

#ifdef SPDLOG_USE_ROLLING_LOG
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        log_dir_ + log_name_ + ".log", 10 * 1024 * 1024, 10);
#else
    auto file_sink = std::make_shared<spdlog::sinks::daily_file_sink_mt>(
        log_dir_ + log_name_ + ".log", 0, 0, false, keep_days);
#endif

    file_sink->set_level(spdlog::level::info);

    auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    console_sink->set_color(spdlog::level::info, "\033[1;32m");
    console_sink->set_color(spdlog::level::warn, "\033[1;33m");
    console_sink->set_color(spdlog::level::err, "\033[1;31m");
    console_sink->set_level(also_log_to_stderr ? spdlog::level::info : spdlog::level::off);

    auto logger = std::make_shared<spdlog::async_logger>(
        component + "." + std::to_string(getpid()),
        spdlog::sinks_init_list{file_sink, console_sink}, spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
    logger->set_pattern("[%Y-%m-%d %T.%f][%t][%^%l%$][%s:%#] %v");
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::err);
    spdlog_logger_              = logger;
    spdlog_loggers_[module]     = logger;
    spdlog_loggers_[component]  = logger;

    spdlog::register_logger(logger);
  }

protected:
  template <typename... Args>
  void InfoImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::info, fmt,
                std::forward<Args>(args)...);
  }

  template <typename... Args>
  void WarningImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::warn, fmt,
                std::forward<Args>(args)...);
  }

  template <typename... Args>
  void ErrorImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::err, fmt,
                std::forward<Args>(args)...);
  }

  template <typename... Args>
  void FatalImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::critical, fmt,
                std::forward<Args>(args)...);
    std::abort();
  }

protected:
  void CleanupOldLogs(const std::string &log_dir)
  {
    auto now_time = std_fs::file_time_type::clock::now();
    for (const auto &entry : std_fs::directory_iterator(log_dir))
    {
      // 兼容 C++14 experimental 和 C++17 filesystem
      if (std_fs::is_regular_file(entry.path()))
      {
        auto age_hours = std::chrono::duration_cast<std::chrono::hours>(
                             now_time - std_fs::last_write_time(entry.path()))
                             .count();
        if (age_hours > keep_days * 24)
        {
          std_fs::remove(entry.path());
        }
      }
    }
  }

  const std::shared_ptr<spdlog::logger> &GetActiveLogger(const char *file)
  {
    std::lock_guard<std::mutex> lk(mu_);
    std_fs::path p(file);
    // 1) 文件名 stem 精确匹配（兼容旧行为）
    std::string stem = p.stem().string();
    if (logger_likely(!stem.empty() && spdlog_loggers_.count(stem)))
    {
      return spdlog_loggers_[stem];
    }
    // 2) 逐级向上遍历父目录名，匹配模块级 key（同一模块目录下所有文件自动路由到同一 logger）
    for (auto it = p.has_parent_path() ? p.parent_path() : std_fs::path();
         !it.empty() && it != it.root_path();
         it = it.parent_path())
    {
      std::string dir = it.filename().string();
      if (!dir.empty() && spdlog_loggers_.count(dir))
      {
        return spdlog_loggers_[dir];
      }
    }
    // 3) 兜底
    return spdlog_logger_;
  }

protected:
  Logger() = default;
  ~Logger()
  {
    Shutdown();
  }
  Logger(const Logger &)            = delete;
  Logger &operator=(const Logger &) = delete;

  void Shutdown()
  {
    std::lock_guard<std::mutex> lk(mu_);
    spdlog_logger_.reset();
    spdlog_loggers_.clear();
  }

protected:
  template <typename... Args>
  friend void detail::LogInfo(const char *, int, const char *, Args &&...);
  template <typename... Args>
  friend void detail::LogWarning(const char *, int, const char *, Args &&...);
  template <typename... Args>
  friend void detail::LogError(const char *, int, const char *, Args &&...);
  template <typename... Args>
  friend void detail::LogFatal(const char *, int, const char *, Args &&...);

protected:
  std::string log_dir_{};
  std::string log_name_{};

protected:
  mutable std::mutex                                             mu_;
  std::once_flag                                                 fallback_init_flag_{};
  std::shared_ptr<spdlog::logger>                                  spdlog_logger_{};
  std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> spdlog_loggers_{};
};

namespace detail {
template <typename... Args>
void LogInfo(const char *file, int line, const char *fmt, Args &&...args)
{
  Logger::Instance().InfoImpl(file, line, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void LogWarning(const char *file, int line, const char *fmt, Args &&...args)
{
  Logger::Instance().WarningImpl(file, line, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void LogError(const char *file, int line, const char *fmt, Args &&...args)
{
  Logger::Instance().ErrorImpl(file, line, fmt, std::forward<Args>(args)...);
}
template <typename... Args>
void LogFatal(const char *file, int line, const char *fmt, Args &&...args)
{
  Logger::Instance().FatalImpl(file, line, fmt, std::forward<Args>(args)...);
}
} // namespace detail

// 初始化宏
#define MLOGGER_INIT(log_dir) ::mlogger::Logger::Instance().Init(log_dir)
#define MLOGGER_INIT_ARGS(argc, argv, also_log_to_stderr) \
  ::mlogger::Logger::Instance().Init(argc, argv, also_log_to_stderr)
#define MLOGGER_COMPONENT_INIT(module, component) \
  ::mlogger::Logger::Instance().Init(module, component)
// 基础目录+模块名+组件名初始化（从环境变量读取控制台标志）
#define MLOGGER_MODULE_INIT(base_dir, module, component) \
  ::mlogger::Logger::Instance().InitWithModule(base_dir, module, component)
// 基础目录+模块名+组件名初始化（自定义控制台标志）
#define MLOGGER_MODULE_INIT_EX(base_dir, module, component, also_log_to_stderr) \
  ::mlogger::Logger::Instance().InitWithModule(base_dir, module, component, also_log_to_stderr)
#define MLOGGER_COMPONENT_REGISTER(base_dir, module, component) ::mlogger::Logger::Instance().Register(base_dir, module, component)

#ifdef NDEBUG
#define MLOGGER_DEBUG(...)
#else
#define MLOGGER_DEBUG(...) ::mlogger::detail::LogInfo(__FILE__, __LINE__, __VA_ARGS__)
#endif
#define MLOGGER_INFO(...) ::mlogger::detail::LogInfo(__FILE__, __LINE__, __VA_ARGS__)
#define MLOGGER_WARN(...) ::mlogger::detail::LogWarning(__FILE__, __LINE__, __VA_ARGS__)
#define MLOGGER_ERROR(...) ::mlogger::detail::LogError(__FILE__, __LINE__, __VA_ARGS__)
#define MLOGGER_FATAL(...) ::mlogger::detail::LogFatal(__FILE__, __LINE__, __VA_ARGS__)

#define MLOGGER_ONCE_INFO(...)                               \
  do                                                         \
  {                                                          \
    static bool mlog_once_flag_##__FILE__##__LINE__ = false; \
    if (!mlog_once_flag_##__FILE__##__LINE__)                \
    {                                                        \
      mlog_once_flag_##__FILE__##__LINE__ = true;            \
      MLOGGER_INFO(__VA_ARGS__);                             \
    }                                                        \
  } while (0)

#define MLOGGER_ONCE_WARN(...)                               \
  do                                                         \
  {                                                          \
    static bool mlog_once_flag_##__FILE__##__LINE__ = false; \
    if (!mlog_once_flag_##__FILE__##__LINE__)                \
    {                                                        \
      mlog_once_flag_##__FILE__##__LINE__ = true;            \
      MLOGGER_WARN(__VA_ARGS__);                             \
    }                                                        \
  } while (0)

#define MLOGGER_ONCE_ERROR(...)                              \
  do                                                         \
  {                                                          \
    static bool mlog_once_flag_##__FILE__##__LINE__ = false; \
    if (!mlog_once_flag_##__FILE__##__LINE__)                \
    {                                                        \
      mlog_once_flag_##__FILE__##__LINE__ = true;            \
      MLOGGER_ERROR(__VA_ARGS__);                            \
    }                                                        \
  } while (0)

#define MLOGGER_EVERY_N_INFO(N, ...)                                           \
  do                                                                           \
  {                                                                            \
    static_assert((N) > 0, "N must be a positive integer greater than zero."); \
    static std::atomic<int> mlog_occurrences_##__FILE__##__LINE__ = 0;         \
    if (++mlog_occurrences_##__FILE__##__LINE__ > (N))                         \
    {                                                                          \
      mlog_occurrences_##__FILE__##__LINE__ -= (N);                            \
    }                                                                          \
    if (mlog_occurrences_##__FILE__##__LINE__ == 1)                            \
    {                                                                          \
      MLOGGER_INFO(__VA_ARGS__);                                               \
    }                                                                          \
  } while (0)

#define MLOGGER_EVERY_N_WARN(N, ...)                                           \
  do                                                                           \
  {                                                                            \
    static_assert((N) > 0, "N must be a positive integer greater than zero."); \
    static std::atomic<int> mlog_occurrences_##__FILE__##__LINE__ = 0;         \
    if (++mlog_occurrences_##__FILE__##__LINE__ > (N))                         \
    {                                                                          \
      mlog_occurrences_##__FILE__##__LINE__ -= (N);                            \
    }                                                                          \
    if (mlog_occurrences_##__FILE__##__LINE__ == 1)                            \
    {                                                                          \
      MLOGGER_WARN(__VA_ARGS__);                                               \
    }                                                                          \
  } while (0)

#define MLOGGER_EVERY_N_ERROR(N, ...)                                          \
  do                                                                           \
  {                                                                            \
    static_assert((N) > 0, "N must be a positive integer greater than zero."); \
    static std::atomic<int> mlog_occurrences_##__FILE__##__LINE__ = 0;         \
    if (++mlog_occurrences_##__FILE__##__LINE__ > (N))                         \
    {                                                                          \
      mlog_occurrences_##__FILE__##__LINE__ -= (N);                            \
    }                                                                          \
    if (mlog_occurrences_##__FILE__##__LINE__ == 1)                            \
    {                                                                          \
      MLOGGER_ERROR(__VA_ARGS__);                                              \
    }                                                                          \
  } while (0)

} // namespace mlogger