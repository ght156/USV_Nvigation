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

#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <string_view>
#include <algorithm>
#include <initializer_list>
#include <exception>
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

// 路径前缀匹配（目录边界敏感：/a/bc 不被 /a/b 命中）
inline bool PathStartsWith(const std::string &path, const std::string &prefix)
{
  if (prefix.size() > path.size()) return false;
  if (path.compare(0, prefix.size(), prefix) != 0) return false;
  return prefix.size() == path.size() || path[prefix.size()] == '/';
}

class Logger {
public:
  // 单例真身定义于 libmlogger.so（src/mlogger_singleton.cpp）。
  // 所有 .so 链接同一动态库 -> 符号解析到唯一定义（跨 .so 唯一，
  // C++11 即可，不依赖 C++17 inline 变量或 GNU unique 符号）。
  static Logger &Instance();

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

  // 进程入口：字符串身份注册（main 无 Node 上下文；MLOGGER_MODULE_INIT/_EX 宏使用）
  void InitWithModule(const std::string &base_dir,
                      const std::string &module,
                      const std::string &component)
  {
    auto env                = std::getenv("ALSO_LOG_TO_STDERR");
    bool also_log_to_stderr = (env == nullptr || std::string(env) != "false");
    InitWithModule(base_dir, module, component, also_log_to_stderr);
  }

  void InitWithModule(const std::string &base_dir,
                      const std::string &module,
                      const std::string &component,
                      bool               also_log_to_stderr)
  {
    RegisterImpl(base_dir, module, component, also_log_to_stderr, nullptr);
  }

  // 带目录前缀注册（MLOGGER_NODE_REGISTER 宏使用）：node 为字符串身份
  // （日志文件名，由调用方传入，如节点名/实例名）；src_file = 调用处 __FILE__，
  // 其所在目录登记为路由前缀 —— 同包多节点各自文件。
  void RegisterNode(const std::string &base_dir, const std::string &module,
                    const std::string &node, const char *src_file)
  {
    auto env                = std::getenv("ALSO_LOG_TO_STDERR");
    bool also_log_to_stderr = (env == nullptr || std::string(env) != "false");
    RegisterImpl(base_dir, module, node, also_log_to_stderr, src_file);
  }

  // 显式目录前缀注册：dirs 直接作为路由前缀（不取 parent），供库级 initLog
  // 把整个源码树挂到宿主身份（如 yolo-inference 把包根挂到宿主组件）。
  // 同 (module, identity) 已注册时，新目录追加到该身份的前缀列表（幂等）。
  void RegisterNodeDirs(const std::string &base_dir, const std::string &module,
                        const std::string &identity,
                        std::initializer_list<std::string> dirs)
  {
    auto env                = std::getenv("ALSO_LOG_TO_STDERR");
    bool also_log_to_stderr = (env == nullptr || std::string(env) != "false");
    RegisterImplDirs(base_dir, module, identity, also_log_to_stderr, dirs);
  }

private:
  void RegisterImpl(const std::string &base_dir, const std::string &module,
                    const std::string &identity, bool also_log_to_stderr,
                    const char *src_file)
  {
    std::vector<std::string> dirs;
    if (src_file) dirs.push_back(std_fs::absolute(std_fs::path(src_file).parent_path()).string());
    RegisterImplDirs(base_dir, module, identity, also_log_to_stderr, dirs);
  }

  void RegisterImplDirs(const std::string &base_dir, const std::string &module,
                        const std::string &identity, bool also_log_to_stderr,
                        const std::vector<std::string> &dirs)
  {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = comp_regs_.find(identity);
    if (it != comp_regs_.end())
    {
      if (it->second.module != module)
      {
        fprintf(stderr,
                "[mlogger] 拒绝注册: '%s' 已注册到 module='%s'，忽略 module='%s' 的注册"
                "（同容器内组件请使用唯一节点名）\n",
                identity.c_str(), it->second.module.c_str(), module.c_str());
        return;
      }
      // 同 (module, identity)：追加新目录前缀（如库级 initLog 补挂源码树）
      for (const auto &d : dirs)
      {
        if (std::find(it->second.src_dirs.begin(), it->second.src_dirs.end(), d) ==
            it->second.src_dirs.end())
        {
          it->second.src_dirs.push_back(d);
        }
      }
      return;
    }
    log_name_ = identity;
    log_dir_  = std_fs::absolute(base_dir).string() + "/" + module + "/";
    InitComponentLogger(module, identity, also_log_to_stderr);
    RegEntry e;
    e.module   = module;
    e.src_dirs = dirs;
    e.logger   = spdlog_logger_;
    comp_regs_[identity] = e;
    if (!root_logger_) root_logger_ = spdlog_logger_;
  }

public:

  // Flush all sinks (needed before abort / short-lived processes; async may otherwise drop).
  void Flush()
  {
    std::lock_guard<std::mutex> lk(mu_);
    FlushUnlocked();
  }

protected:
  void FlushUnlocked()
  {
    if (spdlog_logger_) {
      spdlog_logger_->flush();
    }
    for (auto &kv : spdlog_loggers_) {
      if (kv.second) {
        kv.second->flush();
      }
    }
  }

  void InstallExitFlushHandlers()
  {
    static std::once_flag once;
    std::call_once(once, []() {
      std::atexit([]() {
        try {
          Logger::Instance().Flush();
        } catch (...) {
        }
      });
      static std::terminate_handler prev = nullptr;
      prev = std::set_terminate([]() {
        try {
          Logger::Instance().Flush();
        } catch (...) {
        }
        if (prev) {
          prev();
        } else {
          std::abort();
        }
      });
    });
  }

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
      logger->set_pattern("[%Y-%m-%d %T.%f][%t][%n][%^%l%$][%s:%#] %v");
      logger->set_level(spdlog::level::info);
      logger->flush_on(spdlog::level::err);

      std::lock_guard<std::mutex> lk(mu_);
      if (!spdlog_logger_) {
        spdlog_logger_ = logger;
        try {
          try { spdlog::register_logger(logger); } catch (...) { /* 同名已在注册表：本地 shared_ptr 照常可用 */ }
        } catch (...) {
          // ignore: register_logger may throw if name collides; fallback still usable
        }
      }
    });
  }

  void InitSpdlog(bool also_log_to_stderr)
  {
    InstallExitFlushHandlers();
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
    spdlog_logger_->set_pattern("[%Y-%m-%d %T.%f][%t][%n][%^%l%$][%s:%#] %v");
    spdlog_logger_->set_level(spdlog::level::info);
    spdlog_logger_->flush_on(spdlog::level::err);

    spdlog::register_logger(spdlog_logger_);
  }

  void InitComponentLogger(const std::string &module, const std::string &component, bool also_log_to_stderr)
  {
    InstallExitFlushHandlers();
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
        component,
        spdlog::sinks_init_list{file_sink, console_sink}, spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
    logger->set_pattern("[%Y-%m-%d %T.%f][%t][%n][%^%l%$][%s:%#] %v");
    logger->set_level(spdlog::level::info);
    logger->flush_on(spdlog::level::err);
    spdlog_logger_              = logger;
    spdlog_loggers_[module]     = logger;
    spdlog_loggers_[component]  = logger;

    try { spdlog::register_logger(logger); } catch (...) { /* 同名已在注册表：本地 shared_ptr 照常可用 */ }
  }

protected:
  template <typename... Args>
  void InfoImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::info, fmt::runtime(fmt),
                std::forward<Args>(args)...);
  }

  template <typename... Args>
  void WarningImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::warn, fmt::runtime(fmt),
                std::forward<Args>(args)...);
  }

  template <typename... Args>
  void ErrorImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::err, fmt::runtime(fmt),
                std::forward<Args>(args)...);
    Flush();
  }

  template <typename... Args>
  void FatalImpl(const char *file, int line, const char *fmt, Args &&...args)
  {
    EnsureInitialized();
    const auto &logger = GetActiveLogger(file);
    logger->log(spdlog::source_loc{file, line, nullptr}, spdlog::level::critical, fmt::runtime(fmt),
                std::forward<Args>(args)...);
    Flush();
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
    std_fs::path p = std_fs::absolute(file);
    // 1) 文件名 stem 精确匹配（兼容旧行为）
    std::string stem = p.stem().string();
    if (logger_likely(!stem.empty() && spdlog_loggers_.count(stem)))
    {
      return spdlog_loggers_[stem];
    }
    // 2) 节点目录最长前缀匹配：同容器多组件各自文件（组件构造处 __FILE__ 所在目录）
    const std::shared_ptr<spdlog::logger> *best = nullptr;
    size_t best_len = 0;
    for (const auto &kv : comp_regs_)
    {
      for (const std::string &d : kv.second.src_dirs)
      {
        if (!d.empty() && d.size() > best_len &&
            PathStartsWith(p.parent_path().string(), d))
        {
          best = &kv.second.logger;
          best_len = d.size();
        }
      }
    }
    if (best) return *best;
    // 3) 逐级向上遍历父目录名，匹配模块级 key（包级 fallback）
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
    // 4) 兜底：首个注册者（与注册顺序无关）+ 一次性提示
    std::call_once(fallback_warn_flag_, [file]() {
      fprintf(stderr,
              "[mlogger] warn: %s 未被任何节点目录/module 覆盖，路由到首个注册节点的日志文件\n",
              file);
    });
    return root_logger_ ? root_logger_ : spdlog_logger_;
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
    FlushUnlocked();
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

  struct RegEntry
  {
    std::string module;
    std::vector<std::string> src_dirs;  // 路由前缀列表（绝对归一化）；空=进程级注册，不参与前缀路由
    std::shared_ptr<spdlog::logger> logger;
  };
  std::unordered_map<std::string, RegEntry> comp_regs_;  // key: 节点名/组件名（进程内唯一）
  std::shared_ptr<spdlog::logger> root_logger_;          // 首个注册者：路由兜底（注册顺序无关）
  std::once_flag fallback_warn_flag_;
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
// 进程入口（main 无 Node 上下文）：base_dir/module/component 均为字符串身份
#define MLOGGER_MODULE_INIT(base_dir, module, component) \
  ::mlogger::Logger::Instance().InitWithModule(base_dir, module, component)
// 进程入口（自定义控制台标志）
#define MLOGGER_MODULE_INIT_EX(base_dir, module, component, also_log_to_stderr) \
  ::mlogger::Logger::Instance().InitWithModule(base_dir, module, component, also_log_to_stderr)
// 节点/带目录前缀注册：node_name 为字符串身份（日志文件名），由调用方显式传入
// （如 Node 构造内传 this->get_name()，或外部配置传入的实例名）；
// 调用处 __FILE__ 目录 = 路由前缀。不依赖 rclcpp（纯字符串接口）。
#define MLOGGER_NODE_REGISTER(base_dir, module, node_name) \
  ::mlogger::Logger::Instance().RegisterNode(base_dir, module, node_name, __FILE__)

#ifdef NDEBUG
#define MLOGGER_DEBUG(...)
#else
#define MLOGGER_DEBUG(...) ::mlogger::detail::LogInfo(__FILE__, __LINE__, __VA_ARGS__)
#endif
#define MLOGGER_INFO(...) ::mlogger::detail::LogInfo(__FILE__, __LINE__, __VA_ARGS__)
#define MLOGGER_WARN(...) ::mlogger::detail::LogWarning(__FILE__, __LINE__, __VA_ARGS__)
#define MLOGGER_ERROR(...) ::mlogger::detail::LogError(__FILE__, __LINE__, __VA_ARGS__)
#define MLOGGER_FATAL(...) ::mlogger::detail::LogFatal(__FILE__, __LINE__, __VA_ARGS__)

// 按调用点限频输出告警。INTERVAL_MS 使用 steady_clock，避免系统时间校准造成
// 长时间不再打印或短时间重复打印；CAS 保证多线程并发时每个周期最多一条。
#define MLOGGER_THROTTLE_WARN(INTERVAL_MS, ...)                                \
  do                                                                            \
  {                                                                             \
    static_assert((INTERVAL_MS) > 0, "INTERVAL_MS must be positive.");          \
    static std::atomic<int64_t> mlog_last_warn_ms{0};                           \
    const auto mlog_now_ms = std::chrono::duration_cast<std::chrono::milliseconds>( \
      std::chrono::steady_clock::now().time_since_epoch()).count();              \
    auto mlog_previous_ms = mlog_last_warn_ms.load(std::memory_order_relaxed);   \
    if (mlog_now_ms - mlog_previous_ms >= static_cast<int64_t>(INTERVAL_MS) &&   \
      mlog_last_warn_ms.compare_exchange_strong(                                \
        mlog_previous_ms, mlog_now_ms, std::memory_order_relaxed))               \
    {                                                                           \
      MLOGGER_WARN(__VA_ARGS__);                                                 \
    }                                                                           \
  } while (0)

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
