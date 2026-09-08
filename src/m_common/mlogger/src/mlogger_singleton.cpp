// mlogger 单例唯一真身。
// 编译进 SHARED 库 libmlogger.so：所有 .so / 可执行文件链接同一动态库，
// 运行时符号解析到唯一定义 —— 跨 .so 单例一致（C++11 即可，标准行为）。
// 若放 header 用函数局部 static，跨 .so 唯一性依赖 GNU unique 符号（GCC/Linux 特性）。

#include <mlogger/mlogger.hpp>

namespace mlogger {

Logger &Logger::Instance()
{
  static Logger instance;
  return instance;
}

}  // namespace mlogger
