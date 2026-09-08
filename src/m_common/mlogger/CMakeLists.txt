# =============================================================================
# Sub-library: m_common::mlogger
#
# 类型      : 头文件 + libmlogger.so（单例真身；跨 .so 唯一靠动态库符号解析，
#             C++11 即可，不依赖 C++17 inline 变量或 GNU unique 符号）
# 头文件    : ${CMAKE_CURRENT_SOURCE_DIR}/include/mlogger/*.hpp
# 运行依赖  : spdlog
#
# 产出：
#   1) 安装头文件到 <prefix>/include/mlogger/
#   2) SHARED 库 mlogger（libmlogger.so，含 Logger::Instance 单例定义）
#   3) 一个 build-time INTERFACE 桥目标 m_common_mlogger_iface，
#      供同包内其他子库（例如 m_utils）在 build 阶段链接。
#      下游消费方（其它 ROS2 包）请勿直接依赖此桥，应使用
#      install 后由 m_common-extras.cmake 创建的 m_common::mlogger。
# =============================================================================

find_package(spdlog REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(mlogger SHARED
  src/mlogger_singleton.cpp
)
target_include_directories(mlogger PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(mlogger PUBLIC cxx_std_17)
target_link_libraries(mlogger PRIVATE spdlog::spdlog)

install(TARGETS mlogger
  LIBRARY DESTINATION lib
)

add_library(m_common_mlogger_iface INTERFACE)
target_include_directories(m_common_mlogger_iface INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(m_common_mlogger_iface INTERFACE cxx_std_17)
target_link_libraries(m_common_mlogger_iface INTERFACE spdlog::spdlog mlogger)

# 把本子库需要 export 给下游的内容，向上汇总到顶层 CMakeLists：
#   - 运行/编译期依赖：spdlog
#   - 链接库：mlogger（libmlogger.so，下游经 ament_target_dependencies 自动链接，
#     单例符号全局唯一）
list(APPEND M_COMMON_EXPORT_DEPENDS spdlog)
list(APPEND M_COMMON_EXPORT_LIBS mlogger)
set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)
set(M_COMMON_EXPORT_LIBS "${M_COMMON_EXPORT_LIBS}" PARENT_SCOPE)
