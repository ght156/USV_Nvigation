# =============================================================================
# Sub-library: m_common::mlogger
#
# 类型      : header-only（无 .cpp，无产物）
# 头文件    : ${CMAKE_CURRENT_SOURCE_DIR}/include/mlogger/*.hpp
# 运行依赖  : spdlog
#
# 产出：
#   1) 安装头文件到 <prefix>/include/mlogger/
#   2) 一个 build-time INTERFACE 桥目标 m_common_mlogger_iface，
#      供同包内其他子库（例如 m_utils）在 build 阶段链接。
#      下游消费方（其它 ROS2 包）请勿直接依赖此桥，应使用
#      install 后由 m_common-extras.cmake 创建的 m_common::mlogger。
# =============================================================================

find_package(spdlog REQUIRED)

install(
  DIRECTORY include/
  DESTINATION include
)

add_library(m_common_mlogger_iface INTERFACE)
target_include_directories(m_common_mlogger_iface INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(m_common_mlogger_iface INTERFACE cxx_std_17)
target_link_libraries(m_common_mlogger_iface INTERFACE spdlog::spdlog)

# 把本子库需要 export 给下游的内容，向上汇总到顶层 CMakeLists：
#   - 运行/编译期依赖：spdlog
list(APPEND M_COMMON_EXPORT_DEPENDS spdlog)
set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)
