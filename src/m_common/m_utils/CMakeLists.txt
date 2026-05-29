# =============================================================================
# Sub-library: m_common::m_utils
#
# 类型      : 编译型 STATIC 库（示例骨架）
# 源码布局  :
#   include/m_utils/*.hpp   (公共头)
#   src/*.cpp               (实现)
# 输出文件  : libm_common_m_utils.a   （加包前缀避免与全局名冲突）
# 同包依赖  : m_common_mlogger_iface  （build-time INTERFACE 桥，PRIVATE 链接）
# =============================================================================

find_package(nlohmann_json REQUIRED)

file(GLOB M_UTILS_SRCS CONFIGURE_DEPENDS src/*.cpp)

add_library(m_utils STATIC ${M_UTILS_SRCS})
set_target_properties(m_utils PROPERTIES
  OUTPUT_NAME m_common_m_utils
  POSITION_INDEPENDENT_CODE ON)

target_include_directories(m_utils PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_compile_features(m_utils PUBLIC cxx_std_17)

# 同包子库相互依赖（PRIVATE）：
#   实现里 #include "mlogger/mlogger.hpp"，仅编译期需要，不向下游传播 mlogger 头。
#   下游 link m_common::m_utils 时由 extras.cmake 把 m_common::mlogger 作为
#   INTERFACE_LINK_LIBRARIES 注入，从而自动解析 spdlog 符号。
target_link_libraries(m_utils PRIVATE m_common_mlogger_iface)
target_link_libraries(m_utils PRIVATE nlohmann_json::nlohmann_json)

install(TARGETS m_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

# 向顶层汇总要 export 的库名（最终由顶层 ament_export_libraries 统一调用）。
list(APPEND M_COMMON_EXPORT_LIBS m_common_m_utils)
set(M_COMMON_EXPORT_LIBS "${M_COMMON_EXPORT_LIBS}" PARENT_SCOPE)

list(APPEND M_COMMON_EXPORT_DEPENDS nlohmann_json)
set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)
