cmake_minimum_required(VERSION 3.8)
project(m_common)

# -----------------------------------------------------------------------------
# 全局编译选项
# -----------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# -----------------------------------------------------------------------------
# 子库聚合容器
#   每个子目录的 CMakeLists.txt 自治：定义 target / install，
#   并把要 export 的依赖名 / 库名以 PARENT_SCOPE 形式上抛到下面两个变量：
#     - M_COMMON_EXPORT_DEPENDS : 传给 ament_export_dependencies()
#     - M_COMMON_EXPORT_LIBS    : 传给 ament_export_libraries()
# -----------------------------------------------------------------------------
set(M_COMMON_EXPORT_DEPENDS "")
set(M_COMMON_EXPORT_LIBS    "")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StampedStatus.msg"
  "msg/KeyValue.msg"
  "msg/VideoWarningCode.msg"
  "msg/NavStatus.msg"
  "msg/NavTaskEvent.msg"
  "srv/RTSPStreamSwitch.srv"
  "srv/ReportVideoWarning.srv"
  "srv/SendWaypoints.srv"
  "srv/SetPause.srv"
  "srv/EmergencyStop.srv"
  "srv/CancelMission.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_subdirectory(mlogger)
add_subdirectory(m_utils)
add_subdirectory(rtsp_interface)

install(DIRECTORY include/
  DESTINATION include
)

# 新增子库时只需在此追加一行 add_subdirectory(<name>)，
# 顶层 CMakeLists 无需关心其源码细节。

# -----------------------------------------------------------------------------
# 统一 export：给下游 find_package(m_common) 后看到的接口
# -----------------------------------------------------------------------------
if(M_COMMON_EXPORT_DEPENDS)
  list(REMOVE_DUPLICATES M_COMMON_EXPORT_DEPENDS)
  ament_export_dependencies(${M_COMMON_EXPORT_DEPENDS})
endif()

ament_export_include_directories(include)

if(M_COMMON_EXPORT_LIBS)
  list(REMOVE_DUPLICATES M_COMMON_EXPORT_LIBS)
  ament_export_libraries(${M_COMMON_EXPORT_LIBS})
endif()

# 命名空间 IMPORTED 目标（m_common::mlogger / m_common::m_utils 等）
# 仍由 cmake/m_common-extras.cmake.in 集中描述并通过 CONFIG_EXTRAS 暴露。
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/m_common-extras.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/m_common-extras.cmake"
  @ONLY
)

ament_package(
  CONFIG_EXTRAS "${CMAKE_CURRENT_BINARY_DIR}/m_common-extras.cmake"
)
