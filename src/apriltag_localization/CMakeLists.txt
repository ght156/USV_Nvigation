cmake_minimum_required(VERSION 3.20)
project(apriltag_localization)
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(m_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 获取架构
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  set(SYSTEM_ARCHITECTURE  "x86" CACHE STRING "system architecture" FORCE)
else()
  set(SYSTEM_ARCHITECTURE  "arm" CACHE STRING "system architecture" FORCE)
endif()

set(ENABLE_AI ON CACHE BOOL "开启AI-22.04开启" FORCE) # arm64 22.04开启
if(${SYSTEM_ARCHITECTURE} MATCHES "x86")
else()
endif()

link_directories(${TENSORRT_LIBRARY_DIRS})
include_directories(include/)

add_subdirectory(apriltag)
include_directories(./)
add_definitions(${PCL_DEFINITIONS})

# ComposableNodeContainer 加载 perception::AprilTagLocalizationNode
add_library(apriltag_localization_components SHARED
  src/apriltag_node.cpp
  src/apriltag_localization_node.cpp
  src/apriltag_component_register.cpp
)
target_include_directories(apriltag_localization_components PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:include>"
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(apriltag_localization_components PRIVATE
  APRILTAG_LOCALIZATION_COMPONENTS_BUILDING_DLL
)
target_link_libraries(apriltag_localization_components
  ${OpenCV_LIBS}
  m_common::mlogger
  yaml-cpp
  ${PCL_LIBRARIES}
  fmt
  apriltag
)
ament_target_dependencies(apriltag_localization_components
  rclcpp
  rclcpp_components
  cv_bridge
  m_common
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
)

rclcpp_components_register_nodes(
  apriltag_localization_components
  "perception::AprilTagLocalizationNode"
)

add_executable(apriltag_localization_cpp src/apriltag_node_main.cpp)
target_include_directories(apriltag_localization_cpp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(apriltag_localization_cpp rclcpp m_common)
target_link_libraries(apriltag_localization_cpp
  apriltag_localization_components
  m_common::mlogger
)

install(TARGETS apriltag_localization_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS apriltag_localization_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/config/
  DESTINATION config
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_export_include_directories(include)
ament_export_libraries(apriltag_localization_components)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  cv_bridge
  m_common
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
)

ament_package()
