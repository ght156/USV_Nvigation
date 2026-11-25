cmake_minimum_required(VERSION 3.8)
project(usv_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(usv_rs485_driver REQUIRED)
find_package(m_common REQUIRED)

add_library(docking_core
  src/anchor_filter.cpp
  src/docking_common.cpp
  src/motion_control_core.cpp
  src/safety_core.cpp
  src/yaw_rate_estimator.cpp)
target_compile_features(docking_core PUBLIC cxx_std_17)
target_include_directories(docking_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(docking_core m_common::mlogger)
ament_target_dependencies(docking_core m_common)

add_library(docking_components SHARED
  src/docking_fsm.cpp
  src/docking_motion_controller.cpp
  src/docking_pose_estimator.cpp
  src/docking_safety.cpp)
target_compile_features(docking_components PUBLIC cxx_std_17)
target_include_directories(docking_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(docking_components docking_core m_common::mlogger nlohmann_json::nlohmann_json)
ament_target_dependencies(docking_components
  geometry_msgs m_common nav_msgs rclcpp rclcpp_action rclcpp_components
  std_msgs tf2 tf2_ros usv_rs485_driver)
rclcpp_components_register_nodes(docking_components
  "usv_docking::DockingPoseEstimator"
  "usv_docking::DockingFsm"
  "usv_docking::DockingMotionController"
  "usv_docking::DockingSafety")

foreach(node IN ITEMS docking_pose_estimator docking_fsm docking_motion_controller docking_safety)
  add_executable(${node} src/${node}_main.cpp)
  target_compile_features(${node} PRIVATE cxx_std_17)
  target_link_libraries(${node} docking_components)
  ament_target_dependencies(${node} rclcpp)
endforeach()

install(TARGETS docking_core docking_components
  docking_pose_estimator docking_fsm docking_motion_controller docking_safety
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_docking_core test/test_docking_core.cpp)
  target_link_libraries(test_docking_core docking_core)
  ament_add_pytest_test(python_tests test
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    TIMEOUT 120)
endif()

ament_package()
