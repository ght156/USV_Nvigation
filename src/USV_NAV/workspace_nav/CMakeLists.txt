cmake_minimum_required(VERSION 3.8)
project(workspace_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(m_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(mission_bridge_core src/mission_bridge_core.cpp)
target_compile_features(mission_bridge_core PUBLIC cxx_std_17)
target_include_directories(mission_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_bridge_core OpenSSL::Crypto nlohmann_json::nlohmann_json)

add_library(navigation_core
  src/cloud_filter_core.cpp
  src/costmap_reinflater.cpp
  src/nav_status_logic.cpp
  src/navigation_utils.cpp
  src/zone_geometry.cpp)
target_compile_features(navigation_core PUBLIC cxx_std_17)
target_include_directories(navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  navigation_core mission_bridge_core Eigen3::Eigen yaml-cpp m_common::mlogger)
ament_target_dependencies(navigation_core
  ament_index_cpp rcl_interfaces rclcpp sensor_msgs)

add_executable(mission_bridge src/mission_bridge_node.cpp)
target_compile_features(mission_bridge PRIVATE cxx_std_17)
target_link_libraries(mission_bridge mission_bridge_core yaml-cpp m_common::mlogger)
ament_target_dependencies(mission_bridge
  ament_index_cpp
  action_msgs
  geometry_msgs
  m_common
  nav2_msgs
  nav_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  tf2_ros)

set(navigation_node_dependencies
  action_msgs geometry_msgs lifecycle_msgs m_common nav2_msgs nav_msgs
  rcl_interfaces rclcpp rclcpp_action sensor_msgs std_msgs std_srvs tf2_ros)
foreach(node IN ITEMS
    waypoint_transform waypoint_with_state nav_status_aggregator
    zone_manager zone_monitor livox_cloud_filter map_server_guard)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PRIVATE cxx_std_17)
  target_link_libraries(${node} navigation_core)
  ament_target_dependencies(${node} ${navigation_node_dependencies})
endforeach()

install(TARGETS
  mission_bridge mission_bridge_core navigation_core
  waypoint_transform waypoint_with_state nav_status_aggregator
  zone_manager zone_monitor livox_cloud_filter map_server_guard
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config map json DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_mission_bridge_core test/test_mission_bridge_core.cpp)
  target_link_libraries(test_mission_bridge_core mission_bridge_core)

  ament_add_gtest(test_remaining_nodes_core test/test_remaining_nodes_core.cpp)
  target_link_libraries(test_remaining_nodes_core navigation_core)

  ament_add_pytest_test(python_tests test
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    TIMEOUT 120)
endif()

ament_package()
