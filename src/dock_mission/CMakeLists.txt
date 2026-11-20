cmake_minimum_required(VERSION 3.8)
project(dock_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(usv_rs485_driver REQUIRED)
find_package(m_common REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(dock_mission_core
  src/bay_loader.cpp
  src/dock_task_api.cpp
  src/gnss_staging.cpp
  src/mission_logic.cpp
  src/task_event.cpp
  src/types.cpp)
target_compile_features(dock_mission_core PUBLIC cxx_std_17)
target_include_directories(dock_mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dock_mission_core nlohmann_json::nlohmann_json yaml-cpp m_common::mlogger)
ament_target_dependencies(dock_mission_core ament_index_cpp)
ament_target_dependencies(dock_mission_core m_common)

add_executable(dock_mission_node src/dock_mission_node.cpp src/dock_mission_node_main.cpp)
target_compile_features(dock_mission_node PRIVATE cxx_std_17)
target_link_libraries(dock_mission_node dock_mission_core m_common::mlogger nlohmann_json::nlohmann_json)
ament_target_dependencies(dock_mission_node
  geometry_msgs m_common nav_msgs rcl_interfaces rclcpp rclcpp_action std_msgs std_srvs
  usv_rs485_driver)

install(TARGETS dock_mission_core dock_mission_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_dock_mission_core test/test_dock_mission_core.cpp)
  target_link_libraries(test_dock_mission_core dock_mission_core)
  ament_add_pytest_test(python_tests test
    APPEND_ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    TIMEOUT 120)
endif()

ament_package()
