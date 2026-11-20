cmake_minimum_required(VERSION 3.8)
project(usv_ardupilot_velocity_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(m_common REQUIRED)
find_package(spdlog REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(ardupilot_velocity_bridge src/ardupilot_velocity_bridge.cpp)
ament_target_dependencies(
  ardupilot_velocity_bridge
  rclcpp
  m_common
  spdlog
  mavros_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)
target_link_libraries(ardupilot_velocity_bridge m_common::mlogger)

install(TARGETS ardupilot_velocity_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
