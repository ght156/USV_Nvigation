cmake_minimum_required(VERSION 3.16)
project(bw_gi320_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED) 
find_package(m_common REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 自定义消息需要
find_package(rosidl_default_runtime REQUIRED)
find_package(tinyxml2_vendor REQUIRED)  # ROS2 vendor 包
find_package(TinyXML2 REQUIRED)         # 系统 tinyxml2

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Gi320Gngga.msg"
    "msg/Gi320Inspvaa.msg"

    "srv/ResetGgaOdom.srv"
    "srv/ResetInspvaaOdom.srv"

    DEPENDENCIES 
      builtin_interfaces
      std_msgs
)


# 生成动态可链接库
add_library(bw_gi320_driver_components SHARED
  src/bw_gi320_nmea_parser.cpp
  src/bw_gi320_driver_node.cpp
  src/bw_gi320_udp_receiver.cpp
)
target_include_directories(bw_gi320_driver_components PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(bw_gi320_driver_components PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_definitions(bw_gi320_driver_components PRIVATE BW_GI320_DRIVER_COMPONENTS_BUILDING_DLL)


ament_target_dependencies(bw_gi320_driver_components
  rclcpp
  rclcpp_components
  rosidl_default_runtime
  std_msgs
  builtin_interfaces
  rosidl_default_runtime
  yaml-cpp
  ament_index_cpp # 运行时查找资源路径
  m_common
  spdlog
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(bw_gi320_driver_components
  ${cpp_typesupport_target}
  m_common::mlogger
  yaml-cpp
  Threads::Threads
  tinyxml2::tinyxml2
)
rclcpp_components_register_nodes(bw_gi320_driver_components "bw_gi320_driver::BwGi320DriverNode")

add_executable(bw_gi320_driver_node src/bw_gi320_driver_node_main.cpp)
target_include_directories(bw_gi320_driver_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(bw_gi320_driver_node rclcpp m_common spdlog)

target_link_libraries(bw_gi320_driver_node
  bw_gi320_driver_components
  ${cpp_typesupport_target}
)
install(TARGETS bw_gi320_driver_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bw_gi320_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(bw_gi320_driver_components)
ament_export_dependencies(
  rosidl_default_runtime
  rclcpp
  rclcpp_components
  std_msgs
  builtin_interfaces
  rosidl_default_runtime
  geometry_msgs
  ament_index_cpp # 运行时查找资源路径
  m_common
  spdlog
)


ament_package()
