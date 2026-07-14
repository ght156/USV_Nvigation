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
# find_package(VTK REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(m_common REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# if(TARGET Python3::Python)
#     set_target_properties(Python3::Python PROPERTIES
#         IMPORTED_LOCATION "${Python3_LIBRARY}"
#         INTERFACE_INCLUDE_DIRECTORIES "${Python3_INCLUDE_DIR}"
#     )
# endif()
# if(TARGET Python::Python)
#     set_target_properties(Python::Python PROPERTIES
#         IMPORTED_LOCATION "${Python3_LIBRARY}"
#         INTERFACE_INCLUDE_DIRECTORIES "${Python3_INCLUDE_DIR}"
#     )
# endif()


# 获取架构
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")  
  set(SYSTEM_ARCHITECTURE  "x86" CACHE STRING "system architecture" FORCE)
else()
  set(SYSTEM_ARCHITECTURE  "arm" CACHE STRING "system architecture" FORCE)
endif()
# include(SKD/include)

set(ENABLE_AI ON CACHE BOOL "开启AI-22.04开启" FORCE) # arm64 22.04开启
if(${SYSTEM_ARCHITECTURE} MATCHES "x86")  
    # Filter out -flto=auto flags
    # string(REGEX REPLACE "-flto=auto *" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
    # string(REGEX REPLACE "-flto=auto *" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    # string(REGEX REPLACE "-flto=auto *" "" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
    # string(REGEX REPLACE "-flto=auto *" "" CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")
    # string(REGEX REPLACE "-flto=auto *" "" CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS}") #只在x86 cuda禁用flto
    # set(onnxruntime_DIR /home/jzw/Libs/onnxruntime-linux-x64-gpu-1.22.0/)
    # set(TENSORRT_INCLUDE_DIRS /home/jzw/Libs/TensorRT-10.13.0.35/include)
    # set(TENSORRT_LIBRARY_DIRS /home/jzw/Libs/TensorRT-10.13.0.35/lib)
    # find_package(onnxruntime REQUIRED)
#   endif()
  # set(CHIP_TYPE NVIDIA)
else()
  # set(ENABLE_AI ON CACHE BOOL "开启AI-22.04开启" FORCE) # arm64 22.04开启
  # arm64
  # set(TENSORRT_INCLUDE_DIRS /home/jzw/Libs/TensorRT-10.13.0.35/include)
  # set(TENSORRT_LIBRARY_DIRS /home/jzw/Libs/TensorRT-10.13.0.35/lib)
  # set(CHIP_TYPE NVIDIA)
endif()

link_directories(${TENSORRT_LIBRARY_DIRS})
include_directories(include/)

add_subdirectory(apriltag)
include_directories(./)
add_definitions(${PCL_DEFINITIONS})
# 创建可执行文件
add_executable(apriltag_localization_cpp
  src/apriltag_node.cpp
)

target_include_directories(apriltag_localization_cpp PUBLIC 
      include/ 
      ${EIGEN3_INCLUDE_DIR} 
      ${PCL_INCLUDE_DIRS}
      ${tf2_INCLUDE_DIRS}
      ${tf2_geometry_msgs_INCLUDE_DIRS}
      ${geometry_msgs_INCLUDE_DIRS}
      )
set(PCL_LIBS ${PCL_LIBRARIES})
# ament_target_dependencies(apriltag_localization_cpp
#   rclcpp
#   cv_bridge
#   grap_rgbd # 这里会传递库的依赖信息
#   OpenCV
#   Eigen3
#   trt_infer_lib
#   # message_filters
#   yaml-cpp
#   PCL
#   # vtk
#   VTK
# )
target_link_libraries(apriltag_localization_cpp PUBLIC
  ${Python3_LIBRARY}
  rclcpp::rclcpp
  # GTest::gtest_main
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
  m_common::mlogger
  # deploy_core
  # trt_core
  # detection_6d_foundationpose
  yaml-cpp
  ${PCL_LIBRARIES}
  fmt
  apriltag
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  # ${ament_cmake_python_LIBRARIES}
  # yolo
)
# 安装可执行文件（可选）
install(TARGETS apriltag_localization_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY 
#   ${CMAKE_CURRENT_SOURCE_DIR}/camera_config/
#   DESTINATION camera_config
# )
install(DIRECTORY 
  ${CMAKE_CURRENT_SOURCE_DIR}/config/
  DESTINATION config
)

install(DIRECTORY 
  ${CMAKE_CURRENT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
ament_package()