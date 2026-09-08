cmake_minimum_required(VERSION 3.8)
project(usv_transforms)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

install(
  FILES resource/usv_transforms
  DESTINATION share/ament_index/resource_index/packages
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
