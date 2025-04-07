cmake_minimum_required(VERSION 3.8)
project(ee4308_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ee4308_gazebo_plugins REQUIRED)

install(
  DIRECTORY launch rviz params maps urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)


set(ignoreMe "${CMAKE_EXPORT_COMPILE_COMMANDS}")

ament_export_dependencies(gazebo_ros_pkgs)
ament_package()
