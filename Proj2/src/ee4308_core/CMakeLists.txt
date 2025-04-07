cmake_minimum_required(VERSION 3.8)
project(ee4308_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rcutils REQUIRED)
# find_package(eigen3_cmake_module REQUIRED)
# find_package(Eigen3)

install(
  DIRECTORY include/ee4308_core/ 
  DESTINATION include/ee4308_core/)

set(ignoreMe "${CMAKE_EXPORT_COMPILE_COMMANDS}")

ament_export_include_directories(include)
# ament_export_dependencies(eigen3_cmake_module)
# ament_export_dependencies(Eigen3)

ament_package()
