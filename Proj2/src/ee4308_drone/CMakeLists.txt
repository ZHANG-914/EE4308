cmake_minimum_required(VERSION 3.8)
project(ee4308_drone)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(ee4308_core)

set (DEPENDENCIES
  "rclcpp"
  "rcutils"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "Eigen3"
  "ee4308_core"
)

######## TELEOP ########
add_executable(teleop src/teleop.cpp)
target_include_directories(teleop PUBLIC include)
ament_target_dependencies(teleop ${DEPENDENCIES})
target_compile_features(teleop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS teleop DESTINATION lib/${PROJECT_NAME})

######### BEHAVIOR ########
add_executable(behavior src/behavior.cpp)
target_include_directories(behavior PUBLIC include)
ament_target_dependencies(behavior ${DEPENDENCIES})
target_compile_features(behavior PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS behavior DESTINATION lib/${PROJECT_NAME})

######## CONTROLLER ########
add_executable(controller src/controller.cpp)
target_include_directories(controller PUBLIC include)
ament_target_dependencies(controller ${DEPENDENCIES})
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS controller DESTINATION lib/${PROJECT_NAME})

######## ESTIMATOR ########
add_executable(estimator src/estimator.cpp)
target_include_directories(estimator PUBLIC include)
ament_target_dependencies(estimator ${DEPENDENCIES})
target_compile_features(estimator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS estimator DESTINATION lib/${PROJECT_NAME})

######## PLANNER ########
add_executable(planner src/planner.cpp)
target_include_directories(planner PUBLIC include)
ament_target_dependencies(planner ${DEPENDENCIES})
target_compile_features(planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS planner DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
