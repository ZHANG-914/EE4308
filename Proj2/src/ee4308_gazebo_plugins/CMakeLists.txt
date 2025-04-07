cmake_minimum_required(VERSION 3.8)
project(ee4308_gazebo_plugins) #adapted from hector_gazebo_plugins

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

## gazebo
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Set includes
set(INCLUDE_DIRS include 
    ${sensor_msgs_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/srv
)
include_directories(${INCLUDE_DIRS})


# for plugin
set(LIBS
    gazebo_ros
    geometry_msgs
    sensor_msgs
    rclcpp
    std_msgs
    tf2
)

set(GAZEBO_ROS_PLUGINS_LIST "")

add_library(gazebo_ros_baro SHARED src/gazebo_ros_baro.cpp)
target_link_libraries(gazebo_ros_baro ${GAZEBO_LIBRARIES})
list(APPEND GAZEBO_ROS_PLUGINS_LIST gazebo_ros_baro)
ament_target_dependencies(gazebo_ros_baro ${LIBS})

add_library(gazebo_ros_magnetic SHARED src/gazebo_ros_magnetic.cpp)
target_link_libraries(gazebo_ros_magnetic ${GAZEBO_LIBRARIES})
list(APPEND GAZEBO_ROS_PLUGINS_LIST gazebo_ros_magnetic)
ament_target_dependencies(gazebo_ros_magnetic ${LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${GAZEBO_ROS_PLUGINS_LIST}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# ament_export_dependencies(gazebo_ros_pkgs)
ament_package()
