cmake_minimum_required(VERSION 3.22.1)
project(uav_control_plugin)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(uav_control_plugin SHARED 
    src/uav_control_plugin.cpp
    src/fdcl_matrix_utils.cpp
    src/fdcl_ros_utils.cpp
)

target_include_directories(uav_control_plugin PUBLIC
  # ./include
  ../../libraries/eigen
  ${GAZEBO_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(uav_control_plugin
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  uav_control_plugin
  DESTINATION share/${PROJECT_NAME}
)

ament_package()