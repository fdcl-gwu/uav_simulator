cmake_minimum_required(VERSION 3.22.1)
project(fdcl_uav_plugins)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

add_library(control_plugin SHARED 
    src/control_plugin.cpp
    src/fdcl_matrix_utils.cpp
    src/fdcl_ros_utils.cpp
)

target_include_directories(control_plugin PUBLIC
  ./include
  ../../libraries/eigen
  ${GAZEBO_INCLUDE_DIRS}
)


ament_target_dependencies(control_plugin
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  control_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()