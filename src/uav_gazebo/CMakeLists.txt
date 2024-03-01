cmake_minimum_required(VERSION 3.22.1)
project(uav_gazebo)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)

ament_export_include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDFormat_INCLUDE_DIRS}
)

ament_export_dependencies(
    gazebo_ros
)

install(DIRECTORY
  launch
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()