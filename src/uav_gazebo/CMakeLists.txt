cmake_minimum_required(VERSION 2.8.3)
project(uav_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

include_directories(
    #include 
    ${catkin_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDFormat_INCLUDE_DIRS}
)

# Build whatever you need here
# add_library(...) # TODO

catkin_package(
    DEPENDS
    gazebo_ros
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)
