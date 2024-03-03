cmake_minimum_required(VERSION 3.22.1)
project(uav_gazebo)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files("urdf/uav.urdf.xacro" INSTALL DESTINATION urdf/)

# ament_export_include_directories(
#     # ${catkin_INCLUDE_DIRS} 
#     ${GAZEBO_INCLUDE_DIRS} 
#     # ${SDFormat_INCLUDE_DIRS}
# )

# ament_export_dependencies(
#     gazebo_ros
# )

install(DIRECTORY launch meshes worlds
  # urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()