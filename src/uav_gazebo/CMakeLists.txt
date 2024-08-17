cmake_minimum_required(VERSION 3.22.1)
project(uav_gazebo)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)

xacro_add_files("urdf/uav.urdf.xacro" INSTALL DESTINATION urdf/)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DesiredData.msg"
  "msg/ErrorData.msg"
  "msg/StateData.msg"
  DEPENDENCIES std_msgs
 )

install(DIRECTORY launch meshes worlds msg
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()