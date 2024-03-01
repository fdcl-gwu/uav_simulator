from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
pkg_uav_gazebo = get_package_share_directory('uav_gazebo')

def generate_launch_description():

    urdf_file_name = 'uav.xacro'
    urdf_file = os.path.join(pkg_uav_gazebo, 'urdf', urdf_file_name)
    print(urdf_file)

    declare_urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_uav_gazebo, 'urdf', urdf_file_name),
        description='urdf/' + urdf_file_name
    )

    world_path = os.path.join(pkg_uav_gazebo, 'worlds', 'simple.world')

    # World
    world_file = DeclareLaunchArgument(
        'world',
        default_value=[world_path],
        description='SDF world file'
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    ) 

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #     respawn=False,
    #     arguments=['-urdf', "-model", "uav", "-param", urdf],
    #     output='screen')

    spawn_uav = Node(package='gazebo_ros', executable='spawn_entity.py', 
        arguments=['-entity', 'uav', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '1'],
        output='screen')

    # spawn_uav = Node(
    #     package='gazebo_ros',
    #     executable='create',
    #     output='screen',
    #     arguments=["-file", urdf]
    # )   

    return LaunchDescription([
        declare_urdf_file_arg,
        world_file,
        gazebo,
        spawn_uav
        # world_argument,
        # verbose_argument,
        # gazebo_gui,
        # include_gazebo
    ])