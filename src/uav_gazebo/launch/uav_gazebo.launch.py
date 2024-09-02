from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from scripts import GazeboRosPaths

import os

pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
pkg_uav_gazebo = get_package_share_directory('uav_gazebo')

def generate_launch_description():

    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    # print(plugin_path)

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

    # World
    world_path = os.path.join(pkg_uav_gazebo, 'worlds', 'simple.world')
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

    # UAV
    urdf_file_name = 'uav.urdf'
    urdf_file = os.path.join(pkg_uav_gazebo, 'urdf', urdf_file_name)

    spawn_uav = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=['-entity', 'uav', '-file', urdf_file, \
            '-x', '0', '-y', '0', '-z', '0.2'],
        output='screen',
        additional_env=env
    )

    return LaunchDescription([
        world_file,
        gazebo,
        spawn_uav
    ])