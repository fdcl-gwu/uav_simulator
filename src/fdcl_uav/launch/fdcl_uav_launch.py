from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fdcl_uav',
            namespace='fdcl_uav',
            executable='estimator',
            name='sim'
        ),
        Node(
            package='fdcl_uav',
            namespace='fdcl_uav',
            executable='control',
            name='sim'
        )
    ])