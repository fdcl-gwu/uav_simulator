from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fdcl_uav',
            namespace='estimator',
            executable='estimator',
            name='sim'
        ),
        Node(
            package='fdcl_uav',
            namespace='control',
            executable='control',
            name='sim'
        ),
        Node(
            package='fdcl_uav',
            namespace='gui',
            executable='gui',
            name='sim'
        )
    ])