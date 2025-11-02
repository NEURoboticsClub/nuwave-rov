from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_publisher',
            parameters=['config/gscam_params.yaml'],
            output='screen'
        )
    ])