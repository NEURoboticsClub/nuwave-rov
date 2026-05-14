from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('thruster_pkg')
    params = os.path.join(pkg, 'config', 'general_motor_config.yaml')

    return LaunchDescription([
        Node(
            package='thruster_pkg',
            executable='pwm_node',
            name='pwm_node',
            parameters=[params],
            output='screen'
        )
    ])
