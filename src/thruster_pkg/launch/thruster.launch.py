from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('thruster_pkg')
    params = os.path.join(pkg, 'config', 'thruster_run_config.yaml')

    return LaunchDescription([
        Node(
            package='thruster_pkg',
            executable='thruster_node',
            name='thruster_node',
            parameters=[params],
            output='screen'
        )
    ])
