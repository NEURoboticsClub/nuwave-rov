import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _create_nodes(context, *args, **kwargs):
    config_file = LaunchConfiguration('config').perform(context)
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    monitors = config.get('power_monitor', [])
    pkg = 'power_monitor_pkg'
    nodes = []

    for monitor in monitors:
        monitor_id = monitor['id']

        params = {
            'topic': f'power_monitor/monitor_{monitor_id}',
            'i2c_address': monitor['i2c_address'],
            'i2c_bus': monitor['i2c_bus'],

        }
        nodes.append(
            Node(
                package=pkg,
                executable='power_monitor_node',
                name=f'power_monitor_{monitor_id}',
                parameters=[params],
                output='screen'
            )
        )
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('power_monitor_pkg')
    default_config = os.path.join(pkg_share, 'config', 'power_monitor_run_config.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to power monitor YAML config file'),
        OpaqueFunction(function=_create_nodes),
    ])
