from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _create_nodes(context, *args, **kwargs):
    # read launch configs
    count = int(LaunchConfiguration('count').perform(context))
    base_name = LaunchConfiguration('base_name').perform(context)
    i2c_bus = int(LaunchConfiguration('i2c_bus').perform(context))
    i2c_address = int(LaunchConfiguration('i2c_address').perform(context))
    pkg = 'thruster_pkg'

    nodes = []
    for i in range(count):
        name = f"thruster/{base_name}_{i}"
        params = {
            'topic': f'thruster/{base_name}_{i}',
            'i2c_bus': i2c_bus,
            'i2c_address': i2c_address,
            'channel': i,
        }
        nodes.append(
            Node(
                package=pkg,
                executable='new_thruster_node',
                name=name,
                parameters=[params],
                output='screen'
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('count', default_value='8', description='Number of thruster nodes to launch'),
        DeclareLaunchArgument('base_name', default_value='thruster', description='Base name for thruster nodes'),
        DeclareLaunchArgument('i2c_bus', default_value='7', description='I2C bus number for PCA9685'),
        DeclareLaunchArgument('i2c_address', default_value='64', description='I2C address (decimal) for PCA9685'),
        OpaqueFunction(function=_create_nodes)
    ])
