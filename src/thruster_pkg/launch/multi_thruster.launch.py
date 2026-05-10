from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(config_path: str) -> dict:
    if not os.path.exists(config_path):
        print(f"[thruster_pkg launch] Config file not found: {config_path}")
        return {}
    try:
        with open(config_path, 'r') as f:
            return yaml.safe_load(f) or {}
    except yaml.YAMLError as e:
        print(f"[thruster_pkg launch] Failed to parse {config_path}: {e}")
        return {}

def _create_nodes(context, *args, **kwargs):
    # read launch configs
    count = int(LaunchConfiguration('count').perform(context))
    base_name = LaunchConfiguration('base_name').perform(context)
    i2c_bus = int(LaunchConfiguration('i2c_bus').perform(context))
    i2c_address = int(LaunchConfiguration('i2c_address').perform(context))

    simulate = LaunchConfiguration('simulate').perform(context).lower() in ('true', '1', 'yes')

    pkg = get_package_share_directory('thruster_pkg')

    config_channel_path = os.path.join(pkg, 'config', 'thruster_channel_config.yaml')
    config_run_path = os.path.join(pkg, 'config', 'thruster_run_config.yaml')

    config = load_yaml(config_channel_path)
    run_config = load_yaml(config_run_path)

    thrusters = config.get('thrusters', [])
    # Temp Fix, ideally we have a nicer config situation
    slew_rate_us_per_s = run_config.get('thruster_node', []).get('ros__parameters', []).get('slew_us_per_s')
    if (count > len(thrusters)):
        print(f"[thruster_pkg launch] Warning: Requested {count} thrusters, but only {len(thrusters)} defined in config. Launching {len(thrusters)} thruster nodes.")
        count = len(thrusters)

    nodes = []
    for i in range(count):
        channel = thrusters[i].get('channel')
        name = f"{base_name}_{i}"
        params = {
            'topic': f'/thruster/{base_name}_{i}',
            'i2c_bus': i2c_bus,
            'i2c_address': i2c_address,
            'channel': channel,
            'simulate': simulate,
            'slew_rate_us_per_s' : slew_rate_us_per_s,
        }
        nodes.append(
            Node(
                package='thruster_pkg',
                executable='thruster_node',
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
        DeclareLaunchArgument('simulate', default_value='false', description='If true, skip PCA9685 init'),
        OpaqueFunction(function=_create_nodes)
    ])
