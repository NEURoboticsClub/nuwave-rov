"""
sim.launch.py: drive the simulated NUWave ROV with the real NUWave software.

  ros2 launch mate_rov_sim sim.launch.py                        # everything, gamepads + keyboard -> sim
  ros2 launch mate_rov_sim sim.launch.py topside:=false         # sim + bottomside only (bring your own topside)
  ros2 launch mate_rov_sim sim.launch.py web_gui:=true          # also start the web dashboard
  ros2 launch mate_rov_sim sim.launch.py gz_args:="-r -s"       # headless (no Gazebo GUI)

Starts:
  1. Gazebo with worlds/mate_pool.sdf (the ROV is included by the world)
  2. ros_gz_bridge with config/bridge.yaml (Gazebo topics <-> /sim/...)
  3. Simulator stand-ins for the bottomside hardware drivers:
       sim_motor_interface  replaces the PCA9685 / ESCs / motors behind pwm_node
       imu_sim              replaces imu_pkg/imu_pub            -> /imu
       depth_sensor_sim     replaces rov_depth_sensor           -> /depth/*
       camera_sim           replaces camera_pkg                 -> /camera_N/image/compressed
  4. The real bottomside nodes that don't touch hardware:
       thruster_pkg/pwm_node (simulate:=true)   stabilization_pkg/stabilization_node
  5. The real topside (topside:=true):
       joy_node x2 ----------+
                             +-> joy_mux -> houston_pkg/houston -> controller/thruster_controller_node
       keyboard_teleop ------+                                  -> arm_controller/arm_controller_node
       (keyboard:=true; a held key overrides the gamepad)
  6. web_gui/bridge_node (web_gui:=true)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def on_wsl():
    try:
        with open('/proc/version') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False


def generate_launch_description():
    pkg = get_package_share_directory('mate_rov_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    params = os.path.join(pkg, 'config', 'rov_params.yaml')
    sim_time = {'use_sim_time': True}

    args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg, 'worlds', 'mate_pool.sdf'),
            description='World SDF file to load'),
        DeclareLaunchArgument(
            'gz_args', default_value='-r -v 3',
            description='Extra gz sim flags: -r run on start, -s server only (no GUI), -v verbosity'),
        DeclareLaunchArgument(
            'render_engine', default_value='ogre' if on_wsl() else 'ogre2',
            description='Gazebo render engine for the GUI and the cameras. ogre2 looks better; '
                        'ogre is ~5x faster on WSL, where ogre2 drops the sim far below real time'),
        DeclareLaunchArgument(
            'topside', default_value='true',
            description='Start the joysticks, houston, thruster controller and arm controller'),
        DeclareLaunchArgument(
            'keyboard', default_value='true',
            description='Open the keyboard teleop window (needs a display). A held key overrides the gamepad'),
        DeclareLaunchArgument(
            'use_pwm_node', default_value='true',
            description='Run the real pwm_node (simulate:=true) between the controllers and the sim. '
                        'false = the sim reads the controllers\' duty cycles directly'),
        DeclareLaunchArgument(
            'stabilization', default_value='true',
            description='Start stabilization_pkg/stabilization_node (toggle it from the gamepad as usual)'),
        DeclareLaunchArgument(
            'web_gui', default_value='false',
            description='Start web_gui/bridge_node'),
        DeclareLaunchArgument(
            'joy_thruster_device', default_value='PowerA Xbox Series X Controller',
            description='SDL name of the ROV-driving gamepad (see: ros2 run joy joy_enumerate_devices). '
                        'Empty = use joy_thruster_id'),
        DeclareLaunchArgument('joy_thruster_id', default_value='0',
                              description='Device index used when joy_thruster_device is empty'),
        DeclareLaunchArgument(
            'joy_arm_device', default_value='Xbox 360 Controller',
            description='SDL name of the arm gamepad. Empty = use joy_arm_id'),
        DeclareLaunchArgument('joy_arm_id', default_value='1',
                              description='Device index used when joy_arm_device is empty'),
    ]
    topside = IfCondition(LaunchConfiguration('topside'))

    # Let Gazebo find model://mate_rov, keeping anything already on the path (works on Humble and newer)
    resource_path = os.pathsep.join(
        p for p in [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.path.join(pkg, 'models')] if p)
    model_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            LaunchConfiguration('gz_args'), ' --render-engine ', LaunchConfiguration('render_engine'),
            ' ', LaunchConfiguration('world')]}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}],
        output='screen',
    )

    # --- Simulator stand-ins for the hardware drivers ---
    input_suffix = PythonExpression(
        ["'/response_pwm' if '", LaunchConfiguration('use_pwm_node'), "'.lower() == 'true' else ''"])
    sim_nodes = [
        Node(package='mate_rov_sim', executable='sim_motor_interface', output='screen',
             parameters=[params, sim_time, {'input_suffix': input_suffix}]),
        Node(package='mate_rov_sim', executable='imu_sim', output='screen',
             parameters=[params, sim_time]),
        Node(package='mate_rov_sim', executable='depth_sensor_sim', output='screen',
             parameters=[params, sim_time]),
        Node(package='mate_rov_sim', executable='camera_sim', output='screen',
             parameters=[params, sim_time]),
    ]

    # --- Real NUWave bottomside nodes (no hardware access) ---
    bottomside = [
        Node(package='thruster_pkg', executable='pwm_node', output='screen',
             parameters=[{'simulate': True}],
             condition=IfCondition(LaunchConfiguration('use_pwm_node'))),
        Node(package='stabilization_pkg', executable='stabilization_node', output='screen',
             condition=IfCondition(LaunchConfiguration('stabilization'))),
    ]

    # --- Real NUWave topside: same nodes and remaps as houston_pkg/joystick_launch.launch.py ---
    def joy(name, device_arg, id_arg):
        return Node(
            package='joy', executable='joy_node', name=name, output='screen',
            remappings=[('/joy', '/' + name)],
            parameters=[{
                'device_name': LaunchConfiguration(device_arg),
                'device_id': LaunchConfiguration(id_arg),
            }],
            condition=topside,
        )

    # Keyboard and gamepads share houston through joy_mux: a held key overrides the gamepad,
    # otherwise the gamepad passes through untouched. houston reads the mux output.
    keyboard = IfCondition(PythonExpression(
        ["'", LaunchConfiguration('topside'), "'.lower() == 'true' and '",
         LaunchConfiguration('keyboard'), "'.lower() == 'true'"]))
    topside_nodes = [
        joy('joy_thruster', 'joy_thruster_device', 'joy_thruster_id'),
        joy('joy_arm', 'joy_arm_device', 'joy_arm_id'),
        Node(package='mate_rov_sim', executable='joy_mux', output='screen', condition=topside),
        Node(package='mate_rov_sim', executable='keyboard_teleop', output='screen', condition=keyboard),
        Node(package='houston_pkg', executable='houston_node', output='screen', condition=topside,
             parameters=[{'joy_thruster': '/teleop/joy_thruster', 'joy_arm': '/teleop/joy_arm'}]),
        Node(package='thruster_controller_pkg', executable='thruster_controller_node', output='screen', condition=topside),
        Node(package='arm_controller_pkg', executable='arm_controller_node', output='screen', condition=topside),
    ]

    web_gui = Node(package='web_gui_pkg', executable='bridge_node', output='screen',
                   condition=IfCondition(LaunchConfiguration('web_gui')))

    return LaunchDescription([
        *args,
        model_path,
        gazebo, bridge,
        *sim_nodes,
        *bottomside,
        *topside_nodes,
        web_gui,
    ])
