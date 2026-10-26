"""
sim.launch.py: drive the simulated NUWave ROV with the real NUWave software.

  ros2 launch mate_rov_sim sim.launch.py                        # everything, gamepads + keyboard -> sim
  ros2 launch mate_rov_sim sim.launch.py topside:=false         # sim + bottomside only (bring your own topside)
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
       web_gui_pkg/web_bridge_node
"""
import os
import re
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def on_wsl():
    try:
        with open('/proc/version') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False


def wsl_gpu_env():
    """
    Point at the GPU on WSL instead of llvmpipe (Use GPU instead of CPU).
    """
    if not on_wsl() or not os.path.exists('/usr/lib/wsl/lib/libd3d12.so'):
        return []
    defaults = {'GALLIUM_DRIVER': 'd3d12', 'MESA_D3D12_DEFAULT_ADAPTER_NAME': 'NVIDIA'}
    return [SetEnvironmentVariable(k, v) for k, v in defaults.items() if k not in os.environ]


def gz_server_cpus(setting):
    """
    CPUs to pin the Gazebo server to (a taskset list), or '' to leave it unpinned.

    Gazebo splits every physics step across threads that wait for each other. Under WSL,
    waking threads on other virtual CPUs is slow: free to use every CPU, the server ran the
    physics ~3x slower than when pinned to 2.
    'auto' pins it to the last two CPUs on WSL. Only the server: the GUI and ROS nodes stay free.
    """
    if setting == 'auto':
        cpus = sorted(os.sched_getaffinity(0))
        return ','.join(str(c) for c in cpus[-2:]) if on_wsl() and len(cpus) >= 4 else ''
    return '' if setting in ('', 'all') else setting


def launch_gazebo(context):
    """Start the Gazebo server and, unless gz_args has -s, the GUI as separate processes."""
    gz_args = shlex.split(LaunchConfiguration('gz_args').perform(context))
    engine = LaunchConfiguration('render_engine').perform(context)
    cpus = gz_server_cpus(LaunchConfiguration('gz_cpus').perform(context))

    server = ['gz', 'sim', '-s', *[a for a in gz_args if a != '-s'],
              '--render-engine', engine, LaunchConfiguration('world').perform(context)]
    actions = [ExecuteProcess(cmd=['taskset', '-c', cpus, *server] if cpus else server,
                              name='gz_server', output='screen')]
    if cpus:
        actions.insert(0, LogInfo(msg=f'Gazebo server pinned to CPUs {cpus} (gz_cpus:=all to disable)'))
    step = LaunchConfiguration('physics_step').perform(context)
    if step:
        # Change the step once the server is up (the world file's own step is 2 ms)
        world = re.search(r'<world\s+name="([^"]+)"', open(server[-1]).read()).group(1)
        call = (f'gz service -s /world/{world}/set_physics --reqtype gz.msgs.Physics '
                f'--reptype gz.msgs.Boolean --timeout 1000 '
                f'--req "max_step_size: {float(step)}, real_time_factor: 1.0"')
        actions.append(ExecuteProcess(
            name='set_physics_step', output='screen', shell=True,
            cmd=[f'for i in $(seq 30); do {call} | grep -q "data: true" && '
                 f'echo "Physics step set to {float(step)} s" && exit 0; sleep 1; done; '
                 f'echo "Could not set the physics step"; exit 1']))
    if '-s' not in gz_args:
        verbosity = gz_args[gz_args.index('-v'):][:2] if '-v' in gz_args else []
        actions.append(ExecuteProcess(
            cmd=['gz', 'sim', '-g', *verbosity, '--render-engine', engine],
            name='gz_gui', output='screen',
            on_exit=Shutdown(reason='Gazebo window closed')))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('mate_rov_sim')
    params = os.path.join(pkg, 'config', 'rov_params.yaml')

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
                        'ogre uses about half the CPU, so it is the default on WSL'),
        DeclareLaunchArgument(
            'physics_step', default_value='',
            description='Physics step in seconds. Empty = the world file\'s 2 ms. 0.004 halves the '
                        'physics CPU cost for slow laptops (IMU samples then jitter +-2 ms around 10 ms)'),
        DeclareLaunchArgument(
            'gz_cpus', default_value='auto',
            description='CPUs to pin the Gazebo server to, e.g. "2,3". auto = last two CPUs on WSL '
                        '(physics runs ~3x faster there), unpinned elsewhere. all = never pin'),
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

    gazebo = OpaqueFunction(function=launch_gazebo)

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml')}],
        output='screen',
    )

    # --- Simulator stand-ins for the hardware drivers ---
    # They run on wall time like the real drivers and every NUWave node. use_sim_time would
    # make each one process /clock at every physics step (500 Hz), which cost more CPU than
    # all their real work combined.
    input_suffix = PythonExpression(
        ["'/response_pwm' if '", LaunchConfiguration('use_pwm_node'), "'.lower() == 'true' else ''"])
    sim_nodes = [
        Node(package='mate_rov_sim', executable='sim_motor_interface', output='screen',
             parameters=[params, {'input_suffix': input_suffix}]),
        Node(package='mate_rov_sim', executable='imu_sim', output='screen',
             parameters=[params]),
        Node(package='mate_rov_sim', executable='depth_sensor_sim', output='screen',
             parameters=[params]),
        Node(package='mate_rov_sim', executable='camera_sim', output='screen',
             parameters=[params]),
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
        Node(package='web_gui_pkg', executable='web_bridge_node', output='screen', condition=topside)
    ]

    return LaunchDescription([
        *args,
        model_path,
        *wsl_gpu_env(),
        gazebo, bridge,
        *sim_nodes,
        *bottomside,
        *topside_nodes,
    ])
