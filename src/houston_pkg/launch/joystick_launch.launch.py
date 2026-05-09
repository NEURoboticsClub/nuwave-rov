import subprocess
import sys

from launch import LaunchDescription
from launch_ros.actions import Node


def detect_joysticks():
    rov_joy_id = -1
    arm_joy_id = -1

    try:
        result = subprocess.check_output(
            ['ros2', 'run', 'joy', 'joy_enumerate_devices'],
            timeout=5,
            stderr=subprocess.STDOUT,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired, subprocess.CalledProcessError) as e:
        print(f"[joystick_launch] enumerate failed: {e}", file=sys.stderr)
        return rov_joy_id, arm_joy_id

    # Skip the 2 header lines, take up to 2 device rows
    for device_id, line in enumerate(result.split(b'\n')[2:4]):
        parts = line.split(b':')
        if len(parts) < 2:
            continue
        name = parts[-1].strip()
        first = name[:1]  # bytes slice — b'' if empty, never IndexError
        if first == b'P':
            rov_joy_id = device_id
        elif first == b'X':
            arm_joy_id = device_id

    if rov_joy_id == -1:
        print("[joystick_launch] WARNING: ROV (PowerA) joystick not detected.", file=sys.stderr)
    if arm_joy_id == -1:
        print("[joystick_launch] WARNING: arm (X360) joystick not detected.", file=sys.stderr)

    return rov_joy_id, arm_joy_id


def generate_launch_description():
    rov_joy_id, arm_joy_id = detect_joysticks()
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_thruster',
            remappings=[
                ('/joy', '/joy_thruster')
            ],
            parameters=[{
                # 'device_name': 'PowerA Xbox Series X Controller',
                'device_id': rov_joy_id
            }]

        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_arm',
            remappings=[
                ('/joy', '/joy_arm')
            ],
            parameters=[{
                # 'device_name': 'X360 Controller',
                'device_id': arm_joy_id
            }]
        ),
    ])