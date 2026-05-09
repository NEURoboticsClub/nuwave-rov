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
    except FileNotFoundError:
        print("[joystick_launch] 'ros2' not on PATH; not sourced", file=sys.stderr)
        return rov_joy_id, arm_joy_id
    except subprocess.TimeoutExpired:
        print("[joystick_launch] joy_enumerate_devices timed out after 5s", file=sys.stderr)
        return rov_joy_id, arm_joy_id
    except subprocess.CalledProcessError as e:
        print(f"[joystick_launch] joy_enumerate_devices failed (exit {e.returncode}): {e.output!r}", file=sys.stderr)
        return rov_joy_id, arm_joy_id

    lines = result.split(b'\n')

    # Device lines start at index 2 (header lines first)
    #   of the form "Device 0: PowerA Xbox Series X Controller"
    for device_id, line in enumerate(lines[2:4]):
        parts = line.split(b':')
        name = parts[-1].lstrip() if len(parts) > 1 else b''
        if not name:
            continue
        first = name[0]
        if first == ord('P'):       # PowerA: ROV thrusters
            rov_joy_id = device_id
        elif first == ord('X'):     # X360: arm
            arm_joy_id = device_id
        else:
            print(f"[joystick_launch] Unrecognized device on slot {device_id}: {name!r}", file=sys.stderr)

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