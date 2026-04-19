from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess

def generate_launch_description():
    result = subprocess.check_output(['ros2', 'run', 'joy', 'joy_enumerate_devices'])
    result_word1 = result.split(b'\n')[2].split(b':')[-1].lstrip()
    result_word2 = result.split(b'\n')[3].split(b':')[-1].lstrip()
    rov_joy_id = -1
    arm_joy_id = -1
    if result_word1[0] == 80:
        rov_joy_id = 0
    elif result_word1[0] == 88:
        arm_joy_id = 0
    if result_word2[0] == 80:
        rov_joy_id = 1
    elif result_word2[0] == 88:
        arm_joy_id = 1
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