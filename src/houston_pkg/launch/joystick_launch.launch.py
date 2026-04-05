from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_thruster',
            remappings=[
                ('/joy', '/joy_thruster')
            ],
            parameters=[{
                'dev': '/dev/input/by-id/usb-PowerA_Xbox_Series_X_EnWired_Controller_Blue_inline_0000010A899E80B3-joystick',
                # 'device_id': 0
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
                'dev': '/dev/input/by-id/usb-©Microsoft_Corporation_Controller_86231B39-joystick',
                # 'device_id': 1
            }]
        ),
    ])