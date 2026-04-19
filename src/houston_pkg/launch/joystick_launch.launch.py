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
                'device_name': 'PowerA Xbox Series X Controller',
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
                # 'device_name': 'X360 Controller',
                # 'device_id': 1
            }]
        ),
    ])