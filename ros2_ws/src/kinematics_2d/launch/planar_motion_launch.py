import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<your_package>',  # Replace with your package name
            executable='ik_solver',
            name='ik_solver',
            output='screen',
            parameters=[{
                'L1': 0.2032,
                'L2': 0.3175
            }]
        ),
        Node(
            package='<your_package>',  # Replace with your package name
            executable='serial_coms',
            name='serial_coms',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600
            }]
        ),
    ])
