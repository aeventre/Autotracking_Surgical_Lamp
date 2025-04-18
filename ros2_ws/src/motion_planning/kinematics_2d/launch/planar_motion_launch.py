import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinematics_2d',  # Replace with your package name
            executable='ik_node_2d',
            name='ik_node_2d',
            output='screen',
            parameters=[{
                'L1': 0.2032,
                'L2': 0.3175
            }]
        ),
        Node(
            package='serial_coms',  # Replace with your package name
            executable='serial_coms_node',
            name='serial_coms',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 9600
            }]
        ),
    ])

# Launch command:
#   ros2 launch kinematics_2d planar_motion_launch.py

# This will publish desired positions to the IK solver:
#   ros2 topic pub /target_position geometry_msgs/msg/Point "{x: 0.2, y: 0.3, z: 0.0}"



