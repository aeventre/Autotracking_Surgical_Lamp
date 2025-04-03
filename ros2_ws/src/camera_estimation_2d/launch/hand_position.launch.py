from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_estimation_2d',
            executable='hand_position_publisher',
            name='hand_position_publisher',
            output='screen'
        )
    ])
