# goal_pose_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_pose_calculator',
            executable='goal_pose_calculator',
            name='goal_pose_calculator',
            parameters=[{'offset_distance': 0.3}]
        )
    ])
