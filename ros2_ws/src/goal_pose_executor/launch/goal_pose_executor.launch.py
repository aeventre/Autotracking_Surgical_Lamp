from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='goal_pose_executor',
            executable='goal_pose_executor_node',
            name='goal_pose_executor',
            output='screen',
        )
    ])
