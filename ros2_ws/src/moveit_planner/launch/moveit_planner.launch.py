from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_planner',
            executable='moveit_planner_node',
            name='moveit_planner_node',
            output='screen',
        )
    ])
