from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='2d_camera_estimation',
            executable='hand_position_publisher',
            name='hand_position_publisher',
            output='screen'
        ),
        Node(
            package='ik_solver_2d',
            executable='ik_solver_2d',
            name='ik_solver_2d',
            output='screen',
            parameters=[
                {'L1': 0.2032},
                {'L2': 0.3175}
            ]
        ),
        Node(
            package='controller_coms',
            executable='controller_coms_node',
            name='controller_coms',
            output='screen'
        )
    ])
