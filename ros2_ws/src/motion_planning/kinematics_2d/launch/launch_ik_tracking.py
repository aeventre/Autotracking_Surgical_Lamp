from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_estimation_2d',
            executable='hand_position_publisher',
            name='hand_position_publisher',
            output='screen'
        ),
        Node(
            package='kinematics_2d',
            executable='ik_node_2d',
            name='ik_node_2d',
            output='screen',
            parameters=[
                {'L1': 0.2032},
                {'L2': 0.3175}
            ]
        ),
        Node(
            package='controller_coms',
            executable='coms_node',
            name='controller_coms',
            output='screen'
        )
    ])
