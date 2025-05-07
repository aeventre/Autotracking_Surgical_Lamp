from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    moveit_config_dir = get_package_share_directory('surg_bot_moveit_config')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_dir, 'launch', 'demo.launch.py')
            )
        ),
        Node(
            package='goal_pose_executor',
            executable='goal_pose_executor_node',
            name='goal_pose_executor',
            output='screen',
        )
    ])
