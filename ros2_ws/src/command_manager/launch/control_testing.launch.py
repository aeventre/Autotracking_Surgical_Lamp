from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Start MoveIt demo (includes RViz and planning scene)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("surg_bot_moveit_config"),
                    "launch",
                    "demo.launch.py"
                ])
            ])
        ),

        # Command Manager (core logic)
        Node(
            package='command_manager',
            executable='command_manager_node',
            name='command_manager',
            output='screen',
        ),

        # GUI (PyQt5 frontend)
        Node(
            package='command_manager',
            executable='gui_node',
            name='lamp_gui',
            output='screen',
        ),

        # Goal Pose Executor (planning + joint publishing)
        Node(
            package='goal_pose_executor',
            executable='goal_pose_executor_node',
            name='goal_pose_executor',
            output='screen',
        ),

        # Goal Pose Computer (computes pose from vision or remote)
        Node(
            package='goal_pose_computer',
            executable='goal_pose_computer',  # <-- THIS is the correct name
            name='goal_pose_computer',
            output='screen',
        ),

    ])
