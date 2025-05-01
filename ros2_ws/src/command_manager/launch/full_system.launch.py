from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Start MoveIt demo (includes RViz and planning scene)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('surg_bot_moveit_config'),
                    'launch',
                    'demo.launch.py'
                ])
            ])
        ),

        # Launch camera_02 container first
        ComposableNodeContainer(
            name='camera_02_camera_container',
            namespace='camera_02',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='orbbec_camera',
                    plugin='orbbec_camera::OBCameraNodeDriver',
                    name='camera_02_driver',
                    parameters=[
                        {'serial_number': 'AY3134100C3'},
                        {'enable_depth': True},
                        {'enable_color': True},
                        {'depth_width': 640},
                        {'depth_height': 400},
                        {'depth_fps': 15},
                        {'color_width': 640},
                        {'color_height': 480},
                        {'color_fps': 15},
                    ],
                    remappings=[
                        ('color/image_raw', 'camera_02/color/image_raw'),
                        ('color/image_raw/compressed', 'camera_02/color/image_raw/compressed'),
                        ('depth/image_raw', 'camera_02/depth/image_raw'),
                        ('depth/image_raw/compressed', 'camera_02/depth/image_raw/compressed'),
                        ('color/camera_info', 'camera_02/color/camera_info'),
                        ('depth/camera_info', 'camera_02/depth/camera_info'),
                    ]
                ),
            ],
            output='screen',
        ),

        # Delay before launching camera_01 container
        TimerAction(
            period=2.0,
            actions=[
                ComposableNodeContainer(
                    name='camera_01_camera_container',
                    namespace='camera_01',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='orbbec_camera',
                            plugin='orbbec_camera::OBCameraNodeDriver',
                            name='camera_01_driver',
                            parameters=[
                                {'serial_number': 'AY31341002B'},
                                {'enable_depth': True},
                                {'enable_color': True},
                                {'depth_width': 640},
                                {'depth_height': 400},
                                {'depth_fps': 15},
                                {'color_width': 640},
                                {'color_height': 480},
                                {'color_fps': 15},
                            ],
                            remappings=[
                                ('color/image_raw', 'camera_01/color/image_raw'),
                                ('color/image_raw/compressed', 'camera_01/color/image_raw/compressed'),
                                ('depth/image_raw', 'camera_01/depth/image_raw'),
                                ('depth/image_raw/compressed', 'camera_01/depth/image_raw/compressed'),
                                ('color/camera_info', 'camera_01/color/camera_info'),
                                ('depth/camera_info', 'camera_01/depth/camera_info'),
                            ]
                        ),
                    ],
                    output='screen',
                ),
            ]
        ),

        # Delay before launching remote_tracker_node
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='remote_tracker',
                    executable='remote_tracker_node',
                    name='remote_tracker_node',
                    output='screen',
                ),
            ]
        ),

        # Delay before launching controller, command manager, GUI, and executor
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_coms',
                    executable='coms_node',
                    name='controller_coms',
                    output='screen',
                ),
                Node(
                    package='command_manager',
                    executable='command_manager_node',
                    name='command_manager',
                    output='screen',
                ),
                Node(
                    package='command_manager',
                    executable='gui_node',
                    name='lamp_gui',
                    output='screen',
                ),
                Node(
                    package='goal_pose_executor',
                    executable='goal_pose_executor_node',
                    name='goal_pose_executor',
                    output='screen',
                ),
            ]
        ),
    ])
