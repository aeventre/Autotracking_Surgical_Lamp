from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os, yaml

# Absolute path to your calibration_data directory
CALIB_DIR = os.path.expanduser(
    '~/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/camera_calibration/calibration_data'
)


def load_camera_transform(filename):
    filepath = os.path.join(CALIB_DIR, filename)
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)
    pos = data['position']
    ori = data['orientation']
    return [str(pos['x']), str(pos['y']), str(pos['z']),
            str(ori['x']), str(ori['y']), str(ori['z']), str(ori['w'])]


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

        # Static TF for camera 01 optical frame
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='cam1_static_tf',
            arguments=cam1_tf + ['base_link', 'camera_01/color_optical_frame'],
            output='screen'
        ),
        # Static TF for camera 02 optical frame
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='cam2_static_tf',
            arguments=cam2_tf + ['base_link', 'camera_02/color_optical_frame'],
            output='screen'
        ),

        # camera_02 driver in composable container
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
                        ('color/camera_info', 'camera_02/color/camera_info'),
                        ('depth/camera_info', 'camera_02/depth/camera_info'),
                    ]
                ),
            ],
        ),

        # Delay before launching camera_01 container
        TimerAction(
            period=2.0,
            actions=[
                ComposableNodeContainer(
                    name='camera_01_camera_container', namespace='camera_01',
                    package='rclcpp_components', executable='component_container', output='screen',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='orbbec_camera', plugin='orbbec_camera::OBCameraNodeDriver',
                            name='camera_01_driver',
                            parameters=[
                                {'serial_number': 'AY31341002B'}, {'enable_depth': True}, {'enable_color': True},
                                {'depth_width': 640}, {'depth_height': 400}, {'depth_fps': 15},
                                {'color_width': 640}, {'color_height': 480}, {'color_fps': 15},
                            ],
                            remappings=[
                                ('color/image_raw', 'camera_01/color/image_raw'),
                                ('color/camera_info', 'camera_01/color/camera_info'),
                                ('depth/camera_info', 'camera_01/depth/camera_info'),
                            ]
                        ),
                    ],
                ),
            ]
        ),

        # Delay before launching remote_tracker_node
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='remote_tracker', executable='remote_tracker_node',
                    name='remote_tracker_node', output='screen'
                ),
            ]
        ),

        # Delay before launching controller, command manager, GUI, and executor
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_coms', executable='coms_node',
                    name='controller_coms', output='screen'
                ),
                Node(
                    package='command_manager', executable='command_manager_node',
                    name='command_manager', output='screen'
                ),
                Node(
                    package='command_manager', executable='gui_node',
                    name='lamp_gui', output='screen'
                ),
                Node(
                    package='goal_pose_executor', executable='goal_pose_executor_node',
                    name='goal_pose_executor', output='screen'
                ),
            ]
        ),
    ])
