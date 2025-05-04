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
    return [
        str(pos['x']), str(pos['y']), str(pos['z']),
        str(ori['x']), str(ori['y']), str(ori['z']), str(ori['w'])
    ]

def generate_launch_description():
    cam1_tf = load_camera_transform('camera_01_poses.yaml')
    cam2_tf = load_camera_transform('camera_02_poses.yaml')

    return LaunchDescription([
        # MoveIt demo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('surg_bot_moveit_config'),
                    'launch', 'demo.launch.py'
                ])
            )
        ),

        # Static TFs
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='cam1_static_tf',
            arguments=cam1_tf + ['base_link', 'camera_01/color_optical_frame'],
            output='screen'
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='cam2_static_tf',
            arguments=cam2_tf + ['base_link', 'camera_02/color_optical_frame'],
            output='screen'
        ),

        # Camera 02
        ComposableNodeContainer(
            name='camera_02_camera_container', namespace='camera_02',
            package='rclcpp_components', executable='component_container', output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='orbbec_camera', plugin='orbbec_camera::OBCameraNodeDriver',
                    name='camera_02_driver',
                    parameters=[
                        {'serial_number': 'AY3134100C3'},  # Restored original CAM2 ID
                        {'enable_depth': True}, {'enable_color': True},
                        {'depth_width': 640}, {'depth_height': 400}, {'depth_fps': 15},
                        {'color_width': 640}, {'color_height': 480}, {'color_fps': 15},
                        {'depth_to_color': True},
                        {'sync_mode': 0},  # <- Force standalone mode
                    ],
                    remappings=[
                        ('color/image_raw',            'camera_02/color/image_raw'),
                        ('color/image_raw/compressed', 'camera_02/color/image_raw/compressed'),
                        ('depth/image_raw',            'camera_02/depth/image_raw'),
                        ('depth/image_raw/compressed', 'camera_02/depth/image_raw/compressed'),
                        ('depth_to_color',             'camera_02/depth_to_color'),
                        ('color/camera_info',          'camera_02/color/camera_info'),
                        ('depth/camera_info',          'camera_02/depth/camera_info'),
                    ]
                ),
            ],
        ),

        # Delay then camera 01
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
                                {'serial_number': 'AY31341002B'},  # Restored original CAM1 ID
                                {'enable_depth': True}, {'enable_color': True},
                                {'depth_width': 640}, {'depth_height': 400}, {'depth_fps': 15},
                                {'color_width': 640}, {'color_height': 480}, {'color_fps': 15},
                                {'depth_to_color': True},
                                {'use_device_time': True},
                                {'sync_mode': 0},  # <- Force standalone mode
                            ],
                            remappings=[
                                ('color/image_raw',            'camera_01/color/image_raw'),
                                ('color/image_raw/compressed', 'camera_01/color/image_raw/compressed'),
                                ('depth/image_raw',            'camera_01/depth/image_raw'),
                                ('depth/image_raw/compressed', 'camera_01/depth/image_raw/compressed'),
                                ('depth_to_color',             'camera_01/depth_to_color'),
                                ('color/camera_info',          'camera_01/color/camera_info'),
                                ('depth/camera_info',          'camera_01/depth/camera_info'),
                            ]
                        ),
                    ],
                ),
            ]
        ),

        # Tracker
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='remote_tracker',
                    executable='remote_tracker_node',
                    name='remote_tracker_node',
                    output='screen',
                    parameters=[
                        {'hsv_lower': [38, 101, 102]},
                        {'hsv_upper': [74, 189, 248]},
                        {'reprojection_threshold': 10.0},
                    ]
                ),
            ]
        ),

        # Controller / GUI / Manager / Executor
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='controller_coms',
                    executable='coms_node',
                    name='controller_coms',
                    output='screen'
                ),
                Node(
                    package='command_manager',
                    executable='command_manager_node',
                    name='command_manager',
                    output='screen'
                ),
                Node(
                    package='command_manager',
                    executable='gui_node',
                    name='lamp_gui',
                    output='screen'
                ),
                Node(
                    package='goal_pose_executor',
                    executable='goal_pose_executor_node',
                    name='goal_pose_executor',
                    output='screen'
                ),
            ]
        ),
    ])
