from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # --- Start camera_02 (SLAVE) FIRST ---
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
                        {'depth_width': 1280},
                        {'depth_height': 800},
                        {'depth_fps': 30},
                        {'color_width': 1280},
                        {'color_height': 720},
                        {'color_fps': 30},
                        {'sync_mode': True},
                        {'sync_signal_output': False},
                        {'sync_signal_input': True},
                    ],
                    remappings=[
                        ('color/image_raw', 'camera_02/color/image_raw'),
                        ('depth/image_raw', 'camera_02/depth/image_raw'),
                        ('color/camera_info', 'camera_02/color/camera_info'),
                        ('depth/camera_info', 'camera_02/depth/camera_info'),
                    ]
                ),
            ],
            output='screen',
        ),

        # Calibration for camera_02
        Node(
            package='camera_calibration',
            executable='camera_calibration_node',
            name='camera_calibrator_node_camera_02',
            namespace='camera_02',
            output='screen',
            parameters=[
                {'marker_length': 0.045},
                {'camera_frame': 'camera_02_link'},
                {'camera_poses_yaml': '/home/alec/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/camera_calibration/calibration_data/camera_02_poses.yaml'},
            ],
            remappings=[
                ('image_raw', '/camera_02/color/image_raw'),
                ('camera_info', '/camera_02/color/camera_info'),
            ]
        ),

        # --- Delay before launching camera_01 (MASTER) ---
        TimerAction(
            period=5.0,  # Delay to ensure slave is ready
            actions=[
                # Start camera_01 (MASTER)
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
                                {'depth_width': 1280},
                                {'depth_height': 800},
                                {'depth_fps': 30},
                                {'color_width': 1280},
                                {'color_height': 720},
                                {'color_fps': 30},
                                {'sync_mode': True},
                                {'sync_signal_output': True},
                                {'sync_signal_input': False},
                            ],
                            remappings=[
                                ('color/image_raw', 'camera_01/color/image_raw'),
                                ('depth/image_raw', 'camera_01/depth/image_raw'),
                                ('color/camera_info', 'camera_01/color/camera_info'),
                                ('depth/camera_info', 'camera_01/depth/camera_info'),
                            ]
                        ),
                    ],
                    output='screen',
                ),

                # Calibration for camera_01
                Node(
                    package='camera_calibration',
                    executable='camera_calibration_node',
                    name='camera_calibrator_node_camera_01',
                    namespace='camera_01',
                    output='screen',
                    parameters=[
                        {'marker_length': 0.045},
                        {'camera_frame': 'camera_01_link'},
                        {'camera_poses_yaml': '/home/alec/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/camera_calibration/calibration_data/camera_01_poses.yaml'},
                    ],
                    remappings=[
                        ('image_raw', '/camera_01/color/image_raw'),
                        ('camera_info', '/camera_01/color/camera_info'),
                    ]
                ),
            ]
        ),
    ])
