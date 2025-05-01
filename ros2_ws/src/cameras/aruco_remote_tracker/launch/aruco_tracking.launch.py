from launch import LaunchDescription
from launch.actions import TimerAction, GroupAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        # Start camera_01 container FIRST
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

        # Camera 01 ArUco detector node
        Node(
            package='aruco_remote_tracker',
            executable='marker_pose_estimator_node',
            name='marker_pose_estimator_camera_01',
            namespace='camera_01',
            output='screen',
            parameters=[
                {'camera_frame': 'camera_01_link'},
                {'image_topic': '/camera_01/color/image_raw'},
                {'camera_info_topic': '/camera_01/color/camera_info'},
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                GroupAction([
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
                                    {'depth_fps': 30},
                                    {'color_width': 640},
                                    {'color_height': 360},
                                    {'color_fps': 30},
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

                    Node(
                        package='aruco_remote_tracker',
                        executable='marker_pose_estimator_node',
                        name='marker_pose_estimator_camera_02',
                        namespace='camera_02',
                        output='screen',
                        parameters=[
                            {'camera_frame': 'camera_02_link'},
                            {'image_topic': '/camera_02/color/image_raw'},
                            {'camera_info_topic': '/camera_02/color/camera_info'},
                        ],
                    ),
                ])
            ]
        )
    ])
