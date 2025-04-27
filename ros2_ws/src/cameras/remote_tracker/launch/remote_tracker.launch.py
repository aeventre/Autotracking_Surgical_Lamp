from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Start camera_02 container FIRST (no delay)
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

        # Delay before launching camera_01
        TimerAction(
            period=2.0,  # wait 2 seconds to start camera 1
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

        # Further delay before launching remote_tracker_node
        TimerAction(
            period=5.0,  # wait a total of 5 seconds before starting tracker
            actions=[
                Node(
                    package='remote_tracker',
                    executable='remote_tracker_node',
                    name='remote_tracker_node',
                    output='screen',
                ),
            ]
        ),
    ])
