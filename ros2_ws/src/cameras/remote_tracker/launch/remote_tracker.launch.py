from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        # Start camera_02 first
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
                        {'color_height': 400},
                        {'color_fps': 15},
                        {'depth_to_color': True},
                        {'sync_mode': True},
                        {'sync_signal_output': False},
                        {'sync_signal_input': True},  
                        {'use_device_time': True},         # Makes timestamps consistent between cameras
                        {'ir_mirror': False},              # Prevents flipped depth images
                        {'color_mirror': False},           # Prevents flipped RGB images
                    ],
                    remappings=[
                        ('color/image_raw',            'camera_02/color/image_raw'),
                        ('color/image_raw/compressed', 'camera_02/color/image_raw/compressed'),
                        ('depth/image_raw',            'camera_02/depth/image_raw'),
                        ('depth/image_raw/compressed', 'camera_02/depth/image_raw/compressed'),
                        # <— add this so your node can subscribe to the aligned depth
                        ('depth_to_color',             'camera_02/depth_to_color'),
                        ('color/camera_info',          'camera_02/color/camera_info'),
                        ('depth/camera_info',          'camera_02/depth/camera_info'),
                    ]
                ),
            ],
            output='screen',
        ),

        # Then camera_01 after 2 s
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
                                {'color_height': 400},
                                {'color_fps': 15},
                                {'depth_to_color': True},
                                {'sync_mode': True},
                                {'sync_signal_output': True},
                                {'sync_signal_input': False},
                                {'use_device_time': True},         # Makes timestamps consistent between cameras
                                {'ir_mirror': False},              # Prevents flipped depth images
                                {'color_mirror': False},           # Prevents flipped RGB images
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
                    output='screen',
                ),
            ]
        ),

        # Finally the tracker
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='remote_tracker',
                    executable='remote_tracker_node',
                    name='remote_tracker_node',
                    output='screen',
                    # make sure this node is subscribing to:
                    #   – /camera_0X/color/image_raw[/compressed]
                    #   – /camera_0X/depth_to_color
                ),
            ]
        ),
    ])
