import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    config_file_path = (
        '/home/alec/Documents/Autotracking_Surgical_Lamp/ros2_ws/src/cameras/OrbbecSDK_ROS2/orbbec_camera/config/depthfilter/Gemini2_v1.7.json'
    )

    return LaunchDescription([
        Node(
            package='orbbec_camera',
            executable='orbbec_camera_node',
            namespace='camera2',
            name='camera2',
            output='screen',
            parameters=[{
                'camera_name': 'camera2',
                'usb_port': '2-3',         # secondary camera
                'device_num': 2,
                'sync_mode': 'secondary',
                'use_config_file': True,   # Make sure it's True
                'config_file_path': config_file_path,
                'enable_depth': True,      # <-- ENABLE depth even if not needed
                'enable_color': True,
                'enable_ir': True          # <-- ENABLE IR even if not needed
            }]
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='orbbec_camera',
                    executable='orbbec_camera_node',
                    namespace='camera1',
                    name='camera1',
                    output='screen',
                    parameters=[{
                        'camera_name': 'camera1',
                        'usb_port': '2-1',     # primary camera
                        'device_num': 2,
                        'sync_mode': 'primary',
                        'use_config_file': True,
                        'config_file_path': config_file_path,
                        'enable_depth': True,
                        'enable_color': True,
                        'enable_ir': True
                    }]
                )
            ]
        )
    ])
