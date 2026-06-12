"""Launch the RealSense D455 (color + aligned depth) and the person_distance node.

Set start_camera:=false if the camera is already running (e.g. via rs_slam_bringup).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    start_foxglove = LaunchConfiguration('start_foxglove')
    model = LaunchConfiguration('model')

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        # Set to false when another stack (e.g. rs_slam_bringup) already runs the bridge
        DeclareLaunchArgument('start_foxglove', default_value='true'),
        DeclareLaunchArgument(
            'model',
            default_value=os.path.expanduser('~/SLAM/models/yolo11n-seg.pt')),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            condition=IfCondition(start_camera),
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'rgb_camera.color_profile': '640x480x30',
                'depth_module.depth_profile': '640x480x30',
            }],
            output='screen',
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            condition=IfCondition(start_foxglove),
            parameters=[{'port': 8765}],
            output='screen',
        ),

        Node(
            package='person_distance',
            executable='person_distance_node',
            name='person_distance',
            parameters=[{
                'model': model,
            }],
            output='screen',
        ),
    ])
