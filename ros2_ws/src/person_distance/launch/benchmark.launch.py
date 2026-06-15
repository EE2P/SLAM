"""Bring up the RealSense (D435i/D455) and the perception benchmark node.

Runs the SAME camera configuration as person_distance.launch.py (colour +
aligned depth, 640x480x30) so the measured FPS reflects real deployment load,
but launches the benchmark node instead of person_distance so nothing else
competes for the GPU.

    ros2 launch person_distance benchmark.launch.py
    ros2 launch person_distance benchmark.launch.py duration_s:=30.0 \
        csv_path:=~/SLAM/bags/perception_benchmark.csv
    ros2 launch person_distance benchmark.launch.py start_camera:=false   # camera already up
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_model():
    # Prefer the TensorRT engine when it has been exported on this machine
    # (this is what the deployed node uses, so the benchmark matches it).
    engine = os.path.expanduser('~/SLAM/models/yolo11n-seg.engine')
    return engine if os.path.exists(engine) else os.path.expanduser(
        '~/SLAM/models/yolo11n-seg.pt')


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    model = LaunchConfiguration('model')
    duration_s = LaunchConfiguration('duration_s')
    csv_path = LaunchConfiguration('csv_path')

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('model', default_value=_default_model()),
        DeclareLaunchArgument('duration_s', default_value='0.0',
                              description='0 = run until Ctrl-C'),
        DeclareLaunchArgument('csv_path', default_value='',
                              description='optional per-frame CSV output path'),

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
            package='person_distance',
            executable='perception_benchmark',
            name='perception_benchmark',
            parameters=[{
                'model': model,
                'duration_s': duration_s,
                'csv_path': csv_path,
            }],
            output='screen',
        ),
    ])
