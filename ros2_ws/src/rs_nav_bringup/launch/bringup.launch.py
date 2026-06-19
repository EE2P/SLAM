"""Top-level RealSense-only bringup: RGB-D SLAM (+ optional Nav2, follow, foxglove).

This is the fallback for when the MID-360 LiDAR is unavailable. Defaults bring up
SLAM only; add start_nav:=true for navigation.

Examples:
  ros2 launch rs_nav_bringup bringup.launch.py                                  # SLAM only
  ros2 launch rs_nav_bringup bringup.launch.py start_nav:=true                  # + Nav2
  ros2 launch rs_nav_bringup bringup.launch.py start_nav:=true start_follow:=true
  ros2 launch rs_nav_bringup bringup.launch.py camera_model:=d435i
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rs_nav_bringup')

    def inc(name):
        return PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', name))

    camera_model = LaunchConfiguration('camera_model')
    start_nav = LaunchConfiguration('start_nav')
    start_follow = LaunchConfiguration('start_follow')
    start_foxglove = LaunchConfiguration('start_foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='d455'),
        DeclareLaunchArgument('start_nav', default_value='false'),
        DeclareLaunchArgument('start_follow', default_value='false'),
        DeclareLaunchArgument('start_foxglove', default_value='true'),

        # RGB-D SLAM (camera + RTAB-Map + base_link static TF).
        IncludeLaunchDescription(
            inc('rs_slam.launch.py'),
            launch_arguments={'camera_model': camera_model}.items(),
        ),

        # Nav2 + costmaps.
        IncludeLaunchDescription(inc('nav.launch.py'),
                                 condition=IfCondition(start_nav)),

        # Person follow: YOLO + person->goal node (needs start_nav).
        Node(package='person_distance', executable='person_distance_node',
             name='person_distance', output='screen',
             condition=IfCondition(start_follow)),
        Node(package='person_distance', executable='person_goal_node',
             name='person_goal', output='screen',
             condition=IfCondition(start_follow)),

        # Foxglove bridge.
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='foxglove_bridge', parameters=[{'port': 8765}],
             output='screen', condition=IfCondition(start_foxglove)),
    ])
