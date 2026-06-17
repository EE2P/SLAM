"""Top-level bringup: LiDAR SLAM (+ optional camera, navigation, follow, foxglove).

Layered so you can enable pieces as you progress through P0-P6. Defaults bring up
the SLAM core only.

Examples:
  ros2 launch lidar_nav_bringup bringup.launch.py                       # SLAM only
  ros2 launch lidar_nav_bringup bringup.launch.py start_nav:=true       # + Nav2
  ros2 launch lidar_nav_bringup bringup.launch.py start_nav:=true start_camera:=true start_follow:=true
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
    pkg = get_package_share_directory('lidar_nav_bringup')

    def inc(name):
        return PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', name))

    start_camera = LaunchConfiguration('start_camera')
    start_nav = LaunchConfiguration('start_nav')
    start_follow = LaunchConfiguration('start_follow')
    start_foxglove = LaunchConfiguration('start_foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('start_nav', default_value='false'),
        DeclareLaunchArgument('start_follow', default_value='false'),
        DeclareLaunchArgument('start_foxglove', default_value='true'),

        # SLAM core (always): static TF + Livox driver + MOLA odometry/SLAM.
        IncludeLaunchDescription(inc('lidar.launch.py')),

        # RealSense for YOLO (perception only).
        IncludeLaunchDescription(inc('camera_only.launch.py'),
                                 condition=IfCondition(start_camera)),

        # Nav2 + STVL.
        IncludeLaunchDescription(inc('nav.launch.py'),
                                 condition=IfCondition(start_nav)),

        # Person follow: YOLO perception + person->goal node. Needs start_camera
        # and start_nav too.
        Node(package='person_distance', executable='person_distance_node',
             name='person_distance', output='screen',
             condition=IfCondition(start_follow)),
        Node(package='person_distance', executable='person_goal_node',
             name='person_goal', output='screen',
             condition=IfCondition(start_follow)),

        # Foxglove bridge for visualisation.
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             name='foxglove_bridge', parameters=[{'port': 8765}],
             output='screen', condition=IfCondition(start_foxglove)),
    ])
