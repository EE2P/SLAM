"""Nav2 for the RealSense-only stack. Consumes TF (map->odom->base_link from
RTAB-Map) + RTAB-Map's /rtabmap/map (global static layer) + the depth cloud
(local STVL layer). Run rs_slam.launch.py first.

Run:
  ros2 launch rs_nav_bringup nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('rs_nav_bringup')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_launch = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',
            }.items(),
        ),
    ])
