"""P3-P4: Nav2 navigation stack with the STVL 3D voxel obstacle layer.

Consumes ONLY TF (map->odom->base_link from MOLA) + the LiDAR cloud (/livox/lidar).
It does NOT start the camera or LiDAR -- run lidar.launch.py first.

This uses nav2_bringup's navigation_launch.py (controller/planner/behaviors/
bt_navigator/velocity_smoother + lifecycle manager). No map_server/AMCL: MOLA
provides map->odom, and the costmaps run rolling-window (mapless) to start. For
navigation against a saved map, add a map_server + static_layer later (see
docs/lidar_nav_plan.md).

Run:
  ros2 launch lidar_nav_bringup nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('lidar_nav_bringup')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_launch = os.path.join(
        FindPackageShare('nav2_bringup').find('nav2_bringup'),
        'launch', 'navigation_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 params (controllers, costmaps, STVL layer)'),
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
