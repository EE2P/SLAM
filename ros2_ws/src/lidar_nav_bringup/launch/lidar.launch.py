"""P0-P2: Livox MID-360 driver + static TF + MOLA LiDAR-inertial odometry/SLAM.

This is the SLAM core. It brings up:
  1. static_tf.launch.py          (base_link -> sensor frames)
  2. livox_ros_driver2 node       (publishes /livox/lidar PointCloud2 + /livox/imu)
  3. MOLA lidar odometry          (publishes TF map->odom->base_link, builds the map)

MOLA is fed the LiDAR pose via the base_link->livox_frame static TF, and the IMU
via base_link->imu_link, so the body frame for odometry is base_link (REP-105).
With use_state_estimator:=True it fuses the MID-360 built-in IMU and keeps odom
gravity-aligned -- that is what makes the wheel-legged pitch/height wobble harmless.

Run:
  ros2 launch lidar_nav_bringup lidar.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('lidar_nav_bringup')
    default_cfg = os.path.join(pkg, 'config', 'MID360_config.json')

    user_config = LaunchConfiguration('user_config_path')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    return LaunchDescription([
        DeclareLaunchArgument('user_config_path', default_value=default_cfg,
                              description='Livox MID360_config.json (edit the LiDAR IP inside)'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),

        # 1. base_link -> sensor static transforms (rough; edit static_tf.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'static_tf.launch.py'))
        ),

        # 2. Livox MID-360 driver. xfer_format=0 -> sensor_msgs/PointCloud2 (what
        #    MOLA and Nav2/STVL want). frame_id MUST match the static TF child.
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                'xfer_format': 0,          # 0=PointCloud2, 1=Livox CustomMsg (for FAST-LIO)
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': user_config,
            }],
        ),

        # 3. MOLA LiDAR-inertial odometry + loop closure.
        #    Publishes REP-105 TF: map -> odom -> base_link.
        #    NOTE (assumption): launch path is <mola_lidar_odometry>/ros2-launchs/
        #    ros2-lidar-odometry.launch.py -- confirm on the Jetson with
        #    `ros2 pkg prefix mola_lidar_odometry` if the include fails.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('mola_lidar_odometry'),
                'ros2-launchs', 'ros2-lidar-odometry.launch.py',
            ])),
            launch_arguments={
                'lidar_topic_name': lidar_topic,
                'imu_topic_name': imu_topic,
                'use_state_estimator': 'True',          # fuse IMU -> gravity-aligned odom
                'mola_tf_base_link': 'base_link',
                'mola_bridge_odometry_frame': 'odom',
                'mola_state_estimator_reference_frame': 'map',
                'publish_localization_following_rep105': 'True',
                'ignore_lidar_pose_from_tf': 'False',   # use base_link->livox_frame TF
            }.items(),
        ),
    ])
