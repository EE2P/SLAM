"""One-shot bringup: RealSense + person_distance detector + follow_bridge.

Includes person_distance's own launch (camera + YOLO-seg + depth distance, now with
track_id) and starts the follow_bridge node on top, so a single command brings up the
whole perception -> follow -> /cmd_vel chain:

    ros2 launch follow_bridge track_and_follow.launch.py desired_distance_m:=2.0
    ros2 launch follow_bridge track_and_follow.launch.py start_camera:=false   # camera already up

Pass-through args mirror the two packages' own launch files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_camera = LaunchConfiguration('start_camera')
    start_foxglove = LaunchConfiguration('start_foxglove')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    desired_distance_m = LaunchConfiguration('desired_distance_m')

    person_distance_launch = os.path.join(
        get_package_share_directory('person_distance'), 'launch', 'person_distance.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_foxglove', default_value='true'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('desired_distance_m', default_value='1.5'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(person_distance_launch),
            launch_arguments={
                'start_camera': start_camera,
                'start_foxglove': start_foxglove,
            }.items(),
        ),

        Node(
            package='follow_bridge',
            executable='follow_bridge_node',
            name='follow_bridge',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'desired_distance_m': desired_distance_m,
            }],
            output='screen',
        ),
    ])
