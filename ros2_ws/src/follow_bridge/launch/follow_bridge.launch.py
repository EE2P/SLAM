"""Launch the follow bridge.

Assumes the Jetson person_distance node (publishing /person_distances WITH track_id)
is already running. Override the follow distance or topics from the CLI, e.g.:

    ros2 launch follow_bridge follow_bridge.launch.py desired_distance_m:=2.0
    ros2 launch follow_bridge follow_bridge.launch.py cmd_vel_topic:=/momento/cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    person_topic = LaunchConfiguration('person_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    desired_distance_m = LaunchConfiguration('desired_distance_m')

    return LaunchDescription([
        DeclareLaunchArgument('person_topic', default_value='/person_distances'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('desired_distance_m', default_value='1.5'),

        Node(
            package='follow_bridge',
            executable='follow_bridge_node',
            name='follow_bridge',
            parameters=[{
                'person_topic': person_topic,
                'cmd_vel_topic': cmd_vel_topic,
                'desired_distance_m': desired_distance_m,
            }],
            output='screen',
        ),
    ])
