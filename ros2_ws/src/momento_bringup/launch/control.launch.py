import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    start_joy = LaunchConfiguration('start_joy')
    start_perception = LaunchConfiguration('start_perception')
    start_camera = LaunchConfiguration('start_camera')
    start_foxglove = LaunchConfiguration('start_foxglove')
    serial_port = LaunchConfiguration('serial_port')
    default_leg = LaunchConfiguration('default_leg')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    follow_launch = os.path.join(
        get_package_share_directory('follow_bridge'),
        'launch',
        'track_and_follow.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('default_leg', default_value='0.07'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('start_joy', default_value='true'),
        DeclareLaunchArgument('start_perception', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_foxglove', default_value='true'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            condition=IfCondition(start_joy),
        ),

        Node(
            package='momento_xbox',
            executable='xbox_mode_node',
            name='momento_xbox',
            parameters=[{
                'default_leg': default_leg,
            }],
            output='screen',
        ),

        Node(
            package='momento_command_mux',
            executable='command_mux_node',
            name='momento_command_mux',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic,
                'default_leg': default_leg,
            }],
            output='screen',
        ),

        Node(
            package='momento_usb_bridge',
            executable='usb_bridge_node',
            name='momento_usb_bridge',
            parameters=[{
                'port': serial_port,
                'rate_hz': 100.0,
                'default_leg': default_leg,
            }],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(follow_launch),
            launch_arguments={
                'start_camera': start_camera,
                'start_foxglove': start_foxglove,
                'cmd_vel_topic': cmd_vel_topic,
            }.items(),
            condition=IfCondition(start_perception),
        ),
    ])
