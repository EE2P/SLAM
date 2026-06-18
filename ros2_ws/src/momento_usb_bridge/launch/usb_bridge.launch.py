from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('rate_hz', default_value='100.0'),
        DeclareLaunchArgument('default_leg', default_value='0.12'),

        Node(
            package='momento_usb_bridge',
            executable='usb_bridge_node',
            name='momento_usb_bridge',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'rate_hz': LaunchConfiguration('rate_hz'),
                'default_leg': LaunchConfiguration('default_leg'),
            }],
            output='screen',
        ),
    ])
