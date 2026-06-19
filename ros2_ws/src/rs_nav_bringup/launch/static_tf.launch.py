"""Static TF: base_link -> camera_link for the RealSense-only nav stack.

RTAB-Map odometry runs with frame_id=base_link and publishes map->odom->base_link;
the RealSense driver publishes camera_link -> its optical child frames. This one
static edge ties the camera onto the robot body. Rough is fine (+/- a few cm/deg).

>>> EDIT THE 6 NUMBERS (x y z roll pitch yaw, metres / radians) <<<
base_link = ground-projected body centre (z=0 at the floor); x=forward, y=left.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_camera_link',
            arguments=[
                # TODO: tape-measure where the camera sits on the robot.
                '--x', '0.12', '--y', '0.0', '--z', '0.20',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ],
            output='screen',
        ),
    ])
