"""Static TF: base_link -> sensor frames (replaces a URDF for this rigid rig).

For a wheel-legged robot we deliberately do NOT model the legs. base_link is the
GROUND-PROJECTED body centre; the only numbers that matter are roughly where the
LiDAR/camera sit and how high the LiDAR is off the ground. Rough is fine
(+/- a few cm, +/- a few deg) -- mapping treats the LiDAR as the reference and
navigation only needs "good enough" to place the footprint and the ground filter.

>>> EDIT THE 6 NUMBERS PER SENSOR (x y z roll pitch yaw, metres / radians) <<<
Measure with a tape; you do not need calibration.

Frames (per Livox Mid-360 User Manual v1.2, coordinate system O-XYZ):
  base_link    : ground-projected body centre (z=0 at the floor)
  livox_frame  : MID-360 point-cloud origin O. X=forward, Y=left, Z=up (right-handed).
                 O sits INSIDE the sensor (~mid-height of the 65x65x60 mm body, at the
                 laser window centre) -- so measure z to the MIDDLE of the unit, not the
                 base. Must match the driver's frame_id (lidar.launch.py).
  imu_link     : MID-360 built-in IMU (ICM40609). Offset from O is only a few cm
                 (~ -0.011, -0.023, +0.044 m per Livox SDK), and the driver tags
                 /livox/imu in the LiDAR frame, so co-locating it with livox_frame is fine.
  camera_link  : RealSense root frame (its optical children hang off this).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(child, x, y, z, roll, pitch, yaw):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{child}',
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', 'base_link', '--child-frame-id', child,
        ],
        output='screen',
    )


def generate_launch_description():
    return LaunchDescription([
        # ---- TODO: replace with your tape-measured offsets (rough is OK) ----
        # x=forward, y=left, z=height of the sensor MIDDLE (origin O) above the floor.
        _static_tf('livox_frame', x=0.10, y=0.0, z=0.25, roll=0.0, pitch=0.0, yaw=0.0),
        # IMU offset from O is ~cm -> same pose as livox_frame is fine.
        _static_tf('imu_link', x=0.10, y=0.0, z=0.25, roll=0.0, pitch=0.0, yaw=0.0),
        # RealSense mounting (only used by the perception pipeline, not by SLAM).
        _static_tf('camera_link', x=0.12, y=0.0, z=0.20, roll=0.0, pitch=0.0, yaw=0.0),
    ])
