"""RealSense D455 as a PERCEPTION-ONLY sensor (color + aligned depth) for YOLO.

Deliberately NO rtabmap / rgbd_odometry here: in the LiDAR stack, MOLA owns the
odom->base_link TF and the loop closure owns map->odom. Running the camera's own
SLAM/odometry would publish a competing TF edge and corrupt navigation.

The base_link->camera_link transform comes from static_tf.launch.py (started by
lidar.launch.py), so this only launches the driver.

Run (with the LiDAR stack already up):
  ros2 launch lidar_nav_bringup camera_only.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'rgb_camera.color_profile': '640x480x30',
                'depth_module.depth_profile': '640x480x30',
                # IMU off: the LiDAR provides inertial data; the D455 IMU is
                # unavailable on this L4T kernel anyway (see docs/setup_jetson.md).
                'enable_gyro': False,
                'enable_accel': False,
            }],
            output='screen',
        ),
    ])
