"""RealSense RGB-D SLAM for navigation: camera + RTAB-Map, with base_link as the
robot frame so Nav2 can plan for the body (not the camera).

Differs from rs_slam_bringup/slam.launch.py in three ways:
  1. frame_id = base_link (+ static base_link->camera_link) instead of camera_link,
     so the TF chain is map->odom->base_link->camera_link->optical (Nav2-ready).
  2. The depth point cloud is enabled (pointcloud.enable) so Nav2's local costmap
     (STVL) gets 3D obstacles within the camera FOV.
  3. No LiDAR. RTAB-Map's 2D occupancy grid (/rtabmap/map) feeds Nav2's global
     costmap static layer -- important because the camera only sees ~87 deg
     forward, so the SLAM map is what "remembers" obstacles off to the sides/behind.

RTAB-Map parameters are REUSED from rs_slam_bringup (rtabmap_common.yaml +
<camera_model>.yaml) so tuning stays in one place.

Run:
  ros2 launch rs_nav_bringup rs_slam.launch.py                      # D455 (default)
  ros2 launch rs_nav_bringup rs_slam.launch.py camera_model:=d435i
  ros2 launch rs_nav_bringup rs_slam.launch.py localization:=true   # use existing map
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

COLOR_TOPIC = '/camera/camera/color/image_raw'
DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/camera/color/camera_info'


def load_rtabmap_params(model):
    """Reuse rs_slam_bringup's tuned RTAB-Map params (common + camera model)."""
    share = get_package_share_directory('rs_slam_bringup')
    params = {}
    for name in ('rtabmap_common.yaml', f'{model}.yaml'):
        with open(os.path.join(share, 'config', name)) as f:
            params.update(yaml.safe_load(f) or {})
    return params


def setup_nodes(context, *args, **kwargs):
    model = context.launch_configurations['camera_model']
    start_camera = context.launch_configurations['start_camera'].lower() == 'true'
    localization = context.launch_configurations['localization'].lower() == 'true'
    delete_db = context.launch_configurations['delete_db'].lower() == 'true'

    rtabmap_params = load_rtabmap_params(model)
    if localization:
        rtabmap_params['Mem/IncrementalMemory'] = 'false'  # localize against saved map

    nodes = []

    if start_camera:
        nodes.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'enable_sync': True,
                'pointcloud.enable': True,   # depth cloud for the Nav2 STVL local layer
                'rgb_camera.color_profile': '640x480x30',
                'depth_module.depth_profile': '640x480x30',
                'enable_gyro': False,        # D455/D435i IMU unavailable on this L4T kernel
                'enable_accel': False,
            }],
            output='screen',
        ))

    nodes.append(Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        namespace='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'publish_tf': True,
            'approx_sync': True,
            'wait_imu_to_init': False,
            'qos': 2,
        }],
        remappings=[
            ('rgb/image', COLOR_TOPIC),
            ('depth/image', DEPTH_TOPIC),
            ('rgb/camera_info', INFO_TOPIC),
        ],
        output='screen',
    ))

    nodes.append(Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'approx_sync': True,
            'qos_image': 2,
            'database_path': '~/.ros/rtabmap.db',
            **rtabmap_params,
        }],
        remappings=[
            ('rgb/image', COLOR_TOPIC),
            ('depth/image', DEPTH_TOPIC),
            ('rgb/camera_info', INFO_TOPIC),
        ],
        arguments=['-d'] if (delete_db and not localization) else [],
        output='screen',
    ))

    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('rs_nav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='d455',
                              description='d455 | d435i (sets RTAB-Map depth limits)'),
        DeclareLaunchArgument('start_camera', default_value='true',
                              description='false if the RealSense driver runs elsewhere'),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = localize in the saved map, do not extend'),
        DeclareLaunchArgument('delete_db', default_value='true',
                              description='true = fresh map each run; false = continue'),

        # base_link -> camera_link (edit the offsets in static_tf.launch.py)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'static_tf.launch.py'))
        ),

        OpaqueFunction(function=setup_nodes),
    ])
