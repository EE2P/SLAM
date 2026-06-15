"""离线建图：回放 record.launch.py 录的 bag，解压缩后喂给 RTAB-Map。

用法（Jetson 或任何装了 ROS2+rtabmap 的 Ubuntu 机器）:
    ros2 launch rs_slam_bringup offline.launch.py bag:=~/SLAM/bags/corridor
    ros2 launch rs_slam_bringup offline.launch.py bag:=... rate:=2.0   # 2 倍速重建
    ros2 launch rs_slam_bringup offline.launch.py bag:=... camera_model:=d435i

建图过程可用 Foxglove 连本机 8765 端口观看。结束后数据库在 ~/.ros/rtabmap.db，
导出见 docs/setup_jetson.md。
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

COLOR_TOPIC = '/camera/camera/color/image_raw'
DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/camera/color/camera_info'


def load_rtabmap_params(model):
    share = get_package_share_directory('rs_slam_bringup')
    params = {}
    for name in ('rtabmap_common.yaml', f'{model}.yaml'):
        with open(os.path.join(share, 'config', name)) as f:
            params.update(yaml.safe_load(f) or {})
    return params


def setup_nodes(context, *args, **kwargs):
    bag = os.path.expanduser(context.launch_configurations['bag'])
    if not os.path.exists(bag):
        raise RuntimeError(f'bag 不存在: {bag}')
    rate = context.launch_configurations['rate']
    model = context.launch_configurations['camera_model']
    rtabmap_params = load_rtabmap_params(model)
    sim_time = {'use_sim_time': True}

    return [
        # 解压缩: jpeg -> raw 彩色
        Node(
            package='image_transport',
            executable='republish',
            name='republish_color',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', COLOR_TOPIC + '/compressed'),
                ('out', COLOR_TOPIC),
            ],
            parameters=[sim_time],
            output='screen',
        ),
        # 解压缩: png -> raw 深度
        Node(
            package='image_transport',
            executable='republish',
            name='republish_depth',
            arguments=['compressedDepth', 'raw'],
            remappings=[
                ('in/compressedDepth', DEPTH_TOPIC + '/compressedDepth'),
                ('out', DEPTH_TOPIC),
            ],
            parameters=[sim_time],
            output='screen',
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            namespace='rtabmap',
            parameters=[{
                'frame_id': 'camera_link',
                'publish_tf': True,
                'approx_sync': True,
                'wait_imu_to_init': False,
                **sim_time,
            }],
            remappings=[
                ('rgb/image', COLOR_TOPIC),
                ('depth/image', DEPTH_TOPIC),
                ('rgb/camera_info', INFO_TOPIC),
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'approx_sync': True,
                'database_path': '~/.ros/rtabmap.db',
                **sim_time,
                **rtabmap_params,
            }],
            remappings=[
                ('rgb/image', COLOR_TOPIC),
                ('depth/image', DEPTH_TOPIC),
                ('rgb/camera_info', INFO_TOPIC),
            ],
            arguments=['-d'],
            output='screen',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765, **sim_time}],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--rate', rate],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='record.launch.py 录的 bag 目录'),
        DeclareLaunchArgument('rate', default_value='1.0',
                              description='回放倍速；机器快可以 2.0'),
        DeclareLaunchArgument('camera_model', default_value='d455',
                              description='录制时的相机型号: d455 | d435i'),
        OpaqueFunction(function=setup_nodes),
    ])
