"""现场录包：只启动相机并录制压缩后的 RGB-D 流，供 offline.launch.py 离线建图。

用法（Jetson 上，手持相机+电池）:
    ros2 launch rs_slam_bringup record.launch.py                 # 存到 ~/SLAM/bags/rs_<时间戳>
    ros2 launch rs_slam_bringup record.launch.py bag:=~/SLAM/bags/corridor

录制内容：JPEG 压缩彩色图 + PNG 压缩深度图 + 相机内参 + 静态 TF，
640x480x30 下码率约 4~6 MB/s（原始流是 43 MB/s）。Ctrl-C 结束录制。
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

RECORD_TOPICS = [
    '/camera/camera/color/image_raw/compressed',
    '/camera/camera/color/camera_info',
    '/camera/camera/aligned_depth_to_color/image_raw/compressedDepth',
    '/camera/camera/aligned_depth_to_color/camera_info',
    '/tf_static',
]


def setup_nodes(context, *args, **kwargs):
    bag = context.launch_configurations['bag']
    if not bag:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag = os.path.expanduser(f'~/SLAM/bags/rs_{stamp}')
    else:
        bag = os.path.expanduser(bag)
    print(f'[rs_slam_bringup] recording to: {bag}')

    return [
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'enable_sync': True,
                'rgb_camera.color_profile': '640x480x30',
                'depth_module.depth_profile': '640x480x30',
            }],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag] + RECORD_TOPICS,
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bag', default_value='',
            description='输出 bag 路径；留空自动用 ~/SLAM/bags/rs_<时间戳>'),
        OpaqueFunction(function=setup_nodes),
    ])
