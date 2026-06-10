"""RGB-D SLAM bringup: RealSense (D455/D435i) + IMU filter + RTAB-Map + Foxglove bridge.

用法（Jetson 上）:
    ros2 launch rs_slam_bringup slam.launch.py                # 自动识别相机型号
    ros2 launch rs_slam_bringup slam.launch.py camera_model:=d435i
    ros2 launch rs_slam_bringup slam.launch.py use_imu:=false # IMU 不可用时退化为纯 RGB-D
    ros2 launch rs_slam_bringup slam.launch.py delete_db:=false  # 续建上次的图
"""

import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

# USB PID -> 型号
KNOWN_MODELS = {'0b5c': 'd455', '0b3a': 'd435i'}

COLOR_TOPIC = '/camera/camera/color/image_raw'
DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw'
INFO_TOPIC = '/camera/camera/color/camera_info'
RAW_IMU_TOPIC = '/camera/camera/imu'
FILTERED_IMU_TOPIC = '/imu/data'


def detect_camera_model():
    """通过 lsusb 的 Intel PID 识别插着的 RealSense 型号。"""
    try:
        out = subprocess.run(['lsusb'], capture_output=True, text=True,
                             timeout=5).stdout.lower()
    except (OSError, subprocess.TimeoutExpired):
        return None
    for pid, model in KNOWN_MODELS.items():
        if f'8086:{pid}' in out:
            return model
    return None


def load_rtabmap_params(model):
    share = get_package_share_directory('rs_slam_bringup')
    params = {}
    for name in ('rtabmap_common.yaml', f'{model}.yaml'):
        with open(os.path.join(share, 'config', name)) as f:
            params.update(yaml.safe_load(f) or {})
    return params


def setup_nodes(context, *args, **kwargs):
    model = context.launch_configurations['camera_model']
    if model == 'auto':
        model = detect_camera_model()
        if model is None:
            raise RuntimeError(
                '未检测到 RealSense 相机（lsusb 没有 8086:0b5c / 8086:0b3a），'
                '请检查 USB 连接，或显式指定 camera_model:=d455|d435i')
    print(f'[rs_slam_bringup] camera model: {model}')

    use_imu = context.launch_configurations['use_imu'].lower() == 'true'
    localization = context.launch_configurations['localization'].lower() == 'true'
    delete_db = context.launch_configurations['delete_db'].lower() == 'true'
    serial_no = context.launch_configurations['serial_no']

    rtabmap_params = load_rtabmap_params(model)
    if localization:
        rtabmap_params['Mem/IncrementalMemory'] = 'false'

    camera_params = {
        'enable_color': True,
        'enable_depth': True,
        'align_depth.enable': True,
        'enable_sync': True,
        'rgb_camera.color_profile': '640x480x30',
        'depth_module.depth_profile': '640x480x30',
        'enable_gyro': use_imu,
        'enable_accel': use_imu,
    }
    if use_imu:
        camera_params['unite_imu_method'] = 2  # 陀螺仪/加速度计线性插值合并为 /imu
    if serial_no:
        camera_params['serial_no'] = serial_no

    nodes = [
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[camera_params],
            output='screen',
        ),
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[{
                'frame_id': 'camera_link',
                'publish_tf': True,
                'approx_sync': True,
                'wait_imu_to_init': use_imu,
                'qos': 2,
                'qos_imu': 2,
            }],
            remappings=[
                ('rgb/image', COLOR_TOPIC),
                ('depth/image', DEPTH_TOPIC),
                ('rgb/camera_info', INFO_TOPIC),
                ('imu', FILTERED_IMU_TOPIC),
            ],
            output='screen',
        ),
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[{
                'frame_id': 'camera_link',
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
                ('imu', FILTERED_IMU_TOPIC),
            ],
            arguments=['-d'] if delete_db else [],
            output='screen',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765}],
            output='screen',
        ),
    ]

    if use_imu:
        nodes.append(Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
            }],
            remappings=[
                ('imu/data_raw', RAW_IMU_TOPIC),
                ('imu/data', FILTERED_IMU_TOPIC),
            ],
            output='screen',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_model', default_value='auto',
            description='auto | d455 | d435i'),
        DeclareLaunchArgument(
            'serial_no', default_value='',
            description='多台相机同时插入时用序列号区分'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='false 时退化为纯 RGB-D 里程计（IMU 权限问题时的逃生口）'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='true 时加载已有地图做定位，不再扩展地图'),
        DeclareLaunchArgument(
            'delete_db', default_value='true',
            description='启动时删除旧数据库重新建图；续建请设为 false'),
        OpaqueFunction(function=setup_nodes),
    ])
