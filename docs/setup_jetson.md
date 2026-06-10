# Jetson SLAM 环境与使用说明

设备：Jetson (L4T R39.2, Ubuntu 24.04, ROS2 Jazzy)，相机 RealSense D455 / D435i（USB 自动识别）。

## 一次性安装（已完成，重装机时参考）

```bash
sudo apt install ros-jazzy-realsense2-camera \
                 ros-jazzy-rtabmap-ros \
                 ros-jazzy-imu-filter-madgwick \
                 ros-jazzy-foxglove-bridge \
                 python3-colcon-common-extensions

# RealSense udev 规则（IMU 需要；装完后拔插一次相机 USB）
curl -sSL https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules \
  | sudo tee /etc/udev/rules.d/99-realsense-libusb.rules > /dev/null
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 构建

```bash
cd ~/SLAM/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

## 启动 SLAM

```bash
source /opt/ros/jazzy/setup.bash
source ~/SLAM/ros2_ws/install/setup.bash
ros2 launch rs_slam_bringup slam.launch.py
```

常用参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `camera_model` | `auto` | `auto` 按 USB PID 识别；可强制 `d455` / `d435i` |
| `use_imu` | `true` | IMU 拿不到数据时设 `false` 退化为纯 RGB-D |
| `delete_db` | `true` | 每次启动重新建图；设 `false` 续建 `~/.ros/rtabmap.db` |
| `localization` | `false` | `true` = 加载已有地图做定位，不扩展地图 |
| `serial_no` | 空 | 两台相机同时插时用序列号指定 |

## Foxglove（在 Ubuntu 设备上看）

1. 安装 Foxglove Studio（<https://foxglove.dev/download>）。
2. Open connection → Foxglove WebSocket → `ws://<jetson-ip>:8765`。
3. Layout → Import from file → 选仓库里的
   `ros2_ws/src/rs_slam_bringup/foxglove/slam_layout.json`。

布局内容：3D 面板（点云地图 `/rtabmap/cloud_map`、**轨迹 `/rtabmap/mapPath`**、2D 栅格 `/map`、TF）、RGB/深度图像、控制按钮（重置/暂停/继续建图）。

热点带宽不够时：3D 面板里关掉 `/rtabmap/cloud_map`，只留 `/map` 和轨迹。

## 保存地图

数据库自动存在 `~/.ros/rtabmap.db`。导出：

```bash
# 2D 栅格图 (pgm+yaml)
ros2 run nav2_map_server map_saver_cli -f ~/SLAM/maps/room   # 需 nav2-map-server
# 或直接拷贝数据库
cp ~/.ros/rtabmap.db ~/SLAM/maps/room.db
```

## 已知注意事项

- D435i 的 IMU 出厂未标定；RTAB-Map 不敏感，但以后换 cuVSLAM 之类 VIO 前建议先跑 Intel IMU 标定工具。
- iPhone 热点会对 macOS 客户端下发 DHCP Option 108（IPv6-only），Mac 需手动静态 IPv4 才能进 172.20.10.x 网段（Ubuntu/Jetson 不受影响）。
