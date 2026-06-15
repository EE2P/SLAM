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
| `use_imu` | `auto` | 自动检测内核 IMU 支持；可强制 `true`/`false`（本机 L4T 内核无 hid_sensor 模块，IMU 不可用）|
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

## 现场录包 + 离线建图

适合走廊等大场景：现场只录数据（轻量、不怕卡），回来反复调参重建。

### 1. 现场录制（Jetson + 手持相机 + 电池）

```bash
ros2 launch rs_slam_bringup record.launch.py                  # 自动存 ~/SLAM/bags/rs_<时间戳>
ros2 launch rs_slam_bringup record.launch.py bag:=~/SLAM/bags/corridor
```

录的是 JPEG 彩色 + PNG 深度 + 内参 + 静态 TF，约 4~6 MB/s（10 分钟 ≈ 3GB）。
Ctrl-C 结束。采集走法与在线建图相同：走慢、转弯缓、终点回到起点闭环。

### 2. 离线重建（Jetson 本机，或任何装了 ROS2 Jazzy 的 Ubuntu）

Ubuntu 机器一次性准备（不需要相机驱动）：

```bash
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-image-transport-plugins \
                 ros-jazzy-foxglove-bridge python3-colcon-common-extensions
git clone git@github.com:EE2P/SLAM.git ~/SLAM
cd ~/SLAM/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

重建（bag 用 scp 拷过来）：

```bash
source /opt/ros/jazzy/setup.bash && source ~/SLAM/ros2_ws/install/setup.bash
ros2 launch rs_slam_bringup offline.launch.py bag:=~/SLAM/bags/corridor
# 机器快可加 rate:=2.0；D435i 录的加 camera_model:=d435i
```

重建过程可用 Foxglove 连本机 `ws://localhost:8765` 实时围观（本机连接不卡）。
回放结束后 Ctrl-C，结果在 `~/.ros/rtabmap.db`，导出方法见下节。

## 保存地图

数据库自动存在 `~/.ros/rtabmap.db`。导出：

```bash
# 2D 栅格图 (pgm+yaml)
ros2 run nav2_map_server map_saver_cli -f ~/SLAM/maps/room --ros-args -r map:=/rtabmap/map   # 需 nav2-map-server
# 或直接拷贝数据库
cp ~/.ros/rtabmap.db ~/SLAM/maps/room.db
```

## 已知注意事项

- D435i 的 IMU 出厂未标定；RTAB-Map 不敏感，但以后换 cuVSLAM 之类 VIO 前建议先跑 Intel IMU 标定工具。
- iPhone 热点会对 macOS 客户端下发 DHCP Option 108（IPv6-only），Mac 需手动静态 IPv4 才能进 172.20.10.x 网段（Ubuntu/Jetson 不受影响）。
