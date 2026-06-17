# Livox MID-360 SLAM + Nav2 导航 —— 设计、运行与假设

记录这套 `lidar_nav_bringup` 的全部设计思路与假设，供上机调试和后续接手参考。

设备：Jetson Orin Nano / Ubuntu 24.04 / **ROS2 Jazzy**（见 `setup_jetson.md`），轮腿底盘，
Livox **MID-360**（自带 IMU），可视化用 Foxglove。

---

## 1. 目标

MID-360 建图 → 实时导航 + **3D 避障** → 把 YOLO 检测到的人当 Nav2 目标点“跟人”。

## 2. 架构与 TF 铁律

```
livox_ros_driver2 ─┬─/livox/lidar(PointCloud2)─┐
 (MID-360 网口)     ├─/livox/imu (200Hz内置)────┤ MOLA LO ─► TF map→odom→base_link, 建图
3×static_tf ─► base_link→{livox_frame,imu_link,camera_link}（粗略，卷尺即可）
RealSense ─► YOLO(person_distance, GPU) ─► /person_distances ─► person_goal_node ─► /goal_pose
Nav2 + STVL(吃 /livox/lidar) ◄─ TF ─► /cmd_vel ─► 板载底盘控制器
```

**TF 铁律**：`odom→base` 只准 MOLA 发；`map→odom` 只准 MOLA 的闭环/状态估计发。
相机**只做感知**，绝不发任何里程计/SLAM（否则 TF 边冲突）。

## 3. 选型与理由（已联网核实于 Jazzy）

| 角色 | 选择 | 理由 / 状态 |
|---|---|---|
| 雷达驱动 | `livox_ros_driver2` | 官方支持 Jazzy + MID-360；**源码编译** |
| 里程计/SLAM（主） | **MOLA** `mola_lidar_odometry` | apt 装 Jazzy、自带闭环、用 IMU、纯 CPU（GPU 留给 YOLO） |
| 里程计（备选/升级） | FAST-LIO2 ROS2 fork（Caltech-AMBER 带 MID360） | Livox 更原生、更轻，但官方仅 ROS1、fork 多为 Humble，Jazzy/24.04 需改编译 |
| 导航 | **Nav2** | apt，官方标准 |
| 3D 避障 | **STVL** `spatio_temporal_voxel_layer` | apt 装 Jazzy（v2.5.5）；3D 体素障碍投影到 2D 代价 |
| 闭环/栅格（可选） | RTAB-Map | 已有；如需可作后端 |

里程计节点**可抽换**：换成 FAST-LIO2 时其余栈（驱动/TF/Nav2/STVL/跟人）不动。

## 4. 关键设计决定

1. **不写 URDF**，用 3 条 `static_transform_publisher`（`static_tf.launch.py`）。Foxglove 显示点云/TF/代价地图/路径都不需要 URDF。
2. **外参只需粗略**（±cm/±度）：建图把雷达当参考；导航只需大致定位 footprint + 雷达高度。`lidar↔imu` 不测（MID-360 出厂值，且两者近乎同位，用相同 base TF 近似）。
3. **轮腿俯仰/升降自动消化**：MOLA 用 IMU 输出**重力对齐 odom**（z 朝天、地面恒在 z≈0），STVL 在全局重力系按高度带滤波 → 车身晃动不会把地面误判成障碍。只需给 `min_obstacle_height` 留余量（设 0.12）。
4. **footprint 用保守圆** `robot_radius`（罩住最大轮廓）→ 腿任何姿态都安全，无需跟踪腿位姿。
5. **相机退为纯感知**：`camera_only.launch.py` 只起 realsense，关掉 IMU 与一切 SLAM。
6. **跟人用全局导航**：`person_goal_node` 选最近有效的人 → TF 到 `map` → 发**带 stand-off 的目标点**（停在人前方 1m，朝向人），Nav2+STVL 自动绕障。比旧的单目像素跟随鲁棒。

## 5. 包 / 文件

```
ros2_ws/src/lidar_nav_bringup/
  config/MID360_config.json   雷达网络（改 LiDAR IP；host=192.168.1.5）
  config/nav2_params.yaml     Nav2 全栈 + STVL 层 + 圆 footprint
  launch/static_tf.launch.py  base→传感器 静态 TF（填粗略外参）
  launch/lidar.launch.py      静态TF + Livox 驱动 + MOLA（SLAM 核心）
  launch/camera_only.launch.py 仅 realsense（给 YOLO）
  launch/nav.launch.py        Nav2（含 STVL），消费 TF+点云
  launch/bringup.launch.py    顶层一键（开关 start_camera/nav/follow/foxglove）
ros2_ws/src/person_distance/
  person_distance/person_goal_node.py  /person_distances → /goal_pose
```

## 6. 构建（在 Jetson 上，Mac 无法编译）

```bash
# 1) apt 依赖
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-spatio-temporal-voxel-layer \
  ros-jazzy-mola ros-jazzy-mola-state-estimation ros-jazzy-mola-lidar-odometry \
  ros-jazzy-rtabmap-ros ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs

# 2) 源码编译 Livox 驱动（克隆进 ws/src）
#    先装 Livox-SDK2，再装 livox_ros_driver2，按其 README 以 ROS2/Jazzy 方式编。
#    git clone https://github.com/Livox-SDK/Livox-SDK2 ... && cmake/make/install
#    git clone https://github.com/Livox-SDK/livox_ros_driver2 ~/SLAM/ros2_ws/src/

# 3) 编译本工作区
cd ~/SLAM/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 7. 运行（按阶段，每步一个验证 gate）

```bash
# P0 仅驱动：Foxglove 看 /livox/lidar 形状对、/livox/imu 有数
ros2 launch lidar_nav_bringup lidar.launch.py        # （含驱动+TF+MOLA）

# P1 TF：base_link→{livox,imu,camera} 不断裂
ros2 run tf2_tools view_frames

# P2 里程计：手推走一圈→地图一致、odom→base 不漂、回原点闭环（lidar.launch.py 已含 MOLA）

# P3/P4 导航：另开终端
ros2 launch lidar_nav_bringup nav.launch.py
#   Foxglove 发 2D Goal Pose → 规划→绕障→到达

# P6 跟人：一键全开
ros2 launch lidar_nav_bringup bringup.launch.py \
    start_camera:=true start_nav:=true start_follow:=true
```

## 8. 假设值（上机务必核对/调） ⚠️

**外参（占位，必须按实测改）** — `static_tf.launch.py`：
- `base_link→livox_frame`：x=0.10, y=0, **z=0.25**。⚠️ z 量到传感器**中段（点云原点 O，约机身一半高、出光窗中心）**，不是量到底座（手册 P14 坐标系）。MID-360 帧为 X 前 / Y 左 / Z 上。
- `base_link→imu_link`：同 livox_frame（IMU 相对 O 仅几 cm 偏移，且驱动把 /livox/imu 标在雷达帧）
- `base_link→camera_link`：x=0.12, y=0, z=0.20

**网络** — `MID360_config.json`：
- host IP `192.168.1.5`（Jetson 网卡需设此静态 IP，掩码 255.255.255.0）
- LiDAR IP `192.168.1.112` —— **改成 `192.168.1.1XX`，XX=你雷达广播码后两位**

**Nav2/STVL** — `nav2_params.yaml`（全部按机器人实测调）：
- `robot_radius: 0.40`、`inflation_radius: 0.55`（保守圆，按最大轮廓改）
- `vx_max: 0.5`、`wz_max: 1.0`、加减速 `2.0/3.0`（按底盘能力改）
- `motion_model: DiffDrive`（轮腿对 Nav2 = 差速；若阿克曼改控制器）
- `min_obstacle_height: 0.12`、`max_obstacle_height: 2.0`（按车高与地面平整度调）
- `xy_goal_tolerance: 0.25`（“大致到”即可）
- MID-360 视场写死：`vertical_fov_angle: 1.0297`（59°）、`horizontal_fov_angle: 6.2832`（360°）

**MID-360 规格（已对 Livox 用户手册 v1.2 / `Code/Others/Livox_Mid-360_User_Manual_CHS.pdf` 核实）**：
垂直 FOV **-7°~+52°**、水平 360°、近处盲区 0.1m、量程 40m@10%/70m@80%、点率 20万/s、帧率 10Hz、
内置 IMU **ICM40609**（输出率约 200Hz，取自 SDK；手册规格表未列）、数据口 100 BASE-TX 以太网（静态 IP）、
尺寸 65×65×60mm、重量约 265g、供电 9–27V（荐 12V）、底部 4×M3 安装孔（深 5mm）。

## 9. 上机需确认的“不确定点”

1. **MOLA launch 路径**：`lidar.launch.py` 假设为
   `<mola_lidar_odometry>/ros2-launchs/ros2-lidar-odometry.launch.py`，参数名
   （`lidar_topic_name`/`imu_topic_name`/`use_state_estimator`/`mola_tf_base_link`/
   `mola_bridge_odometry_frame`/`mola_state_estimator_reference_frame`/
   `publish_localization_following_rep105`/`ignore_lidar_pose_from_tf`）取自其
   develop 分支 launch 源码。装的版本若不同，按 `ros2 launch mola_lidar_odometry
   ros2-lidar-odometry.launch.py --show-args` 校正。
2. **Livox 驱动参数**：`xfer_format=0` 出 PointCloud2、`frame_id=livox_frame`。
3. **Nav2 参数 schema**：基于 Jazzy 默认；若某键被拒，对照
   `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` 调整
   （如 `progress_checker_plugins` 复数形式按发行版）。
4. **MID360 IMU 话题坐标系**：MOLA 经 `base_link→imu_link` TF 取 IMU 外参。

## 10. 风险与回退

- MOLA 对 Livox 调参不顺 / 长走廊漂移 → 抽换为 **FAST-LIO2 fork**（其余不动）。
- MOLA 闭环模块当前可能需源码编（apt 即将上）；先不接闭环也能导航。
- 超简回退：`KISS-ICP`（apt、最稳打包，但不吃 IMU）。

## 11. 后续

- 建图→存图→定位模式（给 Nav2 加 `map_server` + `static_layer`）。
- 跟人加**持久 track id**（在 `person_distance_node` 用 ultralytics `.track()`），
  让 `person_goal_node` 锁定单一目标、不被路人带跑。
