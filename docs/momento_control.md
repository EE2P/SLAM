# Momento Xbox + USB 控制栈

这套控制栈用于 Xbox 手柄模式切换、YOLO/其他节点自动控制仲裁，并通过 USB CDC 以 100Hz 向 STM32 下发平衡控制指令。它不启动任何 SLAM。

## 架构

```text
Xbox -> joy_node -> momento_xbox -> /momento/mode + /momento/xbox_cmd
YOLO/follow_bridge -> /cmd_vel
其他自动节点 -> /momento/auto_cmd

/momento/mode + /momento/xbox_cmd + /cmd_vel + /momento/auto_cmd
  -> momento_command_mux -> /momento/cmd
  -> momento_usb_bridge -> USB CDC 100Hz -> STM32
```

核心原则：只有 `momento_usb_bridge` 写 USB。所有控制源必须先进 `momento_command_mux`。

## 启动

```bash
cd /home/han/Code/SLAM/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to momento_bringup
source install/setup.bash

ros2 launch momento_bringup control.launch.py serial_port:=/dev/ttyACM0
```

默认启动：

- `joy_node`
- `momento_xbox`
- `momento_command_mux`
- `momento_usb_bridge`
- `follow_bridge track_and_follow.launch.py`，即 RealSense + YOLO + `/cmd_vel`
- Foxglove bridge

不启动 `rs_slam_bringup`、`lidar_nav_bringup`、RTAB-Map、MOLA 或 Nav2。

只要手柄 + USB，不要 YOLO：

```bash
ros2 launch momento_bringup control.launch.py start_perception:=false
```

相机已经启动时：

```bash
ros2 launch momento_bringup control.launch.py start_camera:=false
```

## USB 协议

`momento_usb_bridge` 复用 `Legged_Embedded` 里的 balance 包：

```text
header: 0xBA 0x1A
payload: float32[5] + uint8 start_flag
fields: v_set, turn_set, roll_set, leg_set, phi_set, start_flag
crc: CRC-8, poly 0x07, init 0x00
rate: 100Hz
```

映射关系：

```text
linear -> v_set
yaw    -> turn_set，默认把 yaw rate 积分成 heading
roll   -> roll_set
pitch  -> phi_set
leg    -> leg_set
start  -> start_flag
```

如果 `/momento/cmd` 超时，USB bridge 会持续发送 `start=false` 的零命令。

## 模式

`MANUAL`：Xbox 完全控制。

```text
linear / yaw / roll / pitch / leg 全部来自 Xbox
```

`AUTO`：自动节点控制。

```text
优先使用 /momento/auto_cmd
如果没有，则使用 /cmd_vel
```

`/cmd_vel` 会被映射为：

```text
linear = linear.x
yaw = angular.z
roll = 0
pitch = 0
leg = default_leg
start = true
```

`ASSIST`：混合控制。

```text
linear / yaw 来自自动节点
roll / pitch / leg 来自 Xbox
```

用途：自动跟人时，仍然可以用 Xbox 微调姿态和腿高。

`ESTOP`：急停。

```text
linear = 0
yaw = 0
roll = 0
pitch = 0
start = false
```

## Xbox 默认按键

```text
A      Enable
X      Arm
B      Disarm，并 start=false
Back   Disable，并进入 ESTOP
RB     Clear fault；如果当前是 ESTOP，则回到 MANUAL
Start  切换 start_flag
Y      MANUAL -> AUTO -> ASSIST 循环
LB     直接进入 ASSIST
DPad ↑ leg_set 增加
DPad ↓ leg_set 减小
Left Y   linear
Left X   yaw rate
Right X  roll
Right Y  pitch
```

如果实机轴号或按钮号不一致，改 `momento_xbox` 参数：`axis_linear`、`axis_yaw`、`axis_roll`、`axis_pitch`、`axis_dpad_y`、`button_*`。

## 其他节点接入

简单接入发布 `/cmd_vel`：

```text
geometry_msgs/Twist
linear.x  -> 前进速度
angular.z -> yaw rate
```

完整接入发布 `/momento/auto_cmd`：

```text
momento_msgs/MomentoCommand
linear
yaw
roll
pitch
leg
start
yaw_mode
source
```

优先级：

```text
ESTOP 最高
MANUAL 使用 /momento/xbox_cmd
AUTO 使用 /momento/auto_cmd，若没有则使用 /cmd_vel
ASSIST 使用自动 linear/yaw + Xbox roll/pitch/leg
```

## 检查

```bash
ros2 topic echo /joy
ros2 topic echo /momento/mode
ros2 topic echo /momento/xbox_cmd
ros2 topic echo /cmd_vel
ros2 topic echo /momento/cmd
ros2 topic echo /momento/state
```

手动发控制命令：

```bash
ros2 service call /momento/control momento_msgs/srv/MomentoControl "{command: 1, joint_idx: 255}"
ros2 service call /momento/control momento_msgs/srv/MomentoControl "{command: 4, joint_idx: 255}"
```

`command: 1` 是 Enable，`command: 4` 是 Arm。
