# person_distance

YOLO 实例分割（person 类）+ RealSense 对齐深度图 → 每个人的平均距离。

## 运行

```bash
# 独立运行（自带启动 D455，开启 align_depth）
ros2 launch person_distance person_distance.launch.py

# 相机已由其他 launch（如 rs_slam_bringup）启动时
ros2 launch person_distance person_distance.launch.py start_camera:=false
```

## 话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/person_distances` | `person_distance_msgs/PersonDistanceArray` | 每帧检测结果（供其他应用订阅） |
| `/person_distances/debug_image` | `sensor_msgs/Image` | 叠加掩码与距离标注的调试图（有订阅者时才发布） |

`PersonDistance` 字段：`id`、`confidence`、`distance`（掩码内有效深度去除 5%/95% 尾部后的均值，米）、
`position`（相机光学坐标系下的质心 3D 坐标）、`pixel_x/pixel_y`（质心像素）、`valid_depth_pixels`。

## 参数

- `model`（默认 `yolo11n-seg.pt`）：YOLO 分割模型
- `confidence_threshold`（默认 0.5）
- `max_distance`（默认 10.0 米）：超出视为无效深度
- `color_topic` / `depth_topic` / `camera_info_topic`：默认对应 `/camera/camera/...`
- `publish_debug_image`（默认 true）

## 订阅示例（其他应用）

```python
from person_distance_msgs.msg import PersonDistanceArray

def cb(msg):
    for p in msg.persons:
        print(f'person {p.id}: {p.distance:.2f} m at ({p.position.x:.2f}, {p.position.y:.2f}, {p.position.z:.2f})')

node.create_subscription(PersonDistanceArray, '/person_distances', cb, 10)
```
