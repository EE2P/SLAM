# follow_bridge

Follows the **nearest person** seen by the Jetson `person_distance` detector and
publishes a **metric follow velocity** for a robot driver to realise. It is pure
*following intent* — it does **not** do wheel mixing, arming, or balance. That keeps
the vision loop decoupled from the robot's real-time/USB concerns and makes `/cmd_vel`
the clean graft point between the depth detector and the wheel-legged **Momento** robot.

```
person_distance_node ──▶ /person_distances ──▶ [follow_bridge] ──▶ /cmd_vel ──▶ [robot driver] ──▶ wheels
   (Jetson, +track_id)                          (this package)     (Twist)      (Momento, separate repo)
```

## Topics

| Dir | Topic | Type | Notes |
|-----|-------|------|-------|
| sub | `/person_distances` | `person_distance_msgs/PersonDistanceArray` | needs the `track_id` field (see below) |
| sub | `/follow_bridge/estop` | `std_msgs/Bool` (latched) | `true` → publish zero velocity |
| pub | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` m/s, `angular.z` rad/s (REP-103) |
| pub | `/follow_bridge/heartbeat` | `std_msgs/Header` | liveness stamp; `frame_id` = selector state |

The Twist is published at a fixed `publish_rate_hz` regardless of detection cadence, so
the downstream driver can treat a stale heartbeat as "bridge dead → stop".

## Behaviour

- **Metric control** (`follow_bridge.metric_follow_law`, vendored into this package): holds `desired_distance_m`
  (meters) using real depth; yaw from the target's bearing `atan2(pos_x, pos_z)`. Forward
  is gated until the target is roughly centred; reverse is always allowed. If depth is
  missing/untrusted it falls back to pixel-offset yaw only (no forward).
- **Target selection** (`follow_bridge.target_selector`, "nearest person"): each frame
  follows the **nearest valid person** (no identity lock); switches to a closer person
  if one appears. When nobody valid is visible it does a brief search-yaw toward the
  last-seen side for `search_timeout_s`, then holds. No permanent stop — following
  resumes the instant a valid person reappears.

## Key parameters

`desired_distance_m` (1.5), `stop_band_m` (0.2), `kp_linear_m`, `kp_yaw_bearing`,
`max_linear`/`max_reverse`/`max_angular`, `recovery_yaw`, `conf_min`,
`min_valid_depth_pixels`, `max_range_m`, `search_timeout_s` (2.0),
`drive_sign` (**-1.0** — flips law-forward to chassis-forward; confirm on-robot),
`yaw_sign` (**-1.0** — maps the law's yaw-right-positive to REP-103;
confirm on-robot), `cmd_vel_topic`, `person_topic`, `publish_rate_hz`.

## Build & run (on the Jetson)

The control law (`metric_follow_law.py`) is vendored into this package, so it has **no
sim-mvp / stalker dependency** — just build the SLAM workspace and run:

```bash
cd ~/SLAM/ros2_ws
colcon build --packages-select person_distance_msgs person_distance follow_bridge
source install/setup.bash
ros2 launch follow_bridge follow_bridge.launch.py desired_distance_m:=2.0
# watch the output:
ros2 topic echo /cmd_vel
```

## Coordinated changes

1. **Detector `track_id` (this repo — DONE alongside this package).** `follow_bridge`
   cannot lock without it.
   - `person_distance_msgs/msg/PersonDistance.msg`: `int32 track_id` (`-1` = untracked).
   - `person_distance_node.py`: runs `model.track(..., persist=True, tracker='bytetrack.yaml')`
     and sets `person.track_id` from `result.boxes.id`.
   If a build still lacks the field, the node logs a warning once and follows nobody.

2. **Robot driver — a `/cmd_vel` consumer (separate repo, out of scope here).** Subscribes
   `/cmd_vel` + heartbeat; mixes `linear.x`/`angular.z` into left/right wheel rad/s; owns the
   enable→arm sequence and a USB watchdog; treats a stale heartbeat as "bridge dead → stop".
   Needs measured wheel radius & track width.

## Tests

Pure-Python, no ROS/robot needed:

```bash
python -m pytest ros2_ws/src/follow_bridge/test/ -q
```
