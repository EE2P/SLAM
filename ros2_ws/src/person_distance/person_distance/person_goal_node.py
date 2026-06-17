"""P6: turn a detected person into a Nav2 goal (global-navigation "follow").

Subscribes to /person_distances (from person_distance_node), picks the nearest
person with a valid metric distance, transforms their 3D position from the camera
optical frame into `map`, and publishes a PoseStamped goal a fixed stand-off
distance SHORT of the person so the robot follows instead of driving into them.

This replaces the old monocular pixel/box-area follower: Nav2 + the STVL 3D voxel
layer now plan a collision-free path to the person, so the robot avoids obstacles
between it and the target.

Design choices (knobs at the top of __init__):
  * Target = smallest valid `distance` (closest person). No persistent track ID
    yet -- if you need to lock one person and ignore newcomers, add tracking in
    person_distance_node (ultralytics .track()).
  * Throttled: a new goal is only sent when the target moved > `min_goal_shift_m`
    or `min_goal_period_s` has elapsed, so Nav2 is not spammed every frame.
  * NaN / no-person / no-TF -> hold (do not publish), never send a garbage goal.

Run:
  ros2 run person_distance person_goal_node
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  registers do_transform for PointStamped

from person_distance_msgs.msg import PersonDistanceArray


class PersonGoalNode(Node):

    def __init__(self):
        super().__init__('person_goal')

        self.declare_parameter('persons_topic', '/person_distances')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('standoff_distance', 1.0)   # stop this far short (m)
        self.declare_parameter('min_goal_shift_m', 0.3)    # resend if target moved this much
        self.declare_parameter('min_goal_period_s', 1.0)   # or at least this often
        self.declare_parameter('max_target_distance', 8.0)  # ignore people farther than this

        self.target_frame = self.get_parameter('target_frame').value
        self.standoff = float(self.get_parameter('standoff_distance').value)
        self.min_shift = float(self.get_parameter('min_goal_shift_m').value)
        self.min_period = float(self.get_parameter('min_goal_period_s').value)
        self.max_dist = float(self.get_parameter('max_target_distance').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_goal = self.create_publisher(
            PoseStamped, self.get_parameter('goal_topic').value, 10)
        self.sub = self.create_subscription(
            PersonDistanceArray, self.get_parameter('persons_topic').value,
            self._on_persons, 10)

        self._last_goal_xy = None
        self._last_goal_time = None
        self.get_logger().info(
            f'person_goal: nearest valid person -> {self.target_frame}, '
            f'standoff {self.standoff:.2f} m')

    def _on_persons(self, msg: PersonDistanceArray):
        # 1. Pick the nearest person with a usable metric distance.
        best = None
        for p in msg.persons:
            d = p.distance
            if d is None or math.isnan(d) or d <= 0.0 or d > self.max_dist:
                continue
            if best is None or d < best.distance:
                best = p
        if best is None:
            return  # nobody usable this frame -> hold

        # 2. Person position is in the color image's optical frame (msg.header).
        pt = PointStamped()
        pt.header = msg.header
        pt.point.x = best.position.x
        pt.point.y = best.position.y
        pt.point.z = best.position.z

        # 3. Transform into the target (map) frame. Hold if TF not ready.
        try:
            pt_map = self.tf_buffer.transform(
                pt, self.target_frame, timeout=Duration(seconds=0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
            self.get_logger().warn(f'TF {pt.header.frame_id}->{self.target_frame} '
                                   f'failed: {e}', throttle_duration_sec=2.0)
            return

        gx, gy = pt_map.point.x, pt_map.point.y

        # 4. Stand-off: pull the goal back toward the robot along the robot->person
        #    line so we follow at distance instead of colliding.
        gx, gy, yaw = self._apply_standoff(gx, gy)

        # 5. Throttle: only send when meaningfully different / not too frequent.
        now = self.get_clock().now()
        if not self._should_send(gx, gy, now):
            return

        goal = PoseStamped()
        goal.header.stamp = now.to_msg()
        goal.header.frame_id = self.target_frame
        goal.pose.position.x = gx
        goal.pose.position.y = gy
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_goal.publish(goal)
        self._last_goal_xy = (gx, gy)
        self._last_goal_time = now
        self.get_logger().info(f'goal -> ({gx:.2f}, {gy:.2f}) in {self.target_frame}')

    def _apply_standoff(self, gx, gy):
        """Return a goal `standoff` metres before the person, facing them."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except tf2_ros.TransformException:
            return gx, gy, 0.0  # no robot pose -> goal at the person, yaw 0

        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)  # face the person
        if dist <= self.standoff:
            return rx, ry, yaw     # already within stand-off -> stay put, just face
        scale = (dist - self.standoff) / dist
        return rx + dx * scale, ry + dy * scale, yaw

    def _should_send(self, gx, gy, now):
        if self._last_goal_xy is None or self._last_goal_time is None:
            return True
        moved = math.hypot(gx - self._last_goal_xy[0], gy - self._last_goal_xy[1])
        elapsed = (now - self._last_goal_time).nanoseconds * 1e-9
        return moved >= self.min_shift or elapsed >= self.min_period


def main(args=None):
    rclpy.init(args=args)
    node = PersonGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
