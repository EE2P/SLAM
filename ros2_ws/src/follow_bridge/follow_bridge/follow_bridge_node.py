"""ROS 2 follow bridge: follow the nearest person and publish a follow velocity.

Subscribes the Jetson detector's `/person_distances`
(person_distance_msgs/PersonDistanceArray), picks the NEAREST valid person each frame
via target_selector (no identity lock), runs the metric follow law
(follow_bridge.metric_follow_law), and publishes:

    /cmd_vel                  geometry_msgs/Twist   linear.x (m/s) + angular.z (rad/s)
    /follow_bridge/heartbeat  std_msgs/Header       liveness stamp; frame_id = state

and subscribes a latched e-stop:

    /follow_bridge/estop      std_msgs/Bool         true -> publish zero velocity

This node is pure FOLLOWING INTENT. A separate robot driver (e.g. the Momento
/cmd_vel consumer) turns the Twist into wheel motion and owns wheel mixing, arming,
balance and the USB link. Runs on the Jetson beside person_distance_node (ROS 2
Jazzy); it has NO Pi/Hailo imports so the existing follower is untouched.

The control law and selector are pure Python and unit-tested without ROS; this node
is just the ROS adaptor (params, subs, a fixed-rate publish timer) around them.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Header
from person_distance_msgs.msg import PersonDistanceArray

from follow_bridge.metric_follow_law import MetricFollowParams, compute_metric_cmd
from follow_bridge.target_selector import (
    Candidate, TargetSelector, FOLLOW, SEARCH,
)


class FollowBridgeNode(Node):

    def __init__(self):
        super().__init__('follow_bridge')

        # -- topics & wiring params --
        person_topic = self.declare_parameter('person_topic', '/person_distances').value
        cmd_vel_topic = self.declare_parameter('cmd_vel_topic', '/cmd_vel').value
        heartbeat_topic = self.declare_parameter(
            'heartbeat_topic', '/follow_bridge/heartbeat').value
        estop_topic = self.declare_parameter('estop_topic', '/follow_bridge/estop').value
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 30.0).value)
        # Detections older than this are treated as "no candidates" so a dead detector
        # makes the selector search->idle instead of acting on a frozen frame.
        self.detection_timeout_s = float(
            self.declare_parameter('detection_timeout_s', 0.5).value)
        # Chassis sign flips applied at the /cmd_vel boundary, mirroring
        # hailo_follower.py's DRIVE_SIGN/YAW_SIGN. yaw_sign defaults to -1.0 to map the
        # law's "yaw-right-positive" onto REP-103 (+angular.z = left). Confirm on-robot.
        self.drive_sign = float(self.declare_parameter('drive_sign', -1.0).value)
        self.yaw_sign = float(self.declare_parameter('yaw_sign', -1.0).value)

        # -- metric follow-law params (one ROS param per MetricFollowParams field) --
        self.p = self._declare_metric_params()

        # -- target selector (follow nearest, no lock) --
        self.selector = TargetSelector(
            conf_min=self.p.CONF_MIN,
            max_range_m=self.p.MAX_RANGE_M,
            min_valid_depth_pixels=self.p.MIN_VALID_DEPTH_PIXELS,
            search_timeout_s=float(self.declare_parameter('search_timeout_s', 2.0).value),
        )

        self._latest = []           # list[Candidate] from the most recent message
        self._latest_t = None       # node-clock seconds of that message
        self._estop = False

        self.pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pub_hb = self.create_publisher(Header, heartbeat_topic, 10)
        self.create_subscription(
            PersonDistanceArray, person_topic, self._on_persons, qos_profile_sensor_data)
        # Latched e-stop so a late-starting bridge picks up a standing stop.
        estop_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, estop_topic, self._on_estop, estop_qos)

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'follow_bridge up: sub {person_topic} -> pub {cmd_vel_topic} '
            f'(desired={self.p.DESIRED_DISTANCE_M:.2f} m, rate={self.publish_rate_hz:.0f} Hz)')

    # -- params --------------------------------------------------------------
    def _declare_metric_params(self) -> MetricFollowParams:
        defaults = MetricFollowParams()
        kwargs = {}
        for name in MetricFollowParams.field_names():
            default = getattr(defaults, name)
            value = self.declare_parameter(name.lower(), default).value
            # MIN_VALID_DEPTH_PIXELS is the only int field; everything else is float.
            kwargs[name] = int(value) if isinstance(default, int) else float(value)
        return MetricFollowParams(**kwargs)

    # -- time ----------------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- subscriptions -------------------------------------------------------
    def _on_persons(self, msg: PersonDistanceArray):
        cands = []
        for person in msg.persons:
            # track_id is telemetry only now (selection is by nearest distance), so a
            # missing field is harmless -> default to -1 without warning.
            track_id = getattr(person, 'track_id', -1)
            cands.append(Candidate(
                track_id=int(track_id if track_id is not None else -1),
                distance=float(person.distance),
                pos_x=float(person.position.x),
                pos_z=float(person.position.z),
                pixel_x=float(person.pixel_x),
                confidence=float(person.confidence),
                valid_depth_pixels=int(person.valid_depth_pixels),
            ))
        self._latest = cands
        self._latest_t = self._now_s()

    def _on_estop(self, msg: Bool):
        if bool(msg.data) != self._estop:
            self.get_logger().warn(f'e-stop {"ASSERTED" if msg.data else "cleared"}')
        self._estop = bool(msg.data)

    # -- main loop -----------------------------------------------------------
    def _on_timer(self):
        now = self._now_s()

        if self._estop:
            self._publish(0.0, 0.0, 'estop')
            return

        fresh = (
            self._latest_t is not None
            and (now - self._latest_t) <= self.detection_timeout_s
        )
        candidates = self._latest if fresh else []

        decision = self.selector.update(candidates, now)
        if decision.action == FOLLOW and decision.target is not None:
            t = decision.target
            linear, yaw = compute_metric_cmd(
                t.distance, t.pos_x, t.pos_z,
                pixel_x=t.pixel_x, valid_depth_pixels=t.valid_depth_pixels, p=self.p)
        elif decision.action == SEARCH:
            linear, yaw = 0.0, self.p.RECOVERY_YAW * decision.last_seen_sign
        else:  # IDLE -> hold position
            linear, yaw = 0.0, 0.0

        self._publish(linear, yaw, decision.state)

    def _publish(self, linear: float, yaw: float, state: str):
        tw = Twist()
        tw.linear.x = self.drive_sign * float(linear)
        tw.angular.z = self.yaw_sign * float(yaw)
        self.pub_cmd.publish(tw)

        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        hb.frame_id = state
        self.pub_hb.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = FollowBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
