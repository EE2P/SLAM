import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from momento_msgs.msg import MomentoCommand, MomentoMode


class MomentoCommandMux(Node):

    def __init__(self):
        super().__init__('momento_command_mux')

        self.declare_parameter('mode_topic', '/momento/mode')
        self.declare_parameter('xbox_cmd_topic', '/momento/xbox_cmd')
        self.declare_parameter('auto_cmd_topic', '/momento/auto_cmd')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/momento/cmd')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('xbox_timeout_s', 0.25)
        self.declare_parameter('auto_timeout_s', 0.35)
        self.declare_parameter('default_leg', 0.07)

        self.default_leg = float(self.get_parameter('default_leg').value)
        self.xbox_timeout_s = float(self.get_parameter('xbox_timeout_s').value)
        self.auto_timeout_s = float(self.get_parameter('auto_timeout_s').value)

        self.mode = MomentoMode.MODE_MANUAL
        self.start_enabled = False

        self.xbox_cmd = self._safe_command(MomentoCommand.SOURCE_XBOX)
        self.xbox_t = 0.0
        self.auto_cmd = self._safe_command(MomentoCommand.SOURCE_AUTO)
        self.auto_t = 0.0
        self.cmd_vel_cmd = self._safe_command(MomentoCommand.SOURCE_AUTO)
        self.cmd_vel_t = 0.0

        self.pub = self.create_publisher(
            MomentoCommand, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            MomentoMode, self.get_parameter('mode_topic').value, self._on_mode, 10)
        self.create_subscription(
            MomentoCommand, self.get_parameter('xbox_cmd_topic').value, self._on_xbox, 10)
        self.create_subscription(
            MomentoCommand, self.get_parameter('auto_cmd_topic').value, self._on_auto, 10)
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd_vel, 10)

        period = 1.0 / max(1.0, float(self.get_parameter('publish_rate_hz').value))
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            'momento_command_mux: MANUAL/AUTO/ASSIST/ESTOP -> /momento/cmd')

    def _safe_command(self, source: int) -> MomentoCommand:
        cmd = MomentoCommand()
        cmd.linear = 0.0
        cmd.yaw = 0.0
        cmd.roll = 0.0
        cmd.pitch = 0.0
        cmd.leg = self.default_leg if hasattr(self, 'default_leg') else 0.12
        cmd.start = False
        cmd.yaw_mode = MomentoCommand.YAW_RATE
        cmd.source = source
        return cmd

    def _stamp(self, cmd: MomentoCommand) -> MomentoCommand:
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'momento_mux'
        return cmd

    def _on_mode(self, msg: MomentoMode):
        if msg.mode != self.mode:
            self.get_logger().info(f'mode -> {self._mode_name(msg.mode)}')
        self.mode = msg.mode
        self.start_enabled = bool(msg.start)

    def _on_xbox(self, msg: MomentoCommand):
        self.xbox_cmd = msg
        self.xbox_t = time.monotonic()

    def _on_auto(self, msg: MomentoCommand):
        self.auto_cmd = msg
        self.auto_t = time.monotonic()

    def _on_cmd_vel(self, msg: Twist):
        cmd = MomentoCommand()
        cmd.linear = float(msg.linear.x)
        cmd.yaw = float(msg.angular.z)
        cmd.roll = 0.0
        cmd.pitch = 0.0
        cmd.leg = self.default_leg
        cmd.start = True
        cmd.yaw_mode = MomentoCommand.YAW_RATE
        cmd.source = MomentoCommand.SOURCE_AUTO
        self.cmd_vel_cmd = cmd
        self.cmd_vel_t = time.monotonic()

    def _fresh_xbox(self, now: float) -> bool:
        return self.xbox_t > 0.0 and (now - self.xbox_t) <= self.xbox_timeout_s

    def _fresh_auto_cmd(self, now: float) -> bool:
        return self.auto_t > 0.0 and (now - self.auto_t) <= self.auto_timeout_s

    def _fresh_cmd_vel(self, now: float) -> bool:
        return self.cmd_vel_t > 0.0 and (now - self.cmd_vel_t) <= self.auto_timeout_s

    def _select_auto(self, now: float):
        if self._fresh_auto_cmd(now):
            return self.auto_cmd
        if self._fresh_cmd_vel(now):
            return self.cmd_vel_cmd
        return None

    def _with_global_start(self, cmd: MomentoCommand, source: int) -> MomentoCommand:
        out = MomentoCommand()
        out.linear = cmd.linear
        out.yaw = cmd.yaw
        out.roll = cmd.roll
        out.pitch = cmd.pitch
        out.leg = cmd.leg if cmd.leg > 0.0 else self.default_leg
        out.start = self.start_enabled and bool(cmd.start)
        out.yaw_mode = cmd.yaw_mode
        out.source = source
        return self._stamp(out)

    def _on_timer(self):
        now = time.monotonic()

        if self.mode == MomentoMode.MODE_ESTOP:
            self.pub.publish(self._stamp(self._safe_command(MomentoCommand.SOURCE_ESTOP)))
            return

        if self.mode == MomentoMode.MODE_MANUAL:
            if self._fresh_xbox(now):
                self.pub.publish(self._with_global_start(
                    self.xbox_cmd, MomentoCommand.SOURCE_XBOX))
            else:
                self.pub.publish(self._stamp(self._safe_command(MomentoCommand.SOURCE_XBOX)))
            return

        auto = self._select_auto(now)
        if self.mode == MomentoMode.MODE_AUTO:
            if auto is None:
                self.pub.publish(self._stamp(self._safe_command(MomentoCommand.SOURCE_AUTO)))
            else:
                self.pub.publish(self._with_global_start(auto, MomentoCommand.SOURCE_AUTO))
            return

        if self.mode == MomentoMode.MODE_ASSIST:
            out = MomentoCommand()
            if auto is not None:
                out.linear = auto.linear
                out.yaw = auto.yaw
                out.yaw_mode = auto.yaw_mode
                auto_start = auto.start
            else:
                out.linear = 0.0
                out.yaw = 0.0
                out.yaw_mode = MomentoCommand.YAW_RATE
                auto_start = True

            if self._fresh_xbox(now):
                out.roll = self.xbox_cmd.roll
                out.pitch = self.xbox_cmd.pitch
                out.leg = self.xbox_cmd.leg
            else:
                out.roll = 0.0
                out.pitch = 0.0
                out.leg = self.default_leg

            out.start = self.start_enabled and auto_start
            out.source = MomentoCommand.SOURCE_ASSIST
            self.pub.publish(self._stamp(out))
            return

        self.pub.publish(self._stamp(self._safe_command(MomentoCommand.SOURCE_ESTOP)))

    def _mode_name(self, mode: int) -> str:
        names = {
            MomentoMode.MODE_MANUAL: 'MANUAL',
            MomentoMode.MODE_AUTO: 'AUTO',
            MomentoMode.MODE_ASSIST: 'ASSIST',
            MomentoMode.MODE_ESTOP: 'ESTOP',
        }
        return names.get(mode, f'UNKNOWN({mode})')


def main(args=None):
    rclpy.init(args=args)
    node = MomentoCommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
