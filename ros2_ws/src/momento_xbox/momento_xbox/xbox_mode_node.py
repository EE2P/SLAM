import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from momento_msgs.msg import MomentoCommand, MomentoMode
from momento_msgs.srv import MomentoControl


def clamp(value: float, lo: float, hi: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(lo, min(hi, value))


class XboxModeNode(Node):

    def __init__(self):
        super().__init__('momento_xbox')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('mode_topic', '/momento/mode')
        self.declare_parameter('cmd_topic', '/momento/xbox_cmd')
        self.declare_parameter('control_service', '/momento/control')

        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_yaw', 0)
        self.declare_parameter('axis_roll', 2)
        self.declare_parameter('axis_pitch', 3)
        self.declare_parameter('axis_dpad_y', 7)

        self.declare_parameter('button_enable', 0)      # A
        self.declare_parameter('button_disarm', 1)      # B
        self.declare_parameter('button_arm', 3)         # X
        self.declare_parameter('button_mode_cycle', 4)  # Y
        self.declare_parameter('button_assist', 6)      # LB
        self.declare_parameter('button_clear_fault', 7) # RB
        self.declare_parameter('button_disable', 10)    # Back/View
        self.declare_parameter('button_start_toggle', 11) # Start/Menu

        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_yaw_rate', 1.1)
        self.declare_parameter('max_roll', 0.25)
        self.declare_parameter('max_pitch', 0.25)
        self.declare_parameter('linear_sign', -1.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('roll_sign', -1.0)
        self.declare_parameter('pitch_sign', 1.0)
        self.declare_parameter('default_leg', 0.07)
        self.declare_parameter('min_leg', 0.068)
        self.declare_parameter('max_leg', 0.21)
        self.declare_parameter('leg_step', 0.005)

        self.axis_linear = int(self.get_parameter('axis_linear').value)
        self.axis_yaw = int(self.get_parameter('axis_yaw').value)
        self.axis_roll = int(self.get_parameter('axis_roll').value)
        self.axis_pitch = int(self.get_parameter('axis_pitch').value)
        self.axis_dpad_y = int(self.get_parameter('axis_dpad_y').value)

        self.button_enable = int(self.get_parameter('button_enable').value)
        self.button_disarm = int(self.get_parameter('button_disarm').value)
        self.button_arm = int(self.get_parameter('button_arm').value)
        self.button_mode_cycle = int(self.get_parameter('button_mode_cycle').value)
        self.button_assist = int(self.get_parameter('button_assist').value)
        self.button_clear_fault = int(self.get_parameter('button_clear_fault').value)
        self.button_disable = int(self.get_parameter('button_disable').value)
        self.button_start_toggle = int(self.get_parameter('button_start_toggle').value)

        self.deadzone = float(self.get_parameter('deadzone').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.max_roll = float(self.get_parameter('max_roll').value)
        self.max_pitch = float(self.get_parameter('max_pitch').value)
        self.linear_sign = float(self.get_parameter('linear_sign').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        self.roll_sign = float(self.get_parameter('roll_sign').value)
        self.pitch_sign = float(self.get_parameter('pitch_sign').value)
        self.leg = float(self.get_parameter('default_leg').value)
        self.min_leg = float(self.get_parameter('min_leg').value)
        self.max_leg = float(self.get_parameter('max_leg').value)
        self.leg_step = float(self.get_parameter('leg_step').value)

        self.mode = MomentoMode.MODE_MANUAL
        self.start_enabled = False
        self.prev_buttons = []
        self.prev_dpad_y = 0

        self.pub_mode = self.create_publisher(
            MomentoMode, self.get_parameter('mode_topic').value, 10)
        self.pub_cmd = self.create_publisher(
            MomentoCommand, self.get_parameter('cmd_topic').value, 10)
        self.create_subscription(
            Joy, self.get_parameter('joy_topic').value, self._on_joy, 10)

        self.control_client = self.create_client(
            MomentoControl, self.get_parameter('control_service').value)

        self.get_logger().info(
            'momento_xbox: A enable, X arm, B disarm, Back disable/ESTOP, '
            'Start toggles start, Y cycles MANUAL/AUTO/ASSIST')

    def _axis(self, msg: Joy, idx: int) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        value = float(msg.axes[idx])
        return 0.0 if abs(value) < self.deadzone else value

    def _button(self, msg: Joy, idx: int) -> int:
        if idx < 0 or idx >= len(msg.buttons):
            return 0
        return int(msg.buttons[idx])

    def _edge(self, msg: Joy, idx: int) -> bool:
        now = self._button(msg, idx)
        prev = self.prev_buttons[idx] if 0 <= idx < len(self.prev_buttons) else 0
        return now == 1 and prev == 0

    def _cycle_mode(self):
        if self.mode == MomentoMode.MODE_MANUAL:
            self.mode = MomentoMode.MODE_AUTO
        elif self.mode == MomentoMode.MODE_AUTO:
            self.mode = MomentoMode.MODE_ASSIST
        else:
            self.mode = MomentoMode.MODE_MANUAL
        self.get_logger().info(f'mode -> {self._mode_name(self.mode)}')

    def _call_control(self, command: int, joint_idx: int = 0xFF):
        if not self.control_client.service_is_ready():
            self.get_logger().warn('control service not ready')
            return
        req = MomentoControl.Request()
        req.command = command
        req.joint_idx = joint_idx
        self.control_client.call_async(req)

    def _publish_mode(self):
        msg = MomentoMode()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'xbox'
        msg.mode = self.mode
        msg.start = self.start_enabled and self.mode != MomentoMode.MODE_ESTOP
        self.pub_mode.publish(msg)

    def _publish_cmd(self, joy: Joy):
        cmd = MomentoCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'xbox'
        cmd.linear = self.linear_sign * self._axis(joy, self.axis_linear) * self.max_linear
        cmd.yaw = self.yaw_sign * self._axis(joy, self.axis_yaw) * self.max_yaw_rate
        cmd.roll = self.roll_sign * self._axis(joy, self.axis_roll) * self.max_roll
        cmd.pitch = self.pitch_sign * self._axis(joy, self.axis_pitch) * self.max_pitch
        cmd.leg = self.leg
        cmd.start = self.start_enabled
        cmd.yaw_mode = MomentoCommand.YAW_RATE
        cmd.source = MomentoCommand.SOURCE_XBOX
        self.pub_cmd.publish(cmd)

    def _on_joy(self, msg: Joy):
        if self._edge(msg, self.button_enable):
            self._call_control(MomentoControl.Request.CMD_ENABLE)
        if self._edge(msg, self.button_arm):
            self._call_control(MomentoControl.Request.CMD_ARM)
        if self._edge(msg, self.button_disarm):
            self.start_enabled = False
            self._call_control(MomentoControl.Request.CMD_DISARM)
        if self._edge(msg, self.button_disable):
            self.start_enabled = False
            self.mode = MomentoMode.MODE_ESTOP
            self._call_control(MomentoControl.Request.CMD_DISABLE)
            self.get_logger().warn('ESTOP from Xbox Back button')
        if self._edge(msg, self.button_clear_fault):
            if self.mode == MomentoMode.MODE_ESTOP:
                self.mode = MomentoMode.MODE_MANUAL
            self._call_control(MomentoControl.Request.CMD_CLEAR_FAULT)
            self.get_logger().info('clear fault / ESTOP cleared to MANUAL')
        if self._edge(msg, self.button_start_toggle):
            self.start_enabled = not self.start_enabled
            self.get_logger().info(f'start -> {self.start_enabled}')
        if self._edge(msg, self.button_mode_cycle):
            self._cycle_mode()
        if self._edge(msg, self.button_assist):
            self.mode = MomentoMode.MODE_ASSIST
            self.get_logger().info('mode -> ASSIST')

        dpad_y = 0
        if 0 <= self.axis_dpad_y < len(msg.axes):
            dpad_y = 1 if msg.axes[self.axis_dpad_y] > 0.5 else (-1 if msg.axes[self.axis_dpad_y] < -0.5 else 0)
        if dpad_y != 0 and self.prev_dpad_y == 0:
            self.leg = clamp(self.leg + dpad_y * self.leg_step, self.min_leg, self.max_leg)
            self.get_logger().info(f'leg -> {self.leg:.3f} m')
        self.prev_dpad_y = dpad_y

        self._publish_mode()
        self._publish_cmd(msg)
        self.prev_buttons = list(msg.buttons)

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
    node = XboxModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
