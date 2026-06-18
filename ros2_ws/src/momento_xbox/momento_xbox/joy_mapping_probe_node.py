import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


@dataclass(frozen=True)
class Step:
    key: str
    label: str
    kind: str
    instruction: str


BUTTON_STEPS = [
    Step('button_enable', 'A', 'button', 'press A, then release it'),
    Step('button_disarm', 'B', 'button', 'press B, then release it'),
    Step('button_arm', 'X', 'button', 'press X, then release it'),
    Step('button_mode_cycle', 'Y', 'button', 'press Y, then release it'),
    Step('button_assist', 'LB', 'button', 'press LB, then release it'),
    Step('button_clear_fault', 'RB', 'button', 'press RB, then release it'),
    Step('button_disable', 'Back/View', 'button', 'press Back/View, then release it'),
    Step('button_start_toggle', 'Start/Menu', 'button', 'press Start/Menu, then release it'),
]

AXIS_STEPS = [
    Step('axis_yaw', 'Left stick X', 'axis', 'push the LEFT stick RIGHT, then release it'),
    Step('axis_linear', 'Left stick Y', 'axis', 'push the LEFT stick UP/FORWARD, then release it'),
    Step('axis_roll', 'Right stick X', 'axis', 'push the RIGHT stick RIGHT, then release it'),
    Step('axis_pitch', 'Right stick Y', 'axis', 'push the RIGHT stick UP/FORWARD, then release it'),
    Step('axis_dpad_y_up', 'DPad up', 'axis_or_button', 'press DPAD UP, then release it'),
    Step('axis_dpad_y_down', 'DPad down', 'axis_or_button', 'press DPAD DOWN, then release it'),
]

STEPS = BUTTON_STEPS + AXIS_STEPS


class JoyMappingProbe(Node):

    def __init__(self):
        super().__init__('joy_mapping_probe')

        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('axis_threshold', 0.55)
        self.declare_parameter('release_threshold', 0.2)

        self.axis_threshold = float(self.get_parameter('axis_threshold').value)
        self.release_threshold = float(self.get_parameter('release_threshold').value)

        self.step_index = 0
        self.prev_buttons = []
        self.neutral_axes = []
        self.last_msg = None
        self.waiting_for_release = False
        self.results = {}
        self.axis_values = {}

        joy_topic = self.get_parameter('joy_topic').value
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)

        self.get_logger().info(f'Listening on {joy_topic}')
        self.get_logger().info('Keep all buttons/sticks released until the first prompt appears.')
        self.get_logger().info('Sequence: A, B, X, Y, LB, RB, Back/View, Start/Menu, '
                               'LeftX right, LeftY up, RightX right, RightY up, DPad up, DPad down.')

    def _on_joy(self, msg: Joy):
        if self.last_msg is None:
            self.last_msg = msg
            self.prev_buttons = list(msg.buttons)
            self.neutral_axes = list(msg.axes)
            self._prompt()
            return

        if self.step_index >= len(STEPS):
            return

        step = STEPS[self.step_index]

        if self.waiting_for_release:
            if self._is_released(msg):
                self.waiting_for_release = False
                self.neutral_axes = list(msg.axes)
                self.step_index += 1
                if self.step_index >= len(STEPS):
                    self._finish()
                else:
                    self._prompt()
            self.last_msg = msg
            self.prev_buttons = list(msg.buttons)
            return

        if step.kind == 'button':
            detected = self._detect_button_edge(msg)
            if detected is not None:
                self.results[step.key] = detected
                self.get_logger().info(f'{step.label}: button index {detected}')
                self.waiting_for_release = True
        elif step.kind == 'axis':
            detected = self._detect_axis_motion(msg)
            if detected is not None:
                axis, value = detected
                self.results[step.key] = axis
                self.axis_values[step.key] = value
                self.get_logger().info(f'{step.label}: axis index {axis}, value {value:+.3f}')
                self.waiting_for_release = True
        else:
            axis_detected = self._detect_axis_motion(msg)
            button_detected = self._detect_button_edge(msg)
            if axis_detected is not None:
                axis, value = axis_detected
                self.results[step.key] = axis
                self.axis_values[step.key] = value
                self.get_logger().info(f'{step.label}: axis index {axis}, value {value:+.3f}')
                self.waiting_for_release = True
            elif button_detected is not None:
                self.results[step.key] = f'button:{button_detected}'
                self.get_logger().info(f'{step.label}: button index {button_detected} '
                                       '(this DPad is reported as buttons, not axes)')
                self.waiting_for_release = True

        self.last_msg = msg
        self.prev_buttons = list(msg.buttons)

    def _prompt(self):
        step = STEPS[self.step_index]
        self.get_logger().info(f'[{self.step_index + 1}/{len(STEPS)}] {step.instruction}')

    def _detect_button_edge(self, msg: Joy):
        count = max(len(msg.buttons), len(self.prev_buttons))
        edges = []
        for idx in range(count):
            now = int(msg.buttons[idx]) if idx < len(msg.buttons) else 0
            prev = int(self.prev_buttons[idx]) if idx < len(self.prev_buttons) else 0
            if now == 1 and prev == 0:
                edges.append(idx)

        if len(edges) == 1:
            return edges[0]
        if len(edges) > 1:
            self.get_logger().warn(f'Multiple button edges detected {edges}; press only one control.')
        return None

    def _detect_axis_motion(self, msg: Joy):
        count = min(len(msg.axes), len(self.neutral_axes))
        candidates = []
        for idx in range(count):
            value = float(msg.axes[idx])
            base = float(self.neutral_axes[idx])
            delta = value - base
            if math.isfinite(delta) and abs(delta) >= self.axis_threshold:
                candidates.append((idx, value, abs(delta)))

        if len(candidates) == 1:
            idx, value, _ = candidates[0]
            return idx, value
        if len(candidates) > 1:
            candidates.sort(key=lambda item: item[2], reverse=True)
            idx, value, _ = candidates[0]
            self.get_logger().warn(
                f'Multiple axes moved; using strongest axis {idx} value {value:+.3f}')
            return idx, value
        return None

    def _is_released(self, msg: Joy):
        buttons_released = all(int(button) == 0 for button in msg.buttons)
        axis_count = min(len(msg.axes), len(self.neutral_axes))
        axes_released = all(
            abs(float(msg.axes[idx]) - float(self.neutral_axes[idx])) <= self.release_threshold
            for idx in range(axis_count)
        )
        return buttons_released and axes_released

    def _axis_sign_param(self, key: str) -> float:
        value = self.axis_values.get(key, 0.0)
        return 1.0 if value >= 0.0 else -1.0

    def _finish(self):
        self.get_logger().info('Mapping complete.')

        lines = [
            'Recommended momento_xbox parameters:',
            f"  'button_enable': {self.results.get('button_enable')},",
            f"  'button_disarm': {self.results.get('button_disarm')},",
            f"  'button_arm': {self.results.get('button_arm')},",
            f"  'button_mode_cycle': {self.results.get('button_mode_cycle')},",
            f"  'button_assist': {self.results.get('button_assist')},",
            f"  'button_clear_fault': {self.results.get('button_clear_fault')},",
            f"  'button_disable': {self.results.get('button_disable')},",
            f"  'button_start_toggle': {self.results.get('button_start_toggle')},",
            f"  'axis_linear': {self.results.get('axis_linear')},",
            f"  'axis_yaw': {self.results.get('axis_yaw')},",
            f"  'axis_roll': {self.results.get('axis_roll')},",
            f"  'axis_pitch': {self.results.get('axis_pitch')},",
        ]

        dpad_up = self.results.get('axis_dpad_y_up')
        dpad_down = self.results.get('axis_dpad_y_down')
        if isinstance(dpad_up, int) and dpad_up == dpad_down:
            lines.append(f"  'axis_dpad_y': {dpad_up},")
            up_value = self.axis_values.get('axis_dpad_y_up', 0.0)
            if up_value < 0.0:
                lines.append('  # Warning: DPad up is negative; current xbox_mode_node will '
                             'decrease leg on DPad up.')
        else:
            lines.append(f'  # DPad up detected as {dpad_up}, down detected as {dpad_down}.')
            lines.append('  # Current xbox_mode_node expects DPad Y to be one axis.')

        lines.extend([
            f"  'linear_sign': {self._axis_sign_param('axis_linear'):.1f},",
            f"  'yaw_sign': {self._axis_sign_param('axis_yaw'):.1f},",
            f"  'roll_sign': {self._axis_sign_param('axis_roll'):.1f},",
            f"  'pitch_sign': {self._axis_sign_param('axis_pitch'):.1f},",
            '',
            'Raw axis values for the requested positive directions:',
        ])

        for key in ('axis_yaw', 'axis_linear', 'axis_roll', 'axis_pitch',
                    'axis_dpad_y_up', 'axis_dpad_y_down'):
            if key in self.axis_values:
                lines.append(f'  {key}: {self.axis_values[key]:+.3f}')

        for line in lines:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = JoyMappingProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
