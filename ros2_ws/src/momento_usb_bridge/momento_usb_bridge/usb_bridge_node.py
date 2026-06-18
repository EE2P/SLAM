import math
import struct
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from momento_msgs.msg import MomentoCommand, MomentoState
from momento_msgs.srv import MomentoControl

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - surfaced at runtime on the robot
    serial = None
    list_ports = None


BALANCE_HDR = b'\xBA\x1A'
CTRL_HDR = b'\xCC\x33'
STATE_HDR = b'\xAA\x55'
BALANCE_FLOATS = 5
BALANCE_PKT_SIZE = 2 + BALANCE_FLOATS * 4 + 1 + 1
STATE_PKT_SIZE = 2 + 1 + 6 * 4 + 6 * 4 + 6 + 1


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def clamp(value: float, lo: float, hi: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(lo, min(hi, value))


class MomentoUsbBridge(Node):

    def __init__(self):
        super().__init__('momento_usb_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('cmd_timeout_s', 0.2)
        self.declare_parameter('reconnect_period_s', 1.0)
        self.declare_parameter('default_leg', 0.07)
        self.declare_parameter('min_linear', -1.0)
        self.declare_parameter('max_linear', 1.0)
        self.declare_parameter('max_yaw_rate', 2.5)
        self.declare_parameter('max_roll', 0.4)
        self.declare_parameter('max_pitch', 0.4)
        self.declare_parameter('min_leg', 0.068)
        self.declare_parameter('max_leg', 0.21)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.cmd_timeout_s = float(self.get_parameter('cmd_timeout_s').value)
        self.reconnect_period_s = float(self.get_parameter('reconnect_period_s').value)
        self.default_leg = float(self.get_parameter('default_leg').value)

        self.min_linear = float(self.get_parameter('min_linear').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_yaw_rate = abs(float(self.get_parameter('max_yaw_rate').value))
        self.max_roll = abs(float(self.get_parameter('max_roll').value))
        self.max_pitch = abs(float(self.get_parameter('max_pitch').value))
        self.min_leg = float(self.get_parameter('min_leg').value)
        self.max_leg = float(self.get_parameter('max_leg').value)

        self._serial = None
        self._serial_lock = threading.Lock()
        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_buf = bytearray()

        self._latest_cmd = self._safe_command()
        self._latest_cmd_t = 0.0
        self._last_send_t = time.monotonic()
        self._last_connect_try_t = 0.0
        self._turn_set = 0.0
        self._tx_count = 0
        self._rx_count = 0
        self._crc_errors = 0
        self._last_connected = False

        self.pub_state = self.create_publisher(MomentoState, '/momento/state', 10)
        self.create_subscription(MomentoCommand, '/momento/cmd', self._on_command, 10)
        self.create_service(MomentoControl, '/momento/control', self._on_control)

        period = 1.0 / max(1.0, self.rate_hz)
        self.create_timer(period, self._on_timer)
        self._rx_thread.start()

        self.get_logger().info(
            f'momento_usb_bridge: {self.port or "auto"} at {self.rate_hz:.0f} Hz')

    def _safe_command(self) -> MomentoCommand:
        cmd = MomentoCommand()
        cmd.linear = 0.0
        cmd.yaw = 0.0
        cmd.roll = 0.0
        cmd.pitch = 0.0
        cmd.leg = self.default_leg if hasattr(self, 'default_leg') else 0.12
        cmd.start = False
        cmd.yaw_mode = MomentoCommand.YAW_RATE
        cmd.source = MomentoCommand.SOURCE_ESTOP
        return cmd

    def _on_command(self, msg: MomentoCommand):
        self._latest_cmd = msg
        self._latest_cmd_t = time.monotonic()

    def _on_control(self, request: MomentoControl.Request, response: MomentoControl.Response):
        pkt_body = CTRL_HDR + bytes([request.command & 0xFF, request.joint_idx & 0xFF])
        pkt = pkt_body + bytes([crc8(pkt_body)])
        ok = self._write(pkt)
        response.accepted = ok
        response.message = 'sent' if ok else 'serial not connected'
        return response

    def _resolve_port(self) -> Optional[str]:
        if self.port:
            return self.port
        if list_ports is None:
            return None
        for port in list_ports.comports():
            device = port.device or ''
            if 'ACM' in device or 'USB' in device:
                return device
        return None

    def _connect_if_needed(self):
        if self._serial is not None:
            return
        now = time.monotonic()
        if (now - self._last_connect_try_t) < self.reconnect_period_s:
            return
        self._last_connect_try_t = now

        if serial is None:
            self.get_logger().error('pyserial is not installed; install python3-serial')
            return

        port = self._resolve_port()
        if not port:
            self.get_logger().warn('no serial port available for Momento USB bridge')
            return

        try:
            ser = serial.Serial(port=port, baudrate=self.baudrate, timeout=0.01, write_timeout=0.02)
            with self._serial_lock:
                self._serial = ser
            self.get_logger().info(f'connected to {port}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'failed to open {port}: {exc}')

    def _disconnect(self):
        with self._serial_lock:
            ser = self._serial
            self._serial = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

    def _write(self, data: bytes) -> bool:
        with self._serial_lock:
            ser = self._serial
            if ser is None or not ser.is_open:
                return False
            try:
                ser.write(data)
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'USB write failed: {exc}')
                try:
                    ser.close()
                except Exception:
                    pass
                self._serial = None
                return False

    def _rx_loop(self):
        while not self._stop.is_set():
            with self._serial_lock:
                ser = self._serial
            if ser is None or not ser.is_open:
                time.sleep(0.05)
                continue
            try:
                n = ser.in_waiting
                data = ser.read(n if n > 0 else 1)
            except Exception:
                self._disconnect()
                continue
            if data:
                self._rx_buf.extend(data)
                self._parse_rx()

    def _parse_rx(self):
        while len(self._rx_buf) >= STATE_PKT_SIZE:
            idx = self._rx_buf.find(STATE_HDR)
            if idx < 0:
                del self._rx_buf[:-1]
                return
            if idx > 0:
                del self._rx_buf[:idx]
            if len(self._rx_buf) < STATE_PKT_SIZE:
                return

            pkt = bytes(self._rx_buf[:STATE_PKT_SIZE])
            if crc8(pkt[:-1]) != pkt[-1]:
                self._crc_errors += 1
                del self._rx_buf[0]
                continue

            del self._rx_buf[:STATE_PKT_SIZE]
            sys_state = pkt[2]
            pos = struct.unpack_from('<6f', pkt, 3)
            vel = struct.unpack_from('<6f', pkt, 3 + 6 * 4)
            fault_off = 3 + 6 * 4 + 6 * 4
            fault = list(pkt[fault_off:fault_off + 6])
            self._rx_count += 1

            msg = MomentoState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'momento_usb'
            msg.connected = True
            msg.sys_state = sys_state
            msg.position = list(pos)
            msg.velocity = list(vel)
            msg.fault = fault
            msg.rx_packets = self._rx_count
            msg.tx_packets = self._tx_count
            msg.crc_errors = self._crc_errors
            self.pub_state.publish(msg)

    def _build_balance_packet(self, cmd: MomentoCommand, dt: float) -> bytes:
        linear = clamp(cmd.linear, self.min_linear, self.max_linear)
        roll = clamp(cmd.roll, -self.max_roll, self.max_roll)
        pitch = clamp(cmd.pitch, -self.max_pitch, self.max_pitch)
        leg = clamp(cmd.leg, self.min_leg, self.max_leg)

        if not cmd.start:
            self._turn_set = 0.0
        elif cmd.yaw_mode == MomentoCommand.YAW_ABSOLUTE:
            self._turn_set = clamp(cmd.yaw, -math.pi, math.pi)
        else:
            yaw_rate = clamp(cmd.yaw, -self.max_yaw_rate, self.max_yaw_rate)
            self._turn_set += yaw_rate * dt

        body = BALANCE_HDR + struct.pack(
            '<5fB', linear, self._turn_set, roll, leg, pitch, 1 if cmd.start else 0)
        return body + bytes([crc8(body)])

    def _publish_disconnected_state(self):
        msg = MomentoState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'momento_usb'
        msg.connected = False
        msg.sys_state = 0
        msg.position = [0.0] * 6
        msg.velocity = [0.0] * 6
        msg.fault = [0] * 6
        msg.rx_packets = self._rx_count
        msg.tx_packets = self._tx_count
        msg.crc_errors = self._crc_errors
        self.pub_state.publish(msg)

    def _on_timer(self):
        self._connect_if_needed()

        now = time.monotonic()
        dt = max(0.0, min(0.05, now - self._last_send_t))
        self._last_send_t = now

        fresh = self._latest_cmd_t > 0.0 and (now - self._latest_cmd_t) <= self.cmd_timeout_s
        cmd = self._latest_cmd if fresh else self._safe_command()
        pkt = self._build_balance_packet(cmd, dt)

        connected = self._write(pkt)
        if connected:
            self._tx_count += 1
        if connected != self._last_connected:
            self._last_connected = connected
            if not connected:
                self._publish_disconnected_state()

    def destroy_node(self):
        self._stop.set()
        self._rx_thread.join(timeout=1.0)
        self._disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MomentoUsbBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
