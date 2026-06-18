#!/usr/bin/env python3
"""
Momento wheel-legged robot control GUI.

PySide6 application for real-time visualisation and manual control of the
6 motors (2 hips + 1 wheel per leg) via USB CDC to the Momento master
(STM32H723).

Layout:
    [0] R_hip1   (DM4340, position)
    [1] R_hip2   (DM4340, position)
    [2] R_wheel  (AK60-6, velocity)
    [3] L_hip1   (DM4340, position)
    [4] L_hip2   (DM4340, position)
    [5] L_wheel  (AK60-6, velocity)

For wheels the `pos_des` slot in the wire protocol carries v_des (rad/s);
the GUI exposes this to the user as rpm and converts on TX.

Usage:
    pip install PySide6 pyserial
    python momento_gui.py
"""

from __future__ import annotations

import math
import os
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

import serial
import serial.tools.list_ports
from PySide6.QtCore import Qt, QTimer, Signal, QObject, QRectF, QPointF
from PySide6.QtGui import (
    QColor,
    QFont,
    QPainter,
    QPen,
    QKeyEvent,
    QPaintEvent,
    QMouseEvent,
)
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from xbox_gamepad_qt import XboxGamepadPanel
from xbox_controller_read import GAMEPAD_DPAD_DOWN, GAMEPAD_DPAD_UP

# -- Protocol constants -------------------------------------------------------

JOINT_COUNT = 6

TX_HDR = bytes([0xAA, 0x55])
RX_HDR = bytes([0x55, 0xAA])
CMD_HDR = bytes([0xCC, 0x33])
BALANCE_CMD_HDR = bytes([0xBA, 0x1A])
LEGJOG_CMD_HDR = bytes([0xBB, 0x2B])

# TX layout: hdr(2) + sys_state(1) + pos(6*4) + vel(6*4) + fault(6) + crc(1) = 58
TX_PKT_SIZE  = 2 + 1 + JOINT_COUNT * 2 * 4 + JOINT_COUNT + 1
RX_PKT_SIZE  = 2 + JOINT_COUNT * 4 + 1      # 27
CMD_PKT_SIZE = 5
BALANCE_CMD_FLOATS = 5
BALANCE_CMD_PKT_SIZE = 2 + BALANCE_CMD_FLOATS * 4 + 1 + 1

# Command codes (match joint_bus.h)
CMD_ENABLE       = 0x01
CMD_DISABLE      = 0x02
CMD_SET_ZERO     = 0x03
CMD_ARM          = 0x04
CMD_DISARM       = 0x05
CMD_CLEAR_FAULT  = 0x06

# System state codes (match joint_bus.h JBUS_STATE_*)
STATE_BOOT     = 0
STATE_IDLE     = 1
STATE_ENABLED  = 2
STATE_ARMING   = 3
STATE_ARMED    = 4
STATE_FAULT    = 5

STATE_NAMES = {
    STATE_BOOT:    "BOOT",
    STATE_IDLE:    "IDLE",
    STATE_ENABLED: "ENABLED",
    STATE_ARMING:  "ARMING",
    STATE_ARMED:   "ARMED",
    STATE_FAULT:   "FAULT",
}

STATE_COLORS = {
    STATE_BOOT:    "#888",
    STATE_IDLE:    "#8af",
    STATE_ENABLED: "#8f8",
    STATE_ARMING:  "#fd6",
    STATE_ARMED:   "#ff0",
    STATE_FAULT:   "#f66",
}

# Fault bits (match joint_bus.h JOINT_FAULT_*)
FAULT_FB_TIMEOUT  = 0x01
FAULT_OVER_LIMIT  = 0x02
FAULT_USB_TIMEOUT = 0x04
FAULT_MOTOR_ERR   = 0x08

FAULT_NAMES = [
    (FAULT_FB_TIMEOUT,  "FB"),
    (FAULT_OVER_LIMIT,  "LIM"),
    (FAULT_USB_TIMEOUT, "USB"),
    (FAULT_MOTOR_ERR,   "MOT"),
]

# -- Arm-guard parameters -----------------------------------------------------
#
# For Momento the arm-guard checks position stability of HIP joints only.
# Wheel velocity can legitimately be non-zero, so excluding it avoids
# false negatives that would block Arm.

ARM_MIN_RX_COUNT   = 5
ARM_STABLE_WINDOW  = 5
ARM_STABLE_EPSILON = 0.01   # rad

# Wheel velocity scaling -----------------------------------------------------

WHEEL_MAX_RPM = 300.0
RAD_PER_S_TO_RPM = 60.0 / (2.0 * math.pi)
RPM_TO_RAD_PER_S = (2.0 * math.pi) / 60.0

# -- CRC-8 --------------------------------------------------------------------

_CRC8_TABLE: bytes | None = None

def _build_crc8_table() -> bytes:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return bytes(table)

def crc8(data: bytes) -> int:
    global _CRC8_TABLE
    if _CRC8_TABLE is None:
        _CRC8_TABLE = _build_crc8_table()
    crc = 0x00
    for b in data:
        crc = _CRC8_TABLE[crc ^ b]
    return crc

# -- Packet build / parse -----------------------------------------------------

def build_cmd_packet(setpoints: list[float]) -> bytes:
    """Build host -> STM32 setpoint packet.

    `setpoints[i]` is interpreted by the firmware as:
        - p_des (rad)    for hip slots  (0, 1, 3, 4)
        - v_des (rad/s)  for wheel slots (2, 5)
    """
    assert len(setpoints) == JOINT_COUNT
    body = RX_HDR + struct.pack(f"<{JOINT_COUNT}f", *setpoints)
    return body + bytes([crc8(body)])

def build_ctrl_packet(cmd_type: int, joint_idx: int = 0xFF) -> bytes:
    body = CMD_HDR + bytes([cmd_type, joint_idx])
    return body + bytes([crc8(body)])

def build_balance_packet(v_set: float, turn_set: float, roll_set: float,
                         leg_set: float, phi_set: float, start_flag: int) -> bytes:
    body = (
        BALANCE_CMD_HDR
        + struct.pack("<5f", v_set, turn_set, roll_set, leg_set, phi_set)
        + bytes([1 if start_flag else 0])
    )
    return body + bytes([crc8(body)])

def build_legjog_packet(l0_r: float, phi0_r: float, l0_l: float, phi0_l: float,
                        enable: int) -> bytes:
    body = (
        LEGJOG_CMD_HDR
        + struct.pack("<4f", l0_r, phi0_r, l0_l, phi0_l)
        + bytes([1 if enable else 0])
    )
    return body + bytes([crc8(body)])

def parse_state_packet(raw: bytes):
    """Parse one TX packet from STM32. Returns (sys_state, pos, vel, fault, remainder)
    or None if no complete packet available."""
    while len(raw) >= TX_PKT_SIZE:
        idx = raw.find(TX_HDR)
        if idx < 0:
            return None
        if idx > 0:
            raw = raw[idx:]
            continue
        if len(raw) < TX_PKT_SIZE:
            return None
        pkt = raw[:TX_PKT_SIZE]
        body = pkt[:-1]
        if crc8(body) != pkt[-1]:
            raw = raw[1:]
            continue
        # body = hdr(2) + sys_state(1) + pos(24) + vel(24) + fault(6)
        sys_state = body[2]
        floats = struct.unpack(f"<{JOINT_COUNT * 2}f", body[3:3 + JOINT_COUNT * 8])
        pos = list(floats[:JOINT_COUNT])
        vel = list(floats[JOINT_COUNT:])
        fault_off = 3 + JOINT_COUNT * 8
        fault = list(body[fault_off:fault_off + JOINT_COUNT])
        return sys_state, pos, vel, fault, raw[TX_PKT_SIZE:]
    return None

# -- Joint definitions --------------------------------------------------------

@dataclass
class JointDef:
    idx: int
    name: str
    is_wheel: bool = False
    # Hip joints: asymmetric position window (rad). pos_des is clamped to
    # [pos_min, pos_max]. Mirror the firmware motors[].pos_min/pos_max.
    # Wheel joints leave both at 0.0 (no position limit) and use `limit`
    # below as the velocity saturation (rpm).
    pos_min: float = 0.0
    pos_max: float = 0.0
    limit: float = 0.0          # wheel only: max |rpm|
    coarse_inc: float = 0.0     # rad for hip, rpm for wheel
    fine_inc: float = 0.0

# Hip critical angles match firmware motors[].pos_min/pos_max:
#   R_hip1:  0 .. -1.9 rad     R_hip2:   0 .. +1.9 rad
#   L_hip1:  0 .. +1.9 rad     L_hip2:   0 .. -1.9 rad
# Wheels saturate at WHEEL_MAX_RPM.
JOINTS: list[JointDef] = [
    JointDef(0, "R_hip1",  is_wheel=False, pos_min=0.0, pos_max=0.0,
             coarse_inc=0.10, fine_inc=0.01),
    JointDef(1, "R_hip2",  is_wheel=False, pos_min=0.0, pos_max=0.0,
             coarse_inc=0.10, fine_inc=0.01),
    JointDef(2, "R_wheel", is_wheel=True,  limit=WHEEL_MAX_RPM,
             coarse_inc=10.0, fine_inc=1.0),
    JointDef(3, "L_hip1",  is_wheel=False, pos_min=0.0, pos_max=0.0,
             coarse_inc=0.10, fine_inc=0.01),
    JointDef(4, "L_hip2",  is_wheel=False, pos_min=0.0, pos_max=0.0,
             coarse_inc=0.10, fine_inc=0.01),
    JointDef(5, "L_wheel", is_wheel=True,  limit=WHEEL_MAX_RPM,
             coarse_inc=10.0, fine_inc=1.0),
]

HIP_INDICES   = [0, 1, 3, 4]
WHEEL_INDICES = [2, 5]

# -- Gamepad mapping ----------------------------------------------------------
#
# DIRECT (absolute) mapping. Stick position IS the command, not the rate of
# change. Releasing the sticks returns the robot to neutral: hips back to
# HIP_NEUTRAL_RAD, wheels back to 0 rpm. When the gamepad is connected and
# the robot is ARMED, sticks override slider / keyboard input every tick.
#
# Right stick = differential drive (both wheels):
#   RY (up = forward)       -> robot moves forward in WORLD frame
#                              (because the two wheel motors are mirror-mounted
#                              on opposite sides of the chassis, their MOTOR-frame
#                              commands have opposite signs for the same world
#                              motion -- handled by WHEEL_MOTOR_SIGN below).
#   RX (right = yaw right)  -> world: R wheel back, L wheel forward
#                              motor: both motors get the SAME-sign yaw term
#                              once WHEEL_MOTOR_SIGN is applied.
#
# Left stick = leg posture (4 hip joints, 5-bar antisymmetric):
#   LY  -> INTEGRATED leg extension (velocity input, not spring-return).
#          LY = +1 extends at GAMEPAD_LEG_EXT_RATE rad/s, LY = 0 HOLDS the
#          current leg length, LY = -1 retracts at the same rate. The
#          accumulator clamps to [0, GAMEPAD_LEG_EXT_MAX_RAD] and is reset
#          to 0 whenever the robot leaves ARMED state (so re-arming always
#          starts from the retracted pose).
#   LX  -> direct tilt offset (spring-return). Push right = right leg shorter,
#          left leg longer; release = body returns upright.
#
# Tune the per-joint neutral angles and extension signs to match the
# mechanical convention of the assembly (linkage orientation, motor mount).

GAMEPAD_DEADZONE = 0.15

# How much each control affects its joints at full stick deflection.
GAMEPAD_LEG_EXT_RATE    = 0.6    # LY -> integrated leg-extension rate (rad/s)
GAMEPAD_LEG_EXT_MAX_RAD = 1.9    # |leg_ext| upper bound (matches DM range)
GAMEPAD_HIP_TILT_RAD    = 0.20   # LX -> per-hip tilt offset (rad) at |LX|=1
GAMEPAD_MAX_YAW_RPM     = 150.0  # RX -> per-wheel rpm differential at |RX|=1
_GAMEPAD_TICK_S         = 0.05   # must match the UI tick (50 ms / 20 Hz)

# Balance-mode high-level command limits. These feed the firmware's
# chassis_t command adapter, not the joint debug position/velocity path.
BALANCE_DEFAULT_LEG_SET_M = 0.07   # initial leg length on entering balance mode
BALANCE_LEG_SET_MIN_M = 0.068        # D-pad lower bound (== initial)
BALANCE_LEG_SET_MAX_M = 0.21        # D-pad upper bound
BALANCE_LEG_SET_STEP_M = 0.01       # D-pad up/down increment per press
BALANCE_MAX_V_SET_MPS = 0.50
BALANCE_MAX_ROLL_RAD = 0.15
BALANCE_MAX_PHI_RAD = 0.15
BALANCE_MAX_YAW_RATE_RAD_S = 0.80

# Leg-jog mode (suspended Jacobian bring-up). Per-leg (L0, theta) target fed
# to the firmware Cartesian PD -> F0/Tp -> Jacobian -> torque. theta is the
# unified swing state: 0 == leg straight down.
LEGJOG_L0_MIN_M = 0.068
LEGJOG_L0_MAX_M = 0.21
LEGJOG_L0_DEFAULT_M = 0.068
LEGJOG_L0_RATE = 0.06                        # leg-length slew (m/s) at full stick
LEGJOG_THETA_MAX_RAD = math.radians(46.0)    # +/- swing target bound
LEGJOG_THETA_RATE = math.radians(60.0)       # swing slew (rad/s) at full stick

# Per-hip neutral angle (rad). Robot pose when both sticks are at rest.
HIP_NEUTRAL_RAD = {0: 0.0, 1: 0.0, 3: 0.0, 4: 0.0}

# Sign of hip rotation that EXTENDS the leg (5-bar antisymmetric pair).
# Chosen to match the critical-angle windows in JOINTS (each hip only has
# motion AWAY from 0 toward its non-zero limit): pushing LY up moves each
# hip into its available working range. Flip per assembly if it does the
# opposite mechanically.
HIP_EXT_SIGN = {
    0: -1.0,   # R_hip1: range  0 .. -1.9 rad  -> extend goes negative
    1: +1.0,   # R_hip2: range  0 .. +1.9 rad  -> extend goes positive
    3: +1.0,   # L_hip1: range  0 .. +1.9 rad
    4: -1.0,   # L_hip2: range  0 .. -1.9 rad
}

# Per-wheel sign that maps DESIRED WORLD-FRAME forward rpm to MOTOR rpm.
# The two wheel motors are mirror-mounted on opposite sides of the chassis,
# so to roll the robot forward together the motors must spin in OPPOSITE
# motor-frame directions -- hence the two signs differ by default.
# Flip the corresponding entry if a wheel spins backwards in practice.
WHEEL_MOTOR_SIGN = {2: +1.0, 5: -1.0}   # R_wheel, L_wheel  (mirror-mounted)


def _apply_deadzone(v: float, dz: float = GAMEPAD_DEADZONE) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


# -- Colour palette -----------------------------------------------------------

COL_BG          = QColor(64, 64, 64)
COL_CIRCLE_BG   = QColor(110, 110, 110)
COL_ARC_OK      = QColor(100, 220, 100)
COL_ARC_WARN    = QColor(255, 80, 80)
COL_SELECTED    = QColor(0, 180, 255)
COL_TEXT        = QColor(230, 230, 230)
COL_WHEEL_BG    = QColor(95, 95, 110)

# -- Serial bridge (runs in background thread) --------------------------------

class SerialBridge(QObject):
    state_received = Signal(int, list, list, list)   # sys_state, pos, vel, fault
    connection_lost = Signal()

    def __init__(self):
        super().__init__()
        self._ser: serial.Serial | None = None
        self._stop = threading.Event()
        self._rx_thread: threading.Thread | None = None
        self._tx_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self.armed = False
        # pos_des[i] meaning:
        #   hip   -> position target in rad
        #   wheel -> velocity target in rad/s (GUI internally tracks rpm)
        self.pos_des = [0.0] * JOINT_COUNT
        # Wheel display value (rpm); kept in sync with pos_des[wheel]
        self.wheel_rpm_des = [0.0, 0.0]   # index by 0=R, 1=L
        self.last_pos = [0.0] * JOINT_COUNT
        self.last_vel = [0.0] * JOINT_COUNT
        self.last_fault = [0] * JOINT_COUNT
        self.sys_state = STATE_BOOT
        self.rx_count = 0
        self.tx_count = 0
        self._pos_history: deque[list[float]] = deque(maxlen=ARM_STABLE_WINDOW + 1)
        self.balance_enabled = False
        self.balance_v_set = 0.0
        self.balance_turn_set = 0.0
        self.balance_roll_set = 0.0
        self.balance_leg_set = BALANCE_DEFAULT_LEG_SET_M
        self.balance_phi_set = 0.0
        # Leg-jog mode: per-leg [right, left] (L0 metres, theta radians).
        self.legjog_enabled = False
        self.legjog_l0 = [LEGJOG_L0_DEFAULT_M, LEGJOG_L0_DEFAULT_M]
        self.legjog_theta = [0.0, 0.0]

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def is_stable(self) -> bool:
        """Hip positions have not changed meaningfully over the last
        ARM_STABLE_WINDOW packets.  Wheels are excluded (velocity allowed)."""
        hist = self._pos_history
        if len(hist) < ARM_STABLE_WINDOW + 1:
            return False
        oldest = hist[0]
        newest = hist[-1]
        for j in HIP_INDICES:
            if abs(oldest[j] - newest[j]) > ARM_STABLE_EPSILON:
                return False
        return True

    @property
    def feedback_ok(self) -> bool:
        return self.rx_count > 0 and not any(self.last_fault)

    @property
    def arm_ready(self) -> bool:
        return self.sys_state == STATE_ENABLED and self.feedback_ok

    def connect(self, port: str) -> bool:
        try:
            self._ser = serial.Serial(port, timeout=0.005)
            self._ser.reset_input_buffer()
            self._stop.clear()
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._rx_thread.start()
            self._tx_thread.start()
            return True
        except Exception:
            self._ser = None
            return False

    def disconnect(self):
        self.armed = False
        self._stop.set()
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._tx_thread:
            self._tx_thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None
        self.rx_count = 0
        self._pos_history.clear()

    def send_ctrl(self, cmd_type: int, joint_idx: int = 0xFF):
        if not self.connected:
            return
        pkt = build_ctrl_packet(cmd_type, joint_idx)
        with self._lock:
            try:
                self._ser.write(pkt)
            except Exception:
                pass

    def send_balance_stop(self):
        if not self.connected:
            return
        pkt = build_balance_packet(0.0, self.balance_turn_set, 0.0,
                                   self.balance_leg_set, 0.0, 0)
        with self._lock:
            try:
                self._ser.write(pkt)
            except Exception:
                pass

    def send_legjog_stop(self):
        if not self.connected:
            return
        pkt = build_legjog_packet(self.legjog_l0[0], self.legjog_theta[0],
                                  self.legjog_l0[1], self.legjog_theta[1], 0)
        with self._lock:
            try:
                self._ser.write(pkt)
            except Exception:
                pass

    def _rx_loop(self):
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = self._ser.read(512)
            except Exception:
                self.connection_lost.emit()
                return
            if not chunk:
                continue
            buf += chunk
            while True:
                result = parse_state_packet(buf)
                if result is None:
                    break
                sys_state, pos, vel, fault, buf = result
                self.sys_state = sys_state
                self.last_pos = pos
                self.last_vel = vel
                self.last_fault = fault
                self._pos_history.append(pos)
                self.rx_count += 1
                self.state_received.emit(sys_state, pos, vel, fault)

    def _tx_loop(self):
        period = 1.0 / 100.0
        while not self._stop.is_set():
            t0 = time.perf_counter()
            if self.armed and self.connected:
                if self.balance_enabled:
                    pkt = build_balance_packet(
                        self.balance_v_set,
                        self.balance_turn_set,
                        self.balance_roll_set,
                        self.balance_leg_set,
                        self.balance_phi_set,
                        1,
                    )
                elif self.legjog_enabled:
                    pkt = build_legjog_packet(
                        self.legjog_l0[0],
                        self.legjog_theta[0],
                        self.legjog_l0[1],
                        self.legjog_theta[1],
                        1,
                    )
                else:
                    pkt = build_cmd_packet(self.pos_des)
                with self._lock:
                    try:
                        self._ser.write(pkt)
                        self.tx_count += 1
                    except Exception:
                        pass
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)


# -- Hip joint circle widget (position dial) ---------------------------------

class JointCircle(QWidget):
    clicked = Signal(int)

    def __init__(self, jdef: JointDef, parent=None):
        super().__init__(parent)
        self.jdef = jdef
        self.position = 0.0
        self.velocity = 0.0
        self.fault = 0
        self.selected = False
        self.setFixedSize(130, 170)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, ev: QMouseEvent):
        self.clicked.emit(self.jdef.idx)

    def paintEvent(self, ev: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        cx, cy, r = 65, 58, 46

        if self.fault != 0:
            p.setPen(QPen(QColor(255, 80, 80), 3))
        elif self.selected:
            p.setPen(QPen(COL_SELECTED, 3))
        else:
            p.setPen(QPen(QColor(180, 180, 180), 2))
        p.setBrush(COL_CIRCLE_BG)
        p.drawEllipse(QPointF(cx, cy), r, r)

        # Visual scale: magnitude of the larger endpoint of the asymmetric
        # [pos_min, pos_max] window. Arc growth is signed (CCW = -, CW = +)
        # so a hip with range [0, +1.9] will only ever fill the right half,
        # and one with [-1.9, 0] only the left half.
        limit = max(abs(self.jdef.pos_min), abs(self.jdef.pos_max))
        frac = max(-1.0, min(1.0, self.position / limit)) if limit > 0 else 0.0

        arc_color = COL_ARC_OK if abs(frac) < 0.9 else COL_ARC_WARN
        p.setPen(QPen(arc_color, 6))
        p.setBrush(Qt.NoBrush)

        span_angle = int(-frac * 150 * 16)
        start_angle = 90 * 16
        arc_rect = QRectF(cx - r + 7, cy - r + 7, (r - 7) * 2, (r - 7) * 2)
        if span_angle != 0:
            p.drawArc(arc_rect, start_angle, span_angle)

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(100, 100, 100))
        p.drawEllipse(QPointF(cx, cy), r - 11, r - 11)

        p.setPen(COL_TEXT)
        p.setFont(QFont("Segoe UI", 10, QFont.Bold))
        p.drawText(QRectF(0, cy - 12, 130, 24), Qt.AlignCenter, self.jdef.name)

        p.setFont(QFont("Segoe UI", 9))
        pos_str = f"{self.position:.2f} rad"
        vel_str = f"{self.velocity:.2f} r/s"
        p.drawText(QRectF(0, cy + r + 4, 130, 18), Qt.AlignCenter, pos_str)
        p.drawText(QRectF(0, cy + r + 20, 130, 18), Qt.AlignCenter, vel_str)

        if self.fault != 0:
            tags = [name for (bit, name) in FAULT_NAMES if (self.fault & bit)]
            p.setPen(QColor(255, 120, 120))
            p.setFont(QFont("Segoe UI", 8, QFont.Bold))
            p.drawText(QRectF(0, cy + r + 36, 130, 14),
                       Qt.AlignCenter, " | ".join(tags))

        p.end()


# -- Wheel circle widget (rpm dial) ------------------------------------------
#
# Same circular footprint as JointCircle but the arc represents *velocity*:
#   * 0 rpm  -> arc length 0 (vertical needle at 12 o'clock).
#   * CCW rotation positive (arc grows to the LEFT).
#   * CW  rotation negative (arc grows to the RIGHT).
#   * |rpm| > WHEEL_MAX_RPM -> red ring + red arc, the dial saturates.

class WheelCircle(QWidget):
    clicked = Signal(int)

    def __init__(self, jdef: JointDef, parent=None):
        super().__init__(parent)
        assert jdef.is_wheel
        self.jdef = jdef
        # `position` is shaft angle (rad). Not used for the dial itself
        # (wheels are velocity-controlled), but displayed as a small
        # "encoder alive" readout so the user can confirm feedback is
        # flowing even when the wheel is stationary (velocity ~= 0).
        self.position = 0.0
        self.velocity_rpm = 0.0    # measured (from fb.velocity)
        self.cmd_rpm = 0.0         # target (from GUI / TX)
        self.fault = 0
        self.selected = False
        self.setFixedSize(130, 190)
        self.setCursor(Qt.PointingHandCursor)

    def mousePressEvent(self, ev: QMouseEvent):
        self.clicked.emit(self.jdef.idx)

    def paintEvent(self, ev: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        cx, cy, r = 65, 58, 46

        # Saturation detection
        over_limit = abs(self.velocity_rpm) > WHEEL_MAX_RPM

        if self.fault != 0 or over_limit:
            p.setPen(QPen(QColor(255, 80, 80), 3))
        elif self.selected:
            p.setPen(QPen(COL_SELECTED, 3))
        else:
            p.setPen(QPen(QColor(180, 180, 180), 2))
        p.setBrush(COL_WHEEL_BG)
        p.drawEllipse(QPointF(cx, cy), r, r)

        frac = max(-1.0, min(1.0, self.velocity_rpm / WHEEL_MAX_RPM))
        warn = abs(self.velocity_rpm) >= 0.9 * WHEEL_MAX_RPM
        arc_color = COL_ARC_WARN if (warn or over_limit) else COL_ARC_OK
        p.setPen(QPen(arc_color, 6))
        p.setBrush(Qt.NoBrush)

        # CCW positive: positive frac -> positive (Qt CCW) span. Start at 12 o'clock.
        span_angle = int(frac * 150 * 16)
        start_angle = 90 * 16
        arc_rect = QRectF(cx - r + 7, cy - r + 7, (r - 7) * 2, (r - 7) * 2)
        if span_angle != 0:
            p.drawArc(arc_rect, start_angle, span_angle)

        # Inner well + target needle (dashed) showing cmd_rpm
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(100, 100, 100))
        p.drawEllipse(QPointF(cx, cy), r - 11, r - 11)

        if abs(self.cmd_rpm) > 0.1:
            cmd_frac = max(-1.0, min(1.0, self.cmd_rpm / WHEEL_MAX_RPM))
            # CCW positive -> +angle in math sense (Qt screen y is down, so flip)
            theta = math.radians(90.0 + cmd_frac * 150.0)
            x2 = cx + (r - 14) * math.cos(theta)
            y2 = cy - (r - 14) * math.sin(theta)
            p.setPen(QPen(QColor(255, 220, 80), 2, Qt.DashLine))
            p.drawLine(QPointF(cx, cy), QPointF(x2, y2))

        p.setPen(COL_TEXT)
        p.setFont(QFont("Segoe UI", 10, QFont.Bold))
        p.drawText(QRectF(0, cy - 12, 130, 24), Qt.AlignCenter, self.jdef.name)

        p.setFont(QFont("Segoe UI", 9))
        rpm_str = f"{self.velocity_rpm:+.1f} rpm"
        pos_str = f"shaft: {self.position:+.2f} rad"
        tgt_str = f"target: {self.cmd_rpm:+.0f} rpm"
        p.drawText(QRectF(0, cy + r + 4,  130, 18), Qt.AlignCenter, rpm_str)
        p.drawText(QRectF(0, cy + r + 20, 130, 18), Qt.AlignCenter, pos_str)
        p.drawText(QRectF(0, cy + r + 36, 130, 18), Qt.AlignCenter, tgt_str)

        if self.fault != 0:
            tags = [name for (bit, name) in FAULT_NAMES if (self.fault & bit)]
            p.setPen(QColor(255, 120, 120))
            p.setFont(QFont("Segoe UI", 8, QFont.Bold))
            p.drawText(QRectF(0, cy + r + 52, 130, 14),
                       Qt.AlignCenter, " | ".join(tags))

        p.end()


# -- Main window --------------------------------------------------------------

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Momento Control Dashboard")
        self.setMinimumSize(960, 700)
        self.resize(1024, 768)
        self.setStyleSheet(f"background-color: {COL_BG.name()};")

        self.bridge = SerialBridge()
        self.bridge.state_received.connect(self._on_state)
        self.bridge.connection_lost.connect(self._on_disconnect)

        self.selected_joint: int = -1

        # Integrated leg-extension state for gamepad LY (rad, range
        # [0, GAMEPAD_LEG_EXT_MAX_RAD]). Held between ticks; reset to 0
        # when the robot leaves ARMED state. See _gamepad_integrate.
        self._leg_ext_rad: float = 0.0

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(6, 6, 6, 6)
        root_layout.setSpacing(4)

        # -- Toolbar --------------------------------------------------
        toolbar = QHBoxLayout()
        toolbar.setSpacing(4)

        lbl_style = "color: #ddd; font: 10pt 'Segoe UI';"
        btn_style = (
            "QPushButton { background: #555; color: #eee; border: 1px solid #777; "
            "border-radius: 5px; padding: 4px 8px; font: 9pt 'Segoe UI'; } "
            "QPushButton:hover { background: #666; } "
            "QPushButton:pressed { background: #444; } "
            "QPushButton:disabled { background: #444; color: #888; }"
        )
        combo_style = (
            "QComboBox { background: #555; color: #eee; border: 1px solid #777; "
            "border-radius: 5px; padding: 3px 6px; font: 9pt 'Segoe UI'; }"
        )

        lbl_port = QLabel("COM Port:")
        lbl_port.setStyleSheet(lbl_style)
        toolbar.addWidget(lbl_port)

        self.combo_port = QComboBox()
        self.combo_port.setStyleSheet(combo_style)
        self.combo_port.setMinimumWidth(105)
        toolbar.addWidget(self.combo_port)

        self.btn_refresh = QPushButton("\u21bb")
        self.btn_refresh.setStyleSheet(btn_style)
        self.btn_refresh.setToolTip("Refresh COM ports")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        toolbar.addWidget(self.btn_refresh)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setStyleSheet(btn_style)
        self.btn_connect.clicked.connect(self._toggle_connect)
        toolbar.addWidget(self.btn_connect)

        toolbar.addSpacing(8)

        self.btn_enable = QPushButton("Enable")
        self.btn_enable.setStyleSheet(btn_style)
        self.btn_enable.setEnabled(False)
        self.btn_enable.clicked.connect(self._enable_all)
        toolbar.addWidget(self.btn_enable)

        self.btn_disable = QPushButton("Disable")
        self.btn_disable.setStyleSheet(btn_style)
        self.btn_disable.setEnabled(False)
        self.btn_disable.clicked.connect(self._disable_all)
        toolbar.addWidget(self.btn_disable)

        toolbar.addSpacing(8)

        self.btn_arm = QPushButton("Arm")
        self.btn_arm.setStyleSheet(btn_style)
        self.btn_arm.setEnabled(False)
        self.btn_arm.clicked.connect(self._arm)
        toolbar.addWidget(self.btn_arm)

        self.btn_disarm = QPushButton("Disarm")
        self.btn_disarm.setStyleSheet(btn_style)
        self.btn_disarm.setEnabled(False)
        self.btn_disarm.clicked.connect(self._disarm)
        toolbar.addWidget(self.btn_disarm)

        toolbar.addSpacing(8)

        self.btn_balance = QPushButton("Balance")
        self.btn_balance.setCheckable(True)
        self.btn_balance.setStyleSheet(btn_style)
        self.btn_balance.setEnabled(False)
        self.btn_balance.clicked.connect(self._toggle_balance_mode)
        toolbar.addWidget(self.btn_balance)

        self.btn_legjog = QPushButton("Leg Jog")
        self.btn_legjog.setCheckable(True)
        self.btn_legjog.setStyleSheet(btn_style)
        self.btn_legjog.setEnabled(False)
        self.btn_legjog.clicked.connect(self._toggle_legjog_mode)
        toolbar.addWidget(self.btn_legjog)

        toolbar.addSpacing(8)

        clear_fault_style = (
            "QPushButton { background: #844; color: #fdd; border: 1px solid #a77; "
            "border-radius: 5px; padding: 4px 8px; font: 9pt 'Segoe UI'; } "
            "QPushButton:hover { background: #955; } "
            "QPushButton:disabled { background: #533; color: #888; }"
        )
        self.btn_clear_fault = QPushButton("Fault")
        self.btn_clear_fault.setStyleSheet(clear_fault_style)
        self.btn_clear_fault.setEnabled(False)
        self.btn_clear_fault.clicked.connect(self._clear_fault)
        toolbar.addWidget(self.btn_clear_fault)

        # Toolbar buttons must NOT keep keyboard focus, otherwise after a click
        # the arrow keys get consumed by focus navigation and never reach
        # keyPressEvent (which is how the keyboard motor-jog works).
        for _b in (self.btn_refresh, self.btn_connect, self.btn_enable,
                   self.btn_disable, self.btn_arm, self.btn_disarm,
                   self.btn_balance, self.btn_legjog, self.btn_clear_fault):
            _b.setFocusPolicy(Qt.NoFocus)

        toolbar.addStretch()

        self.lbl_status = QLabel("Disconnected")
        self.lbl_status.setStyleSheet("color: #f88; font: 10pt 'Segoe UI';")
        toolbar.addWidget(self.lbl_status)

        root_layout.addLayout(toolbar)

        # -- Robot visualisation --------------------------------------
        self.viz = RobotVizWidget(self)
        root_layout.addWidget(self.viz, stretch=1)

        # -- Bottom row: gamepad (left) + title (right) ---------------
        bottom_row = QHBoxLayout()
        bottom_row.setContentsMargins(0, 0, 0, 0)
        bottom_row.setSpacing(10)

        self.gamepad_panel = XboxGamepadPanel(central)
        self.gamepad_panel.setMaximumHeight(170)
        self.gamepad_panel.setMaximumWidth(800)
        bottom_row.addWidget(self.gamepad_panel, stretch=1)

        title_label = QLabel("MOMENTO\nWheel-Legged")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setMaximumWidth(210)
        title_label.setStyleSheet(
            "color: #e6e6e6; font: bold 17pt 'Segoe UI'; background: transparent;"
        )
        bottom_row.addWidget(title_label, stretch=0)

        root_layout.addLayout(bottom_row, stretch=0)

        # -- UI timer -------------------------------------------------
        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._ui_tick)
        self._ui_timer.start(50)

        self._refresh_ports()
        self.setFocusPolicy(Qt.StrongFocus)

    # -- Port management ----------------------------------------------

    def _refresh_ports(self):
        self.combo_port.clear()
        for p in serial.tools.list_ports.comports():
            self.combo_port.addItem(p.device, p.device)

    def _toggle_connect(self):
        if self.bridge.connected:
            self._disarm()
            self.bridge.disconnect()
            self._update_conn_ui(False)
        else:
            port = self.combo_port.currentData()
            if port and self.bridge.connect(port):
                self._update_conn_ui(True)
            else:
                self.lbl_status.setText("Connection failed")
                self.lbl_status.setStyleSheet("color: #f88; font: 11pt 'Segoe UI';")

    def _update_conn_ui(self, connected: bool):
        self.btn_connect.setText("Disconnect" if connected else "Connect")
        if not connected:
            self.btn_enable.setEnabled(False)
            self.btn_disable.setEnabled(False)
            self.btn_arm.setEnabled(False)
            self.btn_disarm.setEnabled(False)
            self.btn_balance.setEnabled(False)
            self.btn_balance.setChecked(False)
            self.btn_legjog.setEnabled(False)
            self.btn_legjog.setChecked(False)
            self.btn_clear_fault.setEnabled(False)
            for zbtn in self.viz.zero_buttons.values():
                zbtn.setEnabled(False)
        if connected:
            self.lbl_status.setText("Connected")
            self.lbl_status.setStyleSheet("color: #8f8; font: 11pt 'Segoe UI';")
        else:
            self.lbl_status.setText("Disconnected")
            self.lbl_status.setStyleSheet("color: #f88; font: 11pt 'Segoe UI';")

    def _on_disconnect(self):
        self._disarm()
        self._update_conn_ui(False)

    # -- Enable / Arm / Disarm / Clear Fault --------------------------

    def _enable_all(self):
        self.bridge.send_ctrl(CMD_ENABLE)

    def _disable_all(self):
        self.bridge.send_ctrl(CMD_DISABLE)
        self._set_balance_mode(False)
        self._set_legjog_mode(False)
        self.bridge.armed = False
        # Reset wheel targets so Arm starts from coast
        for k in range(2):
            self.bridge.wheel_rpm_des[k] = 0.0
        for j in WHEEL_INDICES:
            self.bridge.pos_des[j] = 0.0

    def _arm(self):
        if not self.bridge.arm_ready:
            self.lbl_status.setText("ARM BLOCKED: feedback not ready")
            self.lbl_status.setStyleSheet(
                "color: #f80; font: 11pt 'Segoe UI'; font-weight: bold;"
            )
            return
        # Hip targets latched to current measured position (no jump);
        # wheel targets reset to 0 rpm (coast).
        new_des = list(self.bridge.last_pos)
        for k, jw in enumerate(WHEEL_INDICES):
            new_des[jw] = 0.0
            self.bridge.wheel_rpm_des[k] = 0.0
        self.bridge.pos_des = new_des
        self.bridge.send_ctrl(CMD_ARM)
        self.bridge.armed = True

    def _disarm(self):
        self._set_balance_mode(False)
        self._set_legjog_mode(False)
        self.bridge.send_ctrl(CMD_DISARM)
        self.bridge.armed = False

    def _clear_fault(self):
        self._set_balance_mode(False)
        self._set_legjog_mode(False)
        self.bridge.send_ctrl(CMD_CLEAR_FAULT)
        self.bridge.armed = False

    def _set_balance_mode(self, enabled: bool):
        enabled = bool(enabled and self.bridge.connected and self.bridge.armed)
        if enabled:
            self._set_legjog_mode(False)
        if not enabled and self.bridge.balance_enabled:
            self.bridge.send_balance_stop()
        self.bridge.balance_enabled = enabled
        self.btn_balance.setChecked(enabled)
        if enabled:
            self.bridge.balance_v_set = 0.0
            self.bridge.balance_roll_set = 0.0
            self.bridge.balance_phi_set = 0.0
            self.bridge.balance_leg_set = BALANCE_DEFAULT_LEG_SET_M
            self._prev_dpad_bits = 0
            self.lbl_status.setText("BALANCE MODE")
            self.lbl_status.setStyleSheet(
                "color: #7df; font: 11pt 'Segoe UI'; font-weight: bold;"
            )

    def _toggle_balance_mode(self):
        self._set_balance_mode(self.btn_balance.isChecked())

    def _set_legjog_mode(self, enabled: bool):
        enabled = bool(enabled and self.bridge.connected and self.bridge.armed)
        if enabled:
            self._set_balance_mode(False)
        if not enabled and self.bridge.legjog_enabled:
            self.bridge.send_legjog_stop()
        self.bridge.legjog_enabled = enabled
        self.btn_legjog.setChecked(enabled)
        if enabled:
            self.bridge.legjog_l0 = [LEGJOG_L0_DEFAULT_M, LEGJOG_L0_DEFAULT_M]
            self.bridge.legjog_theta = [0.0, 0.0]
            self.lbl_status.setText("LEG JOG MODE")
            self.lbl_status.setStyleSheet(
                "color: #fd7; font: 11pt 'Segoe UI'; font-weight: bold;"
            )

    def _toggle_legjog_mode(self):
        self._set_legjog_mode(self.btn_legjog.isChecked())

    # -- Feedback -----------------------------------------------------

    def _on_state(self, sys_state: int, pos: list[float],
                  vel: list[float], fault: list[int]):
        for j, jw in self.viz.hip_circles.items():
            jw.position = pos[j]
            jw.velocity = vel[j]
            jw.fault = fault[j]
        for k, j in enumerate(WHEEL_INDICES):
            ww = self.viz.wheel_circles[j]
            ww.position = pos[j]                          # encoder-alive readout
            ww.velocity_rpm = vel[j] * RAD_PER_S_TO_RPM
            ww.cmd_rpm = self.bridge.wheel_rpm_des[k]
            ww.fault = fault[j]
        self._refresh_state_ui(sys_state)

    def _refresh_state_ui(self, sys_state: int):
        connected  = self.bridge.connected
        is_idle    = (sys_state == STATE_IDLE)
        is_enabled = (sys_state == STATE_ENABLED)
        is_armed   = (sys_state == STATE_ARMING or sys_state == STATE_ARMED)
        is_fault   = (sys_state == STATE_FAULT)
        is_powered = (is_enabled or is_armed or is_fault)
        arm_ready  = self.bridge.arm_ready

        if self.bridge.balance_enabled and is_armed:
            name = "BALANCE MODE"
            color = "#7df"
        elif self.bridge.legjog_enabled and is_armed:
            name = "LEG JOG MODE"
            color = "#fd7"
        elif connected and is_enabled and not arm_ready:
            if self.bridge.rx_count == 0:
                name = "WAITING FEEDBACK"
            elif any(self.bridge.last_fault):
                name = "FEEDBACK FAULT"
            else:
                name = "ENABLED"
            color = "#f80"
        else:
            name  = STATE_NAMES.get(sys_state, f"S{sys_state}")
            color = STATE_COLORS.get(sys_state, "#fff")
        self.lbl_status.setText(name)
        self.lbl_status.setStyleSheet(
            f"color: {color}; font: 11pt 'Segoe UI'; font-weight: bold;"
        )

        self.btn_enable.setEnabled(connected and (is_idle or is_fault))
        self.btn_disable.setEnabled(connected and is_powered)
        self.btn_arm.setEnabled(connected and is_enabled and arm_ready)
        self.btn_disarm.setEnabled(connected and is_armed)
        self.btn_balance.setEnabled(connected and is_armed)
        self.btn_legjog.setEnabled(connected and is_armed)
        self.btn_clear_fault.setEnabled(connected and is_fault)

        if not is_armed and self.bridge.balance_enabled:
            self._set_balance_mode(False)
        if not is_armed and self.bridge.legjog_enabled:
            self._set_legjog_mode(False)

        zero_ok = connected and is_enabled
        for zbtn in self.viz.zero_buttons.values():
            zbtn.setEnabled(zero_ok)

        self.bridge.armed = is_armed

    def _ui_tick(self):
        self._gamepad_integrate()
        self.viz.update()
        for jw in self.viz.hip_circles.values():
            jw.update()
        for ww in self.viz.wheel_circles.values():
            ww.update()

    # -- Gamepad direct mapping ---------------------------------------
    #
    # When connected and armed:
    #   - Right stick drives both wheels (differential).
    #   - Left  stick poses both legs   (extension + tilt).
    # Every tick the gamepad writes pos_des and wheel_rpm_des directly,
    # overriding sliders / keyboard input until released (centered).

    def _gamepad_integrate(self):
        gp = self.gamepad_panel
        if not gp.connected or not self.bridge.armed:
            # Reset the integrator on disarm so re-arming always starts at the
            # retracted pose (avoids a sudden setpoint jump on re-ARM).
            self._leg_ext_rad = 0.0
            return

        if self.bridge.balance_enabled:
            self._gamepad_balance_integrate()
            return

        if self.bridge.legjog_enabled:
            self._gamepad_legjog_integrate()
            return

        rx = _apply_deadzone(gp.cur_rx / 32768.0)
        ry = _apply_deadzone(gp.cur_ry / 32768.0)

        # -- Right stick: differential wheel drive ------------------
        # 1) Compute desired WORLD-frame velocity per wheel (rpm-equivalent).
        #    RY > 0 = both wheels roll forward in world frame.
        #    RX > 0 = turn right -> R wheel rolls backward, L wheel forward.
        fwd_rpm = ry * WHEEL_MAX_RPM
        yaw_rpm = rx * GAMEPAD_MAX_YAW_RPM
        v_world_rpm = {
            2: fwd_rpm - yaw_rpm,   # R_wheel (world frame)
            5: fwd_rpm + yaw_rpm,   # L_wheel (world frame)
        }
        # 2) Apply per-wheel mirror-mount sign to convert to MOTOR-frame rpm.
        for idx in WHEEL_INDICES:
            rpm = v_world_rpm[idx] * WHEEL_MOTOR_SIGN[idx]
            rpm = max(-WHEEL_MAX_RPM, min(WHEEL_MAX_RPM, rpm))
            k = WHEEL_INDICES.index(idx)
            self.bridge.wheel_rpm_des[k] = rpm
            self.bridge.pos_des[idx] = rpm * RPM_TO_RAD_PER_S

        # -- Hips: NOT driven by the gamepad in plain armed mode ----
        # The left stick used to pose the legs, but that overrode the
        # arm-latched hip positions every tick (snapping legs to the neutral
        # pose). In plain armed mode the hips are left to keyboard / slider
        # control only; the gamepad touches wheels exclusively.

    def _gamepad_balance_integrate(self):
        gp = self.gamepad_panel

        lx = _apply_deadzone(gp.cur_lx / 32768.0)
        ly = _apply_deadzone(gp.cur_ly / 32768.0)
        rx = _apply_deadzone(gp.cur_rx / 32768.0)
        ry = _apply_deadzone(gp.cur_ry / 32768.0)

        # Balance mode:
        #   LY -> forward/back speed v_set
        #   LX -> yaw-rate command integrated into turn_set heading
        #   RY -> body pitch command (firmware phi_set)
        #   RX -> roll angle command
        #   D-pad Up/Down -> step leg length by BALANCE_LEG_SET_STEP_M per press
        self.bridge.balance_v_set = ly * BALANCE_MAX_V_SET_MPS
        self.bridge.balance_roll_set = rx * BALANCE_MAX_ROLL_RAD
        self.bridge.balance_phi_set = ry * BALANCE_MAX_PHI_RAD
        self.bridge.balance_turn_set += lx * BALANCE_MAX_YAW_RATE_RAD_S * _GAMEPAD_TICK_S

        bits = gp.cur_buttons
        prev = getattr(self, "_prev_dpad_bits", 0)
        up_edge = (bits & GAMEPAD_DPAD_UP) and not (prev & GAMEPAD_DPAD_UP)
        down_edge = (bits & GAMEPAD_DPAD_DOWN) and not (prev & GAMEPAD_DPAD_DOWN)
        if up_edge:
            self.bridge.balance_leg_set = min(
                BALANCE_LEG_SET_MAX_M,
                self.bridge.balance_leg_set + BALANCE_LEG_SET_STEP_M,
            )
        elif down_edge:
            self.bridge.balance_leg_set = max(
                BALANCE_LEG_SET_MIN_M,
                self.bridge.balance_leg_set - BALANCE_LEG_SET_STEP_M,
            )
        self._prev_dpad_bits = bits

    def _gamepad_legjog_integrate(self):
        # Leg-jog mode (suspended Jacobian test):
        #   Left  stick -> LEFT  leg (index 1): Y = leg length, X = swing theta
        #   Right stick -> RIGHT leg (index 0): Y = leg length, X = swing theta
        # Both axes integrate (hold target on release).
        gp = self.gamepad_panel
        lx = _apply_deadzone(gp.cur_lx / 32768.0)
        ly = _apply_deadzone(gp.cur_ly / 32768.0)
        rx = _apply_deadzone(gp.cur_rx / 32768.0)
        ry = _apply_deadzone(gp.cur_ry / 32768.0)

        def step_leg(k, sy, sx):
            self.bridge.legjog_l0[k] = max(
                LEGJOG_L0_MIN_M,
                min(LEGJOG_L0_MAX_M,
                    self.bridge.legjog_l0[k] + sy * LEGJOG_L0_RATE * _GAMEPAD_TICK_S))
            self.bridge.legjog_theta[k] = max(
                -LEGJOG_THETA_MAX_RAD,
                min(LEGJOG_THETA_MAX_RAD,
                    self.bridge.legjog_theta[k] + sx * LEGJOG_THETA_RATE * _GAMEPAD_TICK_S))

        step_leg(0, ry, rx)   # right leg from right stick
        step_leg(1, ly, lx)   # left leg from left stick

    # -- Keyboard control ---------------------------------------------

    def keyPressEvent(self, ev: QKeyEvent):
        if ev.isAutoRepeat():
            return
        if self.selected_joint < 0 or not self.bridge.armed:
            return super().keyPressEvent(ev)

        jdef = JOINTS[self.selected_joint]
        delta = 0.0

        if ev.key() == Qt.Key_Up:
            delta = jdef.coarse_inc
        elif ev.key() == Qt.Key_Down:
            delta = -jdef.coarse_inc
        elif ev.key() == Qt.Key_Right:
            delta = jdef.fine_inc
        elif ev.key() == Qt.Key_Left:
            delta = -jdef.fine_inc
        else:
            return super().keyPressEvent(ev)

        if jdef.is_wheel:
            # `delta` units = rpm; track wheel_rpm_des and convert to rad/s.
            k = WHEEL_INDICES.index(self.selected_joint)
            new_rpm = self.bridge.wheel_rpm_des[k] + delta
            new_rpm = max(-WHEEL_MAX_RPM, min(WHEEL_MAX_RPM, new_rpm))
            self.bridge.wheel_rpm_des[k] = new_rpm
            self.bridge.pos_des[self.selected_joint] = new_rpm * RPM_TO_RAD_PER_S
        else:
            new_val = self.bridge.pos_des[self.selected_joint] + delta
            if jdef.pos_max > jdef.pos_min:
                new_val = max(jdef.pos_min, min(jdef.pos_max, new_val))
            self.bridge.pos_des[self.selected_joint] = new_val

    def select_joint(self, idx: int):
        self.selected_joint = idx
        for jw in self.viz.hip_circles.values():
            jw.selected = (jw.jdef.idx == idx)
        for ww in self.viz.wheel_circles.values():
            ww.selected = (ww.jdef.idx == idx)
        # Keep keyboard focus on the main window so arrow-key jog keeps working.
        self.setFocus(Qt.OtherFocusReason)

    def _can_zero(self) -> bool:
        return (self.bridge.connected and
                self.bridge.sys_state == STATE_ENABLED)

    def set_zero(self, idx: int):
        if not self._can_zero():
            self.lbl_status.setText("ZERO BLOCKED: must be in ENABLED state")
            self.lbl_status.setStyleSheet(
                "color: #f80; font: 11pt 'Segoe UI'; font-weight: bold;"
            )
            return
        self.bridge.send_ctrl(CMD_SET_ZERO, idx)
        self.bridge.pos_des[idx] = 0.0
        if JOINTS[idx].is_wheel:
            k = WHEEL_INDICES.index(idx)
            self.bridge.wheel_rpm_des[k] = 0.0


# -- Robot visualisation composite widget -------------------------------------

class RobotVizWidget(QWidget):
    def __init__(self, main_win: MainWindow):
        super().__init__(main_win)
        self.main_win = main_win
        self.hip_circles: dict[int, JointCircle] = {}
        self.wheel_circles: dict[int, WheelCircle] = {}
        self.zero_buttons: dict[int, QPushButton] = {}

        zbtn_style = (
            "QPushButton { background: #555; color: #ccc; border: 1px solid #777; "
            "border-radius: 4px; padding: 2px 5px; font: 9pt 'Segoe UI'; } "
            "QPushButton:hover { background: #666; }"
        )

        for idx in HIP_INDICES:
            jdef = JOINTS[idx]
            jw = JointCircle(jdef, self)
            jw.clicked.connect(main_win.select_joint)
            self.hip_circles[idx] = jw

            zbtn = QPushButton("zero", self)
            zbtn.setStyleSheet(zbtn_style)
            zbtn.setToolTip(f"Set zero: {jdef.name} (ENABLED only)")
            zbtn.setFixedSize(42, 22)
            zbtn.setFocusPolicy(Qt.NoFocus)
            zbtn.clicked.connect(lambda checked=False, i=idx: main_win.set_zero(i))
            self.zero_buttons[idx] = zbtn

        for idx in WHEEL_INDICES:
            jdef = JOINTS[idx]
            ww = WheelCircle(jdef, self)
            ww.clicked.connect(main_win.select_joint)
            self.wheel_circles[idx] = ww
            # Wheel "zero" sets target rpm to 0 (does not flash any motor zero)
            zbtn = QPushButton("stop", self)
            zbtn.setStyleSheet(zbtn_style)
            zbtn.setToolTip(f"Reset wheel target to 0 rpm")
            zbtn.setFixedSize(42, 22)
            zbtn.setFocusPolicy(Qt.NoFocus)
            zbtn.clicked.connect(lambda checked=False, i=idx: main_win.set_zero(i))
            self.zero_buttons[idx] = zbtn

    def resizeEvent(self, ev):
        self._layout_children()

    def _layout_children(self):
        w, h = self.width(), self.height()
        cx = w // 2
        cy = h // 2 - 20

        # Mirror layout: viewer faces the robot, so R is on screen-left.
        # Right leg (screen-left): hip1, hip2, wheel
        self._place(0, cx - 320, cy - 110)    # R_hip1
        self._place(1, cx - 320, cy + 60)     # R_hip2
        self._place(2, cx - 160, cy + 60)     # R_wheel

        # Left leg (screen-right)
        self._place(3, cx + 200, cy - 110)    # L_hip1
        self._place(4, cx + 200, cy + 60)     # L_hip2
        self._place(5, cx + 40,  cy + 60)     # L_wheel

    def _place(self, idx: int, x: int, y: int):
        widget = self.hip_circles.get(idx) or self.wheel_circles.get(idx)
        if widget is None:
            return
        widget.move(x, y)
        zbtn = self.zero_buttons.get(idx)
        if zbtn:
            zbtn.move(x + 44, y - 4)

    def paintEvent(self, ev: QPaintEvent):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        cx = w // 2

        p.setPen(QPen(QColor(120, 120, 120), 1, Qt.DashLine))
        p.drawLine(cx, 14, cx, h - 10)

        p.setFont(QFont("Segoe UI", 13))
        p.setPen(QColor(160, 160, 160))
        p.drawText(QRectF(0, 2, cx, 26), Qt.AlignCenter, "RIGHT")
        p.drawText(QRectF(cx, 2, cx, 26), Qt.AlignCenter, "LEFT")

        p.end()


# -- Entry point --------------------------------------------------------------

def find_stm32_port() -> str | None:
    for p in serial.tools.list_ports.comports():
        vid = p.vid or 0
        desc = (p.description or "").lower()
        if vid == 0x0483 or "stm" in desc:
            return p.device
    return None

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    win = MainWindow()
    screen = app.primaryScreen()
    available = screen.availableGeometry() if screen is not None else None
    if available is not None and (available.width() <= 1100 or available.height() <= 800):
        win.showMaximized()
    else:
        win.show()

    stm_port = find_stm32_port()
    if stm_port:
        idx = win.combo_port.findData(stm_port)
        if idx >= 0:
            win.combo_port.setCurrentIndex(idx)

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
