# -*- coding: utf-8 -*-
"""Flat Xbox gamepad status panel for Qt (PySide6).

Windows uses XInput. Linux / Ubuntu uses pygame's SDL joystick backend.
"""
from __future__ import annotations

import ctypes
import os
import sys
from ctypes import wintypes
from types import SimpleNamespace

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from xbox_controller_read import (
    ERROR_SUCCESS,
    GAMEPAD_A,
    GAMEPAD_B,
    GAMEPAD_BACK,
    GAMEPAD_DPAD_DOWN,
    GAMEPAD_DPAD_LEFT,
    GAMEPAD_DPAD_RIGHT,
    GAMEPAD_DPAD_UP,
    GAMEPAD_LEFT_SHOULDER,
    GAMEPAD_LEFT_THUMB,
    GAMEPAD_RIGHT_SHOULDER,
    GAMEPAD_RIGHT_THUMB,
    GAMEPAD_START,
    GAMEPAD_X,
    GAMEPAD_Y,
    XINPUT_GAMEPAD,
    XINPUT_STATE,
    _load_xinput,
    read_state,
)

# Align with momento_gui.py greys / accents
COL_BORDER = QColor(119, 119, 119)
COL_INACTIVE = QColor(85, 85, 85)
COL_ACTIVE = QColor(0, 180, 255)
COL_TEXT = QColor(230, 230, 230)
COL_MUTED = QColor(160, 160, 160)
COL_TRIGGER_BG = QColor(90, 90, 90)
COL_TRIGGER_FILL = QColor(100, 220, 100)
COL_STICK_GUIDE = QColor(80, 80, 80)
COL_STICK_DOT = QColor(0, 200, 255)


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)))
    except ValueError:
        return default


def _norm(v: int) -> float:
    return max(-1.0, min(1.0, v / 32768.0))


def _neutral_gamepad():
    return SimpleNamespace(
        wButtons=0,
        bLeftTrigger=0,
        bRightTrigger=0,
        sThumbLX=0,
        sThumbLY=0,
        sThumbRX=0,
        sThumbRY=0,
    )


def _axis_to_short(value: float, invert: bool = False) -> int:
    if invert:
        value = -value
    value = max(-1.0, min(1.0, value))
    return int(value * 32767)


def _trigger_to_byte(value: float) -> int:
    value = max(-1.0, min(1.0, value))
    if value <= -0.9:
        norm = 0.0
    elif value >= 0.0:
        norm = value
    else:
        norm = (value + 1.0) * 0.5
    return int(max(0.0, min(1.0, norm)) * 255)


class _PygameBackend:
    """Small pygame/SDL wrapper that returns XInput-shaped gamepad data."""

    # Default Linux Xbox mapping used by SDL joystick:
    # axes: LX, LY, LT, RX, RY, RT. Override via env if a controller differs.
    AXIS_LX = _env_int("MOMENTO_GAMEPAD_AXIS_LX", 0)
    AXIS_LY = _env_int("MOMENTO_GAMEPAD_AXIS_LY", 1)
    AXIS_LT = _env_int("MOMENTO_GAMEPAD_AXIS_LT", 2)
    AXIS_RX = _env_int("MOMENTO_GAMEPAD_AXIS_RX", 3)
    AXIS_RY = _env_int("MOMENTO_GAMEPAD_AXIS_RY", 4)
    AXIS_RT = _env_int("MOMENTO_GAMEPAD_AXIS_RT", 5)

    BUTTON_MAP = [
        (0, GAMEPAD_A),
        (1, GAMEPAD_B),
        (2, GAMEPAD_X),
        (3, GAMEPAD_Y),
        (4, GAMEPAD_LEFT_SHOULDER),
        (5, GAMEPAD_RIGHT_SHOULDER),
        (6, GAMEPAD_BACK),
        (7, GAMEPAD_START),
        (9, GAMEPAD_LEFT_THUMB),
        (10, GAMEPAD_RIGHT_THUMB),
    ]

    DPAD_BUTTON_MAP = [
        (11, GAMEPAD_DPAD_UP),
        (12, GAMEPAD_DPAD_DOWN),
        (13, GAMEPAD_DPAD_LEFT),
        (14, GAMEPAD_DPAD_RIGHT),
    ]

    def __init__(self):
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        import pygame

        self.pygame = pygame
        pygame.init()
        pygame.joystick.init()
        self._joysticks = {}

    def _axis(self, joy, idx: int) -> float:
        if idx < 0 or idx >= joy.get_numaxes():
            return 0.0
        return float(joy.get_axis(idx))

    def _get_joystick(self, slot: int):
        self.pygame.event.pump()
        count = self.pygame.joystick.get_count()
        if slot < 0 or slot >= count:
            return None
        joy = self._joysticks.get(slot)
        if joy is None or not joy.get_init():
            joy = self.pygame.joystick.Joystick(slot)
            joy.init()
            self._joysticks[slot] = joy
        return joy

    def read(self, slot: int):
        joy = self._get_joystick(slot)
        if joy is None:
            return None, ""

        bits = 0
        button_count = joy.get_numbuttons()
        for button_idx, mask in self.BUTTON_MAP:
            if button_idx < button_count and joy.get_button(button_idx):
                bits |= mask

        if joy.get_numhats() > 0:
            hat_x, hat_y = joy.get_hat(0)
            if hat_y > 0:
                bits |= GAMEPAD_DPAD_UP
            elif hat_y < 0:
                bits |= GAMEPAD_DPAD_DOWN
            if hat_x < 0:
                bits |= GAMEPAD_DPAD_LEFT
            elif hat_x > 0:
                bits |= GAMEPAD_DPAD_RIGHT
        else:
            for button_idx, mask in self.DPAD_BUTTON_MAP:
                if button_idx < button_count and joy.get_button(button_idx):
                    bits |= mask

        gamepad = SimpleNamespace(
            wButtons=bits,
            bLeftTrigger=_trigger_to_byte(self._axis(joy, self.AXIS_LT)),
            bRightTrigger=_trigger_to_byte(self._axis(joy, self.AXIS_RT)),
            sThumbLX=_axis_to_short(self._axis(joy, self.AXIS_LX)),
            sThumbLY=_axis_to_short(self._axis(joy, self.AXIS_LY), invert=True),
            sThumbRX=_axis_to_short(self._axis(joy, self.AXIS_RX)),
            sThumbRY=_axis_to_short(self._axis(joy, self.AXIS_RY), invert=True),
        )
        return gamepad, joy.get_name()


class _FlatChip(QWidget):
    """Small flat label chip; on = accent fill."""

    def __init__(self, text: str, w: int = 36, h: int = 22, parent=None,
                 on_color: QColor | None = None):
        super().__init__(parent)
        self._txt = text
        self._on = False
        self._on_color = on_color or COL_ACTIVE
        self.setFixedSize(w, h)

    def set_on(self, on: bool) -> None:
        if self._on != on:
            self._on = on
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        bg = self._on_color if self._on else COL_INACTIVE
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(bg)
        p.drawRoundedRect(0, 0, self.width() - 1, self.height() - 1, 3, 3)
        p.setPen(COL_TEXT if self._on else COL_MUTED)
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.drawText(self.rect(), Qt.AlignCenter, self._txt)
        p.end()


class _FlatStick(QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self._title = title
        self._sx = 0
        self._sy = 0
        self.setFixedSize(96, 96)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

    def set_values(self, sx: int, sy: int) -> None:
        self._sx = sx
        self._sy = sy
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        m = 4
        s = min(self.width(), self.height()) - 2 * m
        x0 = (self.width() - s) // 2
        y0 = (self.height() - s) // 2 + 6

        p.setPen(COL_MUTED)
        p.setFont(QFont("Segoe UI", 8))
        p.drawText(0, 0, self.width(), 12, Qt.AlignCenter, self._title)

        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(COL_INACTIVE)
        p.drawRoundedRect(x0, y0, s, s, 4, 4)

        cx = x0 + s / 2
        cy = y0 + s / 2
        r = s / 2 - 6
        p.setPen(QPen(COL_STICK_GUIDE, 1))
        p.drawLine(int(cx - r), int(cy), int(cx + r), int(cy))
        p.drawLine(int(cx), int(cy - r), int(cx), int(cy + r))

        nx = _norm(self._sx) * r
        ny = -_norm(self._sy) * r
        p.setPen(Qt.NoPen)
        p.setBrush(COL_STICK_DOT)
        p.drawEllipse(int(cx + nx - 5), int(cy + ny - 5), 10, 10)

        p.setPen(COL_MUTED)
        p.setFont(QFont("Consolas", 7))
        p.drawText(
            x0,
            y0 + s + 2,
            s,
            12,
            Qt.AlignCenter,
            f"{self._sx},{self._sy}",
        )
        p.end()


class _FlatTrigger(QWidget):
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self._label = label
        self._v = 0
        self.setFixedHeight(26)
        self.setMinimumWidth(100)

    def set_value(self, v: int) -> None:
        v = max(0, min(255, v))
        if v != self._v:
            self._v = v
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        h = self.height()
        lw = 28
        p.setPen(COL_MUTED)
        p.setFont(QFont("Segoe UI", 8))
        p.drawText(0, 0, lw, h, Qt.AlignVCenter | Qt.AlignLeft, self._label)

        x0, y0 = lw + 4, 7
        w = self.width() - lw - 36
        bar_h = 10
        p.setPen(QPen(COL_BORDER, 1))
        p.setBrush(COL_TRIGGER_BG)
        p.drawRoundedRect(x0, y0, w, bar_h, 2, 2)
        fill_w = int(w * (self._v / 255.0))
        if fill_w > 0:
            p.setPen(Qt.NoPen)
            p.setBrush(COL_TRIGGER_FILL)
            p.drawRoundedRect(x0, y0, max(2, fill_w), bar_h, 2, 2)

        p.setPen(COL_TEXT)
        p.setFont(QFont("Consolas", 8))
        p.drawText(x0 + w + 6, 0, 28, h, Qt.AlignVCenter | Qt.AlignLeft, str(self._v))
        p.end()


class _FlatDpad(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._bits = 0
        self.setFixedSize(72, 72)

    def set_buttons(self, w: int) -> None:
        if w != self._bits:
            self._bits = w
            self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cx, cy = self.width() / 2, self.height() / 2
        cell = 18
        gap = 2

        def cell_on(mask: int) -> bool:
            return bool(self._bits & mask)

        def draw_cell(dx: float, dy: float, mask: int) -> None:
            x = int(cx + dx * (cell + gap) - cell / 2)
            y = int(cy + dy * (cell + gap) - cell / 2)
            on = cell_on(mask)
            p.setPen(QPen(COL_BORDER, 1))
            p.setBrush(COL_ACTIVE if on else COL_INACTIVE)
            p.drawRoundedRect(x, y, cell, cell, 2, 2)

        draw_cell(0, -1, GAMEPAD_DPAD_UP)
        draw_cell(-1, 0, GAMEPAD_DPAD_LEFT)
        draw_cell(0, 1, GAMEPAD_DPAD_DOWN)
        draw_cell(1, 0, GAMEPAD_DPAD_RIGHT)

        p.end()


class XboxGamepadPanel(QFrame):
    """Flat gamepad mapper; poll XInput on Windows or SDL on Linux."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("xboxGamepadPanel")
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(
            "#xboxGamepadPanel { background-color: rgb(58, 58, 58); border: 1px solid #777; "
            "border-radius: 4px; }"
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(4)

        self._xinput = None
        self._pygame_backend = None
        self._backend_name = "XInput" if sys.platform == "win32" else "SDL"
        self._usable = False

        # Defaults so main window can safely read even when XInput unavailable.
        # Buttons are still tracked so the on-screen chips can light up, but
        # the panel itself no longer drives any control logic via buttons --
        # all robot interaction is via the two analog sticks (see
        # momento_gui._gamepad_integrate).
        self.cur_lx: int = 0
        self.cur_ly: int = 0
        self.cur_rx: int = 0
        self.cur_ry: int = 0
        self.cur_lt: int = 0
        self.cur_rt: int = 0
        self.cur_buttons: int = 0
        self.connected: bool = False

        if sys.platform == "win32":
            self._xinput = _load_xinput()
            if self._xinput is None:
                lab = QLabel("Gamepad: could not load XInput DLL.")
                lab.setStyleSheet("color: #f88; font: 10pt 'Segoe UI';")
                root.addWidget(lab)
                return
            self._xinput.XInputGetState.argtypes = [wintypes.DWORD, ctypes.POINTER(XINPUT_STATE)]
            self._xinput.XInputGetState.restype = wintypes.DWORD
            self._usable = True
        else:
            try:
                self._pygame_backend = _PygameBackend()
                self._usable = True
            except Exception as exc:
                lab = QLabel(f"Gamepad: install pygame for Ubuntu SDL input ({exc}).")
                lab.setStyleSheet("color: #f88; font: 10pt 'Segoe UI';")
                root.addWidget(lab)
                return

        row0 = QHBoxLayout()
        row0.setSpacing(6)
        t = QLabel(f"Xbox ({self._backend_name})")
        t.setStyleSheet("color: #ccc; font: 10pt 'Segoe UI'; font-weight: bold;")
        row0.addWidget(t)

        # Static mapping help (replaces the old A/B leg-select tally lights).
        self._mode_lbl = QLabel(
            "L stick: legs   |   R stick: drive"
        )
        self._mode_lbl.setStyleSheet("color: #aaa; font: 8pt 'Consolas';")
        row0.addWidget(self._mode_lbl)

        row0.addSpacing(8)
        row0.addWidget(QLabel("Player:"))
        self._slot = QComboBox()
        for i in range(4):
            self._slot.addItem(str(i), i)
        self._slot.setFixedWidth(52)
        self._slot.setStyleSheet(
            "QComboBox { background: #555; color: #eee; border: 1px solid #777; "
            "border-radius: 3px; padding: 2px 6px; font: 9pt 'Segoe UI'; }"
        )
        row0.addWidget(self._slot)
        self._status = QLabel("...")
        self._status.setStyleSheet("color: #fa0; font: 9pt 'Segoe UI';")
        row0.addWidget(self._status)
        self._pkt = QLabel("")
        self._pkt.setStyleSheet("color: #888; font: 9pt 'Consolas';")
        row0.addWidget(self._pkt)
        row0.addStretch()
        root.addLayout(row0)

        row2 = QHBoxLayout()
        row2.setSpacing(6)

        left_wrap = QWidget()
        left_col = QVBoxLayout(left_wrap)
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.setSpacing(3)
        self._lb = _FlatChip("LB", 32, 18)
        self._lt = _FlatTrigger("LT")
        self._lt.setFixedSize(92, 22)
        left_col.addStretch(1)
        left_col.addWidget(self._lb, 0, Qt.AlignHCenter)
        left_col.addWidget(self._lt, 0, Qt.AlignHCenter)
        left_col.addStretch(1)
        row2.addWidget(left_wrap)

        self._stick_l = _FlatStick("L stick")
        row2.addWidget(self._stick_l)

        self._dpad = _FlatDpad()
        row2.addWidget(self._dpad)

        face = QGridLayout()
        face.setHorizontalSpacing(4)
        face.setVerticalSpacing(4)
        self._btn_y = _FlatChip("Y", 30, 22)
        self._btn_x = _FlatChip("X", 30, 22)
        self._btn_b = _FlatChip("B", 30, 22)
        self._btn_a = _FlatChip("A", 30, 22)
        face.addWidget(self._btn_y, 0, 1)
        face.addWidget(self._btn_x, 1, 0)
        face.addWidget(self._btn_b, 1, 2)
        face.addWidget(self._btn_a, 2, 1)
        wface = QWidget()
        wface.setLayout(face)
        row2.addWidget(wface)

        meta = QVBoxLayout()
        meta.setSpacing(4)
        row_bs = QHBoxLayout()
        self._btn_back = _FlatChip("Back", 44, 20)
        self._btn_start = _FlatChip("Start", 44, 20)
        row_bs.addWidget(self._btn_back)
        row_bs.addWidget(self._btn_start)
        meta.addLayout(row_bs)
        row_lr = QHBoxLayout()
        self._btn_l3 = _FlatChip("L3", 32, 20)
        self._btn_r3 = _FlatChip("R3", 32, 20)
        row_lr.addWidget(self._btn_l3)
        row_lr.addWidget(self._btn_r3)
        meta.addLayout(row_lr)
        wmeta = QWidget()
        wmeta.setLayout(meta)
        row2.addWidget(wmeta)

        row2.addStretch(1)

        self._stick_r = _FlatStick("R stick")
        row2.addWidget(self._stick_r)

        right_wrap = QWidget()
        right_col = QVBoxLayout(right_wrap)
        right_col.setContentsMargins(0, 0, 0, 0)
        right_col.setSpacing(3)
        self._rb = _FlatChip("RB", 32, 18)
        self._rt = _FlatTrigger("RT")
        self._rt.setFixedSize(92, 22)
        right_col.addStretch(1)
        right_col.addWidget(self._rb, 0, Qt.AlignHCenter)
        right_col.addWidget(self._rt, 0, Qt.AlignHCenter)
        right_col.addStretch(1)
        row2.addWidget(right_wrap)

        root.addLayout(row2)

        self._raw = QLabel("")
        self._raw.setStyleSheet("color: #777; font: 8pt 'Consolas';")
        self._raw.setWordWrap(True)
        root.addWidget(self._raw)

        self._btn_map: list[tuple[int, _FlatChip]] = [
            (GAMEPAD_LEFT_SHOULDER, self._lb),
            (GAMEPAD_RIGHT_SHOULDER, self._rb),
            (GAMEPAD_A, self._btn_a),
            (GAMEPAD_B, self._btn_b),
            (GAMEPAD_X, self._btn_x),
            (GAMEPAD_Y, self._btn_y),
            (GAMEPAD_BACK, self._btn_back),
            (GAMEPAD_START, self._btn_start),
            (GAMEPAD_LEFT_THUMB, self._btn_l3),
            (GAMEPAD_RIGHT_THUMB, self._btn_r3),
        ]

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._poll)
        self._timer.start(16)

    def _poll(self) -> None:
        if not self._usable:
            return
        slot = int(self._slot.currentData())
        if self._xinput is not None:
            ret, st = read_state(self._xinput, slot)
            connected = (ret == ERROR_SUCCESS and st is not None)
            gamepad = st.Gamepad if connected else _neutral_gamepad()
            packet_text = "pkt=%d" % int(st.dwPacketNumber) if connected else ""
        elif self._pygame_backend is not None:
            gamepad, name = self._pygame_backend.read(slot)
            connected = gamepad is not None
            gamepad = gamepad if connected else _neutral_gamepad()
            packet_text = name if connected else ""
        else:
            return

        if not connected:
            self._status.setText("Not connected")
            self._status.setStyleSheet("color: #f66; font: 9pt 'Segoe UI';")
            self._pkt.setText("")
            self._apply(gamepad, False)
            return

        self._status.setText("Connected")
        self._status.setStyleSheet("color: #8c8; font: 9pt 'Segoe UI';")
        self._pkt.setText(packet_text)
        self._apply(gamepad, True)

    def _apply(self, g: XINPUT_GAMEPAD, connected: bool) -> None:
        lt = int(g.bLeftTrigger) if connected else 0
        rt = int(g.bRightTrigger) if connected else 0
        bits = int(g.wButtons) if connected else 0

        # Store live state for the main window integration loop.
        self.connected = connected
        self.cur_lx = int(g.sThumbLX)
        self.cur_ly = int(g.sThumbLY)
        self.cur_rx = int(g.sThumbRX)
        self.cur_ry = int(g.sThumbRY)
        self.cur_lt = lt
        self.cur_rt = rt
        self.cur_buttons = bits

        # Visual widgets only -- no per-leg / per-joint control logic here.
        self._lt.set_value(lt)
        self._rt.set_value(rt)
        self._stick_l.set_values(self.cur_lx, self.cur_ly)
        self._stick_r.set_values(self.cur_rx, self.cur_ry)
        self._dpad.set_buttons(bits)
        for mask, chip in self._btn_map:
            chip.set_on(bool(bits & mask))
        if connected:
            self._raw.setText(
                "wButtons=0x%04X  LT=%3d RT=%3d  LX=%6d LY=%6d  RX=%6d RY=%6d"
                % (bits, lt, rt, self.cur_lx, self.cur_ly, self.cur_rx, self.cur_ry)
            )
        else:
            self._raw.setText("")
