# -*- coding: utf-8 -*-
"""Read Xbox-compatible gamepad on Windows (USB or Bluetooth) via XInput. No pip deps."""
from __future__ import annotations

import ctypes
import sys
import time
from ctypes import wintypes

ERROR_SUCCESS = 0
ERROR_DEVICE_NOT_CONNECTED = 0x48F

# https://learn.microsoft.com/en-us/windows/win32/api/xinput/ns-xinput-xinput_gamepad
GAMEPAD_DPAD_UP = 0x0001
GAMEPAD_DPAD_DOWN = 0x0002
GAMEPAD_DPAD_LEFT = 0x0004
GAMEPAD_DPAD_RIGHT = 0x0008
GAMEPAD_START = 0x0010
GAMEPAD_BACK = 0x0020
GAMEPAD_LEFT_THUMB = 0x0040
GAMEPAD_RIGHT_THUMB = 0x0080
GAMEPAD_LEFT_SHOULDER = 0x0100
GAMEPAD_RIGHT_SHOULDER = 0x0200
GAMEPAD_A = 0x1000
GAMEPAD_B = 0x2000
GAMEPAD_X = 0x4000
GAMEPAD_Y = 0x8000


class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ("wButtons", wintypes.WORD),
        ("bLeftTrigger", wintypes.BYTE),
        ("bRightTrigger", wintypes.BYTE),
        ("sThumbLX", wintypes.SHORT),
        ("sThumbLY", wintypes.SHORT),
        ("sThumbRX", wintypes.SHORT),
        ("sThumbRY", wintypes.SHORT),
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ("dwPacketNumber", wintypes.DWORD),
        ("Gamepad", XINPUT_GAMEPAD),
    ]


def _load_xinput():
    for name in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            return ctypes.WinDLL(name)
        except OSError:
            continue
    return None


def _buttons_str(w: int) -> str:
    labels = []
    mapping = [
        (GAMEPAD_DPAD_UP, "DPAD_U"),
        (GAMEPAD_DPAD_DOWN, "DPAD_D"),
        (GAMEPAD_DPAD_LEFT, "DPAD_L"),
        (GAMEPAD_DPAD_RIGHT, "DPAD_R"),
        (GAMEPAD_START, "START"),
        (GAMEPAD_BACK, "BACK"),
        (GAMEPAD_LEFT_THUMB, "L3"),
        (GAMEPAD_RIGHT_THUMB, "R3"),
        (GAMEPAD_LEFT_SHOULDER, "LB"),
        (GAMEPAD_RIGHT_SHOULDER, "RB"),
        (GAMEPAD_A, "A"),
        (GAMEPAD_B, "B"),
        (GAMEPAD_X, "X"),
        (GAMEPAD_Y, "Y"),
    ]
    for mask, text in mapping:
        if w & mask:
            labels.append(text)
    return "+".join(labels) if labels else "-"


def read_state(xinput, user_index: int) -> tuple[int, XINPUT_STATE | None]:
    state = XINPUT_STATE()
    ret = xinput.XInputGetState(user_index, ctypes.byref(state))
    if ret == ERROR_SUCCESS:
        return ret, state
    return ret, None


def main() -> int:
    if sys.platform != "win32":
        print("This script uses Windows XInput only.")
        return 1

    xinput = _load_xinput()
    if xinput is None:
        print("Could not load xinput1_4 / xinput1_3 / xinput9_1_0.")
        return 1

    xinput.XInputGetState.argtypes = [wintypes.DWORD, ctypes.POINTER(XINPUT_STATE)]
    xinput.XInputGetState.restype = wintypes.DWORD

    print("Polling XInput slots 0-3 (50 Hz). Pair/connect the Xbox controller first.")
    print("If nothing prints: Settings -> Bluetooth -> confirm gamepad is connected.\n")

    last_print: dict[int, str] = {}
    try:
        while True:
            for slot in range(4):
                ret, st = read_state(xinput, slot)
                if ret != ERROR_SUCCESS or st is None:
                    continue
                g = st.Gamepad
                line = (
                    f"[{slot}] pkt={st.dwPacketNumber} "
                    f"btn={_buttons_str(g.wButtons)} "
                    f"LT={g.bLeftTrigger:3d} RT={g.bRightTrigger:3d} "
                    f"LX={g.sThumbLX:6d} LY={g.sThumbLY:6d} "
                    f"RX={g.sThumbRX:6d} RY={g.sThumbRY:6d}"
                )
                if last_print.get(slot) != line:
                    print(line)
                    last_print[slot] = line
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
