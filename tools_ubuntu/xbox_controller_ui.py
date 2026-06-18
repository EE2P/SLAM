# -*- coding: utf-8 -*-
"""Xbox controller (XInput) visual mapper: all buttons + analog channels. Windows only. No pip deps."""
from __future__ import annotations

import ctypes
import os
import sys
import tkinter as tk
from ctypes import wintypes
from tkinter import ttk

# Ensure sibling import works regardless of cwd
_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
if _TOOLS_DIR not in sys.path:
    sys.path.insert(0, _TOOLS_DIR)

from xbox_controller_read import (  # noqa: E402
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


def _norm_stick(v: int) -> float:
    return max(-1.0, min(1.0, v / 32768.0))


class StickCanvas(tk.Canvas):
    def __init__(self, master: tk.Misc, title: str, size: int = 140, **kw):
        super().__init__(
            master,
            width=size,
            height=size,
            highlightthickness=1,
            highlightbackground="#555",
            bg="#1e1e1e",
            **kw,
        )
        self._size = size
        self._cx = size // 2
        self._cy = size // 2
        self._r = (size // 2) - 14
        self.create_text(self._cx, 12, text=title, fill="#ccc", font=("Segoe UI", 9))
        self._cross = self.create_line(
            self._cx - self._r,
            self._cy,
            self._cx + self._r,
            self._cy,
            fill="#444",
        )
        self._cross_v = self.create_line(
            self._cx,
            self._cy - self._r,
            self._cx,
            self._cy + self._r,
            fill="#444",
        )
        self._outline = self.create_oval(
            self._cx - self._r,
            self._cy - self._r,
            self._cx + self._r,
            self._cy + self._r,
            outline="#666",
        )
        self._dot = self.create_oval(0, 0, 0, 0, outline="", fill="#4fc3f7")
        self._raw = self.create_text(
            self._cx,
            size - 8,
            text="0, 0",
            fill="#888",
            font=("Consolas", 8),
        )

    def set_stick(self, sx: int, sy: int) -> None:
        nx = _norm_stick(sx)
        ny = _norm_stick(sy)
        dx = nx * self._r
        dy = -ny * self._r
        x1 = self._cx + dx - 6
        y1 = self._cy + dy - 6
        x2 = self._cx + dx + 6
        y2 = self._cy + dy + 6
        self.coords(self._dot, x1, y1, x2, y2)
        self.itemconfigure(self._raw, text=f"{sx:6d}, {sy:6d}")


class TriggerBar(tk.Canvas):
    def __init__(self, master: tk.Misc, label: str, width: int = 200, height: int = 22, **kw):
        super().__init__(
            master,
            width=width,
            height=height,
            highlightthickness=0,
            bg="#1e1e1e",
            **kw,
        )
        # Do not use self._w: tkinter reserves it for the Tcl widget path.
        self._track_w = width - 4
        self._track_h = height - 6
        self.create_text(2, height // 2, text=label, anchor="w", fill="#aaa", font=("Segoe UI", 9))
        x0, y0 = 48, 4
        self._bg = self.create_rectangle(
            x0, y0, x0 + self._track_w, y0 + self._track_h, fill="#333", outline="#555"
        )
        self._fill = self.create_rectangle(x0, y0, x0, y0 + self._track_h, fill="#81c784", outline="")
        self._val = self.create_text(
            x0 + self._track_w + 8,
            height // 2,
            text="0",
            anchor="w",
            fill="#ccc",
            font=("Consolas", 9),
        )
        self._x0 = x0
        self._y0 = y0

    def set_value(self, v: int) -> None:
        v = max(0, min(255, v))
        x1 = self._x0
        y1 = self._y0
        frac = v / 255.0
        x2 = self._x0 + int(self._track_w * frac)
        y2 = self._y0 + self._track_h
        self.coords(self._fill, x1, y1, x2, y2)
        self.itemconfigure(self._val, text=str(v))


class XboxMapperApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("XInput mapper (Xbox controller)")
        self.configure(bg="#252526")
        self.minsize(720, 520)

        if sys.platform != "win32":
            ttk.Label(self, text="Windows + XInput only.").pack(padx=20, pady=20)
            return

        self._xinput = _load_xinput()
        if self._xinput is None:
            ttk.Label(self, text="Could not load XInput DLL.").pack(padx=20, pady=20)
            return

        self._xinput.XInputGetState.argtypes = [wintypes.DWORD, ctypes.POINTER(XINPUT_STATE)]
        self._xinput.XInputGetState.restype = wintypes.DWORD

        top = tk.Frame(self, bg="#252526")
        top.pack(fill=tk.X, padx=10, pady=8)
        tk.Label(top, text="Player index", fg="#ccc", bg="#252526").pack(side=tk.LEFT)
        self._slot = tk.IntVar(value=0)
        sb = tk.Spinbox(
            top,
            from_=0,
            to=3,
            width=4,
            textvariable=self._slot,
            command=self._on_slot_change,
            font=("Segoe UI", 10),
        )
        sb.pack(side=tk.LEFT, padx=(6, 16))
        self._status = tk.Label(top, text="Polling...", fg="#ff9800", bg="#252526", font=("Segoe UI", 10, "bold"))
        self._status.pack(side=tk.LEFT, padx=8)
        self._pkt = tk.Label(top, text="pkt=-", fg="#888", bg="#252526", font=("Consolas", 9))
        self._pkt.pack(side=tk.LEFT)

        body = tk.Frame(self, bg="#252526")
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=4)

        # Shoulders + triggers row
        sh = tk.Frame(body, bg="#252526")
        sh.pack(fill=tk.X, pady=4)
        self._btn_lb = self._make_face_button(sh, "LB")
        self._btn_lb.pack(side=tk.LEFT, padx=(0, 40))
        self._lt = TriggerBar(sh, "LT")
        self._lt.pack(side=tk.LEFT, padx=8)
        tk.Frame(sh, width=40, bg="#252526").pack(side=tk.LEFT, expand=True)
        self._btn_rb = self._make_face_button(sh, "RB")
        self._btn_rb.pack(side=tk.RIGHT, padx=(40, 0))
        self._rt = TriggerBar(sh, "RT")
        self._rt.pack(side=tk.RIGHT, padx=8)

        mid = tk.Frame(body, bg="#252526")
        mid.pack(fill=tk.BOTH, expand=True, pady=8)

        left_col = tk.Frame(mid, bg="#252526")
        left_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._dpad_frame = tk.Frame(left_col, bg="#252526")
        self._dpad_frame.pack(pady=12)
        self._dpad_widgets = self._build_dpad(self._dpad_frame)

        center_col = tk.Frame(mid, bg="#252526")
        center_col.pack(side=tk.LEFT, padx=20)
        self._btn_y = self._make_face_button(center_col, "Y")
        self._btn_y.grid(row=0, column=1, pady=2)
        self._btn_x = self._make_face_button(center_col, "X")
        self._btn_x.grid(row=1, column=0, padx=2)
        self._btn_b = self._make_face_button(center_col, "B")
        self._btn_b.grid(row=1, column=2, padx=2)
        self._btn_a = self._make_face_button(center_col, "A")
        self._btn_a.grid(row=2, column=1, pady=2)

        right_col = tk.Frame(mid, bg="#252526")
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        row_m = tk.Frame(right_col, bg="#252526")
        row_m.pack(pady=8)
        self._btn_back = self._make_face_button(row_m, "Back")
        self._btn_back.pack(side=tk.LEFT, padx=4)
        self._btn_start = self._make_face_button(row_m, "Start")
        self._btn_start.pack(side=tk.LEFT, padx=4)
        row_t = tk.Frame(right_col, bg="#252526")
        row_t.pack(pady=4)
        self._btn_l3 = self._make_face_button(row_t, "L3 (L stick)")
        self._btn_l3.pack(side=tk.LEFT, padx=4)
        self._btn_r3 = self._make_face_button(row_t, "R3 (R stick)")
        self._btn_r3.pack(side=tk.LEFT, padx=4)

        sticks = tk.Frame(body, bg="#252526")
        sticks.pack(fill=tk.X, pady=8)
        self._stick_l = StickCanvas(sticks, "Left stick (LX, LY)")
        self._stick_l.pack(side=tk.LEFT, padx=(0, 24), expand=True)
        self._stick_r = StickCanvas(sticks, "Right stick (RX, RY)")
        self._stick_r.pack(side=tk.LEFT, padx=(24, 0), expand=True)

        foot = tk.Frame(self, bg="#252526")
        foot.pack(fill=tk.X, padx=10, pady=(0, 8))
        self._raw_line = tk.Label(
            foot,
            text="",
            fg="#777",
            bg="#252526",
            font=("Consolas", 9),
            justify=tk.LEFT,
        )
        self._raw_line.pack(anchor="w")

        self._btn_map: list[tuple[int, tk.Label]] = [
            (GAMEPAD_DPAD_UP, self._dpad_widgets["U"]),
            (GAMEPAD_DPAD_DOWN, self._dpad_widgets["D"]),
            (GAMEPAD_DPAD_LEFT, self._dpad_widgets["L"]),
            (GAMEPAD_DPAD_RIGHT, self._dpad_widgets["R"]),
            (GAMEPAD_A, self._btn_a),
            (GAMEPAD_B, self._btn_b),
            (GAMEPAD_X, self._btn_x),
            (GAMEPAD_Y, self._btn_y),
            (GAMEPAD_BACK, self._btn_back),
            (GAMEPAD_START, self._btn_start),
            (GAMEPAD_LEFT_THUMB, self._btn_l3),
            (GAMEPAD_RIGHT_THUMB, self._btn_r3),
            (GAMEPAD_LEFT_SHOULDER, self._btn_lb),
            (GAMEPAD_RIGHT_SHOULDER, self._btn_rb),
        ]

        self._poll_ms = 16
        self.after(self._poll_ms, self._tick)

    def _on_slot_change(self) -> None:
        pass

    @staticmethod
    def _make_face_button(parent: tk.Misc, text: str) -> tk.Label:
        w = tk.Label(
            parent,
            text=text,
            width=max(10, len(text) + 2),
            height=2,
            fg="#e0e0e0",
            bg="#3c3c3c",
            relief=tk.FLAT,
            font=("Segoe UI", 9, "bold"),
            padx=6,
            pady=4,
        )
        return w

    def _set_btn(self, w: tk.Label, on: bool) -> None:
        w.configure(bg="#1976d2" if on else "#3c3c3c")

    def _build_dpad(self, parent: tk.Misc) -> dict[str, tk.Label]:
        g = tk.Frame(parent, bg="#252526")
        g.pack()
        u = self._make_face_button(g, "Up")
        l = self._make_face_button(g, "Left")
        d = self._make_face_button(g, "Down")
        r = self._make_face_button(g, "Right")
        u.grid(row=0, column=1, pady=2)
        l.grid(row=1, column=0, padx=2)
        d.grid(row=2, column=1, pady=2)
        r.grid(row=1, column=2, padx=2)
        tk.Label(g, text="D-Pad", fg="#888", bg="#252526", font=("Segoe UI", 8)).grid(
            row=3, column=0, columnspan=3, pady=(6, 0)
        )
        return {"U": u, "D": d, "L": l, "R": r}

    def _tick(self) -> None:
        if self._xinput is None:
            return
        slot = int(self._slot.get())
        ret, st = read_state(self._xinput, slot)
        if ret != ERROR_SUCCESS or st is None:
            self._status.configure(text="Not connected (slot %d)" % slot, fg="#f44336")
            self._pkt.configure(text="pkt=-")
            g = XINPUT_GAMEPAD()
            g.wButtons = 0
            g.bLeftTrigger = g.bRightTrigger = 0
            g.sThumbLX = g.sThumbLY = g.sThumbRX = g.sThumbRY = 0
            self._apply_visual(g, connected=False)
        else:
            self._status.configure(text="Connected (slot %d)" % slot, fg="#8bc34a")
            self._pkt.configure(text="pkt=%d" % int(st.dwPacketNumber))
            self._apply_visual(st.Gamepad, connected=True)
        self.after(self._poll_ms, self._tick)

    def _apply_visual(self, g: XINPUT_GAMEPAD, connected: bool) -> None:
        self._lt.set_value(g.bLeftTrigger if connected else 0)
        self._rt.set_value(g.bRightTrigger if connected else 0)
        self._stick_l.set_stick(g.sThumbLX, g.sThumbLY)
        self._stick_r.set_stick(g.sThumbRX, g.sThumbRY)
        bits = g.wButtons if connected else 0
        for mask, w in self._btn_map:
            self._set_btn(w, bool(bits & mask))
        if connected:
            self._raw_line.configure(
                text=(
                    "wButtons=0x%04X | LT=%3d RT=%3d | LX=%6d LY=%6d | RX=%6d RY=%6d"
                    % (
                        int(bits),
                        int(g.bLeftTrigger),
                        int(g.bRightTrigger),
                        int(g.sThumbLX),
                        int(g.sThumbLY),
                        int(g.sThumbRX),
                        int(g.sThumbRY),
                    )
                )
            )
        else:
            self._raw_line.configure(text="")


def main() -> int:
    if sys.platform != "win32":
        print("Windows only.")
        return 1
    app = XboxMapperApp()
    app.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
