#!/usr/bin/env python3
"""
4-channel software remote over USB CDC.
Keys: Arrow (ch0/1), Q/A (ch2), W/S (ch3). Modes: Spring (return to 0) / Hold (keep value).
Sends 9-byte frames: 4 x int16 LE + 1 mode byte. Range -4096..4095 per channel.
"""

import sys
import struct
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install: pip install pyserial")
    sys.exit(1)

# Try tkinter for UI (optional)
try:
    import tkinter as tk
    from tkinter import ttk
    from tkinter import messagebox
    HAS_TK = True
except ImportError:
    HAS_TK = False

# Protocol
REMOTE_FRAME_SIZE = 9
CH_MIN = -4096
CH_MAX = 4095
RAMP_PER_SEC = 2500  # units per second when key held
SEND_RATE_HZ = 50
KEY_MAP = {
    "Up": (0, 1), "Down": (0, -1),
    "Left": (1, -1), "Right": (1, 1),
    "q": (2, 1), "a": (2, -1),
    "w": (3, -1), "s": (3, 1),
}


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def build_remote_frame(channels, mode_byte):
    buf = bytearray(REMOTE_FRAME_SIZE)
    for i in range(4):
        v = max(CH_MIN, min(CH_MAX, channels[i]))
        struct.pack_into("<h", buf, i * 2, v)
    buf[8] = mode_byte & 0x0F
    return bytes(buf)


class RemoteState:
    def __init__(self):
        self.channels = [0] * 4
        self.mode_byte = 0  # bit N: 0=spring, 1=hold for channel N
        self.keys_down = set()
        self.last_send = 0.0
        self.serial_port = None
        self.running = True
        self.lock = threading.Lock()

    def key_down(self, key):
        k = key.lower() if len(key) == 1 else key
        if k in KEY_MAP:
            self.keys_down.add(k)

    def key_up(self, key):
        k = key.lower() if len(key) == 1 else key
        self.keys_down.discard(k)
        ch_idx, _ = KEY_MAP.get(k, (None, None))
        if ch_idx is not None and not self.is_hold(ch_idx):
            # Only return to 0 if no other key for this channel is still down
            any_other = any(KEY_MAP.get(o, (None, None))[0] == ch_idx for o in self.keys_down)
            if not any_other:
                self.channels[ch_idx] = 0

    def is_hold(self, ch_idx):
        return (self.mode_byte >> ch_idx) & 1

    def set_mode(self, ch_idx, hold):
        if hold:
            self.mode_byte |= 1 << ch_idx
        else:
            self.mode_byte &= ~(1 << ch_idx)

    def tick(self, dt):
        for k in list(self.keys_down):
            if k not in KEY_MAP:
                continue
            ch_idx, direction = KEY_MAP[k]
            step = int(RAMP_PER_SEC * dt * direction)
            with self.lock:
                self.channels[ch_idx] = max(CH_MIN, min(CH_MAX, self.channels[ch_idx] + step))

    def get_snapshot(self):
        with self.lock:
            return list(self.channels), self.mode_byte


def send_loop(state):
    interval = 1.0 / SEND_RATE_HZ
    last_tick = time.perf_counter()
    while state.running:
        now = time.perf_counter()
        dt = now - last_tick
        last_tick = now
        state.tick(dt)
        if now - state.last_send >= interval and state.serial_port and state.serial_port.is_open:
            ch, mode = state.get_snapshot()
            frame = build_remote_frame(ch, mode)
            try:
                state.serial_port.write(frame)
                state.last_send = now
            except Exception:
                pass
        time.sleep(0.01)


def run_console(port_name):
    state = RemoteState()
    try:
        state.serial_port = serial.Serial(port_name, 115200, timeout=0.01)
    except Exception as e:
        print("Open port failed:", e)
        return
    print("Connected to", port_name)
    print("Keys: Up/Down (ch0), Left/Right (ch1), Q/A (ch2), W/S (ch3). Spring=default, toggle with 1-4 for hold.")
    print("Press Ctrl+C to quit.")
    t = threading.Thread(target=send_loop, args=(state,), daemon=True)
    t.start()
    try:
        while True:
            time.sleep(0.05)
            ch, _ = state.get_snapshot()
            print("\r ch0:%+5d ch1:%+5d ch2:%+5d ch3:%+5d   " % tuple(ch), end="")
    except KeyboardInterrupt:
        pass
    state.running = False
    state.serial_port.close()


def run_ui(port_name):
    if not HAS_TK:
        print("tkinter not available, run in console mode.")
        run_console(port_name)
        return

    state = RemoteState()
    try:
        state.serial_port = serial.Serial(port_name, 115200, timeout=0.01)
    except Exception as e:
        print("Open port failed:", e)
        return

    root = tk.Tk()
    root.title("4CH USB Remote")
    root.geometry("420x280")
    root.resizable(True, True)

    # Key bind
    def on_key(e):
        if e.keysym and e.type == "2":  # KeyPress
            state.key_down(e.keysym)
        if e.char and e.type == "2":
            state.key_down(e.char)

    def on_key_release(e):
        if e.keysym:
            state.key_up(e.keysym)
        if e.char:
            state.key_up(e.char)

    root.bind("<KeyPress>", on_key)
    root.bind("<KeyRelease>", on_key_release)
    root.focus_set()

    # Channel bars and mode toggles
    labels = []
    bars = []
    mode_vars = []

    for i in range(4):
        names = ("Ch0 前后", "Ch1 左右", "Ch2 上/下", "Ch3 逆/顺")
        fr = ttk.Frame(root, padding=4)
        fr.pack(fill=tk.X)
        ttk.Label(fr, text=names[i], width=10).pack(side=tk.LEFT)
        var = tk.IntVar(value=0)
        mode_vars.append(var)
        cb = ttk.Checkbutton(fr, text="Hold", variable=var,
                            command=lambda idx=i, v=var: state.set_mode(idx, v.get()))
        cb.pack(side=tk.LEFT, padx=8)
        bar = ttk.Progressbar(fr, length=280, mode="determinate")
        bar.pack(side=tk.LEFT, fill=tk.X, expand=True)
        bars.append(bar)
        lab = ttk.Label(fr, text="0", width=6)
        lab.pack(side=tk.RIGHT)
        labels.append(lab)

    status = ttk.Label(root, text=f"Port: {port_name} | Send {SEND_RATE_HZ} Hz")
    status.pack(pady=4)

    def update_ui():
        if not state.running:
            return
        ch, mode = state.get_snapshot()
        for i in range(4):
            v = ch[i]
            # Progressbar expects 0..100
            pct = (v - CH_MIN) / (CH_MAX - CH_MIN) * 100
            bars[i]["value"] = pct
            labels[i]["text"] = str(v)
        root.after(50, update_ui)

    t = threading.Thread(target=send_loop, args=(state,), daemon=True)
    t.start()
    root.after(50, update_ui)

    def on_closing():
        state.running = False
        if state.serial_port:
            state.serial_port.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


def choose_port_dialog():
    """Show a small dialog to select COM port. Returns selected port string or None."""
    if not HAS_TK:
        return None
    ports = list_ports()
    if not ports:
        root = tk.Tk()
        root.withdraw()
        tk.messagebox.showerror("No ports", "No COM ports found. Connect STM32 USB.")
        root.destroy()
        return None
    choice = [None]

    root = tk.Tk()
    root.title("Select COM port")
    root.resizable(False, False)
    ttk.Label(root, text="COM port:").pack(pady=(10, 2))
    var = tk.StringVar(value=ports[0])
    cb = ttk.Combobox(root, textvariable=var, values=ports, state="readonly", width=16)
    cb.pack(pady=4)
    if len(ports) > 1:
        cb.current(0)

    def on_ok():
        choice[0] = var.get()
        root.destroy()

    def on_refresh():
        new_ports = list_ports()
        if new_ports:
            cb["values"] = new_ports
            var.set(new_ports[0])
            if len(new_ports) > 1:
                cb.current(0)

    btn_fr = ttk.Frame(root)
    btn_fr.pack(pady=10)
    ttk.Button(btn_fr, text="Refresh", command=on_refresh).pack(side=tk.LEFT, padx=4)
    ttk.Button(btn_fr, text="Connect", command=on_ok).pack(side=tk.LEFT, padx=4)
    root.mainloop()
    return choice[0]


def main():
    ports = list_ports()
    if not ports:
        print("No COM ports found. Connect STM32 USB.")
        return
    port_name = None
    if len(sys.argv) > 1 and sys.argv[1] != "--console":
        port_name = sys.argv[1]
    if port_name is None and HAS_TK and ("--console" not in sys.argv):
        port_name = choose_port_dialog()
    if port_name is None:
        if not ports:
            return
        port_name = ports[0]
        if len(ports) > 1:
            print("Available:", ports)
            print("Using:", port_name)
    if "--console" in sys.argv:
        run_console(port_name)
    else:
        run_ui(port_name)


if __name__ == "__main__":
    main()
