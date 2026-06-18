#!/usr/bin/env python3
"""
Joint Bus — Jetson Nano / PC side example.

Protocol (USB CDC byte stream):
    STM32 → Jetson:  0xAA 0x55 + 20×float32(LE) + 1B CRC-8  = 83 bytes
    Jetson → STM32:  0x55 0xAA + 10×float32(LE) + 1B CRC-8  = 43 bytes

    CRC-8: polynomial 0x07 (x^8+x^2+x+1), init=0x00.

Usage:
    python joint_bus_example.py                          # sine wave on joint 0
    python joint_bus_example.py --port /dev/ttyACM0      # explicit port
    python joint_bus_example.py --garbage                 # send random garbage data
    python joint_bus_example.py --garbage --rate 200      # garbage at 200 Hz
"""

import sys
import struct
import time
import math
import random
import argparse
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install: pip install pyserial")
    sys.exit(1)

# ── Protocol constants ──────────────────────────────────────────────────────

JOINT_COUNT = 10

TX_HDR = bytes([0xAA, 0x55])   # STM32 → Jetson
RX_HDR = bytes([0x55, 0xAA])   # Jetson → STM32

TX_PKT_SIZE = 2 + JOINT_COUNT * 2 * 4 + 1   # 83
RX_PKT_SIZE = 2 + JOINT_COUNT * 4 + 1       # 43

# ── CRC-8 (poly 0x07, init 0x00) ───────────────────────────────────────────

_CRC8_TABLE = None

def _build_crc8_table():
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
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

# ── Packet build / parse ───────────────────────────────────────────────────

def build_cmd_packet(pos_des: list[float]) -> bytes:
    """Build a 43-byte command packet (Jetson → STM32)."""
    assert len(pos_des) == JOINT_COUNT
    body = RX_HDR + struct.pack(f"<{JOINT_COUNT}f", *pos_des)
    return body + bytes([crc8(body)])


def parse_state_packet(raw: bytes):
    """
    Find and parse one 83-byte state packet from raw buffer.
    Returns (pos[10], vel[10], remaining_bytes) or None.
    """
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

        floats = struct.unpack(f"<{JOINT_COUNT * 2}f", body[2:])
        pos = list(floats[:JOINT_COUNT])
        vel = list(floats[JOINT_COUNT:])
        return pos, vel, raw[TX_PKT_SIZE:]

    return None


# ── Garbage data generator ──────────────────────────────────────────────────

def random_pos_des() -> list[float]:
    """Generate 10 random float32 values in [-12.5, 12.5] rad."""
    return [random.uniform(-12.5, 12.5) for _ in range(JOINT_COUNT)]


# ── Auto-detect STM32 CDC port ──────────────────────────────────────────────

def find_stm32_port() -> str | None:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        vid = p.vid or 0
        if vid == 0x0483 or "stm" in desc or "stlink" in desc:
            return p.device
    for p in serial.tools.list_ports.comports():
        if "acm" in (p.device or "").lower() or "usbmodem" in (p.device or "").lower():
            return p.device
    return None


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Joint Bus Jetson/PC example")
    parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyACM0, COM12)")
    parser.add_argument("--rate", type=float, default=100.0, help="Send rate in Hz (default 100)")
    parser.add_argument("--garbage", action="store_true",
                        help="Send random garbage pos_des (all 10 joints) instead of sine wave")
    args = parser.parse_args()

    port = args.port or find_stm32_port()
    if port is None:
        print("No STM32 CDC port found. Specify with --port")
        sys.exit(1)

    mode_str = "GARBAGE (random)" if args.garbage else "SINE (joint 0)"
    print(f"[joint_bus] Opening {port}")
    print(f"[joint_bus] Mode: {mode_str}  Rate: {args.rate:.0f} Hz  CRC-8 enabled")

    ser = serial.Serial(port, timeout=0.005)
    ser.reset_input_buffer()

    rx_buf = b""
    rx_count = 0
    tx_count = 0
    crc_err = 0
    stop = threading.Event()

    last_pos = [0.0] * JOINT_COUNT
    last_vel = [0.0] * JOINT_COUNT

    # ── RX thread ────────────────────────────────────────────────────────
    def rx_loop():
        nonlocal rx_buf, rx_count, crc_err, last_pos, last_vel
        while not stop.is_set():
            chunk = ser.read(256)
            if not chunk:
                continue
            rx_buf += chunk
            while True:
                result = parse_state_packet(rx_buf)
                if result is None:
                    break
                pos, vel, rx_buf = result
                last_pos = pos
                last_vel = vel
                rx_count += 1

    t = threading.Thread(target=rx_loop, daemon=True)
    t.start()

    # ── TX loop ──────────────────────────────────────────────────────────
    print(f"[joint_bus] Running. Ctrl+C to stop.\n")
    period = 1.0 / args.rate
    t0 = time.time()
    print_interval = max(1, int(args.rate))

    try:
        while True:
            loop_start = time.perf_counter()

            if args.garbage:
                pos_des = random_pos_des()
            else:
                elapsed = time.time() - t0
                pos_des = [0.0] * JOINT_COUNT
                pos_des[0] = 0.5 * math.sin(2.0 * math.pi * 0.25 * elapsed)

            pkt = build_cmd_packet(pos_des)
            ser.write(pkt)
            tx_count += 1

            if tx_count % print_interval == 0:
                if args.garbage:
                    print(
                        f"  TX:{tx_count:6d}  RX:{rx_count:6d}  "
                        f"pos_des[0]={pos_des[0]:+8.3f}  pos_des[1]={pos_des[1]:+8.3f}  "
                        f"fb_pos[0]={last_pos[0]:+8.4f}  fb_vel[0]={last_vel[0]:+8.4f}"
                    )
                else:
                    print(
                        f"  TX:{tx_count:6d}  RX:{rx_count:6d}  "
                        f"pos_des[0]={pos_des[0]:+8.3f}  "
                        f"fb_pos[0]={last_pos[0]:+8.4f}  fb_vel[0]={last_vel[0]:+8.4f}"
                    )

            dt = time.perf_counter() - loop_start
            if dt < period:
                time.sleep(period - dt)

    except KeyboardInterrupt:
        print(f"\n[joint_bus] Stopped. TX={tx_count}  RX={rx_count}  CRC_err={crc_err}")

    stop.set()
    ser.close()


if __name__ == "__main__":
    main()
