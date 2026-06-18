#!/usr/bin/env python3
"""
USB CDC throughput and packet loss stress test.
Sends 64-byte packets (magic STST + seq number); STM32 echoes them.
Measures: throughput (MB/s), lost packets, out-of-order.
"""

import sys
import struct
import time
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Install: pip install pyserial")
    sys.exit(1)

MAGIC = b"STST"
MAGIC_STST = b"STST"
MAGIC_STAT = b"STAT"
MAGIC_STOW = b"STOW"
PKT_SIZE = 64
SEQ_OFF = 4
CRC_OFF = 8
STAT_REPLY_SIZE = 8


def crc32_bytes(data):
    crc = 0xFFFFFFFF
    for i in range(len(data)):
        crc ^= data[i]
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if (crc & 1) else 0)
    return crc & 0xFFFFFFFF


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def make_packet(seq, magic=MAGIC_STST, with_crc=False):
    buf = bytearray(PKT_SIZE)
    buf[:4] = magic
    struct.pack_into("<I", buf, SEQ_OFF, seq)
    if with_crc:
        struct.pack_into("<I", buf, CRC_OFF, crc32_bytes(buf[4:64]))
    return bytes(buf)


def run_stress(port_name, num_packets=5000, baud=115200, one_way=False, with_crc=False, duration_sec=None):
    try:
        ser = serial.Serial(port_name, baud, timeout=0.5)
    except Exception as e:
        print("Open port failed:", e)
        return

    print(f"Port: {port_name}, sending {num_packets} packets of {PKT_SIZE} bytes each.")
    if duration_sec is not None:
        print(f"Mode: aggregate (both directions) for {duration_sec} s; host sends STST, STM32 echoes.")
    elif one_way:
        print("Mode: one-way RX (STOW), then STAT request; STM32 replies with total bytes received.")
    else:
        print("Mode: echo (STST); STM32 echoes packets.")
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    if one_way:
        start = time.perf_counter()
        for seq in range(num_packets):
            ser.write(make_packet(seq, magic=MAGIC_STOW))
            if (seq + 1) % 1000 == 0:
                print(f"  sent {seq + 1}/{num_packets} ...")
        ser.write(make_packet(0, magic=MAGIC_STAT))
        ser.flush()
        end_send = time.perf_counter()
        # Time is stopped before waiting for reply → throughput = send rate, not affected by STAT RTT
        reply = ser.read(STAT_REPLY_SIZE)
        elapsed = end_send - start
        if len(reply) >= STAT_REPLY_SIZE and reply[:4] == MAGIC_STAT:
            rx_total, = struct.unpack_from("<I", reply, 4)
            total_bytes = num_packets * PKT_SIZE
            # rx_total includes the STAT request packet (64 bytes), so cap at payload for throughput/loss
            payload_rx = min(rx_total, total_bytes)
            throughput_mbps = (payload_rx / (1024 * 1024)) / elapsed if elapsed > 0 else 0
            print()
            print("One-way RX result:")
            print(f"  Sent:       {num_packets} packets, {total_bytes} bytes")
            print(f"  STM32 received: {rx_total} bytes (includes STAT request if sent)")
            print(f"  Time:       {elapsed:.3f} s")
            print(f"  One-way RX throughput: {throughput_mbps:.2f} MB/s")
            if total_bytes > 0:
                loss_pct = 100.0 * (1.0 - payload_rx / total_bytes)
                print(f"  Implied loss: {loss_pct:.2f}%")
        else:
            print("  No valid STAT reply (expected 8 bytes starting with STAT).")
        ser.close()
        return

    if duration_sec is not None:
        # Aggregate test: run for duration_sec, send as fast as possible, count echoes → (sent + received) / time
        received = {}
        recv_lock = threading.Lock()

        def reader():
            buf = bytearray()
            while reader_running[0]:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                buf += chunk
                while len(buf) >= PKT_SIZE:
                    pkt = bytes(buf[:PKT_SIZE])
                    buf = buf[PKT_SIZE:]
                    if pkt[:4] == MAGIC_STST:
                        seq, = struct.unpack_from("<I", pkt, SEQ_OFF)
                        with recv_lock:
                            received[seq] = received.get(seq, 0) + 1

        reader_running = [True]
        t_recv = threading.Thread(target=reader, daemon=True)
        t_recv.start()
        time.sleep(0.1)

        start = time.perf_counter()
        seq = 0
        while time.perf_counter() - start < duration_sec:
            ser.write(make_packet(seq, magic=MAGIC_STST))
            seq += 1
        ser.flush()
        end_send = time.perf_counter()
        # Wait for remaining echoes
        time.sleep(1.0)
        reader_running[0] = False
        time.sleep(0.2)

        with recv_lock:
            got = len(received)
        bytes_sent = seq * PKT_SIZE
        bytes_received = got * PKT_SIZE
        elapsed = end_send - start
        aggregate_mbps = (bytes_sent + bytes_received) / (1024 * 1024) / elapsed if elapsed > 0 else 0

        print()
        print("Aggregate (both directions) result:")
        print(f"  Duration:  {elapsed:.2f} s")
        print(f"  Sent:     {seq} packets, {bytes_sent} bytes (host → STM32)")
        print(f"  Echoed:   {got} packets, {bytes_received} bytes (STM32 → host)")
        print(f"  Aggregate (半双工总线总速率): {aggregate_mbps:.2f} MB/s  = (sent + received) / time")
        if seq > 0:
            print(f"  Echo loss: {100.0 * (seq - got) / seq:.1f}%")
        ser.close()
        return

    received = {}
    corrupt = [0]
    recv_lock = threading.Lock()

    def reader():
        buf = bytearray()
        while reader_running[0]:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            while len(buf) >= PKT_SIZE:
                pkt = bytes(buf[:PKT_SIZE])
                buf = buf[PKT_SIZE:]
                if pkt[:4] == MAGIC_STST:
                    seq, = struct.unpack_from("<I", pkt, SEQ_OFF)
                    if with_crc:
                        expected_crc, = struct.unpack_from("<I", pkt, CRC_OFF)
                        if crc32_bytes(pkt[4:64]) != expected_crc:
                            with recv_lock:
                                corrupt[0] += 1
                    with recv_lock:
                        received[seq] = received.get(seq, 0) + 1

    reader_running = [True]
    t_recv = threading.Thread(target=reader, daemon=True)
    t_recv.start()

    time.sleep(0.2)
    start = time.perf_counter()
    for seq in range(num_packets):
        ser.write(make_packet(seq, magic=MAGIC_STST, with_crc=with_crc))
        if (seq + 1) % 1000 == 0:
            print(f"  sent {seq + 1}/{num_packets} ...")
    ser.flush()
    end_send = time.perf_counter()

    # Wait for echoes (allow 2x round-trip time)
    wait_until = end_send + 2.0
    while time.perf_counter() < wait_until and len(received) < num_packets:
        time.sleep(0.05)
    reader_running[0] = False
    time.sleep(0.1)

    total_bytes = num_packets * PKT_SIZE
    elapsed = end_send - start
    with recv_lock:
        got = len(received)
        duplicates = sum(1 for c in received.values() if c > 1)
        missing = num_packets - got
        corrupt_count = corrupt[0]

    throughput_mbps = (total_bytes * 2 / (1024 * 1024)) / elapsed if elapsed > 0 else 0
    throughput_tx_mbps = (total_bytes / (1024 * 1024)) / elapsed if elapsed > 0 else 0

    print()
    print("Result:")
    print(f"  Sent:     {num_packets} packets, {total_bytes} bytes")
    print(f"  Received: {got} unique sequences (echoes)")
    print(f"  Missing:  {missing} packets")
    if with_crc:
        print(f"  Corrupt (CRC mismatch): {corrupt_count}")
    print(f"  Duplicates: {duplicates}")
    print(f"  Time:     {elapsed:.3f} s")
    print(f"  TX throughput: {throughput_tx_mbps:.2f} MB/s")
    print(f"  Aggregate (半双工总线总速率, sent+echo): {throughput_mbps:.2f} MB/s")
    if num_packets > 0:
        loss_pct = 100.0 * missing / num_packets
        print(f"  Loss:     {loss_pct:.2f}%")
    ser.close()


def main():
    ports = list_ports()
    if not ports:
        print("No COM ports found.")
        return
    args = [a for a in sys.argv[1:] if a.startswith("--")]
    pos = [a for a in sys.argv[1:] if not a.startswith("--")]
    port_name = pos[0] if pos else ports[0]
    num = int(pos[1]) if len(pos) > 1 else 5000
    one_way = "--one-way" in args
    with_crc = "--crc" in args
    duration_sec = None
    for a in args:
        if a.startswith("--duration="):
            try:
                duration_sec = float(a.split("=")[1])
            except (IndexError, ValueError):
                duration_sec = 5.0
            break
        if a == "--duration":
            duration_sec = 5.0
            break
    if len(ports) > 1 and port_name == ports[0] and not pos:
        print("Available:", ports)
    run_stress(port_name, num_packets=num, one_way=one_way, with_crc=with_crc, duration_sec=duration_sec)


if __name__ == "__main__":
    main()
