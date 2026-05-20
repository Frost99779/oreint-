#!/usr/bin/env python3
"""Serial validation for orientation firmware (on-demand packet architecture)."""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import List, Optional, Tuple

PACKET_LEN = 17
START_BYTE = 0xAA


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc


def parse_packet(buf: bytes) -> Optional[dict]:
    if len(buf) != PACKET_LEN or buf[0] != START_BYTE:
        return None
    start, seq, yaw, pitch, roll, flags = struct.unpack("<BBfffB", buf[:15])
    rx_crc = struct.unpack("<H", buf[15:17])[0]
    if crc16(buf[:15]) != rx_crc:
        return None
    return {
        "start": start,
        "seq": seq,
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
        "flags": flags,
        "crc_ok": True,
    }


def reset_device(ser) -> None:
    ser.reset_input_buffer()
    ser.dtr = False
    time.sleep(0.15)
    ser.dtr = True
    time.sleep(0.05)


def wait_ready(ser, timeout_s: float = 15.0) -> bool:
    deadline = time.time() + timeout_s
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if b"READY" in buf or b"ERROR:" in buf:
                return b"READY" in buf
        else:
            time.sleep(0.02)
    return False


def read_one_packet(ser, timeout_s: float = 1.0) -> Optional[dict]:
    """Read one 17-byte packet from mixed ASCII/binary stream."""
    buf = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        i = 0
        while i <= len(buf) - PACKET_LEN:
            if buf[i] != START_BYTE:
                i += 1
                continue
            pkt = parse_packet(buf[i : i + PACKET_LEN])
            if pkt:
                return pkt
            i += 1
        time.sleep(0.01)
    return None


def request_orientation(ser, timeout_s: float = 1.0) -> Optional[dict]:
    ser.write(b"CMD:GET_ORIENTATION\n")
    ser.flush()
    return read_one_packet(ser, timeout_s)


def send_cmd(ser, cmd: str, markers: Tuple[bytes, ...], timeout_s: float = 2.0) -> str:
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    buf = b""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
        else:
            time.sleep(0.02)
    for marker in markers:
        idx = buf.find(marker)
        if idx >= 0:
            end = buf.find(b"\n", idx)
            if end < 0:
                end = min(len(buf), idx + 200)
            return buf[idx:end].decode("ascii", errors="replace").strip()
    return ""


def collect_orientations(ser, count: int, gap_s: float = 0.1) -> List[dict]:
    packets: List[dict] = []
    for _ in range(count):
        pkt = request_orientation(ser, 1.0)
        if pkt:
            packets.append(pkt)
        time.sleep(gap_s)
    return packets


def run_phases(port: str, baud: int, phase56: bool, settle_s: float = 10.0) -> int:
    import serial

    ser = serial.Serial(port, baud, timeout=0.1)
    reset_device(ser)

    if not wait_ready(ser, 15.0):
        print("SENSOR_READ_FAIL: READY not received within 15s")
        ser.close()
        return 1

    # Let Madgwick converge after boot reset (device must stay still / flat)
    if settle_s > 0.0:
        print(f"SETTLE: waiting {settle_s:.0f}s for filter convergence (keep device flat)")
        time.sleep(settle_s)
        ser.reset_input_buffer()

    # Phase 1: on-demand orientation (filter + sensors alive)
    packets = collect_orientations(ser, 5, 0.15)
    if len(packets) >= 3:
        p = packets[0]
        print(
            "SENSOR_READ_OK: "
            f"yaw={p['yaw']:.2f} pitch={p['pitch']:.2f} roll={p['roll']:.2f} "
            "(via CMD:GET_ORIENTATION)"
        )
    else:
        print(f"SENSOR_READ_FAIL: only {len(packets)}/5 orientation packets")
        ser.close()
        return 1

    # Phase 2: attitude stability (first 5 samples)
    pitch = sum(abs(x["pitch"]) for x in packets[:5]) / min(5, len(packets))
    roll = sum(abs(x["roll"]) for x in packets[:5]) / min(5, len(packets))
    yaw = packets[0]["yaw"]
    if pitch < 2.0 and roll < 2.0:
        print(f"FILTER_OK: READY received, pitch={pitch:.2f}deg roll={roll:.2f}deg yaw={yaw:.2f}")
    else:
        print(f"FILTER_FAIL: pitch={pitch:.2f} roll={roll:.2f} (need <2deg each)")
        ser.close()
        return 1

    # Phase 3: 100 on-demand packets, CRC + seq
    packets_ok = 0
    last_seq: Optional[int] = None
    seq_ok = True
    for i in range(100):
        pkt = request_orientation(ser, 1.0)
        if pkt:
            if last_seq is not None and pkt["seq"] != ((last_seq + 1) & 0xFF):
                seq_ok = False
            last_seq = pkt["seq"]
            packets_ok += 1
        else:
            print(f"PACKETS_FAIL: no valid packet at request {i}")
        time.sleep(0.1)

    if packets_ok >= 95 and seq_ok:
        print(f"PACKETS_OK: {packets_ok}/100 packets valid, all CRC OK, seq increments")
    else:
        print(f"PACKETS_FAIL: valid={packets_ok}/100 seq_ok={seq_ok}")
        ser.close()
        return 1

    # Phase 4: commands
    status = send_cmd(ser, "CMD:STATUS", markers=(b"OK state=",))
    bad = send_cmd(ser, "CMD:NOT_A_COMMAND", markers=(b"ERR unknown_command", b"ERR "))
    if status.startswith("OK state=") and bad.startswith("ERR"):
        print("COMMANDS_OK: CMD:STATUS→OK, unknown→ERR")
    else:
        print(f"COMMANDS_FAIL: status={status!r} unknown={bad!r}")
        ser.close()
        return 1

    ser.close()

    if phase56:
        from validate_cal import run_step10
        return run_step10(port, baud, 60.0, False)

    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--phase", default="14", help="14 or 56")
    ap.add_argument(
        "--settle",
        type=float,
        default=10.0,
        help="Seconds to wait after READY before sampling (device flat, still)",
    )
    args = ap.parse_args()
    return run_phases(args.port, args.baud, args.phase == "56", args.settle)


if __name__ == "__main__":
    sys.exit(main())
