#!/usr/bin/env python3
"""MASTER_PLAN Step 10 — phases 5–6 calibration validation."""

from __future__ import annotations

import argparse
import re
import sys
import time
from typing import List, Optional, Tuple

from validate_serial import reset_device, send_cmd, wait_ready


def drain_and_collect(ser, duration_s: float) -> Tuple[bytes, List[str]]:
    """Read serial for duration_s; return raw bytes and extracted ASCII lines."""
    lines: List[str] = []
    raw = b""
    end = time.time() + duration_s
    buf = b""
    while time.time() < end:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            raw += chunk
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", errors="replace").strip()
                if text:
                    lines.append(text)
        else:
            time.sleep(0.02)
    return raw, lines


def send_cmd_long(ser, cmd: str, markers: Tuple[bytes, ...], timeout_s: float = 2.0) -> str:
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


def parse_coverage(line: str) -> Optional[int]:
    m = re.search(r"coverage=(\d+)%", line)
    return int(m.group(1)) if m else None


def parse_status_fields(line: str) -> dict:
    out = {}
    for key in ("mag_cal", "gyro_cal", "accel_cal", "still", "filter_ticks"):
        m = re.search(rf"{key}=(\d+)", line)
        if m:
            out[key] = int(m.group(1))
    return out


def run_step10(port: str, baud: int, mag_seconds: float, skip_reboot: bool) -> int:
    import serial

    ser = serial.Serial(port, baud, timeout=0.1)
    if not wait_ready(ser, 15.0):
        # try fresh reset
        reset_device(ser)
        if not wait_ready(ser, 15.0):
            print("PHASE5_FAIL: READY not received")
            ser.close()
            return 1

    print("=== STEP 10 — Phase 5: Magnetometer calibration ===")
    cleared = send_cmd_long(ser, "CMD:CLEAR_CAL", (b"OK", b"ERR "), 2.0)
    print(f"CMD:CLEAR_CAL → {cleared}")

    start = send_cmd_long(ser, "CMD:CAL_MAG", (b"OK mag_cal=", b"ERR "))
    print(f"CMD:CAL_MAG → {start}")
    if not start.startswith("OK"):
        ser.close()
        return 1

    print(f"\n>>> ROTATE DEVICE IN FIGURE-8 FOR {int(mag_seconds)} SECONDS <<<\n")
    _, lines = drain_and_collect(ser, mag_seconds)
    last_cov = 0
    for ln in lines:
        if "CAL_PROG" in ln:
            print(ln)
            c = parse_coverage(ln)
            if c is not None:
                last_cov = c

    stop = send_cmd_long(ser, "CMD:CAL_MAG_STOP", (b"OK coverage=", b"ERR "), 3.0)
    print(f"CMD:CAL_MAG_STOP → {stop}")
    cov_stop = parse_coverage(stop) or last_cov
    p5_ok = stop.startswith("OK") and cov_stop is not None and cov_stop >= 60
    if p5_ok:
        print(f"PHASE5_OK: coverage={cov_stop}% mag cal saved to NVS (via stopAndSave)")
    else:
        print(f"PHASE5_FAIL: need OK coverage>=60%, got {stop!r}")
        ser.close()
        return 1

    status = send_cmd_long(ser, "CMD:STATUS", (b"OK state=",), 2.0)
    print(f"CMD:STATUS → {status[:100]}...")
    st = parse_status_fields(status)
    if st.get("mag_cal") != 1:
        print(f"PHASE5_WARN: mag_cal={st.get('mag_cal')} expected 1 in RAM")

    if not skip_reboot:
        print("\nReboot test (NVS persist)...")
        reset_device(ser)
        if not wait_ready(ser, 15.0):
            print("PHASE5_FAIL: READY after reboot")
            ser.close()
            return 1
        status2 = send_cmd_long(ser, "CMD:STATUS", (b"OK state=",), 2.0)
        print(f"After reboot: {status2[:100]}...")
        st2 = parse_status_fields(status2)
        if st2.get("mag_cal") != 1:
            print("PHASE5_FAIL: mag_cal not loaded from NVS after reboot")
            ser.close()
            return 1
        print("PHASE5_NVS_OK: mag_cal=1 after reboot")

    print("\n=== STEP 10 — Phase 6: IMU still + static detection ===")
    print(">>> HOLD DEVICE COMPLETELY STILL (flat) <<<\n")
    time.sleep(2.0)
    ser.write(b"CMD:CAL_IMU\n")
    ser.flush()
    _, imu_lines = drain_and_collect(ser, 6.0)
    imu_reply = ""
    for ln in imu_lines:
        print(ln)
        if "cal_imu" in ln:
            imu_reply = ln
    print(f"CAL_IMU result: {imu_reply or '(no cal_imu line)'}")
    if "OK cal_imu=done" not in imu_reply:
        print("PHASE6_FAIL: CAL_IMU failed — keep device still on flat surface")
        ser.close()
        return 1

    load = send_cmd_long(ser, "CMD:LOAD_CAL", (b"OK", b"ERR "), 2.0)
    print(f"CMD:LOAD_CAL → {load}")
    if load != "OK":
        print("PHASE6_WARN: LOAD_CAL failed after IMU cal")

    save = send_cmd_long(ser, "CMD:SAVE_CAL", (b"OK", b"ERR "), 2.0)
    print(f"CMD:SAVE_CAL → {save}")

    print("Hold still 5s for static detector...")
    time.sleep(5.0)
    status6 = send_cmd_long(ser, "CMD:STATUS", (b"OK state=",), 2.0)
    print(f"CMD:STATUS → {status6[:110]}...")
    st6 = parse_status_fields(status6)
    p6_ok = st6.get("still") == 1
    if p6_ok:
        print("PHASE6_OK: still=1 in STATUS after hold-still")
    else:
        print(f"PHASE6_FAIL: still={st6.get('still')} (need 1); gyro_cal={st6.get('gyro_cal')}")

    ser.close()
    if p5_ok and p6_ok:
        print("\n=== STEP 10 COMPLETE ===")
        return 0
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(description="Step 10 calibration validation")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mag-seconds", type=float, default=60.0)
    ap.add_argument("--skip-reboot", action="store_true")
    args = ap.parse_args()
    return run_step10(args.port, args.baud, args.mag_seconds, args.skip_reboot)


if __name__ == "__main__":
    sys.exit(main())
