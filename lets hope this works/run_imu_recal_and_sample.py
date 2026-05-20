#!/usr/bin/env python3
"""Load mag cal from NVS, recal IMU, sample orientation 1 Hz for 5 minutes."""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from validate_serial import parse_packet, reset_device, wait_ready

PACKET_LEN = 17
START_BYTE = 0xAA


def transact(ser, cmd: str, wait_s: float = 3.0, stop_markers: tuple[bytes, ...] = ()) -> str:
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    buf = b""
    deadline = time.time() + wait_s
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            for m in stop_markers:
                if m in buf:
                    return buf.decode("ascii", errors="replace")
        else:
            time.sleep(0.05)
    return buf.decode("ascii", errors="replace")


def read_orientation(ser, timeout_s: float = 1.0) -> dict | None:
    ser.reset_input_buffer()
    ser.write(b"CMD:GET_ORIENTATION\n")
    ser.flush()
    raw = b""
    deadline = time.time() + timeout_s
    while len(raw) < PACKET_LEN and time.time() < deadline:
        if ser.in_waiting:
            raw += ser.read(ser.in_waiting)
        else:
            time.sleep(0.005)
    for i in range(len(raw) - PACKET_LEN + 1):
        if raw[i] == START_BYTE:
            pkt = parse_packet(raw[i : i + PACKET_LEN])
            if pkt:
                return pkt
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sample-minutes", type=float, default=5.0)
    ap.add_argument("--skip-validate", action="store_true")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    import serial

    root = Path(__file__).resolve().parent
    out_path = (
        Path(args.out)
        if args.out
        else root / f"samples_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    reset_device(ser)
    print(f"[{datetime.now():%H:%M:%S}] Waiting for READY...")
    if not wait_ready(ser, 20.0):
        print("FAIL: READY not received")
        ser.close()
        return 1

    print(f"[{datetime.now():%H:%M:%S}] CMD:LOAD_CAL (mag cal from NVS)")
    load = transact(ser, "CMD:LOAD_CAL", 2.0, (b"OK", b"ERR "))
    print(" ", load.strip().split("\n")[-1] if load else "(no response)")
    if "OK" not in load and "ERR" in load:
        ser.close()
        return 1

    print(f"[{datetime.now():%H:%M:%S}] CMD:STATUS (before IMU recal)")
    status = transact(ser, "CMD:STATUS", 2.0, (b"OK state=",))
    print(" ", status.strip()[-120:])

    print(f"[{datetime.now():%H:%M:%S}] CMD:CAL_IMU — keep device FLAT and STILL (~40s)")
    imu_out = transact(
        ser,
        "CMD:CAL_IMU",
        50.0,
        (b"cal_complete", b"imu_cal_failed", b"ERR "),
    )
    for line in imu_out.splitlines():
        s = line.strip()
        if s and not s.startswith("ets"):
            print(" ", s)
    if "cal_complete" not in imu_out:
        print("WARN: full cal_complete not seen; continuing with sampling")

    print(f"[{datetime.now():%H:%M:%S}] CMD:STATUS (after IMU recal)")
    status2 = transact(ser, "CMD:STATUS", 2.0, (b"OK state=",))
    print(" ", status2.strip()[-160:])

    ser.close()

    if not args.skip_validate:
        print(f"\n[{datetime.now():%H:%M:%S}] Running validate_serial.py...")
        r = subprocess.run(
            [
                sys.executable,
                str(root / "validate_serial.py"),
                "--port",
                args.port,
                "--baud",
                str(args.baud),
                "--settle",
                "10",
            ],
            cwd=str(root),
        )
        if r.returncode != 0:
            print("validate_serial: FAIL (see above)")

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    n_samples = int(args.sample_minutes * 60.0)
    print(
        f"\n[{datetime.now():%H:%M:%S}] Sampling 1 Hz for "
        f"{args.sample_minutes:.0f} min ({n_samples} samples) → {out_path}"
    )
    print("Keep device still if you want stable pitch/roll.\n")

    t0 = time.time()
    ok_count = 0
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "wall_time",
                "elapsed_s",
                "yaw_deg",
                "pitch_deg",
                "roll_deg",
                "flags",
                "seq",
            ]
        )
        for i in range(n_samples):
            target = t0 + float(i)
            now = time.time()
            if now < target:
                time.sleep(target - now)
            pkt = read_orientation(ser, 1.0)
            elapsed = time.time() - t0
            wall = datetime.now().isoformat(timespec="seconds")
            if pkt:
                ok_count += 1
                w.writerow(
                    [
                        wall,
                        f"{elapsed:.3f}",
                        f"{pkt['yaw']:.4f}",
                        f"{pkt['pitch']:.4f}",
                        f"{pkt['roll']:.4f}",
                        pkt["flags"],
                        pkt["seq"],
                    ]
                )
                if i % 30 == 0 or i == n_samples - 1:
                    print(
                        f"  [{wall}] #{i+1}/{n_samples} "
                        f"yaw={pkt['yaw']:.2f} pitch={pkt['pitch']:.2f} "
                        f"roll={pkt['roll']:.2f} flags=0x{pkt['flags']:02x}"
                    )
            else:
                w.writerow([wall, f"{elapsed:.3f}", "", "", "", "", ""])
                if i % 30 == 0:
                    print(f"  [{wall}] #{i+1}/{n_samples} (no valid packet)")

    ser.close()
    print(
        f"\n[{datetime.now():%H:%M:%S}] Done: {ok_count}/{n_samples} valid packets "
        f"→ {out_path}"
    )
    return 0 if ok_count >= n_samples * 0.95 else 1


if __name__ == "__main__":
    sys.exit(main())
