#!/usr/bin/env python3
"""ESP32 orientation logger.

Polls CMD:GET_ORIENTATION at ESP_POLL_HZ and atomically writes the latest
yaw/pitch/roll to /tmp/varaha_orientation for the Prometheus exporter.
No CSV — the file stays fixed size forever.

Usage:
    python3 esp_csv_logger.py

Environment:
    ESP_PORT            /dev/ttyUSB0
    ESP_BAUD            115200
    ESP_POLL_HZ         10
    ORIENTATION_PATH    /tmp/varaha_orientation
"""

import os
import signal
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed — run: pip install pyserial")

PORT             = os.environ.get("ESP_PORT",          "/dev/ttyUSB0")
BAUD             = int(os.environ.get("ESP_BAUD",      "115200"))
POLL_HZ          = float(os.environ.get("ESP_POLL_HZ", "10"))
ORIENTATION_PATH = os.environ.get("ORIENTATION_PATH",  "/tmp/varaha_orientation")

POLL_INTERVAL_S = 1.0 / POLL_HZ

PKT_START = 0xAA
PKT_LEN   = 17


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def parse_packet(raw: bytes):
    """Return (seq, yaw, pitch, roll, flags) or None on bad start/CRC."""
    if len(raw) != PKT_LEN or raw[0] != PKT_START:
        return None
    if crc16_ccitt(raw[:15]) != struct.unpack_from("<H", raw, 15)[0]:
        return None
    seq   = raw[1]
    yaw   = struct.unpack_from("<f", raw, 2)[0]
    pitch = struct.unpack_from("<f", raw, 6)[0]
    roll  = struct.unpack_from("<f", raw, 10)[0]
    flags = raw[14]
    return seq, yaw, pitch, roll, flags


def write_shared(yaw: float, pitch: float, roll: float, ts: float) -> None:
    tmp = ORIENTATION_PATH + ".tmp"
    with open(tmp, "w") as f:
        f.write(f"{yaw:.4f},{pitch:.4f},{roll:.4f},{ts:.3f}\n")
    os.replace(tmp, ORIENTATION_PATH)


def wait_ready(ser: serial.Serial, timeout_s: float = 10.0) -> str:
    deadline = time.monotonic() + timeout_s
    buf = b""
    while time.monotonic() < deadline:
        buf += ser.read(64)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("ascii", errors="replace").strip()
            if text.startswith("READY"):
                return text
            if text.startswith("ERROR"):
                raise RuntimeError(f"ESP boot error: {text}")
            if text:
                print(f"[boot] {text}")
    raise TimeoutError("ESP did not send READY within timeout")


def read_response(ser: serial.Serial, timeout_s: float = 0.35):
    """Return 17-byte binary packet, ASCII str, or None on timeout."""
    deadline = time.monotonic() + timeout_s
    buf = b""
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == PKT_START and not buf:
            rest = b""
            inner = time.monotonic() + 0.1
            while len(rest) < PKT_LEN - 1 and time.monotonic() < inner:
                rest += ser.read(PKT_LEN - 1 - len(rest))
            return b + rest
        buf += b
        if buf.endswith(b"\n"):
            text = buf.decode("ascii", errors="replace").strip()
            buf = b""
            if text.startswith(("SETTLE", "CAL_PROG", "WARN", "CAL_AUTO")):
                print(f"[esp] {text}")
                continue
            return text
    return None


_running = True


def _stop(sig, _frame):
    global _running
    print(f"\nSignal {sig} — stopping.")
    _running = False


signal.signal(signal.SIGINT,  _stop)
signal.signal(signal.SIGTERM, _stop)


def main():
    print(f"Opening {PORT} @ {BAUD} baud …")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {PORT}: {e}")

    print("Waiting for ESP READY …")
    try:
        print(f"[esp] {wait_ready(ser)}")
    except (TimeoutError, RuntimeError) as e:
        ser.close()
        sys.exit(str(e))

    print(f"Writing → {ORIENTATION_PATH}  at {POLL_HZ:.0f} Hz")

    total = 0
    bad   = 0
    next_poll = time.monotonic()

    try:
        while _running:
            sleep_s = next_poll - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            next_poll += POLL_INTERVAL_S

            ser.write(b"CMD:GET_ORIENTATION\n")
            resp = read_response(ser)

            if isinstance(resp, bytes):
                parsed = parse_packet(resp)
                if parsed:
                    seq, yaw, pitch, roll, flags = parsed
                    total += 1
                    write_shared(yaw, pitch, roll, time.time())
                    if total % (int(POLL_HZ) * 10) == 0:
                        print(f"#{total}  yaw={yaw:6.1f}°  pitch={pitch:+5.1f}°  roll={roll:+6.1f}°")
                else:
                    bad += 1
                    print(f"[warn] bad packet (CRC/framing): {resp.hex()}")

            elif isinstance(resp, str) and resp and resp != "ERR not_converged":
                print(f"[esp] {resp}")

    finally:
        ser.close()
        print(f"Done. packets={total}  bad_crc={bad}")


if __name__ == "__main__":
    main()
