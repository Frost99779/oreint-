#!/usr/bin/env python3
"""Lightweight Prometheus metrics exporter for NVIDIA Jetson.

Single-threaded HTTP server on port 9100 serving GET /metrics.
No external dependencies — stdlib only (sysfs/procfs + POSIX sockets).
"""

import os
import signal
import sys
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

SENSOR_ID = os.environ.get("SENSOR_ID", "sensor-1")
NET_IFACE = os.environ.get("NET_IFACE", "eth0")
LISTEN_PORT = int(os.environ.get("METRICS_PORT", "9100"))
PACKET_FREQ_PATH = "/tmp/varaha_packet_freq"
ORIENTATION_PATH = os.environ.get("ORIENTATION_PATH", "/tmp/varaha_orientation")
# esp_csv_logger writes every 1/ESP_POLL_HZ seconds; allow a few missed polls.
ORIENTATION_STALE_SECONDS = float(os.environ.get("ORIENTATION_STALE_SECONDS", "5"))

# Host-side timer writes `chronyc -c tracking` here; we parse it (same
# file-handoff pattern as PACKET_FREQ_PATH). chronyc isn't in this container's
# slim image, so the host produces the data and we just read it.
CHRONY_CSV_PATH = os.environ.get("CHRONY_CSV_PATH", "/tmp/varaha_chrony.csv")
# If the writer stops updating the file, the data is no longer trustworthy.
# The writer runs every ~16s; allow a few missed ticks before flagging unsynced.
CHRONY_STALE_SECONDS = float(os.environ.get("CHRONY_STALE_SECONDS", "60"))

# When running inside a container, /proc and /sys are the container's own.
# Mount the host's procfs/sysfs and set these env vars so we read host metrics.
HOST_PROC = os.environ.get("HOST_PROC", "/proc")
HOST_SYS = os.environ.get("HOST_SYS", "/sys")

# CPU stat cache for delta computation
_prev_cpu = {"idle": 0, "total": 0, "usage": 0.0}


def read_temperature() -> float:
    """Read SoC temperature from thermal_zone0 (millidegrees -> degrees)."""
    try:
        with open(f"{HOST_SYS}/devices/virtual/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return 0.0


def read_cpu_usage() -> float:
    """Compute CPU usage % from /proc/stat delta since last call."""
    try:
        with open(f"{HOST_PROC}/stat") as f:
            parts = f.readline().split()
        # user nice system idle iowait irq softirq steal
        values = [int(v) for v in parts[1:]]
        idle = values[3] + values[4]  # idle + iowait
        total = sum(values)

        d_idle = idle - _prev_cpu["idle"]
        d_total = total - _prev_cpu["total"]

        if d_total > 0:
            _prev_cpu["usage"] = (1.0 - d_idle / d_total) * 100.0

        _prev_cpu["idle"] = idle
        _prev_cpu["total"] = total
        return _prev_cpu["usage"]
    except (OSError, ValueError):
        return 0.0


def read_memory_usage() -> float:
    """Compute memory usage % from /proc/meminfo."""
    try:
        info = {}
        with open(f"{HOST_PROC}/meminfo") as f:
            for line in f:
                parts = line.split()
                key = parts[0].rstrip(":")
                if key in ("MemTotal", "MemAvailable"):
                    info[key] = int(parts[1])
                if len(info) == 2:
                    break
        total = info["MemTotal"]
        avail = info["MemAvailable"]
        return (total - avail) / total * 100.0 if total > 0 else 0.0
    except (OSError, ValueError, KeyError):
        return 0.0


def read_uptime() -> float:
    """Read system uptime in seconds from /proc/uptime."""
    try:
        with open(f"{HOST_PROC}/uptime") as f:
            return float(f.read().split()[0])
    except (OSError, ValueError):
        return 0.0


def read_packet_loss() -> float:
    """Compute packet loss % from /proc/net/dev for NET_IFACE."""
    try:
        with open(f"{HOST_PROC}/net/dev") as f:
            for line in f:
                line = line.strip()
                if line.startswith(NET_IFACE + ":"):
                    fields = line.split(":")[1].split()
                    # fields[1] = rx_packets, fields[3] = rx_drop
                    rx_packets = int(fields[1])
                    rx_drop = int(fields[3])
                    denom = rx_packets + rx_drop
                    return (rx_drop / denom * 100.0) if denom > 0 else 0.0
    except (OSError, ValueError, IndexError):
        pass
    return 0.0


def read_packet_freq() -> float:
    """Read packet frequency from shared file written by audio pipeline."""
    try:
        with open(PACKET_FREQ_PATH) as f:
            return float(f.read().strip())
    except (OSError, ValueError):
        return 0.0


def read_orientation():
    """Parse the shared file written by esp_csv_logger.py.

    Format: yaw,pitch,roll,timestamp_unix
    Returns a dict, or None if the file is missing/unreadable/stale.
    Yaw/pitch/roll are only trusted when the sample is fresh — returning
    None when stale prevents the dashboard from showing a frozen heading.
    """
    try:
        age = max(0.0, time.time() - os.stat(ORIENTATION_PATH).st_mtime)
        with open(ORIENTATION_PATH) as f:
            parts = f.read().strip().split(",")
        if len(parts) < 4:
            return None
        return {
            "yaw":   float(parts[0]),
            "pitch": float(parts[1]),
            "roll":  float(parts[2]),
            "age":   age,
            "fresh": age <= ORIENTATION_STALE_SECONDS,
        }
    except (OSError, ValueError, IndexError):
        return None


def read_chrony():
    """Parse the host-written `chronyc -c tracking` CSV.

    Returns a dict of parsed fields, or None if the file is
    missing/unreadable/malformed. chrony's CSV tracking field order:
      0 refid(hex)  1 reference     2 stratum        3 ref-time
      4 current-offset(s,+ve=fast)  5 last-offset(s) 6 rms-offset(s)
      7 freq  8 resid-freq  9 skew  10 root-delay  11 root-disp
      12 update-interval  13 leap-status
    """
    try:
        age = max(0.0, time.time() - os.stat(CHRONY_CSV_PATH).st_mtime)
        with open(CHRONY_CSV_PATH) as f:
            row = f.read().strip().split(",")
        if len(row) < 14:
            return {"age": age}  # present but malformed -> treated as unsynced
        leap = row[13].strip()
        stratum = int(row[2])
        fresh = age <= CHRONY_STALE_SECONDS
        return {
            "offset": float(row[4]),   # current correction, seconds (+ = fast)
            "rms": float(row[6]),      # RMS offset, seconds (jitter/quality)
            "stratum": stratum,
            "leap": leap,
            "age": age,
            "synced": 1 if (leap == "Normal" and stratum > 0 and fresh) else 0,
        }
    except (OSError, ValueError, IndexError):
        return None


def chrony_metric_lines(sid: str) -> list:
    """Exposition lines for chrony clock-sync metrics.

    `chrony_synced` and `chrony_sample_age_seconds` are always emitted so the
    dashboard can tell "synced", "drifting", and "no data" apart. The numeric
    offset/rms/stratum are emitted ONLY when a fresh, synced sample exists —
    emitting 0 when unsynced would read as a perfect clock, which is a lie.
    """
    data = read_chrony()
    lines = []

    def emit(name, help_text, value, fmt):
        lines.append(f"# HELP {name} {help_text}")
        lines.append(f"# TYPE {name} gauge")
        lines.append(f'{name}{{sensor_id="{sid}"}} {fmt.format(value)}')

    synced = data.get("synced", 0) if data else 0
    emit("varaha_sensor_chrony_synced",
         "1 if the sensor clock is locked to its source and the sample is fresh, else 0",
         synced, "{:d}")

    if data is not None and "age" in data:
        emit("varaha_sensor_chrony_sample_age_seconds",
             "Seconds since the host wrote the chrony tracking sample (staleness)",
             data["age"], "{:.1f}")

    if synced:
        emit("varaha_sensor_chrony_offset_seconds",
             "Current system-clock offset from the time source (+ = clock fast)",
             data["offset"], "{:.9f}")
        emit("varaha_sensor_chrony_rms_offset_seconds",
             "RMS clock offset over recent samples (sync jitter / quality)",
             data["rms"], "{:.9f}")
        emit("varaha_sensor_chrony_stratum",
             "chrony stratum (hops from the reference clock)",
             data["stratum"], "{:d}")
    return lines


def orientation_metric_lines(sid: str) -> list:
    """Exposition lines for ESP32 orientation metrics.

    Sample age is always emitted so the dashboard can distinguish
    'fresh data', 'logger stopped', and 'ESP not converged'.
    Yaw/pitch/roll are only emitted when the sample is fresh —
    emitting 0 when stale would look like a flat, north-facing sensor.
    """
    data  = read_orientation()
    lines = []

    def emit(name, help_text, value, fmt):
        lines.append(f"# HELP {name} {help_text}")
        lines.append(f"# TYPE {name} gauge")
        lines.append(f'{name}{{sensor_id="{sid}"}} {fmt.format(value)}')

    emit("varaha_sensor_orientation_sample_age_seconds",
         "Seconds since esp_csv_logger wrote the last orientation sample",
         data["age"] if data else ORIENTATION_STALE_SECONDS + 1,
         "{:.1f}")

    if data is not None and data["fresh"]:
        emit("varaha_sensor_orientation_yaw_deg",
             "Filtered yaw 0-360 deg clockwise from true north",
             data["yaw"], "{:.2f}")
        emit("varaha_sensor_orientation_pitch_deg",
             "Filtered pitch -90 to +90 deg (nose-up positive)",
             data["pitch"], "{:.2f}")
        emit("varaha_sensor_orientation_roll_deg",
             "Filtered roll -180 to +180 deg (right-side-down positive)",
             data["roll"], "{:.2f}")

    return lines


def build_metrics() -> str:
    """Build full Prometheus exposition text."""
    sid = SENSOR_ID
    metrics = [
        ("varaha_sensor_temperature_celsius", "Sensor temperature in celsius",
         read_temperature()),
        ("varaha_sensor_cpu_usage_percent", "Sensor CPU usage percent",
         read_cpu_usage()),
        ("varaha_sensor_memory_usage_percent", "Sensor memory usage percent",
         read_memory_usage()),
        ("varaha_sensor_uptime_seconds", "Sensor uptime in seconds",
         read_uptime()),
        ("varaha_sensor_packet_loss_percent", "Sensor packet loss percent",
         read_packet_loss()),
        ("varaha_sensor_packet_freq_hz", "Sensor packet frequency in Hz",
         read_packet_freq()),
        ("varaha_sensor_up", "Sensor is up (always 1)", 1),
    ]

    lines = []
    for name, help_text, value in metrics:
        lines.append(f"# HELP {name} {help_text}")
        lines.append(f"# TYPE {name} gauge")
        if isinstance(value, float):
            lines.append(f'{name}{{sensor_id="{sid}"}} {value:.1f}')
        else:
            lines.append(f'{name}{{sensor_id="{sid}"}} {value}')

    # Clock-sync metrics (read from the host-written chronyc CSV).
    lines.extend(chrony_metric_lines(sid))

    # ESP32 orientation (read from esp_csv_logger shared file).
    lines.extend(orientation_metric_lines(sid))

    return "\n".join(lines) + "\n"


class MetricsHandler(BaseHTTPRequestHandler):
    """Serves /metrics in Prometheus exposition format."""

    def do_GET(self):
        if self.path == "/metrics":
            body = build_metrics().encode()
            self.send_response(200)
            self.send_header("Content-Type",
                             "text/plain; version=0.0.4; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, fmt, *args):
        # Suppress per-request logs; too noisy for Prometheus scrapes
        pass


def main():
    # Seed CPU stats so the first /metrics call has a baseline
    read_cpu_usage()

    server = HTTPServer(("0.0.0.0", LISTEN_PORT), MetricsHandler)

    def shutdown(sig, frame):
        print(f"\nReceived signal {sig}, shutting down...")
        # _BaseServer.shutdown() must be called from a different thread
        # than serve_forever(), otherwise it deadlocks.
        import threading
        threading.Thread(target=server.shutdown).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"metrics_exporter listening on :{LISTEN_PORT}  "
          f"sensor_id={SENSOR_ID}  iface={NET_IFACE}")
    server.serve_forever()
    print("Exited cleanly.")


if __name__ == "__main__":
    main()
