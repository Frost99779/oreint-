#!/usr/bin/env python3
"""Live yaw/pitch/roll via on-demand CMD:GET_ORIENTATION polling."""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime

from .serial_handler import SerialHandler


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument(
        "--poll-rate",
        type=float,
        default=10.0,
        help="Orientation requests per second (default 10)",
    )
    args = ap.parse_args()

    h = SerialHandler(args.port, args.baud, poll_rate_hz=args.poll_rate)
    if not h.open_and_wait_ready():
        print("READY timeout", file=sys.stderr)
        return 1

    print(f"Connected @ {args.poll_rate:.1f} Hz. Ctrl+C to exit.")
    try:
        while True:
            pkt = h.latest_data
            if pkt:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                fc = "filter_converged" if (pkt["flags"] & 0x04) else ""
                print(
                    f"[{ts}] yaw={pkt['yaw']:7.2f} pitch={pkt['pitch']:7.2f} "
                    f"roll={pkt['roll']:7.2f} flags=0x{pkt['flags']:02x} {fc}"
                )
            line = h.pop_line()
            if line:
                print(f"[ascii] {line}")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        h.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
