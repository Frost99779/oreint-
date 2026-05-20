"""Serial: wait READY, poll orientation via CMD:GET_ORIENTATION."""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, Optional

import serial

from .packet import PACKET_LEN, START_BYTE, parse_packet


class SerialHandler:
    def __init__(self, port: str, baud: int = 115200, poll_rate_hz: float = 10.0) -> None:
        self._ser = serial.Serial(port, baud, timeout=0.05)
        self._poll_interval = 1.0 / poll_rate_hz if poll_rate_hz > 0.0 else 0.1
        self._rx_buf = bytearray()
        self._ascii_lines: Deque[str] = deque(maxlen=64)
        self._packets: Deque[dict] = deque(maxlen=128)
        self.latest_data: Optional[dict] = None
        self._ready = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, daemon=True)

    def open_and_wait_ready(self, timeout_s: float = 10.0) -> bool:
        deadline = time.time() + timeout_s
        buf = b""
        while time.time() < deadline:
            chunk = self._ser.read(self._ser.in_waiting or 1)
            if chunk:
                buf += chunk
                if b"READY" in buf:
                    self._ready = True
                    self._thread.start()
                    return True
                if b"ERROR:" in buf:
                    return False
            time.sleep(0.02)
        return False

    def request_orientation(self) -> Optional[bytes]:
        if not self._ser.is_open:
            return None
        self._ser.write(b"CMD:GET_ORIENTATION\n")
        self._ser.flush()
        response = b""
        deadline = time.time() + 1.0
        while len(response) < PACKET_LEN and time.time() < deadline:
            n = self._ser.in_waiting
            if n:
                response += self._ser.read(n)
            else:
                time.sleep(0.005)
        if len(response) >= PACKET_LEN:
            for i in range(len(response) - PACKET_LEN + 1):
                if response[i] == START_BYTE:
                    return response[i : i + PACKET_LEN]
        return None

    def _parse_packet(self, raw: bytes) -> None:
        pkt = parse_packet(raw)
        if pkt:
            self.latest_data = pkt
            self._packets.append(pkt)

    def _drain_ascii(self) -> None:
        while True:
            nl = self._rx_buf.find(b"\n")
            if nl < 0:
                break
            line = bytes(self._rx_buf[:nl]).decode("ascii", errors="replace").strip()
            del self._rx_buf[: nl + 1]
            if line:
                self._ascii_lines.append(line)

    def _reader(self) -> None:
        last_request = 0.0
        while not self._stop.is_set():
            now = time.time()
            if now - last_request >= self._poll_interval:
                raw = self.request_orientation()
                if raw:
                    self._parse_packet(raw)
                last_request = now
            else:
                if self._ser.in_waiting:
                    self._rx_buf.extend(self._ser.read(self._ser.in_waiting))
                    self._drain_ascii()
                time.sleep(0.01)

    def send_command(self, cmd: str) -> None:
        self._ser.write((cmd + "\n").encode("ascii"))
        self._ser.flush()

    def pop_packet(self) -> Optional[dict]:
        try:
            return self._packets.popleft()
        except IndexError:
            return None

    def pop_line(self) -> Optional[str]:
        try:
            return self._ascii_lines.popleft()
        except IndexError:
            return None

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._ser.close()
