"""17-byte orientation packet parse + CRC16."""

from __future__ import annotations

import struct
from typing import Optional

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


def parse_packet(data: bytes) -> Optional[dict]:
    if len(data) != PACKET_LEN:
        return None
    if data[0] != START_BYTE:
        return None
    start, seq, yaw, pitch, roll, flags = struct.unpack("<BBfffB", data[:15])
    rx_crc = struct.unpack("<H", data[15:17])[0]
    if crc16(data[:15]) != rx_crc:
        return None
    return {
        "start": start,
        "seq": seq,
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
        "flags": flags,
        "crc": rx_crc,
    }
