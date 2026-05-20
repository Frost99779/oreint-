import struct

from orientation_host.packet import PACKET_LEN, START_BYTE, crc16, parse_packet


def _build(yaw=90.0, pitch=0.0, roll=0.0, seq=0, flags=0):
    body = struct.pack("<BBfffB", START_BYTE, seq, yaw, pitch, roll, flags)
    c = crc16(body)
    return body + struct.pack("<H", c)


def test_parse_good():
    raw = _build()
    assert len(raw) == PACKET_LEN
    p = parse_packet(raw)
    assert p is not None
    assert p["yaw"] == 90.0
    assert p["flags"] == 0


def test_parse_bad_crc():
    raw = _build()
    bad = bytearray(raw)
    bad[-1] ^= 0xFF
    assert parse_packet(bytes(bad)) is None


def test_parse_bad_start():
    raw = _build()
    bad = bytearray(raw)
    bad[0] = 0x55
    assert parse_packet(bytes(bad)) is None
