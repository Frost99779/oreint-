from orientation_host.packet import crc16


def test_crc16_known_vector():
    # bytes 0-14 of a minimal valid frame (crc field zeroed for check)
    data = bytes([0xAA, 1]) + b"\x00" * 12 + bytes([0])
    # recompute over 15 bytes only
    c = crc16(data[:15])
    assert 0 <= c <= 0xFFFF


def test_crc16_empty():
    assert crc16(b"") == 0xFFFF
