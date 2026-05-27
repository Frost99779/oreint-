#pragma once
#include <cstddef>
#include <stdint.h>

// Binary packet format (17 bytes, little-endian):
//   Byte 0:      START  = 0xAA
//   Byte 1:      SEQ    uint8  (wraps 0-255)
//   Bytes 2-5:   YAW    float32 (0.0–360.0 deg)
//   Bytes 6-9:   PITCH  float32 (-90.0–+90.0 deg)
//   Bytes 10-13: ROLL   float32 (-180.0–+180.0 deg)
//   Byte 14:     FLAGS  uint8
//   Bytes 15-16: CRC16  uint16 (CRC-16/CCITT-FALSE over bytes 0-14, little-endian)
//
// FLAGS bits:
//   0: MAG_CAL_VALID
//   1: IMU_CAL_VALID
//   2: FILTER_CONVERGED
//   3: STATIC
//   4-7: reserved = 0

namespace comms {

static constexpr uint8_t FLAG_MAG_CAL_VALID    = 0x01U;
static constexpr uint8_t FLAG_IMU_CAL_VALID    = 0x02U;
static constexpr uint8_t FLAG_FILTER_CONVERGED = 0x04U;
static constexpr uint8_t FLAG_STATIC           = 0x08U;

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no reflection.
// Input: bytes 0-14 of the packet (NOT including CRC bytes).
uint16_t crc16(const uint8_t* data, size_t len);

// Build and write a 17-byte packet directly to Serial.
// Returns the number of bytes written (always 17 on success, 0 on error).
size_t sendPacket(uint8_t seq, float yaw, float pitch, float roll, uint8_t flags);

}  // namespace comms
