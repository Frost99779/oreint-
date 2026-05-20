#include "PacketWriter.h"

#include <Arduino.h>
#include <string.h>

namespace comms {

uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < len; i++) {
        crc ^= static_cast<uint16_t>(static_cast<uint16_t>(data[i]) << 8U);
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                                  : static_cast<uint16_t>(crc << 1U);
        }
    }
    return crc;
}

size_t sendPacket(uint8_t seq, float yaw, float pitch, float roll, uint8_t flags) {
    uint8_t buf[17];

    // Start byte + sequence
    buf[0] = 0xAAU;
    buf[1] = seq;

    // Yaw (little-endian float32)
    uint32_t tmp32;
    memcpy(&tmp32, &yaw, 4U);
    buf[2] = static_cast<uint8_t>(tmp32 & 0xFFU);
    buf[3] = static_cast<uint8_t>((tmp32 >> 8U) & 0xFFU);
    buf[4] = static_cast<uint8_t>((tmp32 >> 16U) & 0xFFU);
    buf[5] = static_cast<uint8_t>((tmp32 >> 24U) & 0xFFU);

    // Pitch
    memcpy(&tmp32, &pitch, 4U);
    buf[6] = static_cast<uint8_t>(tmp32 & 0xFFU);
    buf[7] = static_cast<uint8_t>((tmp32 >> 8U) & 0xFFU);
    buf[8] = static_cast<uint8_t>((tmp32 >> 16U) & 0xFFU);
    buf[9] = static_cast<uint8_t>((tmp32 >> 24U) & 0xFFU);

    // Roll
    memcpy(&tmp32, &roll, 4U);
    buf[10] = static_cast<uint8_t>(tmp32 & 0xFFU);
    buf[11] = static_cast<uint8_t>((tmp32 >> 8U) & 0xFFU);
    buf[12] = static_cast<uint8_t>((tmp32 >> 16U) & 0xFFU);
    buf[13] = static_cast<uint8_t>((tmp32 >> 24U) & 0xFFU);

    // Flags
    buf[14] = flags;

    // CRC16 over bytes 0-14
    const uint16_t crc = crc16(buf, 15U);
    buf[15] = static_cast<uint8_t>(crc & 0xFFU);         // little-endian low byte
    buf[16] = static_cast<uint8_t>((crc >> 8U) & 0xFFU); // little-endian high byte

    return Serial.write(buf, 17U);
}

}  // namespace comms
