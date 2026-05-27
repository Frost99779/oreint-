#include "DrvBmm150.h"

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "HalI2c.h"

using namespace core::config;

static constexpr uint8_t kBmm150RegChipId   = 0x40U;
static constexpr uint8_t kBmm150RegDataXLsb = 0x42U;
static constexpr uint8_t kBmm150RegPwrCtrl  = 0x4BU;
static constexpr uint8_t kBmm150RegOpMode   = 0x4CU;
static constexpr uint8_t kBmm150ChipIdVal   = 0x32U;

bool Bmm150Dev::writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true) == 0;
}

bool Bmm150Dev::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t got = Wire.requestFrom(addr, len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool Bmm150Dev::beginAuto() {
  addr = 0;
  delay(kBmm150StartupDelayMs);

  for (size_t i = 0; i < kBmm150AddrN; i++) {
    const uint8_t a = kBmm150AddrCandidates[i];
    if (!i2cProbe(a)) continue;

    addr = a;

    // Wake from suspend (chip powers up in suspend mode).
    if (!writeReg(kBmm150RegPwrCtrl, 0x01U)) {
      addr = 0;
      continue;
    }
    delay(3);

    // Verify CHIP_ID = 0x32.
    uint8_t id = 0;
    if (!readRegs(kBmm150RegChipId, &id, 1U) || id != kBmm150ChipIdVal) {
      addr = 0;
      continue;
    }

    // Normal mode, 10 Hz ODR.
    if (!writeReg(kBmm150RegOpMode, 0x00U)) {
      addr = 0;
      continue;
    }
    delay(5);

    // Sanity: two reads must produce non-zero output.
    float x0, y0, z0, x1, y1, z1;
    if (!readmT(x0, y0, z0)) {
      addr = 0;
      continue;
    }
    delay(120);
    if (!readmT(x1, y1, z1)) {
      addr = 0;
      continue;
    }
    if (x0 == 0.0f && y0 == 0.0f && z0 == 0.0f && x1 == 0.0f && y1 == 0.0f && z1 == 0.0f) {
      addr = 0;
      continue;
    }

#ifdef DEBUG_PRINT
    Serial.printf("[bmm150] found at 0x%02X\n", (unsigned)addr);
    Serial.flush();
#endif
    return true;
  }
  return false;
}

bool Bmm150Dev::readmT(float &mx, float &my, float &mz) {
  uint8_t raw[6];
  if (!readRegs(kBmm150RegDataXLsb, raw, 6U)) return false;

  // X: 13-bit signed. raw[1] = X[12:5], raw[0][7:3] = X[4:0].
  int16_t xr = static_cast<int16_t>(
      (static_cast<uint16_t>(raw[1]) << 5U) | (static_cast<uint16_t>(raw[0]) >> 3U));
  xr = static_cast<int16_t>(xr << 3) >> 3;

  // Y: 13-bit signed. raw[3] = Y[12:5], raw[2][7:3] = Y[4:0].
  int16_t yr = static_cast<int16_t>(
      (static_cast<uint16_t>(raw[3]) << 5U) | (static_cast<uint16_t>(raw[2]) >> 3U));
  yr = static_cast<int16_t>(yr << 3) >> 3;

  // Z: 15-bit signed. raw[5] = Z[14:7], raw[4][7:1] = Z[6:0].
  int16_t zr = static_cast<int16_t>(
      (static_cast<uint16_t>(raw[5]) << 7U) | (static_cast<uint16_t>(raw[4]) >> 1U));
  zr = static_cast<int16_t>(zr << 1) >> 1;

  mx = static_cast<float>(xr) * kBmm150ScaleXY_mT;
  my = -(static_cast<float>(yr) * kBmm150ScaleXY_mT);  // BMM150 Y is opposite to MPU Y
  mz = static_cast<float>(zr) * kBmm150ScaleZ_mT;
  return true;
}
