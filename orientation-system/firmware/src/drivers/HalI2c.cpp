// -----------------------------------------------------------------------------
// HARDWARE REQUIREMENT: External 4.7 kΩ pull-up resistors must be fitted on
// SDA (GPIO 21) and SCL (GPIO 22) to 3.3 V. Internal ESP32 pull-ups (~45 kΩ)
// are insufficient at 100 kHz and will cause silent I2C read errors.
// -----------------------------------------------------------------------------

#include "HalI2c.h"

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

using namespace core::config;

void halI2cBegin() {
  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz);
  Wire.setTimeOut(kI2cTimeoutMs);
}

void halI2cSetClock(uint32_t hz) { Wire.setClock(hz); }

bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

uint8_t i2cReadReg8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(addr, static_cast<uint8_t>(1)) != 1) return 0xFF;
  return static_cast<uint8_t>(Wire.read());
}

bool i2cWriteReg8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t got = Wire.requestFrom(addr, len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) dst[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

void i2cScan(const char *tag) {
  (void)tag;
  // Step 2: no Serial here — default TX buffer is small; logging only the one-line
  // summary in main.ino after both scans avoids truncated/garbled output.
  for (uint8_t a = 1; a < 0x7F; a++) {
    (void)i2cProbe(a);
    // PDF step 2: was delay(2) per address (~500ms/scan); 200us ≈ 25ms/scan.
    delayMicroseconds(200);
  }
}
