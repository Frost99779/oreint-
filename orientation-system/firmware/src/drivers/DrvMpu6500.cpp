#include "DrvMpu6500.h"

#include <Arduino.h>

#include "Config.h"
#include "HalI2c.h"

using namespace core::config;

// MPU-6500 register map (subset) — same ordering as legacy main.ino, PDF-corrected ranges/DLPF/scale.
static constexpr uint8_t REG_WHO_AM_I = 0x75;
static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_SMPLRT_DIV = 0x19;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

bool Mpu6500Dev::beginAuto() {
  const uint8_t who68 = i2cReadReg8(kMpu6500AddrDefault, REG_WHO_AM_I);
  const uint8_t who69 = i2cReadReg8(0x69U, REG_WHO_AM_I);

  if (who68 == kMpu6500WhoAmI0 || who68 == kMpu6500WhoAmI1)
    addr = kMpu6500AddrDefault;
  else if (who69 == kMpu6500WhoAmI0 || who69 == kMpu6500WhoAmI1)
    addr = 0x69U;
  else
    return false;

  // Wake + PLL.
  if (!i2cWriteReg8(addr, REG_PWR_MGMT_1, 0x01)) return false;
  delay(100);

  // 100 Hz effective sample rate, DLPF from config, gyro ±250 dps, accel ±2g
  if (!i2cWriteReg8(addr, REG_SMPLRT_DIV, 9)) return false;
  if (!i2cWriteReg8(addr, REG_CONFIG, kDlpfCfgReg)) return false;
  if (!i2cWriteReg8(addr, REG_GYRO_CONFIG, kGyroRangeReg)) return false;
  if (!i2cWriteReg8(addr, REG_ACCEL_CONFIG, 0x00)) return false;

  return true;
}

bool Mpu6500Dev::readAccelGyro(float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps,
                               float &gz_dps) {
  uint8_t b[14];
  if (!i2cReadRegs(addr, REG_ACCEL_XOUT_H, b, 14)) return false;

  auto s16 = [&](int i) -> int16_t {
    return static_cast<int16_t>(static_cast<uint16_t>(b[i]) << 8 | b[i + 1]);
  };

  const int16_t rax = s16(0), ray = s16(2), raz = s16(4);
  const int16_t rgx = s16(8), rgy = s16(10), rgz = s16(12);

  ax_g = static_cast<float>(rax) / kAccelScale2G;
  ay_g = static_cast<float>(ray) / kAccelScale2G;
  az_g = static_cast<float>(raz) / kAccelScale2G;

  gx_dps = static_cast<float>(rgx) / kGyroScale;
  gy_dps = static_cast<float>(rgy) / kGyroScale;
  gz_dps = static_cast<float>(rgz) / kGyroScale;

  return true;
}
