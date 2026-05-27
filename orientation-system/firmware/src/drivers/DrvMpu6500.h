#ifndef PANTILT_DRV_MPU6500_H
#define PANTILT_DRV_MPU6500_H

#include <stdint.h>

struct Mpu6500Dev {
  uint8_t addr{0x68};
  bool beginAuto();
  bool readAccelGyro(float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps,
                     float &gz_dps);
};

#endif  // PANTILT_DRV_MPU6500_H
