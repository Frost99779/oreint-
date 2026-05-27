#ifndef PANTILT_DRV_BMM150_H
#define PANTILT_DRV_BMM150_H

#include <stdint.h>

struct Bmm150Dev {
  uint8_t addr{0};

  bool beginAuto();
  bool readmT(float &mx, float &my, float &mz);

 private:
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};

#endif  // PANTILT_DRV_BMM150_H
