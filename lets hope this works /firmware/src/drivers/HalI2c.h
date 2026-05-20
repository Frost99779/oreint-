#ifndef PANTILT_HAL_I2C_H
#define PANTILT_HAL_I2C_H

#include <stdint.h>

void halI2cBegin();
void halI2cSetClock(uint32_t hz);
bool i2cProbe(uint8_t addr);
uint8_t i2cReadReg8(uint8_t addr, uint8_t reg);
bool i2cWriteReg8(uint8_t addr, uint8_t reg, uint8_t val);
bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t len);
void i2cScan(const char *tag);

#endif  // PANTILT_HAL_I2C_H
