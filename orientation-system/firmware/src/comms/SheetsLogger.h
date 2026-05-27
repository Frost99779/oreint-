#pragma once
#include <stdint.h>
#include <stddef.h>

struct SheetsSample {
    uint32_t t_ms;
    float    yaw;
    float    pitch;
    float    roll;
};

namespace sheets {

void begin();
bool enqueue(const SheetsSample& s);
bool wifiUp();

}  // namespace sheets
