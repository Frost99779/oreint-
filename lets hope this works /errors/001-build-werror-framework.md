# 001 — -Werror vs Arduino/FreeRTOS headers

## symptom
pio run failed: WString.h conversion, Esp.cpp shadow byte, sign-conversion in freertos portmacro.h

## cause
`-Werror` + `-Wextra` + C++17 `std::byte` + Arduino core not clean under strict warnings

## fix
templates/platformio.ini added (our src still uses -Werror):
- `-Wno-error=conversion`
- `-Wno-error=shadow`
- `-Wno-error=float-conversion`
- `-Wno-error=sign-conversion`

## also fixed in src
- CalImu: Mpu6500Dev, float math, esp_timer not Arduino.h
- CalMag: drop Arduino.h
- NvsCal: esp_timer for millis
- PacketWriter.h: `#include <cstddef>`
- Madgwick: kRadToDeg float, no Arduino.h
