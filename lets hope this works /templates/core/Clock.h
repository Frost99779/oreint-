#pragma once

#include <Arduino.h>
#include <cstdint>

namespace core {

class IClock {
 public:
  virtual ~IClock() = default;
  virtual uint32_t millis() const = 0;
};

class ArduinoClock final : public IClock {
 public:
  uint32_t millis() const override { return ::millis(); }
};

inline bool elapsed(const uint32_t now, const uint32_t previous, const uint32_t period) {
  return static_cast<uint32_t>(now - previous) >= period;
}

}  // namespace core
