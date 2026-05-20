#include "DrvTlv493d.h"

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "HalI2c.h"

using namespace core::config;

static void tlvGeneralCallReset() {
  // Datasheet: general call reset after power-up (no reset pin).
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  (void)Wire.endTransmission(true);
  delay(10);
}

static bool tlvSanityCheck(tlv493d_embed::Tlv493dLite &dev) {
  // Two reads ~20ms apart:
  // - data should not be all zeros
  // - frame counter should change (or at least not stay stuck)
  if (!dev.updateData()) return false;
  const uint8_t fc0 = dev.getFrameCounter();
  const float x0 = dev.getXmT(), y0 = dev.getYmT(), z0 = dev.getZmT();

  delay(20);
  if (!dev.updateData()) return false;
  const uint8_t fc1 = dev.getFrameCounter();
  const float x1 = dev.getXmT(), y1 = dev.getYmT(), z1 = dev.getZmT();

  const bool all0 = (x0 == 0.0f && y0 == 0.0f && z0 == 0.0f && x1 == 0.0f && y1 == 0.0f && z1 == 0.0f);
  if (all0) return false;
  if (fc0 == fc1) return false;
  // Reject obvious "stuck same sample" case as well.
  if (x0 == x1 && y0 == y1 && z0 == z1) return false;
  return true;
}

bool Tlv493dDev::beginAuto() {
  addr = 0;
  m_lastFrameCounter = 0xFF;

  delay(kTlv493dStartupDelayMs);

  // Attempt deterministic bring-up:
  // - general-call reset
  // - scan both possible latched addresses (0x1F / 0x5E) + known candidates
  // - sanity check (frame counter progresses, not all-zero)
  constexpr uint8_t kAttempts = 4;
  for (uint8_t attempt = 0; attempt < kAttempts; attempt++) {
    tlvGeneralCallReset();

    for (size_t i = 0; i < kTlv493dAddrN; i++) {
      const uint8_t a = kTlv493dAddrCandidates[i];
      if (!i2cProbe(a)) continue;

      // No general-call reset inside the library; we do it explicitly above.
      if (!dev.begin(Wire, a, false)) continue;
      (void)dev.setAccessMode(tlv493d_embed::Tlv493dLite::FASTMODE);

      if (!tlvSanityCheck(dev)) {
        continue;
      }

      addr = a;
#ifdef DEBUG_PRINT
      Serial.printf("[tlv] found at 0x%02X\n", (unsigned)addr);
      Serial.flush();
#endif
      return true;
    }

    delay(10);
  }

  return false;
}

bool Tlv493dDev::readmT(float &mx, float &my, float &mz) {
  if (!dev.updateData()) return false;

  const uint8_t fc = dev.getFrameCounter();
  if (fc == m_lastFrameCounter) return false;

  m_lastFrameCounter = fc;
  mx = dev.getXmT();
  my = dev.getYmT();
  mz = dev.getZmT();
  return true;
}
