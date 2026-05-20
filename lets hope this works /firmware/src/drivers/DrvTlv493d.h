#ifndef PANTILT_DRV_TLV493D_H
#define PANTILT_DRV_TLV493D_H

#include <stdint.h>

#include <tlv493d_driver_pantilt.h>

struct Tlv493dDev {
  tlv493d_embed::Tlv493dLite dev{};
  uint8_t addr{0};
  /** Last frame counter (2-bit); 0xFF = never updated / after reset. */
  uint8_t m_lastFrameCounter{0xFF};

  bool beginAuto();
  bool readmT(float &mx, float &my, float &mz);
};

#endif  // PANTILT_DRV_TLV493D_H
