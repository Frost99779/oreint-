#pragma once
#include <stdint.h>
#include "Config.h"

// Magnetometer calibration:
// - Median-norm outlier reject
// - Min/max per axis → hard iron offset + diagonal soft iron scale
// - 72-bin sphere coverage metric (12 azimuth × 6 elevation)

class MagCalibrator {
public:
    void reset();

    // Accept a raw mag sample (mT). Returns false if rejected as outlier.
    // If active, accumulates min/max and coverage bins.
    bool acceptSample(float mx_mT, float my_mT, float mz_mT);

    // Compute hard/soft iron from accumulated min/max. Writes g_cal mag fields.
    // Returns false and sets last_stop_reason if insufficient data.
    bool finalizeToGCal();

    void start();
    bool stopAndSave();  // finalize + calSave()

    uint32_t acceptedCount() const { return m_samples; }
    uint8_t  coveragePct() const;   // 0-100
    char     nextAxisToRotate() const;

    bool isActive() const { return m_active; }

private:
    bool     m_active{false};
    uint32_t m_samples{0};

    // Median-norm outlier ring
    float    m_norms[core::config::kMagMedianRingN]{};
    int      m_nidx{0};
    int      m_nfill{0};

    // Min/max per axis
    float    m_min_mT[3]{};
    float    m_max_mT[3]{};

    // 72-bin sphere coverage
    uint8_t  m_bins[72]{};  // 1 = visited

    void markBin(float mx_mT, float my_mT, float mz_mT);
    uint32_t countFilledBins() const;
};

extern MagCalibrator g_magCalibrator;
extern uint8_t       g_magcal_stop_reason;  // debug: 0=OK 1=few 2=small_delta 3=bad_scale

// Apply g_cal hard+soft iron correction to a raw mag sample.
// No-op if g_cal.mag_valid == 0.
void magApplyCal(float& mx_mT, float& my_mT, float& mz_mT);

// Confidence [0,1] of calibrated sample being on the expected sphere.
float magConfidence(float mx_mT, float my_mT, float mz_mT);
