#include "CalMag.h"
#include "NvsCal.h"

#include <math.h>
#include <string.h>
using namespace core::config;

MagCalibrator g_magCalibrator{};
uint8_t       g_magcal_stop_reason = 0U;

static inline float vnorm3f(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

static float medianF(float* a, int n) {
    // insertion sort (n is small)
    for (int i = 1; i < n; i++) {
        const float key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
    return a[n / 2];
}

void MagCalibrator::reset() {
    m_active  = false;
    m_samples = 0U;
    m_nidx    = 0;
    m_nfill   = 0;
    for (int i = 0; i < 3; i++) { m_min_mT[i] = 1e9f; m_max_mT[i] = -1e9f; }
    memset(m_bins, 0, sizeof(m_bins));
    memset(m_norms, 0, sizeof(m_norms));
}

void MagCalibrator::markBin(float mx_mT, float my_mT, float mz_mT) {
    const float norm = vnorm3f(mx_mT, my_mT, mz_mT);
    if (norm < 1e-3f) return;
    const float nx = mx_mT / norm;
    const float ny = my_mT / norm;
    const float nz = mz_mT / norm;
    // azimuth 0-360, elevation -90 to +90
    float az_deg = atan2f(ny, nx) * 57.29578f;  // RAD_TO_DEG
    if (az_deg < 0.0f) az_deg += 360.0f;
    const float el_deg = asinf(fmaxf(-1.0f, fminf(1.0f, nz))) * 57.29578f;

    const int az_bucket = static_cast<int>(az_deg / 30.0f) % 12;
    const int el_bucket = static_cast<int>((el_deg + 90.0f) / 30.0f);
    const int el_clamped = (el_bucket < 0) ? 0 : (el_bucket > 5 ? 5 : el_bucket);
    const int bin = az_bucket * 6 + el_clamped;
    if (bin >= 0 && bin < 72) m_bins[bin] = 1U;
}

uint32_t MagCalibrator::countFilledBins() const {
    uint32_t n = 0U;
    for (int i = 0; i < 72; i++) if (m_bins[i]) n++;
    return n;
}

uint8_t MagCalibrator::coveragePct() const {
    return static_cast<uint8_t>(countFilledBins() * 100U / 72U);
}

char MagCalibrator::nextAxisToRotate() const {
    if (m_samples == 0U) return 'x';
    const float dx = m_max_mT[0] - m_min_mT[0];
    const float dy = m_max_mT[1] - m_min_mT[1];
    const float dz = m_max_mT[2] - m_min_mT[2];
    const float need = kMagCalMinDelta_mT;
    if (dx >= need && dy >= need && dz >= need) return '-';
    if (dx <= dy && dx <= dz) return 'x';
    if (dy <= dz) return 'y';
    return 'z';
}

bool MagCalibrator::acceptSample(float mx_mT, float my_mT, float mz_mT) {
    const float norm = vnorm3f(mx_mT, my_mT, mz_mT);
    m_norms[m_nidx] = norm;
    m_nidx = (m_nidx + 1) % static_cast<int>(kMagMedianRingN);
    if (m_nfill < static_cast<int>(kMagMedianRingN)) m_nfill++;

    bool pass = true;
    if (m_nfill >= static_cast<int>(kMagMedianWarmup)) {
        float tmp[kMagMedianRingN];
        for (int i = 0; i < m_nfill; i++) tmp[i] = m_norms[i];
        const float med = medianF(tmp, m_nfill);
        if (med < 1e-6f) return false;
        const float rel = fabsf(norm - med) / med;
        pass = (rel <= kMagOutlierRelMax);
    }

    if (pass && m_active) {
        m_min_mT[0] = fminf(m_min_mT[0], mx_mT);
        m_min_mT[1] = fminf(m_min_mT[1], my_mT);
        m_min_mT[2] = fminf(m_min_mT[2], mz_mT);
        m_max_mT[0] = fmaxf(m_max_mT[0], mx_mT);
        m_max_mT[1] = fmaxf(m_max_mT[1], my_mT);
        m_max_mT[2] = fmaxf(m_max_mT[2], mz_mT);
        markBin(mx_mT, my_mT, mz_mT);
        m_samples++;
    }
    return pass;
}

bool MagCalibrator::finalizeToGCal() {
    g_magcal_stop_reason = 0U;

    if (m_samples < kMagMinSamples) { g_magcal_stop_reason = 1U; return false; }
    if (coveragePct() < static_cast<uint8_t>(kMagMinCoveragePct)) {
        g_magcal_stop_reason = 5U;
        return false;
    }

    const float dx = m_max_mT[0] - m_min_mT[0];
    const float dy = m_max_mT[1] - m_min_mT[1];
    const float dz = m_max_mT[2] - m_min_mT[2];
    if (dx < kMagCalMinDelta_mT || dy < kMagCalMinDelta_mT || dz < kMagCalMinDelta_mT) {
        g_magcal_stop_reason = 2U; return false;
    }

    const float offx = 0.5f * (m_max_mT[0] + m_min_mT[0]);
    const float offy = 0.5f * (m_max_mT[1] + m_min_mT[1]);
    const float offz = 0.5f * (m_max_mT[2] + m_min_mT[2]);
    const float sx = 0.5f * dx;
    const float sy = 0.5f * dy;
    const float sz = 0.5f * dz;
    if (sx <= 1e-3f || sy <= 1e-3f || sz <= 1e-3f) {
        g_magcal_stop_reason = 3U; return false;
    }
    const float sAvg = (sx + sy + sz) / 3.0f;
    const float scx = sAvg / sx;
    const float scy = sAvg / sy;
    const float scz = sAvg / sz;
    const float smax = fmaxf(scx, fmaxf(scy, scz));
    const float smin = fminf(scx, fminf(scy, scz));
    if (smin < 1e-6f || smax / smin > kMagCalMaxScaleRatio) {
        g_magcal_stop_reason = 4U; return false;
    }

    g_cal.mag_offset[0] = offx;
    g_cal.mag_offset[1] = offy;
    g_cal.mag_offset[2] = offz;
    // Diagonal soft iron matrix stored row-major in mag_softmat[9]
    for (int i = 0; i < 9; i++) g_cal.mag_softmat[i] = 0.0f;
    g_cal.mag_softmat[0] = scx;
    g_cal.mag_softmat[4] = scy;
    g_cal.mag_softmat[8] = scz;
    g_cal.mag_valid = 1U;
    return true;
}

void MagCalibrator::start() {
    reset();
    m_active = true;
}

bool MagCalibrator::stopAndSave() {
    m_active = false;
    if (!finalizeToGCal()) return false;
    return calSave();
}

void magApplyCal(float& mx_mT, float& my_mT, float& mz_mT) {
    if (!g_cal.mag_valid) return;
    const float cx = mx_mT - g_cal.mag_offset[0];
    const float cy = my_mT - g_cal.mag_offset[1];
    const float cz = mz_mT - g_cal.mag_offset[2];
    const float* m = g_cal.mag_softmat;
    mx_mT = m[0]*cx + m[1]*cy + m[2]*cz;
    my_mT = m[3]*cx + m[4]*cy + m[5]*cz;
    mz_mT = m[6]*cx + m[7]*cy + m[8]*cz;
}

float magConfidence(float mx_mT, float my_mT, float mz_mT) {
    if (!g_cal.mag_valid) return 0.0f;
    // Expected radius = average of semi-axes stored during calibration
    const float sAvg = (g_cal.mag_softmat[0] > 0.0f) ?
        (1.0f / g_cal.mag_softmat[0] + 1.0f / g_cal.mag_softmat[4] + 1.0f / g_cal.mag_softmat[8]) / 3.0f
        : 1.0f;
    if (sAvg < 1e-3f) return 0.0f;
    const float meas = sqrtf(mx_mT*mx_mT + my_mT*my_mT + mz_mT*mz_mT);
    const float rel  = fabsf(meas - sAvg) / sAvg;
    const float conf = 1.0f - (rel - 0.20f) / 0.40f;
    return (conf < 0.0f) ? 0.0f : (conf > 1.0f) ? 1.0f : conf;
}
