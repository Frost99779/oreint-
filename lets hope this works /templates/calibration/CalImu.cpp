#include "CalImu.h"
#include "NvsCal.h"
#include "DrvMpu6500.h"

#include <math.h>
#include <string.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace core::config;

static inline uint32_t nowMs() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}
static inline void delayMs(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

CalImu g_calImu{};

bool computeMountMatrix(float ax, float ay, float az,
                        float mx, float my, float mz,
                        float R[9]) {
    const float g_mag = sqrtf(ax * ax + ay * ay + az * az);
    if (g_mag < 0.5f) {
        return false;
    }
    float up[3] = {-ax / g_mag, -ay / g_mag, -az / g_mag};

    const float m_mag = sqrtf(mx * mx + my * my + mz * mz);
    if (m_mag < 1.0e-3f) {
        return false;
    }
    const float mn[3] = {mx / m_mag, my / m_mag, mz / m_mag};

    const float dot_mu = mn[0] * up[0] + mn[1] * up[1] + mn[2] * up[2];
    float north[3] = {
        mn[0] - dot_mu * up[0],
        mn[1] - dot_mu * up[1],
        mn[2] - dot_mu * up[2],
    };
    const float n_mag = sqrtf(north[0] * north[0] + north[1] * north[1] + north[2] * north[2]);
    if (n_mag < 1.0e-3f) {
        return false;
    }
    north[0] /= n_mag;
    north[1] /= n_mag;
    north[2] /= n_mag;

    float east[3] = {
        north[1] * up[2] - north[2] * up[1],
        north[2] * up[0] - north[0] * up[2],
        north[0] * up[1] - north[1] * up[0],
    };
    const float e_mag = sqrtf(east[0] * east[0] + east[1] * east[1] + east[2] * east[2]);
    if (e_mag < 1.0e-3f) {
        return false;
    }
    east[0] /= e_mag;
    east[1] /= e_mag;
    east[2] /= e_mag;

    R[0] = east[0];
    R[1] = north[0];
    R[2] = up[0];
    R[3] = east[1];
    R[4] = north[1];
    R[5] = up[1];
    R[6] = east[2];
    R[7] = north[2];
    R[8] = up[2];
    return true;
}

static inline float vnorm3f(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

// Compute variance of a float ring buffer (filled portion only).
static float ringVariance(const float* buf, size_t filled) {
    if (filled < 2U) return 0.0f;
    double sum = 0.0, sum2 = 0.0;
    for (size_t i = 0; i < filled; i++) {
        sum  += static_cast<double>(buf[i]);
        sum2 += static_cast<double>(buf[i]) * static_cast<double>(buf[i]);
    }
    const double n  = static_cast<double>(filled);
    const double var = (sum2 - sum * sum / n) / n;
    return (var > 0.0) ? static_cast<float>(var) : 0.0f;
}

bool CalImu::startupBlockingCal(Mpu6500Dev& mpu, MagSampleFn mag_fn, void* mag_ctx) {
    static constexpr float kDegToRad = 0.01745329251f;
    float gsum[3] = {0.0f, 0.0f, 0.0f};
    float asum[3] = {0.0f, 0.0f, 0.0f};
    float msum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t n = 0U;
    uint32_t n_mag = 0U;
    mag_mean_valid_ = false;
    const uint32_t end_ms = nowMs() + kStartupCalMs;

    while (nowMs() < end_ms) {
        float ax, ay, az, gx, gy, gz;
        if (!mpu.readAccelGyro(ax, ay, az, gx, gy, gz)) {
            delayMs(kStartupCalPeriodMs);
            continue;
        }
        const float anorm = vnorm3f(ax, ay, az);
        const float gnorm = vnorm3f(gx, gy, gz);
        if (anorm >= kAccelStillMeanMinG && anorm <= kAccelStillMeanMaxG && gnorm < 8.0f) {
            gsum[0] += gx * kDegToRad;
            gsum[1] += gy * kDegToRad;
            gsum[2] += gz * kDegToRad;
            asum[0] += ax;
            asum[1] += ay;
            asum[2] += az;
            n++;

            if (mag_fn != nullptr) {
                float mx, my, mz;
                if (mag_fn(mx, my, mz, mag_ctx)) {
                    msum[0] += mx;
                    msum[1] += my;
                    msum[2] += mz;
                    n_mag++;
                }
            }
        }
        delayMs(kStartupCalPeriodMs);
    }

    if (n >= kStartupCalMinSamples) {
        const float inv = 1.0f / static_cast<float>(n);
        g_cal.gyro_bias[0] = gsum[0] * inv;
        g_cal.gyro_bias[1] = gsum[1] * inv;
        g_cal.gyro_bias[2] = gsum[2] * inv;
        g_cal.gyro_valid = 1U;
        mean_ax_ = asum[0] * inv;
        mean_ay_ = asum[1] * inv;
        mean_az_ = asum[2] * inv;
        // Flat ENU: expect ax≈0, ay≈0, az≈1g (same convention as updateAccelBiasIir)
        g_cal.accel_bias[0] = mean_ax_;
        g_cal.accel_bias[1] = mean_ay_;
        g_cal.accel_bias[2] = mean_az_ - 1.0f;
        g_cal.accel_valid = 1U;

        if (n_mag >= kStartupCalMinSamples) {
            const float inv_m = 1.0f / static_cast<float>(n_mag);
            mean_mx_ = msum[0] * inv_m;
            mean_my_ = msum[1] * inv_m;
            mean_mz_ = msum[2] * inv_m;
            mag_mean_valid_ = true;
        }
    }
    return (g_cal.gyro_valid != 0U) && (g_cal.accel_valid != 0U);
}

void CalImu::setMagMean(float mx_mT, float my_mT, float mz_mT) {
    mean_mx_ = mx_mT;
    mean_my_ = my_mT;
    mean_mz_ = mz_mT;
    mag_mean_valid_ = true;
}

bool CalImu::checkStillness() const {
    if (m_filled < kGyroStillRingN) return false;

    // Accel norm mean and std
    double sum_a = 0.0;
    for (size_t i = 0; i < m_filled; i++) sum_a += static_cast<double>(m_aN[i]);
    const float mean_a = static_cast<float>(sum_a / static_cast<double>(m_filled));
    if (mean_a < kAccelStillMeanMinG || mean_a > kAccelStillMeanMaxG) return false;

    const float std_a = sqrtf(ringVariance(m_aN, m_filled));
    if (std_a >= kAccelStillStdMaxG) return false;

    // Per-axis gyro std
    const float std_gx = sqrtf(ringVariance(m_gx, m_filled));
    const float std_gy = sqrtf(ringVariance(m_gy, m_filled));
    const float std_gz = sqrtf(ringVariance(m_gz, m_filled));
    if (std_gx >= kGyroStillThreshRs) return false;
    if (std_gy >= kGyroStillThreshRs) return false;
    if (std_gz >= kGyroStillThreshRs) return false;
    return true;
}

void CalImu::pushSample(float ax_g, float ay_g, float az_g,
                         float gx_rs, float gy_rs, float gz_rs) {
    m_aN[m_idx] = vnorm3f(ax_g, ay_g, az_g);
    m_gx[m_idx] = gx_rs;
    m_gy[m_idx] = gy_rs;
    m_gz[m_idx] = gz_rs;
    m_idx = (m_idx + 1U) % kGyroStillRingN;
    if (m_filled < kGyroStillRingN) m_filled++;

    if (checkStillness()) {
        if (m_streak < 0xFFFFU) m_streak++;
    } else {
        m_streak = 0U;
    }
}

void CalImu::updateGyroBiasIir(float gx_rs, float gy_rs, float gz_rs) {
    const float alpha = kGyroBiasAlpha;
    g_cal.gyro_bias[0] = (1.0f - alpha) * g_cal.gyro_bias[0] + alpha * gx_rs;
    g_cal.gyro_bias[1] = (1.0f - alpha) * g_cal.gyro_bias[1] + alpha * gy_rs;
    g_cal.gyro_bias[2] = (1.0f - alpha) * g_cal.gyro_bias[2] + alpha * gz_rs;
    g_cal.gyro_valid = 1U;
    m_last_recal_ms = nowMs();
}

void CalImu::updateAccelBiasIir(float ax_g, float ay_g, float az_g) {
    // Expected gravity vector in body frame for level device: [0, 0, 1]
    const float alpha = kAccelBiasAlpha;
    g_cal.accel_bias[0] = (1.0f - alpha) * g_cal.accel_bias[0] + alpha * ax_g;
    g_cal.accel_bias[1] = (1.0f - alpha) * g_cal.accel_bias[1] + alpha * ay_g;
    // Z-axis: bias is (measured - 1.0g)
    g_cal.accel_bias[2] = (1.0f - alpha) * g_cal.accel_bias[2] + alpha * (az_g - 1.0f);
    g_cal.accel_valid = 1U;
}
