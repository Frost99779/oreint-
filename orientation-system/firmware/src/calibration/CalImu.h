#pragma once
#include <stdint.h>
#include "Config.h"

struct Mpu6500Dev;  // forward declaration to avoid circular includes

// Optional raw mag sample during blocking cal (return false to skip sample).
typedef bool (*MagSampleFn)(float& mx_mT, float& my_mT, float& mz_mT, void* ctx);

// Sensor mount matrix: where each sensor axis points in true ENU (informational).
bool computeMountMatrix(float ax, float ay, float az,
                        float mx, float my, float mz,
                        float R[9]);

// IMU calibration: startup blocking gyro bias + IIR auto-recal.
// Also drives accel bias via same stillness gate.
class CalImu {
public:
    // Blocking startup cal — call once in Application::begin() before loop().
    // Collects kStartupCalMs of samples at kStartupCalPeriodMs interval.
    // 3 s still capture: writes gyro_bias + accel_bias when >= kStartupCalMinSamples.
    // Returns true when both gyro_valid and accel_valid are set.
    bool startupBlockingCal(Mpu6500Dev& mpu,
                            MagSampleFn mag_fn = nullptr,
                            void* mag_ctx = nullptr);

    float meanAx() const { return mean_ax_; }
    float meanAy() const { return mean_ay_; }
    float meanAz() const { return mean_az_; }
    float meanMx() const { return mean_mx_; }
    float meanMy() const { return mean_my_; }
    float meanMz() const { return mean_mz_; }
    bool  hasMagMean() const { return mag_mean_valid_; }
    void  setMagMean(float mx_mT, float my_mT, float mz_mT);

    // Feed one IMU sample to the stillness detector and IIR learner.
    // Call at IMU rate (200 Hz). After applying gyro_bias, pass rad/s values.
    // ax/ay/az in g (pre-bias-subtraction values).
    void pushSample(float ax_g, float ay_g, float az_g,
                    float gx_rs, float gy_rs, float gz_rs);

    // Returns true when stillness streak threshold is met.
    bool isStill() const { return m_streak >= core::config::kGyroStillStreakMin; }

    // Instant stillness from ring buffer (no streak required).
    bool checkStillness() const;

    // IIR bias update — call when still. Pass current raw gyro (rad/s).
    // Updates g_cal.gyro_bias with slow IIR. Sets g_cal.gyro_valid = 1.
    void updateGyroBiasIir(float gx_rs, float gy_rs, float gz_rs);

    // IIR accel bias update — call when still. Pass current raw accel (g).
    // Updates g_cal.accel_bias. Sets g_cal.accel_valid = 1.
    void updateAccelBiasIir(float ax_g, float ay_g, float az_g);

    // Time of last auto-recal (millis). Zero = never.
    uint32_t lastRecalMs() const { return m_last_recal_ms; }
    void     setLastRecalMs(uint32_t ms) { m_last_recal_ms = ms; }

private:
    // Stillness ring buffer (accel norm + per-axis gyro)
    float    m_aN[core::config::kGyroStillRingN]{};   // accel norm
    float    m_gx[core::config::kGyroStillRingN]{};
    float    m_gy[core::config::kGyroStillRingN]{};
    float    m_gz[core::config::kGyroStillRingN]{};
    size_t   m_idx{0};
    size_t   m_filled{0};
    uint16_t m_streak{0};
    uint32_t m_last_recal_ms{0};

    float mean_ax_{0.0f};
    float mean_ay_{0.0f};
    float mean_az_{0.0f};
    float mean_mx_{0.0f};
    float mean_my_{0.0f};
    float mean_mz_{0.0f};
    bool  mag_mean_valid_{false};
};

extern CalImu g_calImu;
