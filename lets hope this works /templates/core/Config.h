#pragma once
#include <stddef.h>
#include <stdint.h>

// All compile-time constants live here. No magic numbers anywhere else.
// Namespace: core::config (C++ files) or direct access (C files via the constexpr values).

namespace core {
namespace config {

// ---------------------------------------------------------------------------
// Serial / UART
// ---------------------------------------------------------------------------
static constexpr uint32_t kSerialBaud = 115200U;  // Jetson Orin Nano link

// ---------------------------------------------------------------------------
// I2C bus
// ---------------------------------------------------------------------------
static constexpr int      kI2cSdaPin       = 21;
static constexpr int      kI2cSclPin       = 22;
static constexpr uint32_t kI2cClockHz      = 400000U;
static constexpr uint16_t kI2cTimeoutMs    = 10U;   // bounds stuck-bus stall
static constexpr uint16_t kI2cFailRestart  = 200U;  // consecutive fails → restart

// ---------------------------------------------------------------------------
// MPU6500 registers and scales
// ---------------------------------------------------------------------------
static constexpr uint8_t kMpu6500AddrDefault = 0x68U;
// WHO_AM_I valid responses — 0x70 = MPU6500, 0x71 = MPU6500 rev
static constexpr uint8_t kMpu6500WhoAmI0 = 0x70U;
static constexpr uint8_t kMpu6500WhoAmI1 = 0x71U;
// Init register values
static constexpr uint8_t kGyroRangeReg = 0x00U;  // ±250 dps
static constexpr uint8_t kDlpfCfgReg   = 0x03U;  // 44 Hz accel, 42 Hz gyro BW
static constexpr float   kGyroScale    = 131.0f;  // LSB per dps at ±250 dps
static constexpr float   kAccelScale2G = 16384.0f; // LSB per g at ±2g

// ---------------------------------------------------------------------------
// TLV493D addresses
// ---------------------------------------------------------------------------
static constexpr uint8_t kTlv493dAddrCandidates[] = {0x1FU, 0x5EU, 0x35U, 0x36U, 0x37U};
static constexpr size_t  kTlv493dAddrN = 5U;
static constexpr uint32_t kTlv493dStartupDelayMs = 40U;

// ---------------------------------------------------------------------------
// Scheduling
// ---------------------------------------------------------------------------
static constexpr uint32_t kImuPeriodMs    = 5U;   // 200 Hz IMU read
static constexpr uint32_t kFilterPeriodMs = 10U;  // 100 Hz filter + packet output
static constexpr uint32_t kFilterHz       = 100U;
static constexpr uint32_t kCalProgPeriodMs = 1000U; // CAL_PROG report interval

// ---------------------------------------------------------------------------
// Madgwick filter
// ---------------------------------------------------------------------------
static constexpr float kBetaStill = 0.033f;
static constexpr float kBetaMove  = 0.10f;
static constexpr bool  kEnableConfBeta = false; // set true to enable confidence-weighted beta

// ---------------------------------------------------------------------------
// Magnetic declination (Bengaluru, India — update if device relocated)
// ---------------------------------------------------------------------------
static constexpr float kMagDeclinationDeg = -0.8f;

// ---------------------------------------------------------------------------
// Magnetometer calibration
// ---------------------------------------------------------------------------
static constexpr uint32_t kMagMinSamples    = 200U;
static constexpr uint32_t kMagSphereBins    = 72U;  // 12 azimuth × 6 elevation
static constexpr uint32_t kMagMinCoveragePct = 60U;
static constexpr float    kMagCalMinDelta_mT = 2.0f;
static constexpr float    kMagCalMaxScaleRatio = 2.5f;
// Median-norm outlier gate
static constexpr size_t   kMagMedianRingN   = 25U;
static constexpr uint8_t  kMagMedianWarmup  = 7U;
static constexpr float    kMagOutlierRelMax = 0.40f;

// ---------------------------------------------------------------------------
// Gyro / accel calibration
// ---------------------------------------------------------------------------
static constexpr uint32_t kGyroStillRingN       = 200U;   // ring buffer depth
static constexpr uint16_t kGyroStillStreakMin   = 20U;    // consecutive passing windows for "still"
static constexpr float    kGyroStillThreshRs    = 0.01f;  // gyro std-dev (rad/s)
static constexpr float    kAccelStillMeanMinG   = 0.88f;
static constexpr float    kAccelStillMeanMaxG   = 1.12f;
static constexpr float    kAccelStillStdMaxG    = 0.03f;
// Startup blocking gyro bias cal
static constexpr uint32_t kStartupCalMs         = 3000U;  // 3 s
static constexpr uint32_t kStartupCalPeriodMs   = 10U;    // 10 ms intervals
static constexpr uint32_t kStartupCalMinSamples = 50U;
static constexpr uint32_t kImuBlockingCalMaxMs  = 30000U; // CMD:CAL_IMU retry budget
static constexpr uint32_t kCalSettleMs          = 30000U; // post mag-cal settle before IMU
static constexpr uint32_t kCalSettlePrintMs     = 5000U;  // settle countdown interval
static constexpr uint32_t kMountMagSampleMs     = 3000U;  // dedicated mag mean for mount_R
static constexpr uint32_t kMountMagSamplePeriodMs = 50U;
static constexpr uint32_t kMountMagMinSamples     = 10U;
// Auto-recal IIR
static constexpr float    kGyroBiasAlpha        = 0.02f;
static constexpr float    kAccelBiasAlpha        = 0.02f;
static constexpr uint32_t kAutoRecalIntervalMs  = 1800000U; // 30 min

// ---------------------------------------------------------------------------
// NVS calibration blob
// ---------------------------------------------------------------------------
static constexpr uint32_t kCalMagic   = 0xCA11BA11U;
static constexpr uint16_t kCalVersion = 2U;  // v2: mount_R[9] + mount_valid in CalBlob

// ---------------------------------------------------------------------------
// UART RX
// ---------------------------------------------------------------------------
static constexpr size_t   kRxRingSize            = 512U;
static constexpr size_t   kCommandLineSize        = 96U;
static constexpr uint32_t kUartReadBudgetBytes    = 128U;
static constexpr uint32_t kLineTimeoutMs          = 50U;

// ---------------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------------
static constexpr const char* kFirmwareVersion = "orientation-v1.0.0";

}  // namespace config
}  // namespace core
