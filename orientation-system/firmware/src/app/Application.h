#pragma once

#include <cstddef>
#include <stdint.h>

#include "DrvMpu6500.h"
#include "DrvBmm150.h"
#include "Madgwick.h"
#include "NvsCal.h"
#include "CalMag.h"
#include "CalImu.h"
#include "UartTransport.h"
#include "CommandParser.h"
#include "PacketWriter.h"
#include "SystemState.h"

class Application {
public:
    void begin();
    void loopOnce();

private:
    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------
    Mpu6500Dev   mpu_{};
    Bmm150Dev    mag_{};
    bool         mag_ok_{false};

    // -----------------------------------------------------------------------
    // Calibration
    // -----------------------------------------------------------------------
    // g_cal (extern in NvsCal.h) holds the live calibration blob.
    // g_calImu (extern in CalImu.h) drives stillness detection + IIR.
    // g_magCalibrator (extern in CalMag.h) drives mag collection.
    //
    // No local copies — Application uses the globals directly for simplicity.

    // -----------------------------------------------------------------------
    // Fusion
    // -----------------------------------------------------------------------
    Madgwick ahrs_{};
    uint32_t filter_ticks_{0U};

    // Latest output (written by runFilter, read by handleCmdStatus + packet)
    float yaw_{0.0f}, pitch_{0.0f}, roll_{0.0f};

    static constexpr size_t kYawWindowN = 100U;
    float    yaw_cos_[kYawWindowN]{};
    float    yaw_sin_[kYawWindowN]{};
    size_t   yaw_idx_{0U};
    size_t   yaw_filled_{0U};
    bool     converged_{false};

    // -----------------------------------------------------------------------
    // Latest sensor values (written by readSensors, consumed by runFilter)
    // -----------------------------------------------------------------------
    float ax_{0.0f}, ay_{0.0f}, az_{0.0f};  // accel after bias (g)
    float gx_{0.0f}, gy_{0.0f}, gz_{0.0f};  // gyro after bias (rad/s)
    float mx_{0.0f}, my_{0.0f}, mz_{0.0f};  // mag after cal (mT), valid if mag_sample_ok_
    bool  mag_sample_ok_{false};
    bool  using_mag_{false};

    // -----------------------------------------------------------------------
    // Comms
    // -----------------------------------------------------------------------
    comms::UartTransport uart_{};
    uint8_t              seq_{0U};

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    core::SystemStateManager state_{0U};

    // -----------------------------------------------------------------------
    // Scheduler timestamps
    // -----------------------------------------------------------------------
    uint32_t last_imu_ms_{0U};
    uint32_t last_filt_ms_{0U};
    uint32_t last_cal_prog_ms_{0U};
    uint32_t last_auto_recal_check_ms_{0U};
    uint32_t last_sheets_ms_{0U};

    // -----------------------------------------------------------------------
    // I2C fault counter
    // -----------------------------------------------------------------------
    uint16_t i2c_fails_{0U};

    // -----------------------------------------------------------------------
    // Calibration state machine (mag → settle → IMU → mount → NVS)
    // -----------------------------------------------------------------------
    bool     cal_settle_active_{false};
    uint32_t cal_settle_start_ms_{0U};
    uint32_t cal_settle_last_print_ms_{0U};
    bool     waiting_for_mag_cal_{false};

    // -----------------------------------------------------------------------
    // Startup timestamp for beta boost (first 30 seconds)
    // -----------------------------------------------------------------------
    uint32_t startup_ms_{0U};

    // -----------------------------------------------------------------------
    // Private methods
    // -----------------------------------------------------------------------
    void readSensors(uint32_t now_ms);
    void runFilter(uint32_t now_ms);
    void processCommands(uint32_t now_ms);
    void handleCommand(comms::CmdId id, uint32_t now_ms);

    void handleCmdStatus(uint32_t now_ms);
    void handleCmdCalMag(uint32_t now_ms);
    void handleCmdCalMagStop(uint32_t now_ms);
    void handleCmdCalImu(uint32_t now_ms);
    void handleCmdSaveCal(uint32_t now_ms);
    void handleCmdLoadCal(uint32_t now_ms);
    void handleCmdClearCal(uint32_t now_ms);
    void handleCmdGetOrientation(uint32_t now_ms);
    void handleCmdZeroHeading(uint32_t now_ms);
    void handleCmdRaw(uint32_t now_ms);

    void emitCalProgress();
    void checkAutoRecal(uint32_t now_ms);

    void startPostMagCalSettle();
    void runPostMagCalSettle(uint32_t now_ms);
    void runAutoImuCal();

    void applyMountRotation(float& x, float& y, float& z) const;
    bool checkConvergence();
};
