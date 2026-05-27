#include "Application.h"

#include <Arduino.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "HalI2c.h"
#include "Config.h"
#include "SheetsLogger.h"

using namespace core::config;
using namespace comms;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline bool elapsed(uint32_t now, uint32_t last, uint32_t period_ms) {
    return (now - last) >= period_ms;
}

static void serialReply(const char* msg) {
    Serial.println(msg);
    Serial.flush();
}

struct MagReadCtx {
    Bmm150Dev* dev;
};

static bool magReadStill(float& mx_mT, float& my_mT, float& mz_mT, void* ctx) {
    auto* c = static_cast<MagReadCtx*>(ctx);
    // BMM150 readmT returns false on I2C error — retry briefly.
    for (uint8_t attempt = 0U; attempt < 8U; attempt++) {
        if (c->dev->readmT(mx_mT, my_mT, mz_mT)) {
            return true;
        }
        delay(5);
    }
    return false;
}

static void collectMagMeanForMount(Bmm150Dev& mag, uint32_t duration_ms) {
    MagReadCtx ctx{&mag};
    float msum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t n = 0U;
    const uint32_t end_ms = static_cast<uint32_t>(millis()) + duration_ms;
    while (static_cast<uint32_t>(millis()) < end_ms) {
        float mx, my, mz;
        if (magReadStill(mx, my, mz, &ctx)) {
            msum[0] += mx;
            msum[1] += my;
            msum[2] += mz;
            n++;
        }
        delay(25);
    }
    if (n >= 20U) {
        const float inv = 1.0f / static_cast<float>(n);
        g_calImu.setMagMean(msum[0] * inv, msum[1] * inv, msum[2] * inv);
    }
}

static void tryComputeMountFromCalMeans() {
    if (!g_calImu.hasMagMean()) {
        g_cal.mount_valid = 0U;
        return;
    }
    float R[9]{};
    if (computeMountMatrix(g_calImu.meanAx(), g_calImu.meanAy(), g_calImu.meanAz(),
                           g_calImu.meanMx(), g_calImu.meanMy(), g_calImu.meanMz(), R)) {
        memcpy(g_cal.mount_R, R, sizeof(R));
        g_cal.mount_valid = 1U;
    } else {
        g_cal.mount_valid = 0U;
    }
}

// ---------------------------------------------------------------------------
// begin()
// ---------------------------------------------------------------------------

void Application::begin() {
    Serial.begin(kSerialBaud);
    // Brief drain: host may already be sending probes before READY.
    delay(40);
    while (Serial.available()) { (void)Serial.read(); }

    halI2cBegin();

    // Load NVS calibration — failure is non-fatal (fresh device has no cal).
    calResetDefaults();
    calLoad();  // populates g_cal if valid blob exists

    // Init MPU6500
    if (!mpu_.beginAuto()) {
        Serial.println("ERROR:mpu_init");
        Serial.flush();
        // Stay in ERROR — loop will keep calling loopOnce() which just returns.
        state_.transitionTo(core::SystemState::ERROR, "mpu_init_failed", 0U);
        return;
    }

    // Init BMM150 (optional — system works without mag, just yaw drifts)
    mag_ok_ = mag_.beginAuto();

    if (g_cal.mag_valid != 0U) {
        if (g_cal.gyro_valid == 0U || g_cal.accel_valid == 0U) {
            serialReply("CAL_AUTO imu_cal_starting hold_still_3s");
            Serial.flush();
            runAutoImuCal();
        } else if (g_cal.mount_valid == 0U && mag_ok_) {
            collectMagMeanForMount(mag_, kMountMagSampleMs);
            tryComputeMountFromCalMeans();
            if (g_cal.mount_valid != 0U) {
                (void)calSave();
            }
        }
        waiting_for_mag_cal_ = false;
    } else {
        waiting_for_mag_cal_ = true;
        serialReply("WARN no_mag_cal send CMD:CAL_MAG first");
    }

    uart_.begin();
    ahrs_.begin();

    state_.transitionTo(core::SystemState::IDLE, "init_ok", static_cast<uint32_t>(millis()));

    // Subscribe loopTask WDT AFTER init completes (init may take >5 s on cold boot).
    static_cast<void>(esp_task_wdt_add(nullptr));

    if (kEnableSheetsLogger) {
        sheets::begin();
    }

    // Signal host: ready to receive commands and stream packets.
    {
        char buf[64]{};
        snprintf(buf, sizeof(buf), "READY fw=%s mag=%s cal=%s",
                 kFirmwareVersion,
                 mag_ok_  ? "yes" : "no",
                 (g_cal.gyro_valid || g_cal.mag_valid) ? "loaded" : "none");
        serialReply(buf);
    }

    startup_ms_ = static_cast<uint32_t>(millis());
}

// ---------------------------------------------------------------------------
// loopOnce()
// ---------------------------------------------------------------------------

void Application::loopOnce() {
    if (state_.state() == core::SystemState::ERROR) return;

    esp_task_wdt_reset();
    const uint32_t now = static_cast<uint32_t>(millis());

    uart_.service(now);
    processCommands(now);

    if (elapsed(now, last_imu_ms_, kImuPeriodMs)) {
        last_imu_ms_ = now;
        readSensors(now);
    }

    if (elapsed(now, last_filt_ms_, kFilterPeriodMs)) {
        last_filt_ms_ = now;
        runFilter(now);
    }

    if (g_magCalibrator.isActive() &&
        elapsed(now, last_cal_prog_ms_, kCalProgPeriodMs)) {
        last_cal_prog_ms_ = now;
        emitCalProgress();
    }

    if (cal_settle_active_) {
        runPostMagCalSettle(now);
    }

    checkAutoRecal(now);
}

void Application::applyMountRotation(float& x, float& y, float& z) const {
    if (g_cal.mount_valid == 0U) return;
    const float* R = g_cal.mount_R;
    const float xo = R[0]*x + R[3]*y + R[6]*z;
    const float yo = R[1]*x + R[4]*y + R[7]*z;
    const float zo = R[2]*x + R[5]*y + R[8]*z;
    x = xo; y = yo; z = zo;
}

// ---------------------------------------------------------------------------
// readSensors()
// ---------------------------------------------------------------------------

void Application::readSensors(uint32_t /*now_ms*/) {
    float ax_raw, ay_raw, az_raw, gx_dps, gy_dps, gz_dps;
    if (!mpu_.readAccelGyro(ax_raw, ay_raw, az_raw, gx_dps, gy_dps, gz_dps)) {
        if (i2c_fails_ < 0xFFFFU) i2c_fails_++;
        if (i2c_fails_ >= kI2cFailRestart) {
            serialReply("ERR i2c_restart");
            esp_restart();
        }
        return;
    }
    i2c_fails_ = 0U;

    // Convert gyro dps → rad/s
    constexpr float kDegToRad = 0.01745329251f;
    float gx_rs = gx_dps * kDegToRad;
    float gy_rs = gy_dps * kDegToRad;
    float gz_rs = gz_dps * kDegToRad;

    // Apply gyro bias
    if (g_cal.gyro_valid) {
        gx_rs -= g_cal.gyro_bias[0];
        gy_rs -= g_cal.gyro_bias[1];
        gz_rs -= g_cal.gyro_bias[2];
    }

    // Apply accel bias
    float ax = ax_raw, ay = ay_raw, az = az_raw;
    if (g_cal.accel_valid) {
        ax -= g_cal.accel_bias[0];
        ay -= g_cal.accel_bias[1];
        az -= g_cal.accel_bias[2];
    }

    // Update stillness detector
    g_calImu.pushSample(ax, ay, az, gx_rs, gy_rs, gz_rs);

    // Store corrected values
    ax_ = ax; ay_ = ay; az_ = az;
    gx_ = gx_rs; gy_ = gy_rs; gz_ = gz_rs;

    // Magnetometer (optional)
    mag_sample_ok_ = false;
    if (mag_ok_) {
        float mx_raw, my_raw, mz_raw;
        if (mag_.readmT(mx_raw, my_raw, mz_raw)) {
            // Feed raw to mag calibrator (for calibration collection and confidence)
            g_magCalibrator.acceptSample(mx_raw, my_raw, mz_raw);

            // Apply calibration for filter use
            float mx = mx_raw, my = my_raw, mz = mz_raw;
            magApplyCal(mx, my, mz);
            mx_ = mx; my_ = my; mz_ = mz;
            mag_sample_ok_ = true;
        }
    }
}

// ---------------------------------------------------------------------------
// runFilter()
// ---------------------------------------------------------------------------

void Application::runFilter(uint32_t now_ms) {
    // dt in seconds
    static uint32_t last_filt_us = 0U;
    const uint32_t now_us = static_cast<uint32_t>(micros());
    float dt = static_cast<float>(now_us - last_filt_us) * 1.0e-6f;
    last_filt_us = now_us;
    if (dt <= 0.0f || dt > 0.5f) dt = static_cast<float>(kFilterPeriodMs) * 1.0e-3f;

    const bool still   = g_calImu.isStill();
    float beta   = still ? kBetaStill : kBetaMove;

    // Beta boost: faster convergence during first 30 s when held still
    const uint32_t elapsed_ms = now_ms - startup_ms_;
    if (elapsed_ms < 30000U && g_calImu.checkStillness()) {
        beta = kBetaStartupBoost;
    }

    using_mag_ = mag_ok_ && mag_sample_ok_ && (g_cal.mag_valid != 0U);

    const float mx_u = using_mag_ ? mx_ : 0.0f;
    const float my_u = using_mag_ ? my_ : 0.0f;
    const float mz_u = using_mag_ ? mz_ : 0.0f;

    float ax_e = ax_, ay_e = ay_, az_e = az_;
    float gx_e = gx_, gy_e = gy_, gz_e = gz_;
    float mx_e = mx_u, my_e = my_u, mz_e = mz_u;
    applyMountRotation(ax_e, ay_e, az_e);
    applyMountRotation(gx_e, gy_e, gz_e);
    if (using_mag_) applyMountRotation(mx_e, my_e, mz_e);

    // Frame remap: chip (x=fwd, y=left, z=up) → Madgwick (x=fwd, y=right, z=up)
    {
        const float ay_m = -ay_e;
        const float gy_m = -gy_e;
        const float my_m = -my_e;
        ay_e = ay_m;
        gy_e = gy_m;
        my_e = my_m;
    }

    ahrs_.update(gx_e, gy_e, gz_e, ax_e, ay_e, az_e, mx_e, my_e, mz_e, beta, dt);
    filter_ticks_++;

    ahrs_.eulerDeg(roll_, pitch_, yaw_);

    yaw_ += kMagDeclinationDeg;
    if (yaw_ < 0.0f)    yaw_ += 360.0f;
    if (yaw_ >= 360.0f) yaw_ -= 360.0f;

    if (g_cal.heading_offset_valid != 0U) {
        yaw_ -= g_cal.heading_offset_deg;
        if (yaw_ < 0.0f)    yaw_ += 360.0f;
        if (yaw_ >= 360.0f) yaw_ -= 360.0f;
    }

    checkConvergence();

    if (kEnableSheetsLogger && converged_ &&
        elapsed(now_ms, last_sheets_ms_, kSheetsLogPeriodMs)) {
        last_sheets_ms_ = now_ms;
        SheetsSample samp{ now_ms, yaw_, pitch_, roll_ };
        (void)sheets::enqueue(samp);
    }

    if (still) {
        // Use raw gyro (before bias subtraction) for IIR update
        // We stored gx_/gy_/gz_ AFTER bias subtraction. Add back bias to get raw:
        const float gx_raw = gx_ + (g_cal.gyro_valid ? g_cal.gyro_bias[0] : 0.0f);
        const float gy_raw = gy_ + (g_cal.gyro_valid ? g_cal.gyro_bias[1] : 0.0f);
        const float gz_raw = gz_ + (g_cal.gyro_valid ? g_cal.gyro_bias[2] : 0.0f);

        if (elapsed(now_ms, g_calImu.lastRecalMs(), kAutoRecalIntervalMs)) {
            g_calImu.updateGyroBiasIir(gx_raw, gy_raw, gz_raw);
            g_calImu.updateAccelBiasIir(ax_, ay_, az_);
        }
    }
}

bool Application::checkConvergence() {
    const float yaw_rad = yaw_ * 0.01745329251f;
    yaw_cos_[yaw_idx_] = cosf(yaw_rad);
    yaw_sin_[yaw_idx_] = sinf(yaw_rad);
    yaw_idx_ = (yaw_idx_ + 1U) % kYawWindowN;
    if (yaw_filled_ < kYawWindowN) yaw_filled_++;

    if (yaw_filled_ < kYawWindowN) return false;

    const size_t half = kYawWindowN / 2U;
    float c1 = 0.0f, s1 = 0.0f, c2 = 0.0f, s2 = 0.0f;
    for (size_t i = 0; i < half; i++) {
        const size_t k1 = (yaw_idx_ + i) % kYawWindowN;
        const size_t k2 = (yaw_idx_ + half + i) % kYawWindowN;
        c1 += yaw_cos_[k1]; s1 += yaw_sin_[k1];
        c2 += yaw_cos_[k2]; s2 += yaw_sin_[k2];
    }
    const float inv = 1.0f / static_cast<float>(half);
    c1 *= inv; s1 *= inv; c2 *= inv; s2 *= inv;

    const float dot = c1 * c2 + s1 * s2;
    const float drift_rad = acosf(fmaxf(-1.0f, fminf(1.0f, dot)));
    const float drift_deg = drift_rad * 57.29577951f;

    float cf = 0.0f, sf = 0.0f;
    for (size_t i = 0; i < kYawWindowN; i++) { cf += yaw_cos_[i]; sf += yaw_sin_[i]; }
    cf /= static_cast<float>(kYawWindowN);
    sf /= static_cast<float>(kYawWindowN);
    const float R = sqrtf(cf * cf + sf * sf);
    const float std_deg = (R > 0.999999f) ? 0.0f
                          : sqrtf(-2.0f * logf(R)) * 57.29577951f;

    if (!converged_ && std_deg < 1.5f && drift_deg < 0.8f) {
        converged_ = true;
    } else if (converged_ && drift_deg > 2.0f) {
        converged_ = false;
    }
    return converged_;
}

// ---------------------------------------------------------------------------
// processCommands()
// ---------------------------------------------------------------------------

void Application::processCommands(uint32_t now_ms) {
    char line[core::config::kCommandLineSize]{};
    while (uart_.getLine(line, sizeof(line))) {
        const CmdId id = parseCommand(line);
        handleCommand(id, now_ms);
    }
}

void Application::handleCommand(CmdId id, uint32_t now_ms) {
    switch (id) {
        case CmdId::STATUS:      handleCmdStatus(now_ms);    break;
        case CmdId::CAL_MAG:     handleCmdCalMag(now_ms);    break;
        case CmdId::CAL_MAG_STOP:handleCmdCalMagStop(now_ms);break;
        case CmdId::CAL_IMU:     handleCmdCalImu(now_ms);    break;
        case CmdId::SAVE_CAL:    handleCmdSaveCal(now_ms);   break;
        case CmdId::LOAD_CAL:    handleCmdLoadCal(now_ms);   break;
        case CmdId::CLEAR_CAL:   handleCmdClearCal(now_ms);  break;
        case CmdId::RESET:       esp_restart();              break;
        case CmdId::PING:        serialReply("OK");          break;
        case CmdId::GET_ORIENTATION: handleCmdGetOrientation(now_ms); break;
        case CmdId::ZERO_HEADING:    handleCmdZeroHeading(now_ms);    break;
        case CmdId::RAW:             handleCmdRaw(now_ms);            break;
        default:                 serialReply("ERR unknown_command"); break;
    }
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

void Application::handleCmdStatus(uint32_t now_ms) {
    const bool still = g_calImu.isStill();
    char buf[384]{};
    snprintf(buf, sizeof(buf),
             "OK state=%s uptime=%lu mag=%s mag_cal=%u gyro_cal=%u accel_cal=%u "
             "mount_valid=%u filter_ticks=%lu still=%u yaw=%.2f pitch=%.2f roll=%.2f wifi=%u",
             core::toString(state_.state()),
             static_cast<unsigned long>(now_ms),
             mag_ok_ ? "yes" : "no",
             static_cast<unsigned>(g_cal.mag_valid),
             static_cast<unsigned>(g_cal.gyro_valid),
             static_cast<unsigned>(g_cal.accel_valid),
             static_cast<unsigned>(g_cal.mount_valid),
             static_cast<unsigned long>(filter_ticks_),
             static_cast<unsigned>(still ? 1U : 0U),
             static_cast<double>(yaw_),
             static_cast<double>(pitch_),
             static_cast<double>(roll_),
             static_cast<unsigned>((kEnableSheetsLogger && sheets::wifiUp()) ? 1U : 0U));

    if (g_cal.mount_valid != 0U) {
        char m_buf[200]{};
        snprintf(m_buf, sizeof(m_buf),
                 " mount_X=%.2f,%.2f,%.2f mount_Y=%.2f,%.2f,%.2f mount_Z=%.2f,%.2f,%.2f",
                 static_cast<double>(g_cal.mount_R[0]),
                 static_cast<double>(g_cal.mount_R[1]),
                 static_cast<double>(g_cal.mount_R[2]),
                 static_cast<double>(g_cal.mount_R[3]),
                 static_cast<double>(g_cal.mount_R[4]),
                 static_cast<double>(g_cal.mount_R[5]),
                 static_cast<double>(g_cal.mount_R[6]),
                 static_cast<double>(g_cal.mount_R[7]),
                 static_cast<double>(g_cal.mount_R[8]));
        strncat(buf, m_buf, sizeof(buf) - strlen(buf) - 1U);
    }
    serialReply(buf);
}

void Application::handleCmdGetOrientation(uint32_t /*now_ms*/) {
    const bool cal_ok = (g_cal.gyro_valid != 0U) && (g_cal.accel_valid != 0U);
    if (!cal_ok || !converged_) {
        serialReply("ERR not_converged");
        return;
    }
    uint8_t flags = FLAG_FILTER_CONVERGED;
    if (g_cal.mag_valid) flags |= FLAG_MAG_CAL_VALID;
    if (g_cal.gyro_valid && g_cal.accel_valid) flags |= FLAG_IMU_CAL_VALID;
    if (g_calImu.isStill()) flags |= FLAG_STATIC;

    (void)sendPacket(seq_++, yaw_, pitch_, roll_, flags);
}

void Application::handleCmdCalMag(uint32_t /*now_ms*/) {
    if (cal_settle_active_) {
        serialReply("ERR settle_in_progress wait_for_imu_cal");
        return;
    }
    if (g_magCalibrator.isActive()) { serialReply("ERR already_collecting"); return; }
    g_magCalibrator.start();
    serialReply("OK mag_cal=started rotate_device_figure8");
}

void Application::handleCmdCalMagStop(uint32_t /*now_ms*/) {
    if (!g_magCalibrator.isActive()) { serialReply("ERR not_collecting"); return; }
    const uint8_t pct = g_magCalibrator.coveragePct();
    if (g_magCalibrator.stopAndSave()) {
        char buf[64]{};
        snprintf(buf, sizeof(buf), "OK coverage=%u%% samples=%lu settling=%lus",
                 static_cast<unsigned>(pct),
                 static_cast<unsigned long>(g_magCalibrator.acceptedCount()),
                 static_cast<unsigned long>(kCalSettleMs / 1000U));
        serialReply(buf);
        startPostMagCalSettle();
        waiting_for_mag_cal_ = false;
    } else {
        char buf[64]{};
        snprintf(buf, sizeof(buf), "ERR cal_failed reason=%u coverage=%u%%",
                 static_cast<unsigned>(g_magcal_stop_reason),
                 static_cast<unsigned>(pct));
        serialReply(buf);
    }
}

void Application::handleCmdCalImu(uint32_t /*now_ms*/) {
    if (cal_settle_active_) {
        serialReply("ERR settle_in_progress wait_for_auto_imu_cal");
        return;
    }
    if (waiting_for_mag_cal_ || g_cal.mag_valid == 0U) {
        serialReply("ERR run_CMD:CAL_MAG first");
        return;
    }
    serialReply("OK cal_imu=starting hold_still_flat");
    Serial.flush();
    runAutoImuCal();
}

void Application::handleCmdSaveCal(uint32_t /*now_ms*/) {
    serialReply(calSave() ? "OK" : "ERR nvs_write_failed");
}

void Application::handleCmdLoadCal(uint32_t /*now_ms*/) {
    serialReply(calLoad() ? "OK" : "ERR nvs_load_failed");
}

void Application::handleCmdClearCal(uint32_t /*now_ms*/) {
    calClear();
    cal_settle_active_ = false;
    waiting_for_mag_cal_ = true;
    serialReply("OK cal_cleared");
}

void Application::handleCmdZeroHeading(uint32_t /*now_ms*/) {
    if (!converged_) {
        serialReply("ERR not_converged_yet");
        return;
    }
    g_cal.heading_offset_deg = yaw_;
    g_cal.heading_offset_valid = 1U;
    if (calSave()) {
        char buf[64]{};
        snprintf(buf, sizeof(buf), "OK heading_zero=%.2f", static_cast<double>(yaw_));
        serialReply(buf);
    } else {
        serialReply("ERR nvs_save_failed");
    }
}

void Application::handleCmdRaw(uint32_t /*now_ms*/) {
    float ax, ay, az, gx, gy, gz;
    const bool ok_imu = mpu_.readAccelGyro(ax, ay, az, gx, gy, gz);
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    const bool ok_mag = mag_ok_ ? mag_.readmT(mx, my, mz) : false;
    char buf[256]{};
    snprintf(buf, sizeof(buf),
             "RAW imu=%u ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f "
             "mag=%u mx=%.3f my=%.3f mz=%.3f",
             static_cast<unsigned>(ok_imu ? 1U : 0U),
             static_cast<double>(ax), static_cast<double>(ay), static_cast<double>(az),
             static_cast<double>(gx), static_cast<double>(gy), static_cast<double>(gz),
             static_cast<unsigned>(ok_mag ? 1U : 0U),
             static_cast<double>(mx), static_cast<double>(my), static_cast<double>(mz));
    serialReply(buf);
}

// ---------------------------------------------------------------------------
// emitCalProgress()
// ---------------------------------------------------------------------------

void Application::emitCalProgress() {
    const uint8_t coverage = g_magCalibrator.coveragePct();

    char buf[96]{};
    snprintf(buf, sizeof(buf),
             "CAL_PROG samples=%lu coverage=%u%% next_axis=%c",
             static_cast<unsigned long>(g_magCalibrator.acceptedCount()),
             static_cast<unsigned>(coverage),
             g_magCalibrator.nextAxisToRotate());
    serialReply(buf);

    // Auto-stop and save at 100% coverage
    if (coverage >= 100U) {
        if (g_magCalibrator.stopAndSave()) {
            char final_buf[64]{};
            snprintf(final_buf, sizeof(final_buf),
                     "OK coverage=100%% samples=%lu settling=%lus",
                     static_cast<unsigned long>(g_magCalibrator.acceptedCount()),
                     static_cast<unsigned long>(kCalSettleMs / 1000U));
            serialReply(final_buf);
            startPostMagCalSettle();
            waiting_for_mag_cal_ = false;
        } else {
            serialReply("ERR cal_failed coverage=100%%");
        }
    }
}

// ---------------------------------------------------------------------------
// Calibration state machine
// ---------------------------------------------------------------------------

void Application::startPostMagCalSettle() {
    cal_settle_active_        = true;
    cal_settle_start_ms_      = static_cast<uint32_t>(millis());
    cal_settle_last_print_ms_ = cal_settle_start_ms_;
}

void Application::runPostMagCalSettle(uint32_t now_ms) {
    const uint32_t elapsed_ms = now_ms - cal_settle_start_ms_;
    const uint32_t remaining_ms = (elapsed_ms >= kCalSettleMs) ? 0U :
                                  (kCalSettleMs - elapsed_ms);

    if ((now_ms - cal_settle_last_print_ms_) >= kCalSettlePrintMs) {
        cal_settle_last_print_ms_ = now_ms;
        if (remaining_ms > 0U) {
            char buf[64]{};
            snprintf(buf, sizeof(buf),
                     "SETTLE remaining=%lus keep_device_flat_still",
                     static_cast<unsigned long>(remaining_ms / 1000U));
            serialReply(buf);
        }
    }

    if (elapsed_ms >= kCalSettleMs) {
        cal_settle_active_ = false;
        serialReply("SETTLE done running_imu_cal hold_still_3s");
        Serial.flush();
        runAutoImuCal();
    }
}

void Application::runAutoImuCal() {
    esp_task_wdt_reset();

    MagReadCtx mctx{&mag_};
    const MagSampleFn mag_fn = mag_ok_ ? magReadStill : nullptr;
    void* mag_ctx = mag_ok_ ? static_cast<void*>(&mctx) : nullptr;

    const uint32_t deadline = static_cast<uint32_t>(millis()) + kImuBlockingCalMaxMs;
    bool imu_ok = false;
    while (static_cast<uint32_t>(millis()) < deadline) {
        esp_task_wdt_reset();
        if (g_calImu.startupBlockingCal(mpu_, mag_fn, mag_ctx)) {
            imu_ok = true;
            break;
        }
        serialReply("CAL_IMU retry hold_still_flat");
        Serial.flush();
    }

    if (!imu_ok || g_cal.gyro_valid == 0U || g_cal.accel_valid == 0U) {
        serialReply("ERR imu_cal_failed keep_still_and_retry_CMD:CAL_IMU");
        return;
    }

    float sum_mx = 0.0f;
    float sum_my = 0.0f;
    float sum_mz = 0.0f;
    uint32_t mag_samples = 0U;
    const uint32_t mag_start = static_cast<uint32_t>(millis());
    while ((static_cast<uint32_t>(millis()) - mag_start) < kMountMagSampleMs) {
        esp_task_wdt_reset();
        float rx = 0.0f;
        float ry = 0.0f;
        float rz = 0.0f;
        if (mag_ok_ && magReadStill(rx, ry, rz, &mctx)) {
            sum_mx += rx;
            sum_my += ry;
            sum_mz += rz;
            mag_samples++;
        }
        delay(kMountMagSamplePeriodMs);
    }

    if (mag_ok_ && mag_samples >= kMountMagMinSamples) {
        const float inv = 1.0f / static_cast<float>(mag_samples);
        const float mean_mx = sum_mx * inv;
        const float mean_my = sum_my * inv;
        const float mean_mz = sum_mz * inv;
        float R[9]{};
        if (computeMountMatrix(g_calImu.meanAx(), g_calImu.meanAy(), g_calImu.meanAz(),
                               mean_mx, mean_my, mean_mz, R)) {
            memcpy(g_cal.mount_R, R, sizeof(R));
            g_cal.mount_valid = 1U;
        } else {
            g_cal.mount_valid = 0U;
            serialReply("WARN mount_matrix_failed geometry_reject");
        }
    } else {
        g_cal.mount_valid = 0U;
        char buf[64]{};
        snprintf(buf, sizeof(buf), "WARN mount_matrix_skipped mag_samples=%lu",
                 static_cast<unsigned long>(mag_samples));
        serialReply(buf);
    }

    if (calSave()) {
        char buf[128]{};
        snprintf(buf, sizeof(buf),
                 "OK cal_complete gyro_cal=1 accel_cal=1 mount_valid=%u mag_cal=%u",
                 static_cast<unsigned>(g_cal.mount_valid),
                 static_cast<unsigned>(g_cal.mag_valid));
        serialReply(buf);
        waiting_for_mag_cal_ = false;
    } else {
        serialReply("ERR nvs_save_failed cal_in_ram_only");
    }
}

// ---------------------------------------------------------------------------
// checkAutoRecal() — periodic watchdog for auto-recal timer
// ---------------------------------------------------------------------------

void Application::checkAutoRecal(uint32_t now_ms) {
    // The IIR update itself happens in runFilter() when still.
    // This function is a placeholder for future periodic checks (e.g., NVS save
    // after a stable recal period). Currently no-op.
    (void)now_ms;
}
