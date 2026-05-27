#pragma once
#include <stdint.h>

// NVS namespace: "orient_cal", key: "cal_blob"
// Magic: 0xCA11BA11, version: 1
// CRC32 (FNV-1a) computed over entire blob with crc32 field = 0.

struct CalBlob {
    uint32_t magic;       // 0xCA11BA11
    uint16_t version;     // schema version — reject mismatches
    uint16_t size;        // sizeof(CalBlob) — size guard
    uint32_t crc32;       // FNV-1a over entire blob with this field = 0

    uint8_t  mag_valid;   // 1 = mag calibration usable
    uint8_t  accel_valid; // 1 = accel bias usable
    uint8_t  gyro_valid;  // 1 = gyro bias usable
    uint8_t  _pad;        // alignment padding — must stay zero

    float mag_offset[3];  // hard iron offset (mT), XYZ
    float mag_softmat[9]; // 3×3 soft iron matrix, row-major (diagonal for now)
    float accel_bias[3];  // accel bias (g), XYZ
    float gyro_bias[3];   // gyro bias (rad/s), XYZ
    uint32_t saved_at_ms; // millis() at last save (for diagnostics only — not freshness gate)

    float    mount_R[9];      // row-major: sensor axis → ENU (East,North,Up) components
    uint8_t  mount_valid;     // 1 = mount_R computed during CAL_IMU
    uint8_t  _pad_mount[3];   // alignment padding — must stay zero

    float    heading_offset_deg;  // subtracted from yaw at output
    uint8_t  heading_offset_valid;
    uint8_t  _pad_hdg[3];
};

// Global calibration state — written by calLoad()/calSave(), read by drivers and filter.
extern CalBlob g_cal;

void calResetDefaults();
bool calLoad();
bool calSave();
bool calClear();
