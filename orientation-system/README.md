# ESP32 Orientation System

Pan-tilt orientation tracking using MPU6500 (accel + gyro) and BMM150 (magnetometer) fused with a Madgwick AHRS filter. Outputs yaw, pitch, roll over UART to a Jetson Orin Nano host.

---

## Hardware

| Component | Part | Interface | Address |
|-----------|------|-----------|---------|
| MCU | ESP32 (esp32dev) | — | — |
| IMU | MPU6500 | I2C | 0x68 or 0x69 (auto-detected) |
| Magnetometer | BMM150 | I2C | 0x10–0x13 (auto-detected) |
| Host | Jetson Orin Nano | UART | /dev/ttyUSB0, 115200 baud |

**I2C wiring:** SDA = GPIO21, SCL = GPIO22, 400 kHz. Requires external 4.7 kΩ pull-ups to 3.3 V.

---

## Repository Layout

```
orientation-system/
  firmware/
    platformio.ini          PlatformIO build config (esp32dev, Arduino framework)
    src/
      main.cpp              setup() / loop() — instantiates Application
      core/
        Config.h            All compile-time constants (single source of truth)
        Clock.h             IClock interface + elapsed() helper
        Error.h/.cpp        ErrorCode enum + HealthStatus
        SystemState.h/.cpp  State machine (BOOT → IDLE → CALIBRATING → READY → ERROR)
      drivers/
        HalI2c.h/.cpp       Wire wrapper with timeout + auto-restart on consecutive fails
        DrvMpu6500.h/.cpp   MPU6500 init (auto address), accel/gyro read → g / dps
        DrvBmm150.h/.cpp    BMM150 init (auto address), mag read → mT, Y-axis negated
      fusion/
        Madgwick.h/.cpp     Madgwick AHRS — quaternion update + eulerDeg() output
      calibration/
        NvsCal.h/.cpp       CalBlob struct, NVS load/save/clear (FNV-1a CRC32)
        CalMag.h/.cpp       Mag calibration: outlier reject, min/max hard/soft iron, 72-bin coverage
        CalImu.h/.cpp       Gyro/accel blocking cal + IIR auto-recal + stillness detector
      comms/
        UartTransport.h/.cpp  Ring-buffer UART RX, line extraction
        CommandParser.h/.cpp  ASCII command → CmdId enum
        PacketWriter.h/.cpp   17-byte binary packet builder + CRC-16/CCITT-FALSE
        SheetsLogger.h/.cpp   Google Sheets WiFi logger (disabled by default)
      app/
        Application.h/.cpp  Top-level: scheduler, sensor reads, filter, command handlers
  host/
    esp_csv_logger.py       Polls ESP, writes /tmp/varaha_orientation (see Host Software)
    metrics_exporter.py     Prometheus exporter — serves /metrics on port 9100
```

---

## Architecture

### Firmware

```
loopOnce() runs every ~1 ms
│
├─ uart_.service()          drain UART RX into ring buffer
├─ processCommands()        parse complete lines → handleCommand()
│
├─ readSensors()  [200 Hz]  MPU6500 + BMM150 read → apply biases → push to CalImu
│
└─ runFilter()    [100 Hz]
    ├─ compute dt from micros()
    ├─ select beta (still=0.033, moving=0.10, startup boost=0.50 for first 30 s)
    ├─ applyMountRotation() on accel/gyro/mag  (Rᵀ·v using mount_R from CalBlob)
    ├─ frame remap: negate Y  (chip y=left → Madgwick y=right)
    ├─ Madgwick.update()  → quaternion
    ├─ eulerDeg() → roll, pitch, yaw
    ├─ add kMagDeclinationDeg + heading_offset
    ├─ checkConvergence()  (circular std-dev of yaw window)
    └─ IIR auto-recal when still (every 30 min)
```

### Coordinate Frames

**PCB / sensor axes (after driver, before filter):**
- Chip +X = forward (nose-down raises ax)
- Chip +Y = left
- Chip +Z = up  →  flat level reads az ≈ +1.0 g

**Madgwick body frame (after remap in Application.cpp):**
- Negate Y on accel, gyro, mag → converts left-handed Y to right-handed
- Madgwick expects gravity [0, 0, +1] when flat

**ENU calibration target (mount_R):**
- Columns of mount_R are East, North, Up unit vectors in sensor frame
- `applyMountRotation` applies Rᵀ·v to align sensor frame to ENU

**Output convention:**
- Yaw: 0–360°, clockwise from true north (after declination + heading_offset)
- Pitch: −90° to +90°, nose-up positive
- Roll: −180° to +180°, right-side-down positive

### Calibration Blob (NVS)

Stored in NVS namespace `orient_cal`, key `cal_blob`. Version 4. 140 bytes. FNV-1a CRC32.

| Field | Description |
|-------|-------------|
| `mag_offset[3]` | Hard iron offset (mT), XYZ |
| `mag_softmat[9]` | 3×3 soft iron correction matrix |
| `accel_bias[3]` | Accel bias (g), XYZ |
| `gyro_bias[3]` | Gyro bias (rad/s), XYZ |
| `mount_R[9]` | Sensor → ENU rotation matrix |
| `heading_offset_deg` | Subtracted from yaw at output (CMD:ZERO_HEADING) |

Bumping `kCalVersion` in `Config.h` invalidates all stored blobs.

---

## Binary Packet Format

17 bytes, little-endian. Sent only on-demand (`CMD:GET_ORIENTATION`).

```
Byte 0      START   0xAA
Byte 1      SEQ     uint8, wraps 0–255
Bytes 2–5   YAW     float32, 0.0–360.0 deg
Bytes 6–9   PITCH   float32, −90.0–+90.0 deg
Bytes 10–13 ROLL    float32, −180.0–+180.0 deg
Byte 14     FLAGS   uint8 (see below)
Bytes 15–16 CRC16   uint16, CRC-16/CCITT-FALSE over bytes 0–14
```

**FLAGS bits:**
```
Bit 0  MAG_CAL_VALID     mag calibration loaded
Bit 1  IMU_CAL_VALID     gyro + accel biases loaded
Bit 2  FILTER_CONVERGED  filter ran ≥ 2 s and yaw is stable
Bit 3  STATIC            stillness streak active
```

**CRC-16/CCITT-FALSE:** poly=0x1021, init=0xFFFF, no bit reflection, computed over bytes 0–14.

---

## ASCII Command Set

Send over UART, newline-terminated. All responses are newline-terminated ASCII unless noted.

| Command | Response | Notes |
|---------|----------|-------|
| `CMD:PING` | `OK` | Health check |
| `CMD:STATUS` | `OK state=... uptime=... yaw=... pitch=... roll=... ...` | Full system status |
| `CMD:RAW` | `RAW imu=1 ax=... ay=... az=... gx=... gy=... gz=... mag=1 mx=... my=... mz=...` | Post-driver, pre-bias values |
| `CMD:GET_ORIENTATION` | 17-byte binary packet | Requires IMU cal + filter converged |
| `CMD:CAL_MAG` | `OK mag_cal=started ...` | Begin mag collection — rotate device |
| `CMD:CAL_MAG_STOP` | `OK coverage=N%% ...` | Stop mag cal, triggers settle → IMU cal |
| `CMD:CAL_IMU` | `OK cal_imu=starting ...` | Blocking gyro/accel cal — hold still flat |
| `CMD:SAVE_CAL` | `OK` | Write CalBlob to NVS |
| `CMD:LOAD_CAL` | `OK` | Read CalBlob from NVS |
| `CMD:CLEAR_CAL` | `OK cal_cleared` | Erase NVS cal |
| `CMD:ZERO_HEADING` | `OK heading_zero=N.NN` | Store current yaw as offset |
| `CMD:RESET` | — | Calls esp_restart() |

### Boot Handshake

After power-on the ESP sends one of:
```
READY fw=orientation-v1.0.0 mag=yes|no cal=loaded|none
ERROR:<reason>
```
The host must wait up to 10 seconds for `READY` before sending commands.

---

## Build and Flash

Requirements: PlatformIO with espressif32 platform installed.

```bash
# Build
/home/head-node-1/.venv-pio/bin/pio run -d firmware/

# Flash (ESP32 connected via USB)
/home/head-node-1/.venv-pio/bin/pio run -d firmware/ -t upload --upload-port /dev/ttyUSB0

# Monitor serial output
/home/head-node-1/.venv-pio/bin/pio device monitor -d firmware/ --port /dev/ttyUSB0 --baud 115200
```

---

## First-Run Calibration Procedure

Do this once after a fresh flash or if you add/move metal near the sensor. Cal survives reboots indefinitely.

**Step 1 — Mag cal** (device must be free to rotate in 3D)
```
CMD:CAL_MAG
```
Rotate the device slowly through all orientations — tumble it in all axes like a figure-8 in 3D space. Watch `CAL_PROG coverage=N%%`. Cal auto-completes at 100% or you can stop it manually:
```
CMD:CAL_MAG_STOP
```
Minimum coverage for a valid cal is 60%. After stopping, the ESP automatically:
1. Waits 30 seconds for the filter to settle (prints `SETTLE remaining=...`)
2. Runs a blocking 3-second gyro/accel cal (prints `SETTLE done running_imu_cal`)
3. Computes the mount matrix from the still accel + mag means
4. Saves everything to NVS

**Step 2 — Verify**
```
CMD:STATUS
```
Confirm: `mag_cal=1 gyro_cal=1 accel_cal=1 mount_valid=1`

**Step 3 — Optional: zero heading**

Point the device at a known reference direction and send:
```
CMD:ZERO_HEADING
```
This stores the current yaw as an offset so that direction reads as 0°.

---

## Host Software

### esp_csv_logger.py

Polls `CMD:GET_ORIENTATION` at 10 Hz, verifies CRC, and atomically writes the latest values to `/tmp/varaha_orientation`:

```
yaw,pitch,roll,timestamp_unix
12.3456,1.2345,-0.5678,1748390400.123
```

The file is always a single fixed-size line — it never grows.

```bash
cd host/
pip install pyserial
python3 esp_csv_logger.py
```

Environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `ESP_PORT` | `/dev/ttyUSB0` | Serial port |
| `ESP_BAUD` | `115200` | Baud rate |
| `ESP_POLL_HZ` | `10` | Request rate |
| `ORIENTATION_PATH` | `/tmp/varaha_orientation` | Shared file path |

### metrics_exporter.py

Prometheus exporter serving `GET /metrics` on port 9100. Reads `/tmp/varaha_orientation` on every scrape and emits:

| Metric | Description |
|--------|-------------|
| `varaha_sensor_orientation_sample_age_seconds` | Seconds since last write — always emitted |
| `varaha_sensor_orientation_yaw_deg` | 0–360°, clockwise from true north |
| `varaha_sensor_orientation_pitch_deg` | −90° to +90°, nose-up positive |
| `varaha_sensor_orientation_roll_deg` | −180° to +180°, right-side-down positive |

Yaw/pitch/roll are only emitted when the shared file is fresh (age ≤ `ORIENTATION_STALE_SECONDS`, default 5 s). A stale file causes only `sample_age_seconds` to appear — preventing frozen values from looking like real data.

```bash
python3 host/metrics_exporter.py
```

---

## Key Config Constants (`firmware/src/core/Config.h`)

| Constant | Value | Description |
|----------|-------|-------------|
| `kMagDeclinationDeg` | −0.8° | Bengaluru magnetic declination — update if relocated |
| `kBetaStill` | 0.033 | Madgwick gain when stationary |
| `kBetaMove` | 0.10 | Madgwick gain when moving |
| `kBetaStartupBoost` | 0.50 | Gain for first 30 s after boot |
| `kAutoRecalIntervalMs` | 1 800 000 | IIR bias recal interval (30 min) |
| `kCalVersion` | 4 | Bump to invalidate all stored blobs |
| `kEnableSheetsLogger` | false | WiFi/Sheets logging — off by default |
| `kFilterHz` | 100 | Madgwick update rate |
| `kImuPeriodMs` | 5 | IMU read rate (200 Hz) |

---

## Drift and Recalibration Notes

- **Yaw drift:** not an issue when mag cal and mount matrix are valid. The Madgwick filter anchors yaw to Earth's magnetic field at 100 Hz continuously.
- **Pitch/roll drift:** caused by gyro bias creeping with temperature. The IIR auto-recal in `runFilter()` corrects this every 30 minutes when the device is still.
- **Mag cal validity:** distance from calibration origin is irrelevant. Cal corrects the sensor's own hard/soft iron distortions, not a location. Only invalidated by physically adding/moving metal near the sensor.
- **`kMagDeclinationDeg`:** the only location-dependent value. Update it in `Config.h` if the device moves hundreds of km.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `ERROR:mpu_init` at boot | MPU6500 not found on I2C | Check wiring, pull-ups, address jumper |
| `WARN no_mag_cal` at boot | No NVS cal blob | Run `CMD:CAL_MAG` |
| `ERR not_converged` on `CMD:GET_ORIENTATION` | Filter not settled or no IMU cal | Hold still, wait ~2 s after cal |
| `ERR cal_failed reason=2` during mag cal | Rotation range too small for BMM150 | Rotate more aggressively through all axes |
| Yaw slowly drifting | Mag not being used (`using_mag_=false`) | Check `CMD:STATUS` — `mag_cal` must be 1 and `mag=yes` |
| Pitch/roll drifting over hours | Gyro bias temperature drift | Wait for 30-min IIR recal, or send `CMD:CAL_IMU` when still |
| Bad CRC packets in logger | Framing issue or noise on UART | Check cable, reduce `ESP_POLL_HZ`, check for 3.3V/5V mismatch |
