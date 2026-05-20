# Orientation System — Build Context

## Hardware
- ESP32 (PlatformIO, Arduino framework, esp32dev board)
- MPU6500 accel+gyro, I2C auto-detect address 0x68 or 0x69, WHO_AM_I = 0x70 or 0x71
- TLV493D magnetometer, I2C auto-detect 0x1F or 0x5E (general-call reset required at power-up)
- Host: Jetson Orin Nano, Python 3.10+
- UART: 115200 baud, /dev/ttyUSB0
- I2C: SDA=GPIO21, SCL=GPIO22, 400 kHz, external 4.7 kΩ pull-ups required

## Coordinate Frame (ENU — East North Up)
- Both sensors physically aligned to ENU. NO software axis remapping needed.
- X = East, Y = North, Z = Up (Zenith)
- Yaw: 0–360 deg, clockwise from North (compass bearing convention)
- Pitch: -90 to +90 deg (nose up = positive)
- Roll: -180 to +180 deg (right side down = positive)
- Internal representation: quaternion float32. Euler only at output stage, ZYX order.

## Binary Packet Format (17 bytes, little-endian)
```
Byte 0:     START    uint8   Fixed 0xAA
Byte 1:     SEQ      uint8   Sequence 0-255 wrapping
Bytes 2-5:  YAW      float32 0.0–360.0 deg
Bytes 6-9:  PITCH    float32 -90.0–+90.0 deg
Bytes 10-13: ROLL    float32 -180.0–+180.0 deg
Byte 14:    FLAGS    uint8   Status flags
Bytes 15-16: CRC16  uint16  CRC-16/CCITT-FALSE over bytes 0-14 inclusive, little-endian
```

## FLAGS byte bits
```
Bit 0: MAG_CAL_VALID    (mag calibration loaded and applied)
Bit 1: IMU_CAL_VALID    (gyro + accel biases loaded)
Bit 2: FILTER_CONVERGED (filter ran >= 2 seconds)
Bit 3: STATIC           (stillness streak active)
Bits 4-7: RESERVED = 0
```

## CRC-16/CCITT-FALSE
```
poly=0x1021, init=0xFFFF, no reflection
Computed over bytes 0-14 (NOT including the CRC bytes themselves)
```

## Boot Handshake
- After successful init: ESP32 sends `READY\n`
- After failed init: ESP32 sends `ERROR:<reason>\n`
- Host must wait up to 10 seconds for READY

## ASCII Command Set (newline terminated, sent by host)
```
CMD:STATUS     → OK state=<state> cal=<flags> uptime=<ms>
CMD:CAL_MAG    → OK / ERR (start mag calibration collection)
CMD:CAL_MAG_STOP → OK coverage=<pct> / ERR (stop + ellipsoid fit)
CMD:CAL_IMU    → OK / ERR (blocking 3-second gyro bias cal)
CMD:SAVE_CAL   → OK / ERR (write g_cal to NVS)
CMD:LOAD_CAL   → OK / ERR (reload from NVS)
CMD:CLEAR_CAL  → OK (erase NVS, reset to defaults)
CMD:RESET      → (reboots via esp_restart())
CMD:PING       → OK (sanity check)
CMD:GET_ORIENTATION → 17-byte binary packet (one shot; filter runs in background)
```

Binary orientation packets are **on-demand only** (no continuous 100 Hz stream).
During CMD:CAL_MAG collection, ASCII CAL_PROG lines are streamed (no binary packets unless host sends GET_ORIENTATION).
CAL_PROG format: `CAL_PROG samples=NNN coverage=NN% next_axis=X\n`

## NVS Calibration Blob
```
Namespace: "orient_cal", Key: "cal_blob"
magic:      0xCA11BA11
version:    1
CRC32:      FNV-1a over entire blob with crc32 field = 0
Declination: -0.8 deg (Bengaluru, India) — kMagDeclinationDeg constant
```

## Madgwick Filter
- kBetaStill = 0.033f (when still)
- kBetaMove  = 0.1f  (when moving)
- IMU update: 200 Hz (5 ms period)
- Filter + packet output: 100 Hz (10 ms period)
- filter_ticks > (kFilterHz * 2) → FILTER_CONVERGED flag set

## Calibration Constants
- kMagMinSamples = 200 (minimum samples before ellipsoid fit)
- kMagSphereBins = 72 (12 azimuth × 6 elevation)
- kMagMinCoveragePct = 60 (minimum coverage before fit allowed)
- kAutoRecalIntervalMs = 1800000 (30 min between gyro auto-recal)
- kGyroStillRingN = 200 (samples in stillness ring buffer)
- kGyroStillThreshRs = 0.01f (gyro std-dev threshold for still)

## Compile Rules
- -Werror -Wdouble-promotion: NO double in filter or calibration math, EVER
- No Serial.print(float): ALL float formatting goes through snprintf()
- No magic numbers: all constants in src/core/Config.h
- NVS save NEVER called from filter loop — only from command handlers
- Quaternion norm checked after every filter step

## Project Layout
```
firmware/
  platformio.ini
  CLAUDE.md
  src/
    main.cpp
    core/       Config.h  Error.h/.cpp  Clock.h  SystemState.h/.cpp
    drivers/    HalI2c.h/.cpp  DrvMpu6500.h/.cpp  DrvTlv493d.h/.cpp
    fusion/     Madgwick.h/.cpp
    calibration/ NvsCal.h/.cpp  CalMag.h/.cpp  CalImu.h/.cpp
    comms/      UartTransport.h/.cpp  CommandParser.h/.cpp  PacketWriter.h/.cpp
    app/        Application.h/.cpp
host/
  orientation_host/  __init__.py  serial_handler.py  packet.py  cli.py
  tests/  test_packet.py  test_crc.py
  requirements.txt
```

## Build
```bash
/home/head-node-1/.venv-pio/bin/pio run
/home/head-node-1/.venv-pio/bin/pio run -t upload
```

## Validation Pass Criteria
- Phase 1: Non-zero values all 6 IMU axes. accel_z ≈ 1.0g flat.
- Phase 2: READY received. pitch<2deg roll<2deg flat. Yaw changes on rotation.
- Phase 3: 20 packets parsed, all CRC valid, seq increments without gaps.
- Phase 4: CMD:STATUS → OK, unknown command → ERR.
- Phase 5: coverage>=60% before fit. NVS survives reboot.
- Phase 6: static detection visible in STATUS after 3s hold-still.
- Phase 7: All flags correct. Yaw stable pointing North.
