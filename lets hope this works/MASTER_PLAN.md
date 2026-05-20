# MASTER EXECUTION PLAN
# ESP32 + MPU6500 + TLV493D → Orientation System
# Executor: Claude Sonnet 4.6 (autonomous)
# Target directory: /home/head-node-1/Desktop/lets hope this works /firmware/
# After EVERY numbered step: print "=== STEP N COMPLETE ===" to user before continuing.

---

## RULES FOR EXECUTION

1. Read build output completely before acting. One warning missed = build fails later.
2. Fix ALL -Werror warnings before proceeding to the next step.
3. If `pio run` fails: read error, fix, retry. Maximum 5 retries per step. If still failing after 5, report blocker.
4. Never skip a step. Steps are ordered by dependency.
5. All `pio` commands use: `/home/head-node-1/.venv-pio/bin/pio`
6. Report "=== STEP N COMPLETE ===" to user at end of each numbered step.
7. For serial validation: the Python script uses a 15-second capture window. If device is not connected, report "DEVICE NOT CONNECTED - skip validation" and continue with next step.
8. Do not modify the spec constants in Config.h. If a compilation error involves a constant value, fix the code, not the constant.
9. If a file already exists, overwrite it without asking.

---

## PREREQUISITES CHECK (do this before Step 0)

```bash
# Verify PIO
/home/head-node-1/.venv-pio/bin/pio --version

# Verify TLV493D library
ls /home/head-node-1/Arduino/libraries/tlv493d_driver_pantilt/

# Verify serial port
ls /dev/ttyUSB* 2>/dev/null || ls /dev/ttyACM* 2>/dev/null || echo "NO_SERIAL_PORT"

# Python serial library
python3 -c "import serial; print('pyserial OK')" 2>/dev/null || pip3 install pyserial --quiet
```

If PIO or the library is missing, stop and report. Do not continue.

---

## STEP 0: Create firmware project directory

```bash
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/src/core"
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers"
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/src/fusion"
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/src/calibration"
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/src/comms"
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/src/app"
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/include"
mkdir -p "/home/head-node-1/Desktop/lets hope this works /firmware/lib"
```

Copy CLAUDE.md and platformio.ini from templates:
```bash
cp "/home/head-node-1/Desktop/lets hope this works /CLAUDE.md" \
   "/home/head-node-1/Desktop/lets hope this works /firmware/CLAUDE.md"
cp "/home/head-node-1/Desktop/lets hope this works /templates/platformio.ini" \
   "/home/head-node-1/Desktop/lets hope this works /firmware/platformio.ini"
```

Run empty build to verify PIO project structure:
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1 | tail -20
```

Expected: Will fail (no source files yet) but should show PIO initializing toolchain, NOT a PIO config error.
If it shows "Error: Unknown board ID" or similar config error, check platformio.ini.

=== STEP 0 COMPLETE ===

---

## STEP 1: Core layer

Copy Error, Clock, SystemState from `final` (these are correct and tested):

```bash
FINAL=/home/head-node-1/Desktop/final/src/core
FW="/home/head-node-1/Desktop/lets hope this works /firmware/src/core"

cp "$FINAL/Error.h"        "$FW/Error.h"
cp "$FINAL/Error.cpp"      "$FW/Error.cpp"
cp "$FINAL/Clock.h"        "$FW/Clock.h"
cp "$FINAL/SystemState.h"  "$FW/SystemState.h"
cp "$FINAL/SystemState.cpp" "$FW/SystemState.cpp"
```

**Then edit the copied files to remove dependency on FixedQueue and EventDispatcher** (those are in comms now):
- In `Error.h`: remove any include of `FixedQueue.h` or `EventDispatcher.h` if present.
- In `SystemState.cpp`: verify includes — only needs `SystemState.h` and `Error.h`.

Copy Config.h from template (has updated constants for this spec):
```bash
cp "/home/head-node-1/Desktop/lets hope this works /templates/core/Config.h" \
   "/home/head-node-1/Desktop/lets hope this works /firmware/src/core/Config.h"
```

Run build (will still fail — no main.cpp yet, but no compile errors in core files):
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1 | grep -E "(error:|warning:|Success)" | head -30
```

Fix any compile errors in the core files before proceeding.

=== STEP 1 COMPLETE ===

---

## STEP 2: HAL drivers (verbatim copies from refactor)

The refactor's drivers are correct for MPU6500 + TLV493D and tested on hardware.
Copy them verbatim — include paths work because platformio.ini has `-I` flags for all src subdirs.

```bash
REF=/home/head-node-1/Desktop/pantilt/MPU6500_TLV493D_Fusion_refactor/main
FW="/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers"

cp "$REF/hal_i2c.h"       "$FW/HalI2c.h"
cp "$REF/hal_i2c.cpp"     "$FW/HalI2c.cpp"
cp "$REF/drv_mpu6500.h"   "$FW/DrvMpu6500.h"
cp "$REF/drv_mpu6500.cpp" "$FW/DrvMpu6500.cpp"
cp "$REF/drv_tlv493d.h"   "$FW/DrvTlv493d.h"
cp "$REF/drv_tlv493d.cpp" "$FW/DrvTlv493d.cpp"
```

After copying, update includes in the driver files:
- Replace `#include "config.h"` with `#include "Config.h"` (capital C, same include path due to -I flags)
- Replace `#include "hal_i2c.h"` with `#include "HalI2c.h"`
- All other includes stay the same

```bash
sed -i 's/#include "config.h"/#include "Config.h"/g' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers/DrvMpu6500.cpp"
sed -i 's/#include "config.h"/#include "Config.h"/g' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers/DrvTlv493d.cpp"
sed -i 's/#include "hal_i2c.h"/#include "HalI2c.h"/g' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers/DrvMpu6500.cpp"
sed -i 's/#include "hal_i2c.h"/#include "HalI2c.h"/g' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers/DrvTlv493d.cpp"
sed -i 's/#include "config.h"/#include "Config.h"/g' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers/HalI2c.cpp"
```

**IMPORTANT**: In `HalI2c.cpp`, the refactor uses `I2C_HZ` from config.h.
In our new Config.h this is renamed to `kI2cClockHz`. Update the reference:
```bash
sed -i 's/Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ)/Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz)/' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/drivers/HalI2c.cpp"
```

Also add `Wire.setTimeOut(kI2cTimeoutMs)` after `Wire.begin()` in `HalI2c.cpp`:
- Read the file, find the `halI2cBegin()` function, add `Wire.setTimeOut(kI2cTimeoutMs);` on the line after `Wire.begin(...)`.

Build test:
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1 | grep -E "(error:|warning:)" | head -20
```

Fix any constant-name mismatches between Config.h and the driver files.
Common issue: `GYRO_RANGE_REG`, `DLPF_CFG_REG`, `GYRO_SCALE`, `TLV493D_ADDR_CANDIDATES`, `TLV493D_ADDR_N` — these are all defined in Config.h template. Verify they exist.

=== STEP 2 COMPLETE ===

---

## STEP 3: Fusion (verbatim copy from refactor)

```bash
REF=/home/head-node-1/Desktop/pantilt/MPU6500_TLV493D_Fusion_refactor/main
FW="/home/head-node-1/Desktop/lets hope this works /firmware/src/fusion"

cp "$REF/fusion_madgwick.h"   "$FW/Madgwick.h"
cp "$REF/fusion_madgwick.cpp" "$FW/Madgwick.cpp"
```

Update includes:
```bash
sed -i 's/#include "fusion_madgwick.h"/#include "Madgwick.h"/' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/fusion/Madgwick.cpp"
sed -i 's/#include "config.h"/#include "Config.h"/' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/fusion/Madgwick.cpp"
```

The Madgwick.h guard `#ifndef PANTILT_FUSION_MADGWICK_H` should be updated:
```bash
sed -i 's/PANTILT_FUSION_MADGWICK_H/ORIENTATION_MADGWICK_H/g' \
  "/home/head-node-1/Desktop/lets hope this works /firmware/src/fusion/Madgwick.h"
```

The `betaFromConf()` function in Madgwick.cpp references `ENABLE_CONF_BETA`, `BETA_STILL`, `BETA_MOVE`.
These are defined in Config.h template. Verify they match.

Also note: `betaFromConf()` uses `ENABLE_CONF_BETA` preprocessor define — in our Config.h this is a
constexpr bool `kEnableConfBeta`. Change the #if to an if():
Read Madgwick.cpp, find `#if !ENABLE_CONF_BETA`, replace with:
```cpp
if (!kEnableConfBeta) {
    (void)useMag; (void)confA; (void)confM;
    return still ? kBetaStill : kBetaMove;
} else {
```
And replace the `#else` / `#endif` accordingly.

Build test:
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1 | grep -E "(error:|warning:)" | head -20
```

=== STEP 3 COMPLETE ===

---

## STEP 4: Calibration layer

Copy all calibration templates:

```bash
TMP="/home/head-node-1/Desktop/lets hope this works /templates/calibration"
FW="/home/head-node-1/Desktop/lets hope this works /firmware/src/calibration"

cp "$TMP/NvsCal.h"   "$FW/NvsCal.h"
cp "$TMP/NvsCal.cpp" "$FW/NvsCal.cpp"
cp "$TMP/CalMag.h"   "$FW/CalMag.h"
cp "$TMP/CalMag.cpp" "$FW/CalMag.cpp"
cp "$TMP/CalImu.h"   "$FW/CalImu.h"
cp "$TMP/CalImu.cpp" "$FW/CalImu.cpp"
```

Build test:
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1 | grep -E "(error:|warning:)" | head -30
```

Common issues and fixes:
- `undefined reference to 'g_cal'`: g_cal is defined in NvsCal.cpp as `CalBlob g_cal{}`. If missing, add it.
- Mismatched constant names: check against Config.h — all calibration constants start with `k` in Config.h.
- `-Wdouble-promotion`: any `sqrtf()` / `fabsf()` that uses `double` literal (e.g. `0.5` instead of `0.5f`).
  Fix: add `f` suffix to all floating constants in calibration math.

=== STEP 4 COMPLETE ===

---

## STEP 5: Communications layer

Copy comms templates:
```bash
TMP="/home/head-node-1/Desktop/lets hope this works /templates/comms"
FW="/home/head-node-1/Desktop/lets hope this works /firmware/src/comms"

cp "$TMP/UartTransport.h"   "$FW/UartTransport.h"
cp "$TMP/UartTransport.cpp" "$FW/UartTransport.cpp"
cp "$TMP/CommandParser.h"   "$FW/CommandParser.h"
cp "$TMP/CommandParser.cpp" "$FW/CommandParser.cpp"
cp "$TMP/PacketWriter.h"    "$FW/PacketWriter.h"
cp "$TMP/PacketWriter.cpp"  "$FW/PacketWriter.cpp"
```

Build test:
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1 | grep -E "(error:|warning:)" | head -30
```

=== STEP 5 COMPLETE ===

---

## STEP 6: Application layer

```bash
TMP="/home/head-node-1/Desktop/lets hope this works /templates/app"
FW_APP="/home/head-node-1/Desktop/lets hope this works /firmware/src/app"
FW_SRC="/home/head-node-1/Desktop/lets hope this works /firmware/src"

cp "$TMP/Application.h"   "$FW_APP/Application.h"
cp "$TMP/Application.cpp" "$FW_APP/Application.cpp"
cp "$TMP/main.cpp"        "$FW_SRC/main.cpp"
```

Build test:
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1 | grep -E "(error:|warning:)" | head -30
```

=== STEP 6 COMPLETE ===

---

## STEP 7: Full build — zero warnings required

```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run 2>&1
```

**Expected success output** (exact strings):
```
[SUCCESS]
RAM:  XX.X% (NNNNN/327680 bytes)
Flash: XX.X% (NNNNNN/1310720 bytes)
```

**If any warnings remain** (all treated as errors by -Werror):
- `-Wdouble-promotion`: find the line, add `f` suffix to all float literals on that line.
- `-Wconversion`: cast explicitly, e.g. `static_cast<uint8_t>(...)`.
- `-Wshadow`: rename the inner variable.
- `-Wunused-parameter`: add `(void)param;` at top of function body.
- `-Wmissing-field-initializers`: add `{}` to struct/array initializer.

Repeat build until zero errors and `[SUCCESS]` appears.

Report RAM and Flash usage to user.

=== STEP 7 COMPLETE ===

---

## STEP 8: Flash to device

Check serial port first:
```bash
ls /dev/ttyUSB* 2>/dev/null || ls /dev/ttyACM* 2>/dev/null || echo "NO_PORT"
```

If NO_PORT: report "Device not connected" and skip Steps 8-11. Continue to Step 12 (host package).

Flash:
```bash
cd "/home/head-node-1/Desktop/lets hope this works /firmware" && \
  /home/head-node-1/.venv-pio/bin/pio run -t upload 2>&1 | tail -20
```

Expected:
```
Writing at 0x... (100 %)
Hash of data verified.
```

If upload fails:
- "No serial data": press BOOT button on ESP32, retry.
- "Permission denied /dev/ttyUSB0": `sudo chmod 666 /dev/ttyUSB0`, retry.
- Baud error: check platformio.ini upload_speed (should be 115200).

=== STEP 8 COMPLETE ===

---

## STEP 9: Phase 1-4 serial validation

Run the validation script (reads serial for 15 seconds, checks all phases):

```bash
python3 "/home/head-node-1/Desktop/lets hope this works /validate_serial.py" \
  --port $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1) \
  --baud 115200
```

**Phase 1 pass**: "SENSOR_READ_OK: ax=N ay=N az≈1.0 gx=N gy=N gz=N mx=N my=N mz=N"
**Phase 2 pass**: "FILTER_OK: READY received, pitch<2deg roll<2deg yaw=N"
**Phase 3 pass**: "PACKETS_OK: 20 packets parsed, all CRC valid, seq increments"
**Phase 4 pass**: "COMMANDS_OK: CMD:STATUS→OK, unknown→ERR"

If any phase fails:
- Re-read the failure message carefully.
- If "READY not received": boot sequence failed. Check serial for ERROR: message. Fix the init sequence in Application::begin().
- If "SENSOR_READ_FAIL": I2C problem. Check that pull-up resistors are on SDA/SCL. Verify WHO_AM_I read in boot.
- If "CRC_FAIL": PacketWriter CRC computation error. Verify CRC16 algorithm matches spec exactly.
- If "CMD_FAIL": CommandParser not matching CMD: prefix. Check CommandParser::identify().
- After each firmware fix: go back to Step 7 (full build) then Step 8 (flash) then retry Step 9.

=== STEP 9 COMPLETE ===

---

## STEP 10: Phase 5-6 calibration validation

These phases require physical device interaction.
Report to user:
```
=== CALIBRATION VALIDATION REQUIRED ===
Phase 5: Send CMD:CAL_MAG, tumble device in figure-8 for 60 seconds, send CMD:CAL_MAG_STOP.
         Expected: CAL_PROG lines appear with coverage increasing to >=60%. calSave returns OK.
Phase 6: Hold device still for 5 seconds after CMD:CAL_IMU.
         Expected: static detection active in CMD:STATUS output.
Run validate_serial.py --phase 56 to check.
```

If user confirms physical access:
```bash
python3 "/home/head-node-1/Desktop/lets hope this works /validate_serial.py" \
  --port $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1) \
  --baud 115200 --phase 56
```

=== STEP 10 COMPLETE ===

---

## STEP 11: Host Python package

Copy host templates:
```bash
TMP="/home/head-node-1/Desktop/lets hope this works /host"
HOST="/home/head-node-1/Desktop/lets hope this works /firmware/../host"

cp -r "$TMP/orientation_host" "/home/head-node-1/Desktop/lets hope this works /host/"
cp -r "$TMP/tests"           "/home/head-node-1/Desktop/lets hope this works /host/"
cp    "$TMP/requirements.txt" "/home/head-node-1/Desktop/lets hope this works /host/requirements.txt"
```

Run host unit tests (no device needed):
```bash
cd "/home/head-node-1/Desktop/lets hope this works /host" && \
  python3 -m pytest tests/ -v 2>&1
```

Expected: `2 passed` (test_packet.py and test_crc.py).

If tests fail:
- `test_crc.py` fail: CRC16 Python implementation doesn't match ESP32. Fix Python CRC to match spec exactly.
- `test_packet.py` fail: struct unpack format error. Verify `<f` (little-endian float) parsing.

=== STEP 11 COMPLETE ===

---

## STEP 12: Final integration verification

Report summary to user:
- Firmware binary size
- All validation phases that passed
- Any phases that were skipped (no device) or require physical testing
- Host test results
- Next steps if any phases are blocked

=== STEP 12 COMPLETE — BUILD FINISHED ===

---

## ERROR RECOVERY GUIDE

### Common compile errors and fixes

| Error | Cause | Fix |
|---|---|---|
| `'kBetaStill' was not declared` | Config namespace not in scope | Add `using namespace core::config;` at top of .cpp or prefix with `core::config::` |
| `undefined reference to 'calSave'` | NvsCal.cpp not compiled | Check platformio.ini src_filter — it should be `+<src/**>` |
| `implicit conversion from 'double' to 'float'` | -Wdouble-promotion | Add `f` suffix to all float literals: `0.5f` not `0.5` |
| `'Wire' was not declared` | Missing `#include <Wire.h>` | Add to top of HalI2c.cpp |
| `template argument deduction failed` | C++11 vs C++17 | Check build_unflags removes gnu++11 |
| `'esp_task_wdt_add' was not declared` | Missing WDT include | Add `#include <esp_task_wdt.h>` to Application.cpp |
| `error: format '%lu' expects argument of type 'long unsigned int'` | snprintf format | Cast to `static_cast<unsigned long>(val)` |

### Common runtime failures and fixes

| Symptom | Cause | Fix |
|---|---|---|
| `ERROR:mpu_init` on boot | MPU6500 not detected | Check I2C wiring, pull-ups, WHO_AM_I addresses 0x68/0x69 |
| No READY after boot | MPU init failed silently | Add Serial.flush() after ERROR print; check beginAuto() return |
| All zeros from MPU | I2C address wrong or no pull-ups | Try i2cScan(); verify SDA=21, SCL=22; 4.7kΩ pull-ups |
| CRC validation fails | Byte order wrong | Verify CRC is little-endian in bytes 15-16; verify over bytes 0-14 NOT 0-16 |
| Yaw jumps | No mag calibration loaded | Send CMD:CAL_MAG, tumble device, CMD:CAL_MAG_STOP |
| Pitch/roll wrong | Axis mapping wrong | Check ENU mapping in Config.h; verify sensor mounted X=East, Y=North, Z=Up |
| NVS not persisting | Preferences namespace wrong | Must be `orient_cal`, key `cal_blob` |
| Packets not detected by Python | Start byte not 0xAA | Check PacketWriter writes 0xAA as first byte |
