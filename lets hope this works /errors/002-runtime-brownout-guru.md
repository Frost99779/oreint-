# 002 — runtime boot loop (steps 8–9)

## symptom
- validate_serial: READY not in 10s
- serial: `Brownout detector was triggered` (first flash)
- after sdkconfig `CONFIG_ESP32_BROWNOUT_DET=0`: Guru `IllegalInstruction` in `heap_tlsf.c` at CPU0 start — reverted
- after erase+reflash: Guru persists, no READY, 0xAA packets

## tried
- flash upload OK (310448 B)
- revert sdkconfig brownout override + clean build
- `pio run -t erase` + re-upload
- removed `esp_task_wdt_deinit()` (caused early panic when combined with bad sdkconfig)

## likely cause
- USB/host power insufficient for ESP32 + IMU + mag (brownout)
- or hardware/wiring fault (I2C short, strap pins)

## next for operator
- powered USB hub / external 3.3V reg
- measure 3.3V under load
- disconnect sensors, test ESP32 alone with minimal sketch
- when READY stable: rerun `validate_serial.py --port /dev/ttyUSB0`
