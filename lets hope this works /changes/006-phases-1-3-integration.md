# 006 — Phases 1–3 integration

## Phase 1 firmware
| file | fix |
|------|-----|
| NvsCal.cpp | CRC after `saved_at_ms`; readOnly load; `calBlobFinalizeForStore` |
| SystemState.cpp | BOOT → IDLE allowed |
| Application.cpp | READY `cal=` if gyro OR mag valid |

## Phase 2 validate_serial.py
- Phases 1–3 use `CMD:GET_ORIENTATION` (100 requests phase 3)
- Phase 56 delegates to `validate_cal.py`

## Phase 3 host
- serial_handler: poll `GET_ORIENTATION` @ 10 Hz, `latest_data`
- cli: `--poll-rate`, displays `latest_data`
