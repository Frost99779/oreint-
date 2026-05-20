# CONTEXT.md — agent snapshot (2026-05-20)

> Read first. Terse facts only. Spec detail: `CLAUDE.md`. Steps: `MASTER_PLAN.md`.

## done
| area | status |
|------|--------|
| MASTER_PLAN steps 0–8, 11 | ✅ |
| Phases 1–3 integration (NVS, state, READY, validate_serial, host) | ✅ |
| On-demand packets `CMD:GET_ORIENTATION` | ✅ flashed |
| Board | replaced ESP32; boots OK |
| Host pytest | 5 pass (`host/.venv`) |

## open
| area | status |
|------|--------|
| MASTER_PLAN step 9 | rerun `validate_serial.py` device **flat** (FILTER <2°) |
| MASTER_PLAN step 10 | `validate_cal.py` — mag figure-8 + IMU still |
| MASTER_PLAN step 12 | formal E2E report |
| HANDOFF.md | stale sections — trust this file |

## runtime (verified)
- port: `/dev/ttyUSB0` @ 115200
- boot: `READY fw=orientation-v1.0.0 mag=yes cal=loaded` (if NVS has cal)
- `CMD:STATUS`: `state=IDLE` (was BOOT — fixed)
- no continuous 0xAA stream; poll `CMD:GET_ORIENTATION`
- NVS: `LOAD_CAL` OK after fix; reboot `mag_cal=1`

## paths
| path | role |
|------|------|
| `templates/` | source of truth — edit here first |
| `firmware/` | PlatformIO build target — sync from templates |
| `host/` | Python package + pytest |
| `validate_serial.py` | phases 1–4 (on-demand) |
| `validate_cal.py` | step 10 phases 5–6 |
| `changes/` | per-change log (why + path) |
| `errors/` | failures + fixes |

## build
```
pio: /home/head-node-1/.venv-pio/bin/pio
RAM 8.0% (26308)  Flash 23.7% (~310149 B)
```

## key fixes (do not revert)
1. `NvsCal.cpp`: set `saved_at_ms` **before** `computeCrc()` — was NVS load fail
2. `SystemState.cpp`: `BOOT` → `IDLE` allowed
3. `Application.cpp`: READY `cal=loaded` if gyro_valid or mag_valid

## cmds (host)
```
CMD:GET_ORIENTATION   → 17-byte packet
CMD:STATUS / PING / CAL_* / SAVE_CAL / LOAD_CAL / CLEAR_CAL
```

## validate
```bash
python3 validate_serial.py --port /dev/ttyUSB0 --baud 115200
python3 validate_cal.py --port /dev/ttyUSB0
cd host && .venv/bin/python -m orientation_host.cli --port /dev/ttyUSB0 --poll-rate 10
```

## resolved (historical)
- bad ESP32: brownout/Guru — **board replaced**
- see `errors/001`–`004`
