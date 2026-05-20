# rules.md — agent conventions

## read order
1. `CONTEXT.md` — current snapshot
2. `CLAUDE.md` — spec/constants
3. `MASTER_PLAN.md` — step sequence
4. `changes/` + `errors/` — history

## code workflow
- edit `templates/` first → copy/sync to `firmware/`
- never change `Config.h` constants; fix callers
- PIO: `/home/head-node-1/.venv-pio/bin/pio`
- `-Werror`; Arduino core uses `-Wno-error=conversion|shadow|float-conversion|sign-conversion`
- no `Serial.print(float)`; use `snprintf`
- no double in filter/cal math

## architecture (current)
- filter: 100 Hz background (IMU 200 Hz)
- packets: **on-demand** via `CMD:GET_ORIENTATION` only
- ASCII: `Serial.println` for READY/CAL_PROG/commands
- NVS: `orient_cal` / `cal_blob`; `calSave()` via `calBlobFinalizeForStore()`

## validation
- `validate_serial.py`: phases 1–4, DTR reset, 100× GET_ORIENTATION phase 3
- `validate_cal.py`: step 10 (60s figure-8, IMU still, NVS reboot check)
- device flat for phase 2: |pitch|,|roll| < 2°

## docs convention
- terse bullets; tables OK; min prose
- `changes/NNN-short-name.md`: path | action | reason
- `errors/NNN-short-name.md`: symptom | cause | fix
- update `CONTEXT.md` after major milestones

## user rules
- no git commit unless asked
- real shell + hardware; port `/dev/ttyUSB0`
- report `=== STEP N COMPLETE ===` per MASTER_PLAN step
