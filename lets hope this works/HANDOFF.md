# HANDOFF — pointer (2026-05-20)

**Authoritative state: `CONTEXT.md`** (this file was partially stale.)

## quick status
- Firmware built, flashed, boots `READY` on **new ESP32**
- Architecture: on-demand `CMD:GET_ORIENTATION` (no 10 Hz stream)
- NVS + state machine + READY string: **fixed** (see `errors/004`, `changes/006`)
- Next: `validate_serial.py` (flat mount) → `validate_cal.py` (step 10) → step 12 report

## do not redo
- full template tree under `templates/`
- `firmware/` scaffold from MASTER_PLAN 0–7
- host package + `validate_serial.py` / `validate_cal.py` rewrites

## read for detail
| file | use |
|------|-----|
| `CONTEXT.md` | what works / what's open |
| `MASTER_PLAN.md` | numbered steps |
| `CLAUDE.md` | protocol spec |
| `rules.md` | agent conventions |

## hardware note (resolved)
Old board: brownout/Guru. **Replaced.** Sensors on I2C GPIO21/22, 4.7k pull-ups.
