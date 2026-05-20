# 004 — NVS CRC mismatch (root cause)

## symptom
`CMD:SAVE_CAL` OK, `CMD:LOAD_CAL` ERR; DBG `crc stored!=computed`

## cause
`calBlobFinalizeForStore()` set `saved_at_ms` **after** `computeCrc()`, so flash blob included `saved_at_ms` bytes not covered by stored CRC.

## fix
Set `saved_at_ms` before zeroing/computing `crc32` in `NvsCal.cpp`.

## verify
LOAD OK same session; reboot `mag_cal=1`, `state=IDLE`.
