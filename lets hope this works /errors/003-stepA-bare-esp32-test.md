# 003 — Step A bare ESP32 serial test

## when
2026-05-20 agent run, /dev/ttyUSB0 115200
- run1: 12s after DTR reset
- run2 (redo): 15s after DTR reset — same result

## result
- READY: **no**
- brownout: **yes** (`Brownout detector was triggered`)
- Guru IllegalInstruction: **yes** (boot loop)
- binary packets 0xAA: 0

## implication
Per HANDOFF diagnostic tree: **USB cable/port underpowered even for core ESP32** OR sensors still attached (operator must confirm disconnect).

## next
- Option 1 powered USB hub OR Option 2 external PSU (HANDOFF.md)
- do **not** rely on decoupling caps alone until Step A PASS with sensors off
