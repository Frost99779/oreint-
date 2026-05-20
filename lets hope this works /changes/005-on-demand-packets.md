# 005 — on-demand orientation packets

| file | change |
|------|--------|
| Application.cpp | remove periodic `sendPacket` from `runFilter`; add `handleCmdGetOrientation` |
| Application.h | declare `handleCmdGetOrientation` |
| CommandParser.h/cpp | `GET_ORIENTATION` / `CMD:GET_ORIENTATION` |

filter still runs 100 Hz; binary packet only on `CMD:GET_ORIENTATION`.
