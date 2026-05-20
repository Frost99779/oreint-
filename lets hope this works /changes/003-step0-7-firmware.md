# 003 — steps 0–7 firmware tree

| path | reason |
|------|--------|
| firmware/** | MASTER_PLAN scaffold + layer copies |
| firmware/src/core/* | templates/core + Config.h |
| firmware/src/drivers/* | refactor drivers + Config symbol renames |
| firmware/src/fusion/Madgwick.* | refactor + kEnableConfBeta runtime |
| firmware/src/calibration/* | templates |
| firmware/src/comms/* | templates |
| firmware/src/app/*, main.cpp | templates |

build: SUCCESS RAM 8.0% Flash 23.7% (310077 B)
