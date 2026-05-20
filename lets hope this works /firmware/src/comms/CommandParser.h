#pragma once
#include <stdint.h>

namespace comms {

enum class CmdId : uint8_t {
    UNKNOWN = 0,
    STATUS,
    CAL_MAG,
    CAL_MAG_STOP,
    CAL_IMU,
    SAVE_CAL,
    LOAD_CAL,
    CLEAR_CAL,
    RESET,
    PING,
    GET_ORIENTATION,
};

// Parse a null-terminated ASCII line (without newline).
// Returns CmdId::UNKNOWN if unrecognised.
CmdId parseCommand(const char* line);

}  // namespace comms
