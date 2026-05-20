#include "CommandParser.h"
#include <string.h>

namespace comms {

CmdId parseCommand(const char* line) {
    if (line == nullptr) return CmdId::UNKNOWN;
    if (strncmp(line, "CMD:", 4) != 0) return CmdId::UNKNOWN;
    const char* cmd = line + 4;
    if (strcmp(cmd, "STATUS")      == 0) return CmdId::STATUS;
    if (strcmp(cmd, "CAL_MAG")     == 0) return CmdId::CAL_MAG;
    if (strcmp(cmd, "CAL_MAG_STOP")== 0) return CmdId::CAL_MAG_STOP;
    if (strcmp(cmd, "CAL_IMU")     == 0) return CmdId::CAL_IMU;
    if (strcmp(cmd, "SAVE_CAL")    == 0) return CmdId::SAVE_CAL;
    if (strcmp(cmd, "LOAD_CAL")    == 0) return CmdId::LOAD_CAL;
    if (strcmp(cmd, "CLEAR_CAL")   == 0) return CmdId::CLEAR_CAL;
    if (strcmp(cmd, "RESET")       == 0) return CmdId::RESET;
    if (strcmp(cmd, "PING")        == 0) return CmdId::PING;
    if (strcmp(cmd, "GET_ORIENTATION") == 0) return CmdId::GET_ORIENTATION;
    return CmdId::UNKNOWN;
}

}  // namespace comms
