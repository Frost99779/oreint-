#include "Error.h"

namespace core {

const char* toString(const ErrorCode code) {
  switch (code) {
    case ErrorCode::NONE: return "NONE";
    case ErrorCode::UART_OVERFLOW: return "UART_OVERFLOW";
    case ErrorCode::COMMAND_TOO_LONG: return "COMMAND_TOO_LONG";
    case ErrorCode::PARSE_ERROR: return "PARSE_ERROR";
    case ErrorCode::QUEUE_FULL: return "QUEUE_FULL";
    case ErrorCode::INVALID_COMMAND: return "INVALID_COMMAND";
    case ErrorCode::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
    case ErrorCode::INVALID_STATE: return "INVALID_STATE";
    case ErrorCode::ILLEGAL_TRANSITION: return "ILLEGAL_TRANSITION";
    case ErrorCode::SENSOR_NOT_FOUND: return "SENSOR_NOT_FOUND";
    case ErrorCode::IMU_TIMEOUT: return "IMU_TIMEOUT";
    case ErrorCode::CALIBRATION_FAILED: return "CALIBRATION_FAILED";
    case ErrorCode::STORAGE_ERROR: return "STORAGE_ERROR";
    case ErrorCode::INTERNAL_ERROR: return "INTERNAL_ERROR";
  }
  return "UNKNOWN";
}

}  // namespace core
