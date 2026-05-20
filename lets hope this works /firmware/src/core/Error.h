#pragma once

#include <cstdint>

namespace core {

enum class ErrorCode : uint8_t {
  NONE,
  UART_OVERFLOW,
  COMMAND_TOO_LONG,
  PARSE_ERROR,
  QUEUE_FULL,
  INVALID_COMMAND,
  INVALID_ARGUMENT,
  INVALID_STATE,
  ILLEGAL_TRANSITION,
  SENSOR_NOT_FOUND,
  IMU_TIMEOUT,
  CALIBRATION_FAILED,
  STORAGE_ERROR,
  INTERNAL_ERROR,
};

struct HealthStatus {
  ErrorCode last_error{ErrorCode::NONE};
  uint32_t fault_count{0};
  bool recoverable{true};

  // NOT thread-safe. fault_count++ is a non-atomic load-modify-store.
  // Safe only from a single execution context. Before RTOS migration, protect
  // with an atomic or mutex.
  void record(ErrorCode code, const bool can_recover) {
    if (code == ErrorCode::NONE) {
      last_error = ErrorCode::NONE;
      recoverable = true;
      return;
    }
    last_error = code;
    recoverable = can_recover;
    ++fault_count;
  }
};

const char* toString(ErrorCode code);

}  // namespace core
