#pragma once

#include <cstdint>

#include "Error.h"

namespace core {

enum class SystemState : uint8_t {
  BOOT,
  SELF_TEST,
  IDLE,
  CALIBRATING,
  READY,
  ERROR,
};

struct StateSnapshot {
  SystemState state{SystemState::BOOT};
  SystemState previous{SystemState::BOOT};
  uint32_t entered_at_ms{0};
  ErrorCode last_error{ErrorCode::NONE};
  char reason[40]{};
};

const char* toString(SystemState state);

class SystemStateManager final {
 public:
  explicit SystemStateManager(uint32_t now_ms);

  const StateSnapshot& snapshot() const { return snapshot_; }
  SystemState state() const { return snapshot_.state; }
  ErrorCode lastTransitionError() const { return transition_error_; }

  bool transitionTo(SystemState next, const char* reason, uint32_t now_ms);
  bool canTransition(SystemState from, SystemState to) const;
  bool require(SystemState required) const { return snapshot_.state == required; }

 private:
  void copyReason(const char* reason);

  StateSnapshot snapshot_{};
  ErrorCode transition_error_{ErrorCode::NONE};
};

}  // namespace core
