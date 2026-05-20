#include "SystemState.h"

#include <cstring>

namespace core {

const char* toString(const SystemState state) {
  switch (state) {
    case SystemState::BOOT: return "BOOT";
    case SystemState::SELF_TEST: return "SELF_TEST";
    case SystemState::IDLE: return "IDLE";
    case SystemState::CALIBRATING: return "CALIBRATING";
    case SystemState::READY: return "READY";
    case SystemState::ERROR: return "ERROR";
  }
  return "UNKNOWN";
}

SystemStateManager::SystemStateManager(const uint32_t now_ms) {
  snapshot_.state = SystemState::BOOT;
  snapshot_.previous = SystemState::BOOT;
  snapshot_.entered_at_ms = now_ms;
  snapshot_.last_error = ErrorCode::NONE;
  copyReason("power_on");
}

bool SystemStateManager::transitionTo(const SystemState next, const char* const reason, const uint32_t now_ms) {
  if (!canTransition(snapshot_.state, next)) {
    transition_error_ = ErrorCode::ILLEGAL_TRANSITION;
    snapshot_.last_error = transition_error_;
    return false;
  }

  snapshot_.previous = snapshot_.state;
  snapshot_.state = next;
  snapshot_.entered_at_ms = now_ms;
  snapshot_.last_error = ErrorCode::NONE;
  transition_error_ = ErrorCode::NONE;
  copyReason(reason);
  return true;
}

bool SystemStateManager::canTransition(const SystemState from, const SystemState to) const {
  if (from == to) return true;
  if (to == SystemState::ERROR) return true;

  switch (from) {
    case SystemState::BOOT:
      return to == SystemState::SELF_TEST || to == SystemState::IDLE;
    case SystemState::SELF_TEST:
      return to == SystemState::IDLE;
    case SystemState::IDLE:
      return to == SystemState::CALIBRATING || to == SystemState::READY;
    case SystemState::CALIBRATING:
      return to == SystemState::IDLE || to == SystemState::READY;
    case SystemState::READY:
      return to == SystemState::IDLE || to == SystemState::CALIBRATING;
    case SystemState::ERROR:
      return to == SystemState::IDLE || to == SystemState::SELF_TEST;
  }
  return false;
}

void SystemStateManager::copyReason(const char* const reason) {
  const char* const src = (reason == nullptr) ? "unspecified" : reason;
  std::strncpy(snapshot_.reason, src, sizeof(snapshot_.reason) - 1U);
  snapshot_.reason[sizeof(snapshot_.reason) - 1U] = '\0';
}

}  // namespace core
