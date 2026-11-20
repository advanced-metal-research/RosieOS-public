#pragma once
#include <cstdint>

namespace rosie::rtcore {
// A stationary program barrier owns elapsed wall time separately from the
// trajectory clock. No allocation, sleeping, polling thread or UI authority.
class IoWait {
 public:
  enum class Result { pending, satisfied, timed_out, invalid_input };
  void reset() noexcept { active_=false; matching_=false; began_=matched_=0; }
  Result step(uint64_t now, bool valid, bool value, bool expected,
              uint64_t timeout, uint64_t stable) noexcept {
    if (!active_) { active_=true; began_=now; matching_=false; }
    // Invalid evidence cannot complete even a zero-stability wait.
    if (!valid) return Result::invalid_input;
    if (now-began_>=timeout) return Result::timed_out;
    if (value==expected) {
      if (!matching_) { matching_=true; matched_=now; }
      if (now-matched_>=stable) return Result::satisfied;
    } else matching_=false;
    return Result::pending;
  }
 private:
  bool active_=false, matching_=false;
  uint64_t began_=0, matched_=0;
};
}
