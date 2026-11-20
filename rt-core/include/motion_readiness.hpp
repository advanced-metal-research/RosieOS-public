#pragma once

#include "ds402.hpp"
#include "execution_policy.hpp"
#include "start_alignment.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace rosie::rtcore {

// Publish the first failing axis gate in API priority order. Group authority
// remains independent; a lower-priority coordinate failure cannot hide Home.
inline const char* axis_readiness_reason(const char* alarm, bool csp_mode,
    bool home_valid, bool coordinate_valid, bool enabled, bool operation_enabled,
    bool brake_released) noexcept {
  return alarm ? alarm : !csp_mode ? "mode_mismatch"
      : !home_valid ? "home_required" : !coordinate_valid ? "coordinate_invalid"
      : !enabled ? "not_enabled" : !operation_enabled ? "not_operation_enabled"
      : !brake_released ? "brake_wait" : "ready";
}

// Evaluate current received PDOs before activating or sampling motion. Enable
// admission uses a different gate: requiring Operation Enabled to request enable
// would prevent the CiA402 state machine from ever reaching this barrier.
inline bool motion_axis_ready(bool feedback_fresh, uint16_t statusword,
                              int8_t mode, uint16_t error_code) noexcept {
  return feedback_fresh && mode == 8 && error_code == 0 &&
      gradient::ds402::decode_statusword(statusword) ==
          gradient::ds402::State::OperationEnabled;
}

inline bool motion_group_ready(uint32_t requested, uint32_t configured,
                               uint32_t ready) noexcept {
  return requested != 0 && !(requested & ~configured) &&
      (requested & ready) == requested;
}

// Value snapshot collected by the cyclic owner. Keeping every authority and
// readiness input explicit makes Start testable without IPC, clocks or a bus.
// The caller still applies its final output permit after this decision.
struct MotionStartState {
  uint64_t generation = 0;
  uint64_t configuration_epoch = 0;
  uint64_t verified_configuration_epoch = 0;
  uint32_t configured_mask = 0;
  uint32_t config_verified_mask = 0;
  uint32_t home_required_mask = 0;
  uint32_t home_valid_mask = 0;
  uint32_t limits_valid_mask = 0;
  uint32_t csp_mode_mask = 0;
  uint32_t enabled_mask = 0;
  uint32_t motion_ready_mask = 0;
  uint32_t safety_fault_mask = 0;
  bool lease_valid = false;
  bool armed = false;
  bool bus_ready = false;
  bool trajectory_active = false;
  bool jog_active = false;
  bool commissioning_active = false;
};

struct MotionStartRequest {
  uint64_t generation = 0;
  uint64_t first_time_ns = 0;
  uint32_t axis_mask = 0;
  bool cancelled = false;
  uint64_t cycle_ns = 0;
};

struct MotionStartAxis {
  double first_target = 0;
  double held_target = 0;
  bool limit_direction_valid = true;
  double completion_tolerance_counts = 0;
  double first_velocity = 0;
  double max_velocity_counts_per_s = 0;
  double max_acceleration_counts_per_s2 = 0;
};

// Unverified internal reason codes; the cyclic owner maps them to native reasons.
enum class MotionStartReason : uint8_t { None, ModeConflict, NotReady, InvalidTrajectory, OutsideLimitsOutward };

struct MotionStartDecision {
  bool activate = false;
  MotionStartReason reason = MotionStartReason::NotReady;
  uint32_t discontinuous_mask = 0;
  uint64_t alignment_duration_ns = 0;
};

// Pure, allocation-free and bounded by the configured maximum axis count.
// Mode conflict retains precedence over admission errors. Only an otherwise
// admissible Start checks continuity, so a stale/unauthorized request cannot
// fault the machine through an unrelated bad first point.
template<std::size_t Capacity>
inline MotionStartDecision evaluate_motion_start(
    const MotionStartState& state, const MotionStartRequest& request,
    const std::array<MotionStartAxis, Capacity>& axes) noexcept {
  static_assert(Capacity > 0 && Capacity <= 32, "Start masks support one to 32 axes");
  if (state.trajectory_active || state.jog_active || state.commissioning_active)
    return {false, MotionStartReason::ModeConflict, 0};

  constexpr uint32_t available_mask = UINT32_MAX >> (32 - Capacity);
  const uint32_t requested = request.axis_mask;
  if (request.cancelled || request.generation != state.generation ||
      !state.lease_valid || !state.armed || !state.bus_ready || state.safety_fault_mask ||
      state.verified_configuration_epoch != state.configuration_epoch ||
      !requested || (requested & ~available_mask) || (requested & ~state.configured_mask) ||
      (state.config_verified_mask & requested) != requested ||
      (state.home_valid_mask & state.home_required_mask & requested) !=
          (state.home_required_mask & requested) ||
      (state.limits_valid_mask & requested) != requested ||
      (state.csp_mode_mask & requested) != requested ||
      (state.enabled_mask & requested) != requested ||
      !motion_group_ready(requested, state.configured_mask, state.motion_ready_mask))
    return {};

  uint32_t discontinuous = 0;
  for (std::size_t axis = 0; axis < Capacity; ++axis) {
    if (!(requested & (1u << axis))) continue;
    if (!axes[axis].limit_direction_valid)
      return {false, MotionStartReason::OutsideLimitsOutward, 0};
    if (request.first_time_ns != 0 ||
        !continuous_start(axes[axis].first_target, axes[axis].held_target))
      discontinuous |= 1u << axis;
  }
  if (discontinuous) {
    const auto duration = request.first_time_ns == 0
        ? start_alignment_duration_ns(axes, requested, request.cycle_ns) : 0;
    if (duration) return {true, MotionStartReason::None, 0, duration};
    return {false, MotionStartReason::InvalidTrajectory, discontinuous};
  }
  return {true, MotionStartReason::None, 0};
}

} // namespace rosie::rtcore
