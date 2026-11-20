#pragma once

#include "ds402.hpp"
#include "ipc_v1.hpp"
#include <cstdint>
#include <optional>

namespace rosie::rtcore {

// Optional commissioned transition policy; absent keeps the legacy guard exactly.
struct BrakeTransitionConfig {
  uint32_t max_displacement_counts = 0;
  uint32_t max_velocity_counts_per_s = 0;
  uint64_t target_stationary_ns = 0, settling_ns = 0, timeout_ns = 0;
  bool enabled() const noexcept { return max_displacement_counts != 0; }
  bool valid(uint64_t hold_ns, uint32_t tolerance) const noexcept {
    return enabled() && max_displacement_counts >= tolerance &&
        max_displacement_counts <= INT32_MAX && max_velocity_counts_per_s > 0 &&
        max_velocity_counts_per_s <= INT32_MAX && target_stationary_ns > 0 &&
        settling_ns > 0 && timeout_ns > hold_ns && settling_ns <= timeout_ns - hold_ns;
  }
};

// Explicit, machine-specific commissioning envelope. These values are not
// vendor limits or physical acceptance; absent preserves the existing policy.
struct BrakeOpeningConfig {
  uint32_t max_displacement_counts = 0, max_velocity_counts_per_s = 0;
  uint64_t timeout_ns = 0;
  bool enabled() const noexcept { return max_displacement_counts != 0; }
  bool valid(uint32_t held_counts) const noexcept {
    return enabled() && max_displacement_counts >= held_counts &&
        max_displacement_counts <= INT32_MAX && max_velocity_counts_per_s > 0 &&
        max_velocity_counts_per_s <= INT32_MAX && timeout_ns > 0 &&
        timeout_ns <= 500000000;
  }
};

struct BrakeConfig {
  bool present = false;
  // C05.13 default, unverified until drive parameters and relay are measured.
  uint64_t release_delay_ns = 100000000;
  // C05.10 default, unverified until drive parameters and relay are measured.
  uint64_t hold_delay_ns = 100000000;
  bool gravity_axis = false;
  bool feedback_required = false;
  // Unverified legacy tolerance: one count of displacement from the fixed anchor.
  uint32_t hold_displacement_tolerance_counts = 1;
  BrakeTransitionConfig transition{};
  BrakeOpeningConfig opening{};
};

struct BrakeOutput {
  uint32_t state = gradient::ipc::v1::BRAKE_UNKNOWN; // BrakeStateV1 wire values.
  bool motion_permitted = false;
  bool hold_window_active = false;
};

enum class BrakeGuardPhase : uint32_t { Dormant, Enabled, Closing, Settling, Held, Failed, Opening, OpeningEnabled };
enum class BrakeGuardFailure : uint32_t {
  None, DisabledDisplacement, StaleFeedback, InvalidConfiguration, ObservationTime,
  FrameChanged, DriveState, TargetCap, VelocityUnavailable, VelocityCap,
  Corridor, DisplacementCap, SettlingTimeout, BrakeFeedback, OpeningProof, OpeningTimeout
};
inline const char* brake_guard_failure_name(BrakeGuardFailure reason) noexcept {
  switch (reason) {
    case BrakeGuardFailure::None: return "none";
    case BrakeGuardFailure::DisabledDisplacement: return "disabled_displacement";
    case BrakeGuardFailure::StaleFeedback: return "stale_feedback";
    case BrakeGuardFailure::InvalidConfiguration: return "invalid_configuration";
    case BrakeGuardFailure::ObservationTime: return "observation_time";
    case BrakeGuardFailure::FrameChanged: return "frame_changed";
    case BrakeGuardFailure::DriveState: return "drive_state";
    case BrakeGuardFailure::TargetCap: return "stationary_target_cap";
    case BrakeGuardFailure::VelocityUnavailable: return "velocity_unavailable";
    case BrakeGuardFailure::VelocityCap: return "velocity_cap";
    case BrakeGuardFailure::Corridor: return "transition_corridor";
    case BrakeGuardFailure::DisplacementCap: return "transition_displacement_cap";
    case BrakeGuardFailure::SettlingTimeout: return "settling_timeout";
    case BrakeGuardFailure::BrakeFeedback: return "brake_feedback";
    case BrakeGuardFailure::OpeningProof: return "opening_proof";
    case BrakeGuardFailure::OpeningTimeout: return "opening_timeout";
  }
  return "unknown";
}
inline const char* brake_guard_phase_name(BrakeGuardPhase phase) noexcept {
  switch (phase) {
    case BrakeGuardPhase::Dormant: return "dormant";
    case BrakeGuardPhase::Enabled: return "enabled";
    case BrakeGuardPhase::Closing: return "closing";
    case BrakeGuardPhase::Settling: return "settling";
    case BrakeGuardPhase::Held: return "held";
    case BrakeGuardPhase::Failed: return "failed";
    case BrakeGuardPhase::Opening: return "opening";
    case BrakeGuardPhase::OpeningEnabled: return "opening_enabled";
  }
  return "unknown";
}

// Operator wording for a brake-hold warning. The drive owns brake timing; the
// remedy is its delay parameter, never a native fault reset or restart.
inline const char* brake_warning_message(bool torque_on) noexcept {
  return torque_on
      ? "Brake appears to release after torque on - adjust drive brake release delay (C05.13)."
      : "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).";
}

// Latest brake-hold warning for non-RT diagnosis; never a fault or permission
// input. A timer estimate cannot become a claimed observation of the relay or
// brake input. Trip-sample fields describe the detecting cycle; peak and settled
// fields are updated while the guard waits to re-anchor.
struct alignas(8) BrakeHoldEvidence {
  uint64_t observed_ns = 0;
  int64_t displacement_counts = 0;
  int32_t anchor_counts = 0, feedback_counts = 0;
  uint32_t statusword = 0, state = 0, hold_window_active = 0;
  uint32_t released_valid = 0, released = 0;
  BrakeGuardFailure cause = BrakeGuardFailure::DisabledDisplacement;
  BrakeGuardPhase phase = BrakeGuardPhase::Dormant;
  // Exact trip sample and bounds, not a later publication or lifetime maximum.
  int32_t velocity_counts_per_s = 0;
  uint32_t velocity_valid = 0, transition_caps_valid = 0;
  uint32_t effective_velocity_cap_counts_per_s = 0, effective_displacement_cap_counts = 0;
  // Per-axis warning count for this daemon incarnation; zero means none.
  uint64_t warning_sequence = 0;
  // Relative to anchor_counts: largest excursion, and the re-anchored rest.
  int64_t peak_displacement_counts = 0, settled_displacement_counts = 0;
  int32_t peak_velocity_counts_per_s = 0;
  uint32_t torque_on = 0, settled = 0;
  // Same-direction motion warnings in this braked period, including this one,
  // and their net displacement; slip once the slip threshold is reached.
  uint32_t slip = 0, same_direction_count = 0;
  int64_t same_direction_displacement_counts = 0;
  // Latest energized hold window, recorded but never judged: 0 until one
  // closes. Its displacement is relative to the closing's starting anchor.
  uint64_t torque_off_ns = 0;
  int64_t energized_peak_displacement_counts = 0;
  int32_t energized_peak_velocity_counts_per_s = 0, torque_off_anchor_counts = 0;
};

// Pure drive-managed brake timing; no brake output is commanded here. Without
// a mapped released signal, APPLIED/RELEASED are timer-derived estimates, never
// confirmation of physical engagement. Feedback overrides that estimate both
// ways, but cannot shorten C05.13 or authorize motion without enabled holding.
// Missing required feedback fails closed. Re-enable always restarts C05.13.
class BrakeMachine {
 public:
  BrakeOutput step(gradient::ds402::State drive, uint64_t now,
                   const BrakeConfig& config,
                   std::optional<bool> released = std::nullopt) noexcept {
    using namespace gradient::ipc::v1;
    if (!config.present) {
      *this = BrakeMachine{};
      return {BRAKE_RELEASED, true, false};
    }
    const bool enabled = drive == gradient::ds402::State::OperationEnabled;
    if (!initialized_ || enabled != enabled_ || now < transition_ns_) {
      hold_pending_ = initialized_ && enabled_ && !enabled;
      transition_ns_ = now;
      initialized_ = true;
      enabled_ = enabled;
    }
    const uint64_t elapsed = now - transition_ns_;
    const bool holding = !enabled && hold_pending_ && elapsed < config.hold_delay_ns;
    if (config.feedback_required && !released.has_value())
      return {BRAKE_UNKNOWN, false, holding};
    if (enabled) {
      if (elapsed < config.release_delay_ns)
        return {BRAKE_WAIT_RELEASE_DELAY, false, false};
      if (released.has_value() && !*released) return {BRAKE_APPLIED, false, false};
      return {BRAKE_RELEASED, true, false};
    }
    if (released.has_value()) {
      if (!*released) return {BRAKE_APPLIED, false, holding};
      return {holding ? BRAKE_WAIT_HOLD_DELAY : BRAKE_RELEASED, false, holding};
    }
    return {holding ? BRAKE_WAIT_HOLD_DELAY : BRAKE_APPLIED, false, holding};
  }

 private:
  uint64_t transition_ns_ = 0;
  bool initialized_ = false;
  bool enabled_ = false;
  bool hold_pending_ = false;
};

} // namespace rosie::rtcore
