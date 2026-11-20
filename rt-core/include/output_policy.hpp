#pragma once

#include "control_policy.hpp"
#include "coordinate_model.hpp"
#include "ds402.hpp"
#include "execution_policy.hpp"
#include "engine/jog.hpp"
#include "engine/brake.hpp"
#include "brake_transition_guard.hpp"
#include <cmath>
#include <cstdint>
#include <limits>

namespace rosie::rtcore {

struct BrakeGuardWarning {
  uint64_t sequence = 0, observed_ns = 0;
  BrakeGuardFailure cause = BrakeGuardFailure::None;
  BrakeGuardPhase phase = BrakeGuardPhase::Dormant;
  bool torque_on = false;
  int32_t anchor = 0;
  bool caps_valid = false;
  uint32_t velocity_cap = 0, displacement_cap = 0;
  // Relative to anchor while resettling; settled_displacement is the re-anchor.
  int64_t peak_displacement = 0, settled_displacement = 0;
  int32_t peak_velocity = 0;
  bool settled = false;
  // Consecutive same-direction motion warnings in this braked period and the
  // net displacement since that run's first anchor; slip at the threshold.
  uint32_t same_direction_count = 0;
  int64_t same_direction_displacement = 0;
  bool slip = false;
};

// Unverified initial choice, not a measurement: consecutive same-direction
// motion warnings within ONE braked period that suggest a slipping brake. A
// normal settle at every stop goes the same way under gravity, so the run
// never spans separate stops, and a reversal (jitter) restarts it.
inline constexpr uint32_t kBrakeSlipSameDirectionWarnings = 3;

// Compare disabled feedback with a fixed anchor. A per-cycle comparison would
// silently accept cumulative sag of one count per cycle.
// A detection is a WARNING, never a fault: the drive manages its own brake
// (rt-core submits only CW0), absolute feedback keeps position, and a few
// counts of settle under load at torque-off is normal. The drive keeps torque
// for its hold delay after the inhibit, so movement in that energized window
// is recorded, not judged; closing is judged from the torque-off anchor.
// observe() returns true once per warning, then the guard resettles: it
// re-anchors on re-enable, Home or frame change, on an Arm request once the
// drive reports the brake applied, or after a quiet applied dwell, so one
// settle cannot warn repeatedly. Travel beyond the displacement bound while
// resettling warns again from a new reference instead of silently following a
// slipping brake, and repeated same-direction warnings in one braked period
// are marked as suspected slip.
struct DisabledBrakeGuard {
  int32_t anchor = 0;
  uint64_t anchor_epoch = 0;
  bool have_anchor = false;
  bool resettling = false;
  // Set when the latest warning's peak or settled fields change.
  bool warning_updated = false;
  // Latest closed energized window; energized_updated on the torque-off sample.
  BrakeEnergizedWindow energized{};
  bool energized_updated = false;
  BrakeTransitionGuard transition{};
  BrakeGuardWarning warning{};

  bool observe(const BrakeConfig& config, const BrakeOutput& brake,
               gradient::ds402::State drive, bool fresh, int32_t feedback,
               uint64_t frame_epoch = 0,
               bool native_home_service = false, const BrakeTransitionInput& input = {}) noexcept {
    warning_updated = false;
    energized_updated = false;
    // A braked period ends at enable or Arm, Home/frame change or a missing
    // brake: one same-direction settle per stop never accumulates as slip.
    if (fresh && (!config.present || drive == gradient::ds402::State::OperationEnabled ||
        native_home_service || input.enable_requested || input.opening_authorized ||
        frame_epoch != run_epoch_)) run_count_ = 0;
    bool warned = false;
    if (quiet_restart_) {
      restart_quietly(config, brake, drive, fresh, feedback, frame_epoch, native_home_service, input);
    } else if (!resettling) {
      if (detect(config, brake, drive, fresh, feedback, frame_epoch, native_home_service, input)) {
        const bool opt_in = config.transition.enabled();
        if (opt_in && !brake_evidence(transition.failure)) {
          quiet_restart_ = true;
          restart_quietly(config, brake, drive, fresh, feedback, frame_epoch, native_home_service, input);
          return false;
        }
        begin_warning(input, feedback,
            opt_in ? transition.failure : BrakeGuardFailure::DisabledDisplacement,
            opt_in ? transition.failed_phase : BrakeGuardPhase::Dormant, anchor);
        warning.caps_valid = opt_in && transition.diagnostic_caps_valid;
        warning.velocity_cap = warning.caps_valid ? transition.diagnostic_velocity_cap : 0;
        warning.displacement_cap = warning.caps_valid ? transition.diagnostic_displacement_cap : 0;
        warned = true;
      }
    } else if (config.present && fresh && !native_home_service && frame_epoch == anchor_epoch &&
               drive != gradient::ds402::State::OperationEnabled &&
               std::abs(command_delta(feedback, reference_)) > rewarn_counts(config)) {
      begin_warning(input, feedback, BrakeGuardFailure::DisabledDisplacement,
          warning.phase, reference_);
      warned = true;
    }
    if (resettling) resettle(config, brake, drive, fresh, feedback, frame_epoch,
        native_home_service, input);
    return warned;
  }

  // Replaces the transition guard's qualification while resettling: inactive
  // outputs keep reporting the unfinished transition, enabled ones defer to
  // BrakeMachine's release delay. A warning never withholds enabled motion.
  void qualify(const BrakeConfig& config, BrakeOutput& output) const noexcept {
    if (!config.transition.enabled()) return;
    if (!resettling) { transition.qualify(output); return; }
    if (enabled_) return;
    output.motion_permitted = false;
    if (warning.torque_on) {
      output.state = gradient::ipc::v1::BRAKE_WAIT_RELEASE_DELAY;
    } else {
      output.state = gradient::ipc::v1::BRAKE_WAIT_HOLD_DELAY;
      output.hold_window_active = true;
    }
  }

  // Only brake take-up, release or slip evidence is a brake warning. Timing,
  // freshness, frame, drive-state, wire-receipt and configuration causes have
  // their own reports: the drive-state supervisor ends the Arm attempt, and
  // freshness/readiness checks report stale feedback.
  static bool brake_evidence(BrakeGuardFailure cause) noexcept {
    switch (cause) {
      case BrakeGuardFailure::StaleFeedback: case BrakeGuardFailure::InvalidConfiguration:
      case BrakeGuardFailure::ObservationTime: case BrakeGuardFailure::FrameChanged:
      case BrakeGuardFailure::DriveState: case BrakeGuardFailure::OpeningProof:
      case BrakeGuardFailure::OpeningTimeout:
        return false;
      default:
        return true;
    }
  }

 private:
  int32_t reference_ = 0, quiet_reference_ = 0;
  uint64_t quiet_since_ns_ = 0;
  bool quiet_valid_ = false, enabled_ = false, quiet_restart_ = false;
  // Legacy energized window and the following take-up interval.
  BrakeEnergizedWindow legacy_window_{};
  bool torque_off_pending_ = false, takeup_ = false;
  uint64_t takeup_since_ns_ = 0;
  // Current same-direction run of motion warnings.
  uint32_t run_count_ = 0;
  int32_t run_sign_ = 0, run_origin_ = 0;
  uint64_t run_epoch_ = 0;

  // A non-brake detection restarts the transition guard without a warning and
  // keeps restarting until a fresh sample anchors it, so a restart inside a
  // hold window cannot be misread as brake feedback. The last brake warning
  // and consumed opening tokens are unchanged.
  void restart_quietly(const BrakeConfig& config, const BrakeOutput& brake, gradient::ds402::State drive,
                       bool fresh, int32_t feedback, uint64_t frame_epoch,
                       bool native_home_service, const BrakeTransitionInput& input) noexcept {
    transition.restart();
    if (transition.observe(config, brake, drive, fresh, feedback, frame_epoch,
            native_home_service, input) || transition.phase == BrakeTransitionGuard::Phase::Dormant) {
      transition.restart();
      have_anchor = false;
      return;
    }
    quiet_restart_ = false;
    anchor = transition.anchor;
    anchor_epoch = transition.frame_epoch;
    have_anchor = true;
  }

  static uint32_t rewarn_counts(const BrakeConfig& config) noexcept {
    return config.transition.enabled() ? config.transition.max_displacement_counts
                                       : config.hold_displacement_tolerance_counts;
  }
  static uint64_t quiet_ns(const BrakeConfig& config) noexcept {
    return config.transition.enabled() ? config.transition.settling_ns : config.hold_delay_ns;
  }

  bool detect(const BrakeConfig& config, const BrakeOutput& brake,
              gradient::ds402::State drive, bool fresh, int32_t feedback,
              uint64_t frame_epoch, bool native_home_service, const BrakeTransitionInput& input) noexcept {
    if (config.transition.enabled()) {
      const bool failed = transition.observe(config, brake, drive, fresh, feedback,
          frame_epoch, native_home_service, input);
      anchor = transition.anchor;
      anchor_epoch = transition.frame_epoch;
      have_anchor = transition.phase != BrakeTransitionGuard::Phase::Dormant;
      if (transition.energized_closed) torque_off(transition.energized);
      return failed;
    }
    if (!fresh) return false;
    // Native Home changes the count origin. Re-anchor in that frame to prevent
    // interpreting a coordinate change as physical disabled displacement.
    if (!have_anchor || anchor_epoch != frame_epoch || native_home_service ||
        drive == gradient::ds402::State::OperationEnabled) {
      legacy_anchor(drive, feedback, frame_epoch, input.observed_ns);
      return false;
    }
    if (torque_off_pending_) {
      if (brake.hold_window_active) {
        legacy_window_.observe(command_delta(feedback, anchor), input.velocity_valid, input.velocity);
        return false;
      }
      torque_off_pending_ = false;
      legacy_window_.close(input.observed_ns, feedback);
      if (config.present) torque_off(legacy_window_);
      anchor = feedback;
      takeup_ = true;
      takeup_since_ns_ = input.observed_ns;
      return false;
    }
    // A nongravity axis is judged only during brake take-up, one hold delay
    // after torque-off; a gravity axis keeps its permanent hold monitor.
    const bool takeup = takeup_ && input.observed_ns >= takeup_since_ns_ &&
        input.observed_ns - takeup_since_ns_ < config.hold_delay_ns;
    return config.present && (takeup || config.gravity_axis) &&
        std::abs(command_delta(feedback, anchor)) > config.hold_displacement_tolerance_counts;
  }

  // An enabled observation arms the next inhibit's energized window.
  void legacy_anchor(gradient::ds402::State drive, int32_t feedback, uint64_t frame_epoch,
                     uint64_t now) noexcept {
    anchor = feedback;
    anchor_epoch = frame_epoch;
    have_anchor = true;
    if (drive == gradient::ds402::State::OperationEnabled) {
      torque_off_pending_ = true;
      takeup_ = false;
    }
    legacy_window_.open(now, feedback);
  }

  // Torque-off begins a new braked period; it publishes the window it closed.
  void torque_off(const BrakeEnergizedWindow& window) noexcept {
    energized = window;
    energized_updated = true;
    run_count_ = 0;
  }

  void begin_warning(const BrakeTransitionInput& input, int32_t feedback,
                     BrakeGuardFailure cause, BrakeGuardPhase phase, int32_t from) noexcept {
    const auto sequence = warning.sequence + 1;
    warning = {};
    warning.sequence = sequence;
    warning.observed_ns = input.observed_ns;
    warning.cause = cause;
    warning.phase = phase;
    warning.torque_on = phase == BrakeGuardPhase::Opening || phase == BrakeGuardPhase::OpeningEnabled;
    warning.anchor = from;
    warning.peak_displacement = static_cast<int64_t>(command_delta(feedback, from));
    warning.peak_velocity = input.velocity_valid ? input.velocity : 0;
    track_direction(input, feedback, from);
    resettling = true;
    reference_ = feedback;
    quiet_valid_ = false;
  }

  // Counts consecutive same-direction motion warnings. A torque-on warning
  // means enabling began, ending the braked period; a warning without motion
  // evidence (for example stale feedback) neither extends nor breaks the run.
  void track_direction(const BrakeTransitionInput& input, int32_t feedback,
                       int32_t from) noexcept {
    if (warning.torque_on) { run_count_ = 0; return; }
    const auto cause = warning.cause;
    const bool motion = cause == BrakeGuardFailure::DisabledDisplacement ||
        cause == BrakeGuardFailure::VelocityCap || cause == BrakeGuardFailure::Corridor ||
        cause == BrakeGuardFailure::DisplacementCap;
    const int64_t moved = warning.peak_displacement;
    const int32_t sign = moved ? (moved > 0 ? 1 : -1)
        : input.velocity_valid && input.velocity ? (input.velocity > 0 ? 1 : -1) : 0;
    if (!motion || !sign) return;
    if (!run_count_ || sign != run_sign_ || anchor_epoch != run_epoch_) {
      run_count_ = 0;
      run_sign_ = sign;
      run_origin_ = from;
      run_epoch_ = anchor_epoch;
    }
    ++run_count_;
    warning.same_direction_count = run_count_;
    warning.same_direction_displacement = static_cast<int64_t>(command_delta(feedback, run_origin_));
    warning.slip = run_count_ >= kBrakeSlipSameDirectionWarnings;
  }

  void resettle(const BrakeConfig& config, const BrakeOutput& brake,
                gradient::ds402::State drive, bool fresh, int32_t feedback,
                uint64_t frame_epoch, bool native_home_service, const BrakeTransitionInput& input) noexcept {
    using State = gradient::ds402::State;
    if (!fresh) { quiet_valid_ = false; return; }
    enabled_ = drive == State::OperationEnabled;
    if (frame_epoch == anchor_epoch) {
      const auto displacement = static_cast<int64_t>(command_delta(feedback, warning.anchor));
      if (std::abs(displacement) > std::abs(warning.peak_displacement)) {
        warning.peak_displacement = displacement;
        warning_updated = true;
      }
    }
    if (input.velocity_valid && std::abs(int64_t(input.velocity)) > std::abs(int64_t(warning.peak_velocity))) {
      warning.peak_velocity = input.velocity;
      warning_updated = true;
    }
    const bool inactive = drive == State::SwitchOnDisabled || drive == State::ReadyToSwitchOn ||
        drive == State::SwitchedOn;
    const bool applied = brake.state == gradient::ipc::v1::BRAKE_APPLIED && !brake.hold_window_active;
    bool rearm = enabled_ || native_home_service || frame_epoch != anchor_epoch;
    // An opening stage already in progress keeps its token; re-anchoring then
    // would reject that consumed token. It resolves at Operation Enabled.
    if (!rearm && inactive && applied && !input.opening_authorized) {
      if (input.enable_requested) {
        rearm = true;
      } else if (!quiet_valid_ ||
          std::abs(command_delta(feedback, quiet_reference_)) > config.hold_displacement_tolerance_counts) {
        quiet_valid_ = true;
        quiet_since_ns_ = input.observed_ns;
        quiet_reference_ = feedback;
      } else if (input.observed_ns >= quiet_since_ns_ && input.observed_ns - quiet_since_ns_ >= quiet_ns(config)) {
        rearm = true;
      }
    } else if (!rearm) {
      quiet_valid_ = false;
    }
    if (rearm) finish(config, brake, drive, feedback, frame_epoch, native_home_service, input);
  }

  void finish(const BrakeConfig& config, const BrakeOutput& brake, gradient::ds402::State drive,
              int32_t feedback, uint64_t frame_epoch, bool native_home_service,
              const BrakeTransitionInput& input) noexcept {
    const bool same_frame = frame_epoch == anchor_epoch;
    if (config.transition.enabled()) {
      transition.restart();
      // An immediate repeat (for example invalid configuration) keeps waiting
      // without another warning; it retries on the next fresh observation.
      if (transition.observe(config, brake, drive, true, feedback, frame_epoch,
              native_home_service, input)) return;
      anchor = transition.anchor;
      anchor_epoch = transition.frame_epoch;
      have_anchor = transition.phase != BrakeTransitionGuard::Phase::Dormant;
    } else {
      legacy_anchor(drive, feedback, frame_epoch, input.observed_ns);
    }
    resettling = false;
    quiet_valid_ = false;
    warning.settled = true;
    warning.settled_displacement = same_frame
        ? static_cast<int64_t>(command_delta(feedback, warning.anchor)) : 0;
    warning_updated = true;
  }
};

struct AxisOutputConfig {
  AxisLimits limits{};
  double counts_per_unit = 0;
  int32_t sign = 0;
  uint32_t max_profile_velocity = 0;
  bool rotary = false;
  bool coordinate_evidence_required = false;
};

struct AxisOutputInput {
  uint16_t statusword = 0;
  int32_t feedback_position = 0;
  int32_t home_offset = 0;
  bool feedback_fresh = false;
  bool continuous_feedback_valid = false;
  int32_t continuous_feedback = 0;
  bool configuration_current = false;
  bool config_verified = false;
  bool home_valid = false;
  bool armed = false;
  bool enabled = false;
  bool startup_passive = false;
  bool service_active = false;
  uint16_t service_controlword = 0;
  int8_t service_mode = 0; // Caller resolves the no-override sentinel.
  int8_t commanded_mode = 0;
  bool setpoint_present = false;
  int32_t target_position = 0;
  double target_velocity = 0;
  bool snap_hold = false;
  bool jog_active = false;
  bool trajectory_active = false;
  bool brake_motion_permitted = true;
  bool previous_target_valid = false;
  int32_t previous_target = 0; // Last successfully submitted wire target.
  uint64_t now_ns = 0;
  uint64_t previous_cycle_ns = 0;
};

struct AxisOutputState {
  int32_t held_target = 0;
  bool held_target_valid = false;
  bool enable_hold_latched = false;
  PositionRateGuard position_rate{};
  bool position_rate_active = false;
  FollowingErrorGuard following_error{};
  uint16_t quick_stop_cycles_left = 0;
  uint16_t arrest_cycles_left = 0;
  uint8_t fault_reset_cycles_left = 0;
};

struct AxisOutputDecision {
  AxisOutputState next{};
  gradient::ds402::State drive_state = gradient::ds402::State::Unknown;
  uint16_t controlword = 0;
  int32_t target_position = 0;
  int32_t target_velocity = 0;
  int16_t target_torque = 0;
  int8_t mode = 0;
  uint16_t touch_probe_function = 0;
  uint32_t max_profile_velocity = 0;
  InhibitedReference reference{};
  bool invalidate_home = false;
  bool fault = false;
  uint32_t fault_reasons = 0;
};

// Pure per-axis transition: no atomics, PDO access, heap, logging or clocks.
// Fault/revoke and the final whole-group output permit belong to the caller.
// Counter durations retain the checkpoint policy; time-based qualification is
// a separate change. The caller latches new reset requests before this step.
inline AxisOutputDecision evaluate_axis_output(const AxisOutputConfig& config,
    const AxisOutputInput& input, AxisOutputState state) noexcept {
  using State = gradient::ds402::State;
  AxisOutputDecision out{};
  out.drive_state = gradient::ds402::decode_statusword(input.statusword);
  out.target_position = input.feedback_position;
  out.invalidate_home = (input.statusword & 8u) != 0 ||
      (config.coordinate_evidence_required && !input.continuous_feedback_valid);
  const bool axis_ready = input.config_verified &&
      (!config.limits.require_home || (input.home_valid && !out.invalidate_home));
  out.reference = inhibited_reference(out.drive_state, input.feedback_fresh,
      input.configuration_current && axis_ready, input.feedback_position,
      input.previous_target_valid, input.previous_target, input.home_offset,
      config.counts_per_unit, config.sign, config.limits,
      input.continuous_feedback_valid, input.continuous_feedback);
  out.invalidate_home |= out.reference.disposition == ReferenceDisposition::Ambiguous;

  // Enabled hold may reconcile a trusted exterior pose. Keep the inhibited
  // reference unchanged: final group inhibition retains its existing contract.
  auto enabled_reference = out.reference;
  if (input.armed && input.enabled && config.limits.valid) {
    auto hold_limits = config.limits;
    hold_limits.min_q = -std::numeric_limits<double>::max();
    hold_limits.max_q = std::numeric_limits<double>::max();
    enabled_reference = inhibited_reference(out.drive_state, input.feedback_fresh,
        input.configuration_current && axis_ready, input.feedback_position,
        input.previous_target_valid, input.previous_target, input.home_offset,
        config.counts_per_unit, config.sign, hold_limits,
        input.continuous_feedback_valid, input.continuous_feedback);
  }
  const int32_t previous_limit_target = state.held_target_valid ? state.held_target :
      input.continuous_feedback_valid ? input.continuous_feedback : input.feedback_position;
  // Only a selected feedback hold can rebase motion guards. A concurrent
  // setpoint still follows the previous command, even when snap is requested.
  const bool feedback_snap = !input.startup_passive && !input.service_active &&
      !input.setpoint_present && (input.snap_hold || state.arrest_cycles_left > 0);
  const bool trusted_snap = feedback_snap && input.feedback_fresh && input.configuration_current &&
      axis_ready &&
      (!config.coordinate_evidence_required || input.continuous_feedback_valid) &&
      enabled_reference.disposition != ReferenceDisposition::Invalid &&
      enabled_reference.disposition != ReferenceDisposition::Ambiguous;

  auto track_feedback_hold = [&]() {
    if (config.coordinate_evidence_required && input.continuous_feedback_valid) {
      state.held_target = input.continuous_feedback;
      state.held_target_valid = true;
    } else {
      state.held_target = input.feedback_position;
      state.held_target_valid = false;
    }
  };
  auto snap_feedback_hold = [&]() {
    state.held_target = config.coordinate_evidence_required && input.continuous_feedback_valid
        ? input.continuous_feedback
        : input.feedback_position;
    state.held_target_valid = true;
  };
  // A braked axis must hold the pose captured at enable permission through
  // DS402 enable and the brake-release delay. Chasing feedback during that
  // interval lets gravity/inertia drag the reference before motion is admitted.
  // This is a target latch only: it grants no drive or motion permission.
  const bool enable_hold_eligible = !input.startup_passive && !input.service_active &&
      input.armed && input.enabled && input.feedback_fresh && input.configuration_current &&
      axis_ready && !(input.statusword & 8u) &&
      (!config.coordinate_evidence_required || input.continuous_feedback_valid) &&
      enabled_reference.disposition != ReferenceDisposition::Invalid &&
      enabled_reference.disposition != ReferenceDisposition::Ambiguous;
  if (!enable_hold_eligible) state.enable_hold_latched = false;
  else if (!input.brake_motion_permitted && !state.enable_hold_latched) {
    snap_feedback_hold();
    // Preserve the existing inactive-coordinate reconciliation on this first
    // capture. Later enabled samples cannot rebase the motion-rate guard.
    if (enabled_reference.disposition == ReferenceDisposition::Reconciled) {
      state.position_rate.reset(state.held_target, input.previous_cycle_ns);
      state.position_rate_active = true;
    }
    state.enable_hold_latched = true;
  }
  bool invalid_target_input = false;
  if (input.startup_passive) {
    track_feedback_hold(); // Passive startup sends no DS402 state-driving intent.
  } else if (input.service_active) {
    track_feedback_hold();
    out.controlword = input.service_controlword;
    // Native Home also holds under enable until release qualification; its
    // mode-specific start bit must not command motion against an applied brake.
    if (input.service_mode == 6 && !input.brake_motion_permitted)
      out.controlword &= ~uint16_t{0x10};
    out.target_position = state.held_target;
    out.mode = input.service_mode;
    out.max_profile_velocity = config.max_profile_velocity;
  } else {
    const bool want_enable = input.armed && input.enabled;
    bool want_fault_reset = state.fault_reset_cycles_left > 0;
    const bool want_quick_stop = want_enable && state.quick_stop_cycles_left > 0 &&
        !input.jog_active && !input.trajectory_active;
    if (want_fault_reset && out.drive_state != State::Fault) {
      state.fault_reset_cycles_left = 0;
      want_fault_reset = false;
    }
    if (input.snap_hold) snap_feedback_hold();
    if (state.arrest_cycles_left > 0) snap_feedback_hold();
    if (want_enable && out.drive_state == State::OperationEnabled && input.brake_motion_permitted) {
      if (!state.held_target_valid) {
        state.held_target = config.coordinate_evidence_required && input.continuous_feedback_valid
            ? input.continuous_feedback : input.feedback_position;
        state.held_target_valid = true;
      }
      if (input.setpoint_present) {
        state.held_target = input.target_position;
        invalid_target_input = !std::isfinite(input.target_velocity);
        if (!invalid_target_input) {
          out.target_velocity = static_cast<int32_t>(std::llround(std::clamp(
              input.target_velocity, double(std::numeric_limits<int32_t>::min()),
              double(std::numeric_limits<int32_t>::max()))));
        }
      }
    } else if (!state.enable_hold_latched) {
      track_feedback_hold();
    }
    out.controlword = gradient::ds402::controlword_for_enable(
        out.drive_state, want_enable, want_fault_reset);
    if (want_quick_stop && (out.drive_state == State::OperationEnabled ||
                           out.drive_state == State::QuickStopActive))
      out.controlword = gradient::ds402::CW_QUICK_STOP;
    if (want_fault_reset && out.drive_state == State::Fault && state.fault_reset_cycles_left)
      --state.fault_reset_cycles_left;
    out.target_position = state.held_target;
    out.mode = input.commanded_mode;
    out.max_profile_velocity = config.max_profile_velocity;
  }

  if (!input.service_active && !state.enable_hold_latched &&
      enabled_reference.disposition == ReferenceDisposition::Reconciled) {
    state.held_target = enabled_reference.target;
    state.held_target_valid = true;
    out.target_position = enabled_reference.target;
    out.target_velocity = 0;
    state.position_rate.reset(enabled_reference.target, input.previous_cycle_ns);
    state.position_rate_active = true;
  }

  if (input.armed && input.enabled) {
    AxisCoordinateProfile coordinate_profile{};
    coordinate_profile.counts_per_unit = config.counts_per_unit;
    coordinate_profile.sign = config.sign;
    coordinate_profile.home_offset = input.home_offset;
    const AxisCoordinateModel model(coordinate_profile);
    const auto logical_target = model.command_to_logical(out.target_position);
    const double logical_speed = model.count_velocity_to_logical(out.target_velocity);
    if (trusted_snap) {
      state.position_rate.reset(out.target_position, input.previous_cycle_ns);
      state.position_rate_active = true;
    }
    if (!state.position_rate_active) {
      state.position_rate.reset(input.previous_target_valid ? input.previous_target : input.feedback_position,
                                input.previous_cycle_ns);
      state.position_rate_active = true;
    }
    const bool rate_ok = state.position_rate.accept(out.target_position, input.now_ns,
        std::min(std::abs(model.logical_velocity_to_counts(config.limits.max_qd)),
                 double(config.max_profile_velocity)));
    bool following_ok = true;
    if (input.previous_target_valid && input.feedback_fresh &&
        (!config.coordinate_evidence_required || input.continuous_feedback_valid)) {
      // Feedback was received before this cycle's candidate is submitted. Use
      // the last successfully submitted wire target, not a future command the
      // drive has never seen. Do not shift the reference by an estimated servo
      // response delay. The machine's tracking lag allowance instead widens
      // the bound by |commanded velocity| x lag (speed_scaled_bound), so the
      // lag a loaded arm shows at speed is not a fault; the persistence
      // timeout is unchanged and an error beyond the scaled bound still faults.
      const double following_error = std::abs(model.count_delta_to_logical(command_delta(input.previous_target,
          config.coordinate_evidence_required ? input.continuous_feedback : input.feedback_position)));
      following_ok = state.following_error.accept(following_error,
          speed_scaled_bound(config.limits.following_error, logical_speed,
              double(config.limits.tracking_lag_allowance_ns) / 1e9),
          input.now_ns, config.limits.following_error_timeout_ns);
    } else {
      // An absent submitted reference or unknown coordinates cannot supply a measurement or
      // turn the coordinate-loss fault into a spurious tracking diagnosis.
      state.following_error.reset();
    }
    const bool reference_valid = enabled_reference.disposition != ReferenceDisposition::Invalid &&
        enabled_reference.disposition != ReferenceDisposition::Ambiguous && !invalid_target_input &&
        (!config.coordinate_evidence_required || input.continuous_feedback_valid) &&
        model.scaling_valid() && (!feedback_snap || trusted_snap);
    if (!reference_valid) out.fault_reasons |= FaultCoordinateReference;
    if (!rate_ok) out.fault_reasons |= FaultPositionRate;
    if (!following_ok) out.fault_reasons |= FaultFollowingError;
    const auto recovery_limits = command_limits(config.limits,
        config.counts_per_unit * config.sign, input.home_offset);
    const double previous = trusted_snap ? out.target_position :
        enabled_reference.disposition == ReferenceDisposition::Reconciled
            ? enabled_reference.target : previous_limit_target;
    const bool limits_ok = logical_target.valid &&
        limit_recovery_step(recovery_limits, previous, out.target_position) &&
        limit_recovery_velocity(recovery_limits, out.target_position, out.target_velocity);
    out.fault = !reference_valid || !rate_ok || !following_ok ||
        !limits_ok || !config.limits.allows_velocity(logical_speed) ||
        (input.statusword & 8u) || !axis_ready;
    if (out.fault) {
      out.controlword = 0;
      out.target_position = input.feedback_position;
      out.target_velocity = 0;
    }
  }
  if (!input.armed || !input.enabled || out.fault) {
    state.enable_hold_latched = false;
    state.position_rate_active = false;
    state.following_error.reset();
  }
  if (state.quick_stop_cycles_left > 0) --state.quick_stop_cycles_left;
  if (state.arrest_cycles_left > 0) --state.arrest_cycles_left;
  out.next = state;
  return out;
}

} // namespace rosie::rtcore
