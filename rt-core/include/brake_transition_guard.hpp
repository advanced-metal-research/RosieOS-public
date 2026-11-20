#pragma once

#include "coordinate_model.hpp"
#include "engine/brake.hpp"
#include <algorithm>
#include <cstdlib>

namespace rosie::rtcore {

// This describes only a successfully submitted PDO. A requested/planned target
// is not evidence. A disabled command preserves the last enabled command history
// while the drive catches up with the inhibit; service or moving commands do not.
struct BrakeHoldCommand {
  bool valid = false, enabled = false, stationary = false;
  int32_t target = 0;
  uint64_t generation = 0;
  uint64_t submitted_ns = 0;
  uint16_t controlword = 0;
  int8_t mode = 0;
  // Receipt namespace: an ordinary control generation may have the same
  // numeric value as a commissioning-stage token, but is not stage proof.
  bool opening_stage = false;
};
struct BrakeTransitionInput {
  uint64_t observed_ns = 0, now_ns = 0, maximum_age_ns = 0;
  BrakeHoldCommand command{};
  bool velocity_valid = false;
  int32_t velocity = 0;
  uint64_t maximum_gap_ns = 0;
  // The cyclic owner establishes the exact selected Arm opening stage,
  // identity and current native permission. A requested CW15 alone cannot
  // establish this authorization or successful wire proof.
  bool opening_authorized = false;
  uint64_t maximum_wire_age_ns = 0;
  // Ordinary Arm retains its frozen mode8 witness through brake-release
  // qualification. Native Home never authorizes an opening stage.
  bool opening_until_released = false;
  // Armed with this axis enabled. Ends a post-warning resettle once the drive
  // reports the brake applied, so a warning can never block Arm.
  bool enable_requested = false;
};

// The drive keeps torque for its hold delay (C05.10) after an inhibit while it
// applies the brake, then drops torque. Movement in that energized window is
// the servo finishing its correction toward the last target, not brake
// evidence: judging it reported servo settling as a brake latch fault. It is
// recorded here for diagnosis and never judged. torque_off_ns stays 0 until
// the window closes; closing is then judged from torque_off_anchor.
struct BrakeEnergizedWindow {
  uint64_t start_ns = 0, torque_off_ns = 0;
  int32_t anchor = 0, torque_off_anchor = 0;
  int64_t peak_displacement = 0;
  int32_t peak_velocity = 0;
  void open(uint64_t now, int32_t from) noexcept {
    *this = {};
    start_ns = now;
    anchor = from;
  }
  void observe(int64_t displacement, bool velocity_valid, int32_t velocity) noexcept {
    if (std::abs(displacement) > std::abs(peak_displacement)) peak_displacement = displacement;
    if (velocity_valid && std::abs(int64_t(velocity)) > std::abs(int64_t(peak_velocity))) peak_velocity = velocity;
  }
  void close(uint64_t now, int32_t feedback) noexcept {
    torque_off_ns = now;
    torque_off_anchor = feedback;
  }
};

// Detection only. Failed is terminal within this guard; DisabledBrakeGuard
// reports it as a warning and restarts the guard, so a detection never faults.
class BrakeTransitionGuard {
 public:
  using Phase = BrakeGuardPhase;
  BrakeGuardFailure failure = BrakeGuardFailure::None;
  Phase failed_phase = Phase::Dormant;
  Phase phase = Phase::Dormant;
  int32_t anchor = 0;
  uint64_t frame_epoch = 0;
  // Diagnostic-only: bounds actually evaluated on this observation. Frozen on failure.
  bool diagnostic_caps_valid = false;
  uint32_t diagnostic_velocity_cap = 0, diagnostic_displacement_cap = 0;
  // Latest energized window; energized_closed marks the observation whose
  // torque-off closed it.
  BrakeEnergizedWindow energized{};
  bool energized_closed = false;

  bool observe(const BrakeConfig& config, const BrakeOutput& brake,
      gradient::ds402::State drive, bool fresh, int32_t feedback,
      uint64_t epoch, bool native_home_service,
      const BrakeTransitionInput& input) noexcept {
    using State = gradient::ds402::State;
    energized_closed = false;
    if (phase == Phase::Failed) return true;
    diagnostic_caps_valid = false;
    diagnostic_velocity_cap = diagnostic_displacement_cap = 0;
    if (!fresh) {
      return phase == Phase::Dormant ? false : fail(BrakeGuardFailure::StaleFeedback);
    }
    if (!config.present || !config.transition.valid(config.hold_delay_ns,
            config.hold_displacement_tolerance_counts)) return fail(BrakeGuardFailure::InvalidConfiguration);
    if (config.opening.enabled() && !config.opening.valid(config.hold_displacement_tolerance_counts))
      return fail(BrakeGuardFailure::InvalidConfiguration);
    const uint64_t now = input.observed_ns;
    if (!now || !input.maximum_age_ns || now > input.now_ns ||
        input.now_ns - now > input.maximum_age_ns ||
        !input.maximum_gap_ns ||
        (previous_ns_ && (now <= previous_ns_ || now - previous_ns_ > input.maximum_gap_ns))) return fail(BrakeGuardFailure::ObservationTime);
    previous_ns_ = now;
    // A real enable/service receipt permanently ends startup recovery. No later
    // Stop, fault reset or return to inactive state can reopen this exception.
    if (drive == State::OperationEnabled || native_home_service ||
        (input.command.valid && (input.command.enabled ||
         (input.command.controlword != 0 && input.command.controlword != 128))))
      startup_only_ = false;
    startup_unready_ = false;
    if (qualified_closing_eligible_ && frame_epoch != epoch)
      return fail(BrakeGuardFailure::FrameChanged);
    if (qualified_closing_eligible_ && !input.command.valid)
      return fail(BrakeGuardFailure::OpeningProof);
    if (qualified_closing_eligible_ && input.command.controlword != 0) {
      // Losing the first inhibit receipt while feedback still reports OP must
      // not silently recapture a displaced anchor. Every continued OP anchor
      // needs advancing causal enabled-wire evidence until the first CW0.
      const bool enabled_wire = input.command.controlword == 15 && input.command.mode == 8;
      if (!enabled_wire || !opening_wire_time_valid(input, qualified_wire_ns_))
        return fail(BrakeGuardFailure::OpeningProof);
      qualified_wire_ns_ = input.command.submitted_ns;
    }
    if (qualified_closing_eligible_ && input.command.valid && input.command.controlword == 0) {
      if (!opening_wire_time_valid(input, qualified_wire_ns_) || frame_epoch != epoch)
        return fail(frame_epoch != epoch ? BrakeGuardFailure::FrameChanged : BrakeGuardFailure::OpeningProof);
      // Capture exactly once at the first causally observed inhibit, retaining
      // the last observed OP position in its qualified frame. Permission
      // loss cannot discard this witness or follow a sagging axis with anchors.
      qualified_closing_eligible_ = false;
      qualified_closing_ = true;
      transition_since_ns_ = input.command.submitted_ns;
      lower_ = -int64_t(qualified_closing_config_.max_displacement_counts);
      upper_ = int64_t(qualified_closing_config_.max_displacement_counts);
      begin_energized(transition_since_ns_);
      phase = Phase::Closing;
    }
    if (qualified_closing_eligible_ && drive != State::OperationEnabled)
      return fail(BrakeGuardFailure::OpeningProof); // No submitted inhibit proof.
    // Home is allowed to establish a new coordinate frame only after the
    // bounded opening/aborted-opening monitor has completed. It cannot erase
    // the captured raw-frame excursion while that monitor owns the axis.
    if ((phase == Phase::Opening || phase == Phase::OpeningEnabled || opening_closing_ || qualified_closing_) && frame_epoch != epoch)
      return fail(BrakeGuardFailure::FrameChanged);
    const bool was_held = phase == Phase::Held;
    // Preserve Home/frame re-anchoring, but never let an unobserved frame change
    // erase an active failure. Home cannot supply normal stationary-wire proof.
    if (phase != Phase::Dormant && frame_epoch != epoch && !native_home_service) return fail(BrakeGuardFailure::FrameChanged);
    if (phase == Phase::Dormant || (native_home_service && frame_epoch != epoch)) {
      anchor = feedback;
      frame_epoch = epoch;
      phase = drive == State::OperationEnabled ? Phase::Enabled : Phase::Held;
      if (native_home_service && drive != State::OperationEnabled)
        begin_home_closing(config, now, false);
    }
    // A new opening stage needs a prior Held observation, not a freshly
    // initialized/reconciled anchor on the first enabling sample. Tokens are
    // unique monotonically increasing stage identities: toggling authorization
    // cannot repeat an unfinished or consumed stage and reset its deadline.
    if (was_held && phase == Phase::Held && config.opening.enabled() &&
        input.opening_authorized && input.command.controlword == 15) {
      if (!opening_wire_valid(input, 0) || !input.command.generation ||
          input.command.generation <= last_opening_generation_ ||
          std::abs(command_delta(input.command.target, anchor)) >
              config.hold_displacement_tolerance_counts)
        return fail(BrakeGuardFailure::OpeningProof);
      opening_config_ = config.opening;
      opening_target_ = input.command.target;
      opening_until_released_ = input.opening_until_released;
      opening_since_ns_ = input.command.submitted_ns;
      opening_wire_ns_ = 0;
      last_opening_generation_ = input.command.generation;
      phase = Phase::Opening;
    }
    if (phase == Phase::Opening || phase == Phase::OpeningEnabled) {
      if (!opening_bounds(input, feedback)) return true;
      if (brake.state == gradient::ipc::v1::BRAKE_UNKNOWN)
        return fail(BrakeGuardFailure::BrakeFeedback);
      if (drive != State::SwitchOnDisabled && drive != State::ReadyToSwitchOn &&
          drive != State::SwitchedOn && drive != State::OperationEnabled)
        return fail(BrakeGuardFailure::DriveState);
      if (!input.opening_authorized || (input.command.valid && input.command.controlword == 0)) {
        // Inhibition takes effect outside this monitor immediately. Keep the
        // original position envelope through drive-managed closure; neither
        // withdrawal of permission nor a subsequent fresh token re-anchors it.
        // Only an abort after observed enable has an energized window; an
        // abort before enable keeps judging every sample, as opening does.
        opening_closing_ = true;
        transition_since_ns_ = now;
        lower_ = -int64_t(opening_config_.max_displacement_counts);
        upper_ = int64_t(opening_config_.max_displacement_counts);
        if (phase == Phase::OpeningEnabled) begin_energized(now);
        phase = Phase::Closing;
      } else {
        if (!opening_wire_valid(input, opening_wire_ns_) ||
            input.command.generation != last_opening_generation_ ||
            input.command.target != opening_target_ ||
            input.opening_until_released != opening_until_released_)
          return fail(BrakeGuardFailure::OpeningProof);
        opening_wire_ns_ = input.command.submitted_ns;
        if (now - opening_since_ns_ >= opening_config_.timeout_ns)
          return fail(BrakeGuardFailure::OpeningTimeout);
        if (drive != State::OperationEnabled) {
          if (phase == Phase::OpeningEnabled) return fail(BrakeGuardFailure::DriveState);
          return false;
        }
        // Observed enable is not brake release or permission to move. Existing
        // BrakeMachine release delay and all normal motion fences still apply.
        if (opening_until_released_ && !brake.motion_permitted) {
          phase = Phase::OpeningEnabled;
          return false;
        }
        if (opening_until_released_) {
          // Actual fixed-target enable AND release qualify this brake lifecycle,
          // not a motion target. Later ordinary jog/trajectory guards still own
          // movement; its eventual inhibit uses the bounded closing envelope in
          // either direction rather than pretending the servo target predicts
          // mechanical brake take-up. This witness grants no motion authority.
          qualified_closing_eligible_ = true;
          qualified_wire_ns_ = input.command.submitted_ns;
          qualified_closing_config_ = config.transition;
          qualified_hold_ns_ = config.hold_delay_ns;
        }
        phase = Phase::Enabled;
        opening_closing_ = false;
      }
    }
    if (native_home_service) {
      if (drive == State::OperationEnabled && !opening_closing_ && !qualified_closing_) {
        phase = Phase::Enabled;
        anchor = feedback;
        return false;
      }
      if (phase == Phase::Enabled) {
        anchor = feedback;
        begin_home_closing(config, now, true);
      }
      // The authorized CW6 dwell must progress, not reset its anchor/timer on
      // every cycle. An explicit Home frame change above starts a new fixed
      // corridor; otherwise closing and held displacement checks remain live.
    }
    if (drive == State::OperationEnabled && !opening_closing_ && !qualified_closing_) {
      // Re-enable discards the old transition, never commands re-enable itself.
      phase = Phase::Enabled;
      anchor = feedback;
      return false;
    }
    // A drive can report a genuine startup alarm (for example Sync Loss)
    // before any enable has ever been submitted. That is drive readiness, not
    // evidence of failed brake holding. Keep the ORIGINAL fresh raw anchor and
    // Held displacement/frame checks, but publish UNKNOWN and deny motion until
    // an eligible inactive drive state is observed. This does not clear alarms
    // or permit reset by itself; the explicit inhibited reset path owns that.
    if (startup_only_ && phase == Phase::Held &&
        (drive == State::Fault || drive == State::FaultReactionActive || drive == State::NotReady)) {
      startup_unready_ = true;
      if (brake.state != gradient::ipc::v1::BRAKE_APPLIED)
        return fail(BrakeGuardFailure::BrakeFeedback);
      return std::abs(command_delta(feedback, anchor)) >
          config.hold_displacement_tolerance_counts
          ? fail(BrakeGuardFailure::DisabledDisplacement) : false;
    }
    if (drive != State::SwitchOnDisabled && drive != State::ReadyToSwitchOn &&
        drive != State::SwitchedOn && !((opening_closing_ || qualified_closing_) && drive == State::OperationEnabled))
      return fail(BrakeGuardFailure::DriveState);
    const int64_t delta = command_delta(feedback, anchor);
    const int64_t tolerance = config.hold_displacement_tolerance_counts;
    if (phase == Phase::Enabled) {
      transition_since_ns_ = now;
      lower_ = -tolerance;
      upper_ = tolerance;
      begin_energized(now);
      phase = Phase::Closing;
    }
    if (phase == Phase::Closing || phase == Phase::Settling) {
      // Outside the energized window both tests are active on EVERY sample,
      // including quiet-window acquisition. A diagnostic cap can only shrink
      // the fixed corridor, never widen it or learn a new anchor from sag.
      const auto& closing_config = qualified_closing_ ? qualified_closing_config_ : config.transition;
      const auto velocity_cap = opening_closing_ ? opening_config_.max_velocity_counts_per_s
                                                : closing_config.max_velocity_counts_per_s;
      const auto displacement_cap = opening_closing_ ? opening_config_.max_displacement_counts
                                                    : closing_config.max_displacement_counts;
      record_diagnostic_caps(velocity_cap, displacement_cap);
      if (!input.velocity_valid) return fail(BrakeGuardFailure::VelocityUnavailable);
      if (energized_closing_) {
        energized.observe(delta, true, input.velocity);
      } else {
        if (std::abs(int64_t(input.velocity)) > velocity_cap)
          return fail(BrakeGuardFailure::VelocityCap);
        if (delta < lower_ || delta > upper_) return fail(BrakeGuardFailure::Corridor);
        if (std::abs(delta) > displacement_cap)
          return fail(BrakeGuardFailure::DisplacementCap);
      }
      if (now - transition_since_ns_ >= closing_config.timeout_ns)
        return fail(BrakeGuardFailure::SettlingTimeout);
      if (brake.state == gradient::ipc::v1::BRAKE_UNKNOWN) return fail(BrakeGuardFailure::BrakeFeedback);
      if (opening_closing_ || qualified_closing_) {
        // Current successfully submitted inhibit must precede the disabled
        // observation. A requested Stop plus a timer-derived APPLIED value is
        // insufficient to return ownership to the ordinary Held guard.
        const auto& cmd = input.command;
        if (drive == State::OperationEnabled || !cmd.valid || cmd.controlword != 0 ||
            !opening_wire_time_valid(input, qualified_closing_ ? qualified_wire_ns_ : opening_wire_ns_)) {
          phase = Phase::Closing; // A broken quiet proof cannot accumulate time.
          return false;
        }
        if (qualified_closing_) qualified_wire_ns_ = cmd.submitted_ns;
        else opening_wire_ns_ = cmd.submitted_ns;
      }
      if (now - transition_since_ns_ < (qualified_closing_ ? qualified_hold_ns_ : config.hold_delay_ns) || brake.hold_window_active)
        return false;
      if (brake.state != gradient::ipc::v1::BRAKE_APPLIED) return fail(BrakeGuardFailure::BrakeFeedback);
      if (phase == Phase::Closing && energized_closing_) {
        // Torque off: judge closing, settling and hold from here, so every
        // later detection describes brake take-up rather than servo settling.
        energized_closing_ = false;
        energized.close(now, feedback);
        energized_closed = true;
        anchor = feedback;
        lower_ = -tolerance;
        upper_ = tolerance;
        phase = Phase::Settling;
        settle_since_ns_ = now;
        settle_low_ = settle_high_ = 0;
        return std::abs(int64_t(input.velocity)) > velocity_cap ? fail(BrakeGuardFailure::VelocityCap) : false;
      }
      if (phase == Phase::Closing) {
        phase = Phase::Settling;
        settle_since_ns_ = now;
        settle_low_ = settle_high_ = delta;
      } else {
        settle_low_ = std::min(settle_low_, delta);
        settle_high_ = std::max(settle_high_, delta);
        if (settle_high_ - settle_low_ > tolerance) {
          // Restart only the quiet-window evidence, not the fixed transition
          // corridor or its deadline. A drifting axis eventually times out.
          settle_since_ns_ = now;
          settle_low_ = settle_high_ = delta;
        }
      }
      if (now - settle_since_ns_ >= closing_config.settling_ns) {
        anchor = feedback;  // Exactly one final Held anchor per disable.
        phase = Phase::Held;
        opening_closing_ = false;
        qualified_closing_ = false;
      }
      return false;
    }
    if (brake.state != gradient::ipc::v1::BRAKE_APPLIED) return fail(BrakeGuardFailure::BrakeFeedback);
    // Permanent hold monitoring also applies to profiles whose generic gravity
    // flag is false. Never follow feedback incrementally and miss cumulative sag.
    return std::abs(delta) > tolerance ? fail(BrakeGuardFailure::DisabledDisplacement) : false;
  }

  bool startup_reset_eligible() const noexcept {
    return startup_only_ && startup_unready_ && phase == Phase::Held &&
        failure == BrakeGuardFailure::None;
  }

  // Begin a fresh lifecycle after a warning: the next fresh observation anchors
  // at current feedback. A warning ends the startup exemption, so it can never
  // re-establish the startup reset witness, and consumed opening tokens stay
  // consumed. Coordinate/Home origins are unchanged.
  void restart() noexcept {
    const auto consumed_opening = last_opening_generation_;
    *this = BrakeTransitionGuard{};
    startup_only_ = false;
    last_opening_generation_ = consumed_opening;
  }

  void qualify(BrakeOutput& output) const noexcept {
    if (phase == Phase::Dormant || startup_unready_) {
      output.state = gradient::ipc::v1::BRAKE_UNKNOWN;
      output.motion_permitted = false;
    }
    if (phase == Phase::Closing || phase == Phase::Settling) {
      output.state = gradient::ipc::v1::BRAKE_WAIT_HOLD_DELAY;
      output.motion_permitted = false;
      output.hold_window_active = true;
    }
    if (phase == Phase::Opening) {
      output.state = gradient::ipc::v1::BRAKE_WAIT_RELEASE_DELAY;
      output.motion_permitted = false;
    }
    // An Arm opening stays OpeningEnabled until BrakeMachine's independently
    // qualified release delay/signal admits motion.
    if (phase == Phase::OpeningEnabled && opening_until_released_)
      output.motion_permitted = false;
    if (phase == Phase::Failed) output.motion_permitted = false;
  }

 private:
  bool startup_only_ = true;
  bool startup_unready_ = false;
  bool opening_wire_valid(const BrakeTransitionInput& input, uint64_t previous) const noexcept {
    const auto& cmd = input.command;
    return cmd.valid && cmd.opening_stage && cmd.controlword == 15 && cmd.mode == 8 &&
        opening_wire_time_valid(input, previous);
  }
  bool opening_wire_time_valid(const BrakeTransitionInput& input, uint64_t previous) const noexcept {
    const auto& cmd = input.command;
    return cmd.submitted_ns && cmd.submitted_ns > previous && cmd.submitted_ns < input.observed_ns &&
        input.maximum_wire_age_ns && cmd.submitted_ns <= input.now_ns &&
        input.now_ns - cmd.submitted_ns <= input.maximum_wire_age_ns;
  }
  void record_diagnostic_caps(uint32_t velocity, uint32_t displacement) noexcept {
    diagnostic_caps_valid = true;
    diagnostic_velocity_cap = velocity;
    diagnostic_displacement_cap = displacement;
  }
  bool opening_bounds(const BrakeTransitionInput& input, int32_t feedback) noexcept {
    record_diagnostic_caps(opening_config_.max_velocity_counts_per_s, opening_config_.max_displacement_counts);
    if (!input.velocity_valid) { fail(BrakeGuardFailure::VelocityUnavailable); return false; }
    if (std::abs(int64_t(input.velocity)) > opening_config_.max_velocity_counts_per_s) {
      fail(BrakeGuardFailure::VelocityCap); return false;
    }
    if (std::abs(command_delta(feedback, anchor)) > opening_config_.max_displacement_counts) {
      fail(BrakeGuardFailure::DisplacementCap); return false;
    }
    return true;
  }
  // energized: the drive was observed enabled, so it holds torque until the
  // hold delay elapses. A closing that never had torque is judged throughout.
  void begin_home_closing(const BrakeConfig& config, uint64_t now, bool energized_close) noexcept {
    phase = Phase::Closing;
    transition_since_ns_ = now;
    lower_ = -int64_t(config.hold_displacement_tolerance_counts);
    upper_ = int64_t(config.hold_displacement_tolerance_counts);
    if (energized_close) begin_energized(now);
  }
  void begin_energized(uint64_t now) noexcept {
    energized.open(now, anchor);
    energized_closing_ = true;
  }
  bool fail(BrakeGuardFailure reason) noexcept {
    failure = reason; failed_phase = phase; phase = Phase::Failed; return true;
  }
  bool energized_closing_ = false;
  bool opening_closing_ = false;
  bool qualified_closing_eligible_ = false, qualified_closing_ = false;
  BrakeTransitionConfig qualified_closing_config_{};
  uint64_t qualified_wire_ns_ = 0, qualified_hold_ns_ = 0;
  BrakeOpeningConfig opening_config_{};
  int32_t opening_target_ = 0;
  bool opening_until_released_ = false;
  uint64_t opening_since_ns_ = 0, opening_wire_ns_ = 0, last_opening_generation_ = 0;
  uint64_t previous_ns_ = 0, transition_since_ns_ = 0, settle_since_ns_ = 0;
  int64_t lower_ = 0, upper_ = 0, settle_low_ = 0, settle_high_ = 0;
};

} // namespace rosie::rtcore
