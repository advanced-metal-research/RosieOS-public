#pragma once

#include "ds402.hpp"
#include "execution_policy.hpp"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace rosie::rtcore {

// General per-axis drive-state supervision. It applies identically to every
// axis and never reads brake-guard state: joints differ only by gear ratio,
// which the caller applies when converting the joint-unit motion bound to
// counts. Each detection ends the current Arm attempt (or, for disarm motion,
// warns) with a plain reason. Nothing here latches a fault, requires reset or
// restart, or blocks a later Arm: the operator retries Arm.

// Machine-level `drive_supervision` values, compiled by rtctl into the axis
// profile line `drive_supervision=enable_ms|release_slack_ms|disarm_counts`.
// The machine declares one joint-unit value per bound; the compiler floors it
// to counts per axis by gear. No bound is defined in source: an omitted block
// leaves every field zero, which disables the enable, release and
// disarm-motion checks rather than substituting an undeclared number. Home
// acquisition stationarity is the machine's joint-unit held tolerance
// (joint_unit_tolerances), not a second supervisor value.
struct DriveSupervisionConfig {
  // Arm request to Operation Enabled.
  uint64_t enable_timeout_ns = 0;
  // Beyond the axis's brake release delay before an Operation Enabled axis
  // whose brake never reports released ends the Arm attempt.
  uint64_t release_slack_ns = 0;
  // Displacement while disarming with the drive still energized.
  uint32_t disarm_motion_counts = 0;
  bool configured() const noexcept {
    return enable_timeout_ns || release_slack_ns || disarm_motion_counts;
  }
  bool valid() const noexcept {
    return !configured() || (enable_timeout_ns && release_slack_ns && disarm_motion_counts &&
                             disarm_motion_counts <= INT32_MAX);
  }
};

enum class DriveSupervisorKind : uint32_t {
  None, EnableTimeout, ReleaseTimeout, LeftOperationEnabled, ArmCancelled, DisarmMotion
};
enum class ArmCancelCause : uint32_t {
  None, FrameChanged, ContextLost, ModeChanged, TargetChanged, OpeningInterrupted
};

inline const char* drive_supervisor_kind_name(DriveSupervisorKind kind) noexcept {
  switch (kind) {
    case DriveSupervisorKind::None: return "none";
    case DriveSupervisorKind::EnableTimeout: return "enable_timeout";
    case DriveSupervisorKind::ReleaseTimeout: return "release_timeout";
    case DriveSupervisorKind::LeftOperationEnabled: return "left_operation_enabled";
    case DriveSupervisorKind::ArmCancelled: return "arm_cancelled";
    case DriveSupervisorKind::DisarmMotion: return "disarm_motion";
  }
  return "unknown";
}
inline const char* arm_cancel_cause_name(ArmCancelCause cause) noexcept {
  switch (cause) {
    case ArmCancelCause::None: return "none";
    case ArmCancelCause::FrameChanged: return "frame_changed";
    case ArmCancelCause::ContextLost: return "context_lost";
    case ArmCancelCause::ModeChanged: return "mode_changed";
    case ArmCancelCause::TargetChanged: return "target_changed";
    case ArmCancelCause::OpeningInterrupted: return "opening_interrupted";
  }
  return "unknown";
}
inline const char* arm_cancel_cause_text(ArmCancelCause cause) noexcept {
  switch (cause) {
    case ArmCancelCause::FrameChanged: return "its coordinate frame changed";
    case ArmCancelCause::ContextLost: return "its enable context was lost (authority, Home, configuration or drive alarm)";
    case ArmCancelCause::ModeChanged: return "its mode of operation changed";
    case ArmCancelCause::TargetChanged: return "its hold target or command changed";
    case ArmCancelCause::OpeningInterrupted: return "the drive left the enable sequence";
    case ArmCancelCause::None: break;
  }
  return "it was cancelled";
}

// Latest supervisor event for one axis; sequence 0 means none this daemon
// incarnation. arm_ended is false only for the disarm-motion warning.
struct alignas(8) DriveSupervisorEvidence {
  uint64_t sequence = 0, observed_ns = 0;
  // Arm requests accepted before this event; a later Arm makes it historical.
  uint64_t arm_request_sequence = 0;
  uint64_t elapsed_ns = 0, bound_ns = 0;
  int64_t displacement_counts = 0;
  uint32_t bound_counts = 0;
  DriveSupervisorKind kind = DriveSupervisorKind::None;
  ArmCancelCause cause = ArmCancelCause::None;
  uint32_t statusword = 0, arm_ended = 0;
};

// Operator text for a supervisor event (non-RT publication only). Joints are
// named J1-based; displacement is in degrees, or mm on a prismatic axis.
inline std::string drive_supervisor_message(const DriveSupervisorEvidence& e, uint32_t axis,
                                            double counts_per_unit, bool rotary) {
  char text[320] = "";
  const unsigned joint = axis + 1, sw = e.statusword;
  const auto ms = [](uint64_t ns) { return static_cast<unsigned long long>(ns / 1000000); };
  switch (e.kind) {
    case DriveSupervisorKind::EnableTimeout:
      std::snprintf(text, sizeof text, "J%u did not reach Operation Enabled within %llu ms (statusword 0x%04x). Arm ended; retry Arm.",
                    joint, ms(e.bound_ns), sw);
      break;
    case DriveSupervisorKind::ReleaseTimeout:
      std::snprintf(text, sizeof text, "J%u brake did not report released within %llu ms of Operation Enabled "
                    "(release delay plus configured slack; statusword 0x%04x). Arm ended; retry Arm.",
                    joint, ms(e.bound_ns), sw);
      break;
    case DriveSupervisorKind::LeftOperationEnabled:
      std::snprintf(text, sizeof text, "J%u left Operation Enabled while armed (statusword 0x%04x). Arm ended; retry Arm.", joint, sw);
      break;
    case DriveSupervisorKind::ArmCancelled:
      std::snprintf(text, sizeof text, "J%u Arm ended because %s (statusword 0x%04x); retry Arm.",
                    joint, arm_cancel_cause_text(e.cause), sw);
      break;
    case DriveSupervisorKind::DisarmMotion: {
      const double per_unit = std::abs(counts_per_unit);
      const double scale = per_unit > 0 ? (rotary ? 180.0 / M_PI : 1000.0) / per_unit : 0;
      const char* unit = rotary ? "deg" : "mm";
      std::snprintf(text, sizeof text, "J%u moved %.3f %s while disarming (drive still energized); bound %.3f %s.",
                    joint, std::abs(double(e.displacement_counts)) * scale, unit, double(e.bound_counts) * scale, unit);
      break;
    }
    case DriveSupervisorKind::None: break;
  }
  return text;
}

struct DriveEnableObservation {
  uint64_t now_ns = 0;
  bool fresh = false;
  uint16_t statusword = 0;
  bool drive_error = false;
  // Armed with this axis selected under current authority. Inhibition for
  // any other reason (Stop, lease, fault) is not a supervised enable attempt.
  bool enable_requested = false;
  // A commanded quick stop leaves Operation Enabled by design; its re-enable
  // receives a fresh enable bound.
  bool quick_stop_commanded = false;
  // Motion permitted by the brake release delay; always true without a brake.
  bool released = true;
  uint64_t release_delay_ns = 0;
  // DriveSupervisionConfig bounds; zero disables that check.
  uint64_t enable_timeout_ns = 0, release_slack_ns = 0;
  ArmCancelCause cancelled = ArmCancelCause::None;
};

struct DriveOutputObservation {
  uint64_t now_ns = 0;
  bool fresh = false;
  uint16_t statusword = 0, controlword = 0;
  // No Home/commissioning service, trajectory, jog, halt, setpoint or jog-stop
  // arrest owns this axis.
  bool stationary_command = false;
  int32_t feedback = 0;
  uint32_t bound_counts = 0;
  uint64_t frame_epoch = 0;
  // Drive torque persists this long after it leaves Operation Enabled (the
  // C05.10 hold delay mirror; 0 without a brake).
  uint64_t energized_hold_ns = 0;
};

class DriveStateSupervisor {
 public:
  // Before outputs: returns true when this axis ends the Arm attempt. The
  // caller clears Arm (never the grant or enable selection) so the same
  // controller may retry.
  bool observe_enable(const DriveEnableObservation& in) noexcept {
    using State = gradient::ds402::State;
    if (!in.enable_requested) { stage_ = Stage::Idle; return false; }
    if (stage_ == Stage::Ended) return false;
    if (in.cancelled != ArmCancelCause::None)
      return end(in, DriveSupervisorKind::ArmCancelled, in.cancelled, 0, 0);
    if (stage_ == Stage::Idle || in.quick_stop_commanded) { stage_ = Stage::Enabling; since_ns_ = in.now_ns; }
    if (in.quick_stop_commanded || !in.fresh) return false;
    const auto drive = gradient::ds402::decode_statusword(in.statusword);
    const bool fault = (in.statusword & 8u) || in.drive_error;
    const uint64_t elapsed = in.now_ns >= since_ns_ ? in.now_ns - since_ns_ : 0;
    if (stage_ == Stage::Enabling) {
      if (drive != State::OperationEnabled) {
        return in.enable_timeout_ns && elapsed >= in.enable_timeout_ns
            ? end(in, DriveSupervisorKind::EnableTimeout, ArmCancelCause::None, elapsed, in.enable_timeout_ns)
            : false;
      }
      stage_ = in.released ? Stage::Running : Stage::Releasing;
      since_ns_ = in.now_ns;
      return false;
    }
    // A drive fault bit is the existing drive-alarm/readiness path's report.
    if (drive != State::OperationEnabled)
      return fault ? false : end(in, DriveSupervisorKind::LeftOperationEnabled, ArmCancelCause::None, 0, 0);
    if (stage_ == Stage::Releasing) {
      if (in.released) { stage_ = Stage::Running; return false; }
      const uint64_t bound = in.release_delay_ns + in.release_slack_ns;
      if (in.release_slack_ns && elapsed >= bound) return end(in, DriveSupervisorKind::ReleaseTimeout, ArmCancelCause::None, elapsed, bound);
    }
    return false;
  }

  // After outputs: watches the energized window after an inhibit issued while
  // the axis held a stationary command. Returns true once per inhibit when
  // displacement from the inhibit position exceeds the bound. A warning only.
  bool observe_output(const DriveOutputObservation& in) noexcept {
    using State = gradient::ds402::State;
    const auto drive = gradient::ds402::decode_statusword(in.statusword);
    const bool energized = drive == State::OperationEnabled || drive == State::QuickStopActive;
    // Enable operation, including mode-specific bits (HM35 start is CW31).
    const auto enabling = [](uint16_t cw) {
      return (cw & gradient::ds402::CW_ENABLE_OP) == gradient::ds402::CW_ENABLE_OP;
    };
    const bool enable = enabling(in.controlword);
    const bool inhibit_edge = enabling(last_controlword_) && !enable;
    last_controlword_ = in.controlword;
    if (enable) {
      watching_ = false;
      stationary_ = in.stationary_command && in.fresh && drive == State::OperationEnabled;
      return false;
    }
    if (inhibit_edge && stationary_ && in.fresh && energized && in.bound_counts) {
      watching_ = true; warned_ = false;
      anchor_ = in.feedback; epoch_ = in.frame_epoch; inhibit_ns_ = in.now_ns; left_ns_ = 0;
    }
    stationary_ = false;
    if (!watching_ || !in.fresh) return false;
    if (in.frame_epoch != epoch_) { watching_ = false; return false; }
    if (!energized && !left_ns_) left_ns_ = in.now_ns;
    if (left_ns_ && in.now_ns - left_ns_ >= in.energized_hold_ns) { watching_ = false; return false; }
    const auto delta = static_cast<int64_t>(command_delta(in.feedback, anchor_));
    if (warned_ || std::llabs(delta) <= int64_t(in.bound_counts)) return false;
    warned_ = true;
    record(in.now_ns, in.statusword, DriveSupervisorKind::DisarmMotion, ArmCancelCause::None,
           in.now_ns - inhibit_ns_, 0, false);
    evidence_.displacement_counts = delta;
    evidence_.bound_counts = in.bound_counts;
    return true;
  }

  const DriveSupervisorEvidence& evidence() const noexcept { return evidence_; }
  uint16_t last_controlword() const noexcept { return last_controlword_; }

 private:
  enum class Stage { Idle, Enabling, Releasing, Running, Ended };
  bool end(const DriveEnableObservation& in, DriveSupervisorKind kind, ArmCancelCause cause,
           uint64_t elapsed, uint64_t bound) noexcept {
    stage_ = Stage::Ended;
    record(in.now_ns, in.statusword, kind, cause, elapsed, bound, true);
    return true;
  }
  void record(uint64_t now, uint16_t statusword, DriveSupervisorKind kind, ArmCancelCause cause,
              uint64_t elapsed, uint64_t bound, bool arm_ended) noexcept {
    const auto sequence = evidence_.sequence + 1;
    evidence_ = {};
    evidence_.sequence = sequence;
    evidence_.observed_ns = now;
    evidence_.kind = kind;
    evidence_.cause = cause;
    evidence_.statusword = statusword;
    evidence_.elapsed_ns = elapsed;
    evidence_.bound_ns = bound;
    evidence_.arm_ended = arm_ended ? 1u : 0u;
  }
  Stage stage_ = Stage::Idle;
  uint64_t since_ns_ = 0;
  uint16_t last_controlword_ = 0;
  bool watching_ = false, warned_ = false, stationary_ = false;
  int32_t anchor_ = 0;
  uint64_t epoch_ = 0, inhibit_ns_ = 0, left_ns_ = 0;
  DriveSupervisorEvidence evidence_{};
};

} // namespace rosie::rtcore
