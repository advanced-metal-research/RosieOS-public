#include "drive_state_supervisor.hpp"

#include <cassert>
#include <cstdio>
#include <string>

using namespace rosie::rtcore;

namespace {

constexpr uint64_t ms = 1'000'000;
constexpr uint16_t kSwitchOnDisabled = 0x40, kSwitchedOn = 0x23, kOperationEnabled = 0x27;
constexpr uint16_t kQuickStopActive = 0x07, kFault = 0x08;
// The the 1430 cell machine drive_supervision values (unverified, issue #158).
constexpr uint64_t kEnableTimeout = 500 * ms, kReleaseSlack = 500 * ms;

DriveEnableObservation armed_sample(uint64_t t, uint16_t statusword, bool released = true,
                                    uint64_t release_delay = 0) {
  DriveEnableObservation in{};
  in.now_ns = t; in.fresh = true; in.statusword = statusword;
  in.enable_requested = true; in.released = released; in.release_delay_ns = release_delay;
  in.enable_timeout_ns = kEnableTimeout; in.release_slack_ns = kReleaseSlack;
  return in;
}

// Every axis, with or without a brake, gets the same configured enable bound.
// The axis stops being commanded (the caller clears Arm) and the attempt retries.
void enable_timeout_is_uniform_and_retryable() {
  for (const uint64_t release_delay : {uint64_t{0}, 100 * ms}) {
    DriveStateSupervisor s;
    for (uint64_t t = 0; t < 500 * ms; t += ms)
      assert(!s.observe_enable(armed_sample(t, kSwitchedOn, release_delay == 0, release_delay)));
    assert(s.observe_enable(armed_sample(500 * ms, kSwitchedOn, release_delay == 0, release_delay)));
    const auto& e = s.evidence();
    assert(e.kind == DriveSupervisorKind::EnableTimeout && e.arm_ended && e.sequence == 1);
    assert(e.elapsed_ns == 500 * ms && e.bound_ns == kEnableTimeout && e.statusword == kSwitchedOn);
    // One event per attempt while the request persists.
    assert(!s.observe_enable(armed_sample(501 * ms, kSwitchedOn)));
    // Arm cleared by the caller; retry reaches Operation Enabled at +100 ms.
    DriveEnableObservation idle = armed_sample(502 * ms, kSwitchOnDisabled);
    idle.enable_requested = false;
    assert(!s.observe_enable(idle));
    for (uint64_t t = 600 * ms; t < 700 * ms; t += ms) assert(!s.observe_enable(armed_sample(t, kSwitchedOn)));
    for (uint64_t t = 700 * ms; t < 2000 * ms; t += ms) assert(!s.observe_enable(armed_sample(t, kOperationEnabled)));
    assert(s.evidence().sequence == 1);
  }
  // A drive fault during enable still ends the attempt with its statusword.
  DriveStateSupervisor faulted;
  for (uint64_t t = 0; t < 500 * ms; t += ms) assert(!faulted.observe_enable(armed_sample(t, kFault)));
  assert(faulted.observe_enable(armed_sample(500 * ms, kFault)) && faulted.evidence().statusword == kFault);
}

// Operation Enabled but the brake never reports released: release delay plus
// 500 ms, measured from Operation Enabled.
void release_timeout_follows_configured_delay() {
  DriveStateSupervisor s;
  assert(!s.observe_enable(armed_sample(0, kOperationEnabled, false, 100 * ms)));
  for (uint64_t t = ms; t < 600 * ms; t += ms) assert(!s.observe_enable(armed_sample(t, kOperationEnabled, false, 100 * ms)));
  assert(s.observe_enable(armed_sample(600 * ms, kOperationEnabled, false, 100 * ms)));
  assert(s.evidence().kind == DriveSupervisorKind::ReleaseTimeout && s.evidence().bound_ns == 600 * ms);
  // Release at 599 ms is in time.
  DriveStateSupervisor ok;
  for (uint64_t t = 0; t < 599 * ms; t += ms) assert(!ok.observe_enable(armed_sample(t, kOperationEnabled, false, 100 * ms)));
  for (uint64_t t = 599 * ms; t < 3000 * ms; t += ms) assert(!ok.observe_enable(armed_sample(t, kOperationEnabled, true, 100 * ms)));
  assert(ok.evidence().sequence == 0);
}

// A machine without drive_supervision compiles zero bounds: no enable or
// release timeout fires, while cancellation (not a number) still ends Arm.
void unconfigured_bounds_disable_timeouts() {
  DriveStateSupervisor s;
  for (uint64_t t = 0; t < 5000 * ms; t += ms) {
    auto in = armed_sample(t, t < 2000 * ms ? kSwitchedOn : kOperationEnabled, false, 100 * ms);
    in.enable_timeout_ns = in.release_slack_ns = 0;
    assert(!s.observe_enable(in));
  }
  assert(s.evidence().sequence == 0);
  auto in = armed_sample(5000 * ms, kSwitchedOn); in.cancelled = ArmCancelCause::ModeChanged;
  in.enable_timeout_ns = in.release_slack_ns = 0;
  assert(s.observe_enable(in) && s.evidence().kind == DriveSupervisorKind::ArmCancelled);
  // Partially declared or oversized blocks are refused at admission.
  assert(DriveSupervisionConfig{}.valid() && !DriveSupervisionConfig{}.configured());
  const DriveSupervisionConfig partial{kEnableTimeout, 0, 7509};
  const DriveSupervisionConfig oversized{kEnableTimeout, kReleaseSlack, uint32_t(INT32_MAX) + 1u};
  const DriveSupervisionConfig arm_j4{kEnableTimeout, kReleaseSlack, 7509};
  assert(!partial.valid() && !oversized.valid() && arm_j4.valid());
}

// Leaving Operation Enabled while armed, without a fault bit (STO, external
// enable, drive quick stop), ends the attempt; the policy never re-steps it.
void armed_dropout_without_fault_bit_ends_arm() {
  for (const uint16_t dropped : {kSwitchOnDisabled, kSwitchedOn, kQuickStopActive}) {
    DriveStateSupervisor s;
    for (uint64_t t = 0; t < 50 * ms; t += ms) assert(!s.observe_enable(armed_sample(t, kOperationEnabled)));
    assert(s.observe_enable(armed_sample(50 * ms, dropped)));
    assert(s.evidence().kind == DriveSupervisorKind::LeftOperationEnabled && s.evidence().statusword == dropped);
  }
  // A fault bit belongs to the existing drive-alarm report, not this one.
  DriveStateSupervisor fault;
  for (uint64_t t = 0; t < 50 * ms; t += ms) assert(!fault.observe_enable(armed_sample(t, kOperationEnabled)));
  for (uint64_t t = 50 * ms; t < 1000 * ms; t += ms) assert(!fault.observe_enable(armed_sample(t, kFault | 0x0200)));
  // A commanded quick stop leaves Operation Enabled by design; its re-enable
  // then has its own 500 ms bound.
  DriveStateSupervisor quick;
  for (uint64_t t = 0; t < 50 * ms; t += ms) assert(!quick.observe_enable(armed_sample(t, kOperationEnabled)));
  for (uint64_t t = 50 * ms; t < 60 * ms; t += ms) {
    auto in = armed_sample(t, kQuickStopActive); in.quick_stop_commanded = true;
    assert(!quick.observe_enable(in));
  }
  for (uint64_t t = 60 * ms; t < 559 * ms; t += ms) assert(!quick.observe_enable(armed_sample(t, kSwitchOnDisabled)));
  assert(quick.observe_enable(armed_sample(559 * ms, kSwitchOnDisabled)));
  assert(quick.evidence().kind == DriveSupervisorKind::EnableTimeout);
  // Stale feedback is not evidence of a drop-out.
  DriveStateSupervisor stale;
  for (uint64_t t = 0; t < 50 * ms; t += ms) assert(!stale.observe_enable(armed_sample(t, kOperationEnabled)));
  auto in = armed_sample(50 * ms, kSwitchOnDisabled); in.fresh = false;
  assert(!stale.observe_enable(in));
}

void arm_cancellation_ends_arm_with_cause() {
  for (const auto cause : {ArmCancelCause::FrameChanged, ArmCancelCause::ContextLost, ArmCancelCause::ModeChanged,
                           ArmCancelCause::TargetChanged, ArmCancelCause::OpeningInterrupted}) {
    DriveStateSupervisor s;
    assert(!s.observe_enable(armed_sample(0, kSwitchedOn)));
    auto in = armed_sample(ms, kSwitchedOn); in.cancelled = cause;
    assert(s.observe_enable(in));
    assert(s.evidence().kind == DriveSupervisorKind::ArmCancelled && s.evidence().cause == cause && s.evidence().arm_ended);
  }
}

DriveOutputObservation output(uint64_t t, uint16_t statusword, uint16_t controlword, int32_t feedback,
                              bool stationary = true, uint32_t bound = 7509, uint64_t hold = 100 * ms) {
  DriveOutputObservation in{};
  in.now_ns = t; in.fresh = true; in.statusword = statusword; in.controlword = controlword;
  in.stationary_command = stationary; in.feedback = feedback; in.bound_counts = bound;
  in.frame_epoch = 1; in.energized_hold_ns = hold;
  return in;
}

// J4 gearing (18:1, 131072 x 18 / 2pi counts/rad): rtctl floors the the 1430 cell
// 0.02 rad disarm_motion_tolerance to 7509 counts. Displacement from the
// inhibit position inside the energized window warns once above the bound and
// never at or below it; it never ends Arm.
void disarm_motion_warns_above_bound_only() {
  const double counts_per_rad = 131072.0 * 18.0 / (2 * M_PI);
  assert(static_cast<uint32_t>(std::floor(counts_per_rad * 0.02)) == 7509);
  for (const int32_t moved : {7509, 7510, -7510}) {
    DriveStateSupervisor s;
    assert(!s.observe_output(output(0, kOperationEnabled, 15, 1000)));
    assert(!s.observe_output(output(ms, kOperationEnabled, 0, 1000)));   // inhibit edge, still energized
    assert(!s.observe_output(output(2 * ms, kSwitchOnDisabled, 0, 1000 + moved / 2)));
    const bool warned = s.observe_output(output(50 * ms, kSwitchOnDisabled, 0, 1000 + moved));
    assert(warned == (moved != 7509));
    if (warned) {
      const auto& e = s.evidence();
      assert(e.kind == DriveSupervisorKind::DisarmMotion && !e.arm_ended);
      assert(e.displacement_counts == moved && e.bound_counts == 7509 && e.elapsed_ns == 49 * ms);
      assert(!s.observe_output(output(60 * ms, kSwitchOnDisabled, 0, 1000 + 2 * moved))); // once per inhibit
      assert(drive_supervisor_message(e, 3, counts_per_rad, true) ==
             "J4 moved 1.146 deg while disarming (drive still energized); bound 1.146 deg.");
    }
  }
  // No configured bound (machine without drive_supervision): never watched.
  DriveStateSupervisor unbounded;
  assert(!unbounded.observe_output(output(0, kOperationEnabled, 15, 0, true, 0)));
  assert(!unbounded.observe_output(output(ms, kOperationEnabled, 0, 0, true, 0)));
  assert(!unbounded.observe_output(output(2 * ms, kOperationEnabled, 0, 200000, true, 0)));
  assert(unbounded.evidence().sequence == 0);
  // Window ends at hold delay after the drive leaves Operation Enabled.
  DriveStateSupervisor late;
  assert(!late.observe_output(output(0, kOperationEnabled, 15, 0)));
  assert(!late.observe_output(output(ms, kOperationEnabled, 0, 0)));
  assert(!late.observe_output(output(2 * ms, kSwitchOnDisabled, 0, 0)));
  assert(!late.observe_output(output(102 * ms, kSwitchOnDisabled, 0, 20000)));
  // No brake: the window ends when the drive leaves Operation Enabled.
  DriveStateSupervisor unbraked;
  assert(!unbraked.observe_output(output(0, kOperationEnabled, 15, 0, true, 7510, 0)));
  assert(!unbraked.observe_output(output(ms, kOperationEnabled, 0, 0, true, 7510, 0)));
  assert(!unbraked.observe_output(output(2 * ms, kSwitchOnDisabled, 0, 20000, true, 7510, 0)));
  // Still energized (Operation Enabled) with no brake: watched.
  DriveStateSupervisor energized;
  assert(!energized.observe_output(output(0, kOperationEnabled, 15, 0, true, 7510, 0)));
  assert(!energized.observe_output(output(ms, kOperationEnabled, 0, 0, true, 7510, 0)));
  assert(energized.observe_output(output(2 * ms, kOperationEnabled, 0, 8000, true, 7510, 0)));
  // Inhibit during commanded motion: stopping distance is the motion faults' domain.
  DriveStateSupervisor moving;
  assert(!moving.observe_output(output(0, kOperationEnabled, 15, 0, false)));
  assert(!moving.observe_output(output(ms, kOperationEnabled, 0, 0, false)));
  assert(!moving.observe_output(output(2 * ms, kOperationEnabled, 0, 20000, false)));
  // A mode-specific enable (HM35 start CW31) is not an inhibit.
  DriveStateSupervisor homing;
  assert(!homing.observe_output(output(0, kOperationEnabled, 15, 0)));
  assert(!homing.observe_output(output(ms, kOperationEnabled, 31, 0)));
  assert(!homing.observe_output(output(2 * ms, kOperationEnabled, 31, 20000)));
  // A frame change (Home) is a coordinate change, not motion.
  DriveStateSupervisor frame;
  assert(!frame.observe_output(output(0, kOperationEnabled, 15, 0)));
  assert(!frame.observe_output(output(ms, kOperationEnabled, 0, 0)));
  auto rebased = output(2 * ms, kOperationEnabled, 0, 20000); rebased.frame_epoch = 2;
  assert(!frame.observe_output(rebased));
}

void messages_name_joint_and_statusword() {
  DriveSupervisorEvidence e{};
  e.sequence = 1; e.arm_ended = 1; e.statusword = 0x0233;
  e.kind = DriveSupervisorKind::EnableTimeout; e.bound_ns = kEnableTimeout;
  assert(drive_supervisor_message(e, 3, 1, true) ==
         "J4 did not reach Operation Enabled within 500 ms (statusword 0x0233). Arm ended; retry Arm.");
  e.kind = DriveSupervisorKind::LeftOperationEnabled; e.statusword = 0x0250;
  assert(drive_supervisor_message(e, 4, 1, true) ==
         "J5 left Operation Enabled while armed (statusword 0x0250). Arm ended; retry Arm.");
  e.kind = DriveSupervisorKind::ArmCancelled; e.cause = ArmCancelCause::FrameChanged; e.statusword = 0x0223;
  assert(drive_supervisor_message(e, 3, 1, true) ==
         "J4 Arm ended because its coordinate frame changed (statusword 0x0223); retry Arm.");
  e.kind = DriveSupervisorKind::ReleaseTimeout; e.bound_ns = 600 * ms; e.statusword = 0x0237;
  assert(drive_supervisor_message(e, 5, 1, true) ==
         "J6 brake did not report released within 600 ms of Operation Enabled (release delay plus configured slack; "
         "statusword 0x0237). Arm ended; retry Arm.");
}

} // namespace

int main() {
  enable_timeout_is_uniform_and_retryable();
  release_timeout_follows_configured_delay();
  unconfigured_bounds_disable_timeouts();
  armed_dropout_without_fault_bit_ends_arm();
  arm_cancellation_ends_arm_with_cause();
  disarm_motion_warns_above_bound_only();
  messages_name_joint_and_statusword();
  std::puts("drive state supervisor tests passed");
  return 0;
}
