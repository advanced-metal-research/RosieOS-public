#pragma once
#include "drive_alarm_reset.hpp"
#include "execution_policy.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace rosie::rtcore {

// Unverified recovery policy: a readback mismatch requires corrected bring-up.
inline constexpr uint32_t FaultPdoMappingMismatch = 1u << 11;
// Unverified collision recovery policy: explicit reset requires fresh, clear feedback.
inline constexpr uint32_t FaultCollisionWatchdog = 1u << 12;
// Unverified cell I/O recovery: restart after investigating physical feedback or marker timing.
inline constexpr uint32_t FaultCellIO = 1u << 13;
// A failed stationary program wait preserves reference; explicit reset clears it.
inline constexpr uint32_t FaultIoWait = 1u << 14;

enum class Recovery { reset_clears, reset_after_condition_clears, rehome_required, restart_required };
struct FaultRecovery {
  const char* name;
  bool invalidates_home;
  bool requires_drive_reset;
  bool requires_reverification;
  Recovery recovery;
};

// Unverified recovery policy; bit positions retain the existing wire vocabulary.
inline constexpr FaultRecovery kFaultRecovery[] = {
  // R21 "Invalid/discontinuous/out-of-limit prepared plan": preserve Home.
  {"start_discontinuity", false, false, false, Recovery::reset_clears},
  // R21 "Hard limit/target-lead violation...": emitted-rate trust is undecided;
  // invalidate affected Home evidence (fail closed).
  {"position_rate", true, false, false, Recovery::rehome_required},
  // R21 "Reset after execution fault with continuous trustworthy position...".
  {"cycle_deadline", false, false, false, Recovery::reset_clears},
  // R21 "Reset after execution fault with continuous trustworthy position...".
  {"cycle_sleep", false, false, false, Recovery::reset_clears},
  // A target-lead trip is a command/tracking fault, not lost position: the
  // absolute encoder and saved HM35 anchor are unaffected. Home is invalidated
  // only when coordinate trust was actually lost (untrusted evidence); reset
  // needs fresh feedback and a valid coordinate source. Never re-home to clear it.
  {"target_lead", false, false, false, Recovery::reset_after_condition_clears},
  // R21 "Reset after execution fault with continuous trustworthy position...":
  // conditional invalidation only when feedback/identity trust was lost.
  {"following_error", false, false, false, Recovery::reset_after_condition_clears},
  // R21 "Lost/ambiguous position, wrap...": source must return before reset.
  {"coordinate_reference", true, false, false, Recovery::rehome_required},
  // R21 "Missing a current complete full-group exchange...": lost position
  // while enabled invalidates; missing disabled feedback is undecided, invalidate.
  {"drive_readiness", false, true, true, Recovery::reset_after_condition_clears},
  // R21 "Reset after execution fault with continuous trustworthy position...":
  // disabled transport loss preserves Home only with fresh one-count recovery.
  // This narrows the blanket invalidation in R21's missing-exchange row.
  {"bus_transport", false, false, true, Recovery::reset_after_condition_clears},
  // R21 "Reset after execution fault with continuous trustworthy position...".
  {"completion_timeout", false, false, false, Recovery::reset_clears},
  // Retained wire bit: brake-hold detections are warnings and never latch it.
  // The cyclic owner supplies no brake_clear_mask, so any such latch persists.
  {"brake_hold", false, false, true, Recovery::reset_after_condition_clears},
  {"pdo_mapping_mismatch", true, false, true, Recovery::restart_required},
  {"collision_watchdog", false, false, false, Recovery::reset_after_condition_clears},
  {"cell_io", false, false, false, Recovery::restart_required},
  {"io_wait", false, false, false, Recovery::reset_clears},
};

constexpr const char* recovery_name(Recovery recovery) noexcept {
  switch (recovery) {
    case Recovery::reset_clears: return "reset_clears";
    case Recovery::reset_after_condition_clears: return "reset_after_condition_clears";
    case Recovery::rehome_required: return "rehome_required";
    case Recovery::restart_required: return "restart_required";
  }
  return "restart_required";
}

struct FaultEvidence {
  uint32_t affected_axis_mask = 0;
  uint32_t untrusted_axis_mask = 0;
  uint32_t disabled_axis_mask = 0;
};
using FaultEvidenceArray = std::array<FaultEvidence, std::size(kFaultRecovery)>;

struct FaultRecoveryInput {
  uint32_t faults = 0;
  uint32_t configured_mask = 0;
  uint32_t requested_mask = 0;
  uint32_t fresh_mask = 0;
  uint32_t trusted_mask = 0;
  uint32_t coordinate_source_mask = 0;
  uint32_t drive_fault_mask = 0;
  uint32_t held_match_mask = 0;
  bool bus_healthy = false;
  uint32_t collision_clear_mask = 0;
  // Fresh independent evidence permits clearing only a reference fault so an
  // explicit Home can establish a new frame. It never grants coordinate trust.
  uint32_t reference_recovery_mask = 0;
  uint32_t brake_clear_mask = 0;
  // Existing Home is evidence only alongside fresh, trusted coordinates and
  // cleared loss-of-trust provenance; Reset must never establish it itself.
  uint32_t home_valid_mask = 0;
};
struct FaultResetDecision {
  uint32_t cleared = 0;
  uint32_t persists = 0;
  uint32_t rehome_required = 0;
  std::array<uint32_t, std::size(kFaultRecovery)> rehome_axis_masks{};
  uint32_t drive_reset_mask = 0;
  bool reverify = false;
  FaultEvidenceArray remaining{};
};

// Decide the entire selected reset without mutation. Any persistent condition
// rejects the transaction, preventing a partial clear beside fault_persists.
// Axis selection never clears a different axis sharing the same fault bit.
inline FaultResetDecision decide_fault_reset(const FaultRecoveryInput& input,
                                            const FaultEvidenceArray& evidence) noexcept {
  FaultResetDecision out{};
  out.remaining = evidence;
  const uint32_t selected = input.requested_mask & input.configured_mask;
  for (size_t bit = 0; bit < std::size(kFaultRecovery); ++bit) {
    const uint32_t fault = 1u << bit;
    if (!(input.faults & fault)) continue;
    const auto& row = kFaultRecovery[bit];
    const auto& prior = evidence[bit];
    // Missing latch provenance is not evidence of an unaffected axis.
    const uint32_t affected = prior.affected_axis_mask ? prior.affected_axis_mask : input.configured_mask;
    const uint32_t mask = affected & selected;
    if (!mask) continue;
    const bool fresh = (input.fresh_mask & mask) == mask;
    const bool source = (input.coordinate_source_mask & mask) == mask;
    bool persists = row.recovery == Recovery::restart_required;
    if (fault == FaultBusTransport) persists |= !input.bus_healthy || !fresh;
    if (fault == FaultDriveReadiness) persists |= !input.bus_healthy || !fresh || (input.drive_fault_mask & mask);
    if (fault == FaultCoordinateReference)
      persists |= !fresh || (!source && (input.reference_recovery_mask & mask) != mask);
    if (fault == FaultFollowingError || fault == FaultTargetLead) persists |= !fresh || !source;
    if (fault == FaultCollisionWatchdog) persists |= !fresh || !source || (input.collision_clear_mask & mask) != mask;
    if (fault == FaultBrakeHold) persists |= !input.bus_healthy || !fresh || !source ||
        (input.drive_fault_mask & mask) || (input.brake_clear_mask & mask) != mask ||
        (input.trusted_mask & mask) != mask || (prior.untrusted_axis_mask & mask);
    if (persists) out.persists |= fault;
    uint32_t invalid = prior.untrusted_axis_mask | (mask & ~input.trusted_mask);
    if (row.invalidates_home) {
      // A qualified saved-anchor recovery can restore Home while leaving the
      // coordinate fault latched for explicit operator Reset. Invalidating it
      // again here creates a reset/recovery/fault loop. Preserve only axes
      // already revalidated by that independent recovery, never raw feedback
      // alone or a latch with missing provenance.
      const uint32_t recovered = fault == FaultCoordinateReference
          ? prior.affected_axis_mask & ~prior.untrusted_axis_mask &
              input.home_valid_mask & input.fresh_mask & input.trusted_mask &
              input.coordinate_source_mask
          : 0;
      invalid |= mask & ~recovered;
    }
    if (fault == FaultBusTransport)
      invalid |= mask & ~(prior.disabled_axis_mask & input.held_match_mask & input.trusted_mask);
    out.rehome_axis_masks[bit] = mask & invalid;
    out.rehome_required |= out.rehome_axis_masks[bit];
    out.cleared |= fault;
    out.remaining[bit].affected_axis_mask = affected & ~mask;
    out.remaining[bit].untrusted_axis_mask &= ~mask;
    out.remaining[bit].disabled_axis_mask &= ~mask;
    if (row.requires_drive_reset) out.drive_reset_mask |= mask;
    out.reverify |= row.requires_reverification;
  }
  // Unverified future/unknown bits must never be silently erased.
  out.persists |= input.faults & ~((1u << std::size(kFaultRecovery)) - 1);
  if (out.persists) {
    out.cleared = out.drive_reset_mask = 0;
    out.reverify = false;
    out.remaining = evidence;
  }
  return out;
}

// Advisory for an explicit, full-cell Reset that preserves every existing Home.
// The decision is the same one used by ResetFault, not a second recovery policy.
// Publication additionally needs current full-group feedback and inhibited
// execution. It grants no authority and never establishes a reference.
inline bool fault_reset_preserves_home(const FaultRecoveryInput& input,
                                      const FaultEvidenceArray& evidence,
                                      const FaultResetDecision& decision,
                                      bool inhibited) noexcept {
  const uint32_t mask = input.configured_mask;
  if (!inhibited || !mask || input.requested_mask != mask || !input.faults ||
      !input.bus_healthy || (input.drive_fault_mask & mask) ||
      ((input.fresh_mask & input.trusted_mask &
        input.coordinate_source_mask & input.home_valid_mask) & mask) != mask ||
      decision.persists || decision.rehome_required || decision.cleared != input.faults)
    return false;
  for (size_t bit = 0; bit < evidence.size(); ++bit) {
    if (!(input.faults & (1u << bit))) continue;
    const auto& prior = evidence[bit];
    if (!prior.affected_axis_mask || (prior.affected_axis_mask & ~mask) ||
        prior.untrusted_axis_mask || decision.remaining[bit].affected_axis_mask)
      return false;
  }
  return true;
}

// Axis capacity of the per-axis drive-alarm results (the native axis maximum).
inline constexpr size_t kRecoveryDriveAxes = 16;

// One coherent native observation includes current provenance and the last
// command outcome; sequence correlation prevents reporting a previous reset.
// The drive-alarm results belong to the same reset_sequence, so a reader can
// never pair this press's outcome with an earlier press's cause.
struct FaultRecoverySnapshot {
  FaultEvidenceArray evidence{};
  uint32_t faults = 0;
  uint32_t home_valid_mask = 0;
  uint32_t configured_mask = 0;
  // Current candidate is separate from the last attempted command below.
  FaultResetDecision candidate{};
  bool reset_preserves_home = false;
  uint64_t reset_sequence = 0;
  uint32_t attempted = 0;
  FaultEvidenceArray reset_evidence{};
  FaultResetDecision reset{};
  uint32_t drive_alarm_axis_mask = 0;
  std::array<DriveAlarmResetEvidence, kRecoveryDriveAxes> drive_alarm{};
};

} // namespace rosie::rtcore
