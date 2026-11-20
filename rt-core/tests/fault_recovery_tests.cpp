#define ROSIE_RT_SIMULATION 1
#include "engine/cycle_machine.hpp"
#include <cassert>
#include <cstring>

using namespace rosie::rtcore;

// Synthetic inputs exercise the native reset without threads or the plant;
// assertions use explicit bit masks and encoder counts, never adaptive limits.
int main() {
  static_assert(FaultCoordinateReference == 64 && FaultCompletionTimeout == 512);
  static_assert(std::size(kFaultRecovery) == 15 && FaultPdoMappingMismatch == 2048 && FaultCollisionWatchdog == 4096 && FaultCellIO == 8192 && FaultIoWait == 16384);
  assert(!kFaultRecovery[14].invalidates_home && !kFaultRecovery[14].requires_drive_reset);
  // Unverified policy expectations taken from the R21 recovery decisions, except
  // bit 4: target_lead is a tracking fault and keeps trusted Home (AMR-720).
  const bool home_invalid[] = {false, true, false, false, false, false, true, false, false, false, false, true};
  // Unverified drive-reset policy: only readiness faults request a drive pulse.
  const bool drive_reset[] = {false, false, false, false, false, false, false, true, false, false, false, false};
  // Unverified re-verification policy: readiness, transport and brake faults.
  const bool reverify[] = {false, false, false, false, false, false, false, true, true, false, true, true};
  for (size_t bit = 0; bit < 12; ++bit) {
    assert(kFaultRecovery[bit].invalidates_home == home_invalid[bit]);
    assert(kFaultRecovery[bit].requires_drive_reset == drive_reset[bit]);
    assert(kFaultRecovery[bit].requires_reverification == reverify[bit]);
    FaultEvidenceArray evidence{};
    evidence[bit] = {2, 0, 2};
    FaultRecoveryInput input{1u << bit, 3, 3, 3, 3, 3, 0, 3, true};
    input.brake_clear_mask = 3;
    const auto decision = decide_fault_reset(input, evidence);
    assert(decision.persists == (bit == 11 ? 1u << bit : 0u));
    assert(decision.cleared == (bit == 11 ? 0u : 1u << bit));
    assert(decision.rehome_required == (home_invalid[bit] ? 2u : 0u));
    assert(decision.drive_reset_mask == (drive_reset[bit] ? 2u : 0u));
    assert(decision.reverify == (reverify[bit] && bit != 11));
    evidence[bit].untrusted_axis_mask = 2;
    assert(decide_fault_reset(input, evidence).rehome_required == 2);
  }
  for (unsigned scenario=0; scenario<8; ++scenario) {
    FaultEvidenceArray evidence{}; evidence[10]={3,0,3};
    FaultRecoveryInput input{FaultBrakeHold,3,1,3,3,3,0,3,true};
    input.brake_clear_mask=1;
    if (scenario==1) input.brake_clear_mask=2;
    if (scenario==2) input.fresh_mask=2;
    if (scenario==3) input.trusted_mask=2;
    if (scenario==4) input.coordinate_source_mask=2;
    if (scenario==5) input.drive_fault_mask=1;
    if (scenario==6) input.bus_healthy=false;
    if (scenario==7) evidence[10].untrusted_axis_mask=1;
    const auto result=decide_fault_reset(input,evidence);
    assert(result.persists==(scenario ? uint32_t(FaultBrakeHold) : 0u));
    assert(result.cleared==(scenario ? 0u : uint32_t(FaultBrakeHold)));
    assert(result.remaining[10].affected_axis_mask==(scenario ? 3u : 2u));
    assert(result.drive_reset_mask==0);
    if (!scenario) assert(result.rehome_required==0 && result.reverify);
  }
  for (uint32_t fault : {uint32_t(FaultCoordinateReference), uint32_t(FaultBusTransport), uint32_t(FaultDriveReadiness)}) {
    FaultEvidenceArray evidence{};
    evidence[2] = {3, 0, 3};
    for (size_t bit = 0; bit < 11; ++bit) if (fault & (1u << bit)) evidence[bit] = {2, 0, 2};
    FaultRecoveryInput input{fault | FaultCycleDeadline, 3, 3, 0, 3, 0, 2, 3, false};
    const auto decision = decide_fault_reset(input, evidence);
    assert(decision.persists == fault);
    assert(decision.cleared == 0 && decision.drive_reset_mask == 0 && !decision.reverify);
    assert(decision.remaining[2].affected_axis_mask == 3);
  }
  {
    FaultEvidenceArray evidence{};
    evidence[2] = {3, 0, 3};
    FaultRecoveryInput input{FaultCycleDeadline, 3, 1, 3, 3, 3, 0, 3, true};
    assert(decide_fault_reset(input, evidence).remaining[2].affected_axis_mask == 2);
    input.faults |= 1u << 31;
    assert(decide_fault_reset(input, evidence).cleared == 0);
  }

  // Fresh unbound raw evidence is exclusively a route to explicit re-Home.
  // It cannot stand in for trusted coordinates when clearing other faults.
  for (const auto fault : {uint32_t(FaultCoordinateReference), uint32_t(FaultFollowingError), uint32_t(FaultTargetLead),
                           uint32_t(FaultCollisionWatchdog)}) {
    FaultEvidenceArray evidence{};
    for (size_t bit = 0; bit < std::size(kFaultRecovery); ++bit)
      if ((1u << bit) == uint32_t(fault)) evidence[bit] = {3, 3, 3};
    FaultRecoveryInput input{};
    input.faults = fault; input.configured_mask = 3; input.requested_mask = 1;
    input.fresh_mask = 3; input.reference_recovery_mask = 1; input.collision_clear_mask = 3;
    const auto result = decide_fault_reset(input, evidence);
    if (fault == FaultCoordinateReference) {
      assert(result.cleared == uint32_t(fault) && result.rehome_required == 1);
      assert(result.remaining[6].affected_axis_mask == 2);
      input.reference_recovery_mask = 2;
      assert(decide_fault_reset(input, evidence).persists == uint32_t(fault));
      input.reference_recovery_mask = 1; input.fresh_mask = 0;
      assert(decide_fault_reset(input, evidence).persists == uint32_t(fault));
    } else assert(result.persists == uint32_t(fault) && result.cleared == 0);
  }

  // A saved reference was independently recovered while its fault remained
  // latched. Reset retains that Home only with every required proof present.
  for (unsigned missing = 0; missing < 7; ++missing) {
    FaultEvidenceArray evidence{}; evidence[6] = {3, 0, 3};
    FaultRecoveryInput input{};
    input.faults = FaultCoordinateReference;
    input.configured_mask = 3; input.requested_mask = 1;
    input.fresh_mask = input.trusted_mask = input.coordinate_source_mask = 3;
    input.home_valid_mask = 3;
    input.reference_recovery_mask = 3;
    if (missing == 1) input.home_valid_mask = 2;
    if (missing == 2) input.trusted_mask = 2;
    if (missing == 3) input.coordinate_source_mask = 2;
    if (missing == 4) evidence[6].untrusted_axis_mask = 1;
    if (missing == 5) evidence[6].affected_axis_mask = 0;
    if (missing == 6) input.fresh_mask = 2;
    const auto result = decide_fault_reset(input, evidence);
    if (missing == 6) {
      assert(result.persists == FaultCoordinateReference && !result.cleared);
    } else {
      assert(result.cleared == FaultCoordinateReference && !result.persists);
      assert(result.rehome_required == (missing ? 1u : 0u));
      assert(result.remaining[6].affected_axis_mask == 2);
    }
    assert(!result.drive_reset_mask && !result.reverify);
  }

  // The current UI candidate fails closed for incomplete full-cell evidence,
  // even when a narrower native Reset is permitted to prepare explicit Home.
  for (unsigned missing = 0; missing < 15; ++missing) {
    FaultEvidenceArray evidence{}; evidence[6] = {2, 0, 2};
    FaultRecoveryInput input{FaultCoordinateReference, 3, 3, 3, 3, 3, 0, 3, true};
    input.home_valid_mask = 3;
    bool inhibited = true;
    if (missing == 1) input.home_valid_mask = 2; // Other axis is not homed.
    if (missing == 2) input.trusted_mask = 1;
    if (missing == 3) input.coordinate_source_mask = 1;
    if (missing == 4) evidence[6].untrusted_axis_mask = 2;
    if (missing == 5) evidence[6].affected_axis_mask = 0;
    if (missing == 6) input.fresh_mask = 2; // Other axis is stale.
    if (missing == 7) input.drive_fault_mask = 1; // Other drive faulted.
    if (missing == 8) input.bus_healthy = false;
    if (missing == 9) input.requested_mask = 2;
    if (missing == 10) input.configured_mask = 0;
    if (missing == 11) inhibited = false;
    if (missing == 12) input.faults |= 1u << 31;
    if (missing == 13) { input.faults |= FaultCellIO; evidence[13] = {3, 0, 3}; }
    if (missing == 14) evidence[6].affected_axis_mask = 6;
    const auto decision = decide_fault_reset(input, evidence);
    assert(fault_reset_preserves_home(input, evidence, decision, inhibited) == (missing == 0));
  }

  Options options;
  options.num_axes = 2;
  OptionsParseResult configuration;
  for (unsigned axis = 0; axis < 2; ++axis) {
    options.axis[axis].counts_per_unit = 1;
    options.axis[axis].sign = 1;
    auto& limits = configuration.axis_profiles[axis].limits;
    limits.valid = true;
    limits.min_q = -1000;
    limits.max_q = 1000;
  }
  auto machine = std::make_unique<MachineState>(options, configuration);
  MachineState::cycle_owner = machine.get();
  const auto real_clock = MachineState::cycle_clock;
  MachineState::cycle_clock = []() -> uint64_t { return 1'000'000'000; };
  auto setup = [&]() {
    machine->fault_evidence = {};
    machine->execution_fault_reasons.store(0);
    machine->safety_fault_mask.store(0);
    machine->control_policy.home_valid_mask.store(3);
    machine->configuration_epoch.store(1);
    machine->verified_configuration_epoch.store(1);
    machine->control_policy.config_verified_mask.store(3);
    machine->fault_reset_request.store(0);
    machine->bus_ready.store(true);
    machine->recovery_trusted_mask = 3;
    machine->latest_feedback.pdo_fresh_axis_mask = 3;
    machine->motion_done.store(1);
    for (unsigned axis = 0; axis < 2; ++axis) {
      machine->latest_feedback.statusword[axis] = 0x40;
      machine->latest_feedback.mode_display[axis] = 8;
      machine->latest_feedback.pos_counts[axis] = 100;
      machine->latest_feedback.pdo_observed_time_ns[axis] = MachineState::cycle_clock();
      machine->have_last_wire_target[axis] = true;
      machine->last_wire_target[axis] = 100;
    }
  };
  auto reset = [&](uint32_t mask) {
    MachineState::ControlRequest request{MachineState::ControlOperation::ResetFault, mask};
    request.reset_sequence = 42;
    return machine->apply_control(request);
  };
  for (int32_t recovered : {99, 100, 101, 102}) {
    setup();
    machine->execution_fault(FaultBusTransport, 1);
    machine->latest_feedback.pos_counts[0] = recovered;
    const auto result = reset(1);
    assert(result.accepted);
    assert(machine->control_policy.home_valid_mask.load() == (recovered == 102 ? 2u : 3u));
    assert(machine->execution_fault_reasons.load() == 0);
    assert(machine->fault_reset_request.load() == 0);
    assert(machine->verified_configuration_epoch.load() != machine->configuration_epoch.load());
    assert(!machine->armed.load() && machine->axis_enable_mask.load() == 0);
  }
  setup();
  machine->execution_fault(FaultCycleDeadline, 3);
  machine->execution_fault(FaultBusTransport, 1);
  machine->bus_ready.store(false);
  const auto persistent = reset(3);
  assert(!persistent.accepted && persistent.fault_persists);
  assert(machine->execution_fault_reasons.load() == (FaultCycleDeadline | FaultBusTransport));
  assert(machine->control_policy.home_valid_mask.load() == 3);
  assert(machine->fault_reset_request.load() == 0);
  setup();
  machine->recovery_trusted_mask = 1;
  machine->execution_fault(FaultCoordinateReference, 2);
  assert(reset(2).accepted);
  assert(machine->control_policy.home_valid_mask.load() == 1);
  assert(machine->verified_configuration_epoch.load() == 1);
  // Reproduce the recovered-but-latched state: qualified recovery has restored
  // the old Home and cleared untrusted provenance, without clearing the fault.
  setup();
  machine->recovery_trusted_mask = 1;
  machine->execution_fault(FaultCoordinateReference, 2);
  machine->recovery_trusted_mask = 3;
  machine->fault_evidence[6].untrusted_axis_mask = 0;
  machine->control_policy.home_valid_mask.store(3);
  const auto recovered_home_epoch = machine->home_epoch.load();
  assert(reset(2).accepted);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  assert(machine->home_epoch.load() == recovered_home_epoch);
  assert(machine->execution_fault_reasons.load() == 0);
  assert(machine->safety_fault_mask.load() == 0);
  assert(machine->verified_configuration_epoch.load() == 1);
  assert(!machine->armed.load() && machine->axis_enable_mask.load() == 0);
  // Current eligibility is refreshed independently of the last reset result.
  // No publication can turn a stale proof or a historical successful Reset
  // into permission; a UI reader sees the full-cell native decision.
  for (unsigned missing = 0; missing < 10; ++missing) {
    setup();
    machine->execution_fault_reasons.store(FaultCoordinateReference);
    machine->fault_evidence[6] = {2, 0, 2};
    machine->latest_feedback.pdo_observed_time_ns[0] = MachineState::cycle_clock();
    machine->latest_feedback.pdo_observed_time_ns[1] = MachineState::cycle_clock();
    machine->armed.store(false); machine->axis_enable_mask.store(0);
    machine->service_mode_axis_mask.store(0); machine->native_home_active_axis_mask.store(0);
    machine->commissioning_phase.store(0);
    if (missing == 1) machine->latest_feedback.pdo_observed_time_ns[0] = MachineState::cycle_clock() - 2 * options.cycle_ns - 1;
    if (missing == 2) machine->latest_feedback.pdo_observed_time_ns[0] = 0;
    if (missing == 3) machine->latest_feedback.pdo_observed_time_ns[0] = MachineState::cycle_clock() + 1;
    if (missing == 4) machine->armed.store(true);
    if (missing == 5) machine->axis_enable_mask.store(1);
    if (missing == 6) machine->service_mode_axis_mask.store(1);
    if (missing == 7) machine->native_home_active_axis_mask.store(1);
    if (missing == 8) machine->commissioning_phase.store(1);
    if (missing == 9) machine->motion_done.store(0);
    const auto old_reset_sequence = machine->recovery_state.reset_sequence;
    const auto old_reset_cleared = machine->recovery_state.reset.cleared;
    machine->publish_recovery();
    assert(machine->recovery_state.reset_preserves_home == (missing == 0));
    assert(machine->recovery_state.reset_sequence == old_reset_sequence);
    assert(machine->recovery_state.reset.cleared == old_reset_cleared);
    std::ostringstream json;
    machine->write_recovery_json(json);
    const std::string current = json.str().substr(0, json.str().find("\"reset_sequence\""));
    assert(current.find("\"rehome_axis_mask\":0") != std::string::npos);
    assert(current.find(missing ? "\"reset_preserves_home\":false" : "\"reset_preserves_home\":true") != std::string::npos);
    assert(json.str().substr(json.str().find("\"outcomes\"")).find("reset_preserves_home") == std::string::npos);
  }
  // A UI may have read the eligible snapshot before the next exchange loses
  // freshness. The command uses the current native input, not that advisory.
  for (unsigned invalid = 0; invalid < 4; ++invalid) {
    setup();
    machine->execution_fault_reasons.store(FaultCoordinateReference);
    machine->fault_evidence[6] = {2, 0, 2};
    machine->publish_recovery();
    assert(machine->recovery_state.reset_preserves_home);
    if (invalid == 0) machine->latest_feedback.pdo_fresh_axis_mask = 1;
    if (invalid == 1) machine->latest_feedback.pdo_observed_time_ns[1] = MachineState::cycle_clock() - 2 * options.cycle_ns - 1;
    if (invalid == 2) machine->latest_feedback.pdo_observed_time_ns[1] = 0;
    if (invalid == 3) machine->latest_feedback.pdo_observed_time_ns[1] = MachineState::cycle_clock() + 1;
    const auto home_before_lost_exchange = machine->home_epoch.load();
    assert(reset(3).fault_persists);
    assert(machine->execution_fault_reasons.load() == FaultCoordinateReference);
    assert(machine->control_policy.home_valid_mask.load() == 3);
    assert(machine->home_epoch.load() == home_before_lost_exchange);
    assert(!machine->recovery_state.reset_preserves_home);
  }
  machine->armed.store(false); machine->axis_enable_mask.store(0);
  machine->service_mode_axis_mask.store(0); machine->native_home_active_axis_mask.store(0);
  machine->commissioning_phase.store(0);
  // AMR-720: a target-lead trip with trusted coordinates keeps Home. Reset
  // waits for fresh feedback, then clears it without any rehome outcome.
  setup();
  machine->execution_fault(FaultTargetLead, 1);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  machine->latest_feedback.pdo_fresh_axis_mask = 2;
  assert(reset(1).fault_persists);
  assert(machine->execution_fault_reasons.load() == FaultTargetLead);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  machine->latest_feedback.pdo_fresh_axis_mask = 3;
  assert(reset(1).accepted);
  assert(machine->execution_fault_reasons.load() == 0);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  assert(machine->recovery_state.reset.rehome_required == 0);
  // Coordinate trust lost at the trip still invalidates that axis's Home.
  setup();
  machine->recovery_trusted_mask = 2;
  machine->execution_fault(FaultTargetLead, 1);
  assert(machine->control_policy.home_valid_mask.load() == 2);
  assert(reset(1).accepted);
  assert(machine->control_policy.home_valid_mask.load() == 2);
  assert(machine->recovery_state.reset.rehome_axis_masks[4] == 1);
  setup();
  machine->execution_fault(FaultDriveReadiness, 1);
  machine->latest_feedback.statusword[0] = 8;
  assert(reset(1).fault_persists);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  machine->latest_feedback.statusword[0] = 0x40;
  assert(reset(1).accepted);
  assert(machine->fault_reset_request.load() == 1);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  setup();
  machine->execution_fault(FaultCycleDeadline, 3);
  assert(reset(1).accepted);
  assert(machine->execution_fault_reasons.load() == FaultCycleDeadline);
  assert(machine->safety_fault_mask.load() == 2);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  assert(reset(2).accepted);
  assert(machine->execution_fault_reasons.load() == 0);
  setup();
  machine->configuration_readbacks.publish({1, 3, 0, machine->verification_request.load()});
  machine->coordinate_reference[1].valid = true;
  machine->coordinate_reference[1].identity = {1, 1, 1};
  machine->require_reverification();
  machine->consume_control_requests();
  assert(machine->verified_configuration_epoch.load() == 0);
  assert(machine->control_policy.config_verified_mask.load() == 0);
  assert(machine->configuration_epoch.load() == 1);
  assert(machine->control_policy.home_valid_mask.load() == 3);
  assert(machine->reverification_coordinate_mask == 2);
  machine->configuration_readbacks.publish({1, 3, 0, machine->verification_request.load()});
  machine->consume_control_requests();
  assert(machine->verified_configuration_epoch.load() == 1);
  assert(machine->control_policy.config_verified_mask.load() == 3);
  assert(machine->reverification_coordinate_mask == 0);
  setup();
  machine->armed.store(true);
  machine->axis_enable_mask.store(3);
  assert(!reset(3).accepted);
  MachineState::ControlRequest legacy{MachineState::ControlOperation::ResetFault, 3};
  legacy.legacy_reset = true;
  assert(machine->apply_control(legacy).accepted);
  assert(machine->control_policy.home_valid_mask.load() == 0);
  assert(machine->fault_reset_request.load() == 3);
  assert(!machine->armed.load() && machine->axis_enable_mask.load() == 0);
  MachineState::cycle_clock = real_clock;
  MachineState::cycle_owner = nullptr;
}
