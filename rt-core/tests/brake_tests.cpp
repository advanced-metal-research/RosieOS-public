#include "engine/brake.hpp"
#include "output_policy.hpp"
#include "simulated_bus.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>

using namespace rosie::rtcore;
using namespace gradient::ipc::v1;
using State = gradient::ds402::State;

static void delay_boundaries() {
  BrakeConfig config{true, 100000000, 100000000, true, false};
  BrakeMachine machine;
  assert(machine.step(State::SwitchOnDisabled, 0, config).state == BRAKE_APPLIED);
  auto out = machine.step(State::OperationEnabled, 1, config);
  assert(out.state == BRAKE_WAIT_RELEASE_DELAY && !out.motion_permitted);
  out = machine.step(State::OperationEnabled, 100000000, config);
  assert(out.state == BRAKE_WAIT_RELEASE_DELAY && !out.motion_permitted);
  out = machine.step(State::OperationEnabled, 100000001, config);
  assert(out.state == BRAKE_RELEASED && out.motion_permitted);
  out = machine.step(State::SwitchOnDisabled, 100000002, config);
  assert(out.state == BRAKE_WAIT_HOLD_DELAY && out.hold_window_active && !out.motion_permitted);
  out = machine.step(State::SwitchOnDisabled, 200000001, config);
  assert(out.state == BRAKE_WAIT_HOLD_DELAY && out.hold_window_active);
  out = machine.step(State::SwitchOnDisabled, 200000002, config);
  assert(out.state == BRAKE_APPLIED && !out.hold_window_active && !out.motion_permitted);
}

static void reenable_and_feedback() {
  BrakeConfig config{true, 10, 20, false, true};
  BrakeMachine machine;
  assert(machine.step(State::OperationEnabled, 0, config).state == BRAKE_UNKNOWN);
  assert(machine.step(State::OperationEnabled, 1, config, true).state == BRAKE_WAIT_RELEASE_DELAY);
  assert(machine.step(State::OperationEnabled, 10, config, false).state == BRAKE_APPLIED);
  assert(machine.step(State::OperationEnabled, 11, config, true).motion_permitted);
  assert(!machine.step(State::OperationEnabled, 12, config, false).motion_permitted);
  auto out = machine.step(State::SwitchOnDisabled, 13, config, false);
  assert(out.state == BRAKE_APPLIED && out.hold_window_active);
  out = machine.step(State::SwitchOnDisabled, 33, config, true);
  assert(out.state == BRAKE_RELEASED && !out.motion_permitted && !out.hold_window_active);
  assert(machine.step(State::OperationEnabled, 34, config, true).state == BRAKE_WAIT_RELEASE_DELAY);
  assert(machine.step(State::OperationEnabled, 44, config, true).motion_permitted);
  assert(machine.step(State::SwitchOnDisabled, 45, config, true).hold_window_active);
  out = machine.step(State::OperationEnabled, 46, config, true);
  assert(out.state == BRAKE_WAIT_RELEASE_DELAY && !out.hold_window_active);
  assert(!machine.step(State::OperationEnabled, 55, config, true).motion_permitted);
  assert(machine.step(State::OperationEnabled, 56, config, true).motion_permitted);
  assert(!machine.step(State::OperationEnabled, 57, config).motion_permitted);
  assert(!machine.step(State::OperationEnabled, 2, config, true).motion_permitted);
  config.release_delay_ns = config.hold_delay_ns = 0;
  BrakeMachine immediate;
  assert(immediate.step(State::OperationEnabled, 0, config, true).motion_permitted);
  assert(!immediate.step(State::SwitchOnDisabled, 1, config, false).hold_window_active);
  BrakeConfig absent;
  for (auto state : {State::Unknown, State::Fault, State::OperationEnabled, State::SwitchOnDisabled}) {
    out = immediate.step(state, 100, absent, false);
    assert(out.state == BRAKE_RELEASED && out.motion_permitted && !out.hold_window_active);
  }
}

static void hold_warnings() {
  BrakeConfig config{true, 100000000, 100000000, false, false};
  DisabledBrakeGuard guard;
  BrakeTransitionInput sample{};
  const auto at = [&](uint64_t t) { sample.observed_ns = sample.now_ns = t; return sample; };
  BrakeOutput holding{BRAKE_WAIT_HOLD_DELAY, false, true}, applied{BRAKE_APPLIED, false, false};
  const auto observe = [&](const BrakeOutput& brake, State drive, int32_t p, uint64_t t,
                           bool fresh = true) {
    return guard.observe(config, brake, drive, fresh, p, 0, false, at(t));
  };
  assert(!observe({}, State::OperationEnabled, 100, 0));
  // Energized hold window: the drive still has torque, so nothing is judged.
  assert(!observe(holding, State::SwitchOnDisabled, 101, 1000000));
  assert(!observe(holding, State::SwitchOnDisabled, 140, 2000000));
  assert(guard.anchor == 100 && !guard.energized_updated);
  // Torque off re-anchors at the rest and records the energized excursion.
  assert(!observe(applied, State::SwitchOnDisabled, 140, 3000000));
  assert(guard.anchor == 140 && guard.energized_updated && guard.energized.peak_displacement == 40);
  assert(!observe(applied, State::SwitchOnDisabled, 141, 4000000));
  assert(observe(applied, State::SwitchOnDisabled, 142, 5000000));
  // One warning per settle: the same displacement does not warn again.
  assert(guard.resettling && guard.warning.sequence == 1);
  assert(!observe(applied, State::SwitchOnDisabled, 142, 6000000));
  assert(!observe({}, State::OperationEnabled, 100, 7000000));
  assert(!guard.resettling && guard.anchor == 100);
  assert(!observe(applied, State::SwitchOnDisabled, 100, 8000000));
  // A nongravity axis is judged only for one hold delay after torque-off.
  assert(!observe(applied, State::SwitchOnDisabled, 102, 108000000));
  config.gravity_axis = true;
  assert(observe(applied, State::SwitchOnDisabled, 102, 109000000));
  assert(!observe(holding, State::SwitchOnDisabled, 2100, 110000000, false));
  config.present = false;
  assert(!observe(holding, State::SwitchOnDisabled, 2100, 111000000));
  // An axis without a brake records no energized window at torque-off.
  { DisabledBrakeGuard unbraked;
    assert(!unbraked.observe(config, {}, State::OperationEnabled, true, 0, 0, false, at(1)));
    assert(!unbraked.observe(config, applied, State::SwitchOnDisabled, true, 0, 0, false, at(2)));
    assert(!unbraked.energized_updated && unbraked.energized.torque_off_ns == 0); }
  config.present = true;
  assert(!observe({}, State::OperationEnabled, 100, 112000000));
  assert(!observe(applied, State::SwitchOnDisabled, 100, 113000000));

  AxisOutputConfig output_config{};
  output_config.counts_per_unit = 1000;
  output_config.sign = 1;
  output_config.max_profile_velocity = 1000;
  output_config.limits = {-10, 10, 1, false, true};
  output_config.limits.following_error = 1;
  output_config.limits.following_error_timeout_ns = 100000000;
  AxisOutputInput input{};
  input.statusword = 0x227;
  input.feedback_position = 100;
  input.feedback_fresh = input.config_verified = input.configuration_current = true;
  input.armed = input.enabled = input.setpoint_present = true;
  input.target_position = 1000;
  input.brake_motion_permitted = false;
  input.now_ns = 1000000;
  const auto waiting = evaluate_axis_output(output_config, input, {});
  assert(waiting.target_position == 100 && waiting.target_velocity == 0);
  assert(!waiting.fault && waiting.controlword == gradient::ds402::CW_ENABLE_OP);
  input.statusword = 0x240;
  input.armed = input.enabled = false;
  // The output policy has no brake-hold input: a warning cannot fault it.
  const auto inhibited = evaluate_axis_output(output_config, input, waiting.next);
  assert(!inhibited.fault && !inhibited.fault_reasons && inhibited.controlword == 0);
  assert(inhibited.reference.disposition == ReferenceDisposition::Reconciled && !inhibited.invalidate_home);
  input.service_active = true;
  input.service_mode = 6;
  input.service_controlword = 0x1f;
  assert(evaluate_axis_output(output_config, input, {}).controlword == 0x0f);
  input.brake_motion_permitted = true;
  assert(evaluate_axis_output(output_config, input, {}).controlword == 0x1f);
}

// Bench resting spread must fit the profile tolerance. The first cumulative
// count beyond it warns even when every incremental change is only one count.
static void measured_hold_tolerance_and_home_frame() {
  BrakeConfig config{true, 100000000, 100000000, true, false, 16};
  BrakeOutput holding{BRAKE_APPLIED, false, false};
  DisabledBrakeGuard guard;
  assert(!guard.observe(config, {}, State::OperationEnabled, true, 65536, 4));
  assert(!guard.observe(config, holding, State::SwitchOnDisabled, true, 65536, 4));
  assert(guard.energized_updated && guard.anchor == 65536); // Torque off: the hold anchor.
  for (int32_t count = 65537; count <= 65552; ++count)
    assert(!guard.observe(config, holding, State::SwitchOnDisabled, true, count, 4));
  assert(guard.anchor == 65536);
  assert(guard.observe(config, holding, State::SwitchOnDisabled, true, 65553, 4));
  // A new origin is not displacement; the next 17 counts in that frame are.
  assert(!guard.observe(config, holding, State::SwitchOnDisabled, true, 100, 5));
  assert(guard.anchor == 100 && guard.anchor_epoch == 5 && !guard.resettling);
  assert(!guard.observe(config, holding, State::SwitchOnDisabled, true, 84, 5));
  assert(guard.observe(config, holding, State::SwitchOnDisabled, true, 83, 5));
  assert(!guard.observe(config, holding, State::SwitchOnDisabled, true, 65536, 5, true));
  assert(!guard.observe(config, holding, State::SwitchOnDisabled, true, 65481, 5, true));
  assert(guard.anchor == 65481);
  assert(!guard.observe(config, holding, State::SwitchOnDisabled, true, 65465, 5));
  assert(guard.observe(config, holding, State::SwitchOnDisabled, true, 65464, 5));
  // Stale data cannot reset an anchor, even across a frame change.
  assert(!guard.observe(config, holding, State::SwitchOnDisabled, false, 0, 6, true));
  assert(guard.anchor == 65481 && guard.anchor_epoch == 5);
}

// Capture once at Arm before drive enable/brake release; never chase the
// measured position throughout the release delay (the integrated-CSP invariant).
static void enable_pose_survives_brake_release() {
  AxisOutputConfig config{};
  config.counts_per_unit = 100000;
  config.sign = 1;
  config.max_profile_velocity = 100000;
  config.limits = {-10, 10, 1, false, true};
  config.limits.following_error = .01;
  config.limits.following_error_timeout_ns = 100000000;
  config.coordinate_evidence_required = true;
  AxisOutputInput input{};
  input.statusword = 0x240;
  input.feedback_position = input.continuous_feedback = 1000;
  input.feedback_fresh = input.continuous_feedback_valid = true;
  input.config_verified = input.configuration_current = true;
  input.armed = input.enabled = true;
  input.brake_motion_permitted = false;
  input.commanded_mode = 8;
  input.now_ns = 1000000;
  AxisOutputState state{};
  input.previous_target_valid = true;
  input.previous_target = 500; // Valid disabled displacement must reconcile, not fault rate.
  state.held_target = 500; // Old disabled observation is not the enable capture.
  state.held_target_valid = true;
  auto out = evaluate_axis_output(config, input, state);
  assert(!out.fault && out.target_position == 1000 && out.next.enable_hold_latched);
  for (auto status : {0x221, 0x223, 0x227, 0x227}) {
    input.statusword = status;
    input.feedback_position = input.continuous_feedback += 2;
    input.previous_target_valid = true;
    input.previous_target = out.target_position;
    input.previous_cycle_ns = input.now_ns;
    input.now_ns += 1000000;
    out = evaluate_axis_output(config, input, out.next);
    assert(!out.fault && out.target_position == 1000 && out.target_velocity == 0);
    assert(out.next.enable_hold_latched);
  }
  input.previous_cycle_ns = input.now_ns; input.now_ns += 1000000;
  input.brake_motion_permitted = true;
  out = evaluate_axis_output(config, input, out.next);
  assert(!out.fault && out.target_position == 1000 && out.next.enable_hold_latched);
  input.previous_cycle_ns = input.now_ns; input.now_ns += 1000000;
  input.setpoint_present = true;
  input.target_position = 1001;
  out = evaluate_axis_output(config, input, out.next);
  assert(!out.fault && out.target_position == 1001);
  input.previous_cycle_ns = input.now_ns; input.now_ns += 1000000;
  input.setpoint_present = false;
  // Completion holds the final trajectory command even with differing feedback.
  auto completed = input;
  completed.previous_target = 1001;
  const auto held = evaluate_axis_output(config, completed, out.next);
  assert(!held.fault && held.target_position == 1001 && held.next.enable_hold_latched);
  // A qualified jog-stop snap still captures measured feedback once.
  auto stopped_jog = completed;
  stopped_jog.snap_hold = true;
  const auto snapped = evaluate_axis_output(config, stopped_jog, out.next);
  assert(!snapped.fault && snapped.target_position == stopped_jog.continuous_feedback);
  // A concurrent motion target must not borrow that snap to bypass rate checks.
  stopped_jog.setpoint_present = true;
  stopped_jog.target_position = 100000;
  const auto invalid_motion = evaluate_axis_output(config, stopped_jog, out.next);
  assert(invalid_motion.fault && (invalid_motion.fault_reasons & FaultPositionRate));
  assert(invalid_motion.controlword == 0 && !invalid_motion.next.enable_hold_latched);
  for (unsigned loss = 0; loss != 9; ++loss) {
    auto denied = input;
    if (loss == 0) denied.armed = false;
    if (loss == 1) denied.enabled = false;
    if (loss == 2) denied.feedback_fresh = false;
    if (loss == 3) denied.continuous_feedback_valid = false;
    if (loss == 4) denied.configuration_current = false;
    if (loss == 5) denied.config_verified = false;
    if (loss == 6) denied.statusword = 0x248;
    if (loss == 7) denied.service_active = true;
    if (loss == 8) denied.startup_passive = true;
    const auto stopped = evaluate_axis_output(config, denied, out.next);
    assert(!stopped.next.enable_hold_latched);
    if (loss == 0 || loss == 1 || loss == 5) assert(stopped.controlword == 0);
  }
  input.armed = input.enabled = false;
  input.statusword = 0x240;
  out = evaluate_axis_output(config, input, out.next);
  assert(out.controlword == 0 && out.target_position == input.feedback_position);
  assert(!out.next.enable_hold_latched);
  input.feedback_position = input.continuous_feedback = 1100;
  input.previous_cycle_ns = input.now_ns; input.now_ns += 1000000;
  input.armed = input.enabled = true;
  input.brake_motion_permitted = false;
  out = evaluate_axis_output(config, input, out.next);
  assert(!out.fault && out.target_position == 1100 && out.next.enable_hold_latched);
}

// Recorded the 1430 cell legacy J1-J3 class: 17 counts against the 16-count anchor
// tolerance, 13-58 ms after disarm while the drive still held torque through
// its 100 ms hold delay: servo settling, not brake. That energized movement is
// recorded and never judged; torque-off re-anchors at the rest, brake take-up
// is then judged from it, and one warning re-anchors after a 100 ms quiet dwell.
static void legacy_energized_settle_is_recorded_and_takeup_is_judged() {
  BrakeConfig config{true, 100000000, 100000000, false, false, 16};
  BrakeMachine machine;
  DisabledBrakeGuard guard;
  BrakeTransitionInput input{};
  const auto at = [&](uint64_t t) { input.observed_ns = input.now_ns = t; return input; };
  const auto enabled = machine.step(State::OperationEnabled, 0, config);
  assert(!guard.observe(config, enabled, State::OperationEnabled, true, 1000, 0, false, at(0)));
  // CW0 is received at 1 ms; the axis reaches +17 counts 13 ms after it.
  for (uint64_t ms = 1; ms <= 100; ++ms) {
    const uint64_t t = ms * 1000000;
    const auto brake = machine.step(State::SwitchOnDisabled, t, config);
    assert(brake.hold_window_active && !brake.motion_permitted);
    const int32_t p = 1000 + int32_t(std::min<uint64_t>(17, (ms - 1) * 17 / 13));
    assert(!guard.observe(config, brake, State::SwitchOnDisabled, true, p, 0, false, at(t)));
    assert(guard.anchor == 1000 && !guard.energized_updated && guard.warning.sequence == 0);
  }
  const auto off = machine.step(State::SwitchOnDisabled, 101000000, config);
  assert(!off.hold_window_active && off.state == BRAKE_APPLIED);
  assert(!guard.observe(config, off, State::SwitchOnDisabled, true, 1017, 0, false, at(101000000)));
  assert(guard.energized_updated && guard.anchor == 1017 && guard.warning.sequence == 0);
  assert(guard.energized.peak_displacement == 17 && guard.energized.torque_off_anchor == 1017 &&
         guard.energized.torque_off_ns == 101000000);
  // Brake take-up after torque off: 16 counts tolerated, 17 warn from 1017.
  assert(!guard.observe(config, off, State::SwitchOnDisabled, true, 1033, 0, false, at(102000000)));
  assert(guard.observe(config, off, State::SwitchOnDisabled, true, 1034, 0, false, at(103000000)));
  assert(guard.resettling && guard.warning.sequence == 1 && guard.warning.anchor == 1017);
  assert(guard.warning.cause == BrakeGuardFailure::DisabledDisplacement);
  assert(!guard.warning.torque_on && guard.warning.peak_displacement == 17);
  assert(guard.warning.same_direction_count == 1 && !guard.warning.slip);
  uint64_t t = 104000000;
  for (; guard.resettling; t += 1000000)
    assert(!guard.observe(config, machine.step(State::SwitchOnDisabled, t, config),
        State::SwitchOnDisabled, true, 1034, 0, false, at(t)));
  assert(t == 204000000 && guard.anchor == 1034 && guard.warning.sequence == 1);
  assert(guard.warning.settled && guard.warning.settled_displacement == 17);
  // Take-up has ended: a nongravity axis is no longer judged.
  const auto applied = machine.step(State::SwitchOnDisabled, t, config);
  assert(!guard.observe(config, applied, State::SwitchOnDisabled, true, 1051, 0, false, at(t)));
  // A gravity axis keeps checking the new rest; 16 more counts are tolerated.
  config.gravity_axis = true;
  assert(guard.observe(config, applied, State::SwitchOnDisabled, true, 1051, 0, false, at(t + 1000000)));
  assert(guard.warning.sequence == 2 && guard.warning.anchor == 1034 && guard.warning.same_direction_count == 2);
  // Continued travel while resettling warns again per 16 counts rather than
  // silently following a slipping brake; the third such warning in one braked
  // period is suspected slip, totalled from the first anchor of the run.
  assert(!guard.observe(config, applied, State::SwitchOnDisabled, true, 1067, 0, false, at(t + 2000000)));
  assert(guard.observe(config, applied, State::SwitchOnDisabled, true, 1068, 0, false, at(t + 3000000)));
  assert(guard.warning.sequence == 3 && guard.warning.anchor == 1051 && guard.resettling);
  assert(guard.warning.slip && guard.warning.same_direction_count == 3 &&
         guard.warning.same_direction_displacement == 51);
}

static void simulated_timing_and_sag() {
  auto* master = ecrt_request_master(0);
  auto* domain = ecrt_master_create_domain(master);
  std::array<ec_slave_config_t*, 3> axes{};
  for (unsigned i = 0; i < axes.size(); ++i) {
    auto* axis = axes[i] = ecrt_master_slave_config(master, 0, i, 1, 1);
    axis->entries = {{{0x6040, 0, 16}, 2}, {{0x607a, 0, 32}, 2}, {{0x6041, 0, 16}, 3}};
    for (auto& entry : axis->entries) assert(sim_register(entry, domain) >= 0);
    axis->brake = {i < 2, i == 0 ? 100000000u : 200000000u, i == 0 ? 100000000u : 0u, true, false};
    EC_WRITE_U16(sim_object(axis, 0x6040), 15);
    EC_WRITE_S32(sim_object(axis, 0x607a), 77);
  }
  axes[0]->brake_signal_index = 0x6041;
  axes[0]->brake_signal_bit = 10;
  master->faults.brake_sag_counts = 2000;
  master->active = true;
  assert(ecrt_domain_queue(domain) == 0 && ecrt_master_send(master) == 0);
  sim_step_plant_at(master, 10);
  assert(axes[0]->position_counts == 0 && axes[1]->position_counts == 0 && axes[2]->position_counts == 77);
  sim_step_plant_at(master, 100000009);
  assert(axes[0]->brake_applied && axes[0]->position_counts == 0);
  sim_step_plant_at(master, 100000010);
  assert(!axes[0]->brake_applied && axes[0]->position_counts == 77 && axes[1]->position_counts == 0);
  sim_present_domain_inputs(domain);
  assert(EC_READ_U16(sim_object(axes[0], 0x6041)) == 0x627);
  sim_step_plant_at(master, 200000010);
  assert(!axes[1]->brake_applied && axes[1]->position_counts == 77);
  for (auto* axis : axes) EC_WRITE_U16(sim_object(axis, 0x6040), 0);
  assert(ecrt_domain_queue(domain) == 0 && ecrt_master_send(master) == 0);
  sim_step_plant_at(master, 200000011);
  assert(!axes[0]->brake_applied && axes[0]->position_counts == 2077);
  assert(axes[1]->brake_applied && axes[1]->position_counts == 77);
  assert(!axes[2]->brake_applied && axes[2]->position_counts == 2077);
  sim_step_plant_at(master, 300000010);
  assert(!axes[0]->brake_applied && axes[0]->position_counts == 2077);
  sim_step_plant_at(master, 300000011);
  assert(axes[0]->brake_applied && axes[0]->position_counts == 2077);
  sim_present_domain_inputs(domain);
  assert(EC_READ_U16(sim_object(axes[0], 0x6041)) == 0x240);
  ecrt_release_master(master);
}


// Opt-in diagnostic settings: a fixed target corridor, never a learned tolerance.
// Nanoseconds are shortened for deterministic policy tests; counts are J4 deltas.
struct TransitionFixture {
  BrakeConfig config{true, 100, 100, false, false, 16, {327, 6553, 100, 100, 500}};
  DisabledBrakeGuard guard;
  BrakeOutput output{};
  BrakeTransitionInput input{};
  uint64_t last_t = 0, last_epoch = 1;
  State last_drive = State::Unknown;
  int32_t last_p = 0;
  BrakeHoldCommand last_command{};
  bool last_holding = false, last_fresh = true, last_home = false;
  int32_t velocity = 0; // Drive velocity reported on every sample, counts/s.
  bool sample(uint64_t t, State drive, int32_t p, BrakeHoldCommand command,
              bool holding = false, bool fresh = true, uint64_t epoch = 1, bool home = false) {
    // The real owner calls once per fresh bus cycle. Fill unchanged intervening
    // ticks rather than treating two distant observations as a stationary proof.
    while (last_t && last_t + 1 < t) {
      if (sample(last_t + 1, last_drive, last_p, last_command, last_holding,
          last_fresh, last_epoch, last_home)) return true;
    }
    last_t=t; last_drive=drive; last_p=p; last_command=command;
    last_holding=holding; last_fresh=fresh; last_epoch=epoch; last_home=home;
    input = {t, t, 1, command, true, velocity, 3};
    output = {drive == State::OperationEnabled ? BRAKE_RELEASED :
        holding ? BRAKE_WAIT_HOLD_DELAY : BRAKE_APPLIED, false, holding};
    const bool warned = guard.observe(config, output, drive, fresh, p, epoch, home, input);
    guard.qualify(config, output);
    return warned;
  }
  void enabled(int32_t target = 22) {
    assert(!sample(1, State::OperationEnabled, 0, {true, true, true, target}));
    assert(!sample(102, State::OperationEnabled, 0, {true, true, true, target}));
  }
  void held() {
    enabled();
    assert(!sample(103, State::SwitchOnDisabled, 0, {true, false, false, 0}, true));
    assert(!sample(116, State::SwitchOnDisabled, 17, {}, true));
    assert(output.state == BRAKE_WAIT_HOLD_DELAY);
    assert(!sample(150, State::SwitchOnDisabled, 31, {}, true));
    assert(!sample(203, State::SwitchOnDisabled, 16, {}));
    assert(output.state == BRAKE_WAIT_HOLD_DELAY);
    assert(!sample(302, State::SwitchOnDisabled, 15, {}));
    assert(output.state == BRAKE_WAIT_HOLD_DELAY);
    assert(!sample(303, State::SwitchOnDisabled, 16, {}));
    assert(output.state == BRAKE_APPLIED && guard.anchor == 16);
  }
};

static void startup_alarm_keeps_fixed_hold_without_brake_misclassification() {
  TransitionFixture f;
  assert(gradient::ds402::decode_statusword(0x1618) == State::Fault);
  assert(!f.sample(1, State::Fault, 100, {}));
  assert(f.guard.transition.startup_reset_eligible());
  assert(f.output.state == BRAKE_UNKNOWN && !f.output.motion_permitted);
  BrakeHoldCommand reset{}; reset.valid = true; reset.controlword = 128;
  assert(!f.sample(2, State::Fault, 106, reset));
  assert(!f.sample(3, State::NotReady, 111, {}));
  assert(f.guard.anchor == 100 && f.output.state == BRAKE_UNKNOWN);
  assert(!f.sample(4, State::SwitchOnDisabled, 116, {}));
  assert(f.output.state == BRAKE_APPLIED && !f.guard.transition.startup_reset_eligible());
  assert(f.sample(5, State::SwitchOnDisabled, 117, {}));
  assert(f.guard.transition.failure == BrakeGuardFailure::DisabledDisplacement);
  // Alarm progression may not follow or erase cumulative disabled movement.
  TransitionFixture sag;
  assert(!sag.sample(1, State::Fault, 0, {}));
  assert(!sag.sample(2, State::FaultReactionActive, 8, {}));
  assert(sag.sample(3, State::NotReady, 17, {}));
  assert(sag.guard.anchor == 0 && sag.guard.transition.failure == BrakeGuardFailure::DisabledDisplacement);
  for (int kind=0; kind<3; ++kind) {
    TransitionFixture bad;
    assert(!bad.sample(1, State::Fault, 0, {}));
    // Stale feedback (kind 0) and a frame change (kind 1) are not brake
    // evidence: no warning, but the restarted guard ends the startup witness.
    assert(!bad.sample(2, State::Fault, 0, {}, false, kind != 0, kind == 1 ? 2 : 1));
    if (kind != 2) {
      assert(bad.guard.warning.sequence == 0 && !bad.guard.resettling);
      assert(!bad.guard.transition.startup_reset_eligible());
    }
    if (kind==2) {
      bad.output = {BRAKE_RELEASED,false,false};
      bad.input.observed_ns = bad.input.now_ns = 3;
      assert(bad.guard.observe(bad.config,bad.output,State::Fault,true,0,1,false,bad.input));
      assert(bad.guard.transition.failure == BrakeGuardFailure::BrakeFeedback);
    }
  }
  // Any submitted enable sequence, even CW6 before OP, ends startup recovery.
  // The resulting drive-state detection is the drive alarm's report, not a
  // brake warning: the guard restarts quietly and re-anchors once inactive.
  for (uint16_t cw : {uint16_t{6},uint16_t{7},uint16_t{15},uint16_t{31}}) {
    TransitionFixture enabled;
    assert(!enabled.sample(1, State::SwitchOnDisabled, 0, {}));
    BrakeHoldCommand command{}; command.valid=true; command.controlword=cw;
    assert(!enabled.sample(2, State::Fault, 0, command));
    assert(enabled.guard.warning.sequence == 0 && !enabled.guard.transition.startup_reset_eligible());
    assert(!enabled.sample(3, State::SwitchOnDisabled, 0, {}) && !enabled.guard.resettling);
    assert(enabled.guard.transition.phase == BrakeGuardPhase::Held && enabled.guard.anchor == 0);
  }
  TransitionFixture lifecycle; lifecycle.enabled();
  assert(!lifecycle.sample(103, State::Fault, 0, {}));
  assert(lifecycle.guard.warning.sequence == 0);
  // Real disabled sag after the restart is still a brake warning.
  assert(!lifecycle.sample(104, State::SwitchOnDisabled, 0, {}));
  assert(lifecycle.sample(105, State::SwitchOnDisabled, 17, {}));
  assert(lifecycle.guard.warning.cause == BrakeGuardFailure::DisabledDisplacement);
}

static void opt_in_closing_settling_and_permanent_hold() {
  // Neither policy judges 17 counts in the energized window. The opt-in guard
  // settles from the torque-off anchor and later checks hold even on
  // nongravity axes.
  DisabledBrakeGuard legacy;
  BrakeConfig old{true,100,100,false,false,16};
  assert(!legacy.observe(old, {}, State::OperationEnabled, true, 0));
  assert(!legacy.observe(old, {BRAKE_WAIT_HOLD_DELAY,false,true}, State::SwitchOnDisabled,true,17));
  TransitionFixture f;
  f.held();
  assert(f.guard.energized_updated == false && f.guard.energized.torque_off_ns == 203);
  assert(f.guard.energized.peak_displacement == 31 && f.guard.energized.torque_off_anchor == 16);
  for (int d = 1; d <= 16; ++d)
    assert(!f.sample(303 + d, State::SwitchOnDisabled, 16 + d, {}));
  assert(f.sample(320, State::SwitchOnDisabled, 33, {}));
  assert(f.guard.anchor == 16); // No per-sample reanchor can erase cumulative sag.
  assert(f.guard.warning.cause == BrakeGuardFailure::DisabledDisplacement);
  assert(f.guard.warning.phase == BrakeGuardPhase::Held && !f.guard.warning.torque_on);
  // The warning holds no authority: re-enable starts a fresh lifecycle at once.
  assert(!f.sample(321, State::OperationEnabled, 16, {true,true,true,16}));
  assert(!f.guard.resettling && f.guard.transition.phase == BrakeGuardPhase::Enabled);
  assert(f.guard.warning.settled && f.guard.warning.sequence == 1);
}

// The energized window judges nothing: beyond the 16-count corridor, the
// 327-count cap and 6553 counts/s it only records the excursion. After torque
// off the corridor is +/-16 around the torque-off anchor and both caps apply
// on every sample; the last servo target no longer widens anything.
static void opt_in_closing_is_judged_from_the_torque_off_anchor() {
  for (int p : {-17, 39, 328}) {
    TransitionFixture f; f.enabled();
    f.velocity = 6554;
    assert(!f.sample(103, State::SwitchOnDisabled, p, {}, true));
    assert(!f.sample(202, State::SwitchOnDisabled, p, {}, true));
    assert(f.guard.transition.phase == BrakeGuardPhase::Closing && f.guard.warning.sequence == 0);
    assert(f.guard.transition.energized.peak_displacement == p &&
           f.guard.transition.energized.peak_velocity == 6554);
    f.velocity = 0;
    assert(!f.sample(203, State::SwitchOnDisabled, p, {}));
    assert(f.guard.energized_updated && f.guard.energized.torque_off_anchor == p);
    assert(f.guard.transition.phase == BrakeGuardPhase::Settling && f.guard.anchor == p);
    for (int d : {16, -16}) {
      TransitionFixture g = f;
      assert(!g.sample(204, State::SwitchOnDisabled, p + d, {}));
      assert(g.sample(205, State::SwitchOnDisabled, p + d + (d > 0 ? 1 : -1), {}));
      assert(g.guard.warning.cause == BrakeGuardFailure::Corridor && !g.guard.warning.torque_on);
      assert(g.guard.warning.phase == BrakeGuardPhase::Settling && g.guard.warning.anchor == p);
    }
  }
  // Capped velocity applies from the torque-off sample itself.
  { TransitionFixture f; f.enabled();
    assert(!f.sample(103, State::SwitchOnDisabled, 0, {}, true));
    assert(!f.sample(202, State::SwitchOnDisabled, 0, {}, true));
    f.velocity = -6554;
    assert(f.sample(203, State::SwitchOnDisabled, 0, {}));
    assert(f.guard.warning.cause == BrakeGuardFailure::VelocityCap && f.guard.warning.peak_velocity == -6554);
    assert(f.guard.warning.phase == BrakeGuardPhase::Settling && f.guard.energized_updated); }
  // Missing velocity is still a refusal of evidence, even while energized.
  { TransitionFixture f; f.enabled();
    f.input = {103,103,1,{},false,0,3};
    assert(f.guard.observe(f.config,{BRAKE_WAIT_HOLD_DELAY,false,true},State::SwitchOnDisabled,
        true,0,1,false,f.input));
    assert(f.guard.warning.cause == BrakeGuardFailure::VelocityUnavailable); }
}

static void opt_in_time_frame_home_and_unknown_guards() {
  { TransitionFixture f;
    assert(!f.sample(1,State::SwitchOnDisabled,0,{},false,false));
    assert(f.output.state == BRAKE_UNKNOWN && !f.output.motion_permitted); }
  // Observation-time, stale-feedback and frame causes are not brake evidence.
  // They never warn; the guard restarts quietly and withholds motion until a
  // fresh, applied sample anchors it (inside a hold window it keeps waiting).
  for (auto input : {BrakeTransitionInput{103,102,1,{},true,0,3},
                     BrakeTransitionInput{103,105,1,{},true,0,3},
                     BrakeTransitionInput{102,103,1,{},true,0,3}}) {
    TransitionFixture f; f.enabled();
    BrakeOutput hold{BRAKE_WAIT_HOLD_DELAY,false,true};
    assert(!f.guard.observe(f.config,hold,State::SwitchOnDisabled,true,0,1,false,input));
    f.guard.qualify(f.config, hold);
    assert(f.guard.warning.sequence == 0 && !hold.motion_permitted);
    assert(f.guard.transition.phase == BrakeGuardPhase::Dormant);
  }
  { TransitionFixture f; f.enabled();
    assert(!f.sample(103,State::SwitchOnDisabled,0,{},true,false));
    assert(f.guard.warning.sequence == 0 && f.output.state == BRAKE_UNKNOWN && !f.output.motion_permitted); }
  for (bool held : {true,false}) {
    TransitionFixture f;
    if (held) f.held(); else f.enabled();
    assert(!f.sample(400,State::SwitchOnDisabled,0,{},false,true,2));
    assert(f.guard.warning.sequence == 0 && f.guard.transition.phase == BrakeGuardPhase::Held);
    assert(f.guard.anchor_epoch == 2 && f.guard.anchor == 0);
  }
  { TransitionFixture f; f.enabled();
    assert(!f.sample(103,State::SwitchOnDisabled,0,{},true));
    // A 17-count spread inside the +/-16 torque-off corridor never qualifies quiet.
    for (uint64_t t=203;t<603;t+=50)
      assert(!f.sample(t,State::SwitchOnDisabled,t==203 ? 0 : ((t-203)/50)%2 ? 9 : -8,{}));
    assert(f.sample(603,State::SwitchOnDisabled,0,{}));
    assert(f.guard.warning.cause == BrakeGuardFailure::SettlingTimeout); }
  { TransitionFixture f; f.enabled();
    // Early APPLIED feedback does not shorten the declared closing interval.
    assert(!f.sample(103,State::SwitchOnDisabled,0,{}));
    assert(f.output.state == BRAKE_WAIT_HOLD_DELAY);
    f.input={110,110,1,{},true,0,3};
    // Missing bus cycles cannot qualify quiet time. The gap is an observation
    // fault, not brake evidence: no warning, and the guard re-anchors afresh.
    assert(!f.guard.observe(f.config,{BRAKE_APPLIED,false,false},State::SwitchOnDisabled,
        true,0,1,false,f.input));
    assert(f.guard.warning.sequence == 0 && f.guard.transition.phase == BrakeGuardPhase::Held);
  }
  for (uint32_t state : {uint32_t(BRAKE_UNKNOWN),uint32_t(BRAKE_RELEASED)}) {
    TransitionFixture f; f.held();
    f.input={304,304,1,{},true,0,3};
    assert(f.guard.observe(f.config,{state,false,false},State::SwitchOnDisabled,
        true,16,1,false,f.input));
  }
  { TransitionFixture f;
    assert(!f.sample(1,State::OperationEnabled,500,{true,true,false,500},false,true,1,true));
    assert(!f.sample(2,State::SwitchOnDisabled,1000,{},false,true,2,true));
    assert(f.guard.anchor == 1000);
    // After Home reconciliation the energized close is recorded, and take-up
    // is judged from the torque-off anchor in the new frame.
    assert(!f.sample(3,State::OperationEnabled,1000,{true,true,false,1200},false,true,2));
    assert(!f.sample(4,State::SwitchOnDisabled,1017,{},true,true,2));
    assert(!f.sample(104,State::SwitchOnDisabled,1017,{},false,true,2));
    assert(f.guard.anchor == 1017 && f.guard.energized.peak_displacement == 17);
    assert(f.sample(105,State::SwitchOnDisabled,1034,{},false,true,2));
    assert(f.guard.warning.cause == BrakeGuardFailure::Corridor && f.guard.warning.anchor == 1017); }
  { TransitionFixture f; f.enabled();
    f.input={103,103,1,{},true,0,3};
    // An unknown drive state is the drive's report; motion stays withheld.
    BrakeOutput unknown{BRAKE_UNKNOWN,false,false};
    assert(!f.guard.observe(f.config,unknown,State::Unknown,true,0,1,false,f.input));
    f.guard.qualify(f.config, unknown);
    assert(f.guard.warning.sequence == 0 && unknown.state == BRAKE_UNKNOWN && !unknown.motion_permitted); }
}

static void opt_in_home_release_requires_close_and_settle() {
  TransitionFixture f;
  assert(!f.sample(1,State::OperationEnabled,500,{true,true,false,500},false,true,1,true));
  assert(!f.sample(2,State::SwitchOnDisabled,1000,{},true,true,2,true));
  assert(f.guard.transition.phase == BrakeGuardPhase::Closing);
  assert(f.output.state == BRAKE_WAIT_HOLD_DELAY);
  // Service permission disappears while the drive-managed closing timer runs.
  assert(!f.sample(3,State::SwitchOnDisabled,1000,{},true,true,2,false));
  assert(!f.sample(102,State::SwitchOnDisabled,1000,{},false,true,2,false));
  assert(f.guard.transition.phase == BrakeGuardPhase::Settling);
  assert(!f.sample(202,State::SwitchOnDisabled,1000,{},false,true,2,false));
  assert(f.guard.transition.phase == BrakeGuardPhase::Held);
  assert(f.sample(203,State::SwitchOnDisabled,1017,{},false,true,2,false));
  TransitionFixture held_service;
  assert(!held_service.sample(1,State::ReadyToSwitchOn,0,{},false,true,1,true));
  assert(!held_service.sample(101,State::ReadyToSwitchOn,0,{},false,true,1,true));
  assert(!held_service.sample(201,State::ReadyToSwitchOn,0,{},false,true,1,true));
  assert(held_service.guard.transition.phase == BrakeGuardPhase::Held);
  // Keeping Home service permission cannot erase disabled cumulative sag.
  assert(held_service.sample(202,State::ReadyToSwitchOn,17,{},false,true,1,true));
}

static void opt_in_warning_diagnostics_are_first_cause() {
  TransitionFixture f; f.enabled();
  assert(!f.sample(103,State::SwitchOnDisabled,0,{},true));
  assert(!f.sample(202,State::SwitchOnDisabled,0,{},true));
  f.input={203,203,1,{},true,6554,3};
  assert(f.guard.observe(f.config,{BRAKE_APPLIED,false,false},State::SwitchOnDisabled,
      true,0,1,false,f.input));
  assert(f.guard.transition.failure == BrakeGuardFailure::VelocityCap);
  assert(f.guard.transition.failed_phase == BrakeGuardPhase::Settling);
  assert(f.guard.transition.diagnostic_caps_valid);
  assert(f.guard.transition.diagnostic_velocity_cap == 6553);
  assert(f.guard.transition.diagnostic_displacement_cap == 327);
  assert(std::string(brake_guard_failure_name(f.guard.transition.failure)) == "velocity_cap");
  assert(std::string(brake_guard_phase_name(f.guard.transition.failed_phase)) == "settling");
  assert(f.guard.warning.cause == BrakeGuardFailure::VelocityCap && f.guard.warning.sequence == 1);
  assert(f.guard.warning.caps_valid && f.guard.warning.velocity_cap == 6553 &&
         f.guard.warning.displacement_cap == 327 && f.guard.warning.peak_velocity == 6554);
  // Stale follow-on samples neither warn again nor replace the first cause.
  assert(!f.guard.observe(f.config,{BRAKE_UNKNOWN,false,false},State::Unknown,
      false,999,2,true,{}));
  assert(f.guard.transition.failure == BrakeGuardFailure::VelocityCap);
  assert(f.guard.transition.failed_phase == BrakeGuardPhase::Settling);
  assert(f.guard.transition.diagnostic_caps_valid && f.guard.transition.diagnostic_velocity_cap == 6553);
  assert(f.guard.warning.sequence == 1 && f.guard.warning.cause == BrakeGuardFailure::VelocityCap);
  assert(std::string(brake_guard_failure_name(BrakeHoldEvidence{}.cause)) == "disabled_displacement");
  assert(std::string(brake_warning_message(false)) ==
         "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).");
  assert(std::string(brake_warning_message(true)) ==
         "Brake appears to release after torque on - adjust drive brake release delay (C05.13).");
}

// Physical observations motivated a phase distinction, not acceptance of a
// particular trace. This independent diagnostic envelope is .05deg/2deg/s for
// the declared J4 scaling; production acceptance still requires commissioning.
struct OpeningFixture {
  BrakeConfig config{true,100000000,100000000,false,false,16,
      {327,6553,100000000,100000000,500000000}, {327,13107,500000000}};
  BrakeTransitionGuard guard;
  BrakeTransitionInput input{};
  BrakeOutput brake{BRAKE_APPLIED,false,false};
  uint64_t time = 0, epoch = 0;
  bool sample(State drive=State::SwitchedOn, int32_t position=0, int32_t velocity=0,
              uint16_t cw=15, bool authorized=true, uint64_t stage=1, int8_t mode=8,
              int32_t target=0, bool home=true, bool until_released=false) {
    time += 1000000;
    input = {time,time,1000000,{true,cw==15,true,target,stage,time-500000,cw,mode,true},
        true,velocity,3000000,authorized,2000000,until_released};
    return guard.observe(config,brake,drive,true,position,epoch,home,input);
  }
  void start() {
    // Existing authorized Home closes/qualifies the initial frame first.
    assert(!sample(State::SwitchedOn,0,0,7,false,1,8,0,false));
    assert(guard.phase == BrakeGuardPhase::Held);
    assert(!sample());
    assert(guard.phase == BrakeGuardPhase::Opening);
  }
  void closed(int32_t position=24) {
    assert(!sample(State::SwitchedOn,position,0,0,false));
    assert(guard.phase == BrakeGuardPhase::Closing && guard.anchor == 0);
    for (int i=0;i<199;++i) {
      assert(!sample(State::SwitchOnDisabled,position,0,0,false));
      assert(guard.anchor == 0);
    }
    assert(!sample(State::SwitchOnDisabled,position,0,0,false));
    assert(guard.phase == BrakeGuardPhase::Held && guard.anchor == position);
  }
};

static void bounded_opening_is_causal_scoped_and_default_off() {
  OpeningFixture f; f.start();
  // Replay the observed 141ms /24count/8305count/s class, then obtain enable.
  // The separate normal brake release delay still controls motion readiness.
  for (int i=0;i<139;++i) assert(!f.sample(State::SwitchedOn,i*24/138,8305));
  assert(f.guard.phase == BrakeGuardPhase::Opening && f.guard.anchor == 0);
  auto shown=f.brake; f.guard.qualify(shown);
  assert(shown.state == BRAKE_WAIT_RELEASE_DELAY && !shown.motion_permitted);
  assert(!f.sample(State::OperationEnabled,24,0));
  assert(f.guard.phase == BrakeGuardPhase::Enabled);
  for (bool opt_in : {false,true}) {
    OpeningFixture old;
    if (!opt_in) old.config.opening={};
    assert(!old.sample(State::SwitchedOn,0,0,7,false,1,8,0,false));
    // No authorization (including the wrong axis) never widens Held16.
    assert(old.sample(State::SwitchedOn,24,0,15,false));
    assert(old.guard.failure == BrakeGuardFailure::DisabledDisplacement);
  }
  OpeningFixture accepted;
  assert(!accepted.sample(State::SwitchedOn,0,0,7,false,1,8,0,false));
  assert(!accepted.sample(State::SwitchedOn,24,8305,15,true,1,8));
  assert(accepted.guard.phase == BrakeGuardPhase::Opening);
  // Homing mode never opens a stage: native Home owns no opening allowance.
  OpeningFixture homing;
  assert(!homing.sample(State::SwitchedOn,0,0,7,false,1,6,0,false));
  assert(homing.sample(State::SwitchedOn,24,8305,15,true,1,6));
  assert(homing.guard.failure == BrakeGuardFailure::OpeningProof);
  // An authorization flag cannot substitute for an actual submitted CW15.
  OpeningFixture requested;
  assert(!requested.sample(State::SwitchedOn,0,0,7,false,1,8,0,false));
  assert(requested.sample(State::SwitchedOn,24,0,7,true));
}

static void bounded_opening_proof_and_bounds_fail_closed() {
  for (int kind=0;kind<12;++kind) {
    OpeningFixture f; f.start();
    auto in=f.input;
    in.observed_ns=in.now_ns=f.time+1000000;
    in.command.submitted_ns=in.observed_ns-500000;
    int32_t position=0;
    uint64_t epoch=f.epoch;
    switch(kind) {
      case 0: in.command.valid=false; break;
      case 1: in.command.submitted_ns=in.observed_ns; break;
      case 2: in.command.submitted_ns=f.input.command.submitted_ns; break;
      case 3: in.command.target=1; break; // Even a small moving target is forbidden.
      case 4: in.command.generation=2; break;
      case 5: in.command.mode=6; break;
      case 6: in.command.controlword=31; break;
      case 7: position=328; break;
      case 8: in.velocity=13108; break;
      case 9: ++epoch; break; // Authorized Home cannot erase the raw envelope.
      case 10: in.velocity_valid=false; break;
      case 11: in.command.opening_stage=false; break; // Numeric generation collision is not stage proof.
    }
    assert(f.guard.observe(f.config,f.brake,State::SwitchedOn,true,position,epoch,true,in));
    assert(f.guard.phase == BrakeGuardPhase::Failed);
    // Restore all evidence: a failure is terminal, not a retry permission.
    assert(f.sample(State::OperationEnabled));
  }
  for (bool stale : {false,true}) {
    OpeningFixture f; f.start();
    f.input.now_ns+=4000000;
    if (!stale) f.input.observed_ns=f.input.now_ns;
    assert(f.guard.observe(f.config,f.brake,State::SwitchedOn,true,0,0,true,f.input));
    assert(f.guard.failure == BrakeGuardFailure::ObservationTime);
  }
  OpeningFixture deadline; deadline.start();
  for (int i=0;i<499;++i) assert(!deadline.sample());
  assert(deadline.sample());
  assert(deadline.guard.failure == BrakeGuardFailure::OpeningTimeout);
  assert(deadline.guard.failed_phase == BrakeGuardPhase::Opening);
  OpeningFixture accumulating; accumulating.start();
  for (int32_t position=1;position<=327;++position)
    assert(!accumulating.sample(State::SwitchedOn,position,1000));
  assert(accumulating.sample(State::SwitchedOn,328,1000));
  assert(accumulating.guard.failure == BrakeGuardFailure::DisplacementCap);
  OpeningFixture aged_wire; aged_wire.start();
  auto aged=aged_wire.input;
  aged.observed_ns=aged.now_ns=5000000; // Fresh PDO, permitted 3ms observation gap.
  aged.command.submitted_ns=2500000; // Advancing, but older than the 2ms wire budget.
  assert(aged_wire.guard.observe(aged_wire.config,aged_wire.brake,State::SwitchedOn,
      true,0,0,true,aged));
  assert(aged_wire.guard.failure == BrakeGuardFailure::OpeningProof);
}

// A named machine trial changes only its explicit speed data. Test both signs
// at the exact inclusive boundary and one count/s above it; the original cap
// remains distinct, and the fixed displacement limit is independent of speed.
static void opening_speed_trial_boundary_is_data_scoped() {
  for (int32_t cap : {13107, 19660}) {
    for (int32_t sign : {-1, 1}) {
      OpeningFixture f; f.config.opening.max_velocity_counts_per_s=cap; f.start();
      assert(!f.sample(State::SwitchedOn,sign*24,sign*cap));
      assert(f.guard.phase == BrakeGuardPhase::Opening && f.guard.anchor == 0);
      assert(f.sample(State::SwitchedOn,sign*25,sign*(cap+1)));
      assert(f.guard.failure == BrakeGuardFailure::VelocityCap);
      assert(f.guard.diagnostic_caps_valid && f.guard.diagnostic_velocity_cap == uint32_t(cap));
      assert(f.guard.diagnostic_displacement_cap == 327);
      OpeningFixture displacement; displacement.config.opening.max_velocity_counts_per_s=cap; displacement.start();
      assert(displacement.sample(State::SwitchedOn,sign*328,0));
      assert(displacement.guard.failure == BrakeGuardFailure::DisplacementCap);
    }
  }
  OpeningFixture old; old.start();
  assert(old.sample(State::SwitchedOn,-41,-14194));
  assert(old.guard.failure == BrakeGuardFailure::VelocityCap);
  OpeningFixture trial; trial.config.opening.max_velocity_counts_per_s=19660; trial.start();
  assert(!trial.sample(State::SwitchedOn,-41,-14194));
  assert(trial.guard.phase == BrakeGuardPhase::Opening);
}

static void bounded_opening_abort_preserves_envelope_and_held_monitor() {
  OpeningFixture f; f.start(); f.closed();
  assert(f.sample(State::SwitchOnDisabled,41,0,0,false)); // Final anchor24 +17.
  assert(f.guard.failure == BrakeGuardFailure::DisabledDisplacement);
  for (bool velocity : {false,true}) {
    OpeningFixture drift; drift.start();
    assert(!drift.sample(State::SwitchedOn,24,0,0,false));
    // Changing live arguments cannot expand the captured opening envelope.
    drift.config.opening={1000,50000,500000000};
    assert(drift.sample(State::SwitchOnDisabled,velocity?24:328,velocity?13108:0,0,false));
  }
  OpeningFixture missing_inhibit; missing_inhibit.start();
  // Lost authority forces Closing but stale CW15 cannot certify disabled hold.
  for (int i=0;i<500;++i) {
    assert(!missing_inhibit.sample(State::SwitchOnDisabled,24,0,15,false));
    assert(missing_inhibit.guard.phase == BrakeGuardPhase::Closing);
    assert(missing_inhibit.guard.anchor == 0);
  }
  assert(missing_inhibit.sample(State::SwitchOnDisabled,24,0,15,false));
  assert(missing_inhibit.guard.failure == BrakeGuardFailure::SettlingTimeout);
  OpeningFixture same_stage; same_stage.start(); same_stage.closed();
  assert(same_stage.sample(State::SwitchedOn,24,0,15,true,1,8,24));
  assert(same_stage.guard.failure == BrakeGuardFailure::OpeningProof);
  OpeningFixture next_stage; next_stage.start(); next_stage.closed();
  assert(!next_stage.sample(State::SwitchedOn,24,0,15,true,2,8,24));
  assert(next_stage.guard.phase == BrakeGuardPhase::Opening);
}

static void ordinary_arm_opening_retains_bounds_through_release() {
  const auto begin=[](OpeningFixture& f) {
    assert(!f.sample(State::SwitchedOn,0,0,7,false,1,8,0,false));
    assert(!f.sample(State::SwitchedOn,24,8305,15,true,1,8,0,false,true));
    f.brake={BRAKE_WAIT_RELEASE_DELAY,false,false};
    assert(!f.sample(State::OperationEnabled,24,0,15,true,1,8,0,false,true));
    assert(f.guard.phase == BrakeGuardPhase::OpeningEnabled && f.guard.anchor == 0);
  };
  OpeningFixture f; begin(f);
  for (unsigned i=0;i<100;++i) {
    assert(!f.sample(State::OperationEnabled,24,0,15,true,1,8,0,false,true));
    auto output=f.brake; f.guard.qualify(output);
    assert(!output.motion_permitted && f.guard.anchor == 0);
  }
  f.brake={BRAKE_RELEASED,true,false};
  assert(!f.sample(State::OperationEnabled,24,0,15,true,1,8,0,false,true));
  assert(f.guard.phase == BrakeGuardPhase::Enabled);
  auto output=f.brake; f.guard.qualify(output); assert(output.motion_permitted);
  // Qualified release earns a closing witness. Its energized close (even far
  // beyond the 327-count envelope) is recorded; take-up is judged after it.
  assert(!f.sample(State::SwitchOnDisabled,41,2792,0,false,99,8,0,false));
  assert(f.guard.phase == BrakeGuardPhase::Closing && f.guard.anchor == 24);
  assert(!f.sample(State::SwitchOnDisabled,352,0,0,false,99,8,0,false));
  f.brake={BRAKE_APPLIED,false,false};
  while (f.guard.phase == BrakeGuardPhase::Closing)
    assert(!f.sample(State::SwitchOnDisabled,352,0,0,false,99,8,0,false));
  assert(f.guard.phase == BrakeGuardPhase::Settling && f.guard.anchor == 352);
  assert(f.guard.energized.peak_displacement == 328 && f.guard.energized.peak_velocity == 2792);
  assert(f.sample(State::SwitchOnDisabled,369,0,0,false,99,8,0,false));
  assert(f.guard.failure == BrakeGuardFailure::Corridor);
  for (int kind=0;kind<5;++kind) {
    OpeningFixture bad; begin(bad);
    if (kind==4) ++bad.epoch;
    assert(bad.sample(State::OperationEnabled,kind==0?328:24,kind==1?13108:0,
        15,true,1,8,kind==2?1:0,false,kind!=3));
  }
  OpeningFixture timeout; begin(timeout);
  bool fault=false;
  for (unsigned i=0;i<501 && !fault;++i)
    fault=timeout.sample(State::OperationEnabled,24,0,15,true,1,8,0,false,true);
  assert(fault && timeout.guard.failure == BrakeGuardFailure::OpeningTimeout);
  assert(timeout.time < 505000000); // Same first-CW15 budget.
  OpeningFixture stop; begin(stop);
  assert(!stop.sample(State::OperationEnabled,24,0,0,false,99,8,0,false,true));
  assert(stop.guard.phase == BrakeGuardPhase::Closing && stop.guard.anchor == 0);
  assert(!stop.sample(State::SwitchOnDisabled,328,0,0,false,99,8,0,false,true));
  stop.brake={BRAKE_APPLIED,false,false};
  while (stop.guard.phase == BrakeGuardPhase::Closing)
    assert(!stop.sample(State::SwitchOnDisabled,328,0,0,false,99,8,0,false,true));
  assert(stop.guard.anchor == 328);
  assert(stop.sample(State::SwitchOnDisabled,345,0,0,false,99,8,0,false,true));
  OpeningFixture wrong_mode;
  assert(!wrong_mode.sample(State::SwitchedOn,0,0,7,false,1,6,0,false));
  assert(wrong_mode.sample(State::SwitchedOn,24,0,15,true,1,6,0,false,true));
  assert(wrong_mode.guard.failure == BrakeGuardFailure::OpeningProof);
}

static void ordinary_release_witness_survives_motion_until_inhibit() {
  const auto after_motion=[](OpeningFixture& f) {
    assert(!f.sample(State::SwitchedOn,0,0,7,false,1,8,0,false));
    assert(!f.sample(State::SwitchedOn,24,8305,15,true,1,8,0,false,true));
    f.brake={BRAKE_WAIT_RELEASE_DELAY,false,false};
    for (unsigned i=0;i<101;++i)
      assert(!f.sample(State::OperationEnabled,24,0,15,true,1,8,0,false,true));
    f.brake={BRAKE_RELEASED,true,false};
    assert(!f.sample(State::OperationEnabled,24,0,15,true,1,8,0,false,true));
    // Normal motion changes targets and uses the ordinary control namespace.
    // Its limits/rate/following-error gates belong to the unchanged output path.
    for (int i=1;i<=30;++i) {
      f.time+=1000000;
      f.input={f.time,f.time,1000000,
          {true,true,false,24+4*i,44,f.time-500000,15,8,false},true,4000,3000000,false,2000000};
      assert(!f.guard.observe(f.config,f.brake,State::OperationEnabled,true,24+4*i,0,false,f.input));
    }
    assert(f.guard.anchor == 144);
  };
  for (int kind=0;kind<7;++kind) {
    OpeningFixture f; after_motion(f);
    if (kind==1) ++f.epoch;
    if (kind==1) {
      assert(f.sample(State::OperationEnabled,145,0,0,false,99,8,140,false));
      assert(f.guard.failure == BrakeGuardFailure::FrameChanged); continue;
    }
    if (kind==2 || kind==3) {
      auto input=f.input; input.observed_ns=input.now_ns=f.time+1000000;
      input.command.controlword=0;
      if (kind==2) input.command.valid=false; // No submitted inhibit.
      // kind3 deliberately reuses the latest enabled wire timestamp.
      // Feedback can lag the first inhibit by a cycle. Missing its receipt
      // must fail BEFORE that OP feedback can shift the pre-inhibit anchor.
      assert(f.guard.observe(f.config,f.brake,State::OperationEnabled,true,161,0,false,input));
      assert(f.guard.failure == BrakeGuardFailure::OpeningProof); continue;
    }
    f.brake={BRAKE_WAIT_HOLD_DELAY,false,true};
    assert(!f.sample(State::OperationEnabled,145,0,0,false,99,8,140,false));
    assert(f.guard.anchor == 144);
    if (kind==4) ++f.epoch;
    if (kind>=4) {
      // A frame change still refuses; energized excursions are only recorded.
      assert(f.sample(State::SwitchOnDisabled,kind==5?472:161,kind==6?6554:2000,
          0,false,99,8,140,false) == (kind==4));
      if (kind!=4) assert(f.guard.phase == BrakeGuardPhase::Closing &&
          f.guard.energized.peak_displacement == (kind==5?328:17) &&
          f.guard.energized.peak_velocity == (kind==6?6554:2000));
      continue;
    }
    assert(!f.sample(State::SwitchOnDisabled,161,2792,0,false,99,8,140,false));
    for (unsigned i=0;i<99;++i)
      assert(!f.sample(State::SwitchOnDisabled,161,0,0,false,99,8,140,false));
    f.brake={BRAKE_APPLIED,false,false};
    for (unsigned i=0;i<101;++i)
      assert(!f.sample(State::SwitchOnDisabled,161,0,0,false,99,8,140,false));
    assert(f.guard.phase == BrakeGuardPhase::Held && f.guard.anchor == 161);
    assert(f.sample(State::SwitchOnDisabled,178,0,0,false,99,8,140,false));
    assert(f.guard.failure == BrakeGuardFailure::DisabledDisplacement);
  }
}

static void effective_trip_caps_follow_phase_not_current_profile() {
  // Opening-abort closure uses its captured opening bound, not ordinary closing.
  OpeningFixture abort; abort.start();
  abort.config.opening.max_velocity_counts_per_s = 99999;
  assert(abort.sample(State::SwitchedOn, 24, -13108, 0, false));
  assert(abort.guard.failure == BrakeGuardFailure::VelocityCap);
  assert(abort.guard.diagnostic_caps_valid && abort.guard.diagnostic_velocity_cap == 13107);
  assert(abort.guard.diagnostic_displacement_cap == 327);
  // A qualified release keeps its captured closing envelope through later config data.
  OpeningFixture close;
  assert(!close.sample(State::SwitchedOn,0,0,7,false,1,8,0,false));
  assert(!close.sample(State::SwitchedOn,0,0,15,true,1,8,0,false,true));
  close.brake = {BRAKE_RELEASED, true, false};
  assert(!close.sample(State::OperationEnabled, 0, 0, 15, true, 1, 8, 0, false, true));
  close.config.transition.max_velocity_counts_per_s = 99999;
  close.brake = {BRAKE_WAIT_HOLD_DELAY, false, true};
  assert(!close.sample(State::SwitchOnDisabled, 24, -6554, 0, false, 2, 8, 0, false));
  assert(close.guard.phase == BrakeGuardPhase::Closing && close.guard.energized.peak_velocity == -6554);
  close.brake = {BRAKE_APPLIED, false, false};
  while (close.guard.phase == BrakeGuardPhase::Closing)
    assert(!close.sample(State::SwitchOnDisabled, 24, 0, 0, false, 2, 8, 0, false));
  assert(close.sample(State::SwitchOnDisabled, 24, -6554, 0, false, 2, 8, 0, false));
  assert(close.guard.failure == BrakeGuardFailure::VelocityCap);
  assert(close.guard.diagnostic_caps_valid && close.guard.diagnostic_velocity_cap == 6553);
  // Held failures have no active transition-speed cap; do not publish a misleading one.
  OpeningFixture held; held.start(); held.closed();
  assert(held.sample(State::SwitchOnDisabled, 41, 0, 0, false));
  assert(!held.guard.diagnostic_caps_valid && held.guard.diagnostic_velocity_cap == 0);
  // Missing velocity remains a refusal, with the relevant bounds retained for diagnosis.
  TransitionFixture invalid; invalid.enabled();
  invalid.input = {103,103,1,{},false,0,3};
  assert(invalid.guard.observe(invalid.config,{BRAKE_WAIT_HOLD_DELAY,false,true},
      State::SwitchOnDisabled,true,0,1,false,invalid.input));
  assert(invalid.guard.transition.failure == BrakeGuardFailure::VelocityUnavailable);
  assert(invalid.guard.transition.diagnostic_caps_valid && invalid.guard.transition.diagnostic_velocity_cap == 6553);
}

// Recorded the 1430 cell J5 Arm opening: -10922 counts/s against the 9102 counts/s
// opening cap at only -8 counts. It is a torque-on warning; the stage keeps its
// token until Operation Enabled restarts the lifecycle, and BrakeMachine's
// release delay alone then gates motion. Arm is never refused.
static void arm_j5_opening_velocity_warns_without_blocking_arm() {
  const BrakeConfig config{true,100000000,100000000,false,false,16,
      {327,19660,100000000,100000000,500000000}, {327,9102,500000000}};
  DisabledBrakeGuard guard;
  BrakeOutput brake{BRAKE_APPLIED,false,false};
  uint64_t time = 0;
  const auto sample = [&](State drive, int32_t position, int32_t velocity, uint16_t cw, bool authorized) {
    time += 1000000;
    const BrakeTransitionInput input{time,time,1000000,{true,cw==15,true,0,1,time-500000,cw,8,true},
        true,velocity,3000000,authorized,2000000,true,true};
    return guard.observe(config,brake,drive,true,position,0,false,input);
  };
  const auto shown = [&]() { auto out = brake; guard.qualify(config, out); return out; };
  assert(!sample(State::SwitchedOn,0,0,7,false));
  assert(!sample(State::SwitchedOn,0,0,15,true));
  assert(guard.transition.phase == BrakeGuardPhase::Opening);
  assert(sample(State::SwitchedOn,-8,-10922,15,true));
  const auto& w = guard.warning;
  assert(w.sequence == 1 && w.cause == BrakeGuardFailure::VelocityCap && w.phase == BrakeGuardPhase::Opening);
  assert(w.torque_on && w.anchor == 0 && w.peak_displacement == -8 && w.peak_velocity == -10922);
  assert(w.caps_valid && w.velocity_cap == 9102 && w.displacement_cap == 327);
  assert(guard.resettling && shown().state == BRAKE_WAIT_RELEASE_DELAY && !shown().motion_permitted);
  // The stage in progress is not re-anchored (its token is consumed) and the
  // same transient does not warn again while the drive finishes enabling.
  for (int i = 0; i < 20; ++i) assert(!sample(State::SwitchedOn,-8,0,15,true));
  assert(guard.resettling && guard.warning.sequence == 1);
  brake = {BRAKE_WAIT_RELEASE_DELAY,false,false};
  assert(!sample(State::OperationEnabled,-8,0,15,true));
  assert(!guard.resettling && guard.transition.phase == BrakeGuardPhase::Enabled);
  assert(guard.warning.settled && guard.warning.settled_displacement == -8);
  assert(!shown().motion_permitted); // C05.13 release delay, not the warning.
  brake = {BRAKE_RELEASED,true,false};
  assert(!sample(State::OperationEnabled,-8,0,15,true));
  assert(shown().motion_permitted && guard.warning.sequence == 1);
}

// One J4-configured stop, sampled every simulated cycle. Nanoseconds are
// shortened as in TransitionFixture (hold delay and quiet window 100 cycles);
// counts and counts/s are J4's.
struct J4Stop {
  const BrakeConfig config{true,100,100,false,false,16,{327,19660,100,100,500}};
  DisabledBrakeGuard guard;
  BrakeOutput shown{};
  uint64_t t = 0;
  bool sample(State drive, int32_t p, int32_t velocity = 0, bool holding = false,
              bool enable_requested = false) {
    ++t;
    const BrakeHoldCommand command = drive == State::OperationEnabled
        ? BrakeHoldCommand{true,true,true,0} : BrakeHoldCommand{true,false,false,0};
    BrakeTransitionInput input{t,t,1,command,true,velocity,3};
    input.enable_requested = enable_requested;
    const BrakeOutput brake{drive == State::OperationEnabled ? BRAKE_RELEASED :
        holding ? BRAKE_WAIT_HOLD_DELAY : BRAKE_APPLIED, drive == State::OperationEnabled, holding};
    const bool warned = guard.observe(config,brake,drive,true,p,1,false,input);
    shown = brake; guard.qualify(config, shown);
    return warned;
  }
  // Enabled at p, then an inhibit whose energized hold delay ends at rest p.
  void stop(int32_t p) {
    for (int i = 0; i < 102; ++i) assert(!sample(State::OperationEnabled,p));
    for (int i = 0; i < 100; ++i) assert(!sample(State::SwitchOnDisabled,p,0,true));
    assert(!sample(State::SwitchOnDisabled,p));
    assert(guard.energized_updated && guard.anchor == p);
    assert(guard.transition.phase == BrakeGuardPhase::Settling);
  }
  // Holds p for the 100-cycle quiet window, so the guard re-anchors there.
  void rest(int32_t p) {
    for (int i = 0; i < 100 && guard.resettling; ++i) assert(!sample(State::SwitchOnDisabled,p));
    assert(!guard.resettling && guard.anchor == p);
  }
};

// Recorded the 1430 cell J4 trip at the torque drop (~102 ms after disarm): 24 counts
// at 20479 counts/s, beyond the 19660 counts/s closing cap. That brake take-up
// follows torque-off, so it is one torque-off (C05.10) warning, then re-anchor
// at the 24-count rest after the quiet window (or at once on an Arm request
// after the drive reports the brake applied).
static void arm_j4_torque_drop_warns_once_and_reanchors_at_rest() {
  for (bool arm_request : {false, true}) {
    J4Stop s;
    s.stop(0);
    assert(s.sample(State::SwitchOnDisabled,24,20479));
    const auto& w = s.guard.warning;
    assert(w.sequence == 1 && w.cause == BrakeGuardFailure::VelocityCap && w.phase == BrakeGuardPhase::Settling);
    assert(!w.torque_on && w.anchor == 0 && w.peak_displacement == 24 && w.peak_velocity == 20479);
    assert(w.caps_valid && w.velocity_cap == 19660 && w.displacement_cap == 327);
    assert(w.same_direction_count == 1 && !w.slip);
    // The unfinished transition remains visible; motion is never granted here.
    assert(s.shown.state == BRAKE_WAIT_HOLD_DELAY && s.shown.hold_window_active && !s.shown.motion_permitted);
    if (arm_request) {
      assert(!s.sample(State::SwitchOnDisabled,24,0,false,true));
      assert(!s.guard.resettling && s.guard.anchor == 24);
    } else {
      s.rest(24);
    }
    assert(s.guard.transition.phase == BrakeGuardPhase::Held);
    assert(s.guard.warning.settled && s.guard.warning.settled_displacement == 24 && s.guard.warning.sequence == 1);
    assert(s.shown.state == BRAKE_APPLIED && !s.shown.hold_window_active);
    // The ordinary Held monitor now judges displacement from the rest pose.
    assert(!s.sample(State::SwitchOnDisabled,40));
    assert(s.sample(State::SwitchOnDisabled,41));
    assert(s.guard.warning.sequence == 2 && s.guard.warning.anchor == 24 &&
           s.guard.warning.cause == BrakeGuardFailure::DisabledDisplacement);
    // An Arm request ended the braked period; otherwise this continues the run.
    assert(s.guard.warning.same_direction_count == (arm_request ? 1u : 2u));
  }
}

// Recorded the 1430 cell class: +17 counts 13 ms after CW0, inside the 100 ms hold
// delay while the drive still holds torque. It is recorded, never a warning;
// movement past the bound after torque-off warns with the C05.10 message.
static void energized_servo_settling_is_recorded_not_warned() {
  const BrakeConfig config{true,100000000,100000000,false,false,16,
      {327,19660,100000000,100000000,500000000}};
  BrakeMachine machine;
  DisabledBrakeGuard guard;
  uint64_t t = 0;
  const auto sample = [&](State drive, int32_t p, int32_t velocity) {
    t += 1000000;
    const BrakeHoldCommand command = drive == State::OperationEnabled
        ? BrakeHoldCommand{true,true,true,0} : BrakeHoldCommand{true,false,false,0};
    const BrakeTransitionInput input{t,t,1000000,command,true,velocity,3000000};
    return guard.observe(config,machine.step(drive,t,config),drive,true,p,1,false,input);
  };
  for (int i = 0; i < 150; ++i) assert(!sample(State::OperationEnabled,0,0));
  for (int ms = 1; ms <= 100; ++ms) {
    const int32_t p = std::min(17, ms * 17 / 13);
    assert(!sample(State::SwitchOnDisabled,p,ms <= 13 ? 1300 : 0));
    assert(guard.transition.phase == BrakeGuardPhase::Closing && guard.warning.sequence == 0);
  }
  assert(!sample(State::SwitchOnDisabled,17,0));
  assert(guard.energized_updated && guard.anchor == 17 && guard.warning.sequence == 0);
  const auto& e = guard.energized;
  assert(e.peak_displacement == 17 && e.peak_velocity == 1300 && e.torque_off_anchor == 17);
  assert(e.torque_off_ns - e.start_ns == 100000000);
  assert(!sample(State::SwitchOnDisabled,33,0));
  assert(sample(State::SwitchOnDisabled,34,0));
  assert(guard.warning.cause == BrakeGuardFailure::Corridor && guard.warning.anchor == 17);
  assert(!guard.warning.torque_on && guard.warning.phase == BrakeGuardPhase::Settling);
  assert(std::string(brake_warning_message(guard.warning.torque_on)) ==
         "Brake appears to latch after torque off - adjust drive brake hold delay (C05.10).");
}

// Slip is a pattern within ONE braked period: consecutive same-direction
// warnings, each after the guard re-anchored at the previous rest. 17 counts
// is one past J4's 16-count tolerance.
static void same_direction_steps_in_one_braked_period_suspect_slip() {
  { J4Stop s; s.stop(0);
    for (int32_t step = 1; step <= 3; ++step) {
      assert(s.sample(State::SwitchOnDisabled,17 * step));
      const auto& w = s.guard.warning;
      assert(w.same_direction_count == uint32_t(step) && w.same_direction_displacement == 17 * step);
      assert(w.slip == (step == 3) && !w.torque_on && w.anchor == 17 * (step - 1));
      s.rest(17 * step);
    }
    // A slip warning is still only a warning: re-enable starts a fresh period.
    assert(!s.sample(State::OperationEnabled,51,0,false,true));
    assert(!s.guard.resettling && s.guard.transition.phase == BrakeGuardPhase::Enabled); }
  // Back-and-forth jitter never accumulates; a reversal restarts the run.
  { J4Stop s; s.stop(0);
    for (int32_t step = 1; step <= 6; ++step) {
      const int32_t p = step % 2 ? 17 : 0;
      assert(s.sample(State::SwitchOnDisabled,p));
      assert(s.guard.warning.same_direction_count == 1 && !s.guard.warning.slip);
      s.rest(p);
    } }
  { J4Stop s; s.stop(0);
    for (int32_t p : {17, 34, 17}) { assert(s.sample(State::SwitchOnDisabled,p)); s.rest(p); }
    assert(s.guard.warning.same_direction_count == 1 && s.guard.warning.same_direction_displacement == -17);
    assert(s.sample(State::SwitchOnDisabled,51));
    assert(s.guard.warning.same_direction_count == 1 && !s.guard.warning.slip); }
  // The same gravity settle at every separate stop never counts as slip.
  { J4Stop s;
    int32_t p = 0;
    for (int stop = 0; stop < 4; ++stop) {
      s.stop(p);
      assert(s.sample(State::SwitchOnDisabled,p + 17));
      assert(s.guard.warning.same_direction_count == 1 && !s.guard.warning.slip);
      p += 17;
      s.rest(p);
    }
    assert(s.guard.warning.sequence == 4); }
  // An enable (Arm) request ends the braked period even before the drive enables.
  { J4Stop s; s.stop(0);
    for (int32_t p : {17, 34}) { assert(s.sample(State::SwitchOnDisabled,p)); s.rest(p); }
    assert(!s.sample(State::SwitchOnDisabled,34,0,false,true));
    assert(s.sample(State::SwitchOnDisabled,51));
    assert(s.guard.warning.same_direction_count == 1 && !s.guard.warning.slip); }
}

int main() {
  arm_j5_opening_velocity_warns_without_blocking_arm();
  arm_j4_torque_drop_warns_once_and_reanchors_at_rest();
  energized_servo_settling_is_recorded_not_warned();
  same_direction_steps_in_one_braked_period_suspect_slip();
  effective_trip_caps_follow_phase_not_current_profile();
  startup_alarm_keeps_fixed_hold_without_brake_misclassification();
  opening_speed_trial_boundary_is_data_scoped();
  bounded_opening_is_causal_scoped_and_default_off();
  bounded_opening_proof_and_bounds_fail_closed();
  bounded_opening_abort_preserves_envelope_and_held_monitor();
  ordinary_arm_opening_retains_bounds_through_release();
  ordinary_release_witness_survives_motion_until_inhibit();
  opt_in_home_release_requires_close_and_settle();
  opt_in_warning_diagnostics_are_first_cause();
  opt_in_closing_settling_and_permanent_hold();
  opt_in_closing_is_judged_from_the_torque_off_anchor();
  opt_in_time_frame_home_and_unknown_guards();
  delay_boundaries();
  reenable_and_feedback();
  hold_warnings();
  measured_hold_tolerance_and_home_frame();
  enable_pose_survives_brake_release();
  legacy_energized_settle_is_recorded_and_takeup_is_judged();
  simulated_timing_and_sag();
  std::cout << "brake tests passed\n";
}
