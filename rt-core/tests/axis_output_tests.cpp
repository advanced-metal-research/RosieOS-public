#include "output_policy.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

using namespace rosie::rtcore;

// Makefile appends -UNDEBUG; make test runs the assertion probe first.
// Keep assertions enabled when compiling this test directly.

static AxisOutputConfig configuration() {
  AxisOutputConfig c{};
  c.limits = {-0.5, 0.5, 0.1, false, true};
  c.limits.following_error = 0.01;
  c.limits.following_error_timeout_ns = 3000000;
  c.counts_per_unit = 100000;
  c.sign = 1;
  c.max_profile_velocity = 10000;
  c.rotary = true;
  return c;
}

static AxisOutputInput enabled_input(uint16_t statusword = 0x27) {
  AxisOutputInput input{};
  input.statusword = statusword;
  input.feedback_fresh = true;
  input.configuration_current = true;
  input.config_verified = true;
  input.armed = true;
  input.enabled = true;
  input.commanded_mode = 8;
  input.previous_target_valid = true;
  input.now_ns = 2000000;
  input.previous_cycle_ns = 1000000;
  return input;
}

// A trusted exterior pose must be held without an automatic step or a fault.
static void submitted_reference_tracking_tests() {
  auto config = configuration();
  config.limits.max_qd = 100;
  config.max_profile_velocity = 10000000;
  auto input = enabled_input();
  input.setpoint_present = true;
  input.target_position = 2000; // Future candidate is outside the error limit.
  AxisOutputState state{};
  for (uint64_t now = 2000000; now <= 6000000; now += 1000000) {
    input.now_ns = now;
    input.previous_cycle_ns = now - 1000000;
    const auto out = evaluate_axis_output(config, input, state);
    assert(!out.fault && out.next.following_error.outside_since == 0);
    state = out.next;
  }
  // Once submitted, the same discrepancy must still trip at the unchanged deadline.
  input.previous_target = 2000;
  state = {};
  for (uint64_t now = 2000000; now <= 5000000; now += 1000000) {
    input.now_ns = now;
    input.previous_cycle_ns = now - 1000000;
    const auto out = evaluate_axis_output(config, input, state);
    assert(bool(out.fault_reasons & FaultFollowingError) == (now == 5000000));
    state = out.next;
  }
  // An unsubmitted candidate cannot establish a tracking reference at startup.
  input.previous_target_valid = false;
  state.following_error.outside_since = 1;
  const auto out = evaluate_axis_output(config, input, state);
  assert(!(out.fault_reasons & FaultFollowingError));
  assert(out.next.following_error.outside_since == 0);
}

// TestTrackingLagAllowanceFollowingError: the 1430 cell gearing (131072 x 100 / 2pi
// counts per joint radian), following_error 0.02 rad held 100 ms. The J6 trip
// of 2026-10-06 (0.0289 rad at ~60 deg/s) faulted without an allowance; with
// the 50 ms allowance the bound at that speed is 0.02 + 1.0472 x 0.05 =
// 0.0724 rad. Beyond the scaled bound the fault still latches after exactly
// the unchanged 100 ms; stationary, the static 0.02 rad bound applies.
static void tracking_lag_allowance_tests() {
  auto config = configuration();
  config.counts_per_unit = 131072.0 * 100 / 6.28318530717958647692;
  config.limits.min_q = -3;
  config.limits.max_q = 3;
  config.limits.max_qd = 2;
  config.limits.following_error = 0.02;
  config.limits.following_error_timeout_ns = 100'000'000;
  config.max_profile_velocity = 10'000'000;
  // Returns ns from the first sample outside the bound to the fault, or 0.
  auto fault_after = [&](double error_rad, double velocity_rad_s, uint64_t allowance_ns) -> uint64_t {
    config.limits.tracking_lag_allowance_ns = allowance_ns;
    auto input = enabled_input();
    const int32_t target = 1'000'000;
    input.setpoint_present = true;
    input.target_position = input.previous_target = target;
    input.target_velocity = velocity_rad_s * config.counts_per_unit;
    input.feedback_position = target - int32_t(std::llround(error_rad * config.counts_per_unit));
    AxisOutputState state{};
    for (uint64_t now = 2'000'000; now <= 400'000'000; now += 1'000'000) {
      input.now_ns = now;
      input.previous_cycle_ns = now - 1'000'000;
      const auto out = evaluate_axis_output(config, input, state);
      if (out.fault) {
        assert(out.fault_reasons == FaultFollowingError && out.controlword == 0);
        return now - 2'000'000;
      }
      assert(out.target_velocity == std::llround(input.target_velocity));
      state = out.next;
    }
    return 0;
  };
  const double speed = 60 * 3.14159265358979323846 / 180, trip = 0.0289;
  const double scaled = 0.02 + speed * 0.05;
  assert(fault_after(trip, speed, 0) == 100'000'000);
  assert(fault_after(trip, speed, 50'000'000) == 0);
  assert(fault_after(trip, -speed, 50'000'000) == 0);
  assert(fault_after(scaled - 1e-4, speed, 50'000'000) == 0);
  assert(fault_after(scaled + 1e-4, speed, 50'000'000) == 100'000'000);
  assert(fault_after(scaled + 1e-4, -speed, 50'000'000) == 100'000'000);
  assert(fault_after(trip, 0, 50'000'000) == 100'000'000);
  assert(fault_after(0.02 - 1e-4, 0, 50'000'000) == 0);
}

static void outside_limits_hold_tests() {
  auto config = configuration();
  config.limits.min_q = 0.01;
  config.limits.max_q = 0.1;
  auto input = enabled_input(0x23);
  auto out = evaluate_axis_output(config, input, {});
  assert(!out.fault && out.controlword == 0x0f && out.target_position == 0);
  assert(out.next.position_rate_active);
  input.statusword = 0x27;
  input.now_ns = 3000000;
  input.previous_cycle_ns = 2000000;
  input.setpoint_present = true;
  input.target_position = 10;
  auto inward = evaluate_axis_output(config, input, out.next);
  assert(!inward.fault && inward.target_position == 10);
  input.target_position = -1;
  auto outward = evaluate_axis_output(config, input, out.next);
  assert(outward.fault && outward.controlword == 0 && outward.target_position == 0);
  input.target_position = 10;
  input.target_velocity = -1;
  assert(evaluate_axis_output(config, input, out.next).fault);
  // Following error still expires after exactly 3,000,000 ns outside 2 counts.
  config.limits.following_error = 0.00002;
  input.target_velocity = 0;
  input.previous_target = 10; // This target has already reached the bus.
  auto state = out.next;
  for (uint64_t now : {3000000ULL, 4000000ULL, 5000000ULL}) {
    input.now_ns = now;
    input.previous_cycle_ns = now - 1000000;
    out = evaluate_axis_output(config, input, state);
    assert(!out.fault && out.next.following_error.outside_since == 3000000);
    state = out.next;
  }
  input.now_ns = 6000000;
  input.previous_cycle_ns = 5000000;
  out = evaluate_axis_output(config, input, state);
  assert(out.fault && (out.fault_reasons & FaultFollowingError));
  // Inhibition retains the original reference and emits no recovery movement.
  input.armed = false;
  input.previous_target = 17;
  input.statusword = 0x23;
  out = evaluate_axis_output(config, input, {});
  assert(out.controlword == 0 && out.reference.target == 17 &&
      out.reference.disposition == ReferenceDisposition::Invalid);

  AxisLimits limits{-1, 1, 1, false, true};
  for (double side : {-1.0, 1.0}) {
    assert(limit_recovery_step(limits, side * 2, side * 2));
    assert(limit_recovery_step(limits, side * 2, side * 1.5));
    assert(limit_recovery_step(limits, side * 2, side));
    assert(limit_recovery_step(limits, side * 2, -side));
    assert(!limit_recovery_step(limits, side * 2, side * 2.001));
    assert(!limit_recovery_step(limits, side * 2, -side * 1.001));
    assert(!limit_recovery_step(limits, side, side * 1.001));
    assert(limit_recovery_velocity(limits, side * 2, -side));
    assert(!limit_recovery_velocity(limits, side * 2, side));
  }
  assert(!limit_recovery_step(limits, 0, std::numeric_limits<double>::quiet_NaN()));
  assert(limit_recovery_feedforward(limits, -2, 0, 1, -1));
  assert(!limit_recovery_feedforward(limits, -2, 0, 1, -2));
  assert(!limit_recovery_feedforward(limits, 2, 0, -1, 2));
  limits.valid = false;
  assert(!limit_recovery_step(limits, 2, 1));

  // Logical direction is independent of motor sign and additive Home offset.
  for (int sign : {-1, 1}) for (int side : {-1, 1}) {
    config = configuration();
    config.sign = sign;
    config.limits.min_q = -.1;
    config.limits.max_q = .1;
    input = enabled_input(0x23);
    input.home_offset = 100;
    input.feedback_position = sign * side * 20000 - 100;
    input.previous_target = input.feedback_position;
    out = evaluate_axis_output(config, input, {});
    assert(!out.fault && out.target_position == input.feedback_position);
    input.statusword = 0x27;
    input.setpoint_present = true;
    input.now_ns = 3000000;
    input.previous_cycle_ns = 2000000;
    input.target_position = input.feedback_position - sign * side * 10;
    assert(!evaluate_axis_output(config, input, out.next).fault);
    input.target_position = input.feedback_position + sign * side;
    assert(evaluate_axis_output(config, input, out.next).fault);
  }
}

// Continuous recovery uses the one exact command interval. An exterior hold
// may move inward but never farther outward.
static void continuous_limit_recovery_tests() {
  auto config = configuration();
  config.counts_per_unit = 1;
  config.limits = {10000, 50000, 10000, false, true};
  config.limits.following_error = 100;
  config.limits.following_error_timeout_ns = 3000000;
  auto check = [&](int32_t feedback, int32_t target, bool accepted) {
    auto input = enabled_input();
    input.feedback_position = input.previous_target = feedback;
    input.setpoint_present = true;
    input.target_position = target;
    AxisOutputState state{};
    state.held_target_valid = true;
    state.held_target = feedback;
    const auto out = evaluate_axis_output(config, input, state);
    if (out.fault == accepted) std::cerr << "feedback=" << feedback << " target=" << target
        << " fault=" << out.fault << " reasons=" << out.fault_reasons << '\n';
    assert(out.fault == !accepted);
    if (accepted) assert(out.target_position == target && out.controlword == 0x0f);
  };
  check(620000, 619999, true);
  check(620000, 620001, false);
  check(60000, 59999, true);
  check(60000, 60001, false);
  check(10000, 10000, true);
  check(10000, 10001, true);
  check(10000, 9999, false);
  check(50000, 49999, true);
  check(50000, 50001, false);

  AxisLimits limits{10000, 50000, 10000, false, true};
  assert(limit_recovery_step(limits, 620000, 10000));
  assert(limit_recovery_step(limits, 620000, 619999.75));
  assert(!limit_recovery_step(limits, 620000, 620001));
  assert(limit_recovery_feedforward(limits, 620000, 20000, -1, 0));
  assert(!limit_recovery_feedforward(limits, 620000, 20000, 1, 0));
  assert(!limit_recovery_velocity(limits, 10000, -1));
  assert(!limit_recovery_velocity(limits, 50000, 1));
}

// A selected trusted snap is a feedback hold, including boundary jitter.
// Ordinary setpoints and untrusted feedback must retain their motion guards.
static void authorised_feedback_snap_tests() {
  auto config = configuration();
  config.limits.min_q = .01;
  config.limits.max_q = .1;
  auto input = enabled_input();
  input.feedback_position = 100;
  input.previous_target = 110;
  input.snap_hold = true;
  AxisOutputState state{};
  state.held_target_valid = true;
  state.held_target = 110;
  state.position_rate_active = true;
  state.position_rate.reset(110, input.previous_cycle_ns);
  auto out = evaluate_axis_output(config, input, state);
  assert(!out.fault && out.controlword == 0x0f && out.target_position == 100);
  assert(out.next.position_rate.previous == 100 && out.next.following_error.outside_since == 0);
  // One count of rate credit cannot authorise a 100-count ordinary command.
  input.snap_hold = false;
  input.setpoint_present = true;
  input.target_position = 210;
  out = evaluate_axis_output(config, input, state);
  assert(out.fault && (out.fault_reasons & FaultPositionRate));
  input.snap_hold = true;
  out = evaluate_axis_output(config, input, state);
  assert(out.fault && (out.fault_reasons & FaultPositionRate));
  input.target_position = 100;
  assert(evaluate_axis_output(config, input, state).fault);
  input.setpoint_present = false;
  input.feedback_fresh = false;
  out = evaluate_axis_output(config, input, state);
  assert(out.fault && (out.fault_reasons & FaultCoordinateReference));
  input.feedback_fresh = true;
  input.snap_hold = false;
  input.previous_target = 1000;
  state.held_target = 1000;
  state.arrest_cycles_left = 6;
  state.position_rate.reset(1000, input.previous_cycle_ns);
  for (int32_t feedback : {997, 1000, 1003, 1000, 997, 1003}) {
    input.feedback_position = feedback;
    out = evaluate_axis_output(config, input, state);
    assert(!out.fault && out.target_position == feedback && out.target_velocity == 0);
    assert(out.next.position_rate.previous == feedback && out.next.following_error.outside_since == 0);
    state = out.next;
    input.previous_target = feedback;
    input.previous_cycle_ns = input.now_ns;
    input.now_ns += 1000000;
  }
  assert(state.arrest_cycles_left == 0);
}

static void qualified_continuous_feedback_tests() {
  auto config = configuration();
  config.coordinate_evidence_required = true;
  config.limits.require_home = true;
  config.limits.following_error = 0.0002; // Twenty continuous counts.

  // The source has independently qualified this continuous coordinate. A
  // small disabled displacement preloads the observed continuous pose without
  // discarding Home or commanding the previous pose.
  auto input = enabled_input(0x23);
  input.home_valid = true;
  input.previous_target = 21999;
  input.feedback_position = 22010;
  input.continuous_feedback = 22010;
  input.continuous_feedback_valid = true;
  AxisOutputState state{};
  state.held_target = 21999;
  state.held_target_valid = true;
  auto out = evaluate_axis_output(config, input, state);
  assert(!out.fault && !out.invalidate_home && out.controlword == 0x0f);
  assert(out.target_position == 22010 && out.target_velocity == 0);
  assert(out.reference.disposition == ReferenceDisposition::Reconciled);
  assert(out.reference.target == 22010 && out.next.held_target == 22010);
  assert(out.next.held_target_valid && out.next.position_rate.previous == 22010);

  input.statusword = 0x27;
  input.previous_target = 22010;
  input.previous_cycle_ns = input.now_ns;
  input.now_ns += 1000000;
  out = evaluate_axis_output(config, input, out.next);
  assert(!out.fault && !out.invalidate_home && out.target_position == 22010);
  assert(out.controlword == 0x0f);

  // A five-count following lag persists longer than the error deadline. The
  // continuous evidence remains valid; ordinary servo lag is not loss of Home.
  input.setpoint_present = true;
  input.target_position = 22015;
  for (unsigned cycle = 0; cycle < 5; ++cycle) {
    input.previous_cycle_ns = input.now_ns;
    input.now_ns += 1000000;
    out = evaluate_axis_output(config, input, out.next);
    assert(!out.fault && !out.invalidate_home && out.controlword == 0x0f);
    assert(out.target_position == 22015 && out.next.held_target == 22015);
    assert(out.next.following_error.outside_since == 0);
    input.previous_target = out.target_position;
  }

  // A whole-turn error in either direction remains ordinary following error.
  // A stationary command isolates it from rate limits.
  for (int32_t continuous : {21000, 23000}) {
    input = enabled_input();
    input.home_valid = true;
    input.feedback_position = continuous;
    input.continuous_feedback = continuous;
    input.continuous_feedback_valid = true;
    input.previous_target = 22000;
    input.setpoint_present = true;
    input.target_position = 22000;
    state = {};
    state.held_target = 22000;
    state.held_target_valid = true;
    for (uint64_t now : {2000000ULL, 3000000ULL, 4000000ULL}) {
      input.now_ns = now;
      input.previous_cycle_ns = now - 1000000;
      out = evaluate_axis_output(config, input, state);
      assert(!out.fault && !out.invalidate_home && out.controlword == 0x0f);
      assert(out.target_position == 22000);
      assert(out.next.following_error.outside_since == 2000000);
      state = out.next;
    }
    input.now_ns = 5000000;
    input.previous_cycle_ns = 4000000;
    out = evaluate_axis_output(config, input, state);
    assert(out.fault && out.fault_reasons == FaultFollowingError);
    assert(out.controlword == 0 && out.target_velocity == 0);
    assert(!out.invalidate_home && !out.next.position_rate_active);
    assert(out.next.following_error.outside_since == 0);
  }

  // Disabling the optional Home requirement cannot authorize an unqualified
  // feedback source/coordinate. A cached valid Home must not substitute for
  // the caller's current continuous-evidence verdict.
  config.limits.require_home = false;
  for (uint16_t statusword : {uint16_t{0x40}, uint16_t{0x21},
                             uint16_t{0x23}, uint16_t{0x27}}) {
    input = enabled_input(statusword);
    input.home_valid = true;
    input.feedback_position = 0;
    input.previous_target = 22000;
    input.continuous_feedback = 0; // Invalid/default value must not become a measurement.
    input.continuous_feedback_valid = false;
    state = {};
    state.held_target = 22000;
    state.held_target_valid = true;
    input.now_ns = 10000000;
    input.previous_cycle_ns = 9000000;
    state.following_error.outside_since = 1; // Already beyond the tracking timeout.
    out = evaluate_axis_output(config, input, state);
    assert(out.fault && (out.fault_reasons & FaultCoordinateReference));
    assert(!(out.fault_reasons & FaultFollowingError));
    assert(out.invalidate_home && out.controlword == 0 && out.target_velocity == 0);
    assert(!out.next.position_rate_active);
  }

  // Qualified exterior feedback preloads enabled hold without changing the
  // reference that final group inhibition uses.
  input = enabled_input(0x23);
  input.feedback_position = 60010;
  input.continuous_feedback = 60010;
  input.continuous_feedback_valid = true;
  input.previous_target = 22010;
  out = evaluate_axis_output(config, input, {});
  assert(!out.fault && out.target_position == 60010);
  assert(out.reference.disposition == ReferenceDisposition::Invalid);
  assert(out.controlword == 0x0f && out.target_velocity == 0);

  // The disabled preload is not available once position output is active.
  // Fresh valid continuous evidence cannot excuse a discontinuous new command.
  input = enabled_input();
  input.feedback_position = 22010;
  input.continuous_feedback = 22010;
  input.continuous_feedback_valid = true;
  input.previous_target = 22010;
  input.setpoint_present = true;
  input.target_position = 21010;
  state = {};
  state.held_target = 22010;
  state.held_target_valid = true;
  out = evaluate_axis_output(config, input, state);
  assert(out.reference.disposition == ReferenceDisposition::Retain);
  assert(out.fault && out.fault_reasons == FaultPositionRate);
  assert(out.controlword == 0 && out.target_velocity == 0 && !out.invalidate_home);
  assert(!out.next.position_rate_active);
}

int main() {
  authorised_feedback_snap_tests();
  continuous_limit_recovery_tests();
  static_assert(std::is_trivially_copyable_v<AxisOutputState>);
  static_assert(std::is_trivially_copyable_v<AxisOutputDecision>);
  const auto config = configuration();

  // A normal enable sequence must emit shutdown, switch-on, then enable-op.
  AxisOutputState state{};
  uint64_t tick_ns = 2000000;
  for (const auto& row : {std::pair<uint16_t, uint16_t>{0x40, 0x06},
                          {0x21, 0x07}, {0x23, 0x0f}, {0x27, 0x0f}}) {
    auto input = enabled_input(row.first);
    input.now_ns = tick_ns;
    input.previous_cycle_ns = tick_ns - 1000000;
    const auto out = evaluate_axis_output(config, input, state);
    assert(out.controlword == row.second && out.target_position == 0);
    assert(out.mode == 8 && !out.fault);
    assert(out.target_torque == 0 && out.touch_probe_function == 0);
    assert(out.max_profile_velocity == 10000);
    state = out.next;
    tick_ns += 1000000;
  }

  // Fresh disabled movement is preloaded before Enable Op. It is not an
  // enabled position jump, and the held target must stay at the observed pose.
  auto displaced = enabled_input(0x23);
  displaced.feedback_position = 2000;
  auto out = evaluate_axis_output(config, displaced, {});
  assert(out.controlword == 0x0f && out.target_position == 2000 && !out.fault);
  assert(out.reference.target == 2000 && out.reference.disposition == ReferenceDisposition::Reconciled);
  assert(out.next.position_rate.previous == 2000 && out.next.held_target == 2000);
  displaced.statusword = 0x27;
  displaced.previous_target = 2000;
  displaced.previous_cycle_ns = displaced.now_ns;
  displaced.now_ns += 1000000;
  out = evaluate_axis_output(config, displaced, out.next);
  assert(!out.fault && out.target_position == 2000);
  displaced.setpoint_present = true;
  displaced.target_position = 0; // Stale pre-displacement target is still forbidden.
  displaced.previous_cycle_ns = displaced.now_ns;
  displaced.now_ns += 1000000;
  out = evaluate_axis_output(config, displaced, out.next);
  assert(out.fault && (out.fault_reasons & FaultPositionRate));
  assert(out.controlword == 0 && out.target_position == 2000 && out.target_velocity == 0);
  assert(!out.next.position_rate_active);

  // An axis still tracking its continuous command must retain its exact count
  // during Stop/Quick Stop; the final group inhibit uses reference.target.
  auto continuous = enabled_input();
  continuous.armed = false;
  continuous.feedback_position = 21999;
  continuous.previous_target = 21999;
  AxisOutputState held{};
  held.held_target = 21999;
  held.held_target_valid = true;
  for (uint16_t sw : {uint16_t{0x27}, uint16_t{0x07}}) {
    continuous.statusword = sw;
    out = evaluate_axis_output(config, continuous, held);
    assert(out.controlword == 0 && out.target_position == 21999);
    assert(out.reference.target == 21999 && out.reference.disposition == ReferenceDisposition::Retain);
  }
  continuous.statusword = 0x27;
  continuous.armed = true;
  held.quick_stop_cycles_left = 2;
  out = evaluate_axis_output(config, continuous, held);
  assert(out.controlword == 0x0b && out.target_position == 21999 && !out.fault);
  assert(out.next.quick_stop_cycles_left == 1 && held.quick_stop_cycles_left == 2);
  continuous.trajectory_active = true;
  out = evaluate_axis_output(config, continuous, held);
  assert(out.controlword == 0x0f && !out.fault); // Stop tail cannot override an active executor.

  // Existing snap/arrest semantics select the exact continuous count.
  continuous.trajectory_active = false;
  continuous.feedback_position = 22010;
  continuous.snap_hold = true;
  held.quick_stop_cycles_left = 0;
  held.arrest_cycles_left = 2;
  out = evaluate_axis_output(config, continuous, held);
  assert(out.target_position == 22010 && !out.fault && out.next.arrest_cycles_left == 1);
  assert(out.reference.target == 21999); // Whole-group Stop can still retain the submitted target.

  // Disabled linear feedback establishes only its exact observed coordinate.
  continuous.statusword = 0x40;
  continuous.snap_hold = false;
  continuous.armed = false;
  out = evaluate_axis_output(config, continuous, held);
  assert(!out.fault && !out.invalidate_home && out.target_position == 22010);
  assert(out.reference.disposition == ReferenceDisposition::Reconciled);

  // Home/service intent uses its explicit mode and controlword. Passive
  // startup inhibits that intent; it must never accidentally run the service.
  auto service = enabled_input(0x40);
  service.armed = false;
  service.enabled = false;
  service.service_active = true;
  service.service_controlword = 0x1f;
  service.service_mode = 6;
  service.feedback_position = 2000;
  out = evaluate_axis_output(config, service, {});
  assert(out.controlword == 0x1f && out.mode == 6 && out.target_position == 2000);
  assert(!out.fault && !out.next.position_rate_active && !out.next.held_target_valid);
  service.startup_passive = true;
  out = evaluate_axis_output(config, service, {});
  assert(out.controlword == 0 && out.mode == 0 && out.max_profile_velocity == 0);

  // Fault reset pulses stop when the state leaves Fault and never enable it.
  auto reset = enabled_input(0x08);
  reset.armed = false;
  reset.enabled = false;
  state = {};
  state.fault_reset_cycles_left = 2;
  out = evaluate_axis_output(config, reset, state);
  assert(out.controlword == 0x80 && out.invalidate_home && out.next.fault_reset_cycles_left == 1);
  reset.statusword = 0x40;
  out = evaluate_axis_output(config, reset, out.next);
  assert(out.controlword == 0 && out.next.fault_reset_cycles_left == 0);

  // Velocity feedforward cannot hide a rate violation, and excessive
  // feedforward is rejected even when the position itself is stationary.
  auto command = enabled_input();
  command.setpoint_present = true;
  command.target_position = 30;
  command.target_velocity = 0;
  out = evaluate_axis_output(config, command, {});
  assert(out.fault && (out.fault_reasons & FaultPositionRate));
  command.target_position = 0;
  command.target_velocity = 20000;
  out = evaluate_axis_output(config, command, {});
  assert(out.fault && out.target_velocity == 0 && out.controlword == 0);
  command.target_velocity = std::numeric_limits<double>::quiet_NaN();
  out = evaluate_axis_output(config, command, {});
  assert(out.fault && (out.fault_reasons & FaultCoordinateReference));

  // Following error is tolerated only until its configured monotonic deadline.
  auto following_config = config;
  following_config.limits.max_qd = 1;
  following_config.max_profile_velocity = 100000;
  following_config.limits.following_error = 0.0002;
  command = enabled_input();
  command.setpoint_present = true;
  command.target_position = 100;
  command.previous_target = 100;
  state = {};
  for (uint64_t now : {2000000ULL, 3000000ULL, 4000000ULL}) {
    command.now_ns = now;
    command.previous_cycle_ns = now - 1000000;
    out = evaluate_axis_output(following_config, command, state);
    assert(!out.fault);
    state = out.next;
  }
  command.now_ns = 5000000;
  command.previous_cycle_ns = 4000000;
  out = evaluate_axis_output(following_config, command, state);
  assert(out.fault && (out.fault_reasons & FaultFollowingError));
  assert(!out.next.following_error.outside_since);

  // Home offset is additive exactly once before sign/scaling conversion.
  auto negative = config;
  negative.sign = -1;
  negative.limits.min_q = -0.003;
  negative.limits.max_q = -0.001;
  auto offset = enabled_input();
  offset.feedback_position = 100;
  offset.previous_target = 100;
  offset.home_offset = 100;
  out = evaluate_axis_output(negative, offset, {});
  assert(!out.fault && out.target_position == 100);
  negative.sign = 0;
  out = evaluate_axis_output(negative, offset, {});
  assert(out.fault && (out.fault_reasons & FaultCoordinateReference));

  auto stale = enabled_input(0x40);
  stale.armed = false;
  stale.feedback_fresh = false;
  stale.feedback_position = 2000;
  out = evaluate_axis_output(config, stale, {});
  assert(out.reference.target == 0 && out.reference.disposition == ReferenceDisposition::Retain);
  stale.feedback_fresh = true;
  stale.feedback_position = 60000;
  stale.armed = true;
  out = evaluate_axis_output(config, stale, {});
  assert(!out.fault && out.target_position == 60000 && out.reference.target == 0);
  auto home_required = config;
  home_required.limits.require_home = true;
  out = evaluate_axis_output(home_required, enabled_input(), {});
  assert(out.fault && out.controlword == 0);

  qualified_continuous_feedback_tests();
  submitted_reference_tracking_tests();
  tracking_lag_allowance_tests();
  outside_limits_hold_tests();
  std::cout << "per-axis output decisions: passed\n";
}
