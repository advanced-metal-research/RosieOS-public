// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "output_policy.hpp"
#include "execution_policy.hpp"
#include <cassert>
#include <iostream>
#include <limits>

// Native Home service submits the descriptor's controlwords with a target that
// tracks feedback at every step, as on origin/main. There is no fixed-target
// pretrigger hold: the vendor-confirmed HM35 transaction defines none.
static void home_service_target_tracks_feedback_at_every_step() {
  using namespace rosie::rtcore;
  AxisOutputConfig config{};
  config.counts_per_unit = 100000; config.sign = 1;
  config.max_profile_velocity = 100000; config.limits.valid = true;
  config.limits.min_q = -100; config.limits.max_q = 100;
  config.limits.max_qd = 1;
  AxisOutputInput input{};
  input.statusword = 0x23; input.feedback_position = 1000;
  input.feedback_fresh = input.config_verified = input.configuration_current = true;
  input.service_active = true; input.service_mode = 6;
  input.now_ns = 1'000'000;
  AxisOutputState state{};
  int32_t observed = 1000;
  for (const uint16_t cw : {uint16_t(0), uint16_t(6), uint16_t(7), uint16_t(15)}) {
    observed += 8;
    input.service_controlword = cw; input.feedback_position = observed; input.now_ns += 1'000'000;
    const auto out = evaluate_axis_output(config, input, state);
    assert(out.controlword == cw && out.mode == 6 && out.target_position == observed);
    assert(!input.armed && !input.home_valid);
    state = out.next;
  }
  // The start bit still waits for brake release qualification.
  input.service_controlword = 31; input.brake_motion_permitted = false;
  input.feedback_position = 1012; input.now_ns += 1'000'000;
  auto trigger_wait = evaluate_axis_output(config, input, state);
  assert(trigger_wait.controlword == 15 && trigger_wait.target_position == 1012);
  input.statusword = 0x27; input.brake_motion_permitted = true;
  input.feedback_position = -6553600; input.home_offset = -6553600;
  input.now_ns += 1'000'000;
  auto rebased = evaluate_axis_output(config, input, trigger_wait.next);
  assert(rebased.controlword == 31 && rebased.target_position == -6553600);
  input.feedback_position = -6553597; input.now_ns += 1'000'000;
  assert(evaluate_axis_output(config, input, rebased.next).target_position == -6553597);
}

int main() {
  home_service_target_tracks_feedback_at_every_step();
  using namespace rosie::rtcore;
  using State = gradient::ds402::State;
  AxisLimits limits{-0.5, 0.5, 0.1, false, true};
  auto reference = [&](State state, bool fresh, bool valid, int32_t feedback,
                       int32_t previous = 0, double scale = 100000,
                       double sign = 1, double offset = 0) {
    return inhibited_reference(state, fresh, valid, feedback, true, previous,
        offset, scale, sign, limits);
  };

  // The external displacement is much larger than one enabled-cycle motion
  // allowance, but a disabled observed preload must not command a return to 0.
  for (auto state : {State::SwitchOnDisabled, State::ReadyToSwitchOn, State::SwitchedOn}) {
    const auto result = reference(state, true, true, 2000);
    assert(result.disposition == ReferenceDisposition::Reconciled);
    assert(result.target == 2000);
    PositionRateGuard rate;
    rate.reset(result.target, 1000000);
    assert(rate.accept(2000, 2000000, 10000));
    assert(!rate.accept(0, 3000000, 10000)); // Old target still fails hard rate guard.
  }

  for (auto state : {State::Unknown, State::NotReady, State::OperationEnabled,
                     State::QuickStopActive, State::FaultReactionActive, State::Fault}) {
    const auto result = reference(state, true, true, 2000);
    assert(result.disposition == ReferenceDisposition::Retain && result.target == 0);
  }
  assert(reference(State::SwitchedOn, false, true, 2000).disposition == ReferenceDisposition::Retain);
  assert(reference(State::SwitchedOn, true, false, 2000).disposition == ReferenceDisposition::Retain);
  assert(reference(State::SwitchedOn, true, true, 60000).disposition == ReferenceDisposition::Invalid);
  assert(reference(State::SwitchedOn, true, true, 2000, 0, 0).disposition == ReferenceDisposition::Invalid);
  assert(reference(State::SwitchedOn, true, true, 2000, 0, 100000, 0).disposition == ReferenceDisposition::Invalid);
  assert(reference(State::SwitchedOn, true, true, 2000, 0, 100000, 1,
      std::numeric_limits<double>::quiet_NaN()).disposition == ReferenceDisposition::Invalid);
  assert(reference(State::SwitchedOn, true, true, -2000, 0, 100000, -1).target == -2000);
  assert(reference(State::SwitchedOn, true, true, 49000, 0, 100000, 1, 2000).disposition == ReferenceDisposition::Invalid);

  // Disabled reconciliation keeps the complete signed feedback count.
  const auto multi_turn = reference(State::SwitchedOn, true, true, 21999, 999);
  assert(multi_turn.disposition == ReferenceDisposition::Reconciled && multi_turn.target == 21999);
  assert(inhibited_reference(State::SwitchedOn, true, true, 0, false, 0,
      0, 100000, 1, limits).disposition == ReferenceDisposition::Reconciled);
  std::cout << "output/re-arm policy: passed\n";
}
