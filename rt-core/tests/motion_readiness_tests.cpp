// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "motion_readiness.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>

// Exhaust every combination so a lower-priority failure cannot mask the first
// refusal, especially simultaneous missing Home and invalid coordinates.
void test_axis_readiness_priority() {
  const char* reasons[] = {"faulted", "mode_mismatch", "home_required", "coordinate_invalid",
                          "not_enabled", "not_operation_enabled", "brake_wait", "ready"};
  for (unsigned failures = 0; failures < 128; ++failures) {
    unsigned first = 0;
    while (first < 7 && !(failures & (1u << first))) ++first;
    const auto reason = rosie::rtcore::axis_readiness_reason(failures & 1 ? "faulted" : nullptr,
        !(failures & 2), !(failures & 4), !(failures & 8), !(failures & 16),
        !(failures & 32), !(failures & 64));
    assert(std::strcmp(reason, reasons[first]) == 0);
  }
  std::puts("PASS test_axis_readiness_priority: 128 combinations");
}

int main() {
  using namespace rosie::rtcore;
  test_axis_readiness_priority();
  for (uint16_t sw : {uint16_t(0), uint16_t(0x40), uint16_t(0x21),
                     uint16_t(0x23), uint16_t(0x07), uint16_t(0x0f), uint16_t(0x08)})
    assert(!motion_axis_ready(true, sw, 8, 0));
  assert(motion_axis_ready(true, 0x27, 8, 0));
  assert(motion_axis_ready(true, 0x1227, 8, 0));
  assert(!motion_axis_ready(false, 0x27, 8, 0));
  assert(!motion_axis_ready(true, 0x27, 6, 0));
  assert(!motion_axis_ready(true, 0x27, 8, 0x1234));
  assert(!motion_group_ready(0, 3, 3));
  assert(!motion_group_ready(4, 3, 7));
  assert(!motion_group_ready(3, 3, 1));
  assert(motion_group_ready(1, 3, 1));
  assert(motion_group_ready(3, 3, 3));

  MotionStartState ready{};
  ready.generation = 7;
  ready.configuration_epoch = ready.verified_configuration_epoch = 19;
  ready.configured_mask = ready.config_verified_mask = ready.limits_valid_mask = 3;
  ready.csp_mode_mask = ready.enabled_mask = ready.motion_ready_mask = 3;
  ready.home_required_mask = ready.home_valid_mask = 1;
  ready.lease_valid = ready.armed = ready.bus_ready = true;
  MotionStartRequest request{7, 0, 3, false};
  std::array<MotionStartAxis, 2> axes{{{100, 100, true}, {-200, -200, true}}};
  const auto accepted = evaluate_motion_start(ready, request, axes);
  assert(accepted.activate && accepted.reason == MotionStartReason::None && !accepted.discontinuous_mask);
  axes[0].limit_direction_valid = false;
  auto exterior = evaluate_motion_start(ready, request, axes);
  assert(!exterior.activate && exterior.reason == MotionStartReason::OutsideLimitsOutward &&
      exterior.discontinuous_mask == 0);
  auto unhomed = ready;
  unhomed.home_valid_mask = 0;
  assert(evaluate_motion_start(unhomed, request, axes).reason == MotionStartReason::NotReady);
  axes[0].limit_direction_valid = true;

  auto reject = [&](const char* condition, const MotionStartState& state,
                    const MotionStartRequest& candidate, MotionStartReason reason) {
    const auto decision = evaluate_motion_start(state, candidate, axes);
    if (decision.activate || decision.reason != reason || decision.discontinuous_mask != 0) {
      std::fprintf(stderr, "Start admitted or misclassified %s\n", condition);
      assert(false);
    }
  };

  struct BooleanRequirement {
    const char* name;
    bool MotionStartState::*member;
  };
  for (const auto& requirement : std::array<BooleanRequirement, 3>{{
      {"expired authority", &MotionStartState::lease_valid},
      {"unarmed group", &MotionStartState::armed},
      {"unready bus", &MotionStartState::bus_ready}}}) {
    auto state = ready;
    state.*requirement.member = false;
    reject(requirement.name, state, request, MotionStartReason::NotReady);
  }
  struct MaskRequirement {
    const char* name;
    uint32_t MotionStartState::*member;
  };
  for (const auto& requirement : std::array<MaskRequirement, 7>{{
      {"unconfigured group axis", &MotionStartState::configured_mask},
      {"unverified drive configuration", &MotionStartState::config_verified_mask},
      {"invalid axis limits", &MotionStartState::limits_valid_mask},
      {"missing home evidence", &MotionStartState::home_valid_mask},
      {"non-CSP mode", &MotionStartState::csp_mode_mask},
      {"disabled group axis", &MotionStartState::enabled_mask},
      {"group axis not Operation Enabled", &MotionStartState::motion_ready_mask}}}) {
    auto state = ready;
    state.*requirement.member &= ~1u;
    reject(requirement.name, state, request, MotionStartReason::NotReady);
  }
  for (uint64_t generation : {uint64_t(6), uint64_t(8)}) {
    auto candidate = request;
    candidate.generation = generation;
    reject("stale/future generation", ready, candidate, MotionStartReason::NotReady);
  }
  for (uint64_t verified_epoch : {uint64_t(18), uint64_t(20)}) {
    auto state = ready;
    state.verified_configuration_epoch = verified_epoch;
    reject("configuration identity mismatch", state, request, MotionStartReason::NotReady);
  }
  for (uint32_t mask : {uint32_t(0), uint32_t(4), uint32_t(5)}) {
    auto candidate = request;
    candidate.axis_mask = mask;
    reject("empty/unknown group", ready, candidate, MotionStartReason::NotReady);
  }
  auto unavailable = ready;
  unavailable.configured_mask = unavailable.config_verified_mask = unavailable.limits_valid_mask = 7;
  unavailable.csp_mode_mask = unavailable.enabled_mask = unavailable.motion_ready_mask = 7;
  auto missing_axis = request;
  missing_axis.axis_mask = 4;
  reject("axis without bounded coordinate evidence", unavailable, missing_axis, MotionStartReason::NotReady);
  auto cancelled = request;
  cancelled.cancelled = true;
  reject("cancelled Ready slot", ready, cancelled, MotionStartReason::NotReady);
  auto faulted = ready;
  faulted.safety_fault_mask = 2;
  reject("latched fault", faulted, request, MotionStartReason::NotReady);
  for (const auto& conflict : std::array<BooleanRequirement, 3>{{
      {"executing trajectory", &MotionStartState::trajectory_active},
      {"active jog", &MotionStartState::jog_active},
      {"commissioning active or draining", &MotionStartState::commissioning_active}}}) {
    auto state = ready;
    state.*conflict.member = true;
    reject(conflict.name, state, request, MotionStartReason::ModeConflict);
    // Keep the existing conflict precedence even if authority has also expired.
    state.lease_valid = false;
    reject(conflict.name, state, cancelled, MotionStartReason::ModeConflict);
  }

  // Missing home/enable/config evidence on an unrequested axis is not a reason
  // to reject a valid subgroup. Home is required only where the profile says so.
  auto subgroup = request;
  subgroup.axis_mask = 1;
  auto only_one = ready;
  only_one.config_verified_mask = only_one.limits_valid_mask = only_one.csp_mode_mask = 1;
  only_one.enabled_mask = only_one.motion_ready_mask = 1;
  assert(evaluate_motion_start(only_one, subgroup, axes).activate);
  auto no_home_required = ready;
  no_home_required.home_required_mask = no_home_required.home_valid_mask = 0;
  assert(evaluate_motion_start(no_home_required, request, axes).activate);

  auto late_first_point = request;
  late_first_point.first_time_ns = 1;
  auto decision = evaluate_motion_start(ready, late_first_point, axes);
  assert(!decision.activate && decision.reason == MotionStartReason::InvalidTrajectory);
  assert(decision.discontinuous_mask == 3); // Nonzero origin affects the whole group.
  axes[1].first_target = -202;
  decision = evaluate_motion_start(ready, request, axes);
  assert(!decision.activate && decision.reason == MotionStartReason::InvalidTrajectory);
  assert(decision.discontinuous_mask == 2);
  axes[0].first_target = 101;
  assert(evaluate_motion_start(ready, subgroup, axes).activate); // One-count quantization is retained.
  axes[0].first_target = 101.01;
  assert(evaluate_motion_start(ready, subgroup, axes).discontinuous_mask == 1);
  axes[0] = {1000, 999, true};
  assert(evaluate_motion_start(ready, subgroup, axes).activate);
  axes[0].first_target = 0;
  assert(evaluate_motion_start(ready, subgroup, axes).discontinuous_mask == 1);
  axes[0].first_target = std::numeric_limits<double>::quiet_NaN();
  assert(evaluate_motion_start(ready, subgroup, axes).discontinuous_mask == 1);
  axes[0].first_target = 100;
  axes[0].held_target = std::numeric_limits<double>::infinity();
  assert(evaluate_motion_start(ready, subgroup, axes).discontinuous_mask == 1);
  // Rejected authority must not manufacture a continuity fault from bad data.
  auto expired = ready;
  expired.lease_valid = false;
  reject("invalid first point without authority", expired, subgroup, MotionStartReason::NotReady);
  std::puts("motion readiness tests passed");
}
