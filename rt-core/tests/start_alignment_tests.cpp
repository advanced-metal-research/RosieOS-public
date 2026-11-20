#include "motion_readiness.hpp"
#include "coordinate_model.hpp"
#include <cassert>
#include <cstdio>
#include <limits>

using namespace rosie::rtcore;

static MotionStartState ready() {
  MotionStartState state{};
  state.generation = 7;
  state.configuration_epoch = state.verified_configuration_epoch = 19;
  state.configured_mask = state.config_verified_mask = state.limits_valid_mask = 3;
  state.csp_mode_mask = state.enabled_mask = state.motion_ready_mask = 3;
  state.home_required_mask = state.home_valid_mask = 3;
  state.lease_valid = state.armed = state.bus_ready = true;
  return state;
}

static MotionStartAxis axis(double held, double first, double tolerance = 20) {
  MotionStartAxis out{};
  out.held_target = held; out.first_target = first;
  out.completion_tolerance_counts = tolerance;
  out.max_velocity_counts_per_s = 100;
  out.max_acceleration_counts_per_s2 = 500;
  return out;
}

int main() {
  const uint64_t cycle = 1'000'000, started = 10'000'000;
  const MotionStartRequest request{7, 0, 3, false, cycle};
  // Separate Start allowance for qualified, homed independent encoders. It
  // does not alter robot completion tolerance or any trajectory rate limit.
  // The floor is the axis profile's compiled start_alignment_floor: the the 1430 cell
  // drive profile's 1.332e-4 rad floored by gear, 50 counts at 18:1, 138 at
  // 50:1 and 277 at 100:1 (rtctl tests pin the conversion).
  const double pi = 3.14159265358979323846;
  const auto counts_per_rad = [&](double gear) { return 131072.0 * gear / (2 * pi); };
  const auto compiled_floor = [](double gear) { return gear == 18 ? 50. : gear == 50 ? 138. : 277.; };
  const double j4_tolerance = counts_per_rad(18) * .0001;
  auto allowance = [&](double tolerance, double gear = 18, unsigned gates = 31) {
    return independent_encoder_start_alignment_allowance(tolerance, compiled_floor(gear),
        gates & 1, gates & 2, gates & 4, gates & 8, gates & 16);
  };
  for (double gear : {100., 18., 50.}) {
    const double tolerance = counts_per_rad(gear) * .0001;
    const double floor_counts = allowance(tolerance, gear);
    assert(floor_counts == compiled_floor(gear));
    const double inside = std::floor(floor_counts);
    for (double distance : {-(inside + 1), -inside, -45., 45., inside, inside + 1}) {
      std::array<MotionStartAxis, 2> axes{axis(1179648 + distance, 1179648, floor_counts), axis(0, 0)};
      const auto decision = evaluate_motion_start(ready(), request, axes);
      assert(decision.activate == (std::abs(distance) <= inside));
      if (decision.activate) assert(decision.alignment_duration_ns > 0);
    }
  }
  assert(std::floor(allowance(j4_tolerance)) == 50);
  assert(std::floor(allowance(counts_per_rad(100) * .0001, 100)) == 277);
  assert(std::floor(allowance(counts_per_rad(50) * .0001, 50)) == 138);
  // A completion tolerance above the floor is kept unchanged.
  const double wide = counts_per_rad(100) * .0006;
  assert(allowance(wide, 100) == wide);
  for (unsigned gates = 0; gates < 31; ++gates) {
    assert(allowance(j4_tolerance, 18, gates) == j4_tolerance);
    std::array<MotionStartAxis, 2> axes{axis(1179603, 1179648, allowance(j4_tolerance, 18, gates)), axis(0, 0)};
    assert(!evaluate_motion_start(ready(), request, axes).activate);
  }
  for (double invalid : {-1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    const auto preserved = allowance(invalid);
    assert(std::isnan(invalid) ? std::isnan(preserved) : preserved == invalid);
    assert(independent_encoder_start_alignment_allowance(10, invalid, true, true, true, true, true) == 10);
  }
  const std::array<MotionStartAxis, 2> floored{axis(1179603, 1179648, allowance(j4_tolerance)), axis(0, 0)};
  for (unsigned failure = 0; failure < 9; ++failure) {
    auto axes = floored;
    auto state = ready();
    auto req = request;
    if (failure == 0) axes[0].first_velocity = .01;
    if (failure == 1) axes[0].max_velocity_counts_per_s = 0;
    if (failure == 2) axes[0].max_acceleration_counts_per_s2 = 0;
    if (failure == 3) axes[0].limit_direction_valid = false;
    if (failure == 4) req.first_time_ns = 1;
    if (failure == 5) state.home_valid_mask = 0;
    if (failure == 6) ++req.generation;
    if (failure == 7) state.lease_valid = false;
    if (failure == 8) ++state.verified_configuration_epoch;
    const auto decision = evaluate_motion_start(state, req, axes);
    assert(!decision.activate);
    if (failure >= 5) assert(decision.reason == MotionStartReason::NotReady && decision.discontinuous_mask == 0);
  }
  for (double distance : {3., 13., 20., -3., -13., -20.}) {
    std::array<MotionStartAxis, 2> axes{axis(137, 137 + distance), axis(-50, -47)};
    const auto decision = evaluate_motion_start(ready(), request, axes);
    assert(decision.activate && decision.alignment_duration_ns > 0);
    assert(decision.alignment_duration_ns % cycle == 0);
    StartAlignment<2> alignment{};
    alignment.begin(axes, 3, started, decision.alignment_duration_ns);
    std::array<double, 2> position{}, velocity{}, previous_position{137, -50}, previous_velocity{};
    std::array<PositionRateGuard, 2> guards{};
    for (int i = 0; i < 2; ++i) guards[i].reset(axes[i].held_target, started);
    for (uint64_t elapsed = 0; elapsed <= decision.alignment_duration_ns; elapsed += cycle) {
      const bool finished = alignment.sample(started + elapsed, position, velocity);
      assert(finished == (elapsed == decision.alignment_duration_ns));
      for (int i = 0; i < 2; ++i) {
        assert(position[i] >= std::min(axes[i].held_target, axes[i].first_target));
        assert(position[i] <= std::max(axes[i].held_target, axes[i].first_target));
        assert(std::abs(velocity[i]) <= axes[i].max_velocity_counts_per_s);
        if (elapsed) {
          assert((position[i] - previous_position[i]) *
              (axes[i].first_target - axes[i].held_target) >= -1e-9);
          assert(std::abs(velocity[i] - previous_velocity[i]) <= axes[i].max_acceleration_counts_per_s2 * .001 + 1e-9);
          assert(guards[i].accept(std::round(position[i]), started + elapsed, axes[i].max_velocity_counts_per_s));
        }
        previous_position[i] = position[i]; previous_velocity[i] = velocity[i];
      }
    }
    assert(!alignment.active && position[0] == axes[0].first_target && velocity[0] == 0);
    assert(position[1] == axes[1].first_target && velocity[1] == 0);
  }

  std::array<MotionStartAxis, 2> axes{axis(0, 20.00001), axis(0, 0)};
  assert(!evaluate_motion_start(ready(), request, axes).activate);
  axes[0] = axis(0, 13);
  // Every requested first derivative must be stationary, even on an axis
  // requiring no positional alignment, to avoid a hidden velocity jump.
  axes[1].first_velocity = .01;
  assert(!evaluate_motion_start(ready(), request, axes).activate);
  auto subgroup = request; subgroup.axis_mask = 1;
  assert(evaluate_motion_start(ready(), subgroup, axes).activate);
  axes[1].first_velocity = 0;
  for (double invalid : {0., -1., std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid_axes = axes;
    invalid_axes[0].max_acceleration_counts_per_s2 = invalid;
    assert(!evaluate_motion_start(ready(), request, invalid_axes).activate);
    invalid_axes = axes; invalid_axes[0].max_velocity_counts_per_s = invalid;
    assert(!evaluate_motion_start(ready(), request, invalid_axes).activate);
  }
  auto nonfinite = axes;
  nonfinite[0].completion_tolerance_counts = std::numeric_limits<double>::quiet_NaN();
  assert(!evaluate_motion_start(ready(), request, nonfinite).activate);
  nonfinite = axes; nonfinite[0].first_target = std::numeric_limits<double>::infinity();
  assert(!evaluate_motion_start(ready(), request, nonfinite).activate);
  nonfinite = axes; nonfinite[0].first_velocity = std::numeric_limits<double>::quiet_NaN();
  assert(!evaluate_motion_start(ready(), request, nonfinite).activate);
  auto late = request; late.first_time_ns = 1;
  assert(!evaluate_motion_start(ready(), late, axes).activate);
  auto cancelled = request; cancelled.cancelled = true;
  assert(!evaluate_motion_start(ready(), cancelled, axes).activate);
  auto expired = ready(); expired.lease_valid = false;
  assert(!evaluate_motion_start(expired, request, axes).activate);
  auto unarmed = ready(); unarmed.armed = false;
  assert(!evaluate_motion_start(unarmed, request, axes).activate);
  auto outward = axes; outward[0].limit_direction_valid = false;
  assert(evaluate_motion_start(ready(), request, outward).reason == MotionStartReason::OutsideLimitsOutward);

  // Existing single-count quantization admission is unchanged and needs no
  // alignment limits, while finite >1-count offsets require explicit tolerance.
  std::array<MotionStartAxis, 2> exact{{{1, 0, 0}, {0, 0, 0}}};
  assert(evaluate_motion_start(ready(), request, exact).alignment_duration_ns == 0);
  exact[0].first_target = 1.01;
  assert(!evaluate_motion_start(ready(), request, exact).activate);

  // Continuous commands retain their complete count. The old 999 -> 2 seam
  // shortcut is refused; the exact nearby target is 1002.
  axes = {axis(999, 2, 4), axis(0, 0)};
  assert(!evaluate_motion_start(ready(), request, axes).activate);
  axes[0].first_target = 1002;
  auto continuous = evaluate_motion_start(ready(), request, axes);
  assert(continuous.activate && continuous.alignment_duration_ns);
  StartAlignment<2> alignment{};
  alignment.begin(axes, 3, started, continuous.alignment_duration_ns);
  std::array<double, 2> position{}, velocity{};
  alignment.sample(started + continuous.alignment_duration_ns / 2, position, velocity);
  assert(std::abs(command_delta(position[0], 999) - 1.5) < 1e-9);
  assert(alignment.sample(started + continuous.alignment_duration_ns + cycle * 5, position, velocity));
  assert(position[0] == 1002 && velocity[0] == 0);

  // A reversed geared coordinate with Home offset must use DELTA conversion
  // for tolerance/rates, and the actual held wire count as its initial state.
  AxisCoordinateProfile profile{};
  profile.counts_per_unit = 1000; profile.sign = -1; profile.home_offset = 137;
  AxisCoordinateModel model(profile);
  const double zero = model.logical_to_counts(0).counts;
  axes = {axis(zero + 13, zero, std::abs(model.logical_delta_to_counts(.02))), axis(0, 0)};
  axes[0].max_velocity_counts_per_s = std::abs(model.logical_delta_to_counts(.1));
  axes[0].max_acceleration_counts_per_s2 = std::abs(model.logical_delta_to_counts(.5));
  const auto reverse = evaluate_motion_start(ready(), request, axes);
  assert(reverse.activate);
  alignment.begin(axes, 3, started, reverse.alignment_duration_ns);
  alignment.sample(started, position, velocity);
  assert(position[0] == zero + 13 && velocity[0] == 0);
  alignment.cancel();
  position.fill(9999); velocity.fill(9999);
  assert(!alignment.sample(started + reverse.alignment_duration_ns, position, velocity));
  assert(position[0] == 9999 && velocity[0] == 9999); // Cancelled state cannot revive.
  std::puts("start alignment tests passed");
}
