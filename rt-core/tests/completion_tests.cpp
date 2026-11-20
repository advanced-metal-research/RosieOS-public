// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "engine/completion.hpp"
#include "motion_readiness.hpp"
#include "start_alignment.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <type_traits>

using namespace rosie::rtcore;

static CompletionInput sample(uint64_t elapsed, uint32_t axes = 1) {
  CompletionInput input{};
  input.elapsed_ns = elapsed;
  input.endpoint_ns = 1000;
  input.axis_mask = axes;
  input.fresh_ready_mask = axes;
  input.timeout_ns[0] = 100;
  input.timeout_ns[1] = 200;
  return input;
}

// Candidate23 calibrates an existing shared position-acceptance field. It
// changes endpoint and stationary-start matching, not their other admission rules.
static void j6_shared_position_acceptance_boundaries() {
  constexpr double counts_per_radian = 6553600.0 / (2.0 * 3.14159265358979323846);
  for (const double tolerance : {0.0001, 0.0002}) {
    const double counts = tolerance * counts_per_radian;
    const int last_inside = int(std::floor(counts));
    assert(last_inside == (tolerance == 0.0001 ? 104 : 208));
    for (const int error : {last_inside, last_inside + 1, 163}) {
      CompletionInput in{};
      in.axis_mask = in.fresh_ready_mask = 32;
      in.endpoint_ns = 20000000000ULL;
      in.elapsed_ns = in.endpoint_ns + 1000000000ULL;
      in.timeout_ns[5] = 1000000000ULL;
      if (error <= counts) in.in_tolerance_mask = 32;
      const auto result = evaluate_completion(in, {});
      assert(result.complete == (error <= counts));
      assert(result.fault_mask == (error <= counts ? 0u : 32u));
      in.fresh_ready_mask = 0;
      assert(evaluate_completion(in, {}).fault_mask == 32);
      in.fresh_ready_mask = 32;
      ++in.elapsed_ns;
      assert(evaluate_completion(in, {}).fault_mask == 32);

      std::array<MotionStartAxis, 6> axes{};
      auto& axis = axes[5];
      axis.first_target = error;
      axis.held_target = 0;
      // The compiled start_alignment_floor on this 50:1 joint: 1.332e-4 rad
      // from the drive profile, floored to 138 counts.
      const double floor_counts = 138;
      axis.completion_tolerance_counts = independent_encoder_start_alignment_allowance(
          counts, floor_counts, true, true, true, true, true);
      axis.max_velocity_counts_per_s = 100;
      axis.max_acceleration_counts_per_s2 = 1000;
      const auto duration = start_alignment_duration_ns(axes, 32, 1000000);
      assert(bool(duration) == (error <= std::max(counts, floor_counts)));
      if (duration) assert(double(duration) / 1e9 >= 2.0 * error / 100);
      axis.first_velocity = 1;
      assert(start_alignment_duration_ns(axes, 32, 1000000) == 0);
    }
  }
}

int main() {
  j6_shared_position_acceptance_boundaries();
  static_assert(std::is_trivially_copyable_v<CompletionState>);
  static_assert(std::is_trivially_copyable_v<CompletionDecision>);
  static_assert(FaultCompletionTimeout == 512);

  // Passing through the final coordinate before the endpoint is not completion.
  auto input = sample(999);
  input.in_tolerance_mask = 1;
  auto out = evaluate_completion(input, {});
  assert(!out.endpoint_due && !out.complete && !out.fault_mask);
  assert(out.state.on_time_mask == 0);
  input.elapsed_ns = 1000;
  out = evaluate_completion(input, out.state);
  assert(out.endpoint_due && out.complete && !out.fault_mask);

  // The configured timeout is a bound, not a mandatory dwell period.
  input = sample(1099);
  out = evaluate_completion(input, {});
  assert(!out.complete && !out.fault_mask);
  input.in_tolerance_mask = 1;
  out = evaluate_completion(input, out.state);
  assert(out.complete && !out.fault_mask);

  // At the exact deadline fresh settled feedback wins. A sample first observed
  // afterwards cannot prove the endpoint was reached before the deadline.
  input = sample(1100);
  input.in_tolerance_mask = 1;
  out = evaluate_completion(input, {});
  assert(out.complete && !out.fault_mask);
  input.elapsed_ns = 1101;
  out = evaluate_completion(input, {});
  assert(!out.complete && out.fault_mask == 1);

  // A timeout is terminal for this execution, even if the next sample settles.
  input = sample(1100);
  out = evaluate_completion(input, {});
  assert(!out.complete && out.fault_mask == 1);
  input.elapsed_ns = 1101;
  input.in_tolerance_mask = 1;
  out = evaluate_completion(input, out.state);
  assert(!out.complete && out.fault_mask == 1);

  // Correct coordinates from stale or unready feedback cannot prove completion.
  input = sample(1090);
  input.in_tolerance_mask = 1;
  input.fresh_ready_mask = 0;
  out = evaluate_completion(input, {});
  assert(!out.complete && !out.fault_mask && !out.state.on_time_mask);
  input.elapsed_ns = 1100;
  out = evaluate_completion(input, out.state);
  assert(!out.complete && out.fault_mask == 1);

  // A settled short-timeout axis remains eligible while a second selected axis
  // uses its longer timeout. Only the still-unsettled expired axes fault.
  input = sample(1050, 3);
  input.in_tolerance_mask = 1;
  out = evaluate_completion(input, {});
  assert(!out.complete && !out.fault_mask && out.state.on_time_mask == 1);
  input.elapsed_ns = 1100;
  out = evaluate_completion(input, out.state);
  assert(!out.complete && !out.fault_mask);
  const auto short_axis_settled = out.state;
  input.elapsed_ns = 1199;
  input.in_tolerance_mask = 3;
  out = evaluate_completion(input, short_axis_settled);
  assert(out.complete && !out.fault_mask);
  input.elapsed_ns = 1200;
  input.in_tolerance_mask = 1;
  out = evaluate_completion(input, short_axis_settled);
  assert(!out.complete && out.fault_mask == 2);
  input = sample(1100, 3);
  input.in_tolerance_mask = 2;
  out = evaluate_completion(input, {});
  assert(!out.complete && out.fault_mask == 1);

  // Settling evidence cannot survive a later excursion or loss of readiness.
  input = sample(1150, 3);
  input.in_tolerance_mask = 0;
  out = evaluate_completion(input, short_axis_settled);
  assert(!out.complete && out.fault_mask == 1);
  input.in_tolerance_mask = 1;
  input.fresh_ready_mask = 2;
  out = evaluate_completion(input, short_axis_settled);
  assert(!out.complete && out.fault_mask == 1);
  input = sample(1090, 3);
  out = evaluate_completion(input, short_axis_settled);
  assert(!out.complete && !out.fault_mask && !out.state.on_time_mask);
  input.elapsed_ns = 1150;
  input.in_tolerance_mask = 3;
  out = evaluate_completion(input, out.state);
  assert(!out.complete && out.fault_mask == 1);

  // Nonparticipating axes do not impose deadlines or readiness requirements.
  input = sample(1150, 2);
  input.timeout_ns[0] = 0;
  input.in_tolerance_mask = 2;
  out = evaluate_completion(input, {});
  assert(out.complete && !out.fault_mask);

  // Invalid/missing native configuration fails closed; an empty selection must
  // not be mistaken for an executable trajectory that has completed.
  input = sample(1000);
  input.timeout_ns[0] = 0;
  input.in_tolerance_mask = 1;
  out = evaluate_completion(input, {});
  assert(!out.complete && out.fault_mask == 1);
  input = sample(1000, 0);
  out = evaluate_completion(input, {});
  assert(!out.complete);
  input = sample(1000, 1u << 16);
  out = evaluate_completion(input, {});
  assert(!out.complete && out.fault_mask == (1u << 16));

  // Endpoint + timeout would overflow here; the elapsed interval remains valid.
  input = sample(std::numeric_limits<uint64_t>::max() - 10);
  input.endpoint_ns = std::numeric_limits<uint64_t>::max() - 50;
  out = evaluate_completion(input, {});
  assert(out.endpoint_due && !out.complete && !out.fault_mask);
  input.elapsed_ns = std::numeric_limits<uint64_t>::max();
  input.endpoint_ns = std::numeric_limits<uint64_t>::max() - 100;
  out = evaluate_completion(input, {});
  assert(!out.complete && out.fault_mask == 1);
  std::cout << "endpoint completion policy: passed\n";
}
