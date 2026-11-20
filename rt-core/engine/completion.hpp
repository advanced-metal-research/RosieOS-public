#pragma once
#include "control_protocol.hpp"
#include "execution_policy.hpp"
#include <array>
#include <cstdint>

namespace rosie::rtcore {

// State belongs to one native execution and is reset when that execution starts.
// An early-settled axis may wait for another axis with a longer configured
// timeout, but must remain settled and ready throughout that wait.
struct CompletionState {
  uint32_t on_time_mask = 0;
  uint32_t timeout_mask = 0;
};

struct CompletionInput {
  uint64_t elapsed_ns = 0;
  uint64_t endpoint_ns = 0;
  uint32_t axis_mask = 0;
  uint32_t fresh_ready_mask = 0;
  uint32_t in_tolerance_mask = 0;
  std::array<uint64_t, protocol::max_axes> timeout_ns{};
};

struct CompletionDecision {
  CompletionState state{};
  bool endpoint_due = false;
  bool complete = false;
  uint32_t fault_mask = 0;
};

// Both times use the execution's native monotonic origin. The deadline never
// moves with feedback arrival. Subtraction after ordering avoids overflowing
// endpoint + timeout. Completion at the exact deadline wins only with fresh,
// ready feedback already inside tolerance; an unsettled axis expires there.
inline CompletionDecision evaluate_completion(const CompletionInput& input,
                                               CompletionState state) noexcept {
  CompletionDecision out{state, input.elapsed_ns >= input.endpoint_ns, false,
                         state.timeout_mask};
  if (!out.endpoint_due || !input.axis_mask) return out;
  static_assert(protocol::max_axes < 32);
  constexpr uint32_t configured_axes = (1u << protocol::max_axes) - 1;
  out.state.timeout_mask |= input.axis_mask & ~configured_axes;
  const uint64_t settling_ns = input.elapsed_ns - input.endpoint_ns;
  const uint32_t settled = input.axis_mask & input.fresh_ready_mask &
                           input.in_tolerance_mask;
  for (uint32_t axis = 0; axis < protocol::max_axes; ++axis) {
    const uint32_t bit = 1u << axis;
    if (!(input.axis_mask & bit)) continue;
    const uint64_t timeout = input.timeout_ns[axis];
    if (!timeout) {
      out.state.timeout_mask |= bit;
      continue;
    }
    if (!(settled & bit)) out.state.on_time_mask &= ~bit;
    else if (settling_ns <= timeout) out.state.on_time_mask |= bit;
    // A first acceptable sample arriving after the deadline cannot prove the
    // axis settled on time. A prior on-time sample remains valid only while
    // every subsequent observation is also fresh, ready and in tolerance.
    if (settling_ns >= timeout && !(settled & out.state.on_time_mask & bit))
      out.state.timeout_mask |= bit;
  }
  out.fault_mask = out.state.timeout_mask;
  out.complete = !out.fault_mask &&
      (settled & out.state.on_time_mask) == input.axis_mask;
  return out;
}

} // namespace rosie::rtcore
