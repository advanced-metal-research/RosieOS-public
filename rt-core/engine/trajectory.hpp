#pragma once
#include "control_protocol.hpp"
#include "interpolation_policy.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace rosie::rtcore {
struct TrajectoryPoint {
  uint64_t t_from_start_ns = 0;
  uint32_t axis_mask = 0, flags = 0;
  std::array<double, protocol::max_axes> target_counts{}, velocity_counts_per_s{};
  uint64_t wait_timeout_ns=0, wait_stable_ns=0;
  uint32_t wait_input=0, wait_value=0, next_wait_index=UINT32_MAX;
  uint32_t io_high = 0;
  uint32_t monitor_mask=0, monitor_values=0;
  uint32_t next_io_index = UINT32_MAX;
};
// LP64 layout includes the stationary input barrier and immutable successor cursors.
static_assert(sizeof(TrajectoryPoint) == 320, "resident plan budget requires a 320-byte point");
// Measured oracle capacity: one active and two prepared slots support transactional replacement.
inline constexpr size_t kResidentPlanSlots = 3;
struct TrajectorySample {
  uint32_t index = 0, axis_mask = 0;
  std::array<double, protocol::max_axes> position{}, velocity{};
};
// Hermite position/derivative or the unchanged linear fallback follow the policy.
// Validated immutable points in continuous command counts, no drive/bus types.
// Work is O(log(samples) + axes), including a delayed or overscheduled cycle.
inline TrajectorySample sample_trajectory(const TrajectoryPoint* points, uint32_t count,
    uint64_t elapsed) noexcept {
  TrajectorySample sample;
  if (!points || !count) return sample;
  const auto* next = std::upper_bound(points, points + count, elapsed,
      [](uint64_t time, const TrajectoryPoint& point) { return time < point.t_from_start_ns; });
  sample.index = next == points ? 0 : static_cast<uint32_t>(next - points - 1);
  const auto& first = points[sample.index];
  sample.axis_mask = first.axis_mask;
  sample.position = first.target_counts;
  if (sample.index + 1 < count) {
    const auto& second = points[sample.index + 1];
    const double duration = double(second.t_from_start_ns - first.t_from_start_ns);
    const double alpha = elapsed <= first.t_from_start_ns ? 0 : double(elapsed - first.t_from_start_ns) / duration;
    // Point flags apply to the whole segment: select once outside the axis loop.
    if ((first.flags & interpolation::has_velocity) && (second.flags & interpolation::has_velocity)) {
      for (size_t axis = 0; axis < protocol::max_axes; ++axis) {
        if (!(sample.axis_mask & (1u << axis))) continue;
        const double delta = second.target_counts[axis] - first.target_counts[axis];
        const interpolation::Hermite<double> curve(delta, duration / 1e9,
            first.velocity_counts_per_s[axis], second.velocity_counts_per_s[axis]);
        sample.position[axis] += curve.displacement(alpha);
        sample.velocity[axis] = curve.velocity(alpha);
      }
    } else for (size_t axis = 0; axis < protocol::max_axes; ++axis) {
      if (!(sample.axis_mask & (1u << axis))) continue;
      const double delta = second.target_counts[axis] - first.target_counts[axis];
      sample.position[axis] += alpha * delta;
      sample.velocity[axis] = delta * 1e9 / duration;
    }
  } // At/after the final point, position holds and velocity is zero.
  return sample;
}
}
