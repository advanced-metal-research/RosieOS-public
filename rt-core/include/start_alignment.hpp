#pragma once

#include "execution_policy.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rosie::rtcore {

// Explicit stationary Start allowance, independent of the Home acquisition
// margin and of runtime encoder disagreement. Completion limits are unchanged.
// floor_counts is the axis profile's compiled start_alignment_floor: a
// drive-profile joint angle converted by this axis's gearing, never a native
// constant.
inline double independent_encoder_start_alignment_allowance(double completion_tolerance,
    double floor_counts, bool independent_source, bool qualified, bool homed,
    bool current_configuration, bool current_coordinate) noexcept {
  if (!std::isfinite(completion_tolerance) || completion_tolerance < 0 ||
      !std::isfinite(floor_counts) || floor_counts < 0 ||
      !independent_source || !qualified || !homed || !current_configuration || !current_coordinate)
    return completion_tolerance;
  return std::max(completion_tolerance, floor_counts);
}

// An ephemeral command prefix, never a change to uploaded points or identity.
// Only stationary starts may use it: the original first feedforward remains
// continuous with the zero-velocity/zero-acceleration endpoint below.
template<std::size_t Capacity, class Axis>
uint64_t start_alignment_duration_ns(const std::array<Axis, Capacity>& axes,
    uint32_t mask, uint64_t cycle_ns) noexcept {
  if (!mask || !cycle_ns) return 0;
  double seconds = 0;
  for (std::size_t i = 0; i < Capacity; ++i) {
    if (!(mask & (1u << i))) continue;
    const auto& axis = axes[i];
    const double distance = std::abs(command_delta(axis.first_target, axis.held_target));
    if (!std::isfinite(distance) || !std::isfinite(axis.completion_tolerance_counts) ||
        axis.completion_tolerance_counts < 0 ||
        (!continuous_start(axis.first_target, axis.held_target) &&
         distance > axis.completion_tolerance_counts) ||
        axis.first_velocity != 0 || !std::isfinite(axis.max_velocity_counts_per_s) ||
        axis.max_velocity_counts_per_s <= 0 || !std::isfinite(axis.max_acceleration_counts_per_s2) ||
        axis.max_acceleration_counts_per_s2 <= 0) return 0;
    // Quintic smoothstep peaks: |v| = 15/8*d/T and |a| = 10/sqrt(3)*d/T^2.
    // Conservative 2 and 6 leave rounding margin without increasing any cap.
    seconds = std::max(seconds, std::max(2 * distance / axis.max_velocity_counts_per_s,
        std::sqrt(6 * distance / axis.max_acceleration_counts_per_s2)));
  }
  const long double cycles = std::max(1.L,
      std::ceil(static_cast<long double>(seconds) * 1e9L / cycle_ns));
  if (!std::isfinite(seconds) || !std::isfinite(cycles) ||
      cycles > static_cast<long double>(std::numeric_limits<uint64_t>::max() / cycle_ns)) return 0;
  return static_cast<uint64_t>(cycles) * cycle_ns;
}

template<std::size_t Capacity> struct StartAlignment {
  bool active = false;
  uint32_t axis_mask = 0;
  uint64_t started_ns = 0, duration_ns = 0;
  std::array<double, Capacity> held{}, first{}, delta{};

  template<class Axis>
  void begin(const std::array<Axis, Capacity>& axes, uint32_t mask,
      uint64_t now, uint64_t duration) noexcept {
    active = duration != 0;
    axis_mask = mask; started_ns = now; duration_ns = duration;
    for (std::size_t i = 0; i < Capacity; ++i) {
      held[i] = axes[i].held_target; first[i] = axes[i].first_target;
      delta[i] = command_delta(first[i], held[i]);
    }
  }

  void cancel() noexcept { active = false; }

  // Called only while the original trajectory still owns execution authority.
  // True means the exact first point was emitted; start its clock THIS cycle,
  // not at the nominal deadline (a late cycle must not skip original samples).
  bool sample(uint64_t now, std::array<double, Capacity>& position,
      std::array<double, Capacity>& velocity) noexcept {
    if (!active) return false;
    const uint64_t elapsed = now >= started_ns ? now - started_ns : 0;
    const bool finished = elapsed >= duration_ns;
    const double u = finished ? 1 : double(elapsed) / double(duration_ns);
    const double blend = u * u * u * (10 + u * (-15 + 6 * u));
    const double derivative = 30 * u * u * (1 - u) * (1 - u) / (double(duration_ns) / 1e9);
    for (std::size_t i = 0; i < Capacity; ++i) {
      if (!(axis_mask & (1u << i))) continue;
      position[i] = finished ? first[i] : held[i] + delta[i] * blend;
      velocity[i] = finished ? 0 : delta[i] * derivative;
    }
    if (finished) active = false;
    return finished;
  }
};

} // namespace rosie::rtcore
