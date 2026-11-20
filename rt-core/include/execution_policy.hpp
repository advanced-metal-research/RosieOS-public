#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include "control_policy.hpp"
#include "engine/trajectory.hpp"
#include "engine/jog.hpp"

namespace rosie::rtcore {

// A cancelled source retains its commanded pose and velocity until zero. The
// same integrator and absolute deadline serve Halt, EndJog and input expiry;
// late dispatch cannot increase deceleration or silently extend the arrest.
// With a jog smoother the integrator stops in v/a and the commanded target
// follows one smoothing window later, so the bound includes that window.
struct HaltRamp {
  double acceleration = 0;
  uint64_t cycle_ns = 0, started_ns = 0, bound_ns = 0, last_ns = 0;
  bool advanced = false;

  bool begin(const JogAxis& axis, double limit, uint64_t period, uint64_t now) noexcept {
    const double velocity = axis.integrator_velocity();
    if (!std::isfinite(axis.position) || !std::isfinite(axis.velocity) || !std::isfinite(velocity) ||
        !std::isfinite(limit) || limit <= 0 || !period) return false;
    // SI conversion: 1000000000 ns/s; round the physical v/a bound outward.
    long double ns = std::ceil(std::abs(static_cast<long double>(velocity)) * 1e9L / limit);
    if (axis.smoothed()) ns += static_cast<long double>(axis.smoother->window()) * period;
    if (!std::isfinite(ns) || ns >= static_cast<long double>(UINT64_MAX - period)) return false;
    acceleration = limit; cycle_ns = period; started_ns = now;
    bound_ns = static_cast<uint64_t>(ns) + period;
    last_ns = now; advanced = false;
    return true;
  }
  // lag_s scales the lead bound with the ramp's commanded velocity
  // (JogAxis::advance); 0 keeps the static bound.
  bool advance(JogAxis& axis, double feedback, double lead, uint64_t now,
               double lag_s = 0) noexcept {
    if (!cycle_ns || now < last_ns ||
        (axis.velocity != 0 && now - started_ns > bound_ns)) return false;
    // The admission frame replaces one scheduled command interval. Later
    // frames use elapsed ns so skipped cycles cannot silently stretch v/a.
    double dt = double(advanced ? now - last_ns : cycle_ns) / 1e9;
    if (!(dt > 0)) return false;
    // The step capped at |v|/a is the one that reaches zero. a*(|v|/a) can
    // round below |v|, and the few-ulp velocity it would leave reads as
    // motion once the bound passes, inhibiting an ordinary release.
    const double velocity = axis.integrator_velocity();
    const bool stops = velocity != 0 && dt >= std::abs(velocity) / acceleration;
    if (velocity != 0) dt = std::min(dt, std::abs(velocity) / acceleration);
    last_ns = now; advanced = true;
    if (!axis.advance(0, feedback, acceleration, lead, dt, lag_s, stops)) return false;
    if (stops && !axis.smoothed()) axis.velocity = 0;
    return true;
  }
};

// Shortest jerk-limited reduction of one scalar speed, shared by the path jog
// lane and (follow-up) a trajectory Halt along its path. From speed v0 and
// acceleration a0 it brings the speed to at most v1 with zero acceleration
// using |jerk| <= j_max and |acceleration| <= a_max (an a0 beyond -a_max is
// first eased back to -a_max). It returns the distance and time that profile
// needs; a caller that keeps distance-to-go >= distance can always stop there,
// which is what keeps a stop on the commanded path instead of a per-axis
// ramp that bends it. distance is 0 when no reduction is needed: a0 can be
// eased to zero without exceeding v1. Units are the caller's (path units).
struct JerkLimitedBrake {
  double distance = 0, time = 0;
  bool valid = false;
};

inline JerkLimitedBrake jerk_limited_brake(double v0, double a0, double v1,
                                           double a_max, double j_max) noexcept {
  JerkLimitedBrake out;
  if (!std::isfinite(v0) || !std::isfinite(a0) || !std::isfinite(v1) ||
      !std::isfinite(a_max) || !std::isfinite(j_max) || !(a_max > 0) || !(j_max > 0) ||
      v0 < 0 || v1 < 0) return out;
  out.valid = true;
  double v = v0, a = a0;
  // Constant-jerk segment from (v, a): advances distance, time and state.
  const auto segment = [&](double jerk, double t) {
    if (!(t > 0)) return;
    out.distance += v * t + a * t * t * .5 + jerk * t * t * t / 6;
    out.time += t;
    v += a * t + jerk * t * t * .5;
    a += jerk * t;
  };
  if (a < -a_max) segment(j_max, (-a_max - a) / j_max);
  // Peak speed reached if acceleration were eased to zero right now.
  const double peak = v + (a > 0 ? a * a / (2 * j_max) : 0);
  if (peak <= v1) return out;
  if (a < 0 && v - a * a / (2 * j_max) <= v1) {
    // Easing the deceleration to zero already ends at or below v1.
    segment(j_max, -a / j_max);
    return out;
  }
  // Peak deceleration a_p: jerk -j to -a_p, optional hold, jerk +j to zero.
  double ap = std::sqrt(std::max(0.0, a * a * .5 + j_max * (v - v1)));
  double hold = 0;
  if (ap > a_max) {
    ap = a_max;
    hold = (v - v1 + a * a / (2 * j_max) - ap * ap / j_max) / ap;
  }
  segment(-j_max, (a + ap) / j_max);
  segment(0, hold);
  segment(j_max, ap / j_max);
  return out;
}

// Map the closed logical interval into unrounded command counts. Applying the
// signed scale and Home offset here prevents mixing motor and logical frames.
inline AxisLimits command_limits(AxisLimits limits, double scale, double offset) noexcept {
  limits.valid &= std::isfinite(scale) && scale != 0 && std::isfinite(offset) &&
      limits.min_q <= limits.max_q;
  const double first = limits.min_q * scale - offset;
  const double second = limits.max_q * scale - offset;
  limits.min_q = std::min(first, second);
  limits.max_q = std::max(first, second);
  return limits;
}

inline bool recovery_frame_valid(const AxisLimits& limits, double position) noexcept {
  return limits.valid && std::isfinite(limits.min_q) && std::isfinite(limits.max_q) &&
      limits.min_q <= limits.max_q && std::isfinite(position);
}

inline bool limit_recovery_contains(const AxisLimits& limits, double position) noexcept {
  return recovery_frame_valid(limits, position) && limits.contains(position);
}

// Bound the next velocity by the distance consumed this cycle plus v^2/(2a)
// needed by EndJog/Halt. Inputs are command counts, including signed Home and
// the command-circle lift. Reserve one whole wire count inside each bound so
// rounding cannot emit an exterior target or outward feedforward at the limit.
// Exterior recovery budgets distance to the opposite bound; it never moves the
// position or relaxes the final output/feedback guards. A smoothed axis is
// judged on its integrator: its commanded target is a mean of integrator
// positions and cannot pass a boundary the integrator respects.
inline double jog_position_scale(const AxisLimits& limits, const JogAxis& axis,
    double requested, double acceleration, double dt) noexcept {
  const double position = axis.integrator_position(), velocity = axis.integrator_velocity();
  if (!recovery_frame_valid(limits, position) ||
      !std::isfinite(velocity) || !std::isfinite(requested) ||
      !std::isfinite(acceleration) || !(acceleration > 0) ||
      !std::isfinite(dt) || !(dt > 0)) return 0;
  if (requested == 0) return 1;
  const double direction = requested > 0 ? 1 : -1;
  const double boundary = direction > 0 ? std::floor(limits.max_q) - 1
                                         : std::ceil(limits.min_q) + 1;
  const double distance = std::max(0.0, direction * (boundary - position));
  const double step = acceleration * dt;
  const double discriminant = step * step + 8 * acceleration * distance -
      4 * step * direction * velocity;
  const double speed = std::max(0.0, (std::sqrt(std::max(0.0, discriminant)) - step) * .5);
  return std::min(1.0, speed / std::abs(requested));
}

// Exterior commands may hold or approach the nearest boundary, then continue
// inside the closed interval. No step may enlarge the previous excursion or
// cross the opposite boundary; nonfinite coordinates fail closed.
inline bool limit_recovery_step(const AxisLimits& limits, double previous,
                                double target) noexcept {
  if (!recovery_frame_valid(limits, previous) || !std::isfinite(target)) return false;
  return target >= std::min(previous, limits.min_q) &&
      target <= std::max(previous, limits.max_q);
}

// Reject outward velocity at or beyond either boundary before integration;
// zero remains a hold request and cannot trigger automatic recovery.
inline bool limit_recovery_velocity(const AxisLimits& limits, double position,
                                    double velocity) noexcept {
  if (!recovery_frame_valid(limits, position) || !std::isfinite(velocity)) return false;
  return !(position <= limits.min_q && velocity < 0) &&
      !(position >= limits.max_q && velocity > 0);
}

// Linear feedforward must stay inward for the entire exterior segment, not
// merely at its endpoints. Checking its boundary value prevents a sign change
// before the interpolated position has entered the configured interval.
inline bool limit_recovery_feedforward(const AxisLimits& limits, double first,
    double second, double first_velocity, double second_velocity) noexcept {
  if (!limit_recovery_step(limits, first, second) ||
      !limit_recovery_velocity(limits, first, first_velocity) ||
      !limit_recovery_velocity(limits, second, second_velocity)) return false;
  if (limit_recovery_contains(limits, first) ||
      !limit_recovery_contains(limits, second) || first == second) return true;
  const double boundary = first < limits.min_q ? limits.min_q : limits.max_q;
  const double fraction = (boundary - first) / (second - first);
  return limit_recovery_velocity(limits, boundary,
      first_velocity + fraction * (second_velocity - first_velocity));
}

enum ExecutionFault : uint32_t {
  FaultStartDiscontinuity = 1u << 0,
  FaultPositionRate = 1u << 1,
  FaultCycleDeadline = 1u << 2,
  FaultCycleSleep = 1u << 3,
  // Retained wire bit. A jog or Halt target leading feedback beyond
  // max_target_lead is a warning (TargetLeadEvidence) and never raises it.
  FaultTargetLead = 1u << 4,
  FaultFollowingError = 1u << 5,
  // Measured wire bit 6: a held coordinate cannot be established safely.
  FaultCoordinateReference = 1u << 6,
  FaultDriveReadiness = 1u << 7,
  FaultBusTransport = 1u << 8,
  // Measured wire bit 9: endpoint settling exceeded its configured deadline.
  FaultCompletionTimeout = 1u << 9,
  // Retained wire bit. Brake-hold detections are warnings and never raise it.
  FaultBrakeHold = 1u << 10,
};

// One target-lead excursion on one axis, published by the cycle for the
// non-realtime logger. sequence identifies the excursion; cleared marks the
// cycle the lead returned within its bound, with the excursion's peak.
// limit_counts is the speed-scaled bound and velocity_counts_per_s the
// commanded velocity at the excursion's first step: the measurement input for
// the tracking lag allowance (GitHub #158).
struct TargetLeadEvidence {
  uint64_t sequence = 0, observed_ns = 0, last_ns = 0, cleared_ns = 0;
  double lead_counts = 0, peak_counts = 0, limit_counts = 0;
  uint32_t source = 0;  // 1 jog, 2 Halt/EndJog ramp
  uint32_t cleared = 0;
  double velocity_counts_per_s = 0;
};

// Coordinates are continuous unless the command profile explicitly wraps.
// Feedback wrapping alone must never turn a continuous command into a seam jump.
inline double command_delta(double target, double previous) noexcept {
  return target - previous;
}

inline bool continuous_start(double first, double held) noexcept {
  // One encoder count accounts for quantization of the previously emitted target.
  return std::isfinite(first) && std::isfinite(held) &&
      std::abs(command_delta(first, held)) <= 1.0;
}

// Always choose a future absolute deadline. Missed cycles are skipped, never
// replayed; a caller separately latches a fault when its lateness budget expires.
inline uint64_t future_deadline(uint64_t target, uint64_t now, uint64_t period) noexcept {
  return now < target ? target : target + ((now - target) / period + 1) * period;
}

// Check actual emitted position changes independently of velocity feedforward.
// A single count of bounded credit accommodates integer target quantization.
// Credit is consumed, not granted every cycle, so rounding cannot create a
// sustained speed excess. State belongs exclusively to the cyclic thread.
struct PositionRateGuard {
  double previous = 0;
  double credit = 1;
  uint64_t time_ns = 0;

  void reset(double position, uint64_t now) noexcept {
    previous = position;
    time_ns = now;
    credit = 1;
  }

  bool accept(double position, uint64_t now, double counts_per_second) noexcept {
    if (!std::isfinite(position) || !std::isfinite(counts_per_second) ||
        counts_per_second <= 0 || now <= time_ns) return false;
    const double allowance = counts_per_second * (double(now - time_ns) / 1e9);
    const double distance = std::abs(command_delta(position, previous));
    // Credit before the new step may cover the full elapsed interval, but only
    // one count can survive afterward. Stationary time cannot bank a later jump.
    const double remaining = credit + allowance - distance;
    if (!std::isfinite(distance) || remaining < -1e-7) return false;
    credit = std::clamp(remaining, 0.0, 1.0);
    previous = position;
    time_ns = now;
    return true;
  }
};

} // namespace rosie::rtcore

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("fp-contract=off")
#endif

namespace rosie::rtcore::interpolation {

// Interior Hermite segments retain the strict position-extrema bound. Exterior
// segments may hold or recover, but every monotone piece must obey the point
// step/velocity/feedforward predicates.
// Linear segments retain upload admission; this check prevents cubic overshoot.
template<class Point>
Validation validate_recovery_position(const Point* points, uint32_t count, uint32_t axis,
    double minimum, double maximum) noexcept {
  AxisLimits limits{};
  limits.min_q = minimum; limits.max_q = maximum; limits.valid = true;
  if (!points || count < 2 || axis >= points[0].target_counts.size() ||
      !recovery_frame_valid(limits, minimum)) return {Failure::InvalidInput};
  for (uint32_t i = 1; i < count; ++i) {
    const auto& first = points[i - 1];
    const auto& second = points[i];
    if (second.t_from_start_ns <= first.t_from_start_ns) return {Failure::InvalidInput, i - 1};
    if (!(first.flags & has_velocity) || !(second.flags & has_velocity)) continue;
    const double q0 = first.target_counts[axis], q1 = second.target_counts[axis];
    const double v0 = first.velocity_counts_per_s[axis], v1 = second.velocity_counts_per_s[axis];
    if (!std::isfinite(q0) || !std::isfinite(q1) || !std::isfinite(v0) || !std::isfinite(v1))
      return {Failure::InvalidInput, i - 1};
    if (limit_recovery_contains(limits, q0) &&
        limit_recovery_contains(limits, q1)) {
      auto result = validate_position(points + i - 1, 2, axis, minimum, maximum);
      if (!result) { result.segment = i - 1; return result; }
      continue;
    }
    auto refused = [&](double q) {
      return Validation{Failure::RecoveryExtremum, i - 1, q,
          q < limits.min_q ? std::min(q0, limits.min_q) : std::max(q0, limits.max_q)};
    };
    if (!limit_recovery_feedforward(limits, q0, q1, v0, v1)) return refused(q1);
    const double delta = q1 - q0;
    const double dt = double(second.t_from_start_ns - first.t_from_start_ns) / 1e9;
    const Hermite<double> curve(delta, dt, v0, v1);
    const auto times = hermite_position_times(curve);
    if (!std::isfinite(times[0])) return {Failure::InvalidInput, i - 1};
    double previous = q0;
    for (double u : times) {
      // Include the analytic endpoint as well as the stored hold knot below.
      const double q = u == 0 ? q0 : u == 1 ? q0 + delta : q0 + curve.displacement(u);
      if (!std::isfinite(q)) return {Failure::InvalidInput, i - 1};
      if (!limit_recovery_step(limits, previous, q)) return refused(q);
      previous = q;
    }
    const double endpoint = q1;
    if (!limit_recovery_step(limits, previous, endpoint)) return refused(endpoint);
  }
  return {};
}
} // namespace rosie::rtcore::interpolation

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
