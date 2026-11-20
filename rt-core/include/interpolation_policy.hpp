#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

// Admission and sampling share unfused binary64 operations with Go producers.
// Disabling contraction prevents a compiler-selected FMA changing boundary bits.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize ("fp-contract=off")
#endif

namespace rosie::rtcore::interpolation {

// Measured native 1.6 point flags: bit zero declares supplied velocity samples.
inline constexpr uint32_t has_velocity = 1u;
// Exact SI conversion: one second contains 1,000,000,000 nanoseconds.
inline constexpr long double nanoseconds_per_second = 1'000'000'000.L;

struct Checks {
  const char* position;
  const char* velocity;
  const char* acceleration;
  const char* jerk;
};
struct Capabilities {
  const char* interpolation;
  const char* feedforward;
  Checks checks;
  const char* endpoint;
};

// Both supplied knot velocities select cubic Hermite position and its derivative.
// Otherwise position remains linear and feedforward is the segment slope.
// At/after the endpoint hold uses zero velocity; continuous arrival at hold
// requires a zero final supplied velocity. No bridge or retiming is inserted.
// Acceleration is a sampled derivative check, not a continuous acceleration
// guarantee at linear corners or at the transition to endpoint hold. No jerk
// bound is claimed. Prepare checks continuous Hermite position/velocity extrema.
constexpr Capabilities capabilities(bool acceleration_declared) noexcept {
  return {"hermite_position_with_velocity, linear_without", "qd_optional",
      {"declared", "declared", acceleration_declared ? "declared" : "not_declared", "unsupported"},
      "hold_last_sample_then_settle"};
}

enum class Failure { None, InvalidInput, MixedFeedforward, Acceleration, VelocityExtremum, PositionExtremum, RecoveryExtremum };
struct Validation {
  Failure failure = Failure::None;
  uint32_t segment = 0; // Zero-based segment starting at this point index.
  long double value = 0; // Counts/s for velocity, signed counts/s^2 for acceleration.
  double limit = 0; // Count-frame position bound on a PositionExtremum refusal.
  explicit operator bool() const noexcept { return failure == Failure::None; }
};

// Evaluate one Hermite segment in normalized time without per-axis decisions.
// Sharing coefficients with admission prevents bounds on a different polynomial.
template<class Real> struct Hermite {
  Real v0, dt, c2, c3;
  Hermite(Real delta, Real seconds, Real first_velocity, Real second_velocity) noexcept
      : v0(first_velocity), dt(seconds),
        c2(3 * delta - seconds * (2 * first_velocity + second_velocity)),
        c3(-2 * delta + seconds * (first_velocity + second_velocity)) {}
  Real displacement(Real u) const noexcept { return u * (dt * v0 + u * (c2 + u * c3)); }
  Real velocity(Real u) const noexcept { return v0 + u * (2 * c2 + u * 3 * c3) / dt; }
};

struct PositionRange { double min, max; };

// A cubic has at most two interior velocity zeros. Ordered turns split it into
// monotone pieces, preventing an exterior reversal from hiding inside its range.
inline std::array<double, 4> hermite_position_times(const Hermite<double>& curve) noexcept {
  std::array<double, 4> times{0, 1, 1, 1};
  const double a = 3 * curve.c3, b = 2 * curve.c2, c = curve.dt * curve.v0;
  const double discriminant = b * b - (4 * a) * c;
  if (!(curve.dt > 0) || !std::isfinite(curve.dt) || !std::isfinite(a) ||
      !std::isfinite(b) || !std::isfinite(c) || !std::isfinite(discriminant)) {
    times.fill(std::numeric_limits<double>::quiet_NaN());
    return times;
  }
  auto include = [&](double u, size_t slot) { if (u > 0 && u < 1) times[slot] = u; };
  if (a == 0) { if (b != 0) include(-c / b, 1); }
  else if (discriminant >= 0) {
    const double numerator = -.5 * (b + std::copysign(std::sqrt(discriminant), b));
    if (numerator == 0) include(-b / (2 * a), 1);
    else { include(numerator / a, 1); include(c / numerator, 2); }
  }
  std::sort(times.begin(), times.end());
  return times;
}

// Bound the sampler's position at both knots and every interior velocity zero.
// A stable quadratic solution retains the small root when subtraction cancels.
// Nonfinite arithmetic fails closed.
inline PositionRange hermite_position_range(double first, double delta, double dt,
    double v0, double v1) noexcept {
  const Hermite<double> curve(delta, dt, v0, v1);
  const auto invalid = std::numeric_limits<double>::quiet_NaN();
  PositionRange range{std::min(first, first + delta), std::max(first, first + delta)};
  const auto times = hermite_position_times(curve);
  if (!std::isfinite(times[0]) || !std::isfinite(first) || !std::isfinite(delta))
    return {invalid, invalid};
  auto include = [&](double u) {
    if (u > 0 && u < 1) {
      const double q = first + curve.displacement(u);
      if (!std::isfinite(q)) range = {invalid, invalid};
      else { range.min = std::min(range.min, q); range.max = std::max(range.max, q); }
    }
  };
  for (double u : times) include(u);
  if (!std::isfinite(range.min) || !std::isfinite(range.max)) return {invalid, invalid};
  return range;
}

// Validate in the actual CSP command frame, including Home offset and sign in
// the caller's bounds. Checking extrema before commit prevents prepared overshoot.
template<class Point>
Validation validate_position(const Point* points, uint32_t count, uint32_t axis,
    double minimum, double maximum) noexcept {
  if (!points || count < 2 || axis >= points[0].target_counts.size() ||
      !std::isfinite(minimum) || !std::isfinite(maximum) || minimum > maximum)
    return {Failure::InvalidInput};
  for (uint32_t i = 1; i < count; ++i) {
    const auto& first = points[i - 1];
    const auto& second = points[i];
    if (second.t_from_start_ns <= first.t_from_start_ns) return {Failure::InvalidInput, i - 1};
    const double delta = second.target_counts[axis] - first.target_counts[axis];
    const double dt = double(second.t_from_start_ns - first.t_from_start_ns) / 1e9;
    const bool supplied = (first.flags & has_velocity) && (second.flags & has_velocity);
    auto range = hermite_position_range(first.target_counts[axis], delta, dt,
        supplied ? first.velocity_counts_per_s[axis] : delta / dt,
        supplied ? second.velocity_counts_per_s[axis] : delta / dt);
    if (!std::isfinite(range.min) || !std::isfinite(range.max)) return {Failure::InvalidInput, i - 1};
    const double endpoint = second.target_counts[axis];
    if (!std::isfinite(endpoint)) return {Failure::InvalidInput, i - 1};
    range.min = std::min(range.min, endpoint);
    range.max = std::max(range.max, endpoint);
    if (range.min < minimum) return {Failure::PositionExtremum, i - 1, range.min, minimum};
    if (range.max > maximum) return {Failure::PositionExtremum, i - 1, range.max, maximum};
  }
  return {};
}

// The quadratic derivative reaches its absolute maximum at an endpoint or its
// single interior vertex. Reject nonfinite coefficients rather than hiding them
// in a max comparison. Units are counts, seconds and counts/s.
inline double hermite_max_abs_velocity(double delta, double dt,
    double v0, double v1) noexcept {
  const Hermite<double> curve(delta, dt, v0, v1);
  if (!(dt > 0) || !std::isfinite(dt) || !std::isfinite(v0) || !std::isfinite(v1) ||
      !std::isfinite(curve.c2) || !std::isfinite(curve.c3))
    return std::numeric_limits<double>::quiet_NaN();
  auto peak = std::max(std::abs(v0), std::abs(v1));
  if (curve.c3 != 0) {
    const auto u = -curve.c2 / (3 * curve.c3);
    if (u > 0 && u < 1) {
      const auto velocity = curve.velocity(u);
      if (!std::isfinite(velocity)) return std::numeric_limits<double>::quiet_NaN();
      peak = std::max(peak, std::abs(velocity));
    }
  }
  return peak;
}

// Binary64 admission rounds a nonzero peak one representable value outward.
// Strict comparison to the binary64 count limit never adds acceptance slack.
inline double conservative_velocity_peak(double peak) noexcept {
  return peak == 0 ? 0 : std::nextafter(peak, std::numeric_limits<double>::infinity());
}

// Validate continuous segment velocity before publishing an immutable plan.
// Use the sampler's continuous command-frame delta.
template<class Point>
Validation validate_velocity(const Point* points, uint32_t count, uint32_t axis,
    double max_counts_per_s) noexcept {
  if (!points || count < 2 || axis >= points[0].target_counts.size() ||
      !std::isfinite(max_counts_per_s) || max_counts_per_s <= 0)
    return {Failure::InvalidInput, 0, std::numeric_limits<long double>::quiet_NaN()};
  for (uint32_t i = 1; i < count; ++i) {
    const auto& first = points[i - 1];
    const auto& second = points[i];
    if (second.t_from_start_ns <= first.t_from_start_ns)
      return {Failure::InvalidInput, i - 1, std::numeric_limits<long double>::quiet_NaN()};
    if (!(first.flags & has_velocity) || !(second.flags & has_velocity)) continue;
    const auto delta = second.target_counts[axis] - first.target_counts[axis];
    const auto peak = conservative_velocity_peak(hermite_max_abs_velocity(delta,
        double(second.t_from_start_ns - first.t_from_start_ns) / 1e9,
        first.velocity_counts_per_s[axis], second.velocity_counts_per_s[axis]));
    if (!std::isfinite(peak)) return {Failure::InvalidInput, i - 1, peak};
    if (peak > max_counts_per_s) return {Failure::VelocityExtremum, i - 1, peak};
  }
  return {};
}

// Validate one axis of immutable, time-ordered points without allocation.
// With qd, a[i] = (qd[i+1]-qd[i])/dt[i]. Without qd, adjacent segment
// slopes are differenced across their midpoint separation:
// a[i] = 2*(slope[i]-slope[i-1])/(dt[i]+dt[i-1]). A two-point linear
// plan has zero sampled acceleration. Positions use the stored continuous
// command frame, matching Prepare's existing segment-velocity validation.
// Mixed qd presence fails closed when a bound is declared: switching derivative
// sources must not hide an unchecked velocity discontinuity. The caller skips
// this function only for an absent profile bound. Nonfinite arithmetic rejects.
template<class Point>
Validation validate_acceleration(const Point* points, uint32_t count,
    uint32_t axis, long double max_counts_per_s2) noexcept {
  if (!points || count < 2 || axis >= points[0].target_counts.size() ||
      !std::isfinite(max_counts_per_s2) || max_counts_per_s2 <= 0)
    return {Failure::InvalidInput, 0, std::numeric_limits<long double>::quiet_NaN()};
  const bool supplied = (points[0].flags & has_velocity) != 0;
  long double prior_slope = 0, prior_dt = 0;
  for (uint32_t i = 1; i < count; ++i) {
    const auto& first = points[i - 1];
    const auto& second = points[i];
    if (((second.flags & has_velocity) != 0) != supplied)
      return {Failure::MixedFeedforward, i - 1, 0};
    if (second.t_from_start_ns <= first.t_from_start_ns)
      return {Failure::InvalidInput, i - 1, std::numeric_limits<long double>::quiet_NaN()};
    const long double dt = (second.t_from_start_ns - first.t_from_start_ns) / nanoseconds_per_second;
    const long double slope = (static_cast<long double>(second.target_counts[axis]) -
        first.target_counts[axis]) / dt;
    const long double acceleration = supplied
        ? (static_cast<long double>(second.velocity_counts_per_s[axis]) -
            first.velocity_counts_per_s[axis]) / dt
        : i > 1 ? 2 * (slope - prior_slope) / (dt + prior_dt) : 0;
    if (!std::isfinite(slope) || !std::isfinite(acceleration))
      return {Failure::InvalidInput, i - 1, acceleration};
    if (std::abs(acceleration) > max_counts_per_s2)
      return {Failure::Acceleration, i - 1, acceleration};
    prior_slope = slope;
    prior_dt = dt;
  }
  return {};
}
} // namespace rosie::rtcore::interpolation

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif
