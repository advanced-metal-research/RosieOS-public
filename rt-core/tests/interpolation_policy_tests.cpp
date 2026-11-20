#include "engine/trajectory.hpp"
#include "execution_policy.hpp"
#include "clients/cpp/include/rosie/rt_control_api_generated.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <limits>

using namespace rosie::rtcore;

// Compare binary64 command conversion with extended count arithmetic. The
// absolute count bounds prevent exact logical equality becoming assumed parity.
static void test_exact_bound_count_divergence() {
  long double product_error = 0, shifted_error = 0, acceleration_error = 0;
  unsigned acceleration_refusals = 0;
  for (double scale : {131072 / (2 * M_PI), 131072 * 50 / (2 * M_PI),
                       131072 * 100 / (2 * M_PI), 2086254.}) {
    for (double q : {-M_PI, -0.5, -0.1, 0.1, 0.5, M_PI}) {
      const double rounded = q * scale;
      const long double extended = static_cast<long double>(q) * scale;
      product_error = std::max(product_error, std::abs(rounded - extended));
      assert(std::abs(rounded - extended) < 0.000000001L);
      for (double home : {0., -2147483648., 2147483647.}) {
        const double shifted = rounded - home;
        shifted_error = std::max(shifted_error, std::abs(shifted - (extended - home)));
        assert(std::abs(shifted - (extended - home)) < 0.000001L);
      }
    }
    for (double limit : {0.01, 0.1, 0.2, 0.3}) {
      TrajectoryPoint points[2]{};
      points[0].flags = points[1].flags = interpolation::has_velocity;
      points[1].t_from_start_ns = 1'000'000'000;
      points[1].velocity_counts_per_s[0] = limit * scale;
      const long double cap = static_cast<long double>(limit) * scale;
      const auto result = interpolation::validate_acceleration(points, 2, 0, cap);
      if (!result) {
        ++acceleration_refusals;
        assert(result.failure == interpolation::Failure::Acceleration);
        acceleration_error = std::max(acceleration_error, result.value - cap);
        assert(result.value - cap < 0.000001L);
      }
      points[1].velocity_counts_per_s[0] = (limit - 1 / scale) * scale;
      assert(interpolation::validate_acceleration(points, 2, 0, cap));
    }
  }
  assert(acceleration_refusals > 0);
  std::printf("PASS test_exact_bound_count_divergence product_counts=%.17Lg shifted_counts=%.17Lg acceleration_counts_per_s2=%.17Lg exact_refusals=%u\n",
      product_error, shifted_error, acceleration_error, acceleration_refusals);
}

// The documented optional-qd contract selects one position/derivative pair;
// an independently interpolated feedforward cannot satisfy these count values.
static void test_documented_optional_velocity_contract() {
  TrajectoryPoint points[2]{};
  for (auto& point : points) { point.axis_mask = 1; point.flags = interpolation::has_velocity; }
  points[0].target_counts[0] = 100;
  points[1].target_counts[0] = 300;
  points[1].t_from_start_ns = 1'000'000'000;
  auto sample = sample_trajectory(points, 2, 250'000'000);
  assert(sample.position[0] == 131.25 && sample.velocity[0] == 225);
  sample = sample_trajectory(points, 2, 1'000'000'000);
  assert(sample.position[0] == 300 && sample.velocity[0] == 0);
  for (uint32_t missing : {0u, 1u}) {
    points[missing].flags = 0;
    sample = sample_trajectory(points, 2, 250'000'000);
    assert(sample.position[0] == 150 && sample.velocity[0] == 200);
    points[missing].flags = interpolation::has_velocity;
  }
  // A tiny negative knot stays in the same continuous count frame.
  points[0].target_counts[0] = 0;
  points[1].target_counts[0] = -1e-12;
  sample = sample_trajectory(points, 2, 1'000'000'000);
  assert(sample.position[0] == -1e-12);
  assert(interpolation::validate_position(points, 2, 0, -10000, 10000));
}

// Measured review fixture: 0.49 + 0.05*u*(1-u) reaches 0.5025 rad.
// Signed/Home-shifted commands must check the sampler's own frame.
static void test_hermite_position_extrema() {
  TrajectoryPoint points[2]{};
  for (auto& point : points) { point.axis_mask = 1; point.flags = interpolation::has_velocity; }
  points[1].t_from_start_ns = 1'000'000'000;
  for (double sign : {-1., 1.}) {
    for (double offset : {0., 123.}) {
      points[0].target_counts[0] = points[1].target_counts[0] = .49 * 4096 * sign - offset;
      points[0].velocity_counts_per_s[0] = .05 * 4096 * sign;
      points[1].velocity_counts_per_s[0] = -.05 * 4096 * sign;
      auto result = interpolation::validate_position(points, 2, 0, -.5 * 4096 - offset, .5 * 4096 - offset);
      assert(result.failure == interpolation::Failure::PositionExtremum && result.segment == 0);
      assert(std::abs((result.value + offset) / (4096 * sign) - .5025) < 1e-12);
      assert(interpolation::validate_position(points, 2, 0, -.51 * 4096 - offset, .51 * 4096 - offset));
    }
  }
  points[0].target_counts[0] = points[1].target_counts[0] = .49 * 4096;
  points[0].velocity_counts_per_s[0] = .05 * 4096;
  points[1].velocity_counts_per_s[0] = -.05 * 4096;
  assert(!interpolation::validate_position(points, 2, 0, -.5 * 4096, .5 * 4096));
  // Both quadratic roots are interior: q=10u-30u^2+20u^3 counts.
  const auto range = interpolation::hermite_position_range(0, 0, 1, 10, 10);
  assert(std::abs(range.min + .9622504486493763) < 1e-12);
  assert(std::abs(range.max - .9622504486493763) < 1e-12);
}

// These independent endpoint and dense-sweep fixtures prevent a sampler and
// admission polynomial from sharing the same undetected coefficient error.
static void test_hermite_continuity_and_extremum() {
  TrajectoryPoint points[3]{};
  for (uint32_t i = 0; i < 3; ++i) {
    points[i].axis_mask = 1;
    points[i].flags = interpolation::has_velocity;
    points[i].t_from_start_ns = i * 1'000'000'000ULL;
    points[i].target_counts[0] = 20 * i;
  }
  points[0].velocity_counts_per_s[0] = 10;
  points[1].velocity_counts_per_s[0] = 10;
  auto sample = sample_trajectory(points, 2, 0);
  assert(sample.position[0] == 0 && sample.velocity[0] == 10);
  sample = sample_trajectory(points, 2, 999'999'999);
  assert(std::abs(sample.position[0] - 20) < 0.00000002);
  assert(std::abs(sample.velocity[0] - 10) < 0.0000001);
  sample = sample_trajectory(points, 3, 1'000'000'000);
  assert(sample.position[0] == 20 && sample.velocity[0] == 10);
  sample = sample_trajectory(points, 3, 1'000'000'001);
  assert(std::abs(sample.position[0] - 20) < 0.00000002);
  assert(std::abs(sample.velocity[0] - 10) < 0.0000001);
  // A zero final knot velocity joins endpoint hold continuously.
  sample = sample_trajectory(points + 1, 2, 2'000'000'000);
  assert(sample.position[0] == 40 && sample.velocity[0] == 0);
  sample = sample_trajectory(points + 1, 2, 1'999'999'999);
  assert(std::abs(sample.position[0] - 40) < 0.00000002);
  assert(std::abs(sample.velocity[0]) < 0.0000002);

  // q(u)=10u+30u^2-20u^3 counts, v(u)=10+60u-60u^2 counts/s.
  // Its exact peak is 25 counts/s at u=0.5, included in the numeric sweep.
  long double numeric_peak = 0;
  for (uint64_t ns = 0; ns < 1'000'000'000; ns += 1000) {
    sample = sample_trajectory(points, 2, ns);
    numeric_peak = std::max(numeric_peak, static_cast<long double>(std::abs(sample.velocity[0])));
  }
  const auto analytic = interpolation::hermite_max_abs_velocity(20, 1, 10, 10);
  assert(std::abs(analytic - 25) < 1e-9L);
  assert(std::abs(analytic - numeric_peak) < 1e-9L);
  assert(!interpolation::validate_velocity(points, 2, 0, 25));
  assert(interpolation::validate_velocity(points, 2, 0, std::nextafter(25., INFINITY)));
  const auto rejected = interpolation::validate_velocity(points, 2, 0, 24);
  assert(rejected.failure == interpolation::Failure::VelocityExtremum);
  assert(rejected.segment == 0 && rejected.value == std::nextafter(25., INFINITY));
  assert(interpolation::hermite_max_abs_velocity(-20, 1, -10, -10) == 25);
  assert(interpolation::hermite_max_abs_velocity(15, 1, 10, 20) == 20);
  assert(interpolation::hermite_max_abs_velocity(20, 1, 20, 20) == 20);
  assert(interpolation::hermite_max_abs_velocity(12, 1, 20, 0) == 20);
  assert(!interpolation::validate_velocity(points, 2, 0, 0));
  points[1].velocity_counts_per_s[0] = std::numeric_limits<double>::infinity();
  assert(!interpolation::validate_velocity(points, 2, 0, 25));
  points[1].velocity_counts_per_s[0] = 10;
  points[1].t_from_start_ns = 0;
  assert(!interpolation::validate_velocity(points, 2, 0, 25));
  points[1].t_from_start_ns = 1'000'000'000;
  points[0].target_counts[0] = 990;
  points[1].target_counts[0] = 2010;
  sample = sample_trajectory(points, 2, 500'000'000);
  assert(sample.position[0] == 1500 && sample.velocity[0] == 1525);
  assert(interpolation::validate_velocity(points, 2, 0, std::nextafter(1525., INFINITY)));
  assert(!interpolation::validate_velocity(points, 2, 0, 1525));

  // Asymmetric 3 s segment: v(u)=30+20u-40u^2 counts/s, peak 32.5
  // at u=0.25. This prevents treating the midpoint as every curve's extremum.
  points[0].target_counts[0] = 0;
  points[1].target_counts[0] = 80;
  points[0].velocity_counts_per_s[0] = 30;
  points[1].velocity_counts_per_s[0] = 10;
  points[1].t_from_start_ns = 3'000'000'000;
  numeric_peak = 0;
  for (uint64_t ns = 0; ns < 3'000'000'000; ns += 3000) {
    sample = sample_trajectory(points, 2, ns);
    numeric_peak = std::max(numeric_peak, static_cast<long double>(std::abs(sample.velocity[0])));
  }
  assert(std::abs(numeric_peak - 32.5L) < 1e-9L);
  assert(std::abs(interpolation::hermite_max_abs_velocity(80, 3, 30, 10) - numeric_peak) < 1e-9L);
  assert(interpolation::validate_velocity(points, 2, 0, std::nextafter(32.5, INFINITY)));
  assert(!interpolation::validate_velocity(points, 2, 0, 32));
  points[0].target_counts[0] = 0;
  points[1].target_counts[0] = 500;
  points[0].velocity_counts_per_s[0] = points[1].velocity_counts_per_s[0] = 0;
  points[1].t_from_start_ns = 1'000'000'000;
  sample = sample_trajectory(points, 2, 500'000'000);
  assert(sample.position[0] == 250 && sample.velocity[0] == 750);
  assert(interpolation::validate_velocity(points, 2, 0, std::nextafter(750., INFINITY)));
}

// Measured review boundary: 131072 counts/revolution, 9 microradians in 10 ms.
// Both languages must refuse the lower cap and admit the upper binary64 cap.
static void test_count_frame_velocity_boundary() {
  const double scale = 131072 / (2 * std::acos(-1.));
  TrajectoryPoint points[2]{};
  for (auto& point : points) { point.axis_mask = 1; point.flags = interpolation::has_velocity; }
  points[1].t_from_start_ns = 10'000'000;
  for (double sign : {-1., 1.}) {
    points[1].target_counts[0] = .000009 * (scale * sign);
    assert(!interpolation::validate_velocity(points, 2, 0, .0013499999999999999 * scale));
    assert(interpolation::validate_velocity(points, 2, 0, .001350000000000001 * scale));
  }
  points[0].target_counts[0] = 0;
  points[1].target_counts[0] = 20;
  points[1].t_from_start_ns = 1'000'000'000;
  assert(!interpolation::validate_velocity(points, 2, 0, std::nextafter(30., 0.)));
  assert(!interpolation::validate_velocity(points, 2, 0, 30));
  assert(interpolation::validate_velocity(points, 2, 0, std::nextafter(30., INFINITY)));
}

// Preserve the arithmetic byte for byte for missing/mixed qd, including
// ignored nonfinite qd, masked axes and irregular clocks.
static void test_linear_bytes_without_velocity() {
  TrajectoryPoint points[2]{};
  points[0].axis_mask = points[1].axis_mask = 3;
  points[0].t_from_start_ns = 100;
  points[1].t_from_start_ns = 10'000'103;
  points[0].target_counts[0] = 990;
  points[1].target_counts[0] = 10;
  points[0].target_counts[1] = -123.456;
  points[1].target_counts[1] = 42.789;
  for (auto& point : points) point.velocity_counts_per_s.fill(std::numeric_limits<double>::quiet_NaN());
  for (uint32_t flags : {0u, 1u, 2u}) {
    points[0].flags = flags == 1 ? interpolation::has_velocity : 0;
    points[1].flags = flags == 2 ? interpolation::has_velocity : 0;
    for (uint64_t ns : {0ULL, 100ULL, 101ULL, 5'000'000ULL, 10'000'102ULL, 10'000'103ULL, 20'000'000ULL}) {
      TrajectorySample expected;
      expected.index = ns < points[1].t_from_start_ns ? 0 : 1;
      expected.axis_mask = 3;
      expected.position = points[expected.index].target_counts;
      if (expected.index == 0) {
        const double duration = double(points[1].t_from_start_ns - points[0].t_from_start_ns);
        const double alpha = ns <= points[0].t_from_start_ns ? 0 : double(ns - points[0].t_from_start_ns) / duration;
        for (size_t axis = 0; axis < 2; ++axis) {
          const double delta = points[1].target_counts[axis] - points[0].target_counts[axis];
          expected.position[axis] += alpha * delta;
          expected.velocity[axis] = delta * 1e9 / duration;
        }
      }
      const auto actual = sample_trajectory(points, 2, ns);
      assert(actual.index == expected.index && actual.axis_mask == expected.axis_mask);
      assert(std::memcmp(actual.position.data(), expected.position.data(), sizeof(actual.position)) == 0);
      assert(std::memcmp(actual.velocity.data(), expected.velocity.data(), sizeof(actual.velocity)) == 0);
    }
  }
}

// Exact count fixtures distinguish interior bounds, exterior envelope overshoot,
// and outward reversals that never exceed the endpoints' overall envelope.
static void test_hermite_recovery_position() {
  using interpolation::Failure;
  struct Case { double q0, q1, v0, v1, minimum, maximum; Failure failure; };
  for (const auto& fixture : {
      Case{49, 49, 5, -5, -50, 50, Failure::PositionExtremum},
      Case{0, 1, 0, 10, 10, 20, Failure::RecoveryExtremum},
      Case{0, 2, 8, 8, 10, 20, Failure::RecoveryExtremum},
      Case{0, 15, 0, 0, 10, 20, Failure::None},
      Case{0, 5, 0, 0, 10, 20, Failure::None},
      Case{0, 0, 0, 0, 10, 20, Failure::None},
      Case{0, 15, 40, -5, 10, 20, Failure::None},
      Case{0, 15, 120, 0, 10, 20, Failure::RecoveryExtremum},
      Case{0, 11, 1, -1, 10, 20, Failure::RecoveryExtremum}}) {
    for (double sign : {-1., 1.}) for (double home : {0., 123.}) {
      TrajectoryPoint points[2]{};
      points[0].flags = points[1].flags = interpolation::has_velocity;
      points[1].t_from_start_ns = 1'000'000'000;
      points[0].target_counts[0] = fixture.q0 * sign - home;
      points[1].target_counts[0] = fixture.q1 * sign - home;
      points[0].velocity_counts_per_s[0] = fixture.v0 * sign;
      points[1].velocity_counts_per_s[0] = fixture.v1 * sign;
      const double lower = std::min(fixture.minimum * sign, fixture.maximum * sign) - home;
      const double upper = std::max(fixture.minimum * sign, fixture.maximum * sign) - home;
      auto result = interpolation::validate_recovery_position(points, 2, 0, lower, upper);
      if (result.failure != fixture.failure)
        std::fprintf(stderr, "recovery q0=%g q1=%g v0=%g v1=%g sign=%g home=%g: failure=%d expected=%d\n",
            fixture.q0, fixture.q1, fixture.v0, fixture.v1, sign, home,
            int(result.failure), int(fixture.failure));
      assert(result.failure == fixture.failure && result.segment == 0);
      // No-velocity recovery remains the responsibility of point admission.
      points[0].flags = points[1].flags = 0;
      assert(interpolation::validate_recovery_position(points, 2, 0, lower, upper));
    }
  }
  // Independent exact turn: q(u)=2u-8u^2+8u^3 has q(1/2)=0 counts.
  // It retreats from q(1/6)=4/27 while staying inside the endpoint envelope.
  const interpolation::Hermite<double> curve(2, 1, 2, 10);
  const auto turns = interpolation::hermite_position_times(curve);
  assert(std::abs(turns[1] - 1./6) < 1e-15 && turns[2] == .5);
  assert(std::abs(curve.displacement(turns[1]) - 4./27) < 1e-15);
  assert(curve.displacement(turns[2]) == 0);
}

int main() {
  test_exact_bound_count_divergence();
  test_hermite_recovery_position();
  test_documented_optional_velocity_contract();
  test_count_frame_velocity_boundary();
  test_hermite_position_extrema();
  test_hermite_continuity_and_extremum();
  test_linear_bytes_without_velocity();
  TrajectoryPoint points[3]{};
  for (uint32_t i = 0; i < 3; ++i) {
    points[i].t_from_start_ns = uint64_t(i) * 1'000'000'000;
    points[i].axis_mask = 1;
    points[i].flags = interpolation::has_velocity;
  }
  points[0].target_counts[0] = 100;
  points[1].target_counts[0] = 300;
  points[0].velocity_counts_per_s[0] = 20;
  points[1].velocity_counts_per_s[0] = 60;
  auto sample = sample_trajectory(points, 2, 500'000'000);
  // P4-14 changes this supplied-qd sampler assertion from linear (200, 40)
  // to Hermite (195, 280). The compiler oracle from 07ca5c14, documented by
  // 72a745f0, contains no trajectory samples and remains byte-identical.
  assert(sample.position[0] == 195);
  assert(sample.velocity[0] == 280);
  assert(sample.index == 0 && sample.axis_mask == 1);
  sample = sample_trajectory(points, 2, 0);
  assert(sample.position[0] == 100 && sample.velocity[0] == 20);
  for (uint64_t time : {1'000'000'000ULL, 2'000'000'000ULL}) {
    sample = sample_trajectory(points, 2, time);
    assert(sample.position[0] == 300 && sample.velocity[0] == 0);
    assert(sample.index == 1);
  }
  assert(interpolation::validate_acceleration(points, 2, 0, 40));
  points[1].velocity_counts_per_s[0] = 61;
  auto check = interpolation::validate_acceleration(points, 2, 0, 40);
  assert(check.failure == interpolation::Failure::Acceleration);
  assert(check.segment == 0 && check.value == 41);
  points[1].velocity_counts_per_s[0] = -21;
  check = interpolation::validate_acceleration(points, 2, 0, 40);
  assert(check.failure == interpolation::Failure::Acceleration && check.value == -41);

  for (auto& point : points) point.flags = 0;
  points[0].target_counts[0] = 0;
  points[1].target_counts[0] = 20;
  points[2].target_counts[0] = 80;
  assert(interpolation::validate_acceleration(points, 3, 0, 40));
  sample = sample_trajectory(points, 2, 500'000'000);
  assert(sample.position[0] == 10 && sample.velocity[0] == 20);
  points[2].target_counts[0] = 81;
  check = interpolation::validate_acceleration(points, 3, 0, 40);
  assert(check.failure == interpolation::Failure::Acceleration);
  assert(check.segment == 1 && check.value == 41);
  assert(interpolation::validate_acceleration(points, 2, 0, 1));

  // Nonuniform intervals: slopes 20 and 80 counts/s, separated by 1.5 s.
  points[2].t_from_start_ns = 3'000'000'000;
  points[2].target_counts[0] = 180;
  assert(interpolation::validate_acceleration(points, 3, 0, 40));
  points[2].target_counts[0] = 183;
  check = interpolation::validate_acceleration(points, 3, 0, 40);
  assert(check.failure == interpolation::Failure::Acceleration && check.value == 41);
  points[1].flags = interpolation::has_velocity;
  check = interpolation::validate_acceleration(points, 3, 0, 40);
  assert(check.failure == interpolation::Failure::MixedFeedforward);
  points[1].flags = 0;
  assert(!interpolation::validate_acceleration(points, 3, 0, 0));
  assert(!interpolation::validate_acceleration(points, 3, 0, std::numeric_limits<double>::infinity()));
  points[1].t_from_start_ns = 0;
  assert(!interpolation::validate_acceleration(points, 3, 0, 40));
  points[1].t_from_start_ns = 1'000'000'000;
  points[2].target_counts[0] = std::numeric_limits<double>::infinity();
  assert(!interpolation::validate_acceleration(points, 3, 0, 40));

  const auto policy = interpolation::capabilities(true);
  assert(std::strcmp(policy.interpolation, "hermite_position_with_velocity, linear_without") == 0);
  assert(std::strcmp(policy.feedforward, "qd_optional") == 0);
  assert(std::strcmp(policy.endpoint, "hold_last_sample_then_settle") == 0);
  assert(std::strcmp(policy.checks.position, "declared") == 0);
  assert(std::strcmp(policy.checks.velocity, "declared") == 0);
  assert(std::strcmp(policy.checks.acceleration, "declared") == 0);
  assert(std::strcmp(policy.checks.jerk, "unsupported") == 0);
  assert(std::strcmp(interpolation::capabilities(false).checks.acceleration, "not_declared") == 0);

  rosie::rt_control::AxisDescription description{};
  description.Interpolation = policy.interpolation;
  description.Feedforward = policy.feedforward;
  description.Endpoint = policy.endpoint;
  description.Checks = rosie::rt_control::Parser(
      R"({"position":"declared","velocity":"declared","acceleration":"declared","jerk":"unsupported"})").parse();
  description.MaxAcceleration = 40;
  rosie::rt_control::AxisDescription decoded{};
  assert(rosie::rt_control::decode(rosie::rt_control::encode(description), decoded));
  assert(decoded.Interpolation == "hermite_position_with_velocity, linear_without" && decoded.Feedforward == "qd_optional");
  assert(decoded.Endpoint == "hold_last_sample_then_settle");
  assert(rosie::rt_control::dump(decoded.Checks) == rosie::rt_control::dump(description.Checks));
  assert(decoded.MaxAcceleration == 40);
  description.MaxAcceleration.reset();
  description.Checks = rosie::rt_control::Parser(
      R"({"position":"declared","velocity":"declared","acceleration":"not_declared","jerk":"unsupported"})").parse();
  assert(rosie::rt_control::decode(rosie::rt_control::encode(description), decoded));
  assert(!decoded.MaxAcceleration);
  assert(rosie::rt_control::dump(decoded.Checks) == rosie::rt_control::dump(description.Checks));
}
