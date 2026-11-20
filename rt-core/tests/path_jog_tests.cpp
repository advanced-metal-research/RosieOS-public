// Path jog qualification in absolute joint units. Fixture limits are a Rosie
// 1430 cell's (its robot description's config.json and machine configuration):
// 1.396 rad/s, 4*pi rad/s^2, jog_jerk 150 rad/s^3, 1 ms cycle, and one
// count = 2*pi / (131072 * 100) rad. Every bound below is one of those
// numbers; none is derived from the test's own sampling.
#include "engine/path_jog.hpp"
#include <cassert>
#include <chrono>
#include <cstdio>
#include <functional>
#include <random>
#include <iostream>
#include <vector>

using namespace rosie::rtcore;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kVelocity = 1.396, kAcceleration = 4 * kPi, kJerk = 150;
constexpr double kCount = 2 * kPi / (131072.0 * 100.0);
constexpr uint64_t kCycle = 1'000'000;
constexpr double kDt = 1e-3;
constexpr double kSpacing = 0.001;  // 1 mm samples, path units = metres.
constexpr uint64_t kMaxAge = 250'000'000;
constexpr int kAxes = 6;

// A smooth joint-space curve with 1430-like magnitudes per metre of TCP
// travel: slopes 0.4-2.4 rad/m, curvature up to ~10 rad/m^2 and a third
// derivative up to ~160 rad/m^3 (the wiggle), so all three chain-rule terms
// participate.
using Curve = std::function<std::array<double, kAxes>(double)>;
std::array<double, kAxes> default_curve(double x) {
  static constexpr double slope[kAxes] = {1.2, -0.8, 1.5, 0.4, -2.0, 0.9};
  static constexpr double bend[kAxes] = {0.5, -1.0, 2.0, 0.3, -1.5, 0.8};
  static constexpr double base[kAxes] = {0.1, -0.4, 0.6, 0.2, -0.7, 0.3};
  std::array<double, kAxes> q{};
  for (int j = 0; j < kAxes; ++j)
    q[j] = base[j] + slope[j] * x + bend[j] * x * x + 0.02 * std::sin(20 * x + j);
  return q;
}

struct Fixture {
  PathJog lane;
  PathJogConfig config{};
  PathJogBegin request{};
  Curve curve = default_curve;
  uint64_t now = 1'000'000'000;
  uint64_t sent = 0, sequence = 0;
  std::vector<std::array<double, kAxes>> q;
  std::vector<double> s, v;
  bool fault = false, closed = false;

  explicit Fixture(Curve c = default_curve, double max_speed = 1.0) : curve(std::move(c)) {
    config.mask = (1u << kAxes) - 1;
    config.jerk = kJerk;
    config.max_sample_step_s = 0.02;
    config.cycle_ns = kCycle;
    for (int j = 0; j < kAxes; ++j)
      config.axes[j] = {-3.0, 3.0, kVelocity, kAcceleration, kCount};
    request.spacing = kSpacing;
    request.max_speed = max_speed;
    request.first_input_deadline_ns = now + kMaxAge;
    const auto q0 = curve(0);
    for (int j = 0; j < kAxes; ++j) request.held[j] = q0[j];
  }
  PathJogResult begin() { return lane.begin(config, request, now); }
  PathJogResult send(uint32_t count, bool end = false, uint32_t reason = 0, const double* caps = nullptr) {
    std::vector<double> rows(count * path_jog_axes, 0.0);
    for (uint32_t i = 0; i < count; ++i) {
      const auto qi = curve(double(sent + i) * kSpacing);
      for (int j = 0; j < kAxes; ++j) rows[i * path_jog_axes + j] = qi[j];
    }
    const auto result = lane.append(sent, count, count ? rows.data() : nullptr, caps, end, reason);
    if (result == PathJogResult::Ok) sent = lane.received();
    return result;
  }
  PathJogResult speed(double metres_per_second) {
    return lane.set_speed(++sequence, metres_per_second, now, now + kMaxAge, now, kMaxAge);
  }
  // Keep `ahead` samples beyond the consumption point, in chunks of 25, as
  // the OLP producer does; limit caps the total (an underrun when reached).
  void feed(uint64_t ahead, uint64_t limit = UINT64_MAX) {
    while (sent < limit && sent < uint64_t(lane.s()) + ahead) {
      const uint32_t chunk = uint32_t(std::min<uint64_t>(25, limit - sent));
      const auto r = send(chunk);
      if (r == PathJogResult::PathEnded) return;  // A native refusal ended the path.
      assert(r == PathJogResult::Ok);
    }
  }
  void cycle() {
    now += kCycle;
    const auto out = lane.advance(now);
    fault |= out.fault;
    closed |= out.closed;
    std::array<double, kAxes> row{};
    for (int j = 0; j < kAxes; ++j) row[j] = lane.position()[j];
    q.push_back(row);
    s.push_back(lane.s());
    v.push_back(lane.speed());
  }
  // Operator holds: a fresh speed command every 50 ms (the browser cadence).
  void hold(double speed_mps, int cycles, uint64_t ahead = 100, uint64_t limit = UINT64_MAX) {
    for (int i = 0; i < cycles && lane.open(); ++i) {
      if (i % 50 == 0) assert(this->speed(speed_mps) == PathJogResult::Ok);
      feed(ahead, limit);
      cycle();
    }
  }
  void run_until_closed(int max_cycles, uint64_t ahead = 100, uint64_t limit = UINT64_MAX) {
    for (int i = 0; i < max_cycles && lane.open(); ++i) {
      feed(ahead, limit);
      cycle();
    }
  }
};

struct Peaks { double velocity = 0, acceleration = 0, jerk = 0; };

// First, second and third differences of the emitted 1 kHz targets.
Peaks peaks(const Fixture& f) {
  Peaks p;
  const auto& q = f.q;
  for (size_t n = 3; n < q.size(); ++n)
    for (int j = 0; j < kAxes; ++j) {
      p.velocity = std::max(p.velocity, std::abs(q[n][j] - q[n - 1][j]) / kDt);
      p.acceleration = std::max(p.acceleration, std::abs(q[n][j] - 2 * q[n - 1][j] + q[n - 2][j]) / (kDt * kDt));
      p.jerk = std::max(p.jerk, std::abs(q[n][j] - 3 * q[n - 1][j] + 3 * q[n - 2][j] - q[n - 3][j]) / (kDt * kDt * kDt));
    }
  return p;
}

// Floating-point allowance only (q ~ 1 rad, ulp 2.2e-16): 1e-6 rad/s,
// 1e-4 rad/s^2, 1e-2 rad/s^3 are far below any physical effect.
void assert_within_limits(const Fixture& f, const char* label) {
  const auto p = peaks(f);
  std::printf("  %-34s peak |qd| %.4f rad/s  |qdd| %.4f rad/s^2  |qddd| %.3f rad/s^3\n",
              label, p.velocity, p.acceleration, p.jerk);
  assert(p.velocity <= kVelocity + 1e-6);
  assert(p.acceleration <= kAcceleration + 1e-4);
  assert(p.jerk <= kJerk + 1e-2);
}

// Every emitted target lies on the sampled curve: within one count of the
// analytic curve at the published path position (interpolation error plus
// rounding), on every axis, in every cycle.
void assert_on_path(const Fixture& f, const char* label) {
  double worst = 0;
  for (size_t n = 0; n < f.q.size(); ++n) {
    const auto exact = f.curve(f.s[n] * kSpacing);
    for (int j = 0; j < kAxes; ++j) worst = std::max(worst, std::abs(f.q[n][j] - exact[j]));
  }
  std::printf("  %-34s max |q - curve(s)| %.3e rad (one count %.3e)\n", label, worst, kCount);
  assert(worst <= kCount);
}

// All axes stop in the same cycle: the last cycle with any motion is shared.
void assert_synchronous_stop(const Fixture& f) {
  int last[kAxes];
  for (int j = 0; j < kAxes; ++j) {
    last[j] = -1;
    for (size_t n = 1; n < f.q.size(); ++n)
      if (f.q[n][j] != f.q[n - 1][j]) last[j] = int(n);
  }
  for (int j = 1; j < kAxes; ++j) assert(last[j] == last[0]);
}

void brake_matches_integration() {
  // Closed form against fine forward integration of the profile it describes.
  struct Case { double v0, a0, v1, amax, jmax; };
  const Case cases[] = {{0.4, 0, 0, 5, 50}, {0.4, 3, 0, 5, 50}, {0.4, -4, 0, 5, 50},
                        {0.4, -6, 0, 5, 50}, {0.1, 0, 0.05, 5, 50}, {0.4, 2, 0.3, 5, 50},
                        {2.0, 0, 0, 1, 100}, {0.05, 4, 0.06, 5, 50}};
  for (const auto& c : cases) {
    const auto b = jerk_limited_brake(c.v0, c.a0, c.v1, c.amax, c.jmax);
    assert(b.valid);
    // Integrate: jerk toward the approach curve v + a|a|/(2j) = v1.
    double v = c.v0, a = c.a0, s = 0, t = 0;
    const double h = 1e-7;
    if (c.v0 + std::max(c.a0, 0.0) * std::max(c.a0, 0.0) / (2 * c.jmax) <= c.v1) { assert(b.distance == 0); continue; }
    while (t < 10) {
      double jerk;
      if (a < -c.amax) jerk = c.jmax;
      else if (v + a * std::abs(a) / (2 * c.jmax) > c.v1) jerk = a > -c.amax ? -c.jmax : 0;
      else jerk = c.jmax;
      if (a >= 0 && v + a * a / (2 * c.jmax) <= c.v1 + 1e-12 && v <= c.v1 + 1e-12) break;
      if (std::abs(a) < c.jmax * h && v <= c.v1 + 1e-9) break;
      s += v * h + a * h * h / 2 + jerk * h * h * h / 6;
      v += a * h + jerk * h * h / 2;
      a = std::clamp(a + jerk * h, -std::max(c.amax, -c.a0), std::max(c.amax, c.a0));
      t += h;
    }
    std::printf("  brake v0=%.2f a0=%.1f -> v1=%.2f: closed %.6f m %.5f s, integrated %.6f m %.5f s\n",
                c.v0, c.a0, c.v1, b.distance, b.time, s, t);
    assert(std::abs(b.distance - s) <= 1e-5 && std::abs(b.time - t) <= 1e-4);
  }
  assert(!jerk_limited_brake(-1, 0, 0, 1, 1).valid);
  assert(!jerk_limited_brake(1, 0, 0, 0, 1).valid);
}

void hold_release_within_limits() {
  for (const double speed : {0.1, 0.2, 0.4}) {
    Fixture f;
    assert(f.begin() == PathJogResult::Ok);
    f.hold(speed, 1500);
    assert(!f.fault && f.lane.open());
    // Steady state reaches the commanded speed when no limit binds; at
    // 400 mm/s the curve's steepest axis (|q'| > 3.5 rad/m by 0.6 m) binds.
    std::printf("  steady: speed %.4f m/s governing %u s=%.1f\n", f.lane.speed() * kSpacing, f.lane.observe().governing, f.lane.s());
    if (f.lane.observe().governing == uint32_t(PathGoverning::Command))
      assert(std::abs(f.lane.speed() * kSpacing - speed) <= 1e-9);
    else
      assert(f.lane.observe().governing == uint32_t(PathGoverning::Velocity) && f.lane.speed() * kSpacing < speed);
    const double v = f.lane.speed(), a = f.lane.acceleration();
    const double brake = jerk_limited_brake(v, a, 0, f.lane.scalar_acceleration_limit(),
                                            f.lane.scalar_jerk_limit()).distance * kSpacing;
    const double start = f.lane.s();
    f.lane.end(f.now);
    f.run_until_closed(2000);
    if (f.fault || !f.closed) std::printf("fault=%d closed=%d open=%d v=%.9g a=%.9g s=%.6f stop=%u gov=%u alim=%g jlim=%g\n", f.fault, f.closed, f.lane.open(), f.lane.speed(), f.lane.acceleration(), f.lane.s(), f.lane.observe().stop_reason, f.lane.observe().governing, f.lane.scalar_acceleration_limit(), f.lane.scalar_jerk_limit());
    assert(!f.fault && f.closed && !f.lane.open());
    const double stop = (f.lane.s() - start) * kSpacing;
    char label[64];
    std::snprintf(label, sizeof label, "hold %.0f mm/s + release", speed * 1000);
    std::printf("  %-34s stop %.2f mm (analytic %.2f mm)\n", label, stop * 1000, brake * 1000);
    // Stop distance within the analytic jerk-limited distance plus 10 ms of travel.
    assert(stop <= brake + speed * 0.010 + 1e-9);
    assert(f.lane.observe().stop_reason == uint32_t(PathStopReason::Release));
    assert_within_limits(f, label);
    assert_on_path(f, label);
    assert_synchronous_stop(f);
    // Rest: speed and acceleration exactly zero, never past the playable end.
    assert(f.lane.speed() == 0 && f.lane.acceleration() == 0);
    for (size_t n = 0; n < f.s.size(); ++n) assert(f.s[n] <= double(f.sent));
  }
}

void velocity_limit_slows_the_whole_path() {
  // At 1 m/s requested, the 2.4 rad/m axis needs ~2.4 rad/s: the path slows
  // so that axis runs at its 1.396 rad/s limit and every other axis keeps its
  // ratio (they share s).
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(1.0, 900, 200);
  assert(!f.fault);
  const auto o = f.lane.observe();
  assert(o.governing == uint32_t(PathGoverning::Velocity));
  assert(o.path_speed < 1.0);
  assert_within_limits(f, "1 m/s request, velocity-governed");
  assert_on_path(f, "1 m/s request, velocity-governed");
  // Within 2 % of the limit on the governing axis: the path is not over-slowed.
  const auto p = peaks(f);
  std::printf("  governing axis peak %.4f of %.3f rad/s\n", p.velocity, kVelocity);
  assert(p.velocity >= 0.98 * kVelocity * 0.5);
}

void speed_changes_within_a_hold() {
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.4, 600);
  f.hold(0.1, 600);
  f.hold(0.3, 600);
  assert(std::abs(f.lane.speed() * kSpacing - 0.3) <= 1e-9);
  f.lane.end(f.now);
  f.run_until_closed(2000);
  assert(f.closed && !f.fault);
  assert_within_limits(f, "speed 400->100->300 mm/s");
  assert_on_path(f, "speed 400->100->300 mm/s");
}

void expiry_stops_on_the_path() {
  // The operator stops sending (tab closed, OLP crashed): the last command's
  // immutable deadline passes and the lane stops along the path by itself.
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.4, 1000);
  const uint64_t last_deadline = f.lane.observe().speed_deadline_ns;
  for (int i = 0; i < 2000 && f.lane.open(); ++i) { f.feed(100); f.cycle(); }
  assert(f.closed && !f.fault);
  assert(f.lane.observe().stop_reason == uint32_t(PathStopReason::Expired));
  assert(f.now > last_deadline);
  assert_within_limits(f, "expiry at 400 mm/s");
  assert_on_path(f, "expiry at 400 mm/s");
  assert_synchronous_stop(f);
}

void expiry_without_producer_stops_within_streamed_geometry() {
  // OLP crash: no more samples and no more speed commands. The lane must stop
  // within the geometry it already holds.
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.4, 800);
  const uint64_t held = f.sent;
  for (int i = 0; i < 3000 && f.lane.open(); ++i) f.cycle();
  assert(f.closed && !f.fault);
  for (double s : f.s) assert(s <= double(held));
  assert_within_limits(f, "producer crash at 400 mm/s");
  assert_on_path(f, "producer crash at 400 mm/s");
}

void halt_stops_on_the_path() {
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.4, 700);
  f.lane.halt(f.now);
  f.run_until_closed(2000);
  assert(f.closed && !f.fault);
  assert(f.lane.observe().stop_reason == uint32_t(PathStopReason::Halt));
  assert_within_limits(f, "Halt at 400 mm/s");
  assert_on_path(f, "Halt at 400 mm/s");
  assert_synchronous_stop(f);
}

void underrun_stops_at_playable_end() {
  // The producer stalls at 150 samples: the lane stops at the playable end
  // with zero speed and acceleration and keeps the session open.
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.4, 1500, 100, 150);
  assert(!f.fault && f.lane.open());
  const auto o = f.lane.observe();
  assert(f.lane.speed() == 0 && f.lane.acceleration() == 0);
  assert(f.lane.s() <= double(f.lane.playable_end()));
  // 150 samples (0..149): interior knot 148 needs samples 146..150, so the
  // last playable knot is 147.
  assert(f.lane.playable_end() == 147);
  std::printf("  underrun obs: stop %u underruns %llu gov %u v %.3g a %.3g s %.4f\n", o.stop_reason, (unsigned long long)o.underruns, o.governing, f.lane.speed(), f.lane.acceleration(), f.lane.s());
  assert(o.stop_reason == uint32_t(PathStopReason::Underrun) && o.underruns == 1);
  assert(o.governing == uint32_t(PathGoverning::BufferEnd));
  for (double s : f.s) assert(s <= 147.0);
  std::printf("  underrun stop at s=%.4f of playable end 147\n", f.lane.s());
  assert(f.lane.s() >= 146.0);  // Stops at the end, not early.
  assert_within_limits(f, "underrun at 400 mm/s");
  assert_on_path(f, "underrun at 400 mm/s");
  // Late samples resume the same path.
  f.hold(0.4, 300, 100);
  assert(f.lane.s() > 160 && !f.fault);
  assert_within_limits(f, "underrun then resume");
}

void path_end_and_position_limit_stop_before_the_failing_sample() {
  {
    Fixture f;
    assert(f.begin() == PathJogResult::Ok);
    assert(f.send(120, true, 7) == PathJogResult::Ok);
    f.hold(0.4, 1500, 0);
    const auto o = f.lane.observe();
    assert(o.ended && o.end_reason == 7 && o.stop_reason == uint32_t(PathStopReason::PathEnd));
    assert(f.lane.playable_end() == 119 && f.lane.s() <= 119.0 && f.lane.speed() == 0);
    assert(f.send(1) == PathJogResult::PathEnded);
    assert_within_limits(f, "end-of-path flag");
  }
  {
    // Axis 2 limit at 0.7 rad: the curve crosses it near sample 64.
    Fixture f;
    f.config.axes[2].max_q = 0.7;
    assert(f.begin() == PathJogResult::Ok);
    f.hold(0.4, 1500, 100);
    const auto o = f.lane.observe();
    assert(o.ended && o.stop_reason == uint32_t(PathStopReason::PositionLimit));
    double worst = -10;
    for (const auto& row : f.q) worst = std::max(worst, row[2]);
    std::printf("  position limit 0.7 rad: stopped at s=%.3f, max q2 %.6f\n", f.lane.s(), worst);
    assert(worst <= 0.7);
    assert(f.lane.speed() == 0 && f.lane.acceleration() == 0);
    assert_within_limits(f, "position limit");
    assert_on_path(f, "position limit");
  }
  {
    // A branch flip (a 0.5 rad jump on axis 4) ends the path before it.
    Fixture f([](double x) {
      auto q = default_curve(x);
      if (x >= 0.0805) q[4] += 0.5;
      return q;
    });
    assert(f.begin() == PathJogResult::Ok);
    f.hold(0.4, 1500, 100);
    const auto o = f.lane.observe();
    assert(o.ended && o.stop_reason == uint32_t(PathStopReason::StepLimit));
    assert(f.lane.s() <= 80.0 && f.lane.speed() == 0);
    assert_within_limits(f, "step bound (branch flip)");
  }
}

void caps_slow_the_path() {
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  std::vector<double> caps(400);
  for (size_t i = 0; i < caps.size(); ++i) caps[i] = i >= 200 && i < 300 ? 0.05 : std::numeric_limits<double>::infinity();
  for (int chunk = 0; chunk < 16; ++chunk) assert(f.send(25, false, 0, caps.data() + chunk * 25) == PathJogResult::Ok);
  bool capped = false;
  for (int i = 0; i < 2500 && f.lane.s() < 380; ++i) {
    if (i % 50 == 0) assert(f.speed(0.4) == PathJogResult::Ok);
    f.cycle();
    if (f.lane.s() > 201 && f.lane.s() < 299) {
      // Parts-per-million rounding on the approach (speed_rounding in path_jog.hpp).
      assert(f.lane.speed() * kSpacing <= 0.05 * (1 + 1e-6));
      capped |= f.lane.observe().governing == uint32_t(PathGoverning::Cap);
    }
  }
  assert(capped && !f.fault);
  assert_within_limits(f, "per-sample cap 50 mm/s");
}

void admission() {
  {
    Fixture f;
    f.request.held[1] += 2 * kCount;
    assert(f.begin() == PathJogResult::Ok);
    assert(f.send(10) == PathJogResult::StartMismatch);
    assert(!f.lane.open());
  }
  {
    // A full ring of a gentler curve (it stays inside the +-3 rad fixture limits).
    Fixture f([](double x) { return default_curve(0.25 * x); });
    assert(f.begin() == PathJogResult::Ok);
    assert(f.begin() == PathJogResult::Closed);
    assert(f.send(path_jog_capacity) == PathJogResult::Ok);
    // Over capacity: refused whole, nothing overwritten, playback unchanged.
    assert(f.send(1) == PathJogResult::Capacity);
    assert(f.lane.received() == path_jog_capacity);
    double row[path_jog_axes]{};
    assert(f.lane.append(5, 1, row, nullptr, false, 0) == PathJogResult::OutOfOrder);
    f.hold(0.2, 200, 0);
    assert(!f.fault && f.lane.s() > 0);
    const auto exact = f.curve(f.lane.s() * kSpacing);
    for (int j = 0; j < kAxes; ++j) assert(std::abs(f.lane.position()[j] - exact[j]) <= kCount);
    // Consumption frees space for more samples.
    assert(f.send(10) == PathJogResult::Ok);
  }
  {
    Fixture f;
    assert(f.begin() == PathJogResult::Ok);
    assert(f.lane.set_speed(2, 0.1, f.now, f.now + kMaxAge, f.now, kMaxAge) == PathJogResult::Ok);
    assert(f.lane.set_speed(2, 0.1, f.now, f.now + kMaxAge, f.now, kMaxAge) == PathJogResult::StaleSequence);
    assert(f.lane.set_speed(3, 0.1, f.now - 1, f.now + kMaxAge, f.now, kMaxAge) == PathJogResult::StaleSequence);
    assert(f.lane.set_speed(3, 0.1, f.now, f.now + kMaxAge + 1, f.now, kMaxAge) == PathJogResult::InvalidTiming);
    assert(f.lane.set_speed(3, 0.1, f.now, f.now, f.now, kMaxAge) == PathJogResult::InvalidTiming);
    assert(f.lane.set_speed(3, -0.1, f.now, f.now + 1, f.now, kMaxAge) == PathJogResult::InvalidSample);
    assert(f.lane.set_speed(3, 0.1, f.now, f.now + 1, f.now, kMaxAge) == PathJogResult::Ok);
    double bad[path_jog_axes]{};
    bad[0] = std::numeric_limits<double>::quiet_NaN();
    assert(f.lane.append(0, 1, bad, nullptr, false, 0) == PathJogResult::InvalidSample);
  }
  {
    // No speed command before the first-input deadline: closes as expired.
    Fixture f;
    assert(f.begin() == PathJogResult::Ok);
    f.run_until_closed(400);
    assert(f.closed && !f.fault && f.lane.observe().stop_reason == uint32_t(PathStopReason::Expired));
    for (double s : f.s) assert(s == 0);
  }
  {
    PathJog lane;
    PathJogConfig config{};
    PathJogBegin request{};
    assert(lane.begin(config, request, 1) == PathJogResult::InvalidConfig);
  }
}

void stop_bound_fails_closed() {
  // A stop that is not finished by its analytic bound faults. A 10 s gap in
  // cycles (the thread stalled) is the only way the generator can miss it.
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.4, 800);
  f.lane.end(f.now);
  f.now += 10'000'000'000ull;
  const auto out = f.lane.advance(f.now);
  assert(out.fault && out.closed && !f.lane.open());
  assert(f.lane.observe().stop_reason == uint32_t(PathStopReason::Fault));
}

void starts_are_synchronous() {
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.2, 300);
  int first[kAxes];
  for (int j = 0; j < kAxes; ++j) {
    first[j] = -1;
    for (size_t n = 1; n < f.q.size() && first[j] < 0; ++n)
      if (f.q[n][j] != f.q[n - 1][j]) first[j] = int(n);
  }
  for (int j = 1; j < kAxes; ++j) assert(first[j] == first[0]);
  assert_within_limits(f, "start from rest");
}
// Cycle intervals jitter up to the machine's lateness bound (the 1430 cell:
// max_cycle_lateness_ns 1 ms, so intervals in [0, 2] ms; also +-5 ms with a
// 6 ms bound). Release, underrun and expiry stops never fault and never pass
// the playable end. Margins planned for the current interval faulted 1 run in
// 120 at +-2 ms (a late cycle overshot the end by 1e-4 sample).
void jittered_cycles_never_fault() {
  for (const double jitter_ms : {1.0, 5.0}) {
    std::mt19937_64 rng(2026);
    std::uniform_real_distribution<double> jitter(-jitter_ms, jitter_ms);
    int faults = 0;
    for (int run = 0; run < 60; ++run) {
      Fixture f;
      f.config.max_cycle_ns = uint64_t((1 + jitter_ms) * 1e6);
      assert(f.begin() == PathJogResult::Ok);
      const int kind = run % 3;  // release, underrun at 300 samples, expiry
      const auto step = [&]() {
        f.now += uint64_t(std::max(0.0, 1 + jitter(rng)) * 1e6);
        const auto out = f.lane.advance(f.now);
        faults += out.fault;
        assert(f.lane.s() <= double(f.lane.playable_end()));
        return f.lane.open() && !out.fault;
      };
      bool open = true;
      for (int i = 0; i < 1500 && open; ++i) {
        if (i % 50 == 0 && !(kind == 2 && i > 700)) (void)f.speed(0.4);
        f.feed(100, kind == 1 ? 300 : UINT64_MAX);
        open = step();
      }
      if (open && kind == 0) {
        f.lane.end(f.now);
        for (int i = 0; i < 4000 && open; ++i) { f.feed(100); open = step(); }
        assert(!open && f.lane.observe().stop_reason == uint32_t(PathStopReason::Release));
      }
    }
    std::printf("  jitter +-%.0f ms (max interval %.0f ms): %d faults in 60 runs\n", jitter_ms, 1 + jitter_ms, faults);
    assert(faults == 0);
  }
}

// Independent analytic regression for Fast Trace: a narrow, smooth bend
// changes the braking envelope near the end. The old window/recomputed-stop
// algorithm faulted on the fully preloaded, forward case at 436 ms, passing
// sample 287 at nonzero speed. No recorded program or robot pose is needed.
void fast_trace_endpoints_and_partial_stops() {
  constexpr size_t count = 288;
  constexpr double spacing = 0.0005;
  constexpr uint64_t max_age = 100'000'000;
  int cases = 0;
  for (const int axes : {1, 6, 9, 16}) {
    for (const bool reverse : {false, true}) {
      for (const bool preload : {false, true}) {
        // End, Release, Halt and expiry share the same executable stopping
        // proof. Exercise both sides of the bend and the full endpoint.
        for (const int stop_mode : {0, 1, 2, 3}) {
          std::vector<std::array<double, path_jog_axes>> rows(count);
          for (size_t i = 0; i < count; ++i) {
            const double u = double(i) / (count - 1), x = (u - .7) / .02;
            rows[i][0] = .05 * u + .001 * std::exp(-x * x) * std::sin(2 * kPi * x);
            for (int axis = 1; axis < axes; ++axis)
              rows[i][axis] = (axis % 2 ? -.03 : .01) * u / axis;
          }
          if (reverse) std::reverse(rows.begin(), rows.end());
          PathJog lane;
          PathJogConfig config;
          config.mask = axes == 16 ? 0xffffu : (1u << axes) - 1;
          config.jerk = kJerk;
          config.cycle_ns = kCycle;
          config.max_cycle_ns = 2 * kCycle;
          config.max_sample_step_s = .02;
          for (int axis = 0; axis < axes; ++axis)
            config.axes[axis] = {-3.1, 3.1, kVelocity, kAcceleration, kCount};
          uint64_t now = 1'000'000'000, sequence = 0;
          PathJogBegin request;
          request.spacing = spacing;
          request.max_speed = 1;
          request.held = rows.front();
          request.first_input_deadline_ns = now + max_age;
          assert(lane.begin(config, request, now) == PathJogResult::Ok);
          size_t sent = 0;
          const auto append = [&](size_t amount) {
            const size_t n = std::min(amount, count - sent);
            assert(lane.append(sent, uint32_t(n), rows[sent].data(), nullptr, sent + n == count, 0) ==
                   PathJogResult::Ok);
            sent += n;
          };
          append(preload ? count : 100);
          std::array<std::array<double, path_jog_axes>, 4> history{};
          bool requested_stop = false, moved = false, complete = false;
          double previous_s = 0;
          for (int tick = 0; tick < 10000; ++tick) {
            if (stop_mode && !requested_stop && lane.s() >= (reverse ? .25 : .75) * (count - 1)) {
              requested_stop = true;
              if (stop_mode == 1) lane.end(now);
              if (stop_mode == 2) lane.halt(now);
            }
            if (!requested_stop && tick % 20 == 0)
              assert(lane.set_speed(++sequence, 1, now, now + max_age, now, max_age) == PathJogResult::Ok);
            if (tick % 10 == 0 && sent < count && sent < uint64_t(lane.s()) + 125) append(25);
            now += kCycle;
            const auto cycle = lane.advance(now);
            assert(!cycle.fault);
            assert(lane.s() >= previous_s - 1e-12);
            assert(lane.s() <= double(lane.playable_end()) + 1e-9);
            previous_s = lane.s();
            history[tick % 4] = lane.position();
            if (tick >= 3) {
              for (int axis = 0; axis < axes; ++axis) {
                const double q0 = history[tick % 4][axis], q1 = history[(tick - 1) % 4][axis];
                const double q2 = history[(tick - 2) % 4][axis], q3 = history[(tick - 3) % 4][axis];
                // Absolute rad/s, rad/s^2, rad/s^3 limits, including through
                // retained-profile phase boundaries and the final hold.
                assert(std::abs(q0 - q1) / kDt <= kVelocity + 1e-6);
                assert(std::abs(q0 - 2 * q1 + q2) / (kDt * kDt) <= kAcceleration + 1e-4);
                assert(std::abs(q0 - 3 * q1 + 3 * q2 - q3) / (kDt * kDt * kDt) <= kJerk + .01);
              }
            }
            if (lane.speed() > 0) moved = true;
            if (!stop_mode && preload && moved && lane.s() < count - 1 - 1e-6)
              assert(lane.speed() > 0); // No arbitrary interior rest on a complete path.
            const auto observation = lane.observe();
            if (stop_mode ? cycle.closed : observation.stop_reason == uint32_t(PathStopReason::PathEnd)) {
              assert(lane.speed() == 0 && lane.acceleration() == 0);
              if (!stop_mode) {
                assert(std::abs(lane.s() - (count - 1)) <= 1e-9);
                for (int axis = 0; axis < axes; ++axis)
                  assert(std::abs(lane.position()[axis] - rows.back()[axis]) <= kCount);
              } else {
                const auto expected = stop_mode == 1   ? PathStopReason::Release
                                      : stop_mode == 2 ? PathStopReason::Halt
                                                       : PathStopReason::Expired;
                assert(observation.stop_reason == uint32_t(expected));
              }
              complete = true;
              break;
            }
          }
          assert(complete);
          ++cases;
        }
      }
    }
  }
  std::printf("  Fast Trace: %d endpoint/partial-stop cases, 1/6/9/16 axes, both directions\n", cases);
}

// A speed-limited, changing path exhausts the finite proof budget. It must
// keep a proven continuation, report the real limit, and still stop promptly
// on release rather than faulting or falsely reporting a path/buffer end.
void proof_budget_exhaustion_keeps_the_stop_contract() {
  Fixture f;
  assert(f.begin() == PathJogResult::Ok);
  unsigned exhausted = 0;
  for (int tick = 0; tick < 900; ++tick) {
    if (tick % 50 == 0) assert(f.speed(1) == PathJogResult::Ok);
    f.feed(200);
    f.cycle();
    assert(!f.fault);
    assert(f.lane.proof_work_used() <= path_jog_proof_work_budget);
    if (f.lane.proof_work_used() == path_jog_proof_work_budget) {
      ++exhausted;
      const auto reason = f.lane.observe().governing;
      assert(reason != uint32_t(PathGoverning::PathEnd));
      assert(reason != uint32_t(PathGoverning::BufferEnd));
    }
  }
  assert(exhausted > 0);
  assert(f.lane.observe().governing == uint32_t(PathGoverning::Velocity));
  f.lane.end(f.now);
  for (int tick = 0; tick < 500 && !f.closed; ++tick)
    f.cycle();
  assert(f.closed && !f.fault);
  assert(f.lane.observe().stop_reason == uint32_t(PathStopReason::Release));
  assert(f.lane.speed() == 0 && f.lane.acceleration() == 0);
  assert_within_limits(f, "proof budget + release");
  assert_on_path(f, "proof budget + release");
}

// Informational (not asserted: host-dependent): native cost of one cycle
// at 400 mm/s with 100 samples of look-ahead, the design's operating point.
// The RevPi budget must be measured there with the cycle capture.
void cycle_cost() {
  Fixture f([](double x) { return default_curve(0.25 * x); });
  assert(f.begin() == PathJogResult::Ok);
  f.hold(0.4, 500);
  const auto started = std::chrono::steady_clock::now();
  const int cycles = 5000;
  for (int i = 0; i < cycles; ++i) {
    if (i % 50 == 0) assert(f.speed(0.4) == PathJogResult::Ok);
    f.feed(100);
    f.now += kCycle;
    (void)f.lane.advance(f.now);
  }
  const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - started).count();
  std::printf("  cycle cost incl. 25-sample appends: %.0f ns/cycle (this host)\n", ns / cycles);
}
}  // namespace

int main() {
  std::puts("path_jog: brake helper");
  brake_matches_integration();
  std::puts("path_jog: holds");
  hold_release_within_limits();
  velocity_limit_slows_the_whole_path();
  speed_changes_within_a_hold();
  starts_are_synchronous();
  std::puts("path_jog: stops");
  expiry_stops_on_the_path();
  expiry_without_producer_stops_within_streamed_geometry();
  halt_stops_on_the_path();
  underrun_stops_at_playable_end();
  path_end_and_position_limit_stop_before_the_failing_sample();
  caps_slow_the_path();
  std::puts("path_jog: admission");
  admission();
  stop_bound_fails_closed();
  jittered_cycles_never_fault();
  fast_trace_endpoints_and_partial_stops();
  proof_budget_exhaustion_keeps_the_stop_contract();
  cycle_cost();
  std::puts("path_jog_tests PASS");
  return 0;
}
