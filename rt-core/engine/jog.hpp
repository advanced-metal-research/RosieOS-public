#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace rosie::rtcore {
// Convert positive profile durations without overflow or truncating a cycle.
// Zero is invalid and must never silently disable an arrest window.
constexpr uint64_t jog_duration_cycles(uint64_t ns, uint64_t cycle_ns) noexcept {
  return ns && cycle_ns ? ns / cycle_ns + (ns % cycle_ns != 0) : 0;
}
static_assert(jog_duration_cycles(1, 10'000'000) >= 1);
static_assert(jog_duration_cycles(200'000'000, 1'000'000) == 200);
static_assert(jog_duration_cycles(300'000'000, 1'000'000) == 300);

struct JogTiming {
  // Unverified defaults preserve the legacy 1 ms arrest and quick-stop windows.
  uint64_t arrest_ns = 200'000'000, quick_stop_ns = 300'000'000;
  uint16_t arrest_cycles = 0, quick_stop_cycles = 0;
  // Machine-level jog_jerk (axes.conf jog_jerk=<jerk>|<cycles>, identical on
  // every axis): jerk bound in joint units/s^3 and the JogSmoother window that
  // enforces it. 0 when the machine omits it: unsmoothed, acceleration-limited.
  uint32_t jerk = 0, smoothing_cycles = 0;

  bool configure(uint64_t cycle_ns) noexcept {
    const auto arrest = jog_duration_cycles(arrest_ns, cycle_ns);
    const auto stop = jog_duration_cycles(quick_stop_ns, cycle_ns);
    if (!arrest || !stop || arrest > UINT16_MAX || stop > UINT16_MAX) return false;
    arrest_cycles = static_cast<uint16_t>(arrest);
    quick_stop_cycles = static_cast<uint16_t>(stop);
    return true;
  }
};

// A servo trails its commanded target by a near-constant time, so the tracking
// error it produces grows with commanded speed and a static bound is crossed
// by ordinary fast motion. The effective bound is the configured static bound
// plus |velocity| x lag_s. bound and velocity share one frame (counts and
// counts/s, or rad and rad/s), so a time allowance needs no per-axis number.
// lag_s is the machine's tracking_lag_allowance; 0 (absent) returns the static
// bound unchanged, and a nonfinite velocity with a nonzero lag yields NaN,
// which every caller treats as invalid rather than as an unbounded allowance.
inline double speed_scaled_bound(double bound, double velocity, double lag_s) noexcept {
  return lag_s > 0 ? bound + std::abs(velocity) * lag_s : bound;
}

// Jerk limit for jog and its Halt/EndJog/expiry ramps. The jog integrator is
// acceleration-limited only (|dv| <= a*dt per cycle), so every start, stop and
// reversal steps the commanded acceleration within one cycle: unbounded jerk.
// A Rosie 1430 arm's 2026-10-06 stop dump: 720 deg/s^2 reached in one 1 ms cycle on every
// jog start and EndJog, and the wrist (J4) rang at ~5.9 Hz with up to 0.45
// degrees of error after jog starts and stops.
//
// The commanded target is the mean of the integrator's last `window`
// positions. For any input sequence, including reversals, the mean's
// acceleration stays within a and its jerk within 2*a/(window*cycle): each
// cycle the mean acceleration changes by (a_entering - a_leaving)/window. The
// compiler sizes window = ceil(2*a_max/jerk/cycle) from the machine's one
// jog_jerk value. Every output is a convex combination of integrator
// positions, so it never leaves the interval the integrator was confined to
// and the position governor (jog_position_scale) keeps judging the integrator.
// The cost is time: motion continues for one window after the integrator
// stops, and a stop travels |v|*window*cycle/2 further than v^2/(2a).
struct JogSmoother {
  // Admission bound on window cycles (1.024 s at 1 kHz); sizes the fixed
  // per-axis history so the cyclic thread never allocates.
  static constexpr uint32_t max_window = 1024;
  double raw_position = 0, raw_velocity = 0, output_position = 0, output_velocity = 0;

  // 0 or 1 cycles disables smoothing.
  bool configure(uint32_t cycles, double cycle_seconds) noexcept {
    if (cycles > max_window || !(cycle_seconds > 0) || !std::isfinite(cycle_seconds)) return false;
    window_ = cycles; cycle_s_ = cycle_seconds;
    return true;
  }
  bool enabled() const noexcept { return window_ > 1; }
  uint32_t window() const noexcept { return window_; }

  // History of steady motion at `velocity` centred on `position`, so the
  // output continues from that state without a position or velocity step. The
  // integrator starts half a window ahead of the output.
  void seed(double position, double velocity) noexcept {
    if (!enabled()) return;
    origin_ = position; head_ = 0; sum_ = 0;
    quiet_ = velocity == 0 ? window_ : 0;
    const double centre = .5 * double(window_ - 1);
    for (uint32_t i = 0; i < window_; ++i) {
      history_[i] = velocity * cycle_s_ * (double(i) - centre);
      sum_ += history_[i];
    }
    raw_position = origin_ + history_[window_ - 1];
    raw_velocity = velocity;
    output_position = position;
    output_velocity = velocity;
  }

  // One integrator step per command cycle. Once the integrator has held zero
  // velocity for a whole window every retained position is the same, so the
  // output equals it exactly and its velocity is exactly zero.
  void push(double position, double velocity) noexcept {
    raw_position = position; raw_velocity = velocity;
    const double entering = position - origin_, leaving = history_[head_];
    sum_ += entering - leaving;
    history_[head_] = entering;
    if (++head_ == window_) {
      head_ = 0;
      sum_ = 0;
      for (uint32_t i = 0; i < window_; ++i) sum_ += history_[i];
    }
    quiet_ = velocity == 0 ? std::min(quiet_ + 1, window_) : 0;
    if (quiet_ == window_) {
      output_position = position; output_velocity = 0;
      return;
    }
    output_position = origin_ + sum_ / double(window_);
    output_velocity = (entering - leaving) / (double(window_) * cycle_s_);
  }

 private:
  std::array<double, max_window> history_{};
  uint32_t window_ = 0, head_ = 0, quiet_ = 0;
  double cycle_s_ = 0, origin_ = 0, sum_ = 0;
};

// All inputs share one coordinate frame. The caller converts configured rad/m
// limits into that frame once; no vendor identity enters this policy.
struct JogAxis {
  double position = 0, velocity = 0;
  // Target minus feedback after the last admitted step, the effective bound it
  // was judged against (max_lead scaled by this step's commanded velocity),
  // and whether it exceeded that bound. The caller reports it as a warning.
  double lead = 0, lead_limit = 0;
  bool lead_exceeded = false;
  // Optional jerk smoother owned by the caller. When enabled, position and
  // velocity are its output (the commanded target) and the acceleration-limited
  // integrator state lives in it.
  JogSmoother* smoother = nullptr;
  bool smoothed() const noexcept { return smoother && smoother->enabled(); }
  double integrator_position() const noexcept { return smoothed() ? smoother->raw_position : position; }
  double integrator_velocity() const noexcept { return smoothed() ? smoother->raw_velocity : velocity; }
  void reset(double held) noexcept {
    position = held; velocity = 0; lead = 0; lead_limit = 0; lead_exceeded = false;
    if (smoothed()) smoother->seed(held, 0);
  }
  // Returns false only for invalid input. A lead beyond the bound is admitted
  // and reported, never refused: refusing it disarmed a Rosie 1430 arm mid-jog on
  // 2026-10-06 (J2, 0.35 deg bound) with nothing wrong at the robot. The bound
  // is max_lead + |commanded velocity| x lag_s (speed_scaled_bound). Tracking
  // stays bounded by the output policy's following-error fault
  // (limits.following_error, scaled the same way, held for
  // limits.following_error_timeout_ns). `stops` marks the step a smoothed
  // Halt ramp sized to end exactly at zero, so no few-ulp residue enters the
  // smoother's history as motion.
  bool advance(double requested, double feedback, double acceleration,
               double max_lead, double dt, double lag_s = 0,
               bool stops = false) noexcept {
    if (!(dt > 0) || !(acceleration > 0) || !(max_lead > 0) ||
        !std::isfinite(requested) || !std::isfinite(feedback) ||
        !std::isfinite(lag_s) || lag_s < 0) return false;
    const double integrated = integrator_velocity();
    double next_velocity = integrated + std::clamp(requested - integrated,
        -acceleration * dt, acceleration * dt);
    if (stops && smoothed()) next_velocity = 0;
    // A final stop occupies only v/a of this cycle. Integrating the whole
    // interval after reaching zero would exceed the v^2/(2a) stopping budget.
    const double motion_dt = requested == 0 && next_velocity == 0
        ? std::min(dt, std::abs(integrated) / acceleration) : dt;
    const double next_position = integrator_position() + (integrated + next_velocity) * .5 * motion_dt;
    double target = next_position, target_velocity = next_velocity;
    if (smoothed()) {
      smoother->push(next_position, next_velocity);
      target = smoother->output_position;
      target_velocity = smoother->output_velocity;
    }
    const double error = target - feedback;
    lead = error;
    lead_limit = speed_scaled_bound(max_lead, target_velocity, lag_s);
    lead_exceeded = std::abs(error) > lead_limit;
    position = target; velocity = target_velocity;
    return true;
  }
  // Take a target produced elsewhere (the path jog lane) without integrating
  // or smoothing it: the path generator already bounds every axis's velocity,
  // acceleration and jerk from one scalar, and a per-axis clamp or moving
  // average here would bend the path. Lead is judged exactly as in advance.
  bool follow(double target, double target_velocity, double feedback, double max_lead,
              double lag_s = 0) noexcept {
    if (!std::isfinite(target) || !std::isfinite(target_velocity) || !std::isfinite(feedback) ||
        !(max_lead > 0) || !std::isfinite(lag_s) || lag_s < 0) return false;
    const double error = target - feedback;
    lead = error;
    lead_limit = speed_scaled_bound(max_lead, target_velocity, lag_s);
    lead_exceeded = std::abs(error) > lead_limit;
    position = target; velocity = target_velocity;
    return true;
  }
};

struct FollowingErrorGuard {
  uint64_t outside_since = 0;
  void reset() noexcept { outside_since = 0; }
  bool accept(double error, double limit, uint64_t now, uint64_t timeout) noexcept {
    if (!std::isfinite(error) || !(limit > 0) || !timeout) return false;
    if (std::abs(error) <= limit) { reset(); return true; }
    if (!outside_since) outside_since = now;
    return now >= outside_since && now - outside_since < timeout;
  }
};
}
