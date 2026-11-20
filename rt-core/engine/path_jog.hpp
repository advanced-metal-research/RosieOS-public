#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include "jog_session.hpp"
#include "execution_policy.hpp"

namespace rosie::rtcore {

// Path jog: generic playback of a streamed joint-space curve along one path
// parameter. It exists because a Cartesian jog streamed as per-axis joint
// velocities left the commanded line by 0.9-75 mm on a Rosie 1430 arm (2026-10-06):
// each axis ramped and stopped on its own and the source steered from a stale
// measured pose. Here an application streams exact joint positions of its
// path (for a Cartesian line: seeded IK every sample) and every masked axis is
// derived from the single path parameter s, so all axes start, slow and stop
// together and the commanded target never leaves the sampled curve.
//
// No robot model enters this file. Samples are opaque joint vectors at a
// constant spacing in opaque path units. Every limit comes from compiled
// configuration (axis velocity and jog acceleration, machine jog_jerk); the
// sample spacing, speed ceiling and per-sample speed caps come from the
// application and can only lower the speed.
//
// Geometry: knot k is sample k. First and second derivatives at each knot are
// native finite differences in sample-index units (central inside, second
// order one-sided at the ends), never application-supplied, so there is no
// consistency tolerance to configure. Each segment is the quintic Hermite
// through its two knots' position, slope and curvature: continuous to the
// second derivative, so joint acceleration has no steps and joint jerk is
// bounded. A segment is playable only when both knots' derivatives are known:
// the last sample becomes playable when its successor or the end-of-path flag
// arrives. playable_end() is the hard stop point.
//
// Limits: with q', q'', q''' the segment's derivatives in s and Q1..Q3 their
// Bernstein bounds over the segment, the chain rule gives for every axis j
//   velocity      |q'| v                        <= V_j
//   acceleration  |q'| |a| + |q''| v^2          <= A_j
//   jerk          |q'| |js| + 3|q''| v|a| + |q'''| v^3 <= J
// where v, a, js are the path speed, acceleration and jerk. A segment's speed
// limit reserves Q2 v^2 <= A/2 and Q3 v^3 <= J/3; its scalar acceleration and
// jerk limits take what remains at the session speed ceiling. Playing within
// those three scalar limits therefore keeps every joint within V, A and J on
// every cycle; no axis is ever clamped on its own, which would bend the path.
//
// Speed: one jerk-limited profile on v tracks min(operator speed, limits).
// Before every cycle it certifies the step and retains an executable stop
// with zero speed and acceleration inside the playable geometry. Future
// lower speed limits are approached with zero acceleration so a limit
// transition does not unnecessarily consume all progress into a full stop.
// Release, input expiry, Halt, an exhausted buffer, a path end, and a
// stalled or crashed source are all the same stop along the path.

inline constexpr uint32_t path_jog_axes = jog_session_axes;
// Fixed ring capacity: generic buffer sizing, not a machine number. 1024
// samples x 16 axes x 8 bytes = 128 KiB owned by the cyclic thread; at 1 mm
// spacing it holds 1 m of look-ahead, ten times the 100 mm design look-ahead.
inline constexpr uint32_t path_jog_capacity = 1024;
// Computation bound, independent of machine motion limits. One unit is one
// segment aggregate check, axis check, or local time-bracket iteration.
inline constexpr uint32_t path_jog_proof_work_budget = 1024;

enum class PathJogResult : uint32_t {
  Ok = 0,
  Closed = 1,          // No open session, or it already ended.
  InvalidConfig = 2,   // Nonfinite or nonpositive limits, spacing or ceiling.
  InvalidSample = 3,   // Nonfinite sample or cap.
  StartMismatch = 4,   // Sample 0 is not the held command; the session closes.
  Capacity = 5,        // Append would overwrite unconsumed samples.
  OutOfOrder = 6,      // Append is not contiguous with the samples held.
  StaleSequence = 7,   // Speed command sequence or origin went backwards.
  InvalidTiming = 8,   // Speed deadline not after now, or beyond the input age.
  PathEnded = 9,       // Append after the end of the path.
};

// What limited the path speed this cycle; published for the operator and logs.
enum class PathGoverning : uint32_t {
  None = 0,
  Command = 1,       // The operator's speed.
  Velocity = 2,      // An axis velocity limit.
  Curvature = 3,     // An axis acceleration limit at the path's curvature.
  Jerk = 4,          // An axis jerk limit at the path's third derivative.
  Cap = 5,           // An application per-sample cap (e.g. near a singularity).
  BufferEnd = 6,     // The end of the samples received so far.
  PathEnd = 7,       // The end-of-path flag or a native sample refusal.
  Stopping = 8,      // Release, expiry or Halt.
};

enum class PathStopReason : uint32_t {
  None = 0,
  Release = 1,       // End requested.
  Expired = 2,       // The speed command's immutable deadline passed.
  Halt = 3,          // Public Halt.
  Underrun = 4,      // Stopped at the end of the samples received so far.
  PathEnd = 5,       // Stopped at the end of the path.
  PositionLimit = 6, // A native position check ended the path.
  StepLimit = 7,     // A native joint-step check ended the path.
  Fault = 8,         // Fail closed: stop bound overrun, playable end passed, invalid state.
  Inhibited = 9,     // abandon(): authority loss, inhibit or Home closed the session.
};

struct PathJogAxisConfig {
  double min_q = 0, max_q = 0;    // Closed logical position interval.
  double velocity = 0;            // Configured maximum velocity, joint units/s.
  double acceleration = 0;        // Configured jog acceleration, joint units/s^2.
  double start_tolerance = 0;     // Sample 0 versus the held command, joint units.
};

struct PathJogConfig {
  uint32_t mask = 0;
  std::array<PathJogAxisConfig, path_jog_axes> axes{};
  double jerk = 0;                // Machine jog_jerk, joint units/s^3, every axis.
  // A sample step |q_i - q_(i-1)| above velocity x this is refused: a branch
  // flip or a near-singular jump would otherwise be interpolated as motion
  // off the application's path.
  double max_sample_step_s = 0;
  uint64_t cycle_ns = 0;
  // Longest interval between two cycles the machine runs without a cycle
  // deadline fault (cycle + max_cycle_lateness_ns); 0 = cycle_ns. Braking
  // margins are planned for it: with margins sized by the current interval,
  // a late cycle (+-2 ms jitter) overshot the playable end by 1e-4 sample.
  uint64_t max_cycle_ns = 0;
};

struct PathJogBegin {
  double spacing = 0;             // Path units per sample.
  double max_speed = 0;           // Path units/s; the ceiling for every speed command.
  uint64_t first_input_deadline_ns = 0;
  std::array<double, path_jog_axes> held{};
};

struct PathJogObservation {
  double path_s = 0, path_speed = 0, path_acceleration = 0;  // Path units.
  uint64_t consumed_index = 0, playable_end_index = 0, received = 0;
  uint64_t underruns = 0, speed_sequence = 0, speed_deadline_ns = 0;
  uint32_t governing = 0, stop_reason = 0, end_reason = 0;
  bool open = false, stopping = false, ended = false, has_speed = false;
};

struct PathJogCycle {
  bool active = false;   // Targets were produced for the masked axes.
  bool fault = false;    // Fail closed; the caller inhibits output.
  bool closed = false;   // The session closed this cycle (stop completed).
};

namespace path_jog_detail {
// Numerical reserve on configured limits (not a tuning value): absorbs
// rounding in the Bernstein bounds and the per-cycle integration, and the
// speed_rounding allowance below, so a path at a limit cannot report even a
// few parts per billion above it.
inline constexpr double limit_reserve = 1 - 2e-6;
// A discrete approach onto a lower speed limit settles from above within
// parts per ten million (2.8e-7 measured on a 50 mm/s cap, path_jog_tests);
// the touched-segment check admits that rounding instead of forcing an
// unnecessary stop. limit_reserve keeps the joints inside their limits.
inline constexpr double speed_rounding = 1 + 1e-6;

// Maximum magnitude on [0, 1] of a polynomial with monomial coefficients
// c[0..n] (n <= 5), by the convex-hull property of its Bernstein form.
inline double bernstein_abs_bound(const double* c, int n) noexcept {
  static constexpr double binomial[6][6] = {
      {1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {1, 2, 1, 0, 0, 0},
      {1, 3, 3, 1, 0, 0}, {1, 4, 6, 4, 1, 0}, {1, 5, 10, 10, 5, 1}};
  double bound = 0;
  for (int i = 0; i <= n; ++i) {
    double b = 0;
    for (int k = 0; k <= i; ++k) b += binomial[i][k] / binomial[n][k] * c[k];
    bound = std::max(bound, std::abs(b));
  }
  return bound;
}
inline void bernstein_range(const double* c, int n, double& low, double& high) noexcept {
  static constexpr double binomial[6][6] = {
      {1, 0, 0, 0, 0, 0}, {1, 1, 0, 0, 0, 0}, {1, 2, 1, 0, 0, 0},
      {1, 3, 3, 1, 0, 0}, {1, 4, 6, 4, 1, 0}, {1, 5, 10, 10, 5, 1}};
  low = std::numeric_limits<double>::infinity();
  high = -low;
  for (int i = 0; i <= n; ++i) {
    double b = 0;
    for (int k = 0; k <= i; ++k) b += binomial[i][k] / binomial[n][k] * c[k];
    low = std::min(low, b);
    high = std::max(high, b);
  }
}

// Quintic Hermite on u in [0, 1]: q(u) = p0 + sum c[n] u^n, n = 1..5, from
// both knots' position p, slope m and curvature k (sample-index units).
// Coefficients use the knot difference so a large absolute position cannot
// cancel the small per-sample motion.
struct Quintic {
  double p0 = 0;
  double c[6]{};
  void set(double q0, double q1, double m0, double m1, double k0, double k1) noexcept {
    const double d = q1 - q0;
    p0 = q0;
    c[0] = 0;
    c[1] = m0;
    c[2] = k0 * .5;
    c[3] = 10 * d - 6 * m0 - 4 * m1 - 1.5 * k0 + .5 * k1;
    c[4] = -15 * d + 8 * m0 + 7 * m1 + 1.5 * k0 - k1;
    c[5] = 6 * d - 3 * m0 - 3 * m1 - .5 * k0 + .5 * k1;
  }
  double position(double u) const noexcept {
    return p0 + u * (c[1] + u * (c[2] + u * (c[3] + u * (c[4] + u * c[5]))));
  }
  double slope(double u) const noexcept {
    return c[1] + u * (2 * c[2] + u * (3 * c[3] + u * (4 * c[4] + u * 5 * c[5])));
  }
  void derivative_coefficients(double d1[5], double d2[4], double d3[3]) const noexcept {
    for (int n = 1; n <= 5; ++n) d1[n - 1] = n * c[n];
    for (int n = 2; n <= 5; ++n) d2[n - 2] = n * (n - 1) * c[n];
    for (int n = 3; n <= 5; ++n) d3[n - 3] = n * (n - 1) * (n - 2) * c[n];
  }
};
}  // namespace path_jog_detail

class PathJog {
 public:
  bool open() const noexcept { return open_; }
  bool stopping() const noexcept { return stop_ != PathStopReason::None && commanded_stop_; }
  // Index of the last knot playback may reach; 0 until the first segment.
  uint64_t playable_end() const noexcept { return playable_; }
  uint64_t received() const noexcept { return received_; }
  double s() const noexcept { return s_; }
  double speed() const noexcept { return v_; }
  double acceleration() const noexcept { return a_; }
  const std::array<double, path_jog_axes>& position() const noexcept { return q_out_; }
  const std::array<double, path_jog_axes>& velocity() const noexcept { return v_out_; }
  // Scalar limits in use (sample-index units), for tests and diagnostics.
  double scalar_acceleration_limit() const noexcept { return a_lim_; }
  double scalar_jerk_limit() const noexcept { return j_lim_; }
  uint32_t proof_work_used() const noexcept { return path_jog_proof_work_budget - proof_work_left_; }
  // Segment k's speed, scalar acceleration and jerk limits (index units), for tests.
  std::array<double, 3> segment_limits(uint64_t k) const noexcept {
    const auto& l = limits_[k % path_jog_capacity];
    return {l.speed, l.acceleration, l.jerk};
  }

  PathJogResult begin(const PathJogConfig& config, const PathJogBegin& request, uint64_t now) noexcept {
    if (open_) return PathJogResult::Closed;
    if (!valid_config(config) || !std::isfinite(request.spacing) || !(request.spacing > 0) ||
        !std::isfinite(request.max_speed) || !(request.max_speed > 0) ||
        request.first_input_deadline_ns <= now)
      return PathJogResult::InvalidConfig;
    for (uint32_t axis = 0; axis < path_jog_axes; ++axis)
      if ((config.mask & (1u << axis)) && !std::isfinite(request.held[axis]))
        return PathJogResult::InvalidConfig;
    // Reset scalar state only: the ring is indexed from zero again and is
    // never read before it is written. No 128 KiB temporary on the RT stack.
    received_ = retained_ = playable_ = next_segment_ = 0;
    truncated_ = false;
    final_knot_ = UINT64_MAX;
    s_ = v_ = a_ = target_ = 0;
    a_lim_ = j_lim_ = 1;
    fallback_ = {};
    certificate_binding_ = PathGoverning::None;
    proof_work_left_ = path_jog_proof_work_budget;
    speed_sequence_ = speed_origin_ = 0;
    stop_bound_ns_ = UINT64_MAX;
    stop_window_end_ = underruns_ = 0;
    end_reason_ = 0;
    stop_ = closed_reason_ = PathStopReason::None;
    end_stop_ = PathStopReason::PathEnd;
    governing_ = PathGoverning::None;
    ended_ = commanded_stop_ = has_speed_ = at_end_ = stopped_at_end_ = moved_ = false;
    first_cycle_ = true;
    v_out_.fill(0);
    config_ = config;
    spacing_ = request.spacing;
    max_speed_ = request.max_speed / request.spacing;
    held_ = request.held;
    q_out_ = request.held;
    speed_deadline_ns_ = request.first_input_deadline_ns;
    last_ns_ = now;
    open_ = true;
    return PathJogResult::Ok;
  }

  // Append count samples starting at first (row-major, path_jog_axes values
  // per sample; unmasked axes are ignored). caps, when given, holds one
  // per-sample speed cap in path units/s (infinity for none); a cap can only
  // lower the speed. end marks the last sample of the path, with the
  // application's reason code passed through to the observation. A sample
  // outside the position limits or beyond the step bound ends the path at the
  // sample before it; the samples after it are dropped.
  PathJogResult append(uint64_t first, uint32_t count, const double* samples, const double* caps,
                       bool end, uint32_t end_reason) noexcept {
    // Appends stay admitted while stopping: the stop window is frozen at the
    // request, so late geometry cannot extend it, and a producer racing a
    // release does not see a spurious refusal.
    if (!open_) return PathJogResult::Closed;
    if (ended_) return PathJogResult::PathEnded;
    if (first != received_) return PathJogResult::OutOfOrder;
    if ((count && !samples) || (!count && !end)) return PathJogResult::InvalidSample;
    if (count > path_jog_capacity - (received_ - retained_)) return PathJogResult::Capacity;
    for (uint32_t i = 0; i < count; ++i) {
      for (uint32_t axis = 0; axis < path_jog_axes; ++axis)
        if ((config_.mask & (1u << axis)) && !std::isfinite(samples[i * path_jog_axes + axis]))
          return PathJogResult::InvalidSample;
      if (caps && (std::isnan(caps[i]) || !(caps[i] > 0))) return PathJogResult::InvalidSample;
    }
    if (received_ == 0 && count) {
      for (uint32_t axis = 0; axis < path_jog_axes; ++axis) {
        if (!(config_.mask & (1u << axis))) continue;
        if (!(std::abs(samples[axis] - held_[axis]) <= config_.axes[axis].start_tolerance)) {
          close(PathStopReason::Fault);
          return PathJogResult::StartMismatch;
        }
      }
    }
    for (uint32_t i = 0; i < count && !ended_; ++i) {
      const double* sample = samples + i * path_jog_axes;
      const PathStopReason refusal = check_sample(sample);
      if (refusal != PathStopReason::None) {
        finish_path(refusal, 0);
        break;
      }
      const uint64_t index = received_;
      auto& slot = ring_[index % path_jog_capacity];
      for (uint32_t axis = 0; axis < path_jog_axes; ++axis)
        slot.q[axis] = (config_.mask & (1u << axis)) ? sample[axis] : 0;
      slot.cap = caps && std::isfinite(caps[i]) ? caps[i] / spacing_ : std::numeric_limits<double>::infinity();
      ++received_;
      // Sample 2 fixes knots 0 and 1 (end rules); sample n >= 4 fixes the
      // interior knot n-2, completing segment n-3.
      const uint64_t ready = index == 2 ? 1 : index >= 4 ? index - 2 : next_segment_;
      while (!ended_ && next_segment_ < ready) complete_segment(next_segment_++);
    }
    if (end && !ended_) finish_path(PathStopReason::PathEnd, end_reason);
    return PathJogResult::Ok;
  }

  // One operator speed command (path units/s). Admission mirrors jog updates:
  // the sequence increases, the origin never goes backwards (equal is
  // allowed), the deadline is after now and at most max_age after the origin.
  // A speed above the session ceiling is applied at the ceiling.
  PathJogResult set_speed(uint64_t sequence, double speed, uint64_t origin, uint64_t deadline,
                          uint64_t now, uint64_t max_age) noexcept {
    if (!open_ || commanded_stop_) return PathJogResult::Closed;
    if (!std::isfinite(speed) || speed < 0) return PathJogResult::InvalidSample;
    if (sequence <= speed_sequence_ || origin < speed_origin_) return PathJogResult::StaleSequence;
    if (deadline <= now || deadline < origin || deadline - origin > max_age)
      return PathJogResult::InvalidTiming;
    speed_sequence_ = sequence;
    speed_origin_ = origin;
    speed_deadline_ns_ = deadline;
    target_ = std::min(speed / spacing_, max_speed_);
    has_speed_ = true;
    return PathJogResult::Ok;
  }

  // Release and Halt stop along the path within the configured limits; the
  // session closes when speed and acceleration reach zero.
  void end(uint64_t now) noexcept { request_stop(PathStopReason::Release, now); }
  void halt(uint64_t now) noexcept { request_stop(PathStopReason::Halt, now); }

  // Fail closed immediately (authority loss, inhibit, Home); the caller holds
  // the last target. Not a path-preserving stop: used only when outputs are
  // already inhibited. Reported as Inhibited, not Fault: a composed run under
  // container load lost authority mid-hold and the client read it as a
  // generator fault.
  void abandon() noexcept { close(PathStopReason::Inhibited); }

  // One cycle at its time base `now`: the instant the cycle's targets stand
  // for (the cycle machine passes its wake time), so a cycle's own work
  // before the call cannot move the target. The first cycle may predate
  // begin(), which runs during that cycle's intake.
  PathJogCycle advance(uint64_t now) noexcept {
    PathJogCycle out;
    if (!open_) return out;
    if (!first_cycle_ && now < last_ns_) return fail(out);
    const double dt = (first_cycle_ ? double(config_.cycle_ns) : double(now - last_ns_)) / 1e9;
    first_cycle_ = false;
    last_ns_ = now;
    out.active = true;
    if (!(dt > 0)) {
      // A repeated timestamp holds the previous target unchanged.
      return out;
    }
    if (!commanded_stop_ && now >= speed_deadline_ns_) request_stop(PathStopReason::Expired, now);
    if (commanded_stop_ && v_ != 0 && now > stop_bound_ns_) return fail(out);
    // After stopping at the end of the geometry, wait for new samples rather
    // than creeping toward the same end in sub-micrometre steps.
    const bool waiting = stopped_at_end_ && playable_ == stopped_playable_;
    const double target = commanded_stop_ || waiting ? 0 : target_;
    update_tracker_limits();
    const double s0 = s_;
    certificate_step(target, dt);
    if (waiting && !commanded_stop_) governing_ = ended_ ? PathGoverning::PathEnd : PathGoverning::BufferEnd;
    if (!std::isfinite(s_) || !std::isfinite(v_) || !std::isfinite(a_) || v_ < -1e-12 ||
        s_ < s0 - 1e-12 || s_ > double(playable_) + 1e-9)
      return fail(out);
    if (v_ < 0) v_ = 0;
    if (s_ > double(playable_)) s_ = double(playable_);
    // Knot k reads samples k-2..k+2: keep two behind the current segment.
    retained_ = std::max<uint64_t>(retained_, segment_at(s_) > 1 ? segment_at(s_) - 2 : 0);
    evaluate();
    if (v_ == 0 && a_ == 0) {
      if (commanded_stop_) {
        close(stop_);
        out.closed = true;
      } else if (target_ > 0 && at_end_ && !stopped_at_end_) {
        // Counted once per arrival: a lane waiting at s = 0 for its first
        // geometry has not run out of samples.
        stopped_at_end_ = true;
        stopped_playable_ = playable_;
        if (ended_) stop_ = end_stop_;
        else if (moved_) { stop_ = PathStopReason::Underrun; ++underruns_; }
      }
    } else {
      stopped_at_end_ = false;
      moved_ = true;
      if (!commanded_stop_) stop_ = PathStopReason::None;
    }
    return out;
  }

  PathJogObservation observe() const noexcept {
    PathJogObservation o;
    o.path_s = s_ * spacing_;
    o.path_speed = v_ * spacing_;
    o.path_acceleration = a_ * spacing_;
    o.consumed_index = static_cast<uint64_t>(std::floor(s_));
    o.playable_end_index = playable_;
    o.received = received_;
    o.underruns = underruns_;
    o.speed_sequence = speed_sequence_;
    o.speed_deadline_ns = speed_deadline_ns_;
    o.governing = static_cast<uint32_t>(governing_);
    o.stop_reason = static_cast<uint32_t>(open_ ? stop_ : closed_reason_);
    o.end_reason = end_reason_;
    o.open = open_;
    o.stopping = commanded_stop_;
    o.ended = ended_;
    o.has_speed = has_speed_;
    return o;
  }

 private:
  struct Sample {
    std::array<double, path_jog_axes> q{};
    double cap = std::numeric_limits<double>::infinity();
  };
  struct SegmentLimit {
    double speed = 0, acceleration = 0, jerk = 0;
    // Bernstein derivative envelopes. Upward-rounded floats keep the fixed
    // ring compact; scalar limits and proofs use these same conservative
    // values. Six normalized bounds accept the common case without an
    // axis loop. A failed aggregate bound is refined per axis.
    std::array<float, path_jog_axes> q1{}, q2{}, q3{};
    double cv = 0, ca1 = 0, ca2 = 0, cj1 = 0, cj2 = 0, cj3 = 0;
    PathGoverning reason = PathGoverning::None;
  };

  static bool valid_config(const PathJogConfig& c) noexcept {
    if (!c.mask || (c.mask & ~jog_session_axis_mask) || !c.cycle_ns || (c.max_cycle_ns && c.max_cycle_ns < c.cycle_ns) || !std::isfinite(c.jerk) ||
        !(c.jerk > 0) || !std::isfinite(c.max_sample_step_s) || !(c.max_sample_step_s > 0))
      return false;
    for (uint32_t axis = 0; axis < path_jog_axes; ++axis) {
      if (!(c.mask & (1u << axis))) continue;
      const auto& a = c.axes[axis];
      if (!std::isfinite(a.min_q) || !std::isfinite(a.max_q) || !(a.min_q < a.max_q) ||
          !std::isfinite(a.velocity) || !(a.velocity > 0) || !std::isfinite(a.acceleration) ||
          !(a.acceleration > 0) || !std::isfinite(a.start_tolerance) || a.start_tolerance < 0)
        return false;
    }
    return true;
  }

  PathStopReason check_sample(const double* sample) const noexcept {
    for (uint32_t axis = 0; axis < path_jog_axes; ++axis) {
      if (!(config_.mask & (1u << axis))) continue;
      const auto& limits = config_.axes[axis];
      if (sample[axis] < limits.min_q || sample[axis] > limits.max_q) return PathStopReason::PositionLimit;
      if (received_ > 0) {
        const double step = std::abs(sample[axis] - ring_[(received_ - 1) % path_jog_capacity].q[axis]);
        if (step > limits.velocity * config_.max_sample_step_s) return PathStopReason::StepLimit;
      }
    }
    return PathStopReason::None;
  }

  const Sample& at(uint64_t index) const noexcept { return ring_[index % path_jog_capacity]; }

  // Slope and curvature at knot k in sample-index units. Interior knots use
  // fourth-order central differences: with second-order slopes the quintic's
  // third derivative overshot the curve's by ~10x (slope error q'''/6 times
  // the quintic's coefficient gain), which would have throttled jerk-governed
  // speed by ~2x. Knots next to an end use second-order central differences,
  // the end knots second-order one-sided ones, and a two-sample path is a
  // straight segment. A knot's rule depends only on k and the final knot, and
  // the final knot is fixed before any segment that uses it is validated, so
  // playback always evaluates the curve that was validated. A native
  // truncation keeps the dropped successors for the interior rule.
  void knot(uint64_t k, uint32_t axis, double& slope, double& curvature) const noexcept {
    if (final_knot_ == 1) { slope = at(1).q[axis] - at(0).q[axis]; curvature = 0; return; }
    if (k == 0) {
      const double q0 = at(0).q[axis], q1 = at(1).q[axis], q2 = at(2).q[axis];
      slope = (-3 * q0 + 4 * q1 - q2) * .5;
      curvature = q0 - 2 * q1 + q2;
    } else if (k == final_knot_) {
      const double q0 = at(k).q[axis], q1 = at(k - 1).q[axis], q2 = at(k - 2).q[axis];
      slope = (3 * q0 - 4 * q1 + q2) * .5;
      curvature = q0 - 2 * q1 + q2;
    } else if (k == 1 || k + 1 == final_knot_) {
      const double qm = at(k - 1).q[axis], q0 = at(k).q[axis], qp = at(k + 1).q[axis];
      slope = (qp - qm) * .5;
      curvature = qp - 2 * q0 + qm;
    } else {
      const double qm2 = at(k - 2).q[axis], qm = at(k - 1).q[axis], q0 = at(k).q[axis];
      const double qp = at(k + 1).q[axis], qp2 = at(k + 2).q[axis];
      slope = (qm2 - qp2 + 8 * (qp - qm)) / 12;
      curvature = (-qp2 + 16 * qp - 30 * q0 + 16 * qm - qm2) / 12;
    }
  }

  path_jog_detail::Quintic segment_curve(uint64_t k, uint32_t axis) const noexcept {
    double m0, k0, m1, k1;
    knot(k, axis, m0, k0);
    knot(k + 1, axis, m1, k1);
    path_jog_detail::Quintic curve;
    curve.set(at(k).q[axis], at(k + 1).q[axis], m0, m1, k0, k1);
    return curve;
  }

  // Fix segment k's limits once both knots are complete. A segment whose
  // interpolant leaves the position interval ends the path at its first knot.
  void complete_segment(uint64_t k) noexcept {
    using namespace path_jog_detail;
    const double vmax = max_speed_;
    SegmentLimit limit;
    limit.speed = std::numeric_limits<double>::infinity();
    double q1[path_jog_axes]{}, q2[path_jog_axes]{}, q3[path_jog_axes]{};
    for (uint32_t axis = 0; axis < path_jog_axes; ++axis) {
      if (!(config_.mask & (1u << axis))) continue;
      const auto curve = segment_curve(k, axis);
      double low, high;
      bernstein_range(curve.c, 5, low, high);
      const auto& limits = config_.axes[axis];
      if (curve.p0 + low < limits.min_q || curve.p0 + high > limits.max_q) {
        truncate_at(k, PathStopReason::PositionLimit);
        return;
      }
      double d1[5], d2[4], d3[3];
      curve.derivative_coefficients(d1, d2, d3);
      q1[axis] = bernstein_abs_bound(d1, 4);
      q2[axis] = bernstein_abs_bound(d2, 3);
      q3[axis] = bernstein_abs_bound(d3, 2);
      const auto upper_float = [](double x) {
        return x == 0 ? 0.0f : std::nextafter(float(x), std::numeric_limits<float>::infinity());
      };
      limit.q1[axis] = upper_float(q1[axis]);
      limit.q2[axis] = upper_float(q2[axis]);
      limit.q3[axis] = upper_float(q3[axis]);
      q1[axis] = limit.q1[axis];
      q2[axis] = limit.q2[axis];
      q3[axis] = limit.q3[axis];
      const double V = limits.velocity * limit_reserve, A = limits.acceleration * limit_reserve;
      const double J = config_.jerk * limit_reserve;
      limit.cv = std::max(limit.cv, q1[axis] / V);
      limit.ca1 = std::max(limit.ca1, q1[axis] / A);
      limit.ca2 = std::max(limit.ca2, q2[axis] / A);
      limit.cj1 = std::max(limit.cj1, q1[axis] / J);
      limit.cj2 = std::max(limit.cj2, q2[axis] / J);
      limit.cj3 = std::max(limit.cj3, q3[axis] / J);
      const auto lower = [&](double value, PathGoverning reason) {
        if (value < limit.speed) { limit.speed = value; limit.reason = reason; }
      };
      if (q1[axis] > 0) lower(V / q1[axis], PathGoverning::Velocity);
      if (q2[axis] > 0) lower(std::sqrt(A * .5 / q2[axis]), PathGoverning::Curvature);
      if (q3[axis] > 0) lower(std::cbrt(J / 3 / q3[axis]), PathGoverning::Jerk);
    }
    // The split below is chosen at the kinematic speed limit, not at an
    // application cap: a cap (e.g. 50 mm/s near a singularity) must not shrink
    // the segment's acceleration, which made braking onto it inconsistent.
    const double kinematic = limit.speed;
    const PathGoverning kinematic_reason = limit.reason;
    // Scalar acceleration a and jerk j(a) from the budget left at the session
    // ceiling vr: per axis Q1 a + Q2 vr^2 <= A and Q1 j + 3 Q2 vr a + Q3 vr^3
    // <= J. A larger a costs jerk through the coupling term, so a is chosen
    // to minimise the jerk-limited stopping distance from vr (golden section;
    // any a in range satisfies the budget, the search only picks the best
    // split). Harness, recorded s2104#2 release at 393 mm/s: fixed half/half
    // split 42.6 mm, this search 42.3 mm (design nominal 40 mm).
    const double vr = std::min(vmax, kinematic);
    double upper = std::numeric_limits<double>::infinity();
    bool moving = false;
    for (uint32_t axis = 0; axis < path_jog_axes; ++axis) {
      if (!(config_.mask & (1u << axis)) || !(q1[axis] > 0)) continue;
      moving = true;
      const double A = config_.axes[axis].acceleration * limit_reserve;
      const double J = config_.jerk * limit_reserve;
      upper = std::min(upper, (A - q2[axis] * vr * vr) / q1[axis]);
      // Keep j(a) at least a thousandth of its zero-acceleration value.
      if (q2[axis] > 0 && vr > 0)
        upper = std::min(upper, (J - q3[axis] * vr * vr * vr) * (1 - 1e-3) / (3 * q2[axis] * vr));
    }
    const auto jerk_at = [&](double a) {
      double j = std::numeric_limits<double>::infinity();
      for (uint32_t axis = 0; axis < path_jog_axes; ++axis) {
        if (!(config_.mask & (1u << axis)) || !(q1[axis] > 0)) continue;
        const double J = config_.jerk * limit_reserve;
        j = std::min(j, (J - 3 * q2[axis] * vr * a - q3[axis] * vr * vr * vr) / q1[axis]);
      }
      return j;
    };
    double acceleration = std::numeric_limits<double>::infinity(), jerk = acceleration;
    if (moving && upper > 0) {
      acceleration = upper;
      if (vr > 0) {
        const auto stop_distance = [&](double a) {
          const auto b = jerk_limited_brake(vr, 0, 0, a, jerk_at(a));
          return b.valid ? b.distance : std::numeric_limits<double>::infinity();
        };
        constexpr double golden = 0.6180339887498949;
        double lo = upper * 1e-3, hi = upper;
        double x1 = hi - golden * (hi - lo), x2 = lo + golden * (hi - lo);
        double f1 = stop_distance(x1), f2 = stop_distance(x2);
        for (int i = 0; i < 24; ++i) {
          if (f1 <= f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - golden * (hi - lo); f1 = stop_distance(x1); }
          else { lo = x1; x1 = x2; f1 = f2; x2 = lo + golden * (hi - lo); f2 = stop_distance(x2); }
        }
        acceleration = f1 <= f2 ? x1 : x2;
        if (stop_distance(upper) <= std::min(f1, f2)) acceleration = upper;
      }
      jerk = jerk_at(acceleration);
    }
    limit.acceleration = acceleration;
    limit.jerk = jerk;
    const double cap = std::min(at(k).cap, at(k + 1).cap);
    limit.speed = std::min(kinematic, cap);
    limit.reason = cap < kinematic ? PathGoverning::Cap : kinematic_reason;
    limits_[k % path_jog_capacity] = limit;
    playable_ = k + 1;
  }

  void truncate_at(uint64_t knot_index, PathStopReason reason) noexcept {
    ended_ = truncated_ = true;
    end_stop_ = reason;
    end_reason_ = 0;
    playable_ = std::min(playable_, knot_index);
  }

  // The path ends at the last sample received: its derivatives become
  // one-sided, which completes the final segment (which may itself truncate).
  void finish_path(PathStopReason reason, uint32_t code) noexcept {
    ended_ = true;
    end_stop_ = reason;
    end_reason_ = code;
    if (received_ >= 2) {
      final_knot_ = received_ - 1;
      while (!truncated_ && next_segment_ < final_knot_) complete_segment(next_segment_++);
    }
  }

  void request_stop(PathStopReason reason, uint64_t now) noexcept {
    if (!open_ || commanded_stop_) return;
    commanded_stop_ = true;
    stop_ = reason;
    stop_window_end_ = playable_;
    // This is the very profile the next cycle will execute. A newly appended
    // tighter segment must not shorten a previously certified stop deadline.
    double seconds = 0;
    for (int phase = fallback_.phase; phase < 3; ++phase)
      seconds += fallback_.duration[phase];
    const long double ns = std::ceil(static_cast<long double>(seconds) * 1e9L) +
                           3.0L * (config_.max_cycle_ns ? config_.max_cycle_ns : config_.cycle_ns);
    stop_bound_ns_ = ns >= static_cast<long double>(UINT64_MAX - now) ? UINT64_MAX : now + static_cast<uint64_t>(ns);
  }

  void close(PathStopReason reason) noexcept {
    open_ = false;
    closed_reason_ = reason;
    commanded_stop_ = false;
  }

  PathJogCycle fail(PathJogCycle out) noexcept {
    close(PathStopReason::Fault);
    out.fault = true;
    out.closed = true;
    return out;
  }

  uint64_t segment_at(double s) const noexcept {
    return s <= 0 ? 0 : static_cast<uint64_t>(std::floor(s));
  }

  // Playback stops at the playable end; while stopping, at the playable end of
  // the stop request, so samples appended during a stop cannot stretch it.
  uint64_t window_end() const noexcept {
    return commanded_stop_ ? std::min(stop_window_end_, playable_) : playable_;
  }

  // The interval braking margins are planned for: this one, or the longest
  // the machine allows, whichever is longer.
  double plan_dt(double dt) const noexcept {
    return std::max(dt, double(config_.max_cycle_ns ? config_.max_cycle_ns : config_.cycle_ns) / 1e9);
  }

  // Candidate tracking uses the present segment. Its safety authority is
  // the retained, geometry-certified stop profile below, not a future window
  // whose changing A/J could invalidate the current braking state.
  void update_tracker_limits() noexcept {
    const auto end = window_end();
    if (!end) return;
    const auto &l = limits_[std::min(segment_at(s_), end - 1) % path_jog_capacity];
    a_lim_ = l.acceleration;
    j_lim_ = l.jerk;
  }
  // Jerk that moves (v, a) onto the time-optimal jerk-limited approach to
  // target, within |jerk| <= j_lim and |acceleration| <= a_lim. The approach
  // curve is v + a|a|/(2j) = target; following it lands on target with zero
  // acceleration, and a zero target never reverses.
  double track(double target, double dt) const noexcept {
    const double j = j_lim_, amax = a_lim_;
    double lo = -j, hi = j;
    if (a_ + hi * dt > amax) hi = std::max(-j, (amax - a_) / dt);
    if (a_ + lo * dt < -amax) lo = std::min(j, (-amax - a_) / dt);
    if (lo > hi) lo = hi;
    const auto f = [&](double x) {
      const double a1 = a_ + x * dt;
      return v_ + a_ * dt + x * dt * dt * .5 + a1 * std::abs(a1) / (2 * j) - target;
    };
    if (f(hi) <= 0) return hi;
    if (f(lo) >= 0) return lo;
    for (int i = 0; i < 64; ++i) {
      const double mid = .5 * (lo + hi);
      (f(mid) > 0 ? hi : lo) = mid;
    }
    return lo;
  }

  // Every accepted step carries a concrete stop over immutable completed
  // geometry. Appending a tighter segment cannot invalidate this profile:
  // when a new candidate cannot be proved, execute the retained phases,
  // splitting at their exact time boundaries, rather than a best-effort jerk.
  struct Certificate {
    double duration[3]{}, jerk[3]{};
    double end = 0;
    int phase = 0;
  } fallback_;

  mutable PathGoverning certificate_binding_ = PathGoverning::None;
  mutable uint32_t proof_work_left_ = path_jog_proof_work_budget;
  bool spend_proof_work() const noexcept {
    if (!proof_work_left_) return false;
    --proof_work_left_;
    return true;
  }
  bool phase_limits(const SegmentLimit &l, double v, double a, double j) const noexcept {
    if (!spend_proof_work()) return false;
    if (v > l.speed * path_jog_detail::speed_rounding) {
      certificate_binding_ = l.reason;
      return false;
    }
    if (l.cv * v <= 1 && l.ca1 * a + l.ca2 * v * v <= 1 && l.cj1 * j + 3 * l.cj2 * v * a + l.cj3 * v * v * v <= 1)
      return true;
    for (uint32_t k = 0; k < path_jog_axes; ++k)
      if (config_.mask & (1u << k)) {
        if (!spend_proof_work()) return false;
        if (l.q1[k] * v > config_.axes[k].velocity * path_jog_detail::limit_reserve ||
            l.q1[k] * a + l.q2[k] * v * v > config_.axes[k].acceleration * path_jog_detail::limit_reserve ||
            l.q1[k] * j + 3 * l.q2[k] * v * a + l.q3[k] * v * v * v > config_.jerk * path_jog_detail::limit_reserve) {
          certificate_binding_ = PathGoverning::Jerk;
          return false;
        }
      }
    return true;
  }

  // A whole-phase envelope is cheap and normally sufficient. If it is too
  // conservative for one segment, bracket the actual entry/exit times. Keep
  // the outer brackets after a fixed 12 steps: incomplete convergence may
  // reject a safe candidate but cannot admit an unproved one.
  bool phase_segment_limits(const SegmentLimit &limit, uint64_t k, double s, double v, double a, double j, double t,
                            double end_s, double peak, double max_a) const noexcept {
    if (phase_limits(limit, peak, max_a, std::abs(j))) return true;
    const auto position = [&](double x) { return s + v * x + a * x * x * .5 + j * x * x * x / 6; };
    const auto time_bound = [&](double pos, bool upper) {
      double lo = 0, hi = t;
      for (int n = 0; n < 12; ++n) {
        if (!spend_proof_work()) break;
        double mid = (lo + hi) * .5;
        if (position(mid) < pos)
          lo = mid;
        else
          hi = mid;
      }
      return upper ? hi : lo;
    };
    const double t0 = double(k) <= s ? 0 : time_bound(double(k), false);
    const double t1 = double(k + 1) >= end_s ? t : time_bound(double(k + 1), true);
    const double v0 = v + a * t0 + j * t0 * t0 * .5, v1 = v + a * t1 + j * t1 * t1 * .5;
    const double a0 = a + j * t0, a1 = a + j * t1;
    double local_peak = std::max(v0, v1);
    if (a0 > 0 && a1 < 0 && j < 0) local_peak = std::max(local_peak, v + a * a / (2 * -j));
    return phase_limits(limit, local_peak, std::max(std::abs(a0), std::abs(a1)), std::abs(j));
  }

  bool make_certificate(double s, double v, double a, Certificate &result) const noexcept {
    const uint64_t end = window_end();
    if (s > double(end) || v < 0 || !std::isfinite(s + v + a)) return false;
    result = {};
    result.end = s;
    if (v == 0 && a == 0) return true;
    if (s >= double(end)) return false;
    const uint64_t first = segment_at(s);
    double A = limits_[first % path_jog_capacity].acceleration;
    double J = limits_[first % path_jog_capacity].jerk;
    // Fixed construction budget; failure leaves the existing stop intact.
    // Each pass visits each segment in at most three constant-jerk phases.
    for (uint64_t pass = 0; pass < 2; ++pass) {
      if (!(A > 0) || !(J > 0) || !std::isfinite(A + J) || a < -A) return false;
      if (a < 0 && v < a * a / (2 * J)) return false;
      double ap = std::sqrt(a * a * .5 + J * v), hold = 0;
      if (ap > A) {
        ap = A;
        hold = (v + a * a / (2 * J) - ap * ap / J) / ap;
      }
      Certificate candidate;
      candidate.duration[0] = (a + ap) / J;
      candidate.duration[1] = hold;
      candidate.duration[2] = ap / J;
      candidate.jerk[0] = -J;
      candidate.jerk[1] = 0;
      candidate.jerk[2] = J;
      double ss = s, vv = v, aa = a;
      bool valid = true;
      double newA = A, newJ = J;
      for (int n = 0; n < 3; ++n) {
        const double t = candidate.duration[n], j = candidate.jerk[n];
        if (t < 0) return false;
        const double sn = ss + vv * t + aa * t * t * .5 + j * t * t * t / 6;
        const double vn = vv + aa * t + j * t * t * .5, an = aa + j * t;
        if (!std::isfinite(sn) || sn > double(end)) return false;
        double peak = std::max(vv, vn);
        if (aa > 0 && an < 0 && j != 0) peak = std::max(peak, vv + aa * aa / (2 * std::abs(j)));
        const double maxa = std::max(std::abs(aa), std::abs(an));
        const uint64_t reached = std::min(segment_at(sn), end - 1);
        for (uint64_t k = segment_at(ss); k <= reached && k < end; ++k) {
          const auto &l = limits_[k % path_jog_capacity];
          if (!phase_segment_limits(l, k, ss, vv, aa, j, t, sn, peak, maxa)) {
            if (!proof_work_left_) return false;
            valid = false;
            newA = std::min(newA, l.acceleration);
            newJ = std::min(newJ, l.jerk);
          }
        }
        ss = sn;
        vv = vn;
        aa = an;
      }
      candidate.end = ss;
      if (valid) {
        result = candidate;
        return true;
      }
      if (A == newA && J == newJ) return false;
      A = newA;
      J = newJ;
    }
    return false;
  }

  bool certify_step(double js, double dt, Certificate &result) const noexcept {
    const double s1 = s_ + v_ * dt + a_ * dt * dt * .5 + js * dt * dt * dt / 6;
    const double v1 = v_ + a_ * dt + js * dt * dt * .5, a1 = a_ + js * dt;
    if (!std::isfinite(js) || !std::isfinite(s1) || !std::isfinite(v1) || !std::isfinite(a1) || s1 < s_ ||
        s1 > double(window_end()) || v1 < 0)
      return false;
    if (a_ < 0 && a1 > 0 && js > 0 && v_ - a_ * a_ / (2 * js) < 0) return false;
    double peak = std::max(v_, v1);
    if (a_ > 0 && a1 < 0 && js != 0) peak = std::max(peak, v_ + a_ * a_ / (2 * std::abs(js)));
    const uint64_t end = window_end();
    if (!end) return s1 == 0 && v1 == 0 && a1 == 0;
    const uint64_t last = std::min(segment_at(s1), end - 1);
    for (uint64_t k = segment_at(s_); k <= last && k < end; ++k) {
      const auto &l = limits_[k % path_jog_capacity];
      if (!phase_limits(l, peak, std::max(std::abs(a_), std::abs(a1)), std::abs(js))) return false;
    }
    return make_certificate(s1, v1, a1, result);
  }

  void follow_certificate(double dt) noexcept {
    while (dt > 0 && fallback_.phase < 3) {
      const int n = fallback_.phase;
      if (!(fallback_.duration[n] > 0)) {
        ++fallback_.phase;
        continue;
      }
      const double t = std::min(dt, fallback_.duration[n]), j = fallback_.jerk[n];
      s_ += v_ * t + a_ * t * t * .5 + j * t * t * t / 6;
      v_ += a_ * t + j * t * t * .5;
      a_ += j * t;
      dt -= t;
      fallback_.duration[n] -= t;
      if (!(fallback_.duration[n] > 0)) ++fallback_.phase;
    }
    if (fallback_.phase == 3) {
      s_ = fallback_.end;
      v_ = a_ = 0;
    }
  }

  void certificate_step(double target, double dt) noexcept {
    // All candidate proofs share this budget. Exhaustion cannot install a
    // partially checked profile: keep the last proved candidate, or execute
    // the previously retained stop. The cap forecast remains one ring scan.
    proof_work_left_ = path_jog_proof_work_budget;
    governing_ = commanded_stop_ ? PathGoverning::Stopping : PathGoverning::Command;
    if (target <= 0) {
      follow_certificate(dt);
      return;
    }
    // Reach the next tighter cap with zero acceleration. Waiting until the
    // stop proof binds can use all velocity reserve to stop just before the
    // cap, causing an unnecessary stop/restart even on preloaded geometry.
    double running = target, ahead_a = a_lim_, ahead_j = j_lim_;
    for (uint64_t k = segment_at(s_); k < window_end(); ++k) {
      const auto &l = limits_[k % path_jog_capacity];
      ahead_a = std::min(ahead_a, l.acceleration);
      ahead_j = std::min(ahead_j, l.jerk);
      if (l.speed < running) {
        const auto b = jerk_limited_brake(v_, a_, l.speed, ahead_a, ahead_j);
        if (b.valid && b.distance + 2 * v_ * plan_dt(dt) >= double(k) - s_) {
          target = std::min(target, l.speed);
          governing_ = l.reason;
        }
        running = l.speed;
      }
    }
    // Scale the finite candidate search to the remaining distance. Without
    // this bound, a tiny final remainder can fall below the search's jerk
    // resolution and repeatedly choose zero from rest.
    const double position_jerk = 6 * (double(window_end()) - s_ - v_ * dt - a_ * dt * dt * .5) / (dt * dt * dt);
    const double desired = std::min(track(target, dt), position_jerk);
    double chosen = desired;
    Certificate next;
    bool valid = certify_step(desired, dt, next);
    if (!valid) {
      if (!commanded_stop_ && proof_work_left_) governing_ = certificate_binding_;
      const double brake = track(0, dt);
      Certificate lower;
      if (certify_step(brake, dt, lower)) {
        double lo = brake, hi = desired;
        next = lower;
        // Every retained candidate is independently proved. This bounded
        // search need not assume global monotonicity of the feasible set.
        for (int n = 0; n < 16 && proof_work_left_; ++n) {
          const double mid = .5 * (lo + hi);
          Certificate test;
          if (certify_step(mid, dt, test)) {
            lo = mid;
            next = test;
          } else
            hi = mid;
        }
        chosen = lo;
        valid = true;
      }
    }
    if (valid) {
      s_ += v_ * dt + a_ * dt * dt * .5 + chosen * dt * dt * dt / 6;
      v_ += a_ * dt + chosen * dt * dt * .5;
      a_ += chosen * dt;
      fallback_ = next;
      at_end_ = playable_ == 0 || s_ >= double(playable_) - 1e-9;
    } else {
      follow_certificate(dt);
      at_end_ = fallback_.end >= double(window_end()) - 1e-9;
      if (!commanded_stop_ && at_end_) governing_ = ended_ ? PathGoverning::PathEnd : PathGoverning::BufferEnd;
    }
  }
  void evaluate() noexcept {
    if (received_ == 0) return;
    if (playable_ == 0) {
      for (uint32_t axis = 0; axis < path_jog_axes; ++axis)
        if (config_.mask & (1u << axis)) { q_out_[axis] = at(0).q[axis]; v_out_[axis] = 0; }
      return;
    }
    uint64_t k = segment_at(s_);
    if (k >= playable_) k = playable_ - 1;
    const double u = s_ - double(k);
    for (uint32_t axis = 0; axis < path_jog_axes; ++axis) {
      if (!(config_.mask & (1u << axis))) continue;
      const auto curve = segment_curve(k, axis);
      q_out_[axis] = u >= 1 ? at(k + 1).q[axis] : u <= 0 ? at(k).q[axis] : curve.position(u);
      v_out_[axis] = curve.slope(std::clamp(u, 0.0, 1.0)) * v_;
    }
  }

  PathJogConfig config_{};
  double spacing_ = 1, max_speed_ = 0;
  std::array<double, path_jog_axes> held_{}, q_out_{}, v_out_{};
  std::array<Sample, path_jog_capacity> ring_{};
  std::array<SegmentLimit, path_jog_capacity> limits_{};
  uint64_t received_ = 0, retained_ = 0, playable_ = 0, final_knot_ = UINT64_MAX, next_segment_ = 0;
  bool truncated_ = false;
  double s_ = 0, v_ = 0, a_ = 0, target_ = 0;
  double a_lim_ = 1, j_lim_ = 1;
  uint64_t speed_sequence_ = 0, speed_origin_ = 0, speed_deadline_ns_ = 0;
  uint64_t last_ns_ = 0, stop_bound_ns_ = UINT64_MAX, stop_window_end_ = 0, underruns_ = 0, stopped_playable_ = 0;
  uint32_t end_reason_ = 0;
  PathStopReason stop_ = PathStopReason::None, end_stop_ = PathStopReason::PathEnd;
  PathStopReason closed_reason_ = PathStopReason::None;
  PathGoverning governing_ = PathGoverning::None;
  bool open_ = false, ended_ = false, commanded_stop_ = false, has_speed_ = false;
  bool first_cycle_ = true, at_end_ = false, stopped_at_end_ = false, moved_ = false;
};

}  // namespace rosie::rtcore
