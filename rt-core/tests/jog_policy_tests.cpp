// Deterministic command-ramp qualification; physical stopping is unverified.
#define ROSIE_RT_SIMULATION 1
#include "engine/jog_lane.hpp"
#include "output_policy.hpp"
#include "ipc_v1.hpp"
#include <cassert>
#include <iostream>
#include <vector>

using namespace rosie::rtcore;
namespace wire = rosie::protocol;

struct JogFixture {
  alignas(8) std::array<uint8_t, fast_control_region_bytes> bytes{};
  FastControlRegion region;
  FastControlRegion::JogControlSlot controls;
  FastControlRegion::JogUpdateSlot updates;
  FastControlRegion::JogObservationSlot observations;
  JogLane lane;
  JogSessionContext context{};
  std::array<JogAxis, jog_session_axes> axes{};
  std::array<double, jog_session_axes> limits{};
  JogTiming timing{};
  uint64_t cycle_ns;
  wire::JogControl command{};
  wire::JogUpdate update{};

  explicit JogFixture(uint64_t period) : cycle_ns(period) {
    wire::FastHeader header{};
    header.magic = fast_control_magic;
    header.version_major = wire::version_major;
    header.version_minor = wire::version_minor;
    header.region_bytes = fast_control_region_bytes;
    header.daemon_incarnation[0] = header.configuration_digest_lo[0] = 1;
    header.connection_id = 1;
    header.cycle_ns = period;
    header.configured_mask = 3;
    assert(FastControlRegion::initialize(bytes.data(), bytes.size(), header, region) == FastControlStatus::Ok);
    assert(FastControlRegion::JogControlSlot::bind(bytes.data() + fast_control_jog_control_offset,
        FastControlRegion::JogControlSlot::region_bytes, controls) == SharedWordsStatus::Ok);
    assert(FastControlRegion::JogUpdateSlot::bind(bytes.data() + fast_control_jog_update_offset,
        FastControlRegion::JogUpdateSlot::region_bytes, updates) == SharedWordsStatus::Ok);
    assert(FastControlRegion::JogObservationSlot::bind(bytes.data() + fast_control_jog_observation_offset,
        FastControlRegion::JogObservationSlot::region_bytes, observations) == SharedWordsStatus::Ok);
    context.identity = {1, 1, {1}, 1, 1, 1, 1};
    context.now_ns = 1'000'000'000;
    context.grant_deadline_ns = 3'000'000'000;
    context.max_input_age_ns = context.max_first_input_wait_ns = 250'000'000;
    context.configured_mask = context.allowed_mask = context.ready_mask = 3;
    context.authority_valid = context.clock_mapping_valid = context.armed = true;
    assert(timing.configure(period));
    for (unsigned axis = 0; axis < 2; ++axis) {
      limits[axis] = 2000;
      lane.configure_axis(axis, axes[axis], timing, 10'000, period);
    }
    command.ticket = command.connection_id = command.native_generation = 1;
    command.capability_id[0] = 1;
    command.app_grant_generation = command.configuration_epoch = command.home_epoch = command.clock_mapping_generation = 1;
    command.origin_host_ns = context.now_ns;
    command.deadline_host_ns = context.now_ns + 200'000'000;
    command.axis_mask = 3;
    command.operation = wire::jog_operation_begin;
    control();
    assert(!lane.cycle(&region, context, limits));
    update.conn_id = update.native_generation = update.app_grant_generation = 1;
    update.capability_id[0] = 1;
    update.config_epoch = update.home_epoch = update.clock_mapping_generation = 1;
    update.jog_generation = update.source_sequence = 1;
    update.axis_mask = 3;
    update.earliest_origin_host_ns = context.now_ns;
    update.immutable_deadline_host_ns = context.now_ns + 200'000'000;
    update.velocity[0] = 2000;
    update.velocity[1] = -1000;
    input();
    assert(!lane.cycle(&region, context, limits));
  }
  void control() {
    assert(controls.publish(shared_words_from_le<FastControlRegion::JogControlSlot::payload_words>(
        wire::encode_jog_control(command))).status == SharedWordsStatus::Ok);
  }
  void input() {
    assert(updates.publish(shared_words_from_le<FastControlRegion::JogUpdateSlot::payload_words>(
        wire::encode_jog_update(update))).status == SharedWordsStatus::Ok);
  }
  wire::JogObservation observe() {
    assert(lane.publish(region, context.identity, context.now_ns).status == SharedWordsStatus::Ok);
    FastControlRegion::JogObservationSlot::Words words{};
    assert(observations.read(words).status == SharedWordsStatus::Ok);
    const auto raw = shared_words_to_le(words);
    wire::JogObservation value{};
    assert(wire::decode_jog_observation(raw.data(), raw.size(), value));
    return value;
  }
  void end() {
    ++command.ticket;
    command.operation = wire::jog_operation_end;
    command.jog_generation = 1;
    command.axis_mask = 0;
    command.origin_host_ns = command.deadline_host_ns = 0;
    control();
  }
};

static void ramp(uint64_t period, bool expiry) {
  JogFixture f(period);
  f.axes[0].velocity = 2000;
  f.axes[1].velocity = -1000;
  if (expiry) f.context.now_ns = f.update.immutable_deadline_host_ns;
  else f.end();
  const auto started = f.context.now_ns;
  assert(!f.lane.cycle(&f.region, f.context, f.limits));
  assert(f.observe().state_reason == wire::jog_reason_ramping);
  assert(f.observe().input_deadline_host_ns == f.update.immutable_deadline_host_ns);
  // A fresh, otherwise valid late update cannot reopen the cancelled generation.
  ++f.update.source_sequence;
  f.update.earliest_origin_host_ns = f.context.now_ns;
  f.update.immutable_deadline_host_ns = f.context.now_ns + 200'000'000;
  f.input();
  assert(!f.lane.cycle(&f.region, f.context, f.limits));
  assert(f.observe().update_reason == wire::jog_reason_closed);
  while (f.lane.ramping()) {
    for (unsigned axis = 0; axis < 2; ++axis) {
      const auto previous = f.axes[axis].velocity;
      assert(f.lane.advance(axis, f.axes[axis], 9999, f.axes[axis].position,
          10'000, 1000, double(period) / 1e9, 0));
      assert(std::abs(f.axes[axis].velocity) <= std::abs(previous));
      // Absolute FP tolerance: 0.000000001 counts/s.
      assert(std::abs(previous - f.axes[axis].velocity) <= 10'000 * double(period) / 1e9 + 1e-9);
    }
    f.context.now_ns += period;
    assert(f.context.now_ns - started <= 200'000'000 + period);
    assert(!f.lane.cycle(&f.region, f.context, f.limits));
  }
  assert(f.context.now_ns - started >= 200'000'000);
  assert(f.context.now_ns - started <= 200'000'000 + period);
  assert(f.observe().state_reason == (expiry ? wire::jog_reason_expired : wire::jog_reason_closed));
  assert(f.observe().open == 0 && f.observe().has_input == 0);
  assert(f.axes[0].velocity == 0 && f.axes[1].velocity == 0);
  // Absolute arithmetic allowance: 0.00000001 counts at the exact hold position.
  assert(std::abs(f.axes[0].position - 200) <= 1e-8);
  const auto held = f.axes[0].position;
  for (unsigned i = 0; i < 10; ++i) {
    assert(f.lane.advance(0, f.axes[0], 0, held, 10'000, 1000, double(period) / 1e9, 0));
    assert(f.axes[0].position == held);
  }
}

// The reported J9 failure: 179.5 degree bound, 180 deg/s cap, 0.5 rad/s²
// acceleration. Run the real integrator and final wire guard with exact-follow
// feedback; both held input and cancellation must retain Home and enable.
static void position_limit(int direction, int sign, int release_cycle) {
  const double scale = 131072.0 / (2 * std::acos(-1));
  const double offset = 137;
  AxisOutputConfig config{};
  config.counts_per_unit = scale; config.sign = sign;
  config.max_profile_velocity = 65536;
  config.limits = {-3.132866007329821, 3.132866007329821, 3.14159265, true, true};
  config.limits.jog_acceleration = .5;
  config.limits.following_error = .04;
  config.limits.following_error_timeout_ns = 100000000;
  const auto bounds = command_limits(config.limits, scale * sign, offset);
  std::array<JogAxis, jog_session_axes> axes{};
  axes[0].reset(-offset);
  JogLane lane;
  AxisOutputState state{};
  state.held_target = -offset; state.held_target_valid = true;
  HaltRamp halt;
  bool cancelling = false;
  const double requested = direction * sign * .75 * 3.14159265 * scale;
  const auto cycle = [&](double velocity, unsigned tick) {
    const auto previous = state.held_target;
    lane.set_limit_feedback(0, config.limits, previous, scale * sign, offset);
    std::array<double, jog_session_axes> request{};
    request[0] = velocity;
    const double before = axes[0].velocity;
    lane.govern(1, axes, request, .001);
    const uint64_t now = 1000000000ULL + uint64_t(tick) * 1000000;
    if (cancelling) assert(halt.advance(axes[0], axes[0].position, .02 * scale, now));
    else assert(lane.advance(0, axes[0], request[0], axes[0].position,
        .5 * scale, .02 * scale, .001, 0));
    assert(std::abs(axes[0].velocity - before) <= .5 * scale * .001 + 1e-8);
    AxisOutputInput input{};
    input.statusword = 0x27; input.home_offset = offset;
    input.feedback_position = previous;
    input.feedback_fresh = input.configuration_current = input.config_verified = input.home_valid = true;
    input.armed = input.enabled = input.setpoint_present = input.jog_active = true;
    input.previous_target_valid = true; input.previous_target = previous;
    input.now_ns = now; input.previous_cycle_ns = now - 1000000;
    input.target_position = std::llround(axes[0].position);
    input.target_velocity = axes[0].velocity;
    auto out = evaluate_axis_output(config, input, state);
    assert(!out.fault && !out.invalidate_home && out.controlword == 0x0f);
    assert(input.target_position >= bounds.min_q && input.target_position <= bounds.max_q);
    state = out.next;
  };
  for (unsigned tick = 1; tick <= 14000; ++tick) {
    if (int(tick) == release_cycle) {
      cancelling = true;
      assert(halt.begin(axes[0], .5 * scale, 1000000, 1000000000ULL + uint64_t(tick) * 1000000));
    }
    cycle(requested, tick);
  }
  assert(std::abs(axes[0].velocity) < 1e-6);
  if (release_cycle < 0) {
    const double endpoint = direction * sign > 0 ? bounds.max_q : bounds.min_q;
    assert(std::abs(axes[0].position - endpoint) <= 2.01); // <= two command counts inside.
  }
  const auto at_limit = axes[0].position;
  cancelling = false;
  for (unsigned tick = 14001; tick <= 15000; ++tick) cycle(-requested, tick);
  assert(direction * sign * (axes[0].position - at_limit) < -1000);
}

// Jerk-limited jog (machine jog_jerk) in the 1430 cell J1 units: 131072 x 100 counts
// per joint revolution, the description's 4*pi rad/s^2 acceleration, jog_jerk
// 150 rad/s^3, and the compiler's window ceil(2 x 4*pi / 150 / 0.001) = 168
// cycles at 1 kHz. Bounds are judged on the commanded positions the drive
// receives, by finite differences, in rad, rad/s^2 and rad/s^3.
namespace smoothing {
const double pi = std::acos(-1), counts_per_rad = 131072.0 * 100 / (2 * pi);
const double acceleration = 4 * pi, jerk = 150, dt = .001;
const uint32_t window = 168;
const uint64_t period = 1'000'000;

struct Bounds { double acceleration = 0, jerk = 0; };
// Peak |second| and |third| differences of commanded positions (counts), in rad.
static Bounds measure(const std::vector<double>& targets, size_t from = 0, size_t to = SIZE_MAX) {
  Bounds peak;
  to = std::min(to, targets.size());
  for (size_t k = std::max<size_t>(from, 3); k < to; ++k) {
    const double v0 = targets[k - 2] - targets[k - 3], v1 = targets[k - 1] - targets[k - 2], v2 = targets[k] - targets[k - 1];
    peak.acceleration = std::max(peak.acceleration, std::abs(v2 - v1) / (dt * dt) / counts_per_rad);
    peak.jerk = std::max(peak.jerk, std::abs(v2 - 2 * v1 + v0) / (dt * dt * dt) / counts_per_rad);
  }
  return peak;
}
// Absolute FP allowances: 1e-6 rad/s^2 and 1e-3 rad/s^3 over ~1e6-count positions.
static void within(const Bounds& b, double max_acceleration, double max_jerk) {
  assert(b.acceleration <= max_acceleration + 1e-6);
  assert(b.jerk <= max_jerk + 1e-3);
}
static bool step(JogAxis& axis, double requested_rad_s) {
  return axis.advance(requested_rad_s * counts_per_rad, axis.position, acceleration * counts_per_rad,
      1e12, dt);
}

static void start_stop_and_reversal() {
  JogSmoother smoother;
  assert(smoother.configure(window, dt));
  JogAxis smoothed{}, plain{};
  smoothed.smoother = &smoother;
  smoothed.reset(1000); plain.reset(1000);
  std::vector<double> targets{1000, 1000, 1000};
  // Start from rest to 0.8 rad/s, cruise, release: steady-state starts and
  // stops change acceleration at a/(window x cycle) = 74.8 rad/s^3 <= 75.
  for (int k = 0; k < 1200; ++k) {
    const double requested = k < 700 ? .8 : 0;
    assert(step(smoothed, requested) && step(plain, requested));
    targets.push_back(smoothed.position);
  }
  within(measure(targets), acceleration, jerk / 2);
  // The smoother changes when the target moves, not where it stops.
  assert(smoothed.velocity == 0 && plain.velocity == 0);
  assert(std::abs(smoothed.position - plain.position) <= 1e-6);
  // Without smoothing the same input reaches full acceleration within two
  // cycles: a/(2 x cycle) = 6283 rad/s^3, 42 times the configured jerk.
  JogAxis raw{};
  raw.reset(0);
  std::vector<double> raw_targets{0, 0, 0};
  for (int k = 0; k < 300; ++k) { assert(step(raw, .8)); raw_targets.push_back(raw.position); }
  assert(measure(raw_targets).jerk > 40 * jerk);

  // Reversals, including mid-acceleration: jerk reaches but never exceeds
  // 2a/(window x cycle) <= 150 rad/s^3; acceleration stays within 4*pi.
  smoothed.reset(0);
  targets.assign(3, 0);
  const double pattern[] = {1.3, -1.3, 1.3, 0, -.4, 1.3, -1.3};
  for (int k = 0; k < 7 * 90 + 600; ++k) {
    const double requested = k < 7 * 90 ? pattern[k / 90] : 0;
    assert(step(smoothed, requested));
    targets.push_back(smoothed.position);
  }
  const auto reversal = measure(targets);
  within(reversal, acceleration, jerk);
  assert(reversal.jerk > jerk / 2);
  assert(smoothed.velocity == 0);
}

// TestJogSmootherHaltBound: Halt/EndJog from 1.2 rad/s. The integrator stops in
// v/a = 95.5 ms; the commanded target settles exactly one window later, inside
// the ramp's bound ceil(v/a) + window x cycle + one cycle.
static void halt_ramp() {
  JogSmoother smoother;
  assert(smoother.configure(window, dt));
  JogAxis axis{};
  axis.smoother = &smoother;
  axis.reset(0);
  std::vector<double> targets{0, 0, 0};
  for (int k = 0; k < 800; ++k) { assert(step(axis, 1.2)); targets.push_back(axis.position); }
  const double raw_stop = axis.integrator_position() + std::pow(1.2 * counts_per_rad, 2) / (2 * acceleration * counts_per_rad);
  HaltRamp ramp;
  uint64_t now = 2'000'000'000;
  assert(ramp.begin(axis, acceleration * counts_per_rad, period, now));
  const uint64_t expected = uint64_t(std::ceil(1.2 / acceleration * 1e9)) + window * period + period;
  assert(ramp.bound_ns == expected);
  const auto started = now;
  while (axis.velocity != 0) {
    assert(ramp.advance(axis, axis.position, 1e12, now));
    targets.push_back(axis.position);
    now += period;
    assert(now - started <= ramp.bound_ns + period);
  }
  within(measure(targets), acceleration, jerk);
  assert(std::abs(axis.position - raw_stop) <= 1e-3);
  // A stopped ramp holds exactly.
  const double held = axis.position;
  for (int k = 0; k < 10; ++k) { assert(ramp.advance(axis, held, 1e12, now)); now += period; assert(axis.position == held); }

  // A Halt that takes over a trajectory at 1.2 rad/s seeds steady motion: no
  // position or velocity step at hand-over, then the same bounded stop.
  JogSmoother seeded;
  assert(seeded.configure(window, dt));
  JogAxis trajectory{5e6, 1.2 * counts_per_rad};
  trajectory.smoother = &seeded;
  seeded.seed(trajectory.position, trajectory.velocity);
  targets.assign({5e6 - 2 * trajectory.velocity * dt, 5e6 - trajectory.velocity * dt, 5e6});
  assert(ramp.begin(trajectory, acceleration * counts_per_rad, period, now));
  while (trajectory.velocity != 0) {
    assert(ramp.advance(trajectory, trajectory.position, 1e12, now));
    targets.push_back(trajectory.position);
    now += period;
  }
  within(measure(targets), acceleration, jerk);
}

// The governor judges the integrator; a mean of integrator positions stays on
// the integrator's side of a limit at full jog speed, released or held.
static void position_limit_smoothed(int direction, int release_cycle) {
  JogSmoother smoother;
  assert(smoother.configure(window, dt));
  AxisLimits limits{-.5, .5, 1.4, true, true};
  limits.jog_acceleration = acceleration;
  const auto bounds = command_limits(limits, counts_per_rad, 0);
  std::array<JogAxis, jog_session_axes> axes{};
  axes[0].smoother = &smoother;
  axes[0].reset(0);
  JogLane lane;
  HaltRamp halt;
  bool cancelling = false;
  std::vector<double> targets{0, 0, 0};
  for (int tick = 1; tick <= 3000; ++tick) {
    const uint64_t now = 1'000'000'000ULL + uint64_t(tick) * period;
    if (tick == release_cycle) {
      cancelling = true;
      assert(halt.begin(axes[0], acceleration * counts_per_rad, period, now));
    }
    lane.set_limit_feedback(0, limits, axes[0].position, counts_per_rad, 0);
    std::array<double, jog_session_axes> request{};
    request[0] = direction * 1.3 * counts_per_rad;
    lane.govern(1, axes, request, dt);
    if (cancelling) assert(halt.advance(axes[0], axes[0].position, 1e12, now));
    else assert(lane.advance(0, axes[0], request[0], axes[0].position, acceleration * counts_per_rad, 1e12, dt, 0));
    assert(axes[0].position >= bounds.min_q && axes[0].position <= bounds.max_q);
    targets.push_back(axes[0].position);
  }
  within(measure(targets), acceleration, jerk);
  if (release_cycle < 0) {
    const double endpoint = direction > 0 ? bounds.max_q : bounds.min_q;
    assert(std::abs(axes[0].position - endpoint) <= 2.01); // <= two command counts inside.
  }
}

// EndJog through the lane: the ramp closes only once the smoothed target has
// stopped, and never trips the lane's overrun bound.
static void lane_end_jog() {
  JogFixture f(period);
  std::array<JogSmoother, 2> smoothers{};
  for (unsigned axis = 0; axis < 2; ++axis) {
    assert(smoothers[axis].configure(50, dt));
    f.axes[axis].smoother = &smoothers[axis];
    f.lane.configure_axis(axis, f.axes[axis], f.timing, 10'000, period);
  }
  smoothers[0].seed(0, 2000); smoothers[1].seed(0, -1000);
  f.axes[0].position = 0; f.axes[0].velocity = 2000;
  f.axes[1].position = 0; f.axes[1].velocity = -1000;
  f.end();
  const auto started = f.context.now_ns;
  assert(!f.lane.cycle(&f.region, f.context, f.limits));
  while (f.lane.ramping()) {
    for (unsigned axis = 0; axis < 2; ++axis)
      assert(f.lane.advance(axis, f.axes[axis], 9999, f.axes[axis].position, 10'000, 1e12, dt, 0));
    f.context.now_ns += period;
    assert(!f.lane.cycle(&f.region, f.context, f.limits));
  }
  // Integrator stop 200 ms (2000 counts/s at 10000 counts/s^2); the target
  // settles 49 cycles after the integrator's stopping step, within the
  // ramp bound 200 ms + 50-cycle window + one cycle.
  assert(f.context.now_ns - started >= 249'000'000);
  assert(f.context.now_ns - started <= 251'000'000);
  assert(f.axes[0].velocity == 0 && f.axes[1].velocity == 0);
  assert(f.observe().state_reason == wire::jog_reason_closed);
}
} // namespace smoothing

static void arrest(uint64_t period) {
  JogTiming timing;
  assert(timing.configure(period));
  AxisOutputState state{};
  state.arrest_cycles_left = timing.arrest_cycles;
  state.quick_stop_cycles_left = timing.quick_stop_cycles;
  AxisOutputConfig config{};
  AxisOutputInput input{};
  uint64_t elapsed = 0, arrest_finished = 0;
  while (state.quick_stop_cycles_left) {
    state = evaluate_axis_output(config, input, state).next;
    elapsed += period;
    if (!state.arrest_cycles_left && !arrest_finished) arrest_finished = elapsed;
    assert(elapsed <= 300'000'000 + period);
  }
  assert(arrest_finished >= 200'000'000 && arrest_finished < 200'000'000 + period);
  assert(elapsed >= 300'000'000 && elapsed < 300'000'000 + period);
  timing.arrest_ns = 200'000'001;
  timing.quick_stop_ns = 300'000'001;
  assert(timing.configure(period));
  assert(uint64_t(timing.arrest_cycles) * period >= 200'000'001);
  assert(uint64_t(timing.arrest_cycles) * period < 200'000'001 + period);
  assert(uint64_t(timing.quick_stop_cycles) * period >= 300'000'001);
  assert(uint64_t(timing.quick_stop_cycles) * period < 300'000'001 + period);
  timing.arrest_ns = 0;
  assert(!timing.configure(period));
  timing.arrest_ns = UINT64_MAX;
  assert(!timing.configure(period));
}

int main() {
  for (int direction : {-1, 1}) for (int sign : {-1, 1})
    for (int release : {-1, 1000, 2500, 3500, 5000}) position_limit(direction, sign, release);
  smoothing::start_stop_and_reversal();
  smoothing::halt_ramp();
  for (int direction : {-1, 1})
    for (int release : {-1, 200, 350, 420, 480}) smoothing::position_limit_smoothed(direction, release);
  smoothing::lane_end_jog();
  // Admission: the history is fixed-size; a window beyond it is refused.
  {
    JogSmoother s;
    assert(s.configure(smoothing::window, .001) && s.enabled());
    assert(!s.configure(JogSmoother::max_window + 1, .001));
    assert(!s.configure(10, 0) && !s.configure(10, std::numeric_limits<double>::infinity()));
    assert(s.configure(1, .001) && !s.enabled());
    JogAxis unsmoothed{};
    unsmoothed.smoother = &s;
    assert(!unsmoothed.smoothed());
  }
  // An exterior pose can move inward; governing cannot permit outward travel.
  AxisLimits exterior{-100, 100, 100, true, true};
  assert(jog_position_scale(exterior, JogAxis{120, 0}, 10, 10, .001) == 0);
  assert(jog_position_scale(exterior, JogAxis{120, 0}, -10, 10, .001) == 1);
  assert(jog_position_scale(exterior, JogAxis{-120, 0}, -10, 10, .001) == 0);
  assert(jog_position_scale(exterior, JogAxis{-120, 0}, 10, 10, .001) == 1);
  // Continuous coordinates use the one exact configured interval.
  assert(jog_position_scale(AxisLimits{10, 50, 100, true, true}, JogAxis{99, 0}, 5, 10, .001) == 0);
  assert(jog_position_scale(AxisLimits{10, 50, 100, true, true}, JogAxis{99, 0}, -5, 10, .001) == 1);
  // TestHaltElapsedClock: 2000 counts/s at 10000 counts/s^2 stops at 200
  // counts despite irregular observations; the absolute deadline stays fixed.
  {
    HaltRamp ramp;
    JogAxis axis{0, 2000};
    assert(ramp.begin(axis, 10'000, 1'000'000, 1'000'000'000));
    assert(ramp.advance(axis, 0, 1000, 1'000'000'000));
    assert(axis.velocity == 1990);
    assert(ramp.advance(axis, 0, 1000, 1'005'000'000));
    assert(axis.velocity == 1940);
    assert(ramp.advance(axis, 0, 1000, 1'199'000'000));
    assert(axis.velocity == 0 && std::abs(axis.position - 200) <= 1e-8);
    assert(ramp.bound_ns == 201'000'000);
    assert(!ramp.advance(axis, 0, 1000, 1'198'000'000));
    axis = {0, 2000};
    assert(ramp.begin(axis, 10'000, 1'000'000, 1'000'000'000));
    assert(!ramp.advance(axis, 0, 1000, 1'201'000'001));
  }
  // TestHaltFinalStepLandsOnZero: 2.618268488438603 counts/s at 10000
  // counts/s^2 stops inside its first step (262 us). a*(|v|/a) rounds below |v|
  // for this velocity; the 4.4e-16 residual it left read as motion past the
  // 1.262 ms bound once the next cycle ran late, and inhibited output on an
  // ordinary Cartesian jog release (Deck simulation, 2026-09-27).
  {
    HaltRamp ramp;
    JogAxis axis{0, 2.618268488438603};
    assert(ramp.begin(axis, 10'000, 1'000'000, 1'000'000'000));
    assert(ramp.bound_ns == 1'261'827);
    assert(ramp.advance(axis, 0, 1000, 1'000'000'000));
    assert(axis.velocity == 0);
    assert(ramp.advance(axis, 0, 1000, 1'001'300'000));
    assert(axis.velocity == 0);
  }
  // TestTrackingLagAllowanceLead: the 1430 cell J3 gearing (131072 x 100 counts per
  // joint revolution) and its 0.35 deg max_target_lead. Feedback trailing the
  // target by the 27 ms measured on the 1430 cell stop dumps (2026-10-06) at 42 deg/s
  // leads by 1.134 deg: beyond the static bound, inside the 50 ms allowance's
  // 0.35 + 42 x 0.05 = 2.45 deg. A 60 ms lag still exceeds the scaled bound.
  {
    const double counts_per_deg = 131072.0 * 100 / 360;
    const double speed = 42 * counts_per_deg, static_lead = .35 * counts_per_deg, dt = .001;
    auto step = [&](double lag_behind_s, double allowance_s) {
      JogAxis axis{0, speed};
      assert(axis.advance(speed, speed * dt - speed * lag_behind_s, 1e9, static_lead, dt, allowance_s));
      assert(std::abs(axis.lead / counts_per_deg - 42 * lag_behind_s) < 1e-9);
      return axis;
    };
    auto axis = step(.027, 0);
    assert(axis.lead_exceeded && axis.lead_limit == static_lead);
    axis = step(.027, .05);
    assert(!axis.lead_exceeded && std::abs(axis.lead_limit / counts_per_deg - 2.45) < 1e-9);
    axis = step(.060, .05);
    assert(axis.lead_exceeded);
    // Reverse travel scales by |v|; a stationary hold gains nothing.
    JogAxis reverse{0, -speed};
    assert(reverse.advance(-speed, -speed * dt + speed * .027, 1e9, static_lead, dt, .05));
    assert(!reverse.lead_exceeded && std::abs(reverse.lead_limit / counts_per_deg - 2.45) < 1e-9);
    JogAxis still{0, 0};
    assert(still.advance(0, -static_lead - 1, 1e9, static_lead, dt, .05));
    assert(still.lead_exceeded && still.lead_limit == static_lead);
    // A negative or nonfinite allowance is invalid input, never a wider bound.
    JogAxis invalid{0, speed};
    assert(!invalid.advance(speed, 0, 1e9, static_lead, dt, -.001));
    assert(!invalid.advance(speed, 0, 1e9, static_lead, dt, std::numeric_limits<double>::quiet_NaN()));
    assert(!invalid.advance(speed, 0, 1e9, static_lead, dt, std::numeric_limits<double>::infinity()));
    // Halt/EndJog scales by the ramp's own decelerating commanded velocity.
    HaltRamp ramp;
    JogAxis halt{0, speed};
    assert(ramp.begin(halt, 10 * speed, 1'000'000, 1'000'000'000));
    assert(ramp.advance(halt, -speed * .027, static_lead, 1'000'000'000, .05));
    assert(std::abs(halt.velocity - .99 * speed) < 1e-6 && !halt.lead_exceeded);
    assert(std::abs(halt.lead_limit - (static_lead + .99 * speed * .05)) < 1e-6);
    JogLane lane;
    JogAxis lane_axis{0, speed};
    assert(lane.advance(0, lane_axis, speed, speed * dt - speed * .027, 1e9, static_lead, dt, .05));
    assert(!lane_axis.lead_exceeded);
  }
  // A6/simulation DC quantum: every supported 250000 ns multiple through 10 ms.
  for (uint64_t period = 250'000; period <= 10'000'000; period += 250'000) {
    arrest(period);
    ramp(period, false);
    ramp(period, true);
    for (unsigned cause = 0; cause < 5; ++cause) {
      JogFixture f(period);
      f.axes[0].velocity = 2000;
      f.end();
      assert(!f.lane.cycle(&f.region, f.context, f.limits));
      if (cause == 0) ++f.context.identity.native_generation;
      if (cause == 1) f.context.authority_valid = false;
      if (cause == 2) f.context.ready_mask = 0;
      if (cause == 3) f.context.identity.connection_id = 0;
      if (cause == 4) f.context.now_ns += 300'000'000 + period;
      assert(f.lane.cycle(&f.region, f.context, f.limits));
      assert(!f.lane.ramping() && !f.lane.state().open);
    }
    JogFixture impossible(period);
    impossible.axes[0].velocity = std::numeric_limits<double>::infinity();
    impossible.end();
    assert(impossible.lane.cycle(&impossible.region, impossible.context, impossible.limits));
    assert(!impossible.lane.state().open);
    JogFixture dispatch(period);
    dispatch.axes[0].velocity = 2000;
    assert(!dispatch.lane.expire_before_dispatch(dispatch.update.immutable_deadline_host_ns));
    assert(dispatch.lane.ramping());
    dispatch.axes[0].velocity = 0;
    dispatch.context.now_ns = dispatch.update.immutable_deadline_host_ns + period;
    assert(!dispatch.lane.cycle(&dispatch.region, dispatch.context, dispatch.limits));
    assert(dispatch.observe().state_reason == wire::jog_reason_expired);
  }
  JogFixture limited(1'000'000);
  limited.limits[1] = 1500;
  limited.update.velocity[0] = 8000;
  limited.update.velocity[1] = -4000;
  ++limited.update.source_sequence;
  limited.input();
  assert(!limited.lane.cycle(&limited.region, limited.context, limited.limits));
  assert(limited.lane.state().velocity[0] == 2000);
  assert(limited.lane.state().velocity[1] == -1000);
  const auto observation = limited.observe();
  assert(observation.velocity_scale_ppm == 250'000 && observation.update_reason == wire::jog_reason_limited);
  assert(observation.open == 1 && observation.has_input == 1);
  const auto valid_update = limited.update;
  for (unsigned cause = 0; cause < 3; ++cause) {
    auto invalid = valid_update;
    if (cause == 0) invalid.velocity[0] = std::numeric_limits<double>::infinity();
    if (cause == 1) invalid.velocity[2] = 1;
    if (cause == 2) invalid.reserved = 1;
    ++invalid.source_sequence;
    limited.update = invalid;
    limited.input();
    assert(!limited.lane.cycle(&limited.region, limited.context, limited.limits));
    assert(limited.observe().update_reason == wire::jog_reason_invalid_vector);
    assert(limited.lane.state().source_sequence == 2);
  }
  // Wire measurements: the observation remains 160 bytes in ABI 1.6, and the
  // generated debug decoder matches the existing signed motor-count fields.
  static_assert(wire::jog_observation_bytes == 160 && wire::version_minor == 15);
  static_assert(sizeof(gradient::ipc::v1::StatusJogDebugV1) == 360);
  std::array<uint8_t, 360> raw{};
  raw[296] = 0xff; raw[297] = 0xff; raw[298] = 0xff; raw[299] = 0xff;
  gradient::ipc::v1::StatusJogDebugV1 debug{};
  assert(gradient::ipc::v1::decode_StatusJogDebugV1(raw.data(), raw.size(), debug));
  assert(debug.output_target_velocity_counts_per_s[0] == -1);
  assert(gradient::ipc::v1::encode_StatusJogDebugV1(debug) == raw);
  std::cout << "PASS: time-based jog arrest, acceleration-bounded halt, urgent inhibition and uniform limiting\n";
}
