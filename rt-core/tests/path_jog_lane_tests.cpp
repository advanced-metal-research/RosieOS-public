// Path jog lane (protocol 1.14) through the shared fast-control region: the
// peer side publishes path_begin, path_samples chunks and path speed updates
// exactly as the adapter does; the cyclic side runs JogLane + PathJog the way
// the cycle machine does. Units are the 1430 cell's: gear 100 on a 131072-count
// encoder, 1.396 rad/s, 4*pi rad/s^2 jog acceleration, 150 rad/s^3 machine
// jog_jerk, 1 ms cycle, 1 mm path spacing. Thresholds are absolute (counts,
// rad/s^3), never derived from the test's own cycle.
#include "engine/jog_lane.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

using namespace rosie::rtcore;
namespace wire = rosie::protocol;

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double counts_per_rad = 131072.0 * 100 / (2 * pi);
constexpr uint32_t mask = 0x3f;
constexpr uint64_t cycle_ns = 1'000'000;
constexpr double spacing = 0.001;  // m per sample (1 mm)
// Joint path per metre of travel: the shape of a seeded-IK line, with
// curvature, so every axis moves a different amount per sample.
constexpr double slope[6] = {1.5, -0.8, 0.6, 2.0, -1.2, 0.9};
constexpr double bend[6] = {0.5, -0.3, 0.8, -0.6, 0.4, 1.1};
constexpr double held[6] = {0.1, -0.4, 0.7, 0.0, 1.2, -2.0};

double curve(uint32_t axis, double s) { return held[axis] + slope[axis] * s + bend[axis] * s * s; }

struct Fixture {
  alignas(8) std::array<uint8_t, fast_control_region_bytes> bytes{};
  FastControlRegion region;
  FastControlRegion::JogControlSlot controls;
  FastControlRegion::JogUpdateSlot updates;
  FastControlRegion::PathSamplesSlot samples;
  FastControlRegion::PathObservationSlot path_observations;
  FastControlRegion::JogObservationSlot observations;
  JogLane lane;
  std::unique_ptr<PathJog> path = std::make_unique<PathJog>();
  JogSessionContext context;
  std::array<double, jog_session_axes> limits{};
  wire::JogControl control{};
  wire::JogUpdate update{};
  uint64_t ticket = 0, chunk = 0, sequence = 0, next_index = 0;

  explicit Fixture(double jerk = 150) {
    wire::FastHeader header{};
    header.magic = fast_control_magic;
    header.version_major = wire::version_major;
    header.version_minor = wire::version_minor;
    header.region_bytes = fast_control_region_bytes;
    header.daemon_incarnation[0] = 1;
    header.connection_id = 1;
    header.configuration_digest_lo[0] = 1;
    header.cycle_ns = cycle_ns;
    header.configured_mask = mask;
    assert(FastControlRegion::initialize(bytes.data(), bytes.size(), header, region) == FastControlStatus::Ok);
    assert(decltype(controls)::bind(bytes.data() + fast_control_jog_control_offset, decltype(controls)::region_bytes, controls) == SharedWordsStatus::Ok);
    assert(decltype(updates)::bind(bytes.data() + fast_control_jog_update_offset, decltype(updates)::region_bytes, updates) == SharedWordsStatus::Ok);
    assert(decltype(samples)::bind(bytes.data() + fast_control_path_samples_offset, decltype(samples)::region_bytes, samples) == SharedWordsStatus::Ok);
    assert(decltype(path_observations)::bind(bytes.data() + fast_control_path_observation_offset,
        decltype(path_observations)::region_bytes, path_observations) == SharedWordsStatus::Ok);
    assert(decltype(observations)::bind(bytes.data() + fast_control_jog_observation_offset,
        decltype(observations)::region_bytes, observations) == SharedWordsStatus::Ok);
    context.identity = {1, 1, {1}, 1, 1, 1, 1};
    context.now_ns = 1'000'000'000;
    context.grant_deadline_ns = UINT64_MAX / 2;
    context.max_input_age_ns = context.max_first_input_wait_ns = 250'000'000;
    context.configured_mask = context.allowed_mask = context.ready_mask = mask;
    context.authority_valid = context.clock_mapping_valid = context.armed = true;
    limits.fill(1.396);
    lane.bind_path(*path);
    lane.set_path_timing(jerk, 0.02, cycle_ns);
    for (uint32_t axis = 0; axis < 6; ++axis) {
      const double count = 1 / counts_per_rad;
      lane.set_path_axis(axis, {-3 + count, 3 - count, 1.396, 4 * pi, count}, held[axis]);
    }
    control.connection_id = control.native_generation = 1;
    control.capability_id[0] = 1;
    control.app_grant_generation = control.configuration_epoch = control.home_epoch = control.clock_mapping_generation = 1;
    update.conn_id = update.native_generation = update.app_grant_generation = 1;
    update.capability_id[0] = 1;
    update.config_epoch = update.home_epoch = update.clock_mapping_generation = 1;
  }
  void publish_control(uint32_t operation, uint64_t generation = 0) {
    control.ticket = ++ticket;
    control.operation = operation;
    control.jog_generation = generation;
    const bool begin = operation != wire::jog_operation_end;
    control.axis_mask = begin ? mask : 0;
    control.origin_host_ns = begin ? context.now_ns : 0;
    control.deadline_host_ns = begin ? context.now_ns + 200'000'000 : 0;
    assert(controls.publish(shared_words_from_le<decltype(controls)::payload_words>(wire::encode_jog_control(control))).status == SharedWordsStatus::Ok);
  }
  void speed(double value, uint64_t generation, uint64_t lifetime = 200'000'000, uint32_t flags = 1) {
    update.flags = flags;
    update.jog_generation = generation;
    update.source_sequence = ++sequence;
    update.axis_mask = mask;
    update.velocity = {};
    update.velocity[0] = value;
    update.earliest_origin_host_ns = context.now_ns;
    update.immutable_deadline_host_ns = context.now_ns + lifetime;
    assert(updates.publish(shared_words_from_le<decltype(updates)::payload_words>(wire::encode_jog_update(update))).status == SharedWordsStatus::Ok);
  }
  wire::PathSamples chunk_of(uint64_t generation, uint64_t first, uint32_t count, bool end = false) {
    wire::PathSamples c{};
    c.conn_id = c.native_generation = c.app_grant_generation = c.config_epoch = c.home_epoch = c.clock_mapping_generation = 1;
    c.capability_id[0] = 1;
    c.jog_generation = generation;
    c.chunk_sequence = ++chunk;
    c.first_index = first;
    c.spacing = spacing;
    c.max_speed = 0.4;
    c.count = count;
    c.flags = end ? 1 : 0;
    c.caps.fill(std::numeric_limits<double>::infinity());
    for (uint32_t i = 0; i < count; ++i)
      for (uint32_t axis = 0; axis < 6; ++axis)
        c.samples[i * path_jog_axes + axis] = curve(axis, double(first + i) * spacing);
    return c;
  }
  void publish_chunk(const wire::PathSamples& c) {
    assert(samples.publish(shared_words_from_le<decltype(samples)::payload_words>(wire::encode_path_samples(c))).status == SharedWordsStatus::Ok);
  }
  // Intake only, as the cycle machine does before dispatch.
  bool intake() { return lane.cycle(&region, context, limits); }
  wire::JogObservation jog() {
    assert(lane.publish(region, context.identity, context.now_ns).status == SharedWordsStatus::Ok);
    decltype(observations)::Words words{};
    assert(observations.read(words).status == SharedWordsStatus::Ok);
    const auto encoded = shared_words_to_le(words);
    wire::JogObservation value{};
    assert(wire::decode_jog_observation(encoded.data(), encoded.size(), value));
    return value;
  }
  wire::PathObservation observed() {
    assert(lane.publish_path(region, context.identity, context.now_ns).status == SharedWordsStatus::Ok);
    decltype(path_observations)::Words words{};
    assert(path_observations.read(words).status == SharedWordsStatus::Ok);
    const auto encoded = shared_words_to_le(words);
    wire::PathObservation value{};
    assert(wire::decode_path_observation_frame(encoded.data(), encoded.size(), value));
    return value;
  }
  // Publish one chunk and run one intake; the acknowledgement must name it.
  wire::PathObservation send(const wire::PathSamples& c) {
    publish_chunk(c);
    assert(!intake());
    const auto o = observed();
    assert(o.chunk_sequence_seen == c.chunk_sequence);
    return o;
  }
  void fill(uint64_t generation, uint64_t through) {
    while (next_index < through) {
      const uint32_t count = uint32_t(std::min<uint64_t>(16, through - next_index));
      assert(send(chunk_of(generation, next_index, count)).chunk_result == wire::jog_reason_none);
      next_index += count;
    }
  }
  uint64_t begin_path() {
    publish_control(wire::jog_operation_path_begin);
    assert(!intake());
    const auto o = jog();
    assert(o.control_accepted == 1 && o.open == 1);
    return o.jog_generation;
  }
  // One dispatch: the cycle machine's advance + path_cycle.
  struct Step { PathJogCycle cycle; bool inhibit; };
  Step dispatch() {
    const auto cycle = path->advance(context.now_ns);
    return {cycle, lane.path_cycle(cycle, context.now_ns)};
  }
  void tick() { context.now_ns += cycle_ns; }
};

// Every emitted target lies on the sampled curve within one count, and joint
// jerk on the commanded target stays at most 150 rad/s^3 (plus 1 % for the
// third difference's discretisation of a continuous bound).
struct CurveCheck {
  std::vector<std::array<double, 6>> q;
  void record(const PathJog& path) {
    const double s = path.s() * spacing;
    std::array<double, 6> sample{};
    for (uint32_t axis = 0; axis < 6; ++axis) {
      sample[axis] = path.position()[axis];
      assert(std::abs(sample[axis] - curve(axis, s)) * counts_per_rad <= 1.0);
    }
    q.push_back(sample);
  }
  void jerk_bounded() const {
    const double dt = double(cycle_ns) / 1e9;
    for (size_t k = 3; k < q.size(); ++k)
      for (uint32_t axis = 0; axis < 6; ++axis) {
        const double jerk = (q[k][axis] - 3 * q[k - 1][axis] + 3 * q[k - 2][axis] - q[k - 3][axis]) / (dt * dt * dt);
        assert(std::abs(jerk) <= 150 * 1.01);
      }
  }
};

void test_admission_and_identity() {
  {
    Fixture f(0);  // No machine jog_jerk: the capability is absent.
    f.publish_control(wire::jog_operation_path_begin);
    assert(!f.intake());
    const auto o = f.jog();
    assert(o.control_accepted == 0 && o.control_reason == wire::jog_reason_path_unavailable && o.open == 0);
  }
  Fixture f;
  const auto generation = f.begin_path();
  assert(f.lane.path_mode() && !f.lane.path_begun());
  // Speed before any geometry is refused; nothing can move without samples.
  f.speed(0.1, generation);
  assert(!f.intake());
  assert(f.jog().update_reason == wire::jog_reason_path_unavailable);
  // A plain velocity update never steers a path session.
  f.update.flags = 0;
  f.update.jog_generation = generation;
  f.update.source_sequence = ++f.sequence;
  f.update.velocity = {};
  f.update.velocity[1] = 0.1;
  f.update.earliest_origin_host_ns = f.context.now_ns;
  f.update.immutable_deadline_host_ns = f.context.now_ns + 100'000'000;
  assert(f.updates.publish(shared_words_from_le<decltype(f.updates)::payload_words>(wire::encode_jog_update(f.update))).status == SharedWordsStatus::Ok);
  assert(!f.intake());
  assert(f.jog().update_reason == wire::jog_reason_mode_conflict);
  // A chunk of another generation is ignored without acknowledgement.
  auto stale = f.chunk_of(generation + 1, 0, 4);
  f.publish_chunk(stale);
  assert(!f.intake());
  assert(f.observed().chunk_sequence_seen == 0);
  f.chunk = 0;
  // A sequence gap is refused and acknowledged; the expected sequence stays.
  ++f.chunk;
  auto gap = f.chunk_of(generation, 0, 4);
  assert(gap.chunk_sequence == 2);
  assert(f.send(gap).chunk_result == wire::jog_reason_path_out_of_order);
  f.chunk = 0;
  // A nonzero unmasked axis is refused.
  auto unmasked = f.chunk_of(generation, 0, 4);
  unmasked.samples[6] = 0.25;
  assert(f.send(unmasked).chunk_result == wire::jog_reason_path_invalid_sample);
  f.chunk = 0;
  f.fill(generation, 64);
  assert(f.lane.path_begun());
  // Spacing is fixed by the first chunk.
  auto respaced = f.chunk_of(generation, 64, 4);
  respaced.spacing = 0.002;
  assert(f.send(respaced).chunk_result == wire::jog_reason_path_invalid_config);
  --f.chunk;
  // Index continuity is enforced by the generator.
  auto skipped = f.chunk_of(generation, 70, 4);
  assert(f.send(skipped).chunk_result == wire::jog_reason_path_out_of_order);
  --f.chunk;
  f.fill(generation, 1024);
  // The ring holds 1024 samples: one more is refused without corruption.
  auto over = f.chunk_of(generation, 1024, 1);
  assert(f.send(over).chunk_result == wire::jog_reason_path_capacity);
  const auto o = f.observed();
  assert(o.received == 1024 && (o.flags & 1) && o.jog_generation == generation);
  // A path speed on a joint session is a mode conflict.
  Fixture joint;
  joint.publish_control(wire::jog_operation_begin);
  assert(!joint.intake());
  const auto joint_generation = joint.jog().jog_generation;
  joint.speed(0.1, joint_generation);
  assert(!joint.intake());
  assert(joint.jog().update_reason == wire::jog_reason_mode_conflict);
  // Path samples on a joint session are acknowledged as a mode conflict.
  assert(joint.send(joint.chunk_of(joint_generation, 0, 4)).chunk_result == wire::jog_reason_mode_conflict);
  std::cout << "PASS: path_begin admission, identity fences and the chunk handshake\n";
}

void test_start_mismatch_is_a_refusal() {
  Fixture f;
  const auto generation = f.begin_path();
  auto c = f.chunk_of(generation, 0, 8);
  for (uint32_t i = 0; i < 8; ++i) c.samples[i * path_jog_axes + 2] += 2 / counts_per_rad;  // Two counts off.
  f.publish_chunk(c);
  assert(!f.intake());  // Nothing moved: refused without inhibiting output.
  const auto o = f.observed();
  assert(o.chunk_sequence_seen == 1 && o.chunk_result == wire::jog_reason_path_start_mismatch);
  assert(!f.lane.state().open && f.jog().state_reason == wire::jog_reason_path_start_mismatch);
  std::cout << "PASS: a sample 0 off the held command closes the session without inhibit\n";
}

// Release, expiry and Halt all stop along the path; the session closes only
// when the stop completes, without inhibiting output.
void test_stops_along_the_path() {
  for (int mode = 0; mode < 3; ++mode) {
    Fixture f;
    const auto generation = f.begin_path();
    f.fill(generation, 400);
    f.speed(0.2, generation);
    assert(!f.intake());
    assert(f.jog().update_reason == wire::jog_reason_none);
    CurveCheck check;
    uint64_t renewed = f.context.now_ns;
    bool stopping = false;
    double stop_speed = 0, stop_s = 0;
    for (int cycle = 0; cycle < 5000 && f.lane.state().open; ++cycle) {
      f.tick();
      // Keep the operator speed fresh for 300 ms, then release / expire / halt.
      const bool active = f.context.now_ns < 1'300'000'000;
      if (active && f.context.now_ns - renewed >= 50'000'000) {
        f.speed(0.2, generation);
        renewed = f.context.now_ns;
      }
      if (!active && !stopping) {
        stopping = true;
        stop_speed = f.path->speed() * spacing;
        stop_s = f.path->s() * spacing;
        if (mode == 0) f.publish_control(wire::jog_operation_end, generation);
        if (mode == 2) f.lane.halt_path(f.context.now_ns);
      }
      assert(!f.intake());
      if (stopping && mode != 1) assert(f.jog().open == 0);  // Input fenced while stopping.
      const auto step = f.dispatch();
      assert(!step.inhibit && !step.cycle.fault);
      check.record(*f.path);
    }
    assert(!f.lane.state().open);
    assert(stop_speed > 0.15);  // It was moving when the stop began.
    // Expiry waits out the last speed command's 200 ms lifetime first.
    const double travelled = f.path->s() * spacing - stop_s;
    assert(travelled > 0 && travelled < (mode == 1 ? 0.2 * 0.2 + 0.05 : 0.05));
    assert(f.path->speed() == 0 && f.path->acceleration() == 0);
    check.jerk_bounded();
    const auto o = f.jog();
    assert(o.state_reason == (mode == 1 ? wire::jog_reason_expired : wire::jog_reason_closed));
    const auto p = f.observed();
    assert(!(p.flags & 1) && !(p.flags & 16) && p.path_speed == 0);
    assert(p.stop_reason == uint32_t(mode == 0 ? PathStopReason::Release : mode == 1 ? PathStopReason::Expired : PathStopReason::Halt));
  }
  std::cout << "PASS: End, expiry and Halt stop on the sampled curve within 1 count, jerk <= 150 rad/s^3\n";
}

// A stop that reaches the end of the received geometry ends exactly there.
void test_underrun_stops_at_playable_end() {
  Fixture f;
  const auto generation = f.begin_path();
  f.fill(generation, 48);
  f.speed(0.3, generation);
  assert(!f.intake());
  CurveCheck check;
  uint64_t renewed = f.context.now_ns;
  for (int cycle = 0; cycle < 1000; ++cycle) {
    f.tick();
    if (f.context.now_ns - renewed >= 50'000'000) {
      f.speed(0.3, generation);
      renewed = f.context.now_ns;
    }
    assert(!f.intake());
    const auto step = f.dispatch();
    assert(!step.inhibit && !step.cycle.fault);
    check.record(*f.path);
  }
  const auto p = f.observed();
  assert(p.path_speed == 0 && p.stop_reason == uint32_t(PathStopReason::Underrun) && p.underruns == 1);
  assert(f.path->s() <= double(p.playable_end_index) && (p.flags & 1));
  check.jerk_bounded();
  std::cout << "PASS: an exhausted buffer stops at the playable end\n";
}

// A generator fault fails closed: the session closes with path_fault and the
// owner is told to inhibit output.
void test_fault_fails_closed() {
  Fixture f;
  const auto generation = f.begin_path();
  f.fill(generation, 64);
  f.speed(0.1, generation);
  assert(!f.intake());
  for (int cycle = 0; cycle < 20; ++cycle) { f.tick(); assert(!f.intake()); assert(!f.dispatch().inhibit); }
  f.context.now_ns -= 2 * cycle_ns;  // Time went backwards: PathJog fails closed.
  const auto cycle = f.path->advance(f.context.now_ns);
  assert(cycle.fault && cycle.closed);
  assert(f.lane.path_cycle(cycle, f.context.now_ns));
  assert(!f.lane.state().open && f.jog().state_reason == wire::jog_reason_path_fault);
  assert(f.observed().flags & 16);
  std::cout << "PASS: a path generator fault closes the session and inhibits\n";
}

// Authority loss abandons the path at its last target (existing inhibit).
void test_authority_loss_inhibits() {
  Fixture f;
  const auto generation = f.begin_path();
  f.fill(generation, 64);
  f.speed(0.1, generation);
  assert(!f.intake());
  for (int cycle = 0; cycle < 20; ++cycle) { f.tick(); assert(!f.intake()); f.dispatch(); }
  f.tick();
  f.context.authority_valid = false;
  assert(f.intake());
  assert(!f.lane.state().open && !f.path->open());
  // Reported as an inhibit, not a generator fault.
  assert(f.path->observe().stop_reason == static_cast<uint32_t>(PathStopReason::Inhibited));
  std::cout << "PASS: authority loss abandons the path and inhibits\n";
}
}  // namespace

int main() {
  test_admission_and_identity();
  test_start_mismatch_is_a_refusal();
  test_stops_along_the_path();
  test_underrun_stops_at_playable_end();
  test_fault_fails_closed();
  test_authority_loss_inhibits();
}
