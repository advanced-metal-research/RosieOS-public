// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "control_policy.hpp"
#include "command_validation.hpp"
#include "startup_sdo.hpp"
#include "execution_policy.hpp"
#include "engine/jog.hpp"
#include "engine/trajectory.hpp"
#include "engine/trajectory_store.hpp"
#include "engine/publication.hpp"
#include <cassert>
#include <limits>
#include <thread>

int main() {
  using namespace rosie::rtcore;
  std::array<TrajectoryPoint, 3> path{};
  for (unsigned i = 0; i < 3; ++i) { path[i].axis_mask = 1; path[i].t_from_start_ns = i * 1000000000ULL; path[i].target_counts[0] = i * 100; }
  auto sampled = sample_trajectory(path.data(), path.size(), 500000000);
  assert(sampled.position[0] == 50 && sampled.velocity[0] == 100 && sampled.index == 0);
  sampled = sample_trajectory(path.data(), path.size(), 3000000000);
  assert(sampled.position[0] == 200 && sampled.velocity[0] == 0 && sampled.index == 2);
  // A segment spanning more than one retired wrap period remains continuous.
  path[0].target_counts[0] = 990; path[1].target_counts[0] = 2010;
  sampled = sample_trajectory(path.data(), 2, 500000000);
  assert(sampled.position[0] == 1500 && sampled.velocity[0] == 1020);
  JogAxis jog;
  jog.reset(0);
  assert(jog.advance(1, 0, 2, .5, .1));
  assert(std::abs(jog.velocity - .2) < 1e-12 && std::abs(jog.position - .01) < 1e-12);
  assert(jog.advance(-1, .01, 2, .5, .1));
  assert(std::abs(jog.velocity) < 1e-12 && std::abs(jog.position - .02) < 1e-12);
  // A lead beyond max_lead is admitted and reported as a warning (2026-10-06).
  assert(jog.advance(1, -1, 2, .1, .1));
  assert(jog.lead_exceeded && std::abs(jog.lead - 1.03) < 1e-12);
  assert(std::abs(jog.position - .03) < 1e-12 && std::abs(jog.velocity - .2) < 1e-12);
  assert(jog.lead_limit == .1);
  // Invalid input is still refused and never accumulates.
  assert(!jog.advance(std::numeric_limits<double>::quiet_NaN(), -1, 2, .1, .1));
  assert(std::abs(jog.position - .03) < 1e-12 && std::abs(jog.velocity - .2) < 1e-12);
  FollowingErrorGuard following;
  assert(following.accept(.2, .1, 100, 50));
  assert(following.accept(.2, .1, 149, 50));
  assert(!following.accept(.2, .1, 150, 50));
  assert(following.accept(0, .1, 151, 50));
  assert(following.accept(.2, .1, 152, 50));
  TrajectoryStore<int> store;
  std::vector<int> points{1,2,3};
  assert(store.prepare(1, 1, 1, points) == 0);
  assert(store.acquire_ready() == -1); // Preparation is inert.
  assert(store.start(1, 1, 10));
  const auto slot = store.acquire_ready();
  assert(slot == 0 && store.view(slot).points[2] == 3);
  store.invalidate(2);
  assert(store.view(slot).points[2] == 3); // Active lifetime belongs to RT.
  store.retire(slot);
  assert(!store.start(1, 1, 11));
  struct Coherent { uint64_t sequence, inverse; };
  AtomicSnapshot<Coherent> snapshot;
  Coherent observed{};
  assert(!snapshot.read(observed));
  std::atomic<bool> finished{false};
  std::thread writer([&] { for (uint64_t i=1;i<40000;++i) snapshot.publish({i,~i}); finished.store(true); });
  while (!finished.load()) if (snapshot.read(observed)) assert(observed.sequence == ~observed.inverse);
  writer.join();
  ControlPolicy policy;
  assert(!policy.lease_valid(100));
  policy.connected.store(true);
  policy.heartbeat_ns.store(1000);
  assert(!policy.lease_valid(999));
  assert(policy.lease_valid(1000));
  assert(policy.lease_valid(1000 + policy.lease_ns - 1));
  assert(!policy.lease_valid(1000 + policy.lease_ns));
  timespec sample{};
  assert(clock_gettime(CLOCK_MONOTONIC, &sample) == 0);
  const uint64_t cycle_start = uint64_t(sample.tv_sec) * 1000000000ULL + uint64_t(sample.tv_nsec);
  do { assert(clock_gettime(CLOCK_MONOTONIC, &sample) == 0); }
  while (uint64_t(sample.tv_sec) * 1000000000ULL + uint64_t(sample.tv_nsec) <= cycle_start);
  policy.heartbeat_ns.store(uint64_t(sample.tv_sec) * 1000000000ULL + uint64_t(sample.tv_nsec));
  assert(!policy.lease_valid(cycle_start)); // A cycle-start timestamp is stale after a concurrent refresh.
  assert(policy.lease_valid_now());
  policy.heartbeat_ns.store(std::numeric_limits<uint64_t>::max());
  assert(!policy.lease_valid_now()); // Actual future clock values still fail closed.
  policy.config_verified_mask.store(2);
  assert(!policy.axis_ready(0, false));
  assert(policy.axis_ready(1, false));
  assert(!policy.axis_ready(1, true));
  policy.home_valid_mask.store(2);
  assert(policy.axis_ready(1, true));
  const auto epoch = policy.generation.load();
  assert(policy.revoke() == epoch + 1);
  AxisLimits limits{-1, 1, .2, true, true};
  assert(limits.contains(-1) && limits.contains(1));
  assert(!limits.contains(1.0001));
  assert(!limits.contains(std::numeric_limits<double>::quiet_NaN()));
  assert(!limits.allows_velocity(std::numeric_limits<double>::infinity()));
  assert(limits.allows_velocity(-.2));
  // Protocol 1.15: 24-byte envelope + 320-byte point with input-monitor masks.
  assert(command_message_size(gradient::ipc::v1::MSG_CMD_TRAJECTORY_POINT) == 344);
  assert(command_message_size(gradient::ipc::v1::MSG_CMD_SET_SOFT_LIMITS) == 0);
  assert(command_message_size(0xffff) == 0);

  assert(continuous_start(100.5, 100));
  assert(!continuous_start(102, 100));
  assert(!continuous_start(0, 1000)); // A whole turn is not an equivalent start.
  assert(!continuous_start(std::numeric_limits<double>::quiet_NaN(), 0));
  assert(future_deadline(1000, 999, 100) == 1000);
  assert(future_deadline(1000, 1000, 100) == 1100);
  assert(future_deadline(1000, 9085, 100) == 9100);

  // 0.1 rad/s at 100,000 counts/rad. Quantized samples at the limit pass;
  // neither zero feedforward nor a repeating extra count may exceed it.
  PositionRateGuard rate;
  rate.reset(0, 1000000);
  for (uint64_t i = 1; i <= 100; ++i)
    assert(rate.accept(double(i * 10), 1000000 + i * 1000000, 10000));
  rate.reset(0, 1000000);
  assert(!rate.accept(100, 2000000, 10000));
  rate.reset(0, 1000000);
  assert(rate.accept(11, 2000000, 10000));
  assert(!rate.accept(22, 3000000, 10000));
  rate.reset(999, 1000000);
  assert(!rate.accept(0, 2000000, 1000)); // A turn-sized jump is not folded.
  rate.reset(0, 1000000);
  assert(!rate.accept(0, 1000000, 10000));
  for (const auto& test : std::array<std::pair<const char*, const char*>, 6>{{
      {"u8", "255"}, {"i8", "-128"}, {"u16", "65535"},
      {"i16", "-32768"}, {"u32", "4294967295"}, {"i32", "-2147483648"}}}) {
    const auto type = startup_sdo_parse_type(test.first);
    int64_t value = 0;
    assert(startup_sdo_parse_value(type, test.second, &value));
    const auto raw = startup_sdo_encode(type, value);
    assert(startup_sdo_decode(type, raw.data()) == value);
    assert(!startup_sdo_parse_value(type, "4294967296", &value));
  }
}
