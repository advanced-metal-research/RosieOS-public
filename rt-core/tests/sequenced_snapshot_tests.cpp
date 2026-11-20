#include "snapshot_values.hpp"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {
using rosie::rtcore::read_sequenced_snapshot;
using rosie::rtcore::SnapshotValue;

void expect(bool condition, const char* message) {
  if (!condition) { std::fprintf(stderr, "%s\n", message); std::abort(); }
}

struct Value {
  uint32_t mask = 0;
  int32_t position = 0;
  uint64_t timestamp = 0;
};

bool same(const Value& a, const Value& b) {
  return a.mask == b.mask && a.position == b.position && a.timestamp == b.timestamp;
}

struct Publication {
  std::atomic<uint64_t> sequence{0};
  SnapshotValue<uint32_t> mask{0};
  SnapshotValue<int32_t> position{0};
  std::atomic<uint64_t> timestamp{0};

  void publish(uint64_t sample) noexcept {
    sequence.fetch_add(1, std::memory_order_acq_rel);
    mask = sample & 1 ? 1u : 2u;
    position = static_cast<int32_t>(sample * 100);
    timestamp.store(sample * 10, std::memory_order_release);
    sequence.fetch_add(1, std::memory_order_release);
  }
  void read(Value& output) const noexcept {
    output.mask = mask.load();
    output.position = position.load();
    output.timestamp = timestamp.load(std::memory_order_acquire);
  }
};

void unpublished_and_odd_never_copy() {
  for (uint64_t unavailable : {uint64_t{0}, uint64_t{3}}) {
    std::atomic<uint64_t> sequence{unavailable};
    Value output{17, -42, 777};
    const auto before = output;
    unsigned calls = 0;
    expect(!read_sequenced_snapshot(sequence, output, [&](Value& candidate) noexcept {
      ++calls;
      candidate = {1, 100, 10};
    }), "unpublished/odd snapshot must be refused");
    expect(calls == 0 && same(output, before), "unpublished/odd must not invoke reader or alter output");
  }
}

void coherent_value_and_mid_copy_mutation() {
  Publication source;
  source.publish(1);
  Value output{17, -42, 777};
  unsigned calls = 0;
  expect(read_sequenced_snapshot(source.sequence, output, [&](Value& candidate) noexcept {
    ++calls;
    source.read(candidate);
  }) && calls == 1 && same(output, {1, 100, 10}), "stable committed value must be accepted");

  const auto before = output;
  uint64_t sample = 1;
  calls = 0;
  expect(!read_sequenced_snapshot(source.sequence, output, [&](Value& candidate) noexcept {
    ++calls;
    candidate.mask = source.mask.load(); // Old validity mask.
    // Deliberate test-only scheduling seam: the sole writer completes another
    // publication between the old mask and new position/timestamp reads.
    source.publish(++sample);
    candidate.position = source.position.load();
    candidate.timestamp = source.timestamp.load(std::memory_order_acquire);
  }), "old mask combined with new data must never be accepted");
  expect(calls == 8 && sample == 9 && same(output, before),
         "mutation retries must be bounded to eight and preserve prior output");

  calls = 0;
  expect(read_sequenced_snapshot(source.sequence, output, [&](Value& candidate) noexcept {
    ++calls;
    source.read(candidate);
  }) && calls == 1 && same(output, {1, 900, 90}), "stable publication after contention must recover");
}

void concurrent_atomic_publications_remain_coherent() {
  Publication source;
  constexpr uint64_t last_sample = 100000;
  std::atomic<bool> begin{false}, done{false};
  std::thread writer([&] {
    while (!begin.load(std::memory_order_acquire)) std::this_thread::yield();
    for (uint64_t sample = 1; sample <= last_sample; ++sample) source.publish(sample);
    done.store(true, std::memory_order_release);
  });
  Value output{17, -42, 777};
  auto read = [&](Value& candidate) noexcept { source.read(candidate); };
  begin.store(true, std::memory_order_release);
  do {
    const auto before = output;
    if (read_sequenced_snapshot(source.sequence, output, read)) {
      const auto sample = output.timestamp / 10;
      expect(sample && sample <= last_sample && output.timestamp == sample * 10 &&
             output.position == static_cast<int32_t>(sample * 100) &&
             output.mask == (sample & 1 ? 1u : 2u),
             "accepted concurrent snapshot mixes distinct publications");
    } else {
      expect(same(output, before), "refused concurrent snapshot altered previous output");
    }
  } while (!done.load(std::memory_order_acquire));
  writer.join();
  expect(read_sequenced_snapshot(source.sequence, output, read) &&
         same(output, {2, 10000000, 1000000}), "final committed publication must remain readable");
}
} // namespace

int main() {
  unpublished_and_odd_never_copy();
  coherent_value_and_mid_copy_mutation();
  concurrent_atomic_publications_remain_coherent();
  std::puts("sequenced snapshot tests passed");
}
