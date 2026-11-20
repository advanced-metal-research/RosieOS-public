#include "connection_failure.hpp"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <thread>

namespace {
using namespace rosie::rtcore;
using Published = ConnectionFailurePublication;
using Consumed = ConnectionFailureConsumption;
using Reason = ConnectionFailureReason;

void expect(bool condition, const char* message) {
  if (!condition) { std::fprintf(stderr, "%s\n", message); std::abort(); }
}

void terminal_identity_and_reason() {
  ConnectionFailureLatch latch;
  ConnectionFailure out{999, Reason::SequenceExhausted};
  expect(latch.consume(1, out) == Consumed::Empty && out.connection_id == 999,
         "empty consumption must preserve caller output");
  expect(latch.publish(0, Reason::Busy) == Published::Invalid,
         "zero is not a connection identity");
  expect(latch.publish(1, Reason::None) == Published::Invalid &&
         latch.publish(1, static_cast<Reason>(255)) == Published::Invalid,
         "only terminal failure reasons can be published");

  expect(latch.publish(41, Reason::Busy) == Published::Published, "publish first failure");
  expect(latch.publish(41, Reason::SequenceExhausted) == Published::Duplicate,
         "duplicate must not replace original reason");
  expect(latch.publish(40, Reason::Busy) == Published::Stale,
         "earlier identity cannot replace pending failure");
  expect(latch.publish(42, Reason::SequenceExhausted) == Published::Occupied,
         "successor cannot overwrite pending failure");
  expect(latch.consume(41, out) == Consumed::Matched && out.connection_id == 41 &&
         out.reason == Reason::Busy, "matching consume returns original complete record");
  expect(latch.publish(41, Reason::Busy) == Published::Duplicate &&
         latch.consume(41, out) == Consumed::Empty,
         "consumption must not permit repeated terminal side effects");

  expect(latch.publish(42, Reason::SequenceExhausted) == Published::Published,
         "occupied attempt can retry after the previous record is consumed");
  const auto before = out;
  expect(latch.consume(41, out) == Consumed::Future && out.connection_id == before.connection_id &&
         out.reason == before.reason, "old consumer context cannot erase a successor notification");
  expect(latch.publish(43, Reason::Busy) == Published::Occupied,
         "future notification remains pending after stale consumer attempt");
  expect(latch.consume(43, out) == Consumed::Stale && out.connection_id == before.connection_id &&
         out.reason == before.reason, "late old failure must not be applied to a successor");
  expect(latch.publish(42, Reason::Busy) == Published::Duplicate &&
         latch.publish(41, Reason::Busy) == Published::Stale,
         "stale discard must preserve producer terminal history");
  expect(latch.publish(43, Reason::SequenceExhausted) == Published::Published &&
         latch.consume(43, out) == Consumed::Matched && out.connection_id == 43 &&
         out.reason == Reason::SequenceExhausted,
         "fresh successor failure remains observable after stale discard");

  expect(latch.publish(44, Reason::InvalidPublisher) == Published::Published &&
         latch.consume(0, out) == Consumed::Future && out.connection_id == 43,
         "no active context cannot consume a future connection's terminal action");
  expect(latch.consume(44, out) == Consumed::Matched && out.connection_id == 44 &&
         out.reason == Reason::InvalidPublisher, "invalid publisher is a distinct terminal reason");
  const auto maximum = std::numeric_limits<uint64_t>::max();
  expect(latch.publish(maximum, Reason::Busy) == Published::Published &&
         latch.consume(maximum, out) == Consumed::Matched && out.connection_id == maximum,
         "last nonzero identity remains representable");
  expect(latch.publish(1, Reason::Busy) == Published::Stale &&
         latch.publish(maximum, Reason::Busy) == Published::Duplicate,
         "identity wrap or reuse cannot start another lifetime");
}

void concurrent_handoff_preserves_reason_and_successor() {
  constexpr uint64_t count = 100000;
  ConnectionFailureLatch latch;
  std::atomic<uint64_t> consumed{0};
  std::thread producer([&] {
    for (uint64_t id = 1; id <= count; ++id) {
      const auto reason = id & 1 ? Reason::Busy : Reason::SequenceExhausted;
      Published result;
      do {
        result = latch.publish(id, reason);
        expect(result == Published::Published || result == Published::Occupied,
               "only a full single slot may defer the next publication");
        if (result == Published::Occupied) std::this_thread::yield();
      } while (result != Published::Published);
      // The consumer can already have cleared the slot at this point.
      expect(latch.publish(id, reason) == Published::Duplicate,
             "concurrent consumption cannot cause duplicate publication");
    }
  });
  std::thread consumer([&] {
    for (uint64_t id = 1; id <= count; ++id) {
      ConnectionFailure out{0, Reason::None};
      Consumed result;
      // Every third record simulates notification delivery after reconnect.
      const bool stale = id % 3 == 0;
      do {
        result = latch.consume(stale ? id + 1 : id, out);
        if (result == Consumed::Empty) std::this_thread::yield();
      } while (result == Consumed::Empty);
      if (stale) {
        expect(result == Consumed::Stale && out.connection_id == 0 && out.reason == Reason::None,
               "stale concurrent notification must not leak a terminal action");
      } else {
        expect(result == Consumed::Matched && out.connection_id == id &&
               out.reason == (id & 1 ? Reason::Busy : Reason::SequenceExhausted),
               "concurrent handoff must preserve matching ID and reason");
      }
      consumed.fetch_add(1, std::memory_order_relaxed);
    }
  });
  producer.join();
  consumer.join();
  expect(consumed.load() == count, "all published records must be consumed exactly once");
  ConnectionFailure out;
  expect(latch.consume(count, out) == Consumed::Empty, "last record was consumed");
}
} // namespace

int main() {
  terminal_identity_and_reason();
  concurrent_handoff_preserves_reason_and_successor();
  std::puts("connection failure latch tests passed");
}
