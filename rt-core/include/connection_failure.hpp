#pragma once

#include <atomic>
#include <cstdint>

namespace rosie::rtcore {

enum class ConnectionFailureReason : uint8_t { None, Busy, SequenceExhausted, InvalidPublisher };

struct ConnectionFailure {
  uint64_t connection_id = 0;
  ConnectionFailureReason reason = ConnectionFailureReason::None;
};

enum class ConnectionFailurePublication : uint8_t {
  Published, Duplicate, Stale, Occupied, Invalid,
};
enum class ConnectionFailureConsumption : uint8_t { Empty, Matched, Stale, Future };

// Process-local, one RT producer and one accept-loop consumer. Connection IDs
// must be nonzero, strictly increasing and never reused within this object's
// lifetime. A terminal failure is published at most once for a connection, even
// after consumption. Destroy only after both threads have stopped using it.
//
// The consumer copies the payload before release-clearing the slot. The producer
// acquires that empty state before writing the next reason, then release-publishes
// its ID. These two handoffs protect the ordinary reason field in both directions;
// neither thread can access it concurrently with the other's write.
class ConnectionFailureLatch {
  static_assert(std::atomic<uint64_t>::is_always_lock_free);

 public:
  [[nodiscard]] ConnectionFailurePublication publish(
      uint64_t connection_id, ConnectionFailureReason reason) noexcept {
    if (!connection_id || (reason != ConnectionFailureReason::Busy &&
                           reason != ConnectionFailureReason::SequenceExhausted &&
                           reason != ConnectionFailureReason::InvalidPublisher))
      return ConnectionFailurePublication::Invalid;
    if (connection_id == last_published_id_) return ConnectionFailurePublication::Duplicate;
    if (connection_id < last_published_id_) return ConnectionFailurePublication::Stale;
    if (published_id_.load(std::memory_order_acquire))
      return ConnectionFailurePublication::Occupied;
    reason_ = reason;
    last_published_id_ = connection_id;
    published_id_.store(connection_id, std::memory_order_release);
    return ConnectionFailurePublication::Published;
  }

  // Consumer only. Stale notifications are drained, never applied to a successor.
  // A future ID remains queued: an old consumer context cannot erase a successor
  // notification. Output is unchanged unless Matched. Occupied publication must
  // be retried after consumption; it never overwrites another pending failure.
  [[nodiscard]] ConnectionFailureConsumption consume(
      uint64_t current_connection_id, ConnectionFailure& output) noexcept {
    const uint64_t connection_id = published_id_.load(std::memory_order_acquire);
    if (!connection_id) return ConnectionFailureConsumption::Empty;
    if (connection_id > current_connection_id) return ConnectionFailureConsumption::Future;
    const bool matches = connection_id == current_connection_id;
    if (matches) output = {connection_id, reason_};
    published_id_.store(0, std::memory_order_release);
    return matches ? ConnectionFailureConsumption::Matched : ConnectionFailureConsumption::Stale;
  }

 private:
  std::atomic<uint64_t> published_id_{0};
  uint64_t last_published_id_ = 0; // RT producer only, including after consumption.
  ConnectionFailureReason reason_ = ConnectionFailureReason::None;
};

} // namespace rosie::rtcore
