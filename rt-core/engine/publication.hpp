#pragma once
#include <array>
#include <atomic>
#include <cstring>
#include <type_traits>
#include <cstdint>

namespace rosie::rtcore {
static_assert(std::atomic<uint64_t>::is_always_lock_free, "RT snapshots require lock-free 64-bit atomics");
template<class T, size_t Capacity>
class SpscQueue {
 public:
  bool push(const T& value) noexcept {
    const auto written = write_.load(std::memory_order_relaxed);
    if (written - read_.load(std::memory_order_acquire) == Capacity) return false;
    entries_[written % Capacity] = value;
    write_.store(written + 1, std::memory_order_release);
    return true;
  }
  bool pop(T& value) noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) return false;
    value = entries_[read % Capacity];
    read_.store(read + 1, std::memory_order_release);
    return true;
  }
 private:
  std::array<T, Capacity> entries_{};
  std::atomic<uint64_t> write_{0}, read_{0};
};

// One writer; readers never race non-atomic storage. A failed bounded read is
// reported as unavailable, never as a newly sampled collection of mixed fields.
template<class T>
class AtomicSnapshot {
  static_assert(std::is_trivially_copyable_v<T> && sizeof(T) % 8 == 0);
 public:
  void publish(const T& value) noexcept {
    std::array<uint64_t, sizeof(T) / 8> raw{};
    std::memcpy(raw.data(), &value, sizeof(T));
    seq_.fetch_add(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    for (size_t i = 0; i < raw.size(); ++i) words_[i].store(raw[i], std::memory_order_relaxed);
    seq_.fetch_add(1, std::memory_order_release);
  }
  bool read(T& value) const noexcept {
    for (unsigned attempt = 0; attempt < 8; ++attempt) {
      const auto seq = seq_.load(std::memory_order_acquire);
      if (!seq || (seq & 1)) continue;
      std::array<uint64_t, sizeof(T) / 8> raw{};
      for (size_t i = 0; i < raw.size(); ++i) raw[i] = words_[i].load(std::memory_order_relaxed);
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq == seq_.load(std::memory_order_relaxed)) {
        std::memcpy(&value, raw.data(), sizeof(T));
        return true;
      }
    }
    return false;
  }
 private:
  std::atomic<uint64_t> seq_{0};
  std::array<std::atomic<uint64_t>, sizeof(T) / 8> words_{};
};
// Freeze asynchronous acquisitions before taking their publication clock. Reading
// a source again during formatting can otherwise pair a newer completion with
// an older timestamp and falsely report an ordinary writer race as future data.
// This is a per-source coherent collection, not a simultaneous multi-axis sample.
template<class T, size_t Capacity>
struct TimedSnapshotBatch {
  std::array<T, Capacity> values{};
  std::array<bool, Capacity> available{};
  uint64_t time_ns = 0;
};

template<class T, size_t Capacity, class Clock>
TimedSnapshotBatch<T, Capacity> snapshot_before_clock(
    const std::array<AtomicSnapshot<T>, Capacity>& sources, size_t count, Clock clock) {
  TimedSnapshotBatch<T, Capacity> result{};
  for (size_t i = 0; i < count && i < Capacity; ++i)
    result.available[i] = sources[i].read(result.values[i]);
  result.time_ns = clock();
  return result;
}
} // namespace rosie::rtcore
