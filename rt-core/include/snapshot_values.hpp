#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rosie::rtcore {
// Read one coherent process-local value from a single writer's publication.
// The writer marks sequence odd before mutation and release-publishes an even
// value afterward. Zero is unpublished; the sequence must never wrap or reset
// while readers can access it. Every concurrently accessed payload field must
// be atomic: use release stores/acquire reads, or a writer release fence after
// the odd mark with atomic payload accesses. Plain seqlock copies are data races.
//
// read(candidate) must be bounded and nonthrowing, copy every needed field using
// that atomic protocol, and leave shared storage untouched. The caller owns the
// output's lifetime and synchronization. Failure preserves its previous value;
// this helper neither treats that previous value as fresh nor retries forever.
template<class T, class Reader>
[[nodiscard]] bool read_sequenced_snapshot(const std::atomic<uint64_t>& sequence,
                                         T& output, Reader&& read) noexcept {
  static_assert(std::atomic<uint64_t>::is_always_lock_free);
  static_assert(std::is_trivially_copyable_v<T> && std::is_nothrow_default_constructible_v<T> &&
                std::is_nothrow_copy_assignable_v<T>, "snapshot output must be a nonthrowing value type");
  static_assert(std::is_nothrow_invocable_v<Reader&, T&>, "snapshot reader must be nonthrowing");
  for (unsigned attempt = 0; attempt < 8; ++attempt) {
    const auto before = sequence.load(std::memory_order_acquire);
    if (!before || (before & 1u)) continue;
    T candidate{};
    read(candidate);
    std::atomic_thread_fence(std::memory_order_acquire);
    if (sequence.load(std::memory_order_acquire) != before) continue;
    output = candidate;
    return true;
  }
  return false;
}

// Diagnostic and mailbox fields have multiple non-RT readers. A publication
// sequence alone does not make concurrent accesses to plain C++ fields legal.
// These small cells prevent data races without putting a blocking lock in the
// cyclic writer. Group coherence is provided by the enclosing publication seq.
template<class T> class SnapshotValue {
  static_assert(std::atomic<T>::is_always_lock_free, "RT snapshot cells must be lock-free");
  std::atomic<T> value_{};
public:
  SnapshotValue() = default;
  SnapshotValue(T value) noexcept : value_(value) {}
  SnapshotValue(const SnapshotValue& other) noexcept : value_(other.load()) {}
  SnapshotValue& operator=(T value) noexcept { value_.store(value, std::memory_order_release); return *this; }
  SnapshotValue& operator=(const SnapshotValue& other) noexcept { return *this = other.load(); }
  T load() const noexcept { return value_.load(std::memory_order_acquire); }
  operator T() const noexcept { return load(); }
};

template<class T, std::size_t N> class SnapshotArray {
  std::array<SnapshotValue<T>, N> values_{};
public:
  auto begin() const noexcept { return values_.begin(); }
  auto end() const noexcept { return values_.end(); }
  SnapshotValue<T>& operator[](std::size_t i) noexcept { return values_[i]; }
  const SnapshotValue<T>& operator[](std::size_t i) const noexcept { return values_[i]; }
  operator std::array<T, N>() const noexcept {
    std::array<T,N> result{};
    for(std::size_t i=0;i<N;++i) result[i]=values_[i].load();
    return result;
  }
  SnapshotArray& operator=(const std::array<T,N>& other) noexcept {
    for(std::size_t i=0;i<N;++i) values_[i]=other[i];
    return *this;
  }
  void fill(T value) noexcept { for(auto& item:values_) item=value; }
};
template<class T> T log_value(const T& value) { return value; }
template<class T> T log_value(const SnapshotValue<T>& value) { return value.load(); }
}
