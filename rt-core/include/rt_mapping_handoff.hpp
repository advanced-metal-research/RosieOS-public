#pragma once

#include <atomic>
#include <cstdint>
#include <limits>

namespace rosie::rtcore {

// One off-cycle owner publishes/retires connection mappings; one RT reader
// borrows them for a complete cycle. This object is process-local, not an mmap
// ABI. A disconnect must detach, wait for take_retired(), then unmap/delete the
// returned region. A timeout never grants permission to unmap.
template<class Region>
class RtMappingHandoff {
  static_assert(std::atomic<Region*>::is_always_lock_free);
  static_assert(std::atomic<uint64_t>::is_always_lock_free);
  static_assert(std::atomic<bool>::is_always_lock_free);

 public:
  class Cycle {
   public:
    explicit Cycle(RtMappingHandoff& owner) noexcept : owner_(owner) {
      // Nested scopes must not acknowledge an outer borrow. They receive no
      // pointer and perform no completion; this is still a single-reader API.
      if (owner_.cycle_active_) return;
      owner_.cycle_active_ = true;
      owns_cycle_ = true;
      if (!owner_.reader_stopped_.load(std::memory_order_seq_cst))
        region_ = owner_.published_.load(std::memory_order_seq_cst);
    }
    ~Cycle() {
      if (owns_cycle_) {
        owner_.cycle_active_ = false;
        owner_.complete_cycle();
      }
    }
    Cycle(const Cycle&) = delete;
    Cycle& operator=(const Cycle&) = delete;
    Region* get() const noexcept { return region_; }
   private:
    RtMappingHandoff& owner_;
    Region* region_ = nullptr;
    bool owns_cycle_ = false;
  };

  // Owner only. A detached mapping must be reclaimed before publishing another
  // one, so a restarted connection cannot reuse storage still borrowed by RT.
  bool publish(Region* region) noexcept {
    if (!region || pending_ || reader_stopped_.load(std::memory_order_seq_cst)) return false;
    Region* empty = nullptr;
    return published_.compare_exchange_strong(empty, region, std::memory_order_seq_cst);
  }

  // Owner only; repeated detach leaves the original retirement ticket intact.
  void detach() noexcept {
    if (pending_) return;
    pending_ = published_.exchange(nullptr, std::memory_order_seq_cst);
    if (pending_) after_cycle_ = completed_cycles_.load(std::memory_order_seq_cst);
  }

  // Owner only. Observing a completed cycle strictly AFTER the detach ticket
  // proves any reader which could have loaded the old pointer has finished.
  // Readers starting afterward see null. Waiting an extra cycle is harmless.
  Region* take_retired() noexcept {
    if (!pending_) return nullptr;
    if (!reader_stopped_.load(std::memory_order_seq_cst) &&
        completed_cycles_.load(std::memory_order_seq_cst) <= after_cycle_) return nullptr;
    Region* retired = pending_;
    pending_ = nullptr;
    return retired;
  }

  // RT reader only, after its last Cycle scope has ended. Once called the
  // reader may never borrow this handoff again; a new runtime needs a new one.
  bool stop_reader() noexcept {
    if (cycle_active_) return false;
    reader_stopped_.store(true, std::memory_order_seq_cst);
    return true;
  }

 private:
  void complete_cycle() noexcept {
    const uint64_t current = completed_cycles_.load(std::memory_order_seq_cst);
    if (current == std::numeric_limits<uint64_t>::max()) {
      // End this mapping reader permanently instead of wrapping a reclamation
      // ticket. No mapping is accessed after the Cycle destructor begins.
      stop_reader();
      return;
    }
    completed_cycles_.store(current + 1, std::memory_order_seq_cst);
  }

  std::atomic<Region*> published_{nullptr};
  std::atomic<uint64_t> completed_cycles_{0};
  std::atomic<bool> reader_stopped_{false};
  bool cycle_active_ = false; // RT-reader-only; pointers never escape their Cycle.
  Region* pending_ = nullptr; // Owner-thread-only retirement state.
  uint64_t after_cycle_ = 0;
};

} // namespace rosie::rtcore
