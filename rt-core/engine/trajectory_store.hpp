#pragma once
#include "trajectory.hpp"
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>
#include <memory>
#include <stdexcept>
#include <unistd.h>

namespace rosie::rtcore {

// The helper owns fixed storage except while a slot is Ready or Active.
// The cyclic owner only acquires a pointer and retires it; it never allocates,
// copies a plan, frees memory or changes a reference count. Three slots bound
// residency to one executing plan plus two staged plans. Each plan is separately
// bounded by admission's maximum sample count.
template<class Point, size_t Capacity = kResidentPlanSlots>
class TrajectoryStore {
 public:
  enum class State : uint8_t { Free, Writing, Prepared, Ready, Active, Retired };
  enum class DiscardResult : uint8_t { Discarded, InUse };
  class Points {
   public:
    const Point* data() const noexcept { return storage_.get(); }
    size_t size() const noexcept { return count_; }
    const Point& operator[](size_t index) const noexcept { return storage_[index]; }
   private:
    friend class TrajectoryStore;
    std::unique_ptr<Point[]> storage_;
    size_t count_ = 0;
  };
  struct Slot {
    std::atomic<State> state{State::Free};
    std::atomic<bool> abort_requested{false};
    uint64_t id = 0, generation = 0, start_sequence = 0;
    uint32_t axis_mask = 0;
    Points points;
  };
  struct CancelledStart {
    uint64_t id = 0, sequence = 0;
    uint32_t axis_mask = 0;
  };
  struct AbortResult {
    std::array<CancelledStart, Capacity> starts{};
    size_t count = 0;
  };

  // Allocate every slot before worker startup; reuse never changes its address
  // or allocates plan data, preventing a fourth full-size upload buffer.
  explicit TrajectoryStore(size_t max_points = protocol::max_trajectory_points)
      : max_points_(max_points) {
    if (!max_points_) throw std::invalid_argument("empty trajectory storage");
    for (auto& slot : slots_) slot.points.storage_ = std::make_unique<Point[]>(max_points_);
  }

  // Startup only, after live mlockall and before publication. Volatile writes
  // establish writable residency for every page, preventing demand/COW faults
  // when the bulk worker fills a slot or RT samples it for the first time.
  void prefault() noexcept {
    const auto page_size = static_cast<size_t>(::sysconf(_SC_PAGESIZE));
    for (auto& slot : slots_) {
      auto* bytes = reinterpret_cast<volatile unsigned char*>(slot.points.storage_.get());
      const size_t count = max_points_ * sizeof(Point);
      for (size_t offset = 0; offset < count; offset += page_size) bytes[offset] = bytes[offset];
      bytes[count - 1] = bytes[count - 1];
    }
  }

  // Bulk owner only. BEGIN reserves capacity before decoding; Writing is never
  // visible to Start or RT, so incomplete points cannot become executable.
  int begin(uint64_t id, uint64_t generation, uint32_t mask, size_t count) noexcept {
    if (!count || count > max_points_) return -1;
    if (find(id, generation) >= 0) return -1;
    for (size_t i = 0; i < Capacity; ++i) {
      auto& slot = slots_[i];
      auto state = slot.state.load(std::memory_order_acquire);
      if (state != State::Free && state != State::Retired) continue;
      if (!slot.state.compare_exchange_strong(state, State::Writing, std::memory_order_acq_rel)) continue;
      slot.points.count_ = count;
      slot.id = id; slot.generation = generation; slot.axis_mask = mask;
      slot.start_sequence = 0;
      slot.abort_requested.store(false, std::memory_order_relaxed);
      return static_cast<int>(i);
    }
    return -1;
  }

  // Bulk owner only, for an outstanding BEGIN. No mutable pointer is returned
  // for a published slot, preventing writes racing with cyclic sampling.
  Point* writing_data(int index) noexcept {
    if (index < 0 || static_cast<size_t>(index) >= Capacity ||
        slots_[index].state.load(std::memory_order_acquire) != State::Writing) return nullptr;
    return slots_[index].points.storage_.get();
  }

  // Publish only after all points were decoded and validated in place. The
  // release transition transfers immutable ownership without a copy or move.
  bool commit(int index) noexcept {
    if (index < 0 || static_cast<size_t>(index) >= Capacity) return false;
    auto writing = State::Writing;
    return slots_[index].state.compare_exchange_strong(writing, State::Prepared,
                                                       std::memory_order_release);
  }

  // Bulk owner only. Failed/abandoned uploads release capacity but retain the
  // allocation; calling this after COMMIT cannot retire a published plan.
  void abandon(int index) noexcept {
    if (index < 0 || static_cast<size_t>(index) >= Capacity) return;
    auto writing = State::Writing;
    slots_[index].state.compare_exchange_strong(writing, State::Free, std::memory_order_release);
  }

  // Compatibility for callers with existing samples (including the frozen
  // store assertions). Production uploads use BEGIN/writing_data/COMMIT.
  int prepare(uint64_t id, uint64_t generation, uint32_t mask, const std::vector<Point>& points) {
    const int index = begin(id, generation, mask, points.size());
    if (index < 0) return -1;
    try { std::copy(points.begin(), points.end(), writing_data(index)); }
    catch (...) { abandon(index); throw; }
    commit(index);
    return index;
  }

  // Helper only. The state is release-published after all immutable fields.
  bool start(uint64_t id, uint64_t generation, uint64_t sequence) noexcept {
    const int index = find(id, generation);
    if (index < 0) return false;
    auto& slot = slots_[index];
    if (slot.state.load(std::memory_order_acquire) != State::Prepared) return false;
    slot.start_sequence = sequence;
    slot.state.store(State::Ready, std::memory_order_release);
    return true;
  }

  // Helper only. Discard releases preparation storage without cancelling a
  // published Start or active execution. Absent, retired and other-generation
  // handles are already discarded. Ready/Active storage remains untouched.
  DiscardResult discard(uint64_t generation, uint64_t id) noexcept {
    const int index = find(id, generation);
    if (index < 0) return DiscardResult::Discarded;
    auto& slot = slots_[index];
    auto prepared = State::Prepared;
    if (slot.state.compare_exchange_strong(prepared, State::Retired, std::memory_order_acq_rel))
      return DiscardResult::Discarded;
    return prepared == State::Ready || prepared == State::Active
        ? DiscardResult::InUse : DiscardResult::Discarded;
  }

  // Cyclic owner only. Always bounded by Capacity, independent of plan size.
  int acquire_ready() noexcept {
    for (size_t i = 0; i < Capacity; ++i) {
      auto ready = State::Ready;
      if (slots_[i].state.compare_exchange_strong(ready, State::Active, std::memory_order_acq_rel))
        return static_cast<int>(i);
    }
    return -1;
  }
  const Slot& view(int index) const noexcept { return slots_[index]; }
  bool abort_requested(int index) const noexcept {
    return index >= 0 && slots_[index].abort_requested.load(std::memory_order_acquire);
  }
  void retire(int index) noexcept {
    if (index >= 0) slots_[index].state.store(State::Retired, std::memory_order_release);
  }

  // Helper only. CAS retires matching preparations unless RT activation wins.
  // An acquired slot keeps its storage until RT observes this per-slot request
  // and retires it. The request cannot be lost through an ID-only mailbox or
  // accidentally cancel a later upload that reuses the same numeric handle.
  // Unknown handles are an idempotent no-op; zero aborts this generation's plans.
  AbortResult abort(uint64_t generation, uint64_t id = 0) noexcept {
    AbortResult result{};
    for (auto& slot : slots_) {
      auto state = slot.state.load(std::memory_order_acquire);
      if (state != State::Prepared && state != State::Ready && state != State::Active) continue;
      if (slot.generation != generation || (id != 0 && slot.id != id)) continue;
      if (state == State::Prepared || state == State::Ready) {
        if (slot.state.compare_exchange_strong(state, State::Retired, std::memory_order_acq_rel)) {
          // A Ready slot's Start has not reached RT, so its deferred result
          // must be returned by the helper that cancelled its publication.
          if (state == State::Ready)
            result.starts[result.count++] = {slot.id, slot.start_sequence, slot.axis_mask};
          continue;
        }
      }
      if (state == State::Active) slot.abort_requested.store(true, std::memory_order_release);
    }
    return result;
  }

  // Helper only. Active storage is retired exclusively by the cyclic owner.
  // CAS prevents invalidation racing with Ready -> Active acquisition.
  void invalidate(uint64_t generation, uint64_t id = 0) noexcept {
    for (auto& slot : slots_) {
      auto state = slot.state.load(std::memory_order_acquire);
      if (state != State::Prepared && state != State::Ready) continue;
      if ((id != 0 && slot.id == id) || slot.generation != generation)
        slot.state.compare_exchange_strong(state, State::Retired, std::memory_order_acq_rel);
    }
  }

 private:
  int find(uint64_t id, uint64_t generation) const noexcept {
    for (size_t i = 0; i < Capacity; ++i) {
      const auto state = slots_[i].state.load(std::memory_order_acquire);
      if (state != State::Free && state != State::Retired &&
          slots_[i].id == id && slots_[i].generation == generation) return static_cast<int>(i);
    }
    return -1;
  }
  std::array<Slot, Capacity> slots_{};
  const size_t max_points_;
};
// Measured LP64 layout: 3 slots * (250,000 points * 320 bytes + 56-byte slot
// header) = 240,000,168 bytes. Allocator/page rounding belongs to daemon overhead.
inline constexpr size_t kResidentPlanBytes = kResidentPlanSlots *
    (protocol::max_trajectory_points * sizeof(TrajectoryPoint) +
     sizeof(TrajectoryStore<TrajectoryPoint>::Slot));

// Startup only. Perf sampling measured an instruction-page fault at the first
// sample_trajectory entry when simulation mlockall was refused. Exercise its
// interpolation, feedforward and endpoint code before accepting work;
// the volatile function pointer preserves the same out-of-line entry RT calls.
inline void prefault_trajectory_sampling() noexcept {
  std::array<TrajectoryPoint, 2> points{};
  points[0].axis_mask = points[1].axis_mask = (1u << protocol::max_axes) - 1;
  points[1].t_from_start_ns = 2;
  auto* volatile sampler = &sample_trajectory;
  (void)sampler(points.data(), points.size(), 1);
  points[0].flags = points[1].flags = 1;
  (void)sampler(points.data(), points.size(), 1);
  (void)sampler(points.data(), points.size(), 2);
}
} // namespace rosie::rtcore
