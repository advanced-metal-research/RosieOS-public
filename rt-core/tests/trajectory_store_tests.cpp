// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "engine/trajectory_store.hpp"
#include <cassert>
#include <thread>

using Store = rosie::rtcore::TrajectoryStore<int, 1>;
static_assert(rosie::rtcore::kResidentPlanBytes == 240'000'168,
              "update the independent Go resident budget if the slot layout changes");

// Deleted copy/move operations make accidental staging transfers a build error.
struct InPlacePoint {
  int value = 0;
  InPlacePoint() = default;
  InPlacePoint(const InPlacePoint&) = delete;
  InPlacePoint& operator=(const InPlacePoint&) = delete;
};

void in_place_staging() {
  rosie::rtcore::prefault_trajectory_sampling();
  using InPlaceStore = rosie::rtcore::TrajectoryStore<InPlacePoint>;
  InPlaceStore store(3);
  store.prefault();
  assert(store.begin(1, 1, 1, 4) == -1);
  assert(store.begin(1, 1, 1, 0) == -1);
  assert(store.begin(1, 1, 1, 3) == 0);
  assert(store.view(0).state.load() == InPlaceStore::State::Writing);
  auto* original = store.writing_data(0);
  assert(original != nullptr);
  original[0].value = 123;
  original[2].value = 456;
  assert(store.begin(1, 1, 1, 3) == -1);
  assert(!store.start(1, 1, 10));
  assert(store.acquire_ready() == -1);
  assert(store.discard(1, 1) == InPlaceStore::DiscardResult::Discarded);
  assert(store.view(0).state.load() == InPlaceStore::State::Writing);
  assert(store.begin(2, 1, 1, 3) == 1);
  assert(store.begin(3, 1, 1, 3) == 2);
  assert(store.begin(4, 1, 1, 3) == -1); // BEGIN, not COMMIT, consumes capacity.
  store.abandon(0);
  assert(store.view(0).state.load() == InPlaceStore::State::Free);
  assert(!store.commit(0));
  assert(store.begin(4, 1, 1, 2) == 0);
  assert(store.writing_data(0) == original);
  assert(original[0].value == 123 && original[2].value == 456);
  original[1].value = 789;
  assert(store.commit(0));
  assert(store.commit(1));
  assert(store.commit(2));
  store.abandon(0); // A successful COMMIT survives upload-scope cleanup.
  assert(store.view(0).state.load() == InPlaceStore::State::Prepared);
  assert(store.view(0).points.size() == 2);
  assert(store.view(0).points.data() == original);
  assert(store.view(0).points[1].value == 789);
  assert(store.writing_data(0) == nullptr);
  assert(store.begin(5, 1, 1, 3) == -1);
  assert(store.start(4, 1, 11));
  assert(store.acquire_ready() == 0);
  assert(store.view(0).points.data() == original);
  store.retire(0);
  assert(store.begin(5, 2, 1, 3) == 0);
  assert(store.writing_data(0) == original);
  store.abandon(0);
  assert(store.writing_data(-1) == nullptr);
  assert(!store.commit(3));
}

int main() {
  in_place_staging();
  {
    Store prepared;
    std::vector<int> samples{1, 2};
    assert(prepared.discard(1, 42) == Store::DiscardResult::Discarded);
    assert(prepared.prepare(42, 1, 1, samples) == 0);
    assert(prepared.discard(2, 42) == Store::DiscardResult::Discarded);
    assert(prepared.view(0).state.load() == Store::State::Prepared);
    assert(prepared.discard(1, 42) == Store::DiscardResult::Discarded);
    assert(prepared.discard(1, 42) == Store::DiscardResult::Discarded);
    assert(!prepared.start(42, 1, 1));
    samples = {3, 4};
    assert(prepared.prepare(43, 1, 1, samples) == 0);
    assert(prepared.start(43, 1, 2));
    assert(prepared.discard(1, 43) == Store::DiscardResult::InUse);
    assert(prepared.view(0).state.load() == Store::State::Ready);
    assert(prepared.acquire_ready() == 0);
    assert(prepared.discard(1, 43) == Store::DiscardResult::InUse);
    assert(!prepared.abort_requested(0));
    assert(prepared.view(0).points[0] == 3);
    prepared.retire(0);
    assert(prepared.discard(1, 43) == Store::DiscardResult::Discarded);
    samples = {5, 6};
    assert(prepared.prepare(43, 2, 1, samples) == 0);
    assert(prepared.discard(1, 43) == Store::DiscardResult::Discarded);
    assert(prepared.start(43, 2, 3)); // Old incarnation cannot discard reuse.
  }
  Store store;
  std::vector<int> points{11, 22, 33};
  assert(store.prepare(7, 1, 3, points) == 0);
  assert(store.abort(2, 7).count == 0); // Wrong generation cannot cancel a handle.
  assert(store.start(7, 1, 70));
  const auto ready_abort = store.abort(1, 7);
  assert(ready_abort.count == 1);
  assert(ready_abort.starts[0].id == 7 && ready_abort.starts[0].sequence == 70);
  assert(ready_abort.starts[0].axis_mask == 3);
  assert(store.acquire_ready() == -1);
  assert(!store.start(7, 1, 71));

  points = {44, 55, 66};
  assert(store.prepare(7, 1, 3, points) == 0);
  assert(store.start(7, 1, 72));
  const int active = store.acquire_ready();
  assert(active == 0);
  assert(!store.abort_requested(active));
  assert(store.abort(1, 7).count == 0); // RT owns the active Start result.
  assert(store.abort_requested(active));
  points = {77, 88, 99};
  assert(store.prepare(8, 1, 3, points) == -1);
  assert(store.view(active).points[0] == 44); // Helper cannot recycle RT storage.
  store.abort(1, 999); // An unknown handle cannot erase an active cancellation.
  assert(store.abort_requested(active));
  store.retire(active);
  assert(store.prepare(7, 1, 3, points) == 0);
  assert(!store.abort_requested(0)); // A reused numeric handle has fresh state.
  assert(store.abort(1, 7).count == 0);
  assert(!store.start(7, 1, 73));

  // Exercise the publication race with RT holding any acquired storage until
  // helper cancellation/reuse has completed. Exactly one thread may retire a
  // Ready slot: successful helper CAS, or RT after observing its abort flag.
  for (unsigned round = 0; round < 2000; ++round) {
    points = {static_cast<int>(round), -1};
    assert(store.prepare(7, 1, 1, points) == 0);
    assert(store.start(7, 1, round + 100));
    std::atomic<bool> begin{false}, observed{false}, release{false};
    std::atomic<int> acquired{-2};
    std::thread cycle([&] {
      while (!begin.load(std::memory_order_acquire)) std::this_thread::yield();
      const int slot = store.acquire_ready();
      acquired.store(slot, std::memory_order_relaxed);
      observed.store(true, std::memory_order_release);
      while (!release.load(std::memory_order_acquire)) std::this_thread::yield();
      if (slot >= 0) {
        assert(store.abort_requested(slot));
        assert(store.view(slot).points[0] == static_cast<int>(round));
        store.retire(slot);
      }
    });
    begin.store(true, std::memory_order_release);
    const auto cancelled = store.abort(1, 7);
    while (!observed.load(std::memory_order_acquire)) std::this_thread::yield();
    const bool rt_acquired = acquired.load(std::memory_order_relaxed) >= 0;
    assert(cancelled.count == (rt_acquired ? 0u : 1u));
    points = {100000, 200000};
    const int replacement = store.prepare(8, 1, 1, points);
    assert((replacement < 0) == rt_acquired);
    release.store(true, std::memory_order_release);
    cycle.join();
    store.abort(1); // Retire the replacement, if helper won the Ready race.
  }
}
