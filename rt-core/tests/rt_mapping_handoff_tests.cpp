// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "rt_mapping_handoff.hpp"
#include <atomic>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <thread>

struct Region { uint64_t generation; uint64_t inverse; };
using Handoff = rosie::rtcore::RtMappingHandoff<Region>;

int main() {
  {
    Handoff handoff;
    Region first{1, ~uint64_t{1}}, next{2, ~uint64_t{2}};
    assert(!handoff.publish(nullptr));
    assert(handoff.publish(&first));
    assert(!handoff.publish(&next));
    {
      Handoff::Cycle cycle(handoff);
      assert(cycle.get() == &first);
      handoff.detach();
      handoff.detach();
      assert(!handoff.take_retired());
      assert(!handoff.publish(&next));
      assert(cycle.get()->generation == 1);
      { Handoff::Cycle nested(handoff); assert(!nested.get()); }
      assert(!handoff.take_retired());
      assert(!handoff.stop_reader());
      assert(!handoff.take_retired());
    }
    assert(handoff.take_retired() == &first);
    assert(!handoff.take_retired());
    assert(handoff.publish(&next));
    handoff.detach();
    assert(!handoff.take_retired()); // A late ticket waits conservatively.
    { Handoff::Cycle cycle(handoff); assert(!cycle.get()); }
    assert(handoff.take_retired() == &next);
    handoff.stop_reader();
    assert(!handoff.publish(&first));
    { Handoff::Cycle cycle(handoff); assert(!cycle.get()); }
  }
  {
    Handoff handoff;
    Region region{9, ~uint64_t{9}};
    assert(handoff.publish(&region));
    handoff.detach();
    handoff.stop_reader(); // No further cycle can acknowledge shutdown.
    assert(handoff.take_retired() == &region);
  }
  {
    Handoff handoff;
    std::atomic<bool> done{false};
    std::atomic<uint64_t> observed{0};
    std::thread reader([&] {
      while (!done.load(std::memory_order_acquire)) {
        Handoff::Cycle cycle(handoff);
        if (auto* region = cycle.get()) {
          assert(region->generation == ~region->inverse);
          observed.fetch_add(1, std::memory_order_relaxed);
        }
      }
      handoff.stop_reader();
    });
    for (uint64_t generation = 1; generation <= 2000; ++generation) {
      auto* region = new Region{generation, ~generation};
      assert(handoff.publish(region));
      const uint64_t before = observed.load(std::memory_order_acquire);
      while (observed.load(std::memory_order_acquire) == before) std::this_thread::yield();
      handoff.detach();
      Region* retired = nullptr;
      while (!(retired = handoff.take_retired())) std::this_thread::yield();
      assert(retired == region);
      delete retired; // ASan checks retirement in the exercised schedules.
    }
    done.store(true, std::memory_order_release);
    reader.join();
  }
  std::cout << "RT mapping retirement: passed\n";
}
