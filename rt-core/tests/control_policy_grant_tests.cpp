// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "control_policy.hpp"
#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

namespace {
void conditional_revoke_preserves_successors() {
  rosie::rtcore::ControlPolicy policy;
  policy.application_required = true;
  policy.connected = true;
  policy.heartbeat_ns = 100;
  policy.lease_ns = 100;
  policy.application_generation = 1;
  policy.application_deadline_ns = 180;
  assert(policy.lease_valid(101));
  assert(!policy.revoke(0));
  assert(policy.generation == 1 && policy.application_generation == 1);
  assert(policy.revoke(1));
  assert(policy.generation == 2 && policy.application_generation == 0);
  assert(!policy.lease_valid(101));
  // Once a successor is admitted, replaying an older terminal event is inert.
  policy.application_generation = 2;
  assert(policy.lease_valid(101));
  for (unsigned retry = 0; retry < 1000; ++retry) assert(!policy.revoke(1));
  assert(policy.generation == 2 && policy.application_generation == 2 && policy.lease_valid(101));
  assert(!policy.revoke(3));
  assert(policy.generation == 2 && policy.application_generation == 2);
  assert(policy.revoke() == 3); // A separate explicit Stop still advances again.
  assert(policy.revoke() == 4);
  assert(!policy.revoke(2));
  policy.generation = std::numeric_limits<uint64_t>::max();
  assert(!policy.revoke(std::numeric_limits<uint64_t>::max()));
  assert(policy.generation == std::numeric_limits<uint64_t>::max());
}

void one_event_has_one_winner() {
  rosie::rtcore::ControlPolicy policy;
  constexpr unsigned contenders = 8;
  std::atomic<unsigned> ready{0}, winners{0};
  std::atomic<bool> start{false};
  std::vector<std::thread> threads;
  for (unsigned i = 0; i < contenders; ++i) threads.emplace_back([&] {
    ready.fetch_add(1, std::memory_order_release);
    while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
    if (policy.revoke(1)) winners.fetch_add(1, std::memory_order_relaxed);
  });
  while (ready.load(std::memory_order_acquire) != contenders) std::this_thread::yield();
  start.store(true, std::memory_order_release);
  for (auto& thread : threads) thread.join();
  assert(winners == 1 && policy.generation == 2);
}

void concurrent_successor_publication_survives_old_cleanup() {
  // A publisher can observe the new generation while the winning revoker is
  // still invalidating its old application mirror. The new mirror must survive.
  rosie::rtcore::ControlPolicy policy;
  constexpr uint64_t final_generation = 10001;
  policy.application_generation = 1;
  std::atomic<uint64_t> published{1};
  std::thread publisher([&] {
    for (uint64_t next = 2; next <= final_generation; ++next) {
      while (policy.generation.load(std::memory_order_acquire) != next) std::this_thread::yield();
      policy.application_generation.store(next, std::memory_order_release);
      published.store(next, std::memory_order_release);
    }
  });
  for (uint64_t before = 1; before < final_generation; ++before) {
    assert(policy.revoke(before));
    while (published.load(std::memory_order_acquire) != before + 1) std::this_thread::yield();
    assert(policy.application_generation == before + 1);
  }
  publisher.join();
  assert(policy.generation == final_generation && policy.application_generation == final_generation);
}

void modeled_terminal_owner_fences_work_after_an_intervening_stop() {
  // Model the composition used by the socket and RT terminal-close paths. A
  // close must fence the generation current after taking ownership, even if a
  // separate Stop and newer work occur while both close observers are delayed.
  rosie::rtcore::ControlPolicy policy;
  policy.connected = true;
  policy.application_generation = 1;
  std::atomic<unsigned> observers_ready{0}, close_winners{0};
  std::atomic<bool> take_ownership{false};
  auto terminal_observer = [&] {
    observers_ready.fetch_add(1, std::memory_order_release);
    while (!take_ownership.load(std::memory_order_acquire)) std::this_thread::yield();
    if (policy.connected.exchange(false, std::memory_order_acq_rel)) {
      close_winners.fetch_add(1, std::memory_order_relaxed);
      policy.revoke();
    }
  };
  std::thread socket_owner(terminal_observer), rt_observer(terminal_observer);
  while (observers_ready.load(std::memory_order_acquire) != 2) std::this_thread::yield();
  assert(policy.revoke() == 2); // Explicit Stop is a separate event.
  const uint64_t newer_work_generation = policy.generation.load();
  policy.application_generation = newer_work_generation;
  take_ownership.store(true, std::memory_order_release);
  socket_owner.join();
  rt_observer.join();
  assert(close_winners == 1 && !policy.connected);
  assert(policy.generation == 3 && policy.generation != newer_work_generation);
  assert(policy.application_generation == 0);
}
} // namespace

int main() {
  conditional_revoke_preserves_successors();
  one_event_has_one_winner();
  concurrent_successor_publication_survives_old_cleanup();
  modeled_terminal_owner_fences_work_after_an_intervening_stop();
  rosie::rtcore::ControlPolicy policy;
  policy.lease_ns = 100;
  policy.heartbeat_ns = 100;
  policy.connected = true;
  policy.application_required = true;
  assert(policy.connection_valid(101));
  assert(!policy.lease_valid(101)); // Transport liveness alone grants nothing.
  policy.application_deadline_ns = 150;
  policy.application_generation = policy.generation.load();
  assert(policy.lease_valid(149));
  assert(!policy.lease_valid(150)); // Exact application expiry is closed.
  assert(!policy.lease_valid(99));

  const auto old_generation = policy.generation.load();
  policy.revoke();
  assert(!policy.lease_valid(120));
  // An RT publication already in flight when Stop advances the fence cannot
  // re-grant motion by finishing its old deadline/generation stores afterward.
  policy.application_deadline_ns = 195;
  policy.application_generation = old_generation;
  assert(!policy.lease_valid(120));
  policy.application_generation = policy.generation.load();
  assert(policy.lease_valid(194));
  policy.application_deadline_ns = 250;
  assert(!policy.lease_valid(200)); // Application renewal cannot renew transport.
  policy.heartbeat_ns = 200;
  assert(policy.lease_valid(201));
  policy.connected = false;
  assert(!policy.lease_valid(201));
  policy.connected = true;
  policy.application_generation = 0;
  assert(!policy.lease_valid(201));
  policy.application_required = false; // Explicit simulation oracle only.
  assert(policy.lease_valid(201));
  std::cout << "control policy grant tests passed\n";
}
