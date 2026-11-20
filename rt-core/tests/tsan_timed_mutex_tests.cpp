// Exercise steady-clock timed mutex ownership and protected data so a missing
// runtime interceptor fails before the larger sanitizer policy suite starts.
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

void timed_mutex() {
  std::timed_mutex mutex;
  // unverified watchdog ceiling: 5 seconds; success requires acquiring the mutex.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  assert(mutex.try_lock_until(deadline));
  mutex.unlock();
  unsigned protected_count = 0;
  auto worker = [&] {
    // unverified contention workload: 1000 increments from each of two threads.
    for (unsigned i = 0; i < 1000; ++i) {
      std::unique_lock<std::timed_mutex> lock(mutex, std::defer_lock);
      assert(lock.try_lock_until(deadline));
      ++protected_count;
    }
  };
  std::thread first(worker), second(worker);
  first.join(); second.join();
  assert(protected_count == 2000);
  std::puts("PASS: TSan timed-mutex ownership and 2000 protected increments");
}

// Force a worker to acquire the mutex while the caller waits, proving that
// timed condition waits expose both the release and reacquisition to TSan.
void condition_variable() {
  std::mutex mutex;
  std::condition_variable changed;
  unsigned handoffs = 0;
  std::unique_lock<std::mutex> lock(mutex);
  std::thread worker([&] {
    { std::lock_guard<std::mutex> guard(mutex); ++handoffs; }
    changed.notify_one();
  });
  // unverified watchdog ceiling: 5 seconds; the protected handoff supplies success.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  assert(changed.wait_until(lock, deadline, [&] { return handoffs == 1; }));
  lock.unlock();
  worker.join();
  assert(handoffs == 1);
  std::puts("PASS: TSan timed condition-variable handoff=1");
}

int main(int argc, char** argv) {
  if (argc == 2 && std::strcmp(argv[1], "--condition-variable") == 0) {
    condition_variable();
    return 0;
  }
  assert(argc == 1);
  timed_mutex();
  condition_variable();
}
