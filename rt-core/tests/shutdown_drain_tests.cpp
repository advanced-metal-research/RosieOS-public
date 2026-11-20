#include "shutdown_drain.hpp"

#include <atomic>
#include <cassert>
#include <initializer_list>

int main() {
  using rosie::rtcore::decide_shutdown_drain;
  constexpr uint64_t ms = 1000000, started = 1000000000, minimum = started + 250 * ms;
  // Completed mailbox work cannot shorten the existing disable grace.
  assert(!decide_shutdown_drain(minimum - 1, minimum, true, true, false).release_bus);
  assert(decide_shutdown_drain(minimum, minimum, true, true, false).release_bus);
  // Delayed mailbox work must retain exchange after the old 250 ms cutoff.
  for (uint64_t elapsed : {250u, 400u, 4999u}) {
    const auto waiting = decide_shutdown_drain(started + elapsed * ms, minimum, false, false, false);
    assert(!waiting.release_bus && !waiting.report_failure);
  }
  const auto expired = decide_shutdown_drain(started + 5000 * ms, minimum, false, false, false);
  assert(!expired.release_bus && expired.report_failure);
  const auto still_waiting = decide_shutdown_drain(started + 9000 * ms, minimum, false, false, true);
  assert(!still_waiting.release_bus && !still_waiting.report_failure);
  // A genuine later acknowledgement permits teardown; expiry does not fake it.
  assert(decide_shutdown_drain(started + 9001 * ms, minimum, true, true, true).release_bus);
  // example-servo 2026-10-07: an exited worker flag is not proof that no mailbox
  // call is in flight. Without the mailbox lock the exchange never ends, at
  // any time, and the five-second budget only declares the failure.
  for (uint64_t elapsed : {250u, 4999u, 60000u}) {
    const auto unowned = decide_shutdown_drain(started + elapsed * ms, minimum, true, false, false);
    assert(!unowned.release_bus && unowned.report_failure == (elapsed >= 5000));
  }
  // The scope counts each blocking call and is exception/early-return safe.
  std::atomic<uint32_t> calls{0};
  {
    rosie::rtcore::MailboxCallScope first(calls);
    assert(calls.load() == 1);
    {
      rosie::rtcore::MailboxCallScope second(calls);
      assert(calls.load() == 2);
    }
    assert(calls.load() == 1);
  }
  assert(calls.load() == 0);
}
