#include "native_home_wire_audit.hpp"
#include <cassert>
#include <iostream>
#include <thread>

using namespace rosie::rtcore;

static NativeHomeWireSample sample(uint64_t cycle, uint16_t cw = 31, int8_t mode = 6) {
  return {cycle, cycle * 1000000, cw, 0x0237, mode, 1002, 1000};
}

int main() {
  NativeHomeWireAudit audit;
  NativeHomeWireSummary out;
  assert(!audit.read_stopped(0, out));
  auto token = audit.begin(1000000);
  assert(!audit.trigger_submitted(token));
  audit.observe(true, sample(1, 15), 1000000);
  audit.observe(false, sample(2), 1000000); // Failed31 cannot establish trigger.
  audit.observe(true, sample(3, 31, 8), 1000000); // Wrong-mode prefix is explicit.
  assert(!audit.trigger_submitted(token)); // Failed/wrong-mode31 is insufficient.
  audit.observe(true, sample(4), 1000000);
  assert(audit.trigger_submitted(token));
  audit.observe(true, sample(5, 15), 1000000);
  audit.observe(true, sample(6, 31, 8), 1000000);
  audit.observe(false, sample(7), 1000000);
  audit.observe(true, sample(10), 1000000); // Missing calls/time cannot look complete.
  audit.observe(true, sample(10), 1000000); // Duplicate cycle is also visible.
  audit.end(token, 10500000);
  assert(!audit.read_stopped(token, out));
  audit.observe(true, sample(11, 0, 8), 1000000); // Later cleanup excluded.
  assert(audit.read_stopped(token, out));
  assert(out.cycles == 9 && out.submitted == 7 && out.submission_failures == 2);
  assert(out.prefix_cycles == 2 && out.triggered_submissions == 5 && out.violations == 2);
  assert(out.first.cycle == 4 && out.first.target == 1002 && out.first.position == 1000);
  assert(out.last.cycle == 10 && out.last.controlword == 31 && out.last.mode == 6);
  assert(out.first_violation.cycle == 5 && out.first_violation.controlword == 15);
  assert(out.missing_cycles == 2 && out.nonmonotonic_cycles == 1 && out.time_gaps == 2);
  assert(out.maximum_gap_ns == 3000000 && out.ended_ns == 10500000 && out.stopped_cycle == 11);
  audit.observe(true, sample(12, 0, 8), 1000000);
  assert(audit.read_stopped(token, out) && out.cycles == 9); // Frozen after ack.

  token = audit.begin(20000000);
  assert(!audit.trigger_submitted(token)); // A prior transaction cannot qualify.

  audit.observe(true, sample(20), 1000000);
  audit.end(token, 22000000);
  audit.observe(true, sample(21), 1000000); // Submission before end, hook after end.
  assert(audit.read_stopped(token, out) && out.triggered_submissions == 2 && out.last.cycle == 21);
  assert(out.violations == 0 && out.submission_failures == 0 && out.missing_cycles == 0);

  token = audit.begin(30000000);
  audit.end(token, 30000001);
  audit.observe(true, sample(31), 1000000);
  assert(audit.read_stopped(token, out) && out.cycles == 0 && out.triggered_submissions == 0);
  // No successful observation is explicitly empty, never a clean triggered trace.
  token = audit.begin(40000000);
  audit.observe(false, sample(40), 1000000);
  audit.end(token, 41000000);
  audit.observe(false, sample(42), 1000000);
  assert(audit.read_stopped(token, out) && out.submission_failures == 1 && out.triggered_submissions == 0);

  // An early precondition refusal must freeze a useful prefix even though
  // no trigger request or successful CW31 submission ever occurred.
  token = audit.begin(100000000, true);
  audit.observe(true, sample(100, 6, 8), 1000000);
  audit.observe(true, sample(101, 6, 8), 1000000);
  audit.end(token, 101500000);
  audit.observe(true, sample(102, 0, 8), 1000000);
  assert(audit.read_stopped(token, out));
  assert(out.cycles == 0 && out.triggered_submissions == 0 && out.first.cycle == 0);
  assert(out.prefix.cycles == 2 && out.prefix.buckets[0].submitted == 2);
  assert(out.prefix.buckets[0].mode8 == 2 && out.prefix.buckets[0].first.cycle == 100);
  assert(out.prefix.buckets[0].last.cycle == 101 && out.prefix.buckets[3].submitted == 0);

  // Step5 cancellation: distinguish requested CW15 whose submission failed
  // from actual wire CW7, and retain both endpoints of each successful CW.
  token = audit.begin(200000000, true);
  audit.observe(true, sample(200, 6, 8), 1000000);
  audit.observe(true, sample(201, 6), 1000000);
  audit.observe(true, sample(202, 7), 1000000);
  audit.observe(false, sample(203, 15), 1000000);
  audit.observe(true, sample(204, 7), 1000000);
  audit.observe(true, sample(207, 15), 1000000);
  audit.observe(true, sample(208, 15, 8), 1000000); // Wrong mode remains visible.
  audit.observe(true, sample(208, 0), 1000000); // Duplicate timestamp/cycle.
  audit.end(token, 208500000);
  audit.observe(true, sample(209, 0, 8), 1000000);
  assert(audit.read_stopped(token, out));
  assert(out.triggered_submissions == 0 && out.cycles == 0);
  assert(out.prefix.cycles == 8 && out.prefix.submitted == 7 && out.prefix.submission_failures == 1);
  assert(out.prefix.missing_cycles == 2 && out.prefix.nonmonotonic_cycles == 1 && out.prefix.time_gaps == 2);
  assert(out.prefix.maximum_gap_ns == 3000000);
  assert(out.prefix.buckets[1].first.cycle == 202 && out.prefix.buckets[1].last.cycle == 204);
  assert(out.prefix.buckets[2].submitted == 2 && out.prefix.buckets[2].first.cycle == 207);
  assert(out.prefix.buckets[2].last.cycle == 208 && out.prefix.buckets[2].mode6 == 1 && out.prefix.buckets[2].mode8 == 1);
  assert(out.prefix.buckets[3].submitted == 1);

  // Successful prefix followed by the original CW31 observation window. A
  // pre-marker submission whose hook runs afterward still belongs to prefix;
  // original counters and violation semantics remain independent of prefix.
  token = audit.begin(300000000, true);
  audit.observe(true, sample(300, 6, 8), 1000000);
  audit.observe(true, sample(301, 7), 1000000);
  audit.trigger_requested(token, 303000000);
  audit.observe(true, sample(302, 15), 1000000);
  audit.observe(true, sample(303, 15), 1000000); // Existing requested31 prefix.
  audit.observe(true, sample(304), 1000000);
  audit.observe(true, sample(305), 1000000);
  audit.end(token, 305500000);
  audit.observe(true, sample(306, 0, 8), 1000000);
  assert(audit.read_stopped(token, out));
  assert(out.prefix.cycles == 3 && out.prefix.buckets[2].last.cycle == 302);
  assert(out.cycles == 3 && out.submitted == 3 && out.prefix_cycles == 1);
  assert(out.triggered_submissions == 2 && out.first.cycle == 304 && out.last.cycle == 305);
  assert(out.violations == 0 && out.submission_failures == 0 && out.missing_cycles == 0);

  // Exercise the worker/cyclic handoff concurrently. Reads before the ack must
  // not touch the producer-owned summary; old acknowledgements cannot satisfy a
  // later session. Suitable for ThreadSanitizer as well as normal C++ execution.
  for (unsigned repeat = 0; repeat < 100; ++repeat) {
    const auto current = audit.begin(50000000);
    assert(!audit.read_stopped(current, out));
    std::atomic<bool> first{false}, quit{false};
    std::thread producer([&] {
      uint64_t cycle = 50;
      while (!quit.load()) {
        audit.observe(true, sample(cycle++), 1000000);
        first.store(true);
        std::this_thread::yield();
      }
    });
    while (!first.load()) std::this_thread::yield();
    audit.end(current, 50000000);
    while (!audit.read_stopped(current, out)) std::this_thread::yield();
    assert(out.token == current && out.triggered_submissions >= 1 && out.violations == 0);
    quit.store(true);
    producer.join();
  }
  std::cout << "native Home submitted-wire audit tests passed\n";
}
