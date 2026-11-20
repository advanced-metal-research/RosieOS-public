#define ROSIE_RT_SIMULATION 1
#include "cycle_bus.hpp"

#include <cstdlib>
#include <cstdio>
#include <new>

namespace {
size_t allocations = 0;
void expect(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "%s\n", message);
    std::exit(1);
  }
}
}

// The daemon may allocate during setup. The cycle boundary may not allocate,
// including successful, rejected and faulted submissions.
void* operator new(size_t bytes) {
  ++allocations;
  if (void* p = std::malloc(bytes ? bytes : 1)) return p;
  throw std::bad_alloc{};
}
void* operator new[](size_t bytes) { return ::operator new(bytes); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }
void operator delete[](void* p, size_t) noexcept { std::free(p); }

namespace {
using namespace rosie::rtcore;

struct Fixture {
  ec_master_t* master = ecrt_request_master(0);
  std::array<ec_domain_t*, 2> domains{};
  std::array<ec_slave_config_t*, 2> slaves{};
  std::array<DomainBinding, 2> bindings{};
  std::array<unsigned int, 2> target_offsets{};
  CycleBus bus;

  explicit Fixture(SimFaultControls faults = {}, bool split = true) {
    domains[0] = ecrt_master_create_domain(master);
    domains[1] = split ? ecrt_master_create_domain(master) : domains[0];
    for (unsigned axis = 0; axis < 2; ++axis) {
      slaves[axis] = ecrt_master_slave_config(master, 0, axis, 1, axis + 1);
      ec_pdo_entry_info_t outputs[]{{0x6040, 0, 16}, {0x607a, 0, 32}, {0x6060, 0, 8}};
      ec_pdo_entry_info_t inputs[]{{0x6041, 0, 16}, {0x6064, 0, 32}, {0x6061, 0, 8}};
      ec_pdo_info_t pdos[]{{0x1600, 3, outputs}, {0x1a00, 3, inputs}};
      ec_sync_info_t syncs[]{{2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
                             {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE},
                             {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}};
      expect(ecrt_slave_config_pdos(slaves[axis], EC_END, syncs) == 0, "configure PDOs");
      for (auto& entry : slaves[axis]->entries) {
        expect(sim_register(entry, domains[axis]) >= 0, "register PDO");
        if (entry.info.index == 0x607a) target_offsets[axis] = entry.offset;
      }
    }
    expect(sim_configure_faults(master, faults), "configure explicit test plant controls");
    expect(ecrt_master_activate(master) == 0, "activate simulated master");
    bindings[0] = {domains[0], split ? 1u : 3u, ecrt_domain_data(domains[0])};
    bindings[1] = {domains[1], 2, ecrt_domain_data(domains[1])};
    expect(bus.configure(master, 3, bindings.data(), split ? 2 : 1), "configure bounded cycle bus");
  }
  ~Fixture() { ecrt_release_master(master); }

  void targets(const std::array<int32_t, 2>& counts) {
    for (unsigned axis = 0; axis < 2; ++axis) bus.axis(axis).write_s32(target_offsets[axis], counts[axis]);
  }
  int32_t submitted(unsigned axis) {
    return EC_READ_S32(domains[axis]->submitted.data() + target_offsets[axis]);
  }
};

void reference_sync_uses_current_clock() {
  Fixture f;
  constexpr uint64_t entry = 1'000'000'000, after_work = entry + 110'371;
  (void)f.bus.begin_cycle(entry);
  f.bus.sync_clocks(after_work);
  expect(f.master->application_time_ns == entry, "cycle phase anchor stays at entry");
  expect(f.master->reference_sync_time_ns == after_work,
         "reference clock must not reuse stale entry or future sleep time");
  expect(f.master->reference_sync_calls == 1 && f.master->dc_calls == 1,
         "one DC cycle writes the reference clock and distributes it once");
  expect(f.bus.queue(3).succeeded() && f.bus.submit(3).covers(3), "first DC cycle submits");
  (void)f.bus.begin_cycle(entry + 1'000'000);
  f.bus.sync_clocks(after_work + 1'000'000);
  expect(f.master->reference_sync_calls == 2 && f.master->reference_sync_time_ns == after_work + 1'000'000,
         "the next cycle steers the reference clock again; no cycle skips drift compensation");
}

void per_axis_submission_history() {
  Fixture f;
  const std::array<int32_t, 2> targets{123, -456};
  std::array<int32_t, 2> previous{8, 9};
  std::array<bool, 2> have_previous{};
  expect(!f.bus.begin_cycle(1), "initial cycle has no prior submission receipt");
  expect(f.bus.domain_state().working_counter == 0, "initial split domains have no received counters");
  f.targets(targets);
  expect(f.bus.queue(1).succeeded(), "queue first axis");
  expect(f.bus.queue(1).succeeded() && f.master->queue_calls == 1, "repeated queue does not issue duplicate ioctl");
  const auto first = f.bus.submit(1);
  expect(first.attempted() && first.succeeded() && first.submitted_axis_mask == 1, "receipt identifies only submitted domain");
  expect(first.covers(1) && !first.covers(3) && !first.covers(0), "receipt cannot acknowledge another axis or empty Stop group");
  record_submitted_targets(first, targets, previous, have_previous);
  expect(previous[0] == 123 && previous[1] == 9 && have_previous[0] && !have_previous[1], "only sent target history advances");
  expect(f.submitted(0) == 123 && f.submitted(1) == 0, "actual submitted process images match receipt");
  expect(f.bus.submit(1).skipped == SubmissionSkip::AlreadyFinalized && f.master->send_calls == 1,
         "cycle cannot send twice");
  expect(!f.bus.begin_cycle(2) && f.bus.feedback_axis_mask() == 1,
         "only the previously submitted axis has fresh input");
  expect(f.bus.queue(2).succeeded(), "queue second axis");
  const auto second = f.bus.submit(2);
  record_submitted_targets(second, targets, previous, have_previous);
  expect(second.submitted_axis_mask == 2 && previous[0] == 123 && previous[1] == -456 && have_previous[1],
         "round robin preserves unsent history and updates sent axis");
}

void indivisible_domain_and_suppression() {
  Fixture f({}, false);
  expect(f.bus.axis(0).data() == f.bus.axis(1).data(), "shared domain exposes one image to its axes");
  expect(!f.bus.begin_cycle(1), "shared initial cycle has no receipt");
  expect(f.bus.queue(1).succeeded(), "request a subset of shared domain");
  expect(f.bus.queued_axis_mask() == 3, "queue expands to every axis sent by binding");
  expect(f.bus.queue(2).succeeded() && f.master->queue_calls == 1, "shared domain is queued once");
  expect(f.bus.submit(1).submitted_axis_mask == 3, "receipt names complete shared image");
  expect(f.bus.begin_cycle(2), "suppressed queue input phase");
  const auto clock_only = f.bus.submit(0);
  expect(clock_only.succeeded() && clock_only.submitted_axis_mask == 0 && !clock_only.covers(3),
         "startup clock-only send cannot acknowledge inhibited outputs");
  expect(!f.bus.axis(2).valid() && !f.bus.axis(32).valid(), "unbound axis has no process image");
}

void partial_queue_and_pending_domains() {
  SimFaultControls faults;
  faults.queue_fail_from_call = 2;
  faults.queue_fail_for_calls = 1;
  Fixture f(faults);
  expect(!f.bus.begin_cycle(1), "partial queue initial input phase");
  expect(f.bus.queue(3).failed() && f.bus.queued_axis_mask() == 1, "later domain failure retains precise successful queue mask");
  expect(f.bus.queue(3).failed() && f.master->queue_calls == 2, "failed queue is not retried implicitly");
  const auto failed = f.bus.submit(3);
  expect(!failed.attempted() && failed.skipped == SubmissionSkip::QueueFailure && f.master->send_calls == 0,
         "partial group is never sent after queue failure");
  expect(f.bus.pending_axis_mask() == 3, "failure retains potentially pending domains");
  expect(!f.bus.begin_cycle(2), "failed queue creates no next input receipt");
  expect(f.bus.queue(1).succeeded(), "fresh queue of only one domain");
  expect(f.bus.submit(1).skipped == SubmissionSkip::PendingDomains && f.master->send_calls == 0,
         "subset send cannot flush stale pending targets from another domain");
  expect(!f.bus.begin_cycle(3), "skipped recovery send creates no input receipt");
  expect(f.bus.queue(3).succeeded(), "fresh inhibited group replaces every pending domain");
  expect(f.bus.submit(3).covers(3) && f.bus.pending_axis_mask() == 0, "full fresh queue restores honest group receipt");

  expect(f.bus.begin_cycle(4), "incomplete required group cycle");
  expect(f.bus.queue(1).succeeded(), "queue insufficient group");
  const auto sends = f.master->send_calls;
  expect(f.bus.submit(3).skipped == SubmissionSkip::IncompleteGroup && f.master->send_calls == sends,
         "required group cannot be acknowledged or sent partially");
}

void failed_send_preserves_history() {
  SimFaultControls faults;
  faults.send_fail_from_call = 1;
  faults.send_fail_for_calls = 1;
  Fixture f(faults);
  const std::array<int32_t, 2> targets{111, 222};
  std::array<int32_t, 2> previous{3, 4};
  std::array<bool, 2> have_previous{};
  expect(!f.bus.begin_cycle(1), "send failure initial input phase");
  f.targets(targets);
  expect(f.bus.queue(1).succeeded(), "queue before failed send");
  const auto failed = f.bus.submit(1);
  expect(failed.attempted() && !failed.succeeded() && failed.result.return_code == -5 && failed.submitted_axis_mask == 0,
         "native failure is distinct from an unattempted send");
  record_submitted_targets(failed, targets, previous, have_previous);
  expect(previous[0] == 3 && previous[1] == 4 && !have_previous[0] && !have_previous[1],
         "failed send never advances target history");
  expect(!f.bus.begin_cycle(2), "failed send supplies no input receipt");
  expect(f.bus.queue(2).succeeded(), "queue other domain after failed send");
  expect(f.bus.submit(2).skipped == SubmissionSkip::PendingDomains && f.master->send_calls == 1,
         "pending first domain cannot be flushed by round-robin send of another");
  expect(!f.bus.begin_cycle(3), "unattempted send supplies no input receipt");
  expect(f.bus.queue(3).succeeded(), "refresh all pending images");
  const auto restored = f.bus.submit(3);
  record_submitted_targets(restored, targets, previous, have_previous);
  expect(restored.covers(3) && previous == targets && have_previous[0] && have_previous[1], "successful recovery records actual wire targets");
}

void failed_input_can_submit_inhibition() {
  for (const bool receive_failure : {false, true}) {
    SimFaultControls faults;
    if (receive_failure) {
      faults.receive_fail_from_call = 2; // Activation consumes first receive.
      faults.receive_fail_for_calls = 1;
    } else {
      faults.process_fail_from_call = 1;
      faults.process_fail_for_calls = 1;
    }
    Fixture f(faults);
    expect(!f.bus.begin_cycle(1) && !f.bus.feedback_fresh(), "failed receive/process invalidates input freshness");
    expect(f.bus.results().first_failure.operation == (receive_failure ? BusOperation::Receive : BusOperation::Process),
           "input failure retains precise operation");
    expect(f.bus.queue(3).succeeded(), "inhibited image may be queued after input failure");
    expect(f.bus.submit(3).covers(3), "successful inhibited send remains honest submission evidence");
    expect(f.bus.results().failed() && !f.bus.feedback_fresh(), "send cannot overwrite input failure or freshness");
    expect(f.bus.begin_cycle(2) && !f.bus.results().failed(), "next independently successful input cycle is fresh");
  }
  Fixture incomplete;
  incomplete.master->active = false;
  expect(!incomplete.bus.begin_cycle(1) && !incomplete.bus.results().failed(),
         "incomplete WKC is not fresh even with successful native operations");
}

void late_fault_and_post_send_query() {
  SimFaultControls faults;
  faults.dc_fail_from_call = 1;
  faults.dc_fail_for_calls = 1;
  Fixture f(faults);
  expect(!f.bus.begin_cycle(1), "initial input before full exchange");
  expect(f.bus.queue(3).succeeded() && f.bus.submit(3).covers(3), "initial full exchange");
  expect(f.bus.begin_cycle(2), "fresh input before DC fault");
  f.bus.sync_clocks(2);
  expect(f.bus.feedback_fresh() && f.bus.results().first_failure.operation == BusOperation::SlaveClockSync,
         "DC error affects output permit without rewriting input-phase observation");
  expect(f.bus.queue(3).succeeded() && f.bus.submit(3).covers(3), "DC failure still permits explicit inhibited image submission");
  f.master->active = false;
  expect(f.bus.query_state().succeeded() && f.bus.domain_state().wc_state == EC_WC_COMPLETE,
         "post-send query reports cached processed state, not a new receipt");
  expect(f.bus.feedback_fresh() && f.bus.results().failed(), "later query cannot erase latched cycle failure");
}

void incomplete_wkc_uses_receive_cycles() {
  SimFaultControls faults;
  faults.wkc_incomplete_from_call = 3; // Activation and initial no-receipt cycle first.
  faults.wkc_incomplete_for_calls = 2;
  Fixture f(faults);
  expect(!f.bus.begin_cycle(1), "WKC control starts without a receipt");
  expect(f.bus.queue(3).succeeded() && f.bus.submit(3).covers(3), "prime WKC exchange");
  expect(!f.bus.begin_cycle(2) && !f.bus.results().failed(),
         "incomplete WKC rejects freshness without inventing an ioctl error");
  for (unsigned query = 0; query < 4; ++query) {
    expect(f.bus.query_state().succeeded() &&
               f.bus.domain_state().wc_state == EC_WC_INCOMPLETE,
           "repeated split-domain/diagnostic queries do not consume WKC injection");
  }
  expect(f.bus.queue(3).succeeded() && f.bus.submit(3).covers(3),
         "incomplete input WKC does not fabricate failed inhibited submission");
  expect(!f.bus.begin_cycle(3), "second receive cycle remains incomplete");
  expect(f.bus.queue(3).succeeded() && f.bus.submit(3).covers(3), "new receipt after bounded fault");
  expect(f.bus.begin_cycle(4) && f.bus.domain_state().wc_state == EC_WC_COMPLETE,
         "freshness returns after exactly the bounded receive interval");
}

void geometry_and_no_allocations() {
  Fixture f;
  CycleBus invalid;
  auto duplicate = f.bindings;
  duplicate[1].axis_mask = 1;
  expect(!invalid.configure(f.master, 3, duplicate.data(), duplicate.size()), "overlapping axis bindings are rejected");
  duplicate[1] = {f.domains[0], 2, f.bindings[0].process_image};
  expect(!invalid.configure(f.master, 3, duplicate.data(), duplicate.size()), "shared domain must have one complete binding");
  expect(!invalid.configure(f.master, 3, f.bindings.data(), 1), "missing axis binding is rejected");
  expect(!invalid.begin_cycle(1) && !invalid.submit(3).attempted(), "unconfigured interface performs no native work");
  expect(!f.bus.configure(f.master, 3, f.bindings.data(), 2), "configuration is immutable after successful binding");

  expect(!f.bus.begin_cycle(1), "initial allocation test has no receipt");
  expect(f.bus.queue(3).succeeded() && f.bus.submit(3).covers(3), "prime allocation test exchange");
  const auto before = allocations;
  for (uint64_t cycle = 2; cycle <= 501; ++cycle) {
    expect(f.bus.begin_cycle(cycle), "allocation test input phase");
    f.bus.sync_clocks(cycle);
    expect(f.bus.queue(3).succeeded(), "allocation test queue");
    expect(f.bus.submit(3).covers(3), "allocation test submit");
    expect(f.bus.query_state().succeeded(), "allocation test observation");
  }
  expect(allocations == before, "cycle interface performed a heap allocation");
}

void pdo_view_byte_order() {
  std::array<uint8_t, 24> bytes{};
  PdoView view{bytes.data()};
  view.write_u16(0, 0x1234);
  view.write_s32(4, -2);
  view.write_u32(8, 0x89abcdefu);
  view.write_s16(12, -3);
  view.write_u8(14, 250);
  view.write_s8(15, -4);
  expect(bytes[0] == 0x34 && bytes[1] == 0x12 && bytes[4] == 0xfe && bytes[7] == 0xff,
         "PDO view preserves pinned byte-order semantics");
  expect(view.read_u16(0) == 0x1234 && view.read_s32(4) == -2 && view.read_u32(8) == 0x89abcdefu &&
             view.read_s16(12) == -3 && view.read_u8(14) == 250 && view.read_u8(15) == 252,
         "typed PDO accessors preserve field width and signedness");
}
}

int main() {
  reference_sync_uses_current_clock();
  per_axis_submission_history();
  indivisible_domain_and_suppression();
  partial_queue_and_pending_domains();
  failed_send_preserves_history();
  failed_input_can_submit_inhibition();
  late_fault_and_post_send_query();
  incomplete_wkc_uses_receive_cycles();
  geometry_and_no_allocations();
  pdo_view_byte_order();
  std::puts("cycle bus tests passed");
}
