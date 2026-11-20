// Isolated R29 candidate tests. No daemon, socket, kernel master or hardware.
#define ROSIE_RT_SIMULATION 1
#include "simulated_bus.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>

namespace faults {
bool application_time = false;
ec_domain_t* state_domain = nullptr;
ec_domain_t* delayed_domain = nullptr;
unsigned delayed_receives = 0;
}
int test_application_time(ec_master_t* master, uint64_t now) {
  return faults::application_time ? -7 : ecrt_master_application_time(master, now);
}
int test_domain_state(ec_domain_t* domain, ec_domain_state_t* observed) {
  return faults::state_domain == domain ? -11 : ecrt_domain_state(domain, observed);
}
int test_receive(ec_master_t* master) {
  auto* delayed = faults::delayed_domain;
  const bool skip_reply = delayed && faults::delayed_receives != 0 &&
      delayed->datagram_state == SimDatagramState::Sent;
  // A successful host receive ioctl does not require every outstanding domain
  // to have returned. Hide this one from the simulator's reply presenter only;
  // physical stepping still occurs for all axes, and this domain remains Sent.
  if (skip_reply) delayed->datagram_state = SimDatagramState::Initial;
  const int result = ecrt_master_receive(master);
  if (skip_reply) {
    delayed->datagram_state = SimDatagramState::Sent;
    --faults::delayed_receives;
  }
  return result;
}
// Test-only native failure seam; production source is not instrumented.
#define ecrt_master_application_time test_application_time
#define ecrt_domain_state test_domain_state
#define ecrt_master_receive test_receive
#include "cycle_bus.hpp"
#undef ecrt_master_application_time
#undef ecrt_domain_state
#undef ecrt_master_receive

namespace {
using namespace rosie::rtcore;
void expect(bool condition, const char* message) {
  if (!condition) { std::fprintf(stderr, "%s\n", message); std::exit(1); }
}
struct Fixture {
  ec_master_t* master = ecrt_request_master(0);
  std::array<ec_domain_t*, 2> domains{};
  std::array<ec_slave_config_t*, 2> slaves{};
  std::array<unsigned int, 2> position_offsets{}, target_offsets{}, cw_offsets{}, mode_offsets{};
  CycleBus bus;
  explicit Fixture(SimFaultControls controls = {}, bool shared = false, bool bind = true) {
    domains[0] = ecrt_master_create_domain(master);
    domains[1] = shared ? domains[0] : ecrt_master_create_domain(master);
    for (unsigned axis = 0; axis < 2; ++axis) {
      slaves[axis] = ecrt_master_slave_config(master, 0, axis, 1, axis + 1);
      ec_pdo_entry_info_t outputs[]{{0x6040, 0, 16}, {0x607a, 0, 32}, {0x6060, 0, 8}};
      ec_pdo_entry_info_t inputs[]{{0x6041, 0, 16}, {0x6064, 0, 32}, {0x6061, 0, 8}};
      ec_pdo_info_t pdos[]{{0x1600, 3, outputs}, {0x1a00, 3, inputs}};
      ec_sync_info_t syncs[]{{2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
                             {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE},
                             {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}};
      expect(ecrt_slave_config_pdos(slaves[axis], EC_END, syncs) == 0, "configure PDO entries");
      for (auto& entry : slaves[axis]->entries) {
        expect(sim_register(entry, domains[axis]) >= 0, "register PDO entry");
        switch (entry.info.index) {
          case 0x6064: position_offsets[axis] = entry.offset; break;
          case 0x607a: target_offsets[axis] = entry.offset; break;
          case 0x6040: cw_offsets[axis] = entry.offset; break;
          case 0x6060: mode_offsets[axis] = entry.offset; break;
          default: break;
        }
      }
    }
    expect(sim_configure_faults(master, controls), "install deterministic simulation controls");
    expect(ecrt_master_activate(master) == 0, "activate isolated simulator");
    if (bind) configure(shared);
  }
  void configure(bool shared = false) {
    const std::array<DomainBinding, 2> bindings{{
      {domains[0], shared ? 3u : 1u, ecrt_domain_data(domains[0])},
      {domains[1], 2, ecrt_domain_data(domains[1])}}};
    expect(bus.configure(master, 3, bindings.data(), shared ? 1 : 2), "bind fixed geometry");
  }
  ~Fixture() {
    faults::application_time = false;
    faults::state_domain = nullptr;
    faults::delayed_domain = nullptr;
    faults::delayed_receives = 0;
    ecrt_release_master(master);
  }
  int32_t input(unsigned axis) const {
    return EC_READ_S32(domains[axis]->data.data() + position_offsets[axis]);
  }
  void plant(int32_t axis0, int32_t axis1) {
    slaves[0]->position_counts = axis0;
    slaves[1]->position_counts = axis1;
  }
  void send(uint32_t mask) {
    expect(bus.queue(mask).succeeded(), "queue selected domains");
    const auto submitted = bus.submit(mask);
    expect(submitted.succeeded() && (mask == 0 || submitted.covers(mask)), "submit selected domains");
  }
  void prime() {
    expect(!bus.begin_cycle(1) && bus.feedback_axis_mask() == 0, "no receipt before first submission");
    send(3);
    expect(bus.begin_cycle(2) && bus.feedback_axis_mask() == 3, "first full exchange is fresh");
  }
};

void initial_full_clock_partial_and_double_begin() {
  Fixture f;
  expect(!f.bus.feedback_fresh() && f.bus.feedback_axis_mask() == 0, "configuration is not input evidence");
  expect(f.bus.query_state().succeeded() && f.bus.domain_state().wc_state == EC_WC_ZERO,
         "initial diagnostics see no completed exchange");
  f.plant(10, 20);
  expect(!f.bus.begin_cycle(1) && f.input(0) == 0 && f.input(1) == 0,
         "plant motion without a submitted domain does not reach the PDO image");
  f.send(3);
  expect(f.bus.begin_cycle(2) && f.bus.feedback_axis_mask() == 3 &&
         f.input(0) == 10 && f.input(1) == 20, "full exchange refreshes both axes");
  expect(f.bus.domain_state().working_counter == 4, "complete split-domain counter");
  expect(f.domains[0]->input_receipts == 1 && f.domains[1]->input_receipts == 1,
         "one actual input receipt per exchanged domain");

  expect(f.bus.submit(0).succeeded(), "clock-only transmission is permitted without pending outputs");
  f.plant(11, 21);
  expect(!f.bus.begin_cycle(3) && f.bus.feedback_axis_mask() == 0,
         "clock-only transmission cannot refresh axes");
  expect(f.bus.domain_state().wc_state == EC_WC_COMPLETE && f.input(0) == 10 && f.input(1) == 20,
         "cached complete WKC coexists with old PDO bytes");
  for (unsigned i = 0; i < 5; ++i) {
    expect(f.bus.query_state().succeeded() && f.bus.feedback_axis_mask() == 0,
           "diagnostic state query cannot create a receipt");
  }
  expect(f.domains[0]->input_receipts == 1 && f.domains[1]->input_receipts == 1,
         "queries do not fabricate received datagrams");

  f.send(1);
  f.plant(12, 22);
  expect(!f.bus.begin_cycle(4) && f.bus.feedback_axis_mask() == 1,
         "one-domain send refreshes only its bound axis");
  expect(f.input(0) == 12 && f.input(1) == 20 && f.bus.domain_state().wc_state == EC_WC_COMPLETE,
         "unexchanged domain retains cached complete counter and old PDO");
  f.plant(13, 23);
  expect(!f.bus.begin_cycle(5) && f.bus.feedback_axis_mask() == 0,
         "second begin without a new send consumes no reused receipt");
  expect(f.input(0) == 12 && f.input(1) == 20, "unexchanged PDOs remain unchanged");
  f.send(3);
  expect(f.bus.begin_cycle(6) && f.input(0) == 13 && f.input(1) == 23,
         "fresh full exchange restores truthful observations");
}

void old_setup_receipt_is_not_a_cycle_receipt() {
  // Direct setup traffic occurs before CycleBus obtains exclusive ownership.
  Fixture f({}, false, false);
  f.plant(44, 55);
  for (auto* d : f.domains) expect(ecrt_domain_queue(d) == 0, "setup queues domains");
  expect(ecrt_master_send(f.master) == 0 && ecrt_master_receive(f.master) == 0, "setup exchanges domains");
  for (auto* d : f.domains) expect(ecrt_domain_process(d) == 0, "setup processes domains");
  f.configure();
  expect(!f.bus.begin_cycle(1) && f.bus.feedback_axis_mask() == 0 &&
         f.bus.domain_state().wc_state == EC_WC_COMPLETE,
         "pre-configuration cached completion is not new cycle evidence");
  f.send(3);
  expect(f.bus.begin_cycle(2), "first owned submission produces trusted receipt");
}

void shared_domain_mask_is_indivisible() {
  Fixture f({}, true);
  expect(!f.bus.begin_cycle(1), "shared setup is not feedback");
  f.send(1);
  f.plant(60, 70);
  expect(f.bus.begin_cycle(2) && f.bus.feedback_axis_mask() == 3 &&
         f.input(0) == 60 && f.input(1) == 70,
         "requesting one axis of a shared domain refreshes its whole image");
  expect(f.domains[0]->input_receipts == 1, "shared binding has one datagram receipt");
}

void failed_queue_and_send_create_no_receipt() {
  for (bool queue_failure : {false, true}) {
    SimFaultControls controls;
    if (queue_failure) {
      controls.queue_fail_from_call = 4; // Two prime queues, then domain1 fails.
      controls.queue_fail_for_calls = 1;
    } else {
      controls.send_fail_from_call = 2; // Prime succeeds, next send fails.
      controls.send_fail_for_calls = 1;
    }
    Fixture f(controls);
    f.prime();
    f.plant(80, 90);
    const auto queued = f.bus.queue(3);
    expect(queued.failed() == queue_failure, "injected queue result");
    const auto submitted = f.bus.submit(3);
    expect(!submitted.succeeded() && submitted.submitted_axis_mask == 0,
           "failed submission cannot create output or input evidence");
    expect(submitted.attempted() != queue_failure, "queue failure prevents native send");
    expect(f.bus.feedback_axis_mask() == 3, "late failure does not rewrite prior valid input");
    expect(!f.bus.begin_cycle(3) && f.bus.feedback_axis_mask() == 0 &&
           f.input(0) == 0 && f.input(1) == 0,
           "failed submission provides no next-cycle inputs");
    expect(f.domains[0]->input_receipts == 1 && f.domains[1]->input_receipts == 1,
           "failed submission produces no simulated input replies");
    f.send(3); // Fresh full group replaces every potentially pending image.
    expect(f.bus.begin_cycle(4) && f.input(0) == 80 && f.input(1) == 90,
           "freshly queued recovery produces new input evidence");
  }
}

void failed_receive_and_process_cannot_replay_receipt() {
  for (bool receive_failure : {false, true}) {
    SimFaultControls controls;
    if (receive_failure) {
      controls.receive_fail_from_call = 3; // Activation, first begin, then the receipt.
      controls.receive_fail_for_calls = 1;
    } else {
      controls.process_fail_from_call = 4; // Domain1 processing of first actual receipt.
      controls.process_fail_for_calls = 1;
    }
    Fixture f(controls);
    expect(!f.bus.begin_cycle(1), "initial no-receipt cycle");
    f.send(3);
    f.plant(100, 200);
    expect(!f.bus.begin_cycle(2) && f.bus.feedback_axis_mask() == (receive_failure ? 0u : 1u),
           "global receive failure or per-domain process failure removes matching freshness");
    expect(f.bus.results().first_failure.operation ==
        (receive_failure ? BusOperation::Receive : BusOperation::Process), "precise input fault retained");
    expect(!f.bus.begin_cycle(3) && f.bus.feedback_axis_mask() == 0,
           "retry cannot reuse a receipt consumed by the failed input phase");
    expect(!f.bus.results().failed() && f.bus.domain_state().wc_state == EC_WC_COMPLETE,
           "later cached completion still does not qualify as new feedback");
    f.send(3);
    expect(f.bus.begin_cycle(4), "a new full exchange can restore freshness");
  }
}

void failed_application_time_and_query_cannot_replay_receipt() {
  for (bool global_failure : {false, true}) {
    Fixture f;
    expect(!f.bus.begin_cycle(1), "initial no-receipt cycle");
    f.send(3);
    if (global_failure) faults::application_time = true;
    else faults::state_domain = f.domains[1];
    expect(!f.bus.begin_cycle(2) && f.bus.feedback_axis_mask() == (global_failure ? 0u : 1u),
           "failed time/query rejects its consumed receipt evidence");
    faults::application_time = false;
    faults::state_domain = nullptr;
    const auto original_mask = f.bus.feedback_axis_mask();
    expect(f.bus.query_state().succeeded() && f.bus.feedback_axis_mask() == original_mask,
           "successful diagnostic retry does not restore failed input evidence");
    expect(!f.bus.begin_cycle(3) && f.bus.feedback_axis_mask() == 0,
           "later begin cannot turn consumed failed evidence fresh");
    f.send(3);
    expect(f.bus.begin_cycle(4), "new exchange qualifies independently");
  }
}

void incomplete_receipt_remains_cached_until_new_exchange() {
  SimFaultControls controls;
  controls.wkc_incomplete_from_call = 3;
  controls.wkc_incomplete_for_calls = 1;
  Fixture f(controls);
  expect(!f.bus.begin_cycle(1), "initial no-receipt cycle");
  f.send(3);
  expect(!f.bus.begin_cycle(2) && f.bus.feedback_axis_mask() == 0 &&
         f.bus.domain_state().wc_state == EC_WC_INCOMPLETE, "injected incomplete received WKC");
  for (unsigned i = 0; i < 4; ++i)
    expect(f.bus.query_state().succeeded() && f.bus.domain_state().wc_state == EC_WC_INCOMPLETE,
           "queries do not consume an injection or restore WKC");
  expect(!f.bus.begin_cycle(3) && f.bus.domain_state().wc_state == EC_WC_INCOMPLETE,
         "passing the injection window without a send cannot repair a cached receipt");
  f.send(3);
  expect(f.bus.begin_cycle(4), "new exchange beyond injection restores complete feedback");
}

void successful_receive_with_absent_then_late_domain() {
  Fixture f;
  f.prime();
  f.send(3);
  f.plant(210, 310);
  faults::delayed_domain = f.domains[1];
  faults::delayed_receives = 1;
  expect(!f.bus.begin_cycle(3) && !f.bus.results().failed() && f.bus.feedback_axis_mask() == 1,
         "successful receive ioctl without one reply refreshes only the returned domain");
  expect(f.domains[0]->input_receipts == 2 && f.domains[1]->input_receipts == 1 &&
         f.domains[1]->datagram_state == SimDatagramState::Sent,
         "missing datagram remains outstanding without fabricated receipt");
  expect(f.input(0) == 210 && f.input(1) == 0 && f.bus.domain_state().wc_state == EC_WC_ZERO,
         "absent reply leaves old input bytes and incomplete group evidence");
  for (unsigned i = 0; i < 3; ++i)
    expect(f.bus.query_state().succeeded() && f.bus.feedback_axis_mask() == 1,
           "diagnostic queries do not complete an absent reply");

  f.plant(211, 311);
  expect(!f.bus.begin_cycle(4) && f.bus.feedback_axis_mask() == 0 && !f.bus.results().failed(),
         "late reply without new submission cannot reuse consumed eligibility");
  expect(f.domains[1]->input_receipts == 2 && f.input(1) == 311 && f.input(0) == 210 &&
         f.bus.domain_state().wc_state == EC_WC_COMPLETE,
         "late reply can update cached data but does not become a new qualified cycle sample");
  f.send(3);
  expect(f.bus.begin_cycle(5) && f.bus.feedback_axis_mask() == 3 &&
         f.input(0) == 211 && f.input(1) == 311,
         "fresh full-domain exchange restores receipt eligibility");
}

void plant_progress_and_pdo_presentation_are_independent() {
  SimFaultControls controls;
  controls.enable_delay_cycles = 2;
  controls.receive_fail_from_call = 3; // First actual reply processing fails.
  controls.receive_fail_for_calls = 1;
  Fixture f(controls);
  expect(!f.bus.begin_cycle(1), "initial plant observation has no reply");
  for (unsigned axis = 0; axis < 2; ++axis) {
    EC_WRITE_U16(f.domains[axis]->data.data() + f.cw_offsets[axis], 15);
    EC_WRITE_S32(f.domains[axis]->data.data() + f.target_offsets[axis], 300 + axis);
    f.domains[axis]->data[f.mode_offsets[axis]] = 8;
  }
  f.send(3);
  expect(!f.bus.begin_cycle(2) && f.slaves[0]->enable_wait_cycles == 1 && f.input(0) == 0,
         "plant progresses under submitted outputs even when receive fails");
  expect(!f.bus.begin_cycle(3) && f.slaves[0]->enable_wait_cycles == 2 && f.input(0) == 0,
         "delayed response cannot restore already consumed input evidence");
  expect(f.bus.submit(0).succeeded(), "clock-only send after successful output delivery");
  expect(!f.bus.begin_cycle(4) && f.slaves[0]->position_counts == 300 && f.input(0) == 0,
         "physical progression without another domain exchange does not refresh PDOs");
  f.send(3);
  expect(f.bus.begin_cycle(5) && f.input(0) == 300 && f.input(1) == 301,
         "new reply presents independently progressed plant state");
}
}

int main() {
  initial_full_clock_partial_and_double_begin();
  old_setup_receipt_is_not_a_cycle_receipt();
  shared_domain_mask_is_indivisible();
  failed_queue_and_send_create_no_receipt();
  failed_receive_and_process_cannot_replay_receipt();
  failed_application_time_and_query_cannot_replay_receipt();
  incomplete_receipt_remains_cached_until_new_exchange();
  successful_receive_with_absent_then_late_domain();
  plant_progress_and_pdo_presentation_are_independent();
  std::puts("R29 candidate receipt tests passed");
}
