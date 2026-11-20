#include "checked_bus.hpp"
#include "simulated_bus.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

void expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

struct Plant {
  ec_master_t* master = ecrt_request_master(0);
  ec_domain_t* domain = ecrt_master_create_domain(master);
  ec_slave_config_t* slave = ecrt_master_slave_config(master, 0, 0, 1, 2);

  explicit Plant(const SimFaultControls& faults = {}) {
    ec_pdo_entry_info_t outputs[]{{0x6040, 0, 16}, {0x607a, 0, 32}, {0x6060, 0, 8}};
    ec_pdo_entry_info_t inputs[]{{0x6041, 0, 16}, {0x6064, 0, 32}, {0x6061, 0, 8}};
    ec_pdo_info_t pdos[]{{0x1600, 3, outputs}, {0x1a00, 3, inputs}};
    ec_sync_info_t syncs[]{{2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
                           {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE},
                           {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}};
    expect(ecrt_slave_config_pdos(slave, EC_END, syncs) == 0, "configure PDOs");
    for (auto& entry : slave->entries) expect(sim_register(entry, domain) >= 0, "register PDO");
    expect(sim_configure_faults(master, faults), "configure explicit simulator faults");
    expect(ecrt_master_activate(master) == 0, "activate simulated master");
  }
  ~Plant() { ecrt_release_master(master); }
  Plant(const Plant&) = delete;
  Plant& operator=(const Plant&) = delete;

  void command(uint16_t cw, int32_t target) {
    EC_WRITE_U16(sim_object(slave, 0x6040), cw);
    EC_WRITE_S32(sim_object(slave, 0x607a), target);
    sim_write<uint8_t>(sim_object(slave, 0x6060), 8);
  }
  void queue() { expect(ecrt_domain_queue(domain) == 0, "queue process image"); }
  void send() { queue(); expect(ecrt_master_send(master) == 0, "submit process image"); }
  void receive() { expect(ecrt_master_receive(master) == 0, "receive process image"); }
  bool enabled() { return (EC_READ_U16(sim_object(slave, 0x6041)) & 0x6f) == 0x27; }
  int32_t position() { return EC_READ_S32(sim_object(slave, 0x6064)); }
};

void checked_results() {
  using namespace rosie::rtcore;
  static_assert(static_cast<uint8_t>(BusOperation::Send) == 5, "preserve observation operation values");
  static_assert(static_cast<uint8_t>(BusOperation::Process) == 6, "process observation value");
  static_assert(static_cast<uint8_t>(BusOperation::Queue) == 7, "queue observation value");
  static_assert(static_cast<uint8_t>(BusOperation::DomainState) == 8, "domain-state observation value");
  static_assert(!BusResult{}.succeeded(), "unattempted send cannot acknowledge Stop");
  static_assert(checked_bus_result(BusOperation::Send, 0).succeeded(), "zero is success");
  static_assert(checked_bus_result(BusOperation::Send, -5).failed(), "negative errno is failure");
  static_assert(checked_bus_result(BusOperation::Send, std::numeric_limits<int64_t>::max()).succeeded(),
                "preserve wider signed return types without narrowing");
  CycleBusResults cycle;
  expect(!cycle.send_succeeded(), "missing send cannot be successful");
  cycle.record(checked_bus_result(BusOperation::Receive, -11));
  cycle.record(checked_bus_result(BusOperation::SlaveClockSync, -5));
  cycle.record(checked_bus_result(BusOperation::Send, -5));
  expect(cycle.failed() && !cycle.send_succeeded(), "failed send cannot acknowledge Stop");
  expect(cycle.first_failure.operation == BusOperation::Receive &&
             cycle.first_failure.return_code == -11, "retain first cycle failure");
  cycle.record(checked_bus_result(BusOperation::Send, 0));
  expect(cycle.send_succeeded() && cycle.failed(), "later send success cannot erase earlier failure");
}

void default_plant() {
  Plant plant;
  expect(!plant.enabled() && plant.position() == 0, "initial plant is disabled at zero");
  plant.command(0xf, 123);
  plant.send();
  plant.receive();
  expect(plant.enabled() && plant.position() == 123, "default enable has no injected delay");
  plant.command(0, 999);
  expect(ecrt_master_send(plant.master) == 0, "empty send still succeeds");
  plant.receive();
  expect(plant.enabled() && plant.position() == 123, "unqueued image cannot affect drive even after send success");
  plant.send();
  plant.receive();
  expect(!plant.enabled() && plant.position() == 123, "submitted disable retains observed position");
  expect(!sim_configure_faults(plant.master, {}), "active fault controls cannot be mutated");
}

void delayed_enable() {
  SimFaultControls faults;
  faults.enable_delay_cycles = 3;
  Plant plant(faults);
  for (int request = 0; request != 2; ++request) {
    plant.command(0xf, 123 + request);
    for (int cycle = 0; cycle != 3; ++cycle) {
      plant.send(); // Each new observation requires its own domain exchange.
      plant.receive();
      expect(!plant.enabled(), "enable delay must withhold OperationEnabled for exact cycle count");
      expect(plant.position() == (request ? 123 : 0), "delayed drive cannot follow target");
    }
    plant.send();
    plant.receive();
    expect(plant.enabled() && plant.position() == 123 + request, "enable after bounded delay");
    plant.command(0, 0);
    plant.send();
    plant.receive();
    expect(!plant.enabled(), "disable cancels enable delay generation");
  }
}

void failed_send() {
  SimFaultControls faults;
  faults.send_fail_from_call = 2;
  faults.send_fail_for_calls = 2;
  faults.send_error = -11;
  Plant plant(faults);
  plant.command(0xf, 123);
  plant.send();
  plant.receive();
  plant.command(0, 0);
  for (int attempt = 0; attempt != 2; ++attempt) {
    plant.queue();
    expect(ecrt_master_send(plant.master) == -11, "send returns exact injected errno");
    expect(plant.domain->queued, "failed send retains pending queue");
    plant.receive();
    expect(plant.enabled() && plant.position() == 123, "failed inhibited image was not delivered");
  }
  plant.send();
  plant.receive();
  expect(!plant.enabled(), "successful retry delivers inhibited image");
}

void failed_queue() {
  SimFaultControls faults;
  faults.queue_fail_from_call = 2;
  faults.queue_fail_for_calls = 2;
  faults.queue_error = -11;
  Plant plant(faults);
  plant.command(0xf, 123);
  plant.send();
  plant.receive();
  expect(!plant.domain->queued, "successful send consumes domain queue");
  plant.command(0, 0);
  for (int attempt = 0; attempt != 2; ++attempt) {
    expect(ecrt_domain_queue(plant.domain) == -11, "queue returns exact injected errno");
    expect(!plant.domain->queued, "failed queue does not enqueue inhibited image");
    expect(ecrt_master_send(plant.master) == 0, "send without queued domain can still succeed");
    plant.receive();
    expect(plant.enabled() && plant.position() == 123, "send success cannot deliver failed domain queue");
  }
  plant.send();
  plant.receive();
  expect(!plant.enabled(), "fresh successful queue and send deliver inhibition");
}

void failed_process() {
  SimFaultControls faults;
  faults.process_fail_from_call = 2;
  faults.process_fail_for_calls = 2;
  faults.process_error = -11;
  Plant plant(faults);
  auto* second = ecrt_master_create_domain(plant.master);
  expect(ecrt_domain_process(plant.domain) == 0, "first process succeeds");
  expect(ecrt_domain_process(second) == -11, "process count is master-wide across domains");
  expect(ecrt_domain_process(plant.domain) == -11, "process failure spans exact call interval");
  expect(ecrt_domain_process(second) == 0, "processing resumes after interval");
}

void split_domain_queue() {
  Plant plant;
  auto* second = ecrt_master_create_domain(plant.master);
  second->used = 1;
  second->data[0] = 42;
  plant.command(0xf, 123);
  plant.queue();
  expect(ecrt_master_send(plant.master) == 0, "send first domain");
  expect(second->submitted[0] == 0, "unqueued second domain is not submitted");
  expect(ecrt_domain_queue(second) == 0, "queue second domain");
  second->data[0] = 43;
  expect(ecrt_master_send(plant.master) == 0, "send second domain");
  expect(second->submitted[0] == 43 && !second->queued, "send snapshots currently queued bytes");
  expect(plant.master->queue_calls == 2, "queue call count is master-wide");
}

void failed_dc_sync() {
  SimFaultControls faults;
  faults.dc_fail_from_call = 2;
  faults.dc_fail_for_calls = 2;
  faults.dc_error = -11;
  Plant plant(faults);
  expect(ecrt_master_sync_slave_clocks(plant.master) == 0, "first slave sync succeeds");
  expect(ecrt_master_sync_reference_clock(plant.master) == 0, "reference sync does not consume slave sync interval");
  expect(ecrt_master_sync_slave_clocks(plant.master) == -11, "slave sync returns exact injected errno");
  expect(ecrt_master_sync_slave_clocks(plant.master) == -11, "slave sync failure spans exact interval");
  expect(ecrt_master_sync_slave_clocks(plant.master) == 0, "slave sync resumes after interval");
}

void failed_receive() {
  SimFaultControls faults;
  faults.receive_fail_from_call = 2; // Activation is receive call one.
  faults.receive_fail_for_calls = 2;
  faults.receive_error = -11;
  Plant plant(faults);
  plant.command(0xf, 123);
  plant.send();
  for (int attempt = 0; attempt != 2; ++attempt) {
    expect(ecrt_master_receive(plant.master) == -11, "receive returns exact injected errno");
    expect(!plant.enabled() && plant.position() == 0, "failed receive cannot publish new feedback");
  }
  plant.receive();
  expect(plant.enabled() && plant.position() == 123, "next successful receive publishes feedback");
}

void disabled_displacement() {
  SimFaultControls faults;
  faults.disabled_displacement_counts = 1000;
  Plant plant(faults);
  plant.command(0, 0);
  plant.send();
  plant.receive();
  expect(plant.position() == 0, "startup disabled state does not trigger displacement");
  plant.command(0xf, 123);
  plant.send();
  plant.receive();
  plant.command(0, 0);
  plant.send();
  plant.receive();
  expect(!plant.enabled() && plant.position() == 1123, "first disable after enabled injects displacement");
  plant.receive();
  expect(plant.position() == 1123, "displacement does not repeat while disabled");
  plant.command(0xf, 1123);
  plant.send();
  plant.receive();
  plant.command(0, 0);
  plant.send();
  plant.receive();
  expect(plant.position() == 1123, "displacement is one-shot across rearm");
}

void fault_configuration() {
  auto* master = ecrt_request_master(0);
  SimFaultControls faults;
  faults.send_error = 0;
  expect(!sim_configure_faults(master, faults), "zero cannot represent injected failure");
  faults.send_error = -5;
  faults.send_fail_for_calls = 1;
  expect(!sim_configure_faults(master, faults), "duration without start is invalid");
  faults.send_fail_from_call = 1;
  faults.send_fail_for_calls = 0;
  expect(sim_configure_faults(master, faults), "persistent failure is valid");
  expect(ecrt_master_send(master) == -5 && ecrt_master_send(master) == -5,
         "zero duration means persistent failure");
  expect(!sim_fault_on_call(10, 0, 0), "zero start disables injection");
  const uint64_t last = std::numeric_limits<uint64_t>::max();
  expect(sim_fault_on_call(last, last - 1, 3), "fault interval avoids overflowing first plus count");
  faults.queue_fail_for_calls = 1;
  expect(!sim_configure_faults(master, faults), "queue duration without start is invalid");
  faults.queue_fail_for_calls = 0;
  faults.process_error = 0;
  expect(!sim_configure_faults(master, faults), "process error must be negative");
  ecrt_release_master(master);
}

} // namespace

int main() {
  checked_results();
  default_plant();
  delayed_enable();
  failed_send();
  failed_receive();
  failed_queue();
  failed_process();
  split_domain_queue();
  failed_dc_sync();
  disabled_displacement();
  fault_configuration();
  std::cout << "bus boundary tests passed\n";
}
