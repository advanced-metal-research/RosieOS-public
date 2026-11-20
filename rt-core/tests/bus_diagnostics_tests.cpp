// Exercise a topology change between the two real diagnostic API calls. A
// mixed count must not reach the cyclic owner's readiness snapshot.
#define ROSIE_RT_SIMULATION 1
// Unverified fixture build identity; no production binary version is implied.
#define ROSIE_RT_VERSION "bus-diagnostics-test"
#include "ecrt_detect.hpp"
bool change_between_queries = false;
bool identity_unavailable = false;
int failed_identity_read(ec_master_t* master, uint16_t position, ec_slave_info_t* slave) {
  if (identity_unavailable && position == 9) return -EIO;
  return ecrt_master_get_slave(master, position, slave);
}
#define ecrt_master_get_slave failed_identity_read
int topology_race_master_state(ec_master_t* master, ec_master_state_t* state) {
  const int result = ecrt_master_state(master, state);
  if (change_between_queries) --state->slaves_responding;
  return result;
}
#define ecrt_master_state topology_race_master_state
#include "engine/cycle_machine.hpp"
#include <cassert>
namespace {
#include "engine/commissioning_worker.hpp"
#include "engine/status_publisher.hpp"
}
// TestBusFailedIdentityPublication preserves each failed observation, preventing
// nine admitted axes from hiding a tenth physical responder with unreadable SII.
void TestBusFailedIdentityPublication(bool hold, bool previously_readable) {
  Options options; options.num_axes = 9; options.bus.hold = hold;
  for (uint16_t position = 0; position < 9; ++position) options.slave_position[position] = position;
  if (previously_readable) options.bus.unused.push_back({4194304,1813,12024,9,true});
  OptionsParseResult parsed;
  auto machine = std::make_unique<MachineState>(options, parsed);
  ec_master_t master; master.active = true;
  for (uint16_t position = 0; position < 9; ++position)
    machine->diagnostic_sc[position] = ecrt_master_slave_config(&master, 0, position, 4194304, 1813);
  if (previously_readable) master.faults.extra_slaves.push_back({9,4194304,1813,12024,true});
  machine->shared_master.store(&master);
  assert(machine->topology_monitor.sample(&master, options, true));
  machine->diagnostic_handles_ready.store(true);
  machine->collect_drive_diagnostics();
  assert(machine->drive_diagnostics_generation.load() == 1);
  if (!previously_readable) master.faults.extra_slaves.push_back({9,4194304,1813,12024,true});
  identity_unavailable = true;
  // Unverified injected count: three consecutive failed SII observations.
  for (unsigned observation = 0; observation < 3; ++observation) {
    machine->collect_drive_diagnostics();
    assert(machine->drive_diagnostics_generation.load() == observation + 2);
    rosie::rtcore::BusTopologySnapshot topology{};
    assert(machine->topology_snapshot.read(topology));
    assert(topology.generation == observation + 3 && topology.count == 1);
    assert(topology.admitted == hold && topology.responding_expected == 10);
    const auto& tenth = topology.slaves[0];
    assert(tenth.present && tenth.identity.position == 9 && !tenth.identity_readable);
    assert(tenth.disposition == rosie::rtcore::BusSlaveDisposition::IdentityUnreadable);
    std::ostringstream json;
    rosie::rtcore::append_bus_json(json, options.bus, topology);
    assert(json.str().find("\"disposition\":\"identity_unreadable\"") != std::string::npos);
    assert(json.str().find("\"observed_vendor_id\":null") != std::string::npos);
    assert(json.str().find(previously_readable ? "\"vendor_id\":4194304" : "\"vendor_id\":null") != std::string::npos);
    MachineState::DriveDiagnostics result{};
    assert(machine->drive_diagnostics.read(result));
    assert(result.topology_admitted == hold && result.responding_expected == 10 && result.master.slaves_responding == 10);
    assert(master.slaves.size() == 9);
    for (const auto& axis : master.slaves) assert(axis->position < 9 && axis->entries.empty() && axis->sdos.empty());
  }
  // Bring-up uses the same unknown-identity policy and still counts the device.
  BusTopologyMonitor initial;
  assert(initial.sample(&master, options, true) == hold);
  assert(initial.snapshot.generation == 1 && initial.snapshot.responding_expected == 10);
  identity_unavailable = false;
  machine->collect_drive_diagnostics();
  MachineState::DriveDiagnostics result{};
  assert(machine->drive_diagnostics.read(result));
  assert(result.topology_admitted == (hold || previously_readable) && result.master.slaves_responding == 10);
  assert(machine->topology_monitor.snapshot.slaves[0].identity_readable);
  machine->shared_master.store(nullptr);
}

int main() {
  for (bool hold : {false, true})
    for (bool previously_readable : {false, true}) TestBusFailedIdentityPublication(hold, previously_readable);
  Options options; options.num_axes = 6; options.bus.hold = true;
  OptionsParseResult parsed;
  for (uint16_t i = 0; i < 6; ++i) options.slave_position[i] = i;
  auto machine = std::make_unique<MachineState>(options, parsed);
  ec_master_t master; master.active = true;
  for (uint16_t i = 0; i < 6; ++i)
    machine->diagnostic_sc[i] = ecrt_master_slave_config(&master, 0, i, 4194304, 1813);
  for (uint16_t i = 6; i < 9; ++i) master.faults.extra_slaves.push_back({i,4194304,1813,12024,true});
  machine->shared_master.store(&master);
  assert(machine->topology_monitor.sample(&master, options, true));
  machine->diagnostic_handles_ready.store(true);
  machine->collect_drive_diagnostics();
  assert(machine->drive_diagnostics_generation.load() == 1);
  MachineState::DriveDiagnostics sample{};
  assert(machine->drive_diagnostics.read(sample));
  assert(sample.responding_expected == 9 && sample.master.slaves_responding == 9 && sample.topology_admitted);
  change_between_queries = true;
  machine->collect_drive_diagnostics();
  assert(machine->drive_diagnostics_generation.load() == 1);
  assert(machine->drive_diagnostics.read(sample));
  assert(sample.responding_expected == 9 && sample.master.slaves_responding == 9);
  change_between_queries = false;
  master.faults.extra_slaves[2].present = false;
  machine->collect_drive_diagnostics();
  assert(machine->drive_diagnostics_generation.load() == 2);
  assert(machine->drive_diagnostics.read(sample));
  assert(sample.responding_expected == 8 && sample.master.slaves_responding == 8 && sample.topology_admitted);
  for (unsigned i = 0; i < 6; ++i) assert(sample.slaves[i].online && sample.slaves[i].operational);
  machine->shared_master.store(nullptr);
  std::puts("bus_diagnostics_tests: PASS (coherent 9 -> 8 responses; six axes remain OP)");
}
