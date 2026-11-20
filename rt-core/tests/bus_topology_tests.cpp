// Pin topology admission against a synthetic scanned bus. Every count and
// identity is absolute; no hardware or process timing determines success.
#define ROSIE_RT_SIMULATION 1
#include "ecrt_detect.hpp"
#include "include/bus_topology.hpp"
#include "include/cell_io.hpp"
#include <cassert>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <sstream>

std::string logs;
void logf(const char* format, ...) {
  char line[1024]; va_list args; va_start(args, format);
  std::vsnprintf(line, sizeof(line), format, args); va_end(args);
  logs += line; logs += '\n';
}
struct Options {
  uint32_t num_axes = 6;
  std::array<uint16_t, 16> slave_position{{0,1,2,3,4,5}};
  rosie::rtcore::BusPolicy bus;
  rosie::rtcore::IoConfig cell_io;
};
#include "engine/bus_topology.hpp"

// A terminal is required independently of unknown-device policy; missing or
// unreadable identity must never become a held device with output permission.
void TestCellIOTopologyAdmission() {
  using Reason = rosie::rtcore::BusAdmissionReason;
  using Disposition = rosie::rtcore::BusSlaveDisposition;
  for (bool hold : {false, true}) {
    Options opt;
    opt.num_axes = 9;
    for (uint16_t i = 0; i < 9; ++i) opt.slave_position[i] = i;
    opt.bus.hold = hold;
    opt.cell_io = rosie::rtcore::parse_cell_io("9|2|4026585352|0|5632|6656|2|3|28672|1|24576|1;consent|0|1|1;gas|0|0|50000000|0|1|1");
    ec_master_t master;
    for (uint16_t i = 0; i < 9; ++i) ecrt_master_slave_config(&master, 0, i, 4194304, 1813);
    BusTopologyMonitor missing;
    assert(!missing.sample(&master, opt, true));
    assert(missing.snapshot.reason == Reason::DeclaredSlaveMissing);
    assert(missing.snapshot.responding_expected == 10 && missing.snapshot.count == 1);
    assert(missing.snapshot.slaves[0].disposition == Disposition::RefusedMissing);
    auto* terminal = ecrt_master_slave_config(&master, 0, 9, 2, 4026585352);
    BusTopologyMonitor topology;
    assert(topology.sample(&master, opt, true));
    assert(topology.snapshot.responding_expected == 10 && topology.snapshot.count == 1);
    assert(topology.snapshot.slaves[0].disposition == Disposition::IoTerminal);
    std::ostringstream json;
    rosie::rtcore::append_bus_json(json, opt.bus, topology.snapshot);
    assert(json.str().find("\"disposition\":\"io_terminal\"") != std::string::npos);
    assert(json.str().find("\"declared_unused\":false") != std::string::npos);
    assert(json.str().find("\"expected_product_code\":4026585352") != std::string::npos);
    master.active = true;
    assert(topology.sample(&master, opt, false));
    for (bool initial : {true, false}) {
      terminal->sii_readable = false;
      assert(!topology.sample(&master, opt, initial));
      assert(topology.snapshot.reason == Reason::IdentityUnreadable);
      terminal->sii_readable = true;
      for (auto* field : {&terminal->vendor, &terminal->product, &terminal->sii_revision}) {
        ++*field;
        assert(!topology.sample(&master, opt, initial));
        assert(topology.snapshot.reason == Reason::DeclaredIdentityMismatch);
        --*field;
      }
      assert(topology.sample(&master, opt, initial));
    }
    terminal->al_state_override = 4;
    assert(!topology.sample(&master, opt, false));
    assert(topology.snapshot.reason == Reason::IoTerminalNotOperational);
    terminal->al_state_override = 0;
    assert(topology.sample(&master, opt, false));
    master.slaves.pop_back();
    assert(!topology.sample(&master, opt, false));
    assert(topology.snapshot.reason == Reason::DeclaredSlaveMissing);
    assert(topology.snapshot.responding_expected == 10);
    assert(topology.snapshot.slaves[0].disposition == Disposition::RefusedMissing);

    // The same physical terminal without an io declaration follows hold/refuse.
    opt.cell_io = {};
    master.faults.extra_slaves.push_back({9,2,4026585352,0,true});
    BusTopologyMonitor unknown;
    assert(unknown.sample(&master, opt, true) == hold);
    assert(unknown.snapshot.responding_expected == 10 && unknown.snapshot.count == 1);
    assert(unknown.snapshot.slaves[0].disposition == (hold ? Disposition::Held : Disposition::RefusedUnknown));
    assert(master.slaves.size() == 9 && master.domains.empty());
  }
}

int main() {
  TestCellIOTopologyAdmission();
  ec_master_t master;
  for (uint16_t i = 0; i < 6; ++i) {
    auto* s = ecrt_master_slave_config(&master, 0, i, 4194304, 1813);
    s->sii_revision = 12024; s->al_state_override = 2;
  }
  for (uint16_t i = 6; i < 9; ++i) master.faults.extra_slaves.push_back({i,4194304,1813,12024,true});
  assert(master.slaves.size() == 6);
  assert(ecrt_master_slave_config(&master, 0, 6, 4194304, 1813) == nullptr);
  auto* domain = ecrt_master_create_domain(&master);
  for (auto& slave : master.slaves) slave->entries.push_back({{0x6040,0,16},2,domain,0});
  assert(sim_expected_working_counter(domain) == 12);
  ec_master_state_t state{};
  master.active = true; ecrt_master_state(&master, &state);
  assert(state.slaves_responding == 9 && state.al_states == 10);
  ec_slave_info_t physical{};
  assert(ecrt_master_get_slave(&master, 6, &physical) == 0 && physical.al_state == 2);
  master.active = false;
  Options options;
  BusTopologyMonitor refused;
  assert(!refused.sample(&master, options, true));
  assert(logs.find("position=6 vendor_id=0x00400000 product_code=0x00000715 revision=0x00002ef8 disposition=refused_unknown") != std::string::npos);
  assert(refused.snapshot.count == 3 && !refused.snapshot.admitted);

  options.bus.hold = true; logs.clear();
  BusTopologyMonitor held;
  assert(held.sample(&master, options, true));
  assert(held.snapshot.count == 3 && held.snapshot.responding_expected == 9 && held.snapshot.generation == 1);
  for (unsigned i = 0; i < 3; ++i) {
    assert(held.snapshot.slaves[i].identity.position == 6+i);
    assert(held.snapshot.slaves[i].identity.vendor_id == 4194304);
    assert(held.snapshot.slaves[i].identity.product_code == 1813);
    assert(held.snapshot.slaves[i].identity.revision == 12024);
    assert(held.snapshot.slaves[i].al_state == 2 && held.snapshot.slaves[i].present);
  }
  const auto initial_log = logs;
  assert(held.sample(&master, options, false));
  assert(logs == initial_log);
  const auto generation = held.snapshot.generation;
  master.scan_busy = true; master.scan_stuck = true;
  assert(!held.sample(&master, options, false));
  assert(held.snapshot.generation == generation && held.snapshot.admitted);
  master.scan_busy = false; master.scan_stuck = false;
  master.faults.extra_slaves[0].product_code = 1814;
  assert(held.sample(&master, options, false));
  assert(logs.find("policy=log_and_continue_axes reason=identity_changed outputs=0") != std::string::npos);
  master.faults.extra_slaves[2].present = false;
  assert(held.sample(&master, options, false));
  assert(held.snapshot.responding_expected == 8 && held.snapshot.count == 3 && !held.snapshot.slaves[2].present);
  assert(sim_expected_working_counter(domain) == 12 && master.slaves.size() == 6);
  assert(logs.find("position=8 present=0") != std::string::npos);
  std::ostringstream json;
  rosie::rtcore::append_bus_json(json, options.bus, held.snapshot);
  assert(json.str().find("\"slave_position\":8") != std::string::npos);
  assert(json.str().find("\"present\":false") != std::string::npos);
  master.faults.extra_slaves[0].product_code = 1813;
  master.faults.extra_slaves[2].present = true;
  options.bus.hold = false;
  for (uint16_t i = 6; i < 9; ++i) options.bus.unused.push_back({4194304,1813,12024,i,true});
  BusTopologyMonitor declared;
  assert(declared.sample(&master, options, true));
  assert(declared.snapshot.count == 3 && declared.snapshot.responding_expected == 9);
  options.bus.unused[1].revision = 12025;
  BusTopologyMonitor mismatch;
  assert(!mismatch.sample(&master, options, true));
  assert(logs.find("disposition=refused_identity") != std::string::npos);
  options.bus.unused[1].revision = 12024;
  master.faults.extra_slaves[0].vendor_id = 1;
  assert(!declared.sample(&master, options, false));
  assert(declared.snapshot.reason == rosie::rtcore::BusAdmissionReason::DeclaredIdentityMismatch);
  assert(declared.snapshot.reason_position == 6);
  assert(!declared.sample(&master, options, true));
  master.faults.extra_slaves[0].vendor_id = 4194304;
  master.faults.extra_slaves[2].present = false;
  assert(!declared.sample(&master, options, false));
  assert(declared.snapshot.responding_expected == 9);
  assert(declared.snapshot.reason == rosie::rtcore::BusAdmissionReason::DeclaredSlaveMissing);
  assert(declared.snapshot.reason_position == 8);
  logs.clear();
  assert(!declared.sample(&master, options, true));
  assert(logs.find("declared_unused missing position=8") != std::string::npos);
  BusTopologyMonitor missing;
  assert(!missing.sample(&master, options, true));
  assert(logs.find("declared_unused missing position=8") != std::string::npos);
  ecrt_master_slave_config(&master, 0, 9, 3, 4);
  assert(!declared.sample(&master, options, false));
  assert(logs.find("position=9 vendor_id=0x00000003 product_code=0x00000004") != std::string::npos);
  // Unsafe AL state at an admitted position must still fail closed.
  master.faults.extra_slaves.erase(master.faults.extra_slaves.begin());
  auto* unsafe = ecrt_master_slave_config(&master, 0, 6, 4194304, 1813);
  unsafe->al_state_override = 8;
  assert(!held.sample(&master, options, false));
  assert(logs.find("policy=refuse_until_PREOP_or_INIT") != std::string::npos);
  std::puts("bus_topology_tests: PASS (9 responses, 6 axes, 3 held; identity and disappearance policies)");
}
