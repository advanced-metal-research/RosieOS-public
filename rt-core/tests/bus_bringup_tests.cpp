// Compile the live constructor against a recording synthetic master. Keep
// physical assignment, fixed preset entries and scanned registration separate.
#define ROSIE_RT_SIMULATION 1
#include "ecrt_detect.hpp"
#undef ROSIE_RT_SIMULATION
#define ROSIE_RT_SIMULATION 0
// Unverified fixture identity; this binary is never a daemon build version.
#define ROSIE_RT_VERSION "bringup-test"
#include <cassert>
#include <cstdio>
#include <map>
#include <string>

enum class Fault { None, Ignore, Late, Refuse, Transient, Entry, Count, AssignmentCount, State, Cache, ScanFast, ScanStuck, Reverify, Download, Startup, PowerCycleReady, PowerCycleMismatch, PowerCycleUnavailable, PowerCycleWide };
const ec_sync_info_t* scanned_syncs = nullptr;
Fault fault = Fault::None;
bool change_assignment = false;
bool explicit_assignment = false;
unsigned fault_axis = 0, registrations = 0, activations = 0, deactivations = 0, mapping_calls = 0;
std::array<unsigned, 9> uploads{}, downloads{}, assignment_reads{};
// Measured CoE assignment objects: SM2 RxPDO 0x1c12 and SM3 TxPDO 0x1c13.
uint16_t assignment = 0x1c13;
uint16_t selected = 0, original = 0;
struct Write { uint16_t position, index; uint8_t sub; std::vector<uint8_t> data; };
std::vector<Write> writes, delayed;
// Startup objects queued through IgH (written at activation and replayed at
// every reconfiguration), and what each drive held before startup.
struct Queued {
  uint16_t position, index; uint8_t sub; int64_t value;
  bool operator==(const Queued& o) const { return position == o.position && index == o.index && sub == o.sub && value == o.value; }
};
std::vector<Queued> queued;
std::map<uint32_t, std::vector<uint8_t>> held;

int recorded_config_sdo(ec_slave_config_t* slave, uint16_t index, uint8_t sub, const uint8_t* data, size_t size) {
  int64_t value = 0;
  for (size_t i = 0; i < size; ++i) value |= int64_t(data[i]) << (8 * i);
  queued.push_back({slave->position, index, sub, value});
  return ecrt_slave_config_sdo(slave, index, sub, data, size);
}

ec_slave_config_t* recorded_slave_config(ec_master_t* master, uint16_t alias,
    uint16_t position, uint32_t vendor, uint32_t product) {
  auto* slave = ecrt_master_slave_config(master, alias, position, vendor, product);
  for (const auto& object : held) slave->sdos[object.first] = object.second;
  if (fault != Fault::PowerCycleMismatch && fault != Fault::PowerCycleUnavailable &&
      fault != Fault::PowerCycleWide) slave->sdos[0x200008] = {1, 0};
  if (fault == Fault::PowerCycleMismatch) slave->sdos[0x200008] = {2, 0};
  if (fault == Fault::PowerCycleWide) slave->sdos[0x200008] = {1};
  if (master->reservations == 2) {
    assert(master->rescan_requests == 1 && master->releases == 2 && !master->scan_busy);
    if (fault == Fault::Reverify && position == fault_axis) slave->sdos[(uint32_t(assignment) << 8 | 1)][0] ^= 1;
    return slave;
  }
  assert(ecrt_slave_config_pdos(slave, EC_END, scanned_syncs) == 0);
  if (change_assignment) slave->sdos[(uint32_t(assignment) << 8 | 1)] = {uint8_t(original), uint8_t(original >> 8)};
  if (position == fault_axis) {
    if (fault == Fault::Entry) slave->sdos[(uint32_t(selected) << 8) | 1][0] ^= 8;
    if (fault == Fault::Count) slave->sdos[uint32_t(selected) << 8][0] += 1;
    if (fault == Fault::AssignmentCount) slave->sdos[(uint32_t(assignment) << 8)][0] = 2;
    if (fault == Fault::Cache || fault == Fault::ScanFast || fault == Fault::ScanStuck || fault == Fault::Reverify) {
      slave->scanned_assignment[assignment - 0x1c10] = original;
      slave->entries.pop_back(); // The old cache also has different entry geometry.
    }
    if (fault == Fault::ScanStuck) master->scan_stuck = true;
  }
  return slave;
}
int recorded_pdos(ec_slave_config_t*, unsigned, const ec_sync_info_t*) { ++mapping_calls; return -1; }
int recorded_sync(ec_slave_config_t*, uint8_t, ec_direction_t, ec_watchdog_mode_t) { return 0; }
int recorded_clear(ec_slave_config_t*, uint8_t) { ++mapping_calls; return explicit_assignment ? 0 : -1; }
int recorded_add(ec_slave_config_t* slave, uint8_t sync, uint16_t preset) {
  ++mapping_calls;
  if (!explicit_assignment) return -1;
  // IgH queues assignment configuration; the physical object changes only at activation.
  delayed.push_back({slave->position, uint16_t(0x1c10 + sync), 0, {1}});
  delayed.push_back({slave->position, uint16_t(0x1c10 + sync), 1, {uint8_t(preset), uint8_t(preset >> 8)}});
  return 0;
}
int recorded_upload(ec_master_t* master, uint16_t pos, uint16_t index, uint8_t sub,
    uint8_t* data, size_t capacity, size_t* size, uint32_t* abort) {
  if (!master->active && index == 0x1c12 && sub == 0) ++uploads[pos];
  if (!master->active && index == assignment && sub == 0) ++assignment_reads[pos];
  if (pos == fault_axis && ((fault == Fault::Refuse && index == assignment) ||
      (fault == Fault::Transient && uploads[pos] == 1 && index == 0x1c12 && sub == 0))) {
    *abort = 0x05040000; *size = 0; return -EIO;
  }
  return ecrt_master_sdo_upload(master, pos, index, sub, data, capacity, size, abort);
}
int recorded_download(ec_master_t* master, uint16_t pos, uint16_t index, uint8_t sub,
    const uint8_t* data, size_t size, uint32_t* abort) {
  assert(!master->active && index == assignment && (sub == 0 || sub == 1));
  assert(size == (sub == 0 ? 1u : 2u));
  ++downloads[pos];
  writes.push_back({pos, index, sub, {data, data + size}});
  if (pos == fault_axis) {
    if (fault == Fault::Download) { *abort = 0x06010002; return -EIO; }
    if (fault == Fault::Ignore) { *abort = 0; return 0; }
    if (fault == Fault::Late) { delayed.push_back(writes.back()); *abort = 0; return 0; }
  }
  return ecrt_master_sdo_download(master, pos, index, sub, data, size, abort);
}
int recorded_state(ec_master_t* master, uint16_t pos, ec_slave_info_t* info) {
  const int rc = ecrt_master_get_slave(master, pos, info);
  if (pos == fault_axis && fault == Fault::State) info->al_state = 8;
  return rc;
}
// Complete the queued scan between polls without an observed busy transition;
// unchanged count/state/busy must not suppress the terminal mapping proof.
int recorded_master_info(ec_master_t* master, ec_master_info_t* info) {
  const int rc = ecrt_master(master, info);
  if (fault == Fault::ScanFast || fault == Fault::Reverify) info->scan_busy = 0;
  return rc;
}
int recorded_register(ec_domain_t* domain, const ec_pdo_entry_reg_t* entries) {
  assert(!domain->master->active);
  for (const auto& slave : domain->master->slaves) {
    assert(uploads[slave->position] >= 1);
    assert(slave->sdos.at((uint32_t(assignment) << 8)) == std::vector<uint8_t>{1});
    assert(slave->sdos.at((uint32_t(assignment) << 8 | 1)) == (std::vector<uint8_t>{uint8_t(selected), uint8_t(selected >> 8)}));
  }
  ++registrations;
  return ecrt_domain_reg_pdo_entry_list(domain, entries);
}
int recorded_activate(ec_master_t* master) {
  ++activations;
  if (explicit_assignment) {
    assert(registrations == 0 && mapping_calls == 36 && delayed.size() == 36);
    for (const auto& slave : master->slaves) {
      assert(uploads[slave->position] == 0 && downloads[slave->position] == 0);
      assert(slave->sdos.at((uint32_t(assignment) << 8 | 1)) == (std::vector<uint8_t>{uint8_t(original), uint8_t(original >> 8)}));
    }
  } else assert(registrations > 0);
  for (const auto& write : delayed) {
    uint32_t abort = 0;
    assert(ecrt_master_sdo_download(master, write.position, write.index, write.sub,
        write.data.data(), write.data.size(), &abort) == 0);
  }
  return ecrt_master_activate(master);
}
int recorded_deactivate(ec_master_t* master) { ++deactivations; return ecrt_master_deactivate(master); }

#define ecrt_master_slave_config recorded_slave_config
#define ecrt_slave_config_pdos recorded_pdos
#define ecrt_slave_config_sync_manager recorded_sync
#define ecrt_slave_config_pdo_assign_clear recorded_clear
#define ecrt_slave_config_pdo_assign_add recorded_add
#define ecrt_domain_reg_pdo_entry_list recorded_register
#define ecrt_master_sdo_upload recorded_upload
#define ecrt_master_sdo_download recorded_download
#define ecrt_master_get_slave recorded_state
#define ecrt_master recorded_master_info
#define ecrt_master_activate recorded_activate
#define ecrt_master_deactivate recorded_deactivate
#define ecrt_slave_config_sdo recorded_config_sdo
#include "engine/cycle_machine.hpp"
namespace {
#include "engine/commissioning_worker.hpp"
}

int main(int argc, char** argv) {
  Options options;
  auto parsed = parse_options(argc, argv, &options);
  assert(!parsed.exit_code && options.num_axes == 1);
  options.use_dc = true;
  options.preop_to_safeop_timeout_ms = 0;
  options.num_axes = 9;
  for (unsigned axis = 0; axis < 9; ++axis) {
    options.slave_position[axis] = axis;
    options.axis[axis] = options.axis[0];
    parsed.axis_profiles[axis] = parsed.axis_profiles[0];
  }
  const auto& pdo = parsed.axis_profiles[0].pdo;
  selected = pdo.tx_pdo;
  original = selected == 0x1b04 ? 0x1b02 : 0x1b04;
  BusHandles::PdoRuntime scanned;
  for (const auto& e : pdo.rx_entries) scanned.rx.push_back({e.index, e.subindex, e.bits});
  for (const auto& e : pdo.tx_entries) scanned.tx.push_back({e.index, e.subindex, e.bits});
  scanned.pdos = {{{pdo.rx_pdo, unsigned(scanned.rx.size()), scanned.rx.data()},
      {pdo.tx_pdo, unsigned(scanned.tx.size()), scanned.tx.data()}}};
  scanned.syncs = {{{2, EC_DIR_OUTPUT, 1, &scanned.pdos[0], EC_WD_ENABLE},
      {3, EC_DIR_INPUT, 1, &scanned.pdos[1], EC_WD_DISABLE},
      {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}}};
  scanned_syncs = scanned.syncs.data();

  auto run = [&](const char* name, Fault injected, bool change, bool restore, bool success, unsigned axis,
                 bool explicit_mode = false) {
    const auto label = std::string(name) + (assignment == 0x1c12 ? "Rx" : "");
    name = label.c_str();
    std::printf("BEGIN: %s axis=%u\n", name, axis);
    std::fflush(stdout);
    sim_rescan_master.reset();
    // Unverified observation bound: 100 ms for completed scans; zero for the
    // stuck fixture proves the first expired observation cannot grant success.
    options.preop_to_safeop_timeout_ms = injected == Fault::Cache || injected == Fault::ScanFast || injected == Fault::Reverify ? 100 : 0;
    fault = injected; change_assignment = change; fault_axis = axis;
    explicit_assignment = explicit_mode;
    options.explicit_pdo_config = explicit_mode;
    registrations = activations = deactivations = mapping_calls = 0;
    uploads = {}; downloads = {}; assignment_reads = {}; writes.clear(); delayed.clear(); g_stop.store(false);
    for (auto& profile : parsed.axis_profiles) profile.pdo.restore_assignment_on_exit = restore;
    const auto startup = parsed.startup_sdos;
    if (injected == Fault::PowerCycleReady || injected == Fault::PowerCycleMismatch ||
        injected == Fault::PowerCycleUnavailable || injected == Fault::PowerCycleWide) {
      assert(parsed.startup_sdos[0].index == 0x2000 && parsed.startup_sdos[0].subindex == 8);
      assert(parsed.startup_sdos[0].values[0] == 1 &&
          parsed.startup_sdos[0].effective_after_power_cycle);
    }
    if (injected == Fault::Startup) {
      assert(!parsed.startup_sdos.empty());
      parsed.startup_sdos[0].index = assignment;
    }
    auto machine = std::make_unique<MachineState>(options, parsed);
    std::atomic<int> exit_code{0};
    BusHandles bus(options, parsed, machine->shared_master, machine->latest_feedback, exit_code);
    assert(bus.ecrt_ok == success && exit_code.load() == (success ? 0 : 1));
    if (success) for (const auto& slave : bus.master.load()->slaves) {
      assert(slave->sync0_cycle_ns == options.cycle_ns);
      assert(slave->sync0_shift_ns == static_cast<int32_t>(options.cycle_ns / 2));
    }
    assert(activations == unsigned(success) && registrations == unsigned(success && !explicit_mode));
    assert(mapping_calls == (explicit_mode ? 36u : 0u));
    if (explicit_mode) {
      assert(bus.master.load()->active && bus.master.load()->rescan_requests == 0);
      assert(writes.empty());
      for (auto count : assignment_reads) assert(count == 0);
      // Unverified OP/complete-exchange fixture: continuous readiness begins at
      // 1000000000 ns; runtime mapping must wait until exactly 1500000000 ns.
      machine->bus_ready.store(true);
      machine->latest_feedback.startup_ready = 1;
      machine->latest_feedback.domain_wc_state = EC_WC_COMPLETE;
      machine->update_mailbox_readiness(true, 1000000000);
      machine->mailbox_clock = +[]() -> uint64_t { return 1499999999; };
      assert(machine->verify_axis_mapping(0) == MappingReadback::Unavailable);
      machine->mailbox_clock = +[]() -> uint64_t { return 1500000000; };
      for (unsigned i = 0; i < 9; ++i)
        assert(machine->verify_axis_mapping(i) == MappingReadback::Matched);
    }
    if ((injected == Fault::Cache || injected == Fault::ScanFast) && !restore) {
      assert(bus.master.load()->rescan_requests == 1 && bus.master.load()->reservations == 2);
      assert(bus.master.load()->releases == 2);
      for (auto count : uploads) assert(count == 3);
      for (auto count : downloads) assert(count == 3);
    }
    if (injected == Fault::ScanStuck) {
      assert(!bus.master && sim_rescan_master && sim_rescan_master->rescan_requests == 1);
      assert(sim_rescan_master->reservations == 1 && sim_rescan_master->scan_observations == 0);
    }
    if (!change) for (auto count : downloads) assert(count == 0);
    if (injected == Fault::Refuse || injected == Fault::State) assert(downloads[axis] == 0);
    if (injected == Fault::Refuse) assert(assignment_reads[axis] == 3);
    if (injected == Fault::Download) assert(downloads[axis] == 3);
    if (injected == Fault::Transient) assert(uploads[axis] == 3);
    if (injected == Fault::Late) assert(delayed.size() == 3);
    if (change && injected == Fault::None && !explicit_mode) {
      assert(writes.size() == 27);
      for (unsigned i = 0; i < 9; ++i) {
        assert(writes[3*i].position == i && writes[3*i].sub == 0 && writes[3*i].data == std::vector<uint8_t>{0});
        assert(writes[3*i+1].sub == 1 && writes[3*i+1].data == (std::vector<uint8_t>{uint8_t(selected), uint8_t(selected >> 8)}));
        assert(writes[3*i+2].sub == 0 && writes[3*i+2].data == std::vector<uint8_t>{1});
      }
    }
    // Defaults preserve the selected assignment, including when the cache
    // refuses activation. Optional rollback must also run on that failure path.
    if (change && !restore && (injected == Fault::None || injected == Fault::Cache))
      for (const auto& slave : machine->shared_master.load()->slaves)
        assert(slave->sdos.at((uint32_t(assignment) << 8 | 1)) == (std::vector<uint8_t>{uint8_t(selected), uint8_t(selected >> 8)}));
    bus.restore_assignments();
    assert(deactivations == unsigned(change && restore && success));
    if (change && restore && (injected == Fault::None || injected == Fault::Cache)) {
      assert(writes.size() == 54);
      for (const auto& slave : machine->shared_master.load()->slaves)
        assert(slave->sdos.at((uint32_t(assignment) << 8 | 1)) == (std::vector<uint8_t>{uint8_t(original), uint8_t(original >> 8)}));
    }
    ecrt_release_master(machine->shared_master.exchange(nullptr));
    parsed.startup_sdos = startup;
    std::printf("PASS: %s\n", name);
    std::fflush(stdout);
  };
  for (bool rx : {false, true}) {
    assignment = rx ? 0x1c12 : 0x1c13;
    selected = rx ? pdo.rx_pdo : pdo.tx_pdo;
    original = rx ? 0x1701 : selected == 0x1b04 ? 0x1b02 : 0x1b04;
    run("TestPowerCycleKeyPreconfigured", Fault::PowerCycleReady, false, false, true, 0);
    run("TestPowerCycleKeyMismatchRefused", Fault::PowerCycleMismatch, false, false, false, 0);
    run("TestPowerCycleKeyUnreadableRefused", Fault::PowerCycleUnavailable, false, false, false, 0);
    run("TestPowerCycleKeyWrongWidthRefused", Fault::PowerCycleWide, false, false, false, 0);
    run("TestPREOPAlreadySelected", Fault::None, false, false, true, 0);
    run("TestPREOPAssignmentWriteOrdering", Fault::None, true, false, true, 0);
    run("TestPREOPRestoreAssignment", Fault::None, true, true, true, 0);
    run("TestPREOPRestoreAlreadySelectedNoWrite", Fault::None, false, true, true, 0);
    run("TestPREOPRestoreAfterRefusedBringup", Fault::Cache, true, true, false, 8);
    run("TestPREOPStartupAssignmentConflict", Fault::Startup, false, false, false, 0);
    run("TestExplicitAssignmentAtActivation", Fault::None, true, false, true, 0, true);
    run("TestFixedPresetRefusesActivationOnlyAssignment", Fault::Late, true, false, false, 0);
    for (unsigned axis : {0u, 8u}) {
      run("TestPREOPIgnoredAssignment", Fault::Ignore, true, false, false, axis);
      run("TestPREOPLateAssignment", Fault::Late, true, false, false, axis);
      run("TestPREOPMailboxRefused", Fault::Refuse, true, false, false, axis);
      run("TestPREOPDownloadRefused", Fault::Download, true, false, false, axis);
      run("TestPREOPTransientRead", Fault::Transient, false, false, true, axis);
      run("TestPREOPEntryMismatch", Fault::Entry, false, false, false, axis);
      run("TestPREOPCountMismatch", Fault::Count, false, false, false, axis);
      run("TestPREOPAssignmentCountMismatch", Fault::AssignmentCount, false, false, false, axis);
      run("TestPREOPWrongState", Fault::State, true, false, false, axis);
      run("TestPREOPAutomaticRescan", Fault::Cache, true, false, true, axis);
      run("TestPREOPRescanFastCompletion", Fault::ScanFast, true, false, true, axis);
      run("TestPREOPRescanBoundedRefusal", Fault::ScanStuck, true, false, false, axis);
      run("TestPREOPRescanReverifyRefusal", Fault::Reverify, true, false, false, axis);
    }
  }
  assignment = 0x1c13; selected = pdo.tx_pdo;
  original = selected == 0x1b04 ? 0x1b02 : 0x1b04;

  // Write on change through the live constructor. `prior` is what drive 0
  // holds per descriptor before startup (absent: unreadable); `want` is the
  // exact sequence IgH is asked to write to it. Verification inputs stay the
  // configured values whether an object is written or skipped, and no startup
  // write ever reaches the HM35 objects (607C, 6098, 60E6).
  using Prior = std::map<size_t, int64_t>;
  auto run_startup = [&](const char* name, auto mutate, auto prior_of, std::vector<Queued> want, bool success) {
    std::printf("BEGIN: %s\n", name);
    std::fflush(stdout);
    sim_rescan_master.reset();
    options.preop_to_safeop_timeout_ms = 0;
    fault = Fault::None; change_assignment = false; fault_axis = 0; explicit_assignment = false;
    options.explicit_pdo_config = false;
    registrations = activations = deactivations = mapping_calls = 0;
    uploads = {}; downloads = {}; assignment_reads = {}; writes.clear(); delayed.clear(); queued.clear(); g_stop.store(false);
    for (auto& profile : parsed.axis_profiles) profile.pdo.restore_assignment_on_exit = false;
    const auto startup = parsed.startup_sdos;
    mutate(parsed.startup_sdos);
    held.clear();
    for (const auto& [d, value] : Prior(prior_of(parsed.startup_sdos))) {
      const auto& descriptor = parsed.startup_sdos[d];
      const auto raw = startup_sdo_encode(descriptor.type, value);
      held[(uint32_t(descriptor.index) << 8) | descriptor.subindex] = {raw.begin(), raw.begin() + startup_sdo_size(descriptor.type)};
    }
    auto machine = std::make_unique<MachineState>(options, parsed);
    std::atomic<int> exit_code{0};
    BusHandles bus(options, parsed, machine->shared_master, machine->latest_feedback, exit_code);
    assert(bus.ecrt_ok == success && exit_code.load() == (success ? 0 : 1));
    for (const auto& q : queued) assert(q.index != 0x607c && q.index != 0x6098 && q.index != 0x60e6);
    if (success) {
      std::vector<Queued> drive0;
      for (const auto& q : queued) if (q.position == 0) drive0.push_back(q);
      assert(drive0 == want);
      for (size_t d = 0; d < parsed.startup_sdos.size(); ++d) {
        const auto& feedback = machine->latest_feedback.startup_drive_config_feedback[d];
        assert(feedback.configured[0] == 1u && feedback.commanded[0] == parsed.startup_sdos[d].values[0]);
      }
    }
    ecrt_release_master(machine->shared_master.exchange(nullptr));
    parsed.startup_sdos = startup;
    held.clear();
    std::printf("PASS: %s\n", name);
    std::fflush(stdout);
  };
  auto none = [](auto&) {};
  auto each = [](const auto& list, auto value) {
    Prior prior;
    for (size_t d = 0; d < list.size(); ++d) prior[d] = value(d, list[d]);
    return prior;
  };
  auto configured = [&](const auto& list) { return each(list, [](size_t, const auto& d) { return d.values[0]; }); };
  auto as_queued = [](const StartupSdoConfig& d, int64_t value) { return Queued{0, d.index, d.subindex, value}; };
  const auto profile_startup = parsed.startup_sdos;
  assert(profile_startup.size() >= 2);
  std::vector<Queued> all;
  for (const auto& d : profile_startup)
    if (!d.effective_after_power_cycle) all.push_back(as_queued(d, d.values[0]));
  // A drive whose objects cannot be read (a fresh simulated slave) is written
  // in order, except power-cycle-effective keys, which must already match.
  run_startup("TestStartupUnreadWritesAll", none, [&](const auto& list) {
    Prior prior;
    for (size_t d = 0; d < list.size(); ++d)
      if (list[d].effective_after_power_cycle) prior[d] = list[d].values[0];
    return prior;
  }, all, true);
  run_startup("TestStartupIdenticalWritesNothing", none, configured, {}, true);
  run_startup("TestStartupDifferentWritesOnlyThatObject", none, [&](const auto& list) {
    return each(list, [](size_t i, const auto& d) { return i == 1 ? d.values[0] + 1 : d.values[0]; });
  }, {as_queued(profile_startup[1], profile_startup[1].values[0])}, true);
  // Storage policy, as rtctl compiles it for a catalog storage object at
  // 0x2013:11 (store 1, RAM-only 0) and one trial at 0x2000:07.
  StartupSdoConfig storage{}, trial{};
  storage.axis_mask = trial.axis_mask = 1;
  storage.valid = trial.valid = true;
  storage.type = trial.type = StartupSdoValueType::kU16;
  storage.key = "test_parameter_storage"; storage.index = 0x2013; storage.subindex = 0x11;
  storage.values[0] = 1; storage.storage_mode = true; storage.storage_off = 0;
  trial.key = "test_inertia_ratio"; trial.index = 0x2000; trial.subindex = 0x07;
  trial.values[0] = 800; trial.ram_only = true;
  auto with_policy = [&](auto& list) { list.insert(list.begin(), storage); list.push_back(trial); };
  auto policy_prior = [&](int64_t storage_held, int64_t trial_held, bool stored_differs) {
    return [&, storage_held, trial_held, stored_differs](const auto& list) {
      return each(list, [&](size_t i, const auto& d) {
        if (d.storage_mode) return storage_held;
        if (d.ram_only) return trial_held;
        return i == 2 && stored_differs ? d.values[0] + 1 : d.values[0];
      });
    };
  };
  run_startup("TestStartupHeldTrialWritesNothing", with_policy, policy_prior(1, 800, false), {}, true);
  run_startup("TestStartupTrialRamOnlyWindow", with_policy, policy_prior(1, 1143, false),
      {as_queued(storage, 0), as_queued(trial, 800), as_queued(storage, 1)}, true);
  // Storage found off: the store value goes first so the stored write after it is stored.
  run_startup("TestStartupStorageRestoredFirst", with_policy, policy_prior(0, 1143, true),
      {as_queued(storage, 1), as_queued(profile_startup[1], profile_startup[1].values[0]),
       as_queued(storage, 0), as_queued(trial, 800), as_queued(storage, 1)}, true);
  run_startup("TestStartupTrialWithoutStorageRefused", [&](auto& list) { list.push_back(trial); },
      [&](const auto& list) { return each(list, [](size_t, const auto& d) { return d.ram_only ? 1143 : d.values[0]; }); },
      {}, false);
  // Unverified injected scan observations: the matching second response arrives
  // at 2000000000 ns, beyond the absolute 1100000000 ns deadline.
  sim_rescan_master.reset(); fault = Fault::None; change_assignment = false; g_stop.store(false);
  std::atomic<ec_master_t*> scan_master{ecrt_request_master(0)};
  for (unsigned i = 0; i < 9; ++i) recorded_slave_config(scan_master, 0, i, 4194304, 1813);
  options.preop_to_safeop_timeout_ms = 100;
  assert(!rescan_pdo_cache(scan_master, options, parsed.axis_profiles, +[]() -> uint64_t {
    static unsigned observations = 0;
    return ++observations < 3 ? 1000000000 : 2000000000;
  }));
  assert(!scan_master && sim_rescan_master->reservations == 1);
  std::puts("PASS: TestPREOPRescanLateCompletionRefused");
  // Unverified synthetic six-axis/nine-device topology: a rescan must retain
  // three unconfigured PREOP devices while proving both PDO directions of axes.
  sim_rescan_master.reset(); fault = Fault::None; change_assignment = false; g_stop.store(false);
  options.num_axes = 6; options.bus.hold = true;
  scan_master.store(ecrt_request_master(0));
  for (unsigned i = 0; i < 6; ++i) recorded_slave_config(scan_master, 0, i, 4194304, 1813);
  for (uint16_t i = 6; i < 9; ++i) scan_master.load()->faults.extra_slaves.push_back({i,4194304,1813,12024,true});
  assert(rescan_pdo_cache(scan_master, options, parsed.axis_profiles, +[]() -> uint64_t { return 1000000000; }, 9));
  assert(scan_master.load()->reservations == 2 && scan_master.load()->rescan_requests == 1);
  assert(scan_master.load()->slaves.size() == 6 && scan_master.load()->faults.extra_slaves.size() == 3);
  for (uint16_t i = 6; i < 9; ++i) {
    ec_slave_info_t info{};
    assert(ecrt_master_get_slave(scan_master, i, &info) == 0 && info.al_state == 2);
  }
  ecrt_release_master(scan_master.exchange(nullptr));
  std::puts("PASS: TestPREOPRescanSixOfNineHold");
}
