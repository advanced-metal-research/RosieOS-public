#pragma once

// Included after the mapping comparator, inside the daemon namespace. IgH
// 1.6.9 master/master.c requests PREOP on reservation and deactivation. Never
// issue an assignment download merely because that request was submitted.
inline bool pdo_in_preop(ec_master_t* master, uint16_t position) {
  ec_slave_info_t info{};
  return master && ecrt_master_get_slave(master, position, &info) == 0 && info.al_state == 2;
}

// Reuse the configured PREOP transition budget; polling only waits for observed
// state, and never grants permission on expiry or on an AL error state.
inline bool wait_pdo_preop(ec_master_t* master, uint16_t position, uint32_t budget_ms,
                           bool shutting_down = false) {
  const uint64_t deadline = now_monotonic_ns() + uint64_t(budget_ms) * 1000000;
  do {
    if (!shutting_down && g_stop.load()) return false;
    if (pdo_in_preop(master, position)) return true;
    if (now_monotonic_ns() >= deadline) return false;
    // Unverified state observation interval: 1000000 ns, not a success condition.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (true);
}

inline bool read_preop_pdo(ec_master_t* master, uint16_t position, uint16_t index,
                           uint8_t subindex, unsigned bits, uint32_t& value) {
  if (!pdo_in_preop(master, position)) return false;
  const auto reply = upload_sdo_read(master, {0, position, index, subindex, 0});
  if (reply.error || reply.abort || reply.size != bits / 8) return false;
  value = 0;
  for (size_t i = 0; i < reply.size; ++i) value |= uint32_t(reply.bytes[i]) << (8 * i);
  return pdo_in_preop(master, position);
}

// Write only the assignment object in PREOP. A rejected step stops the sequence;
// callers must verify the entire mapping before registering any process data.
inline bool write_preop_assignment(ec_master_t* master, uint32_t axis, uint16_t position,
                                   uint16_t index, uint16_t preset) {
  const std::array<uint8_t, 2> selection{{uint8_t(preset), uint8_t(preset >> 8)}};
  const uint8_t empty = 0, count = 1;
  auto write = [&](uint8_t sub, const uint8_t* data, size_t size) {
    uint32_t abort = 0;
    const int rc = pdo_in_preop(master, position) ?
        ecrt_master_sdo_download(master, position, index, sub, data, size, &abort) : -EAGAIN;
    logf("EtherCAT pdo_assignment axis=%u phase=PREOP index=0x%04x sub=0x%02x value=0x%04x rc=%d abort=0x%08x",
         axis, index, sub, size == 1 ? data[0] : preset, rc, abort);
    return rc == 0 && abort == 0;
  };
  return write(0, &empty, 1) && write(1, selection.data(), selection.size()) && write(0, &count, 1);
}

struct OriginalPdoAssignment {
  uint16_t preset = 0;
  bool saved = false, changed = false, write_complete = false;
};

struct OriginalPdoAssignments {
  OriginalPdoAssignment tx, rx;
};

// Capture exactly one original PDO per direction before changing it. Unknown/multiple
// assignments fail closed rather than losing entries during optional restore.
inline MappingReadback prepare_preop_mapping(ec_master_t* master, uint32_t axis,
    uint16_t position, const DrivePdoConfig& pdo, OriginalPdoAssignments& originals,
    MappingDifference& difference, bool own_assignment) {
  auto read = [&](uint16_t index, uint8_t sub, unsigned bits, uint32_t& value) {
    return read_preop_pdo(master, position, index, sub, bits, value);
  };
  auto prepare = [&](uint8_t sync, uint16_t target, OriginalPdoAssignment& original, bool rx) {
    const uint16_t assignment = 0x1c10 + sync;
    uint32_t count = 0, preset = 0;
    if (own_assignment) {
      if (!original.saved) {
        if (!read(assignment, 0, 8, count) || !read(assignment, 1, 16, preset))
          return MappingReadback::Unavailable;
        if (count != 1) {
          difference = {assignment, 0, 1, count};
          return MappingReadback::Mismatch;
        }
        original.preset = static_cast<uint16_t>(preset);
        original.saved = true;
      }
      if (original.preset != target && !original.write_complete) {
        // Mark before the first write: a partial transaction also needs restore.
        original.changed = true;
        if (!write_preop_assignment(master, axis, position, assignment, target))
          return MappingReadback::Unavailable;
        original.write_complete = true;
        logf("EtherCAT fixed_mapping axis=%u slave_pos=%u %s_pdo=0x%04x assignment=written phase=PREOP readback=required",
             axis, position, static_cast<const char*>(rx ? "rx" : "tx"), target);
      } else if (!original.changed && !rx) {
        logf("EtherCAT fixed_mapping axis=%u slave_pos=%u tx_pdo=0x%04x assignment=unchanged phase=PREOP readback=required",
             axis, position, pdo.tx_pdo);
      }
    }
    return MappingReadback::Matched;
  };
  const auto tx = prepare(pdo.tx_sync_index, pdo.tx_pdo, originals.tx, false);
  if (tx != MappingReadback::Matched) return tx;
  const auto rx = prepare(pdo.rx_sync_index, pdo.rx_pdo, originals.rx, true);
  if (rx != MappingReadback::Matched) return rx;
  return verify_pdo_readback(pdo, read, &difference);
}

// IgH's registration API uses the scanned cache, not mailbox data. Verify that
// second representation too; reserving the master disables rescan until release
// (IgH 1.6.9 master/master.c:ec_master_enter_operation_phase). No public ecrt API
// refreshes that cache. Compare it before registration, including after rescan.
inline MappingReadback verify_scanned_pdo(ec_master_t* master, uint16_t position,
    const DrivePdoConfig& pdo, MappingDifference& difference) {
  return verify_pdo_readback(pdo, [&](uint16_t index, uint8_t sub, unsigned, uint32_t& value) {
    const bool rx = index == 0x1c10 + pdo.rx_sync_index || index == pdo.rx_pdo;
    const uint8_t sync_index = rx ? pdo.rx_sync_index : pdo.tx_sync_index;
    if (index == 0x1c10 + sync_index && sub == 0) {
      ec_sync_info_t sync{};
      if (ecrt_master_get_sync_manager(master, position, sync_index, &sync)) return false;
      value = sync.n_pdos;
    } else if (index == 0x1c10 + sync_index || sub == 0) {
      ec_pdo_info_t scanned{};
      if (ecrt_master_get_pdo(master, position, sync_index, 0, &scanned)) return false;
      value = sub == 0 ? scanned.n_entries : scanned.index;
    } else {
      ec_pdo_entry_info_t entry{};
      if (ecrt_master_get_pdo_entry(master, position, sync_index, 0, sub - 1, &entry)) return false;
      value = (uint32_t(entry.index) << 16) | (uint32_t(entry.subindex) << 8) | entry.bit_length;
    }
    return true;
  }, &difference);
}

// IgH 1.6.9 master/ioctl.h and tool/CommandRescan.cpp: RESCAN queues a scan
// without reserving the master. Check the pinned ioctl ABI before issuing it.
inline bool queue_pdo_rescan(unsigned master_index) {
#if !GRADIENT_ECRT_ENABLED
  return sim_queue_pdo_rescan(master_index);
#else
  const auto path = "/dev/EtherCAT" + std::to_string(master_index);
  const int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) return false;
  struct ModuleInfo { uint32_t ioctl_version_magic, master_count; } info{};
  // IgH 1.6.9 ioctl.h: type 0xa4, MODULE 0x00, RESCAN 0x0a, ABI magic 37.
  const bool ok = ioctl(fd, _IOR(0xa4, 0x00, ModuleInfo), &info) == 0 &&
      info.ioctl_version_magic == 37 && master_index < info.master_count &&
      ioctl(fd, _IO(0xa4, 0x0a), 0) == 0;
  close(fd);
  return ok;
#endif
}

// Release inactive configurations before scanning. A queued ioctl alone never
// permits activation: all cached entries must match the proven physical map,
// then the caller must rebuild and physically re-verify under a new reservation.
template<class Profiles>
bool rescan_pdo_cache(std::atomic<ec_master_t*>& master, const Options& opt,
                      const Profiles& profiles, uint64_t (*clock)() = now_monotonic_ns,
                      uint32_t expected_responding = 0) {
  // The admitted physical topology includes unused devices; cached PDO proof
  // still covers axes only. Count changes during this hand-off fail closed.
  if (!expected_responding) expected_responding = opt.num_axes;
  // The daemon reserves master 0 in BusHandles; keep the hand-off on that master.
  constexpr unsigned master_index = 0;
#if !GRADIENT_ECRT_ENABLED
  master.load()->retain_for_rescan = true;
#endif
  ecrt_release_master(master.exchange(nullptr));
  logf("EtherCAT phase=pdo_rescan state=released master=%u restoration=off", master_index);
  if (!queue_pdo_rescan(master_index)) return false;
  logf("EtherCAT phase=pdo_rescan state=queued master=%u", master_index);
  auto* observer = ecrt_open_master(master_index);
  if (!observer) return false;
  const auto deadline = clock() + uint64_t(opt.preop_to_safeop_timeout_ms) * 1000000;
  bool matched = false;
  uint32_t last_count = UINT32_MAX, last_states = UINT32_MAX;
  int last_busy = -1;
  do {
    ec_master_info_t info{};
    const bool observed = ecrt_master(observer, &info) == 0;
    bool layouts = observed && info.link_up && info.slave_count == expected_responding && !info.scan_busy;
    uint32_t states = 0;
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      ec_slave_info_t slave{};
      if (ecrt_master_get_slave(observer, opt.slave_position[i], &slave)) layouts = false;
      else {
        states |= slave.al_state;
        if (slave.al_state != 2) layouts = false;
      }
      MappingDifference difference{};
      if (layouts && verify_scanned_pdo(observer, opt.slave_position[i], profiles[i].pdo,
                                       difference) != MappingReadback::Matched) layouts = false;
    }
    const auto observed_ns = clock();
    const bool complete = layouts && observed_ns <= deadline && !g_stop.load();
    // The cache can finish between polls without a count/state/busy delta.
    // Emit its terminal proof exactly once, before reacquisition is permitted.
    if (complete || info.slave_count != last_count || states != last_states || info.scan_busy != last_busy) {
      logf("EtherCAT phase=pdo_rescan state=observed slaves=%u/%u al_states=0x%x scan_busy=%u mapping=%s",
           info.slave_count, expected_responding, states, info.scan_busy, complete ? "verified" : "pending");
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        ec_slave_info_t slave{};
        const int rc = ecrt_master_get_slave(observer, opt.slave_position[i], &slave);
        logf("EtherCAT phase=pdo_rescan axis=%u slave_pos=%u al_state=0x%x rc=%d",
             i, opt.slave_position[i], slave.al_state, rc);
      }
      last_count = info.slave_count; last_states = states; last_busy = info.scan_busy;
    }
    if (complete) { matched = true; break; }
    if (g_stop.load() || observed_ns >= deadline) break;
    // Unverified observation cadence: 1000000 ns; only scanned data grants success.
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } while (true);
  ecrt_release_master(observer);
  if (!matched || g_stop.load()) return false;
  master.store(ecrt_request_master(master_index));
  if (!master) return false;
  logf("EtherCAT phase=pdo_rescan state=reacquired master=%u rebuild=required readback=PREOP_required", master_index);
  return true;
}
