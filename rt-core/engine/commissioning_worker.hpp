#pragma once

// Read a bounded, checksummed canonical record off the cyclic thread. Restored
// offsets cannot bypass independent coordinate identity, drive or source checks.
uint32_t MachineState::load_restore_anchor(uint32_t axis, RestoreAnchorEvidence& out, std::string* changed_field) {
  auto mismatch = [&](const std::string& field) {
    if (changed_field) *changed_field = field;
    return uint32_t{2};
  };
  const char* directory = std::getenv("ROSIE_RT_ANCHOR_DIR");
  if (!directory || !*directory) return 1;
  std::ifstream file(std::filesystem::path(directory) / ("axis-" + std::to_string(axis) + ".anchor"),
                     std::ios::binary);
  if (!file) return 1;
  // Unverified storage bound: 16384 bytes, including the canonical field table.
  std::array<char, 16385> bytes{};
  file.read(bytes.data(), bytes.size());
  const auto size = file.gcount();
  if (size <= 0 || size > 16384) return 3;
  const std::string encoded(bytes.data(), static_cast<size_t>(size));
  const auto end = encoded.find('\n');
  if (end == std::string::npos || encoded.size() != end + 10 || encoded.back() != '\n') return 3;
  const auto payload = encoded.substr(0, end + 1);
  // IEEE CRC-32 polynomial and initial/final XOR, for stored-byte corruption.
  uint32_t crc = 0xffffffffu;
  for (unsigned char byte : payload) {
    crc ^= byte;
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
  }
  crc ^= 0xffffffffu;
  char checksum[10];
  std::snprintf(checksum, sizeof(checksum), "%08x\n", crc);
  if (encoded.substr(end + 1) != checksum) return 3;
  if (payload.rfind("rosie-anchor-v1 ", 0) == 0) return mismatch("legacy_configuration_binding");
  if (payload.rfind("rosie-anchor-v2 ", 0) == 0) return mismatch("legacy_pair_binding");
  std::istringstream record(payload);
  std::string magic, digest, incarnation, extra, encoded_fields, encoded_pair;
  uint64_t pair_revision = 0;
  uint32_t recorded_axis = 0, vendor = 0, product = 0, revision = 0, serial = 0, serial_valid = 0, period = 0;
  RestoreAnchorEvidence evidence{};
  if (!(record >> magic >> digest >> recorded_axis >> vendor >> product >> revision >> serial >> serial_valid
        >> evidence.home_epoch >> evidence.counts >> evidence.offset >> evidence.turns >> period
        >> evidence.tolerance_counts >> incarnation >> evidence.captured_host_ns >> encoded_fields
        >> encoded_pair >> pair_revision)) return 3;
  // A v5 record is a removed current-pose zero capture, never an HM35 Home.
  // Startup, RestoreAnchor and verified-delta restore all load through here and
  // refuse it as anchor_source_invalid naming it; the file is left untouched.
  if (magic == "rosie-anchor-v5") {
    if (changed_field) *changed_field = "current_reference_anchor";
    return 3;
  }
  evidence.version = magic == "rosie-anchor-v3" ? 3 : 0;
  if (record >> extra) return 3;
  auto hex = [](const std::string& value, size_t length) {
    return value.size() == length && std::all_of(value.begin(), value.end(), [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
    });
  };
  // Pair authority is adapter-owned; native validates its storage encoding only.
  if (magic != "rosie-anchor-v3" || encoded_pair.empty() || encoded_pair.size() % 2 ||
      !hex(encoded_pair, encoded_pair.size()) || !pair_revision || !hex(digest, 64) || !hex(incarnation, 32) ||
      !evidence.home_epoch || evidence.captured_host_ns <= 0 || serial_valid > 1 ||
      !evidence.tolerance_counts) return 3;
  const auto& profile = axis_profiles[axis];
  namespace identity = rosie::rtcore::identity;
  try {
    const auto fields = identity::unhex_text(encoded_fields);
    (void)identity::parse_fields(fields);
    if (identity::digest(fields) != digest) return 3;
    if (profile.coordinate_identity.empty()) return mismatch("missing_coordinate_identity");
    if (digest != identity::digest(profile.coordinate_identity))
      return mismatch(identity::first_difference(fields, profile.coordinate_identity));
  } catch (const std::exception&) { return 3; }
  // Metrics start before mailbox identity acquisition. Pending observations
  // cannot be compared as zero-valued drive identities or offered for restore.
  if (!(observed_revision_valid_mask.load() & (1u << axis)) ||
      !(observed_serial_attempted_mask.load() & (1u << axis))) return 3;
  if (recorded_axis != axis || vendor != profile.vendor_id ||
      product != profile.product_code ||
      revision != observed_revision[axis].load() ||
      serial_valid != ((observed_serial_valid_mask.load() >> axis) & 1u) ||
      (serial_valid && serial != observed_serial[axis].load())) return mismatch("drive.observed_identity");
  if (period != 0) return mismatch("retired_period_counts");
  if (evidence.tolerance_counts != profile.coordinate_evidence.maximum_anchor_error_counts)
    return mismatch("tolerance_counts");
  if (!profile.coordinate_evidence.enabled) return 3;
  if (!profile.independent_absolute_origin()) return 3;
  out = evidence;
  return 0;
}

// Check immediately before and after each blocking request under the teardown
// mutex. A reply crossing a readiness/epoch change is Unavailable, never data.
bool MachineState::mailbox_admitted(uint32_t axis, uint64_t epoch, uint64_t since, bool master_locked) {
#if GRADIENT_HAVE_ECRT
  std::unique_lock<std::mutex> lock(shared_master_sdo_mutex, std::defer_lock);
  if (!master_locked) lock.lock();
  auto current = [&] {
    const auto now = mailbox_clock();
    return axis < opt.num_axes && shared_master && !g_stop.load() && bus_ready.load() &&
        latest_feedback.startup_ready && latest_feedback.domain_wc_state == EC_WC_COMPLETE &&
        since && now >= since && now - since >= kMailboxSettleNs &&
        mailbox_ready_since_ns.load() == since && mailbox_ready_epoch.load() == epoch &&
        configuration_epoch.load() == epoch;
  };
  if (!current()) return false;
  ec_master_state_t state{};
  const bool have_nonaxis_policy = opt.bus.hold || !opt.bus.unused.empty() || opt.cell_io.on_bus();
  const uint32_t required_devices = opt.num_axes + (opt.cell_io.on_bus() ? 1u : 0u);
  // Non-axis PREOP devices contribute to the master count and AL bitset.
  // The topology publisher owns their admission; mailbox permission follows
  // the configured axes' live OP states below, so unused churn cannot clear
  // their settling evidence or suppress verified configuration readbacks.
  if (ecrt_master_state(shared_master, &state) || !state.link_up ||
      (have_nonaxis_policy ? state.slaves_responding < required_devices || !(state.al_states & 8) :
          state.slaves_responding != opt.num_axes || state.al_states != 8)) {
    mailbox_ready_since_ns.store(0);
    return false;
  }
  for (uint32_t i = 0; i < opt.num_axes; ++i) {
    ec_slave_info_t info{};
    if (ecrt_master_get_slave(shared_master, opt.slave_position[i], &info) || info.al_state != 8) {
      mailbox_ready_since_ns.store(0);
      return false;
    }
  }
  if (opt.cell_io.on_bus()) {
    const auto& io = opt.cell_io;
    ec_slave_info_t info{};
    if (ecrt_master_get_slave(shared_master, io.position, &info) || info.al_state != 8 ||
        info.vendor_id != io.vendor || info.product_code != io.product || info.revision_number != io.revision) {
      mailbox_ready_since_ns.store(0);
      return false;
    }
  }
  return current();
#else
  (void)axis; (void)epoch; (void)since; (void)master_locked;
  return false;
#endif
}

// One mailbox owner retains each job identity until its terminal receipt.
// Cancellation removes RT permission; it never releases a blocked operation.
bool MachineState::write_sdo_scalar_axis(uint32_t axis,
                                   uint16_t index,
                                   uint8_t subindex,
                                   SdoScalarType type,
                                   int32_t value,
                                   uint32_t* abort_out,
                                   const rosie::rtcore::CommissioningIdentity* home_identity) {
#if GRADIENT_HAVE_ECRT
    std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
    // Only bring-up owns PDO assignments/maps. A runtime scalar write must
    // never invalidate the layout already registered in the active domain.
    if ((index >= 0x1600 && index <= 0x1bff) || (index >= 0x1c10 && index <= 0x1c2f))
      return false;
    if (g_stop.load(std::memory_order_relaxed) || !shared_master ||
        (home_identity && (!control_policy.lease_valid_now() ||
          control_policy.generation.load() != home_identity->generation ||
          configuration_epoch.load() != home_identity->configuration_epoch ||
          home_epoch.load() != home_identity->home_epoch || !bus_ready.load()))) {
      return false;
    }
    uint8_t raw[sizeof(int32_t)] = {};
    size_t raw_size = 0;
    switch (type) {
      case SdoScalarType::kU8:
        raw[0] = static_cast<uint8_t>(value & 0xFF);
        raw_size = sizeof(uint8_t);
        break;
      case SdoScalarType::kI8:
        raw[0] = static_cast<uint8_t>(static_cast<int8_t>(value));
        raw_size = sizeof(int8_t);
        break;
      case SdoScalarType::kU16:
        EC_WRITE_U16(raw, static_cast<uint16_t>(value & 0xFFFF));
        raw_size = sizeof(uint16_t);
        break;
      case SdoScalarType::kI16:
        EC_WRITE_S16(raw, static_cast<int16_t>(value));
        raw_size = sizeof(int16_t);
        break;
      case SdoScalarType::kU32:
        EC_WRITE_U32(raw, static_cast<uint32_t>(value));
        raw_size = sizeof(uint32_t);
        break;
      case SdoScalarType::kI32:
        EC_WRITE_S32(raw, value);
        raw_size = sizeof(int32_t);
        break;
      case SdoScalarType::kNone:
      default:
        return false;
    }
    uint32_t abort_code = 0;
    const auto epoch = configuration_epoch.load(), since = mailbox_ready_since_ns.load();
    if (!mailbox_admitted(axis, epoch, since, true)) return false;
    const int rc = [&] {
      rosie::rtcore::MailboxCallScope call(mailbox_calls_in_flight);
      return ecrt_master_sdo_download(shared_master, opt.slave_position[axis], index, subindex,
                                      raw, raw_size, &abort_code);
    }();
    if (abort_out) {
      *abort_out = abort_code;
    }
    return mailbox_admitted(axis, epoch, since, true) && rc == 0 && abort_code == 0;
#else
    (void)axis;
    (void)index;
    (void)subindex;
    (void)type;
    (void)value;
    (void)home_identity;
    if (abort_out) {
      *abort_out = 0;
    }
    return false;
#endif
  }

// Accept only a terminal receipt for this scalar's exact identity and width.
// A6-EC 0x203F:00 is declared UInt32; the mini bench on 2026-09-15 also
// returned two bytes, which supply only the external-code low half.
// A refused, late or oversized upload leaves the caller's value untouched.
// Observe-only tuning reads journal what each drive holds. The worker calls
// this once per readback tick, after the startup readback and only while
// disarmed with no axis enabled, so the reads neither delay config_verified_mask
// nor add mailbox traffic during motion. A read is never a write, never feeds
// verification, and goes through read_sdo_scalar_axis: admitted, identity
// checked and counted in flight, so shutdown drains it like any other upload.
bool MachineState::startup_observe_step() {
  auto& p = startup_observe_progress;
#if !GRADIENT_HAVE_ECRT
  p.complete = true;
#endif
  if (p.complete) return true;
  // Unverified retry bound for transient refusals; an object this firmware
  // lacks aborts every time and is journalled once, then skipped.
  constexpr uint32_t kStartupObserveAttempts = 3;
  while (p.axis < opt.num_axes && p.entry >= axis_profiles[p.axis].startup_observe.size()) {
    ++p.axis;
    p.entry = 0;
  }
  if (p.axis >= opt.num_axes) {
    if (p.begun) logf("EtherCAT startup observe complete reads=%u failed=%u", p.reads, p.failed);
    p.complete = true;
    return true;
  }
  if (!p.begun) {
    size_t entries = 0;
    for (uint32_t i = 0; i < opt.num_axes; ++i) entries += axis_profiles[i].startup_observe.size();
    logf("EtherCAT startup observe begin entries=%zu", entries);
    p.begun = true;
  }
  const auto& o = axis_profiles[p.axis].startup_observe[p.entry];
  sample_absolute_due();
  int32_t value = 0;
  uint32_t abort_code = 0;
  bool attempted = false;
  const bool ok = read_sdo_scalar_axis(p.axis, o.index, o.subindex, sdo_scalar_type(o.type), &value, &abort_code, &attempted);
  if (!attempted) return false;
  if (ok) {
    const int64_t observed = o.type == StartupSdoValueType::kU32 ? int64_t(uint32_t(value)) : int64_t(value);
    logf("EtherCAT startup observe axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x value=%lld",
         p.axis, static_cast<unsigned int>(opt.slave_position[p.axis]), o.key.c_str(),
         static_cast<unsigned int>(o.index), static_cast<unsigned int>(o.subindex), static_cast<long long>(observed));
    p.last_value = observed;
    ++p.reads;
  } else if (++p.attempts < kStartupObserveAttempts) {
    return false;
  } else {
    logf("WARNING: EtherCAT startup observe failed axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x abort=0x%08x attempts=%u",
         p.axis, static_cast<unsigned int>(opt.slave_position[p.axis]), o.key.c_str(),
         static_cast<unsigned int>(o.index), static_cast<unsigned int>(o.subindex), abort_code, p.attempts);
    ++p.failed;
  }
  p.attempts = 0;
  ++p.entry;
  return false;
}

bool MachineState::read_sdo_scalar_axis(uint32_t axis,
                                  uint16_t index,
                                  uint8_t subindex,
                                  SdoScalarType type,
                                  int32_t* value_out,
                                  uint32_t* abort_out,
                                  bool* attempted_out) {
    if (attempted_out) *attempted_out = false;
#if GRADIENT_HAVE_ECRT
    std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
    if (abort_out) *abort_out = 0;
    if (!value_out || axis >= opt.num_axes || g_stop.load(std::memory_order_relaxed) || !shared_master) {
      return false;
    }
    size_t expected_size = 0;
    switch (type) {
      case SdoScalarType::kU8: case SdoScalarType::kI8: expected_size = 1; break;
      case SdoScalarType::kU16: case SdoScalarType::kI16: expected_size = 2; break;
      case SdoScalarType::kU32: case SdoScalarType::kI32: expected_size = 4; break;
      default: return false;
    }
    // Unverified sequence capacity: uint64_t invocations across all instances.
    // Saturate instead of reusing an identity after wrap.
    static std::atomic<uint64_t> sequence{0};
    auto previous = sequence.load(std::memory_order_relaxed);
    do {
      if (previous == std::numeric_limits<uint64_t>::max()) return false;
    } while (!sequence.compare_exchange_weak(previous, previous + 1, std::memory_order_relaxed));
    const SdoReadIdentity identity{axis, opt.slave_position[axis], index, subindex, previous + 1};
    const auto epoch = configuration_epoch.load(), since = mailbox_ready_since_ns.load();
    if (!mailbox_admitted(axis, epoch, since, true)) return false;
    if (attempted_out) *attempted_out = true;
    const auto reply = [&] {
      rosie::rtcore::MailboxCallScope call(mailbox_calls_in_flight);
      return upload_sdo_read(shared_master, identity);
    }();
    const bool current = mailbox_admitted(axis, epoch, since, true) && reply.identity == identity;
    if (abort_out && current) *abort_out = reply.abort;
    const bool alarm_low_half = index == 0x203f && subindex == 0 &&
        type == SdoScalarType::kU32 && reply.size == 2;
    if (!current || reply.error != 0 || reply.abort != 0 ||
        (reply.size != expected_size && !alarm_low_half)) {
      logf("SDO readback unavailable axis=%u slave_pos=%u index=0x%04x sub=0x%02x sequence=%llu "
           "rc=%d abort=0x%08x expected_bytes=%zu observed_bytes=%zu identity_match=%u",
           axis, identity.position, index, subindex, static_cast<unsigned long long>(identity.sequence),
           reply.error, reply.abort, expected_size, reply.size, current ? 1u : 0u);
      return false;
    }
    const auto* raw = reply.bytes.data();
    switch (type) {
      case SdoScalarType::kU8:
        *value_out = static_cast<int32_t>(raw[0]);
        return true;
      case SdoScalarType::kI8:
        *value_out = static_cast<int32_t>(static_cast<int8_t>(raw[0]));
        return true;
      case SdoScalarType::kU16:
        *value_out = static_cast<int32_t>(EC_READ_U16(raw));
        return true;
      case SdoScalarType::kI16:
        *value_out = static_cast<int32_t>(EC_READ_S16(raw));
        return true;
      case SdoScalarType::kU32:
        *value_out = static_cast<int32_t>(alarm_low_half ? EC_READ_U16(raw) : EC_READ_U32(raw));
        return true;
      case SdoScalarType::kI32:
        *value_out = EC_READ_S32(raw);
        return true;
      case SdoScalarType::kNone:
      default:
        return false;
    }
#else
    (void)axis;
    (void)index;
    (void)subindex;
    (void)type;
    (void)value_out;
    if (abort_out) {
      *abort_out = 0;
    }
    return false;
#endif
  }

// One due drive-profile monitor object per checkpoint, round-robin over every
// axis's list, after the alarm read: diagnostic reads take the mailbox only
// when the worker is otherwise idle, and an attempted read waits its full
// interval before the next, success or not (docs/decisions/0001).
void MachineState::refresh_monitor(uint64_t (*clock)()) {
  if (!bus_ready.load()) return;
  const uint32_t slots = opt.num_axes * static_cast<uint32_t>(rosie::rtcore::kMaxMonitorPerAxis);
  for (uint32_t offset = 0; offset < slots; ++offset) {
    const uint32_t n = (monitor_next + offset) % slots;
    const uint32_t axis = n / rosie::rtcore::kMaxMonitorPerAxis, entry = n % rosie::rtcore::kMaxMonitorPerAxis;
    const auto& monitors = axis_profiles[axis].monitor;
    if (entry >= monitors.size()) continue;
    auto& slot = drive_monitors[axis].slots[entry];
    const uint64_t now = clock();
    if (now < slot.next_due_ns) continue;
    const auto& m = monitors[entry];
    int32_t raw = 0;
    uint32_t abort = 0;
    bool attempted = false;
    const bool valid = read_sdo_scalar_axis(axis, m.object.index, m.object.subindex, m.object.type,
        &raw, &abort, &attempted);
    if (!attempted) return;  // mailbox not admitted: try again next checkpoint
    slot.next_due_ns = now + m.interval_ns;
    monitor_next = (n + 1) % slots;
    if (valid) slot.publish(raw, clock());
    return;
  }
}

// Poll independently of application connection and metrics requests. An error
// observed before admission remains pending; an attempted upload never retries.
bool MachineState::alarm_readback_pending() const {
  for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
    const auto& alarm = drive_alarms[axis];
    const auto event = alarm.event.load();
    if (static_cast<uint16_t>(event) && event != alarm.attempted_event &&
        (event >> 16) != (std::numeric_limits<uint64_t>::max() >> 16)) return true;
  }
  return false;
}

// One synchronous upload per checkpoint under the commissioning claim prevents
// concurrent requests and bounds diagnostic work independently of axis count.
void MachineState::refresh_drive_alarm() {
  for (uint32_t offset = 0; offset < opt.num_axes; ++offset) {
    const auto axis = (alarm_next_axis + offset) % opt.num_axes;
    auto& alarm = drive_alarms[axis];
    const auto event = alarm.event.load();
    if (!static_cast<uint16_t>(event) || event == alarm.attempted_event ||
        (event >> 16) == (std::numeric_limits<uint64_t>::max() >> 16)) continue;
    const auto epoch = configuration_epoch.load(), since = mailbox_ready_since_ns.load();
    if (!mailbox_admitted(axis, epoch, since)) return;
    int32_t raw = 0;
    uint32_t abort = 0;
    bool attempted = false;
    const bool valid = read_sdo_scalar_axis(axis, 0x203f, 0, SdoScalarType::kU32,
        &raw, &abort, &attempted);
    if (!attempted) return;
    alarm.attempted_event = event;
    alarm_next_axis = (axis + 1) % opt.num_axes;
    if (valid && alarm.event.load() == event && mailbox_admitted(axis, epoch, since))
      alarm.publish({event, epoch, static_cast<uint32_t>(raw), true});
    return;
  }
}

// The commissioning thread is the only caller. Its master mutex also excludes
// master teardown; cyclic code only consumes the resulting immutable snapshot.
MappingReadback MachineState::verify_axis_mapping(uint32_t axis) {
#if GRADIENT_HAVE_ECRT
  if (axis >= opt.num_axes) return MappingReadback::Unavailable;
  const auto epoch = configuration_epoch.load();
  const auto since = mailbox_ready_since_ns.load();
  const auto verification = verification_request.load();
  MappingDifference difference;
  auto result = verify_pdo_readback(axis_profiles[axis].pdo,
      [&](uint16_t index, uint8_t subindex, unsigned bits, uint32_t& value) {
        int32_t raw = 0;
        uint32_t abort = 0;
        const auto type = bits == 8 ? SdoScalarType::kU8 : bits == 16 ? SdoScalarType::kU16 : SdoScalarType::kU32;
        const bool ok = read_sdo_scalar_axis(axis, index, subindex, type, &raw, &abort);
        if (!ok) logf("PDO readback refused axis=%u index=0x%04x sub=0x%02x abort=0x%08x", axis, index, subindex, abort);
        value = static_cast<uint32_t>(raw);
        return ok;
      }, &difference);
  if (!mailbox_admitted(axis, epoch, since) || verification_request.load() != verification || g_stop.load())
    result = MappingReadback::Unavailable;
  if (result == MappingReadback::Mismatch) {
    logf("EtherCAT mapping_readback axis=%u slave_pos=%u result=pdo_mapping_mismatch reason=%u "
         "index=0x%04x sub=0x%02x expected=0x%08x observed=0x%08x",
         axis, opt.slave_position[axis], rosie::protocol::reason_pdo_mapping_mismatch,
         difference.index, difference.subindex, difference.expected, difference.observed);
  } else logf("EtherCAT mapping_readback axis=%u slave_pos=%u result=%s reason=%u", axis,
      opt.slave_position[axis], result == MappingReadback::Matched ? "verified" : result == MappingReadback::Unavailable ? "unavailable_retry" : "pdo_mapping_mismatch",
      result == MappingReadback::Mismatch ? rosie::protocol::reason_pdo_mapping_mismatch : rosie::protocol::reason_none);
  return result;
#else
  (void)axis;
  return MappingReadback::Unavailable;
#endif
}

// The profile selects the admitted continuous per-axis source. Bracket its low
// word with high/low/high to reject torn acquisitions.
// The cyclic PDO history checks acquisition skew and continuous displacement;
// raw :21/:23 use a fixed origin established only by Home or matching Restore.
// Neither retention nor Home origin is assumed from an ordinary acquisition.
bool MachineState::sample_absolute_axis(uint32_t axis, uint64_t (*clock)()) {
  const auto configuration = configuration_epoch.load();
  const auto since = mailbox_ready_since_ns.load();
  if (!mailbox_admitted(axis, configuration, since)) return false;
  const auto frame = absolute_frame_epoch[axis].load();
  const auto source = axis_profiles[axis].absolute_anchor_source();
  rosie::rtcore::SampledAbsolutePosition sample{};
  sample.audit_generation = audit_generation.load();
  // Keep the gated epoch through completion: a concurrent reset cannot stamp
  // an upload with a new configuration that has not passed startup readback.
  sample.configuration_epoch = configuration;
  sample.home_epoch = frame;
  sample.independent_origin = axis_profiles[axis].independent_absolute_origin();
  sample.cycle = rt_cycle_counter.load();
  sample.started_ns = clock();
  sample.start_gap_ns = absolute_last_poll_ns[axis] && sample.started_ns >= absolute_last_poll_ns[axis]
      ? sample.started_ns - absolute_last_poll_ns[axis] : 0;
  if (!absolute_first_started_ns[axis]) absolute_first_started_ns[axis] = sample.started_ns;
  sample.first_started_ns = absolute_first_started_ns[axis];
  sample.samples = ++absolute_samples[axis];
  int32_t high = 0, low = 0, high_before = 0;
  uint32_t abort = 0;
  const bool bracket_ok = bus_ready.load() && (!source.high_subindex || read_sdo_scalar_axis(axis, source.index, source.high_subindex,
      SdoScalarType::kI32, &high_before, &abort));
  sample.low_cycle = rt_cycle_counter.load();
  sample.bracket_completed_ns = source.high_subindex ? clock() : sample.started_ns;
  const bool low_ok = bracket_ok && read_sdo_scalar_axis(axis, source.index, source.low_subindex,
      SdoScalarType::kI32, &low, &abort);
  sample.low_completed_ns = clock();
  const bool ok = low_ok && (!source.high_subindex || read_sdo_scalar_axis(axis, source.index, source.high_subindex,
      SdoScalarType::kI32, &high, &abort));
  sample.completed_ns = source.high_subindex ? clock() : sample.low_completed_ns;
  // Multiplication avoids signed left-shift UB; unsigned low preserves -1/-1.
  sample.encoder_counts = source.high_subindex
      ? int64_t(high) * (int64_t{1} << 32) + static_cast<uint32_t>(low) : low;
  using Reason = rosie::rtcore::SampleInvalidity;
  sample.reason = uint32_t(!ok || !mailbox_admitted(axis, configuration, since) ? Reason::Unavailable :
      sample.audit_generation != audit_generation.load() ? Reason::Epoch :
      configuration_epoch.load() != sample.configuration_epoch || absolute_frame_epoch[axis].load() != sample.home_epoch
          ? Reason::Epoch : high_before != high ? Reason::Torn : Reason::None);
  const auto interval = axis_profiles[axis].coordinate_evidence.sample_interval_ns;
  rosie::rtcore::finish_sampled_acquisition(sample, interval);
  sampled_absolute[axis].publish(sample);
  absolute_last_poll_ns[axis] = sample.started_ns;
  if (metrics_absolute_feedback_poll_enabled) {
    auto& feedback = latest_feedback.absolute_feedback[axis];
    const auto& fields = axis_profiles[axis].absolute;
    for (uint32_t i = 0; i < fields.field_count; ++i) {
      const auto& object = fields.fields[i].object;
      if (object.index == source.index && (object.subindex == source.low_subindex ||
          (source.high_subindex && object.subindex == source.high_subindex))) {
        feedback.fields[i].value = object.subindex == source.low_subindex ? low : high;
        feedback.fields[i].valid = sample.valid;
      }
    }
    feedback.paired_pos_counts = 0;
    // An independent raw origin is not the command PDO phase. Expose its raw
    // fields, but never label raw modulo counts as a paired command position.
    feedback.paired_valid = sample.valid && !sample.independent_origin;
    feedback.paired_sample_time_ns = sample.started_ns;
  }
  return sample.valid;
}

// At most one axis per checkpoint: Home polling cannot wait behind nine
// acquisitions. A new acquisition retains its real start; a missed deadline
// never republishes old counts with a renewed timestamp.
void MachineState::sample_absolute_due() {
  if (!bus_ready.load()) return;
  for (uint32_t n = 0; n < opt.num_axes; ++n) {
    const uint32_t axis = absolute_next_axis++ % opt.num_axes;
    const auto& config = axis_profiles[axis].coordinate_evidence;
    if (axis_profiles[axis].independent_absolute_origin() &&
        (armed.load() || axis_enable_mask.load() || !(absolute_stationary_mask.load() & (1u << axis)))) continue;
    const auto now = now_monotonic_ns();
    if (!config.sdo_sampled || (now >= absolute_last_poll_ns[axis] &&
        now - absolute_last_poll_ns[axis] < config.sample_interval_ns)) continue;
    const auto samples = absolute_samples[axis];
    (void)sample_absolute_axis(axis);
    // A gate issued no upload: keep looking without spending this checkpoint.
    // An attempted acquisition consumes the slot even when its upload fails.
    if (absolute_samples[axis] != samples) return;
  }
}

bool MachineState::read_native_home_truth_axis(uint32_t axis, int32_t* truth_out) {
    const auto& native_home_cfg = axis_profiles[axis].home;
#if GRADIENT_HAVE_ECRT
    if (!truth_out || !native_home_cfg.valid || !native_home_cfg.truth_source.valid) {
      return false;
    }
    uint32_t abort_code = 0;
    int32_t raw_truth = 0;
    if (!read_sdo_scalar_axis(axis,
                              native_home_cfg.truth_source.index,
                              native_home_cfg.truth_source.subindex,
                              native_home_cfg.truth_source.type,
                              &raw_truth,
                              &abort_code)) {
      logf("WARNING: native_home truth upload failed axis=%u slave_pos=%u index=0x%04x sub=0x%02x abort=0x%08x",
           axis,
           static_cast<unsigned int>(opt.slave_position[axis]),
           static_cast<unsigned int>(native_home_cfg.truth_source.index),
           static_cast<unsigned int>(native_home_cfg.truth_source.subindex),
           static_cast<unsigned int>(abort_code));
      return false;
    }
    // The profile defines the sign from its declared truth value to the host's
    // additive logical offset. Interpret U32 before sign/projection, not after.
    return rosie::rtcore::home_offset_from_scalar_bits(raw_truth,
        native_home_cfg.truth_source.type == SdoScalarType::kU32,
        axis_profiles[axis].home_truth_sign, *truth_out);
#else
    (void)axis;
    (void)truth_out;
    return false;
#endif
  }

bool MachineState::read_absolute_feedback_field_axis(uint32_t axis,
                                               uint16_t index,
                                               uint8_t subindex,
                                               SdoScalarType type,
                                               AbsoluteFeedbackFieldSample* out) {
    if (!out) {
      return false;
    }
#if GRADIENT_HAVE_ECRT
    int32_t value = 0;
    uint32_t abort_code = 0;
    if (!read_sdo_scalar_axis(axis, index, subindex, type, &value, &abort_code)) {
      out->valid = 0u;
      out->value = 0;
      return false;
    }
    out->valid = 1u;
    out->value = value;
    return true;
#else
    (void)axis;
    (void)index;
    (void)subindex;
    (void)type;
    out->valid = 0u;
    out->value = 0;
    return false;
#endif
  }

// Name the configured operation in failure diagnostics; numeric enum values
// alone cannot identify which drive transaction failed.
inline const char* native_home_op_name(NativeHomeOpKind op) noexcept {
  switch (op) {
    case NativeHomeOpKind::kSetMode: return "set_mode";
    case NativeHomeOpKind::kWriteSdo: return "write_sdo";
    case NativeHomeOpKind::kControlwordSequence: return "controlword_sequence";
    case NativeHomeOpKind::kWaitStatusword: return "wait_statusword";
    case NativeHomeOpKind::kRefreshTruth: return "refresh_truth";
    case NativeHomeOpKind::kRestoreMode: return "restore_mode";
    case NativeHomeOpKind::kWaitSdo: return "wait_sdo";
    case NativeHomeOpKind::kReleaseServiceOverride: return "release_service_override";
    default: return "invalid";
  }
}

// Entry gate for fresh Home acquisition on brake-equipped independent-source
// axes. Disabling torque and finishing the drive-managed hold delay must precede
// the sample, rather than occur inside its bracket. This wait never establishes
// Home: the native acquisition/displacement checks still decide CompleteHome.
// The 100 ms quiet observation and 500 ms cap are unqualified commissioning
// bounds; a failed wait refuses Home rather than enlarging any capture tolerance.
// Under review: added on a Rosie 1430 arm's branch (9b5e17bc), not present on
// origin/main, and possibly unnecessary. Kept by owner decision until reviewed.
template<class Clock, class Pause>
bool wait_native_home_capture_settle(MachineState& machine, uint32_t axis,
    const rosie::rtcore::CommissioningIdentity& identity, uint64_t deadline,
    Clock&& clock, Pause&& pause) {
  using namespace gradient::ipc::v1;
  constexpr uint64_t quiet_ns = 100000000;
  const uint64_t started = clock();
  uint64_t previous_seq = 0, previous_observed = 0, quiet_since = 0;
  int64_t low = 0, high = 0;
  const auto frame = machine.absolute_frame_epoch[axis].load();
  const int64_t tolerance = std::min<uint32_t>(
      machine.axis_profiles[axis].brake.hold_displacement_tolerance_counts,
      rosie::rtcore::SampledPdoHistory::maximum_home_acquisition_displacement_counts);
  while (clock() < deadline) {
    if (!machine.commissioning_running.load() || g_stop.load() ||
        !machine.control_policy.lease_valid_now() || !machine.bus_ready.load() ||
        machine.safety_fault_mask.load() || machine.armed.load() || machine.axis_enable_mask.load() ||
        machine.service_mode_axis_mask.load() ||
        machine.control_policy.generation.load() != identity.generation ||
        machine.configuration_epoch.load() != identity.configuration_epoch ||
        machine.home_epoch.load() != identity.home_epoch ||
        machine.absolute_frame_epoch[axis].load() != frame) return false;
    const uint64_t before = machine.latest_feedback.seq.load(std::memory_order_acquire);
    const uint64_t observed = machine.latest_feedback.pdo_observed_time_ns[axis];
    const int64_t position = machine.latest_feedback.pos_counts[axis];
    const auto state = gradient::ds402::decode_statusword(machine.latest_feedback.statusword[axis]);
    const bool inactive = state == gradient::ds402::State::SwitchOnDisabled ||
        state == gradient::ds402::State::ReadyToSwitchOn || state == gradient::ds402::State::SwitchedOn;
    const bool applied = machine.latest_feedback.brake_state[axis] == BRAKE_APPLIED;
    const bool fresh = (machine.latest_feedback.pdo_fresh_axis_mask & (1u << axis)) != 0;
    const auto after = machine.latest_feedback.seq.load(std::memory_order_acquire);
    const uint64_t now = clock();
    // A writer in progress is not a physical-motion observation. Skip it;
    // the next coherent read must still be new and from this wait. This is
    // an acquisition-entry delay, not proof of every intervening PDO cycle;
    // CompleteHome retains the authoritative full-interval history check.
    // Frozen/old publications never advance the quiet interval.
    if (!before || before != after || (before & 1u)) { pause(); continue; }
    if (observed >= started && observed <= now && fresh && inactive && applied) {
      if (before == previous_seq || observed <= previous_observed) { pause(); continue; }
      previous_seq = before;
      previous_observed = observed;
      low = quiet_since ? std::min(low, position) : position;
      high = quiet_since ? std::max(high, position) : position;
      if (!quiet_since || high - low > tolerance) {
        quiet_since = observed; low = high = position;
      } else if (observed - quiet_since >= quiet_ns) {
        return true;
      }
    } else {
      // Repeated, torn or stale feedback cannot age into a quiet observation.
      quiet_since = 0;
    }
    pause();
  }
  return false;
}

// Execute one drive transaction with a correlated cyclic request callback.
// First-failure publication survives cleanup and never grants output permission.
template<class Request>
bool MachineState::native_home_axis(uint32_t axis,
    const rosie::rtcore::CommissioningIdentity& home_identity, Request&& commissioning_request) {
  auto home_permitted = [&]() {
    return commissioning_running.load() && !g_stop.load() && control_policy.lease_valid_now() &&
        control_policy.generation.load() == home_identity.generation &&
        configuration_epoch.load() == home_identity.configuration_epoch &&
        home_epoch.load() == home_identity.home_epoch && bus_ready.load();
  };
  const auto& native_home_cfg = axis_profiles[axis].home;
  using namespace gradient::ipc::v1;
  uint32_t home_abort_code = 0;
  size_t step_index = 0;
  const char* step_op = "begin";
  rosie::rtcore::CommissioningReason request_reason = rosie::rtcore::CommissioningReason::None;
  // Publish and log the first failure once. Cleanup cannot overwrite its cause.
  auto fail_home = [&](uint32_t code, const char* reason) {
    if (home_abort_code) return;
    rosie::rtcore::CommissioningIdentity deadline_identity{};
    auto cancellation = request_reason;
    if (commissioning_deadline_identity.read(deadline_identity) &&
        rosie::rtcore::same_commissioning_identity(deadline_identity, home_identity)) {
      cancellation = rosie::rtcore::CommissioningReason::CycleDeadline;
      reason = rosie::rtcore::commissioning_reason_name(cancellation);
    }
    home_abort_code = code;
    NativeHomeFailure failure{home_identity, axis, static_cast<uint32_t>(step_index), code, cancellation};
    std::snprintf(failure.operation, sizeof(failure.operation), "%s", step_op);
    std::snprintf(failure.reason, sizeof(failure.reason), "%s", reason);
    native_home_failures[axis].publish(failure);
    logf("WARNING: native_home failed axis=%u step=%zu op=%s reason=%s abort=0x%08x",
         axis, step_index, step_op, reason, code);
    (void)commissioning_request({0, home_identity, rosie::rtcore::CommissioningOperation::FailHome,
                                1u << axis, 0, 0, 0, home_abort_code});
  };
  auto sample_failure = [&](uint32_t code) {
    rosie::rtcore::SampledAbsolutePosition sample{};
    using Invalidity = rosie::rtcore::SampleInvalidity;
    const auto reason = !sampled_absolute[axis].read(sample) ? uint32_t(Invalidity::Missing) :
        sample.configuration_epoch != configuration_epoch.load() || sample.home_epoch != absolute_frame_epoch[axis].load()
            ? uint32_t(Invalidity::Epoch) : sample.reason ? sample.reason : uint32_t(Invalidity::Unavailable);
    fail_home(code, rosie::rtcore::sampled_invalidity_name(reason));
  };
  if (!home_permitted()) {
    fail_home(NATIVE_HOME_ABORT_PERMISSION_LOST, "permission lost");
    return false;
  }

  auto read_stable_axis_feedback_counts = [&](uint32_t axis, int32_t* pos_counts_out) {
    if (!pos_counts_out || axis >= opt.num_axes) {
      return false;
    }
    for (int attempt = 0; attempt < 20; ++attempt) {
      const uint64_t seq_before = latest_feedback.seq.load(std::memory_order_acquire);
      const int32_t pos_counts = latest_feedback.pos_counts[axis];
      const uint64_t seq_after = latest_feedback.seq.load(std::memory_order_acquire);
      if (seq_before != 0 && seq_before == seq_after && !(seq_before & 1u)) {
        *pos_counts_out = pos_counts;
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  };
  int32_t pos_counts = latest_feedback.pos_counts[axis];
  (void)read_stable_axis_feedback_counts(axis, &pos_counts);

#if GRADIENT_HAVE_ECRT
  if (!native_home_cfg.valid || !native_home_cfg.truth_source.valid ||
      native_home_cfg.transaction.empty()) {
    fail_home(NATIVE_HOME_ABORT_DESCRIPTOR_INVALID, "native-home descriptor invalid");
    return false;
  }
  if (!shared_master) {
    fail_home(NATIVE_HOME_ABORT_MASTER_UNAVAILABLE, "master unavailable");
    return false;
  }
  // Unverified service settling interval retained from the native Home transaction.
  constexpr auto kServiceStepSleep = std::chrono::milliseconds(50);
  // Unverified device status timeout retained from the native Home transaction.
  constexpr uint64_t kWaitStatuswordTimeoutNs = 10000000000ULL;
  // Unverified mailbox polling timeout retained from the native Home transaction.
  constexpr uint64_t kWaitSdoTimeoutNs = 5000000000ULL;
  const uint32_t axis_bit = (1u << axis);
  int32_t requested_mode = native_home_cfg.commissioning_mode;
  uint32_t requested_controlword = 0;
  auto publish_service = [&]() {
    if (!home_permitted()) {
      fail_home(NATIVE_HOME_ABORT_PERMISSION_LOST, "permission lost");
      return false;
    }
    if (!commissioning_request({0, home_identity,
        rosie::rtcore::CommissioningOperation::Service, axis_bit, requested_mode, requested_controlword},
        nullptr, nullptr, &request_reason)) {
      fail_home(NATIVE_HOME_ABORT_SERVICE_REFUSED, rosie::rtcore::commissioning_reason_name(request_reason));
      return false;
    }
    return true;
  };
  uint64_t wire_audit_token = 0;
  bool wire_audit_ended = false;
  const auto end_wire_audit = [&]() {
    if (wire_audit_token && !wire_audit_ended) {
      native_home_wire_audit[axis].end(wire_audit_token, now_monotonic_ns());
      wire_audit_ended = true;
    }
  };
  const auto report_wire_audit = [&]() {
    if (!wire_audit_token) return;
    rosie::rtcore::NativeHomeWireSummary summary{};
    bool available = false;
    // Called only after service release or FailHome cancellation: this
    // diagnostic handshake cannot delay the stop request or grant output.
    for (unsigned attempt = 0; attempt < 20; ++attempt) {
      if (native_home_wire_audit[axis].read_stopped(wire_audit_token, summary)) { available = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto u = [](uint64_t value) { return static_cast<unsigned long long>(value); };
    logf("Native-home wire audit axis=%u available=%u token=%llu requested_ns=%llu ended_ns=%llu cycles=%llu submitted=%llu failures=%llu prefix_submitted=%llu triggered=%llu violations=%llu missing_cycles=%llu nonmonotonic=%llu time_gaps=%llu max_gap_ns=%llu stop_cycle=%llu abort=0x%08x",
         axis, unsigned(available), u(wire_audit_token), u(summary.requested_ns), u(summary.ended_ns),
         u(summary.cycles), u(summary.submitted), u(summary.submission_failures), u(summary.prefix_cycles),
         u(summary.triggered_submissions), u(summary.violations), u(summary.missing_cycles),
         u(summary.nonmonotonic_cycles), u(summary.time_gaps), u(summary.maximum_gap_ns), u(summary.stopped_cycle), home_abort_code);
    logf("Native-home wire samples axis=%u token=%llu first_cycle=%llu first_ns=%llu first_cw=0x%04x first_mode=%d first_sw=0x%04x first_target=%d first_pos=%d last_cycle=%llu last_ns=%llu last_cw=0x%04x last_mode=%d last_sw=0x%04x last_target=%d last_pos=%d abort=0x%08x",
         axis, u(wire_audit_token), u(summary.first.cycle), u(summary.first.time_ns), unsigned(summary.first.controlword), int(summary.first.mode),
         unsigned(summary.first.statusword), summary.first.target, summary.first.position,
         u(summary.last.cycle), u(summary.last.time_ns), unsigned(summary.last.controlword), int(summary.last.mode),
         unsigned(summary.last.statusword), summary.last.target, summary.last.position, home_abort_code);
    if (available && summary.prefix.cycles) {
      const auto& prefix = summary.prefix;
      logf("Native-home wire prefix axis=%u token=%llu cycles=%llu submitted=%llu failures=%llu missing_cycles=%llu nonmonotonic=%llu time_gaps=%llu max_gap_ns=%llu abort=0x%08x",
           axis, u(wire_audit_token), u(prefix.cycles), u(prefix.submitted), u(prefix.submission_failures),
           u(prefix.missing_cycles), u(prefix.nonmonotonic_cycles), u(prefix.time_gaps), u(prefix.maximum_gap_ns), home_abort_code);
      constexpr const char* names[] = {"cw6", "cw7", "cw15", "other"};
      for (unsigned bucket_index = 0; bucket_index < 4; ++bucket_index) {
        const auto& bucket = prefix.buckets[bucket_index];
        logf("Native-home wire prefix bucket axis=%u token=%llu bucket=%s submitted=%llu mode6=%llu mode8=%llu other_mode=%llu first_cycle=%llu first_ns=%llu first_cw=0x%04x first_mode=%d first_sw=0x%04x first_target=%d first_pos=%d last_cycle=%llu last_ns=%llu last_cw=0x%04x last_mode=%d last_sw=0x%04x last_target=%d last_pos=%d",
             axis, u(wire_audit_token), names[bucket_index], u(bucket.submitted), u(bucket.mode6), u(bucket.mode8), u(bucket.other_mode),
             u(bucket.first.cycle), u(bucket.first.time_ns), unsigned(bucket.first.controlword), int(bucket.first.mode),
             unsigned(bucket.first.statusword), bucket.first.target, bucket.first.position,
             u(bucket.last.cycle), u(bucket.last.time_ns), unsigned(bucket.last.controlword), int(bucket.last.mode),
             unsigned(bucket.last.statusword), bucket.last.target, bucket.last.position);
      }
    }
    if (available && summary.violations)
      logf("Native-home wire audit first violation axis=%u token=%llu cycle=%llu cw=0x%04x mode=%d sw=0x%04x target=%d pos=%d",
           axis, u(wire_audit_token), u(summary.first_violation.cycle), unsigned(summary.first_violation.controlword),
           int(summary.first_violation.mode), unsigned(summary.first_violation.statusword),
           summary.first_violation.target, summary.first_violation.position);
  };
  // The A6-EC HM35 transaction is executed exactly as its descriptor declares
  // (config/drives/a6ec.json, vendor-confirmed). The wire audit only records
  // submitted frames; it never delays, inserts or alters a transaction step.
  const bool hm35 = std::any_of(native_home_cfg.transaction.begin(), native_home_cfg.transaction.end(),
      [](const NativeHomeOp& step) {
        return step.kind == NativeHomeOpKind::kWriteSdo && step.object.valid &&
            step.object.index == 0x6098 && step.object.subindex == 0 && step.value_i32 == 35;
      });
  if (hm35) wire_audit_token = native_home_wire_audit[axis].begin(now_monotonic_ns(), true);
  if (!publish_service()) { end_wire_audit(); report_wire_audit(); return false; }
  std::this_thread::sleep_for(kServiceStepSleep);

  auto release_service_override = [&](bool restore_steady_mode) {
    if (restore_steady_mode && home_permitted()) {
      requested_controlword = 0;
      requested_mode = native_home_cfg.steady_state_mode;
      if (!publish_service()) return false;
      std::this_thread::sleep_for(kServiceStepSleep);
    }
    const bool released = commissioning_request({0, home_identity,
        rosie::rtcore::CommissioningOperation::Release, axis_bit}, nullptr, nullptr, &request_reason);
    if (!released) fail_home(NATIVE_HOME_ABORT_RELEASE_FAILED, rosie::rtcore::commissioning_reason_name(request_reason));
    return released;
  };

  bool succeeded = true;
  bool refreshed_truth = false;
  int32_t home_truth = 0;
  bool service_override_released = false;
  for (step_index = 0; step_index < native_home_cfg.transaction.size(); ++step_index) {
    if (!succeeded) break;
    const NativeHomeOp& step = native_home_cfg.transaction[step_index];
    step_op = native_home_op_name(step.kind);
    sample_absolute_due();
    if (!home_permitted()) {
      fail_home(NATIVE_HOME_ABORT_PERMISSION_LOST, "permission lost"); succeeded = false; break;
    }
    switch (step.kind) {
      case NativeHomeOpKind::kSetMode:
      case NativeHomeOpKind::kRestoreMode:
        requested_mode = step.value_i32;
        succeeded = publish_service();
        std::this_thread::sleep_for(kServiceStepSleep);
        break;
      case NativeHomeOpKind::kWriteSdo: {
        uint32_t abort_code = 0;
        if (!step.object.valid ||
            !write_sdo_scalar_axis(axis,
                                   step.object.index,
                                   step.object.subindex,
                                   step.object.type,
                                   step.value_i32,
                                   &abort_code, &home_identity)) {
          fail_home(abort_code ? abort_code : NATIVE_HOME_ABORT_SDO_WRITE_FAILED, "SDO write failed");
          logf("WARNING: native_home write_sdo failed axis=%u slave_pos=%u index=0x%04x sub=0x%02x abort=0x%08x value=%d",
               axis,
               static_cast<unsigned int>(opt.slave_position[axis]),
               static_cast<unsigned int>(step.object.index),
               static_cast<unsigned int>(step.object.subindex),
               static_cast<unsigned int>(abort_code),
               step.value_i32);
          succeeded = false;
        }
        break;
      }
      case NativeHomeOpKind::kControlwordSequence:
        if (step.controlword_count == 0) {
          fail_home(NATIVE_HOME_ABORT_DESCRIPTOR_INVALID, "empty controlword sequence");
          succeeded = false;
          break;
        }
        for (uint32_t step_i = 0; step_i < step.controlword_count; ++step_i) {
          sample_absolute_due();
          if (step.controlword_values[step_i] & 0x10u)
            native_home_wire_audit[axis].trigger_requested(wire_audit_token, now_monotonic_ns());
          requested_controlword = step.controlword_values[step_i];
          if (!publish_service()) { succeeded = false; break; }
          std::this_thread::sleep_for(kServiceStepSleep);
        }
        break;
      case NativeHomeOpKind::kWaitStatusword: {
        bool matched = false;
        const uint64_t deadline_ns = now_monotonic_ns() + kWaitStatuswordTimeoutNs;
        while (now_monotonic_ns() < deadline_ns && home_permitted()) {
          sample_absolute_due();
          const uint16_t statusword = latest_feedback.statusword[axis];
          if ((statusword & step.wait_all_set_mask) == step.wait_all_set_mask &&
              (statusword & step.wait_all_clear_mask) == 0u) {
            matched = true;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (wire_audit_token && requested_controlword == 31 &&
            step.wait_all_set_mask == 0x9000u && step.wait_all_clear_mask == 0x2000u)
          end_wire_audit();
        if (!matched) {
          fail_home(home_permitted() ? NATIVE_HOME_ABORT_STATUS_TIMEOUT : NATIVE_HOME_ABORT_PERMISSION_LOST,
                    home_permitted() ? "statusword timeout" : "permission lost");
          logf("WARNING: native_home wait_statusword timed out axis=%u slave_pos=%u all_set=0x%04x all_clear=0x%04x last_statusword=0x%04x",
               axis,
               static_cast<unsigned int>(opt.slave_position[axis]),
               static_cast<unsigned int>(step.wait_all_set_mask),
               static_cast<unsigned int>(step.wait_all_clear_mask),
               static_cast<unsigned int>(latest_feedback.statusword[axis]));
          succeeded = false;
        }
        break;
      }
      case NativeHomeOpKind::kWaitSdo: {
        bool matched = false;
        int32_t current_value = 0;
        uint32_t abort_code = 0;
        const uint64_t deadline_ns = now_monotonic_ns() + kWaitSdoTimeoutNs;
        while (now_monotonic_ns() < deadline_ns && home_permitted()) {
          sample_absolute_due();
          abort_code = 0;
          if (step.object.valid &&
              read_sdo_scalar_axis(axis,
                                   step.object.index,
                                   step.object.subindex,
                                   step.object.type,
                                   &current_value,
                                   &abort_code) &&
              current_value == step.value_i32) {
            matched = true;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!matched) {
          fail_home(!home_permitted() ? NATIVE_HOME_ABORT_PERMISSION_LOST :
                    abort_code ? abort_code : NATIVE_HOME_ABORT_SDO_TIMEOUT,
                    home_permitted() ? "SDO wait failed" : "permission lost");
          logf("WARNING: native_home wait_sdo timed out axis=%u slave_pos=%u index=0x%04x sub=0x%02x expected=%d last=%d abort=0x%08x",
               axis,
               static_cast<unsigned int>(opt.slave_position[axis]),
               static_cast<unsigned int>(step.object.index),
               static_cast<unsigned int>(step.object.subindex),
               step.value_i32,
               current_value,
               static_cast<unsigned int>(abort_code));
          succeeded = false;
        }
        break;
      }
      case NativeHomeOpKind::kReleaseServiceOverride:
        if (!service_override_released) {
          succeeded = release_service_override(true);
          service_override_released = succeeded;
        }
        break;
      case NativeHomeOpKind::kRefreshTruth: {
        int32_t refreshed_value = 0;
        if (!read_native_home_truth_axis(axis, &refreshed_value)) {
          fail_home(NATIVE_HOME_ABORT_TRUTH_REFRESH_FAILED, "truth refresh failed");
          succeeded = false;
          break;
        }
        home_truth = refreshed_value;
        refreshed_truth = true;
        if (axis_profiles[axis].coordinate_evidence.sdo_sampled) {
          if (absolute_frame_epoch[axis].load() == UINT64_MAX) {
            fail_home(NATIVE_HOME_ABORT_FRAME_EPOCH_EXHAUSTED, "frame epoch exhausted"); succeeded = false; break;
          }
          // Home deliberately changes the command origin. Bind only its
          // completed truth, never the pre-Home encoder/rotation pair.
          absolute_frame_epoch[axis].fetch_add(1);
          if (!sample_absolute_axis(axis)) { sample_failure(NATIVE_HOME_ABORT_REFRESH_SAMPLE_INVALID); succeeded = false; }
        }
        break;
      }
      case NativeHomeOpKind::kNone:
      default:
        fail_home(NATIVE_HOME_ABORT_DESCRIPTOR_INVALID, "unknown transaction operation");
        succeeded = false;
        break;
    }
  }

  step_index = native_home_cfg.transaction.size();
  step_op = "finalize";
  end_wire_audit(); // Also close interrupted/refused transactions before cleanup.
  if (succeeded && !home_permitted()) fail_home(NATIVE_HOME_ABORT_PERMISSION_LOST, "permission lost");
  succeeded = succeeded && home_permitted();
  if (succeeded && !refreshed_truth) {
    step_op = "final_truth";
    int32_t refreshed_value = 0;
    if (!read_native_home_truth_axis(axis, &refreshed_value)) {
      fail_home(NATIVE_HOME_ABORT_FINAL_TRUTH_FAILED, "final truth refresh failed");
      succeeded = false;
    } else {
      home_truth = refreshed_value;
    }
  }

  if (!service_override_released) {
    step_op = "release_service_override";
    succeeded = release_service_override(succeeded) && succeeded;
  }
  report_wire_audit();
  if (!succeeded || !home_permitted()) {
    fail_home(NATIVE_HOME_ABORT_PERMISSION_LOST, "permission lost");
    return false;
  }

  if (axis_profiles[axis].brake.present && axis_profiles[axis].independent_absolute_origin()) {
    step_op = "wait_capture_settle";
    if (!wait_native_home_capture_settle(*this, axis, home_identity,
        now_monotonic_ns() + 500000000ULL, now_monotonic_ns,
        [] { std::this_thread::sleep_for(std::chrono::milliseconds(1)); })) {
      logf("Home settling refused axis=%u seq=%llu now=%llu pdo=%llu fresh=0x%x sw=0x%x brake=%u service=0x%x enabled=0x%x faults=0x%x",
           axis, static_cast<unsigned long long>(latest_feedback.seq.load()),
           static_cast<unsigned long long>(now_monotonic_ns()),
           static_cast<unsigned long long>(uint64_t(latest_feedback.pdo_observed_time_ns[axis])),
           uint32_t(latest_feedback.pdo_fresh_axis_mask), uint32_t(latest_feedback.statusword[axis]),
           uint32_t(latest_feedback.brake_state[axis]), service_mode_axis_mask.load(),
           axis_enable_mask.load(), safety_fault_mask.load());
      fail_home(NATIVE_HOME_ABORT_COMPLETE_REFUSED, "disabled feedback did not settle");
      return false;
    }
  }

  step_op = "final_absolute_sample";
  if (axis_profiles[axis].coordinate_evidence.sdo_sampled && !sample_absolute_axis(axis)) {
    sample_failure(NATIVE_HOME_ABORT_FINAL_SAMPLE_INVALID);
    return false;
  }
  step_op = "CompleteHome";
  if (!commissioning_request({0, home_identity, rosie::rtcore::CommissioningOperation::CompleteHome,
                              axis_bit, 0, 0, home_truth}, nullptr, nullptr, &request_reason)) {
    fail_home(NATIVE_HOME_ABORT_COMPLETE_REFUSED, rosie::rtcore::commissioning_reason_name(request_reason));
    return false;
  }
  logf("EtherCAT native_home axis=%u slave_pos=%u feedback_counts=%d truth_value=%d commissioning_mode=%u steady_state_mode=%u steps=%zu",
       axis,
       static_cast<unsigned int>(opt.slave_position[axis]),
       pos_counts,
       latest_feedback.native_home_position_offset[axis],
       static_cast<unsigned int>(native_home_cfg.commissioning_mode),
       static_cast<unsigned int>(native_home_cfg.steady_state_mode),
       native_home_cfg.transaction.size());
  return true;
#else
  (void)pos_counts;
  (void)native_home_cfg;
  (void)sample_failure;
  (void)request_reason;
  fail_home(NATIVE_HOME_ABORT_BACKEND_UNAVAILABLE, "native-home backend unavailable");
  return false;
#endif
}

// A mailbox acquisition may finish after the current cycle's PDO receipt but
// before that cycle consumes CompleteHome. Wait off RT for a newer published
// receipt; never ask the cyclic proof validator to accept evidence from its future.
template <class Clock, class Pause>
bool wait_home_sample_receipt(MachineState& machine,
    const rosie::rtcore::CommissioningRequest& request, uint32_t axis,
    const rosie::rtcore::SampledAbsolutePosition& sample, uint64_t deadline,
    Clock clock, Pause pause) {
  if (!sample.valid || !sample.completed_ns) return false;
  while (clock() < deadline) {
    if (!machine.commissioning_running.load() || g_stop.load() ||
        !machine.control_policy.lease_valid_now() || !machine.bus_ready.load() ||
        machine.armed.load() || machine.axis_enable_mask.load() ||
        machine.control_policy.generation.load() != request.identity.generation ||
        machine.configuration_epoch.load() != request.identity.configuration_epoch ||
        machine.home_epoch.load() != request.identity.home_epoch ||
        sample.configuration_epoch != request.identity.configuration_epoch ||
        sample.home_epoch != machine.absolute_frame_epoch[axis].load()) return false;
    const auto before = machine.latest_feedback.seq.load(std::memory_order_acquire);
    const uint64_t observed = machine.latest_feedback.pdo_observed_time_ns[axis];
    const uint32_t fresh = machine.latest_feedback.pdo_fresh_axis_mask;
    const auto after = machine.latest_feedback.seq.load(std::memory_order_acquire);
    if (before && before == after && !(before & 1u) && (fresh & (1u << axis)) &&
        observed >= sample.completed_ns && observed <= clock()) return true;
    pause();
  }
  return false;
}

void MachineState::run_commissioning_worker() {
  request_port = &commissioning_controls;
  pthread_setname_np(pthread_self(), "commissioning");
  if (!apply_housekeeping_affinity("commissioning", commissioning_affinity_applied)) return;
  const bool have_sampled_absolute = std::any_of(axis_profiles.begin(), axis_profiles.begin() + opt.num_axes,
      [](const auto& profile) { return profile.coordinate_evidence.sdo_sampled; });

      rosie::rtcore::DeltaAcquisitionEvidence pending_reference_evidence{};
      auto commissioning_request = [&](rosie::rtcore::CommissioningRequest request,
                                       rosie::rtcore::CommissioningIdentity* reserved = nullptr,
                                       bool* job_completed = nullptr,
                                       rosie::rtcore::CommissioningReason* reason = nullptr) {
        if (reason) *reason = rosie::rtcore::CommissioningReason::Invalid;
        if (commissioning_ticket == std::numeric_limits<uint64_t>::max()) return false;
        request.ticket = ++commissioning_ticket;
        const uint64_t deadline = now_monotonic_ns() + 250000000ULL;
        CommissioningEnvelope envelope{request};
        if (request.operation == rosie::rtcore::CommissioningOperation::CompleteReference) {
          envelope.reference_evidence = pending_reference_evidence;
          envelope.home_sample = pending_reference_evidence.sample;
        }
        if (request.operation == rosie::rtcore::CommissioningOperation::CompleteHome) {
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
            if (request.axis_mask == (1u << axis)) sampled_absolute[axis].read(envelope.home_sample);
        }
        if (restore_anchor_job.load() == request.identity.job) {
          envelope.restoring = true;
          envelope.verified_delta_restore = verified_delta_restore_job.load() == request.identity.job;
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
            if ((request.operation == rosie::rtcore::CommissioningOperation::CompleteHome || request.operation == rosie::rtcore::CommissioningOperation::CompleteReference) &&
                request.axis_mask == (1u << axis)) envelope.anchor = restore_evidence[axis];
        }
        if (request.operation == rosie::rtcore::CommissioningOperation::CompleteHome || request.operation == rosie::rtcore::CommissioningOperation::CompleteReference) {
          for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
            if (request.axis_mask != (1u << axis) || !axis_profiles[axis].independent_absolute_origin()) continue;
            auto wait_sample = envelope.home_sample;
            if (request.operation == rosie::rtcore::CommissioningOperation::CompleteReference)
              wait_sample.completed_ns = pending_reference_evidence.offset_after_completed_ns;
            if (!wait_home_sample_receipt(*this, request, axis, wait_sample, deadline,
                now_monotonic_ns, [] { std::this_thread::sleep_for(std::chrono::milliseconds(1)); })) {
              if (reason) *reason = rosie::rtcore::CommissioningReason::NotReady;
              logf("%s refused stage=sample_receipt axis=%u sample_completed_ns=%llu deadline_ns=%llu",
                   envelope.restoring ? "Restore" : "Home", axis, static_cast<unsigned long long>(envelope.home_sample.completed_ns),
                   static_cast<unsigned long long>(deadline));
              return false;
            }
          }
        }
        // Each queued completion owns its evidence even after a worker timeout
        // or a later job. Clearing worker state cannot turn restoration into Home.
        if (!commissioning_requests.push(envelope)) { revoke_outputs(); return false; }
        while (now_monotonic_ns() < deadline) {
          CommissioningReceipt receipt{};
          while (commissioning_receipts.pop(receipt)) {
            auto expected = request.identity;
            if (receipt.accepted && (request.operation == rosie::rtcore::CommissioningOperation::Reserve || request.operation == rosie::rtcore::CommissioningOperation::ReserveReference))
              ++expected.home_epoch;
            if (receipt.ticket != request.ticket ||
                !rosie::rtcore::same_commissioning_identity(receipt.identity, expected)) continue;
            if (receipt.accepted && reserved) *reserved = receipt.identity;
            if (job_completed) *job_completed = receipt.job_completed;
            if (reason) *reason = receipt.reason;
            if (!receipt.accepted && receipt.home_diagnostic.axis != UINT32_MAX) {
              const auto& d = receipt.home_diagnostic;
              logf("Home completion refused axis=%u bind=%s binding_sample=%s coordinate_reason=%u "
                   "qualified_mask(candidate=1/config=2/counts=4/logical=8/limits=16)=%u "
                   "receipt_ns=%llu sample_started_ns=%llu sample_completed_ns=%llu "
                   "sample_frame=%llu history_frame=%llu sample_cycle=%llu stationary_since_cycle=%llu "
                   "stationary_min=%lld stationary_max=%lld acquisition_valid=%u acquisition_reason=%s "
                   "sample_configuration=%llu history_configuration=%llu "
                   "start_generation=%llu latest_generation=%llu stationary_generation=%llu "
                   "interval_valid=%u interval_min=%lld interval_max=%lld",
                   d.axis, rosie::rtcore::independent_bind_refusal_name(d.bind_refusal),
                   rosie::rtcore::sampled_invalidity_name(d.binding_sample_reason), d.coordinate_reason,
                   d.qualification_mask,
                   static_cast<unsigned long long>(d.receipt_ns),
                   static_cast<unsigned long long>(d.sample_started_ns),
                   static_cast<unsigned long long>(d.sample_completed_ns),
                   static_cast<unsigned long long>(d.sample_frame), static_cast<unsigned long long>(d.history_frame),
                   static_cast<unsigned long long>(d.sample_cycle), static_cast<unsigned long long>(d.stationary_since_cycle),
                   static_cast<long long>(d.stationary_min), static_cast<long long>(d.stationary_max),
                   d.acquisition_valid, rosie::rtcore::sampled_invalidity_name(d.acquisition_reason),
                   static_cast<unsigned long long>(d.sample_configuration),
                   static_cast<unsigned long long>(d.history_configuration),
                   static_cast<unsigned long long>(d.start_generation),
                   static_cast<unsigned long long>(d.latest_generation),
                   static_cast<unsigned long long>(d.stationary_generation), d.interval_valid,
                   static_cast<long long>(d.interval_min), static_cast<long long>(d.interval_max));
              logf("Home acquisition margin axis=%u receipt_ns=%llu anchor_counts=%lld "
                   "displacement_counts=%llu displacement_limit=%llu", d.axis,
                   static_cast<unsigned long long>(d.receipt_ns),
                   static_cast<long long>(d.anchor_counts),
                   static_cast<unsigned long long>(d.displacement_counts),
                   static_cast<unsigned long long>(d.displacement_limit));
            }
            return receipt.accepted;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // A lost result is not proof of non-execution. Revoke immediately;
        // Finish still uses the known reservation identity for cleanup.
        revoke_outputs();
        return false;
      };


      // Fresh inhibited acquisition for verified-delta restoration of an HM35 v3
      // anchor. It measures displacement only; the v3 C0/E0/607C stay the zero.
      auto acquire_delta_reference = [&](uint32_t axis, const rosie::rtcore::CommissioningIdentity& identity) {
        auto& e = pending_reference_evidence; e = {};
        e.identity = identity; e.axis_mask = 1u << axis;
        e.reserved_ns = now_monotonic_ns();
        e.deadline_ns = e.reserved_ns + axis_profiles[axis].coordinate_evidence.maximum_acquisition_age_ns;
        e.offset_configuration_epoch = identity.configuration_epoch;
        e.offset_drive_frame_epoch = absolute_frame_epoch[axis].load();
        auto permitted = [&]() {
          return commissioning_running.load() && !g_stop.load() && control_policy.lease_valid_now() &&
              !armed.load() && !axis_enable_mask.load() && !service_mode_axis_mask.load() &&
              !safety_fault_mask.load() && !execution_fault_reasons.load() && bus_ready.load() &&
              control_policy.generation.load() == identity.generation &&
              configuration_epoch.load() == identity.configuration_epoch && home_epoch.load() == identity.home_epoch &&
              absolute_frame_epoch[axis].load() == e.offset_drive_frame_epoch && now_monotonic_ns() < e.deadline_ns;
        };
        const auto& p = axis_profiles[axis];
        if (verified_delta_restore_job.load() != identity.job || !p.reference_verified_delta_enabled || !p.independent_absolute_origin() || !p.home.truth_source.valid ||
            p.home.truth_source.index != 0x607c || p.home.truth_source.subindex || p.home.truth_source.type != SdoScalarType::kI32 ||
            !permitted()) return false;
        uint32_t abort = 0;
        e.offset_before_started_ns = now_monotonic_ns();
        if (!read_sdo_scalar_axis(axis, 0x607c, 0, SdoScalarType::kI32, &e.offset_before_raw, &abort)) return false;
        e.offset_before_completed_ns = now_monotonic_ns();
        if (!permitted() || !sample_absolute_axis(axis) || !sampled_absolute[axis].read(e.sample) ||
            !e.sample.valid || !permitted()) return false;
        e.offset_after_started_ns = now_monotonic_ns();
        if (!read_sdo_scalar_axis(axis, 0x607c, 0, SdoScalarType::kI32, &e.offset_after_raw, &abort)) return false;
        e.offset_after_completed_ns = now_monotonic_ns(); e.offset_valid = true;
        int32_t signed_truth = 0;
        if (!permitted() || !rosie::rtcore::home_offset_from_scalar_bits(e.offset_after_raw, false,
              p.home_truth_sign, signed_truth)) return false;
        return commissioning_request({0, identity, rosie::rtcore::CommissioningOperation::CompleteReference,
            1u << axis, 0, 0, signed_truth});
      };

      auto wait_axis_inactive = [&](uint32_t axis, uint64_t generation) {
        // Unverified inactivity budget: retain 500000000 ns for three fresh
        // inactive observations after the mandatory configuration-epoch settle.
        const uint64_t deadline = now_monotonic_ns() + kMailboxSettleNs + 500000000ULL;
        uint64_t previous_seq = latest_feedback.seq.load();
        int32_t previous_position = latest_feedback.pos_counts[axis];
        uint32_t stable_samples = 0;
        while (now_monotonic_ns() < deadline && commissioning_running.load() &&
               !g_stop.load() && control_policy.lease_valid_now() &&
               control_policy.generation.load() == generation && bus_ready.load()) {
          const uint64_t sample_seq = latest_feedback.seq.load(std::memory_order_acquire);
          if (!(sample_seq & 1u) && sample_seq != previous_seq) {
            const uint16_t sw = latest_feedback.statusword[axis];
            const int32_t position = latest_feedback.pos_counts[axis];
            if (sample_seq == latest_feedback.seq.load(std::memory_order_acquire)) {
              const auto state = gradient::ds402::decode_statusword(sw);
              const bool inactive = state == gradient::ds402::State::SwitchOnDisabled ||
                  state == gradient::ds402::State::ReadyToSwitchOn ||
                  state == gradient::ds402::State::SwitchedOn;
              stable_samples = inactive && position == previous_position ? stable_samples + 1 : 0;
              previous_seq = sample_seq;
              previous_position = position;
              if (stable_samples >= 3 &&
                  mailbox_admitted(axis, configuration_epoch.load(), mailbox_ready_since_ns.load())) return true;
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
      };

      auto service_sdo_write_axes = [&](uint32_t axis_mask,
                                        uint16_t index,
                                        uint8_t subindex,
                                        uint8_t value_type,
                                        uint32_t value_u32) {
#if GRADIENT_HAVE_ECRT
        if (value_type != gradient::ipc::v1::SERVICE_SDO_VALUE_U16 || value_u32 > 0xFFFFu) {
          logf("WARNING: service_sdo_write skipped index=0x%04x sub=0x%02x; unsupported value_type=%u",
               static_cast<unsigned int>(index),
               static_cast<unsigned int>(subindex),
               static_cast<unsigned int>(value_type));
          return false;
        }

        // Service writes are limited to objects declared by the selected
        // profile. Arbitrary controlwords or motion objects must not bypass
        // the cyclic executor. Validate the whole mask before writing any axis.
        const uint64_t generation = control_policy.generation.load();
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          if (!(axis_mask & (1u << axis))) continue;
          // A service write is stored by the drive: never the storage-mode
          // object, and never a RAM-only trial key.
          bool declared = false;
          for (const auto& descriptor : startup_sdos) {
            if ((descriptor.axis_mask & (1u << axis)) && descriptor.index == index &&
                descriptor.subindex == subindex && descriptor.type == StartupSdoValueType::kU16 &&
                !descriptor.storage_mode && !descriptor.ram_only) declared = true;
          }
          if (!declared || !wait_axis_inactive(axis, generation)) {
            logf("WARNING: service_sdo_write rejected axis=%u; declared object and fresh inactive feedback required", axis);
            return false;
          }
        }

        uint8_t raw[sizeof(uint16_t)] = {};
        EC_WRITE_U16(raw, static_cast<uint16_t>(value_u32 & 0xFFFFu));
        for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
          if ((axis_mask & (1u << axis)) == 0u) {
            continue;
          }
          uint32_t abort_code = 0;
          int rc = -1;
          {
            std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
            if (g_stop.load(std::memory_order_relaxed) || !shared_master ||
                !commissioning_running.load() || !control_policy.lease_valid_now() ||
                control_policy.generation.load() != generation || !bus_ready.load()) {
              logf("WARNING: service_sdo_write skipped index=0x%04x sub=0x%02x; master unavailable",
                   static_cast<unsigned int>(index),
                   static_cast<unsigned int>(subindex));
              return false;
            }
            const auto epoch = configuration_epoch.load(), since = mailbox_ready_since_ns.load();
            if (!mailbox_admitted(axis, epoch, since, true)) return false;
            {
              rosie::rtcore::MailboxCallScope call(mailbox_calls_in_flight);
              rc = ecrt_master_sdo_download(shared_master, opt.slave_position[axis], index, subindex,
                                            raw, sizeof(raw), &abort_code);
            }
            if (!mailbox_admitted(axis, epoch, since, true)) return false;
          }
          if (rc != 0) {
            logf("WARNING: service_sdo_write failed axis=%u slave_pos=%u index=0x%04x sub=0x%02x rc=%d abort=0x%08x value=%u type=%u",
                 axis,
                 static_cast<unsigned int>(opt.slave_position[axis]),
                 static_cast<unsigned int>(index),
                 static_cast<unsigned int>(subindex),
                 rc,
                 static_cast<unsigned int>(abort_code),
                 static_cast<unsigned int>(value_u32),
                 static_cast<unsigned int>(value_type));
            return false;
          }
          logf("EtherCAT service_sdo_write axis=%u slave_pos=%u index=0x%04x sub=0x%02x value=%u type=%u",
               axis,
               static_cast<unsigned int>(opt.slave_position[axis]),
               static_cast<unsigned int>(index),
               static_cast<unsigned int>(subindex),
               static_cast<unsigned int>(value_u32),
               static_cast<unsigned int>(value_type));
        }
        return commissioning_running.load() && control_policy.lease_valid_now() &&
            generation == control_policy.generation.load() && bus_ready.load();
#else
        (void)axis_mask;
        (void)index;
        (void)subindex;
        (void)value_type;
        (void)value_u32;
        return false;
#endif
      };

    bool startup_readback_complete =
        startup_sdos.empty() || !metrics_startup_readback_enabled;
    uint32_t recovery_next_axis = 0;
    uint64_t startup_readback_ready_since_ns = 0;
    uint64_t native_home_offset_ready_since_ns = 0;
    std::array<uint64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> pending_home_truth{};
    std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> logged_retained_home_refusal{};
    uint64_t absolute_feedback_ready_since_ns = 0;
    uint64_t absolute_feedback_last_poll_ns = 0;
    uint32_t diagnostic_axis = 0;
    std::array<uint32_t, gradient::ipc::v1::GRADIENT_MAX_AXES> diagnostic_field{};
    bool startup_state_observed = false;
    uint64_t observed_configuration_epoch = configuration_epoch.load();
    uint64_t observed_verification_request = verification_request.load();
    uint32_t mapping_verified_mask = 0, mapping_mismatch_mask = 0;
    std::array<MappingVerification, gradient::ipc::v1::GRADIENT_MAX_AXES> mapping_progress{};
    uint64_t mapping_preserved_write_epoch = 0;
    bool last_startup_ready_flag = false;
    uint32_t last_startup_reset_count = 0;
    // Unverified absolute-feedback cadence, preserved from the publisher.
    constexpr uint64_t kAbsoluteFeedbackPollIntervalNs = 200000000ULL; // 200ms
    // Phase 1 (2026-04-20): when the extended PDO snapshot has published
    // a sample this recently, the SDO absolute_feedback poll is skipped.
    // The PDO path already carries the same multi-turn data atomically
    // with 0x6064, so the SDO mailbox cost is pure overhead in that
    // regime. If the extended PDO mapping was rejected by the drive, the
    // ext_sample_time_ns stays at 0 and the SDO fallback remains active.


    auto reset_startup_drive_config_feedback = [&](auto& feedback_store) {
      configuration_readbacks.publish({configuration_epoch.load(), 0, 0, observed_verification_request});
      observed_revision_valid_mask.store(0);
      feedback_store.fill(StartupSdoFeedback{});
      for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
        const auto& descriptor = startup_sdos[descriptor_i];
        auto& feedback = feedback_store[descriptor_i];
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          feedback.configured[i] = (descriptor.valid && (descriptor.axis_mask & (1u << i))) ? 1u : 0u;
          feedback.commanded[i] = feedback.configured[i] ? descriptor.values[i] : 0u;
        }
      }
    };

    auto perform_startup_drive_config_readback = [&](uint64_t now_ns, bool startup_ready_flag) {
#if GRADIENT_HAVE_ECRT
      if (!metrics_startup_readback_enabled) {
        startup_readback_complete = true;
        return;
      }
      if (startup_readback_complete || startup_sdos.empty()) {
        return;
      }
      if (!startup_ready_flag) {
        startup_readback_ready_since_ns = 0;
        return;
      }
      if (startup_readback_ready_since_ns == 0) {
        startup_readback_ready_since_ns = now_ns;
        return;
      }
      if (!mailbox_admitted(0, configuration_epoch.load(), mailbox_ready_since_ns.load())) {
        return;
      }

      startup_readback_complete = true;
      // Unverified retry count retained for transient mailbox failures.
      constexpr int kStartupReadbackAttempts = 5;
      for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
        const auto& descriptor = startup_sdos[descriptor_i];
        auto& feedback = latest_feedback.startup_drive_config_feedback[descriptor_i];
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          feedback.configured[i] = (descriptor.valid && (descriptor.axis_mask & (1u << i))) ? 1u : 0u;
          feedback.commanded[i] = feedback.configured[i] ? descriptor.values[i] : 0u;
        }
        logf("EtherCAT startup readback begin key=%s delay_ms=%llu",
             descriptor.key.c_str(),
             static_cast<unsigned long long>(kMailboxSettleNs / 1000000ULL));
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          if (!(descriptor.axis_mask & (1u << i))) continue;
          bool verified = false;
          for (int attempt = 1; attempt <= kStartupReadbackAttempts; ++attempt) {
            // Sampling checkpoints prevent a full startup readback group from
            // silently consuming the retained evidence age budget.
            sample_absolute_due();
            uint8_t readback_raw[4] = {};
            size_t readback_size = 0;
            uint32_t abort_code = 0;
            int rc = -1;
            {
              std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
              if (g_stop.load(std::memory_order_relaxed) || !commissioning_running.load() || !shared_master) {
                return;
              }
              const auto epoch = configuration_epoch.load(), since = mailbox_ready_since_ns.load();
              if (!mailbox_admitted(i, epoch, since, true)) { startup_readback_complete = false; return; }
              {
                rosie::rtcore::MailboxCallScope call(mailbox_calls_in_flight);
                rc = ecrt_master_sdo_upload(shared_master, opt.slave_position[i], descriptor.index,
                                            descriptor.subindex, readback_raw, sizeof(readback_raw),
                                            &readback_size, &abort_code);
              }
              if (!mailbox_admitted(i, epoch, since, true)) { startup_readback_complete = false; return; }
            }
            if (rc == 0 && abort_code == 0 && readback_size == startup_sdo_size(descriptor.type)) {
              const int64_t readback_value = startup_sdo_decode(descriptor.type, readback_raw);
              const bool readback_matches =
                  readback_value == descriptor.values[i];
              feedback.readback_valid[i] = 1u;
              feedback.readback[i] = readback_value;
              feedback.verified[i] = readback_matches ? 1u : 0u;
              logf("EtherCAT startup readback axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x commanded=%lld readback=%lld verified=%u%s",
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]),
                   descriptor.key.c_str(),
                   static_cast<unsigned int>(descriptor.index),
                   static_cast<unsigned int>(descriptor.subindex),
                   static_cast<long long>(descriptor.values[i]),
                   static_cast<long long>(readback_value),
                   readback_matches ? 1u : 0u,
                   descriptor.ram_only ? " storage=ram_only" : descriptor.storage_mode ? " storage=mode" : "");
              // A trial skipped as identical at this start is not replayed by
              // the master: a drive power cycle since then reverts it.
              if (descriptor.ram_only && !readback_matches)
                logf("WARNING: startup_trial axis=%u key=%s commanded=%lld readback=%lld: a RAM-only trial value was lost "
                     "(drive power cycle reverts it to the stored value); restart rt-core to write it again",
                     i, descriptor.key.c_str(), static_cast<long long>(descriptor.values[i]),
                     static_cast<long long>(readback_value));
              verified = true;
              break;
            }
            if (attempt < kStartupReadbackAttempts) {
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (attempt == kStartupReadbackAttempts) {
              feedback.readback_valid[i] = 0u;
              feedback.readback[i] = 0u;
              feedback.verified[i] = 0u;
              logf("WARNING: EtherCAT startup readback failed for axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x rc=%d abort=0x%08x size=%zu",
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]),
                   descriptor.key.c_str(),
                   static_cast<unsigned int>(descriptor.index),
                   static_cast<unsigned int>(descriptor.subindex),
                   rc,
                   static_cast<unsigned int>(abort_code),
                   readback_size);
            }
          }
          if (!verified) {
            feedback.readback_valid[i] = 0u;
            feedback.readback[i] = 0u;
            feedback.verified[i] = 0u;
          }
        }
      }
#else
      (void)now_ns;
      (void)startup_ready_flag;
      startup_readback_complete = true;
#endif
    };

    auto perform_startup_native_home_offset_refresh = [&](uint64_t now_ns, bool startup_ready_flag) {
#if GRADIENT_HAVE_ECRT
      if (!metrics_native_home_refresh_enabled || !have_native_home || armed.load() || axis_enable_mask.load() ||
          (control_policy.home_valid_mask.load() & configured_mask) == configured_mask) return;
      if (!startup_ready_flag) { native_home_offset_ready_since_ns = 0; return; }
      if (!native_home_offset_ready_since_ns) { native_home_offset_ready_since_ns = now_ns; return; }
      if (!mailbox_admitted(0, configuration_epoch.load(), mailbox_ready_since_ns.load())) return;
      // One axis per checkpoint; failures cannot monopolize the mailbox owner.
      for (uint32_t n = 0; n < opt.num_axes; ++n) {
        const uint32_t i = recovery_next_axis++ % opt.num_axes;
        if (!axis_profiles[i].home.valid || (control_policy.home_valid_mask.load() & (1u << i))) continue;
        if (axis_profiles[i].independent_absolute_origin() && !(absolute_stationary_mask.load() & (1u << i))) continue;
        if (pending_home_truth[i]) {
          if (home_truth_processed[i].load() != pending_home_truth[i]) continue;
          pending_home_truth[i] = 0;
        }
        const auto refusal = retained_home_refusal[i].load();
        if (refusal != logged_retained_home_refusal[i]) {
          logged_retained_home_refusal[i] = refusal;
          if (refusal == uint32_t(rosie::rtcore::RetainedHomeRefusal::CurrentReferenceAnchor))
            logf("retained_home_refused axis=%u reason=current_reference_anchor_rejected message=\"%s\"", i,
                 rosie::rtcore::kCurrentReferenceAnchorRejected);
          else if (refusal) logf("retained_home_refused axis=%u reason=%s", i,
              rosie::rtcore::retained_home_refusal_name(rosie::rtcore::RetainedHomeRefusal(refusal)));
        }
        if (commissioning_phase.load() != static_cast<uint32_t>(rosie::rtcore::CommissioningPhase::Idle) ||
            armed.load() || axis_enable_mask.load() || home_truth_ticket == UINT64_MAX) return;
        HomeTruthReadback readback{control_policy.generation.load(), configuration_epoch.load(),
                                  home_epoch.load(), ++home_truth_ticket, i, 0};
        readback.audit_generation = audit_generation.load();
        if (!read_native_home_truth_axis(i, &readback.offset)) return;
        if (axis_profiles[i].independent_absolute_origin()) {
          std::string field;
          readback.calibration_reason = load_restore_anchor(i, readback.anchor, &field);
          if (readback.calibration_reason == 2)
            logf("calibration_identity_mismatch axis=%u field=%s", i, field.c_str());
          if (field == "current_reference_anchor")
            retained_home_refusal[i].store(uint32_t(rosie::rtcore::RetainedHomeRefusal::CurrentReferenceAnchor));
          if (!readback.calibration_reason) {
            readback.retained_anchor_valid = true;
            // A failed attempt must never consume the previous sample.
            const auto before = absolute_samples[i];
            sample_absolute_axis(i, now_monotonic_ns);
            if (absolute_samples[i] != before) sampled_absolute[i].read(readback.sample);
            // Encoder-loss flags follow the raw sample in this attempt. A failed
            // or partial upload leaves alarm_objects_read false: no automatic
            // restore, and the explicit restore operations remain available.
            int32_t fault_state = 0, overflow = 0;
            uint32_t fault_abort = 0, overflow_abort = 0;
            const bool fault_read = read_sdo_scalar_axis(i, rosie::rtcore::kA6ecEncoderFaultStateIndex,
                rosie::rtcore::kA6ecEncoderFaultStateSubindex, SdoScalarType::kU16, &fault_state, &fault_abort);
            const bool overflow_read = fault_read && read_sdo_scalar_axis(i, rosie::rtcore::kA6ecMultiTurnOverflowIndex,
                rosie::rtcore::kA6ecMultiTurnOverflowSubindex, SdoScalarType::kU16, &overflow, &overflow_abort);
            readback.alarm_objects_read = fault_read && overflow_read;
            readback.encoder_fault_state = uint16_t(fault_state);
            readback.multi_turn_overflow = uint16_t(overflow);
            if (!readback.alarm_objects_read)
              logf("retained_home_alarm_read_failed axis=%u u41_05=%u abort=0x%08x c10_15=%u abort=0x%08x",
                   i, fault_read ? 1u : 0u, fault_abort, overflow_read ? 1u : 0u, overflow_abort);
          }
        }
        if (home_truth_readbacks.push(readback)) pending_home_truth[i] = readback.ticket;
        return;
      }
#else
      (void)now_ns; (void)startup_ready_flag;
#endif
    };

    auto perform_absolute_feedback_refresh = [&](uint64_t now_ns, bool startup_ready_flag) {
#if GRADIENT_HAVE_ECRT
      if (!metrics_absolute_feedback_poll_enabled) {
        absolute_feedback_ready_since_ns = 0;
        absolute_feedback_last_poll_ns = 0;
        latest_feedback.absolute_feedback.fill(AbsoluteFeedbackAxis{});
        return;
      }
      if (!have_absolute_feedback) {
        return;
      }
      if (!startup_ready_flag) {
        absolute_feedback_ready_since_ns = 0;
        absolute_feedback_last_poll_ns = 0;
        latest_feedback.absolute_feedback.fill(AbsoluteFeedbackAxis{});
        return;
      }
      if (absolute_feedback_ready_since_ns == 0) {
        absolute_feedback_ready_since_ns = now_ns;
        return;
      }
      if (!mailbox_admitted(0, configuration_epoch.load(), mailbox_ready_since_ns.load())) {
        return;
      }
      // One diagnostic upload per sampled cadence across the group keeps the
      // profile's diagnostics allowance independent of the number of axes.
      uint64_t diagnostic_interval = kAbsoluteFeedbackPollIntervalNs;
      for (uint32_t i = 0; i < opt.num_axes; ++i)
        if (axis_profiles[i].coordinate_evidence.sdo_sampled)
          diagnostic_interval = std::max(diagnostic_interval, axis_profiles[i].coordinate_evidence.sample_interval_ns);
      if (absolute_feedback_last_poll_ns != 0 &&
          now_ns - absolute_feedback_last_poll_ns < diagnostic_interval) {
        return;
      }
      // Phase 1 (2026-04-20): skip the SDO poll entirely when the
      // extended PDO snapshot already carries fresh multi-turn samples.
      // Saves mailbox bandwidth for other service SDO traffic (startup
      // config readback, native home config). If the PDO timestamp is
      // 0 (never sampled) or older than the threshold, fall through to
      // the SDO upload so the fallback stays live.
      absolute_feedback_last_poll_ns = now_ns;

      const uint32_t selected_diagnostic_axis = diagnostic_axis++ % opt.num_axes;

      for (uint32_t i = 0; i < opt.num_axes && i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
        const auto& absolute_feedback_cfg = axis_profiles[i].absolute;
        if (!absolute_feedback_cfg.valid) continue;
        if (axis_profiles[i].coordinate_evidence.sdo_sampled) {
          // At most one additional diagnostic upload per poll across the group.
          // The sampler owns the two rotation encoder objects and their pairing;
          // remaining declared fields retain individual latest readbacks.
          if (i == selected_diagnostic_axis && absolute_feedback_cfg.field_count) {
            const auto field = diagnostic_field[i]++ % absolute_feedback_cfg.field_count;
            const auto& cfg = absolute_feedback_cfg.fields[field];
            if (cfg.valid && cfg.object.valid && !(cfg.object.index == 0x2040 &&
                (cfg.object.subindex == 0x2b || cfg.object.subindex == 0x2d)))
              (void)read_absolute_feedback_field_axis(i, cfg.object.index, cfg.object.subindex,
                  cfg.object.type, &latest_feedback.absolute_feedback[i].fields[field]);
          }
          continue;
        }
        AbsoluteFeedbackAxis axis_feedback{};
        // 2026-04-21 (pass 2): latch paired_pos_counts AS CLOSE IN TIME
        // AS POSSIBLE to the multi-turn U40.20/U40.22 reads, not after
        // all 8 fields are polled. SDO uploads take ~5-20 ms each, so
        // polling 8 fields sequentially spans ~40-160 ms. If we latch
        // 6064 only at the END of that window, the paired value lags
        // U40.20/U40.22 by ~100 ms under motion — at 0.1 rad/s that's
        // ~1000 counts of skew and the shaft-frame gate trips. Instead
        // we latch once just BEFORE the U40.20/U40.22 read (best case
        // skew ~1-2 ms) whenever the field config includes multi-turn
        // entries. Fallback: if neither multi-turn subindex is present
        // in the config, latch at end-of-loop as before (skew is then
        // bounded only by the full field-list duration).
        bool paired_latched = false;
        auto latch_paired = [&]() {
          if (paired_latched) return;
          axis_feedback.paired_pos_counts = latest_feedback.pos_counts[i];
          axis_feedback.paired_valid = 1;
          axis_feedback.paired_sample_time_ns = now_monotonic_ns();
          paired_latched = true;
        };
        for (uint32_t field_i = 0; field_i < absolute_feedback_cfg.field_count; ++field_i) {
          const auto& field_cfg = absolute_feedback_cfg.fields[field_i];
          if (!field_cfg.valid || !field_cfg.object.valid) {
            continue;
          }
          // Latch the paired 6064 snapshot just before the FIRST
          // multi-turn subindex read (U40.20 / U40.22 / U40.2B / U40.2D
          // on the A6-EC). This puts paired_pos_counts within ~1-2 ms
          // of the multi-turn value Python will consume for the shaft-
          // frame gate.
          const bool is_multi_turn_field =
              field_cfg.key == axis_profiles[i].absolute_pair_field;
          if (is_multi_turn_field) {
            latch_paired();
          }
          (void)read_absolute_feedback_field_axis(
              i,
              field_cfg.object.index,
              field_cfg.object.subindex,
              field_cfg.object.type,
              &axis_feedback.fields[field_i]);
        }
        // Fallback latch: if the field config does not include any
        // multi-turn subindex, still publish a paired snapshot at the
        // end of the loop so downstream Python can detect "paired is
        // valid" even though it carries the full-loop skew. This keeps
        // the rest-state behavior identical to the original design.
        latch_paired();
        latest_feedback.absolute_feedback[i] = axis_feedback;
      }
#else
      (void)now_ns;
      (void)startup_ready_flag;
#endif
    };


  while (commissioning_running.load() && !g_stop.load()) {
    // Fixed-preset bring-up proved the physical and scanned layouts in PREOP
    // before registering/activating; explicit assignment mode obtains its first
    // proof here, through the admitted runtime readback after activation.
    // Recheck on configuration epochs only after continuous readiness; never
    // repair assignments here.
    bool mapping_readback_available = false;
#if GRADIENT_HAVE_ECRT
    mapping_readback_available = diagnostic_handles_ready.load(std::memory_order_acquire) &&
        mailbox_admitted(0, configuration_epoch.load(), mailbox_ready_since_ns.load());
#endif
    const bool mapping_pending = mapping_readback_available &&
        std::any_of(mapping_progress.begin(), mapping_progress.begin() + opt.num_axes,
                    [](const auto& progress) { return progress.pending(); });
    CommissioningJob job{};
    bool command_job = commissioning_jobs.pop(job);
    if (!command_job) {
      if (!alarm_readback_pending() && !mapping_pending && !have_sampled_absolute &&
          (control_policy.connected.load() || !readbacks_requested.load())) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (!try_claim_commissioning(CommissioningClaim::Readback)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
    }
    CommissioningClaimRelease release{&commissioning_claim};
    if (!command_job) {
      refresh_drive_alarm();
      sample_absolute_due();
      refresh_monitor();
      if (!mapping_pending && (control_policy.connected.load() || !readbacks_requested.load())) {
        release.release();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      readbacks_requested.store(false);
      job = readback_job();
    }
    auto command_result = job.result;
    if (job.kind == CommissioningJobKind::Home && (job.home.axis_mask & restore_anchor_selector)) {
      job.kind = CommissioningJobKind::RestoreAnchor;
      job.home.axis_mask &= ~restore_anchor_selector;
    }
    if (job.kind == CommissioningJobKind::Home || job.kind == CommissioningJobKind::RestoreAnchor || job.kind == CommissioningJobKind::RestoreReferenceVerifiedDelta)
      command_result.axis_mask = job.home.axis_mask;
    if (job.kind == CommissioningJobKind::Readback && commissioning_job_current(job)) {
      const uint64_t now_ns = now_monotonic_ns();
      const uint8_t startup_ready = latest_feedback.startup_ready;
      const uint32_t startup_reset_count = latest_feedback.startup_reset_count;
      const bool startup_ready_flag = startup_ready != 0;
      if (!startup_state_observed) {
        startup_state_observed = true;
        last_startup_ready_flag = startup_ready_flag;
        last_startup_reset_count = startup_reset_count;
      } else if (configuration_epoch.load() != observed_configuration_epoch ||
                 verification_request.load() != observed_verification_request ||
                 startup_reset_count != last_startup_reset_count ||
                 (last_startup_ready_flag && !startup_ready_flag)) {
        // Only a successful declared scalar write can retain the immutable
        // PDO proof across its exact new epoch. A bus reset, explicit verify,
        // or any other epoch change still requires fresh mapping uploads.
        const bool preserve_mapping = configuration_epoch.load() == mapping_preserved_write_epoch &&
            verification_request.load() == observed_verification_request &&
            startup_reset_count == last_startup_reset_count && startup_ready_flag && last_startup_ready_flag;
        mapping_preserved_write_epoch = 0;
        observed_configuration_epoch = configuration_epoch.load();
        observed_verification_request = verification_request.load();
        if (!preserve_mapping) { mapping_progress = {}; mapping_verified_mask = mapping_mismatch_mask = 0; }
        startup_readback_complete =
            startup_sdos.empty() || !metrics_startup_readback_enabled;
        startup_readback_ready_since_ns = 0;
        startup_observe_progress = {};
        native_home_offset_ready_since_ns = 0;
        pending_home_truth.fill(0);
        absolute_feedback_ready_since_ns = 0;
        absolute_feedback_last_poll_ns = 0;
        reset_startup_drive_config_feedback(latest_feedback.startup_drive_config_feedback);
        latest_feedback.absolute_feedback.fill(AbsoluteFeedbackAxis{});
        logf("EtherCAT startup epoch changed: ready=%u->%u reset_count=%u->%u; rearming startup readback/offset refresh",
             last_startup_ready_flag ? 1u : 0u,
             startup_ready_flag ? 1u : 0u,
             static_cast<unsigned int>(last_startup_reset_count),
             static_cast<unsigned int>(startup_reset_count));
        last_startup_ready_flag = startup_ready_flag;
        last_startup_reset_count = startup_reset_count;
      } else {
        last_startup_ready_flag = startup_ready_flag;
        last_startup_reset_count = startup_reset_count;
      }

      perform_startup_drive_config_readback(now_ns, startup_ready_flag);
      perform_startup_native_home_offset_refresh(now_ns, startup_ready_flag);
      perform_absolute_feedback_refresh(now_ns, startup_ready_flag);
      uint32_t verified_mask = 0;
      if (mapping_readback_available && configuration_epoch.load() == observed_configuration_epoch) {
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          if (mapping_progress[i].pending()) {
            sample_absolute_due();
            if (!mailbox_admitted(i, observed_configuration_epoch, mailbox_ready_since_ns.load())) break;
            mapping_progress[i].observe(verify_axis_mapping(i));
            if (mapping_progress[i].verified) mapping_verified_mask |= 1u << i;
            if (mapping_progress[i].faulted) {
              mapping_mismatch_mask |= 1u << i;
              logf("EtherCAT mapping_readback axis=%u result=fault attempts=%u", i, mapping_progress[i].attempts);
            }
          }
          // A mailbox match alone never qualifies incomplete cyclic feedback.
          if (!bus_ready.load(std::memory_order_acquire)) continue;
          bool verified = (mapping_verified_mask & (1u << i)) != 0;
          const uint32_t revision = axis_profiles[i].revision_no;
          if ((revision || axis_profiles[i].coordinate_evidence.enabled) &&
              !(observed_revision_valid_mask.load() & (1u << i))) {
            int32_t value = 0;
            uint32_t abort_code = 0;
            if (read_sdo_scalar_axis(i, 0x1018, 3, SdoScalarType::kU32, &value, &abort_code)) {
              observed_revision[i].store(static_cast<uint32_t>(value));
              observed_revision_valid_mask.fetch_or(1u << i);
            }
          }
          if (axis_profiles[i].coordinate_evidence.enabled &&
              !(observed_serial_attempted_mask.load() & (1u << i))) {
            int32_t serial = 0;
            uint32_t abort_code = 0;
            if (read_sdo_scalar_axis(i, 0x1018, 4, SdoScalarType::kU32, &serial, &abort_code)) {
              observed_serial[i].store(static_cast<uint32_t>(serial));
              observed_serial_valid_mask.fetch_or(1u << i);
            }
            observed_serial_attempted_mask.fetch_or(1u << i);
          }
          if ((revision || axis_profiles[i].coordinate_evidence.enabled) &&
              !(observed_revision_valid_mask.load() & (1u << i))) verified = false;
          if (revision) {
#if GRADIENT_HAVE_ECRT
            // Profile revision is SII (0x2ef8 on this bench); CoE 0x1018:3
            // is firmware 22033/22034 and remains the persisted drive identity.
            std::lock_guard<std::mutex> lock(shared_master_sdo_mutex);
            if (!verify_sii_revision(shared_master, opt.slave_position[i], revision)) verified = false;
#else
            if (!(observed_revision_valid_mask.load() & (1u << i)) ||
                observed_revision[i].load() != revision) verified = false;
#endif
          }
          for (size_t d = 0; d < startup_sdos.size(); ++d) {
            if ((startup_sdos[d].axis_mask & (1u << i)) &&
                !latest_feedback.startup_drive_config_feedback[d].verified[i]) verified = false;
          }
          if (verified) verified_mask |= 1u << i;
        }
      }
      if (configuration_epoch.load() != observed_configuration_epoch) verified_mask = 0;
      configuration_readbacks.publish({observed_configuration_epoch, verified_mask, mapping_mismatch_mask, observed_verification_request});
      // Observe-only reads follow the published verification and never feed it.
      if (startup_ready_flag && startup_readback_complete && bus_ready.load() && !armed.load() && !axis_enable_mask.load() &&
          configuration_epoch.load() == observed_configuration_epoch &&
          std::none_of(mapping_progress.begin(), mapping_progress.begin() + opt.num_axes,
                       [](const auto& progress) { return progress.pending(); }))
        (void)startup_observe_step();

    } else if ((job.kind == CommissioningJobKind::RestoreAnchor || job.kind == CommissioningJobKind::RestoreReferenceVerifiedDelta) && commissioning_job_current(job)) {
      auto identity = job.identity;
      const uint32_t mask = job.home.axis_mask;
      const bool verified_delta = job.kind == CommissioningJobKind::RestoreReferenceVerifiedDelta;
      const char* failed_stage = "admission";
      uint32_t failed_axis = UINT32_MAX, source_reason = 0;
      auto decision_reason = rosie::rtcore::CommissioningReason::None;
      bool ok = !job.home.flags && mask && !(mask & ~configured_mask) &&
          !armed.load() && !axis_enable_mask.load() && !jog_session_open.load();
      for (uint32_t axis = 0; ok && axis < opt.num_axes; ++axis) {
        if (!(mask & (1u << axis))) continue;
        std::string changed_field;
        const auto reason = load_restore_anchor(axis, restore_evidence[axis], &changed_field);
        if (reason == 2) logf("anchor_identity_mismatch axis=%u field=%s Home required", axis, changed_field.c_str());
        if (changed_field == "current_reference_anchor")
          logf("Restore refused axis=%u reason=current_reference_anchor_rejected message=\"%s\"", axis,
               rosie::rtcore::kCurrentReferenceAnchorRejected);
        anchor_reason[axis].store(reason);
        ok = reason == 0;
        // Verified delta re-establishes an HM35 zero only; it never defines one.
        if (ok && verified_delta) ok = !(mask & (mask-1)) && restore_evidence[axis].version == 3 &&
            axis_profiles[axis].reference_verified_delta_enabled;
        if (!ok) { failed_stage = "anchor_load"; failed_axis = axis; source_reason = reason; }
      }
      bool reserved = false, completed = false;
      if (ok) {
        restore_anchor_job.store(identity.job, std::memory_order_release);
        verified_delta_restore_job.store(verified_delta ? identity.job : 0, std::memory_order_release);
        failed_stage = "reserve";
        if (verified_delta && (mask & (mask - 1))) ok = false;
        reserved = ok && commissioning_request({0, identity, verified_delta ? rosie::rtcore::CommissioningOperation::ReserveReference
            : rosie::rtcore::CommissioningOperation::Reserve, mask}, &identity, nullptr, &decision_reason);
        ok = reserved;
      }
      if (reserved) {
        job.identity = identity;
        for (uint32_t axis = 0; ok && axis < opt.num_axes; ++axis) {
          if (!(mask & (1u << axis))) continue;
          failed_axis = axis;
          if (verified_delta) {
            failed_stage = "verify_reference_delta";
            ok = acquire_delta_reference(axis, identity);
            if (!ok) (void)commissioning_request({0, identity, rosie::rtcore::CommissioningOperation::FailReference, 1u << axis});
            continue;
          }
          failed_stage = "absolute_sample";
          if (axis_profiles[axis].coordinate_evidence.sdo_sampled &&
              !sample_absolute_axis(axis)) {
            rosie::rtcore::SampledAbsolutePosition sample{};
            if (sampled_absolute[axis].read(sample)) source_reason = sample.reason;
            ok = false; break;
          }
          failed_stage = "complete_home";
          ok = commissioning_request({0, identity, rosie::rtcore::CommissioningOperation::CompleteHome,
              1u << axis, 0, 0, restore_evidence[axis].offset}, nullptr, nullptr, &decision_reason);
          if (!ok) source_reason = anchor_reason[axis].load();
        }
        auto finish_reason = rosie::rtcore::CommissioningReason::None;
        const bool finished = commissioning_request({0, identity, rosie::rtcore::CommissioningOperation::Finish},
                                                    nullptr, &completed, &finish_reason);
        if (ok && (!finished || !completed)) {
          failed_stage = "finish"; failed_axis = UINT32_MAX; decision_reason = finish_reason;
        }
        ok = ok && finished && completed;
      } else if (restore_anchor_job.load() == identity.job && identity.home_epoch != UINT64_MAX) {
        // A lost reservation receipt still requires cleanup of its allocated epoch.
        ++identity.home_epoch;
        (void)commissioning_request({0, identity, rosie::rtcore::CommissioningOperation::Finish});
      }
      restore_anchor_job.store(0, std::memory_order_release);
      verified_delta_restore_job.store(0, std::memory_order_release);
      if (!ok) logf("Restore failed stage=%s axis=%u source_reason=%u commissioning_reason=%s completed=%u",
          failed_stage, failed_axis, source_reason, rosie::rtcore::commissioning_reason_name(decision_reason), completed ? 1u : 0u);
      command_result.result = ok ? rosie::protocol::result_accepted : rosie::protocol::result_rejected;
      command_result.reason = ok ? rosie::protocol::reason_none : rosie::protocol::reason_not_ready;
    } else if (commissioning_job_current(job)) {
      gradient::ipc::v1::MsgHeader header{};
      header.seq = job.result.sequence;
      header.bytes = sizeof(header) + sizeof(job.home);
      const auto* mh = &header;
      const auto* payload = reinterpret_cast<const uint8_t*>(&job.home);
      switch (job.kind == CommissioningJobKind::Home
                  ? gradient::ipc::v1::MSG_CMD_NATIVE_HOME : 0) {
                case gradient::ipc::v1::MSG_CMD_NATIVE_HOME: {
                  if (!control_policy.lease_valid_now() || !bus_ready.load()) {
                    logf("Native-home admission refused axis_mask=0x%x lease_valid=%u bus_ready=%u",
                         job.home.axis_mask, control_policy.lease_valid_now() ? 1u : 0u, bus_ready.load() ? 1u : 0u);
                    break;
                  }
                  if (armed.load() && !jog_session_open.load(std::memory_order_acquire)) {
                    command_result.reason = rosie::protocol::reason_mode_conflict;
                    break;
                  }
                  if (mh->bytes >= sizeof(*mh) + sizeof(gradient::ipc::v1::CmdNativeHomeV1)) {
                    auto* cmd =
                        reinterpret_cast<const gradient::ipc::v1::CmdNativeHomeV1*>(payload);
                    if (cmd->flags) {
                      command_result.reason = rosie::protocol::reason_invalid_command;
                      break;
                    }
                    uint32_t mask = cmd->axis_mask;
                    const uint32_t valid =
                        (opt.num_axes >= 32) ? 0xFFFFFFFFu : ((1u << opt.num_axes) - 1u);
                    if (!mask || (mask & ~valid)) {
                      command_result.reason = rosie::protocol::reason_invalid_command;
                      break;
                    }
                    const uint32_t unverified = mask & ~control_policy.config_verified_mask.load();
                    if (unverified) {
                      command_result.axis_mask = unverified;
                      logf("Native-home admission refused: configuration unverified axis_mask=0x%x", unverified);
                      break;
                    }
                    mask &= valid;
                    if (mask != 0) {
                      const auto prepared_home = request_control({ControlOperation::PrepareHome, mask});
                      if (!prepared_home.accepted) break;
                      const uint32_t cur = prepared_home.previous_mask;
                      const uint32_t next = prepared_home.enabled_mask;
                      (void)request_control({ControlOperation::AbortMotion});
                      rosie::rtcore::CommissioningIdentity home_identity{
                          job.identity.job, job.identity.generation, job.identity.configuration_epoch, job.identity.home_epoch};
                      const auto requested_identity = home_identity;
                      auto reserve_reason = rosie::rtcore::CommissioningReason::Invalid;
                      if (!commissioning_request({0, requested_identity,
                              rosie::rtcore::CommissioningOperation::Reserve, mask}, &home_identity,
                              nullptr, &reserve_reason)) {
                        // Observation only, after the correlated refusal. Read the
                        // published recovery snapshot rather than RT-owned evidence.
                        rosie::rtcore::FaultRecoverySnapshot recovery{};
                        const bool have_recovery = recovery_snapshot.read(recovery);
                        logf("Native-home admission refused cmd_seq=%llu stage=reserve axis_mask=0x%x "
                             "reason=%s observed_safety_axis_mask=0x%x observed_fault_bits=0x%x recovery_available=%u",
                             static_cast<unsigned long long>(job.result.sequence), mask,
                             rosie::rtcore::commissioning_reason_name(reserve_reason),
                             safety_fault_mask.load(), execution_fault_reasons.load(), have_recovery ? 1u : 0u);
                        if (have_recovery) {
                          for (size_t bit = 0; bit < std::size(rosie::rtcore::kFaultRecovery); ++bit) {
                            if (!(recovery.faults & (1u << bit))) continue;
                            const auto& policy = rosie::rtcore::kFaultRecovery[bit];
                            const auto& evidence = recovery.evidence[bit];
                            logf("Native-home recovery cmd_seq=%llu fault=%s fault_bit=%zu "
                                 "affected_axis_mask=0x%x requested_axis_mask=0x%x recovery=%s rehome_axis_mask=0x%x",
                                 static_cast<unsigned long long>(job.result.sequence), policy.name, bit,
                                 evidence.affected_axis_mask, mask, rosie::rtcore::recovery_name(policy.recovery),
                                 evidence.affected_axis_mask & (evidence.untrusted_axis_mask |
                                     (policy.invalidates_home ? evidence.affected_axis_mask : 0u)));
                          }
                        }
                        command_result.result = rosie::protocol::result_rejected;
                        command_result.reason = rosie::protocol::reason_not_ready;
                        // Reserve may have applied even if its receipt was lost.
                        // The allocated epoch is deterministic and cleanup can
                        // release only this original reservation, never another.
                        if (requested_identity.home_epoch != std::numeric_limits<uint64_t>::max()) {
                          home_identity = requested_identity;
                          ++home_identity.home_epoch;
                          (void)commissioning_request({0, home_identity,
                              rosie::rtcore::CommissioningOperation::Finish});
                        }
                        break;
                      }
                      auto home_job_permitted = [&]() {
                        return commissioning_running.load() && !g_stop.load() && control_policy.lease_valid_now() &&
                            control_policy.generation.load() == home_identity.generation &&
                            configuration_epoch.load() == home_identity.configuration_epoch &&
                            home_epoch.load() == home_identity.home_epoch &&
                            dirty_configuration_epoch.load() == job.dirty_epoch && bus_ready.load();
                      };
                      job.identity = home_identity;
                      for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
                        if (mask & (1u << axis)) native_home_failures[axis].publish({});
                      // Commission only after fresh feedback proves stationary,
                      // inactive drives. Unknown/stale DS402 state is not proof.
                      const uint64_t precondition_generation = home_identity.generation;
                      uint32_t disarm_confirmed_mask = 0;
                      uint32_t disarm_failed_mask = 0;
                      for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
                        if ((mask & (1u << axis)) == 0u) continue;
                        const bool confirmed = wait_axis_inactive(axis, precondition_generation);
                        if (confirmed) {
                          disarm_confirmed_mask |= (1u << axis);
                        } else {
                          disarm_failed_mask |= (1u << axis);
                          (void)commissioning_request({0, home_identity,
                              rosie::rtcore::CommissioningOperation::FailHome, 1u << axis, 0, 0, 0,
                              gradient::ipc::v1::NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT});
                          logf("WARNING: native_home disarm precondition timeout axis=%u last_statusword=0x%04x abort=0x%08x",
                               axis,
                               static_cast<unsigned int>(
                                   latest_feedback.statusword[axis]),
                               static_cast<unsigned int>(
                                   gradient::ipc::v1::
                                       NATIVE_HOME_ABORT_DISARM_PRECONDITION_TIMEOUT));
                        }
                      }
                      if (disarm_confirmed_mask != 0u) {
                        logf("Native-home disarm precondition satisfied axis_mask=0x%x (timeouts=0x%x)",
                             static_cast<unsigned int>(disarm_confirmed_mask),
                             static_cast<unsigned int>(disarm_failed_mask));
                      }
                      // HM35 is per drive, but this worker blocks on each transaction
                      // and consumes one ordered receipt stream. Concurrent masked
                      // axes need separate transaction state and receipt dispatch;
                      // sharing this consumer would discard another axis's receipt.
                      // Keep the group fail-closed: the first failure prevents later
                      // axes starting and drains every active service override.
                      bool all_succeeded = (disarm_failed_mask == 0u);
                      for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
                        if (!all_succeeded || !home_job_permitted()) { all_succeeded = false; break; }
                        if ((disarm_confirmed_mask & (1u << axis)) == 0u) {
                          continue;
                        }
                        if (!native_home_axis(axis, home_identity, commissioning_request)) all_succeeded = false;
                      }
                      all_succeeded = all_succeeded && home_job_permitted();
                      bool job_completed = false;
                      const bool finished = commissioning_request({0, home_identity,
                          rosie::rtcore::CommissioningOperation::Finish}, nullptr, &job_completed);
                      all_succeeded = all_succeeded && finished && job_completed;
                      command_result.result = all_succeeded ? rosie::protocol::result_accepted : rosie::protocol::result_rejected;
                      command_result.reason = all_succeeded ? rosie::protocol::reason_none : rosie::protocol::reason_not_ready;
                      if (all_succeeded) {
                        logf("Native-home success: homed axes remain disabled axis_mask 0x%x -> 0x%x armed=%u",
                             static_cast<unsigned int>(cur),
                             static_cast<unsigned int>(next),
                             static_cast<unsigned int>(
                                 armed.load(std::memory_order_relaxed) ? 1u : 0u));
                      } else {
                        logf("WARNING: native_home left requested axes disarmed after failure axis_mask=0x%x disarm_timeouts=0x%x",
                             static_cast<unsigned int>(mask),
                             static_cast<unsigned int>(disarm_failed_mask));
                      }
                    }
                  }
                  break;
                }

        default: {
          auto identity = job.identity;
          const auto mask = job.write.axis_mask;
          configuration_write_job.store(identity.job, std::memory_order_release);
          if (job.write.flags || !mask || (mask & ~configured_mask)) break;
          const auto requested_identity = identity;
          // A configuration write is how a mismatched drive parameter gets
          // corrected, so its reservation must not require the configuration
          // to be verified already; disarmed, leased, bus-ready and no motion
          // still apply through commissioning_permission.
          rosie::rtcore::CommissioningRequest reserve{0, identity,
                  rosie::rtcore::CommissioningOperation::Reserve, mask};
          reserve.configuration_write = true;
          if (!commissioning_request(reserve, &identity)) {
            // As with Home, loss of Reserve's receipt is not proof that RT
            // refused it; only this deterministically allocated identity cleans up.
            if (requested_identity.home_epoch != UINT64_MAX) {
              identity = requested_identity;
              ++identity.home_epoch;
              (void)commissioning_request({0, identity, rosie::rtcore::CommissioningOperation::Finish});
            }
            break;
          }
          job.identity = identity;
          ControlRequest begin{ControlOperation::BeginService};
          begin.job = identity.job;
          const auto begun = request_control(begin);
          bool succeeded = begun.accepted;
          job.dirty_epoch = dirty_configuration_epoch.load();
          if (succeeded) {
            succeeded = service_sdo_write_axes(mask, job.write.index, job.write.subindex,
                                              job.write.value_type, job.write.value_u32);
            (void)request_control({ControlOperation::EndService});
          }
          const bool finished = commissioning_request({0, identity,
              rosie::rtcore::CommissioningOperation::Finish});
          succeeded = succeeded && control_policy.generation.load() == begun.generation &&
              control_policy.lease_valid_now();
          command_result.result = succeeded && finished ? rosie::protocol::result_accepted
                                                       : rosie::protocol::result_rejected;
          command_result.reason = succeeded && finished ? rosie::protocol::reason_none
                                                       : rosie::protocol::reason_not_ready;
          if (succeeded && finished && bus_ready.load()) mapping_preserved_write_epoch = configuration_epoch.load();
          break;
        }
      }
    }
    configuration_write_job.store(0, std::memory_order_release);
    // Finish cleanup has run for the reservation and service overrides. Receipt
    // consumers may immediately submit another Home, even if the old HTTP
    // request disappeared; status delivery must never retain mailbox ownership.
    release.release();
    if (command_job && job.kind != CommissioningJobKind::Readback)
      logf("Commissioning receipt cmd_seq=%llu operation=0x%x result=%u reason=%u",
           static_cast<unsigned long long>(command_result.sequence), command_result.operation,
           command_result.result, command_result.reason);
    if (command_job && !commissioning_job_receipts.push({job.identity, job.dirty_epoch, command_result, job.kind}))
      revoke_outputs();
  }
}
