#pragma once

// Included inside the daemon namespace after its numeric and logging helpers.
// Keep profile parsing header-only in the existing single translation unit.

struct AxisConfig {
  // Raw scaling inputs.
  uint32_t counts_per_rev = 131072; // common 17-bit encoder counts per rev
  double gear_ratio = 1.0;
  int sign = +1; // +1 or -1 (mechanical orientation)
  uint8_t axis_type = gradient::ipc::v1::AXIS_TYPE_ROTARY; // q is radians by default
  double lead_m_per_rev = 0.0; // only used when axis_type==AXIS_TYPE_LINEAR

  // Derived scaling (RTCore uses this for q->counts conversion).
  double counts_per_unit = 0.0; // counts per rad (rotary) or counts per meter (linear)

  // Explicit compiled drive-frame cap, or legacy --max-rpm derivation;
  // emitted steps use PositionRateGuard.
  uint32_t max_profile_vel_counts_per_s = 0;
};

constexpr double kCompletionWrapTwoPi = 6.28318530717958647692;

struct PdoLayoutEntry {
  std::string semantic;
  uint16_t index = 0;
  uint8_t subindex = 0;
  uint8_t bits = 0;
};

struct StartupSdoConfig {
  uint32_t axis_mask = 0xffffu;
  bool valid = false;
  std::string key;
  StartupSdoValueType type = StartupSdoValueType::kNone;
  uint16_t index = 0;
  uint8_t subindex = 0;
  std::array<int64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> values{};
  // Storage policy (axes.conf parameter_storage and startup_trial), set only on
  // single-axis descriptors. storage_mode marks the drive object that selects
  // whether SDO writes are stored (its value is the store setting, written
  // first when it differs); storage_off is the value that writes to RAM only.
  // ram_only marks a commissioning trial written with storage off, so a drive
  // power cycle reverts it. Neither changes what is verified by readback.
  bool storage_mode = false;
  bool ram_only = false;
  // The configured value becomes active only after drive power cycling.
  // Startup may verify it but must never queue a write and continue.
  bool effective_after_power_cycle = false;
  int64_t storage_off = 0;
};

struct StartupSdoFeedback {
  rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> configured{};
  rosie::rtcore::SnapshotArray<int64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> commanded{};
  rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> readback_valid{};
  rosie::rtcore::SnapshotArray<int64_t, gradient::ipc::v1::GRADIENT_MAX_AXES> readback{};
  rosie::rtcore::SnapshotArray<uint8_t, gradient::ipc::v1::GRADIENT_MAX_AXES> verified{};
};

// Unverified descriptor capacity (whole machine); preserves the existing
// bounded storage. The status publisher copies one feedback block per
// descriptor inside its seqlock read every tick, so a larger array lengthens
// that copy (and its retries) on the servo host.
constexpr uint32_t kMaxStartupSdoDescriptors = 128;

enum class SdoScalarType : uint8_t {
  kNone = 0,
  kU8 = 1,
  kI8 = 2,
  kU16 = 3,
  kI16 = 4,
  kU32 = 5,
  kI32 = 6,
};

// Startup descriptors and mailbox reads name the same six wire scalars.
inline SdoScalarType sdo_scalar_type(StartupSdoValueType type) {
  switch (type) {
    case StartupSdoValueType::kU8: return SdoScalarType::kU8;
    case StartupSdoValueType::kI8: return SdoScalarType::kI8;
    case StartupSdoValueType::kU16: return SdoScalarType::kU16;
    case StartupSdoValueType::kI16: return SdoScalarType::kI16;
    case StartupSdoValueType::kU32: return SdoScalarType::kU32;
    case StartupSdoValueType::kI32: return SdoScalarType::kI32;
    default: return SdoScalarType::kNone;
  }
}

// Convert a numeric scalar to the existing bit-preserving mailbox container.
// A narrower destination must never silently truncate or saturate a Home value.
bool sdo_scalar_bits(int64_t value, SdoScalarType type, int32_t& result) {
  int64_t minimum = 0, maximum = 0;
  switch (type) {
    case SdoScalarType::kU8: maximum = UINT8_MAX; break;
    case SdoScalarType::kI8: minimum = INT8_MIN; maximum = INT8_MAX; break;
    case SdoScalarType::kU16: maximum = UINT16_MAX; break;
    case SdoScalarType::kI16: minimum = INT16_MIN; maximum = INT16_MAX; break;
    case SdoScalarType::kU32: maximum = UINT32_MAX; break;
    case SdoScalarType::kI32: minimum = INT32_MIN; maximum = INT32_MAX; break;
    default: return false;
  }
  if (value < minimum || value > maximum) return false;
  result = static_cast<int32_t>(value > INT32_MAX ? value - (int64_t{1} << 32) : value);
  return true;
}

struct SdoObjectSpec {
  bool valid = false;
  uint16_t index = 0;
  uint8_t subindex = 0;
  SdoScalarType type = SdoScalarType::kNone;
};

// Unverified descriptor capacity; preserves the existing bounded storage.
constexpr uint32_t kMaxAbsoluteFeedbackFields = 16;

struct AbsoluteFeedbackFieldConfig {
  bool valid = false;
  std::string key;
  SdoObjectSpec object{};
};

struct AbsoluteFeedbackConfig {
  bool valid = false;
  uint32_t field_count = 0;
  std::array<AbsoluteFeedbackFieldConfig, kMaxAbsoluteFeedbackFields> fields{};
};

enum class NativeHomeOpKind : uint8_t {
  kNone = 0,
  kSetMode = 1,
  kWriteSdo = 2,
  kControlwordSequence = 3,
  kWaitStatusword = 4,
  kRefreshTruth = 5,
  kRestoreMode = 6,
  kWaitSdo = 7,
  kReleaseServiceOverride = 8,
};

struct NativeHomeOp {
  NativeHomeOpKind kind = NativeHomeOpKind::kNone;
  SdoObjectSpec object{};
  int32_t value_i32 = 0;
  std::array<uint16_t, 8> controlword_values{};
  uint32_t controlword_count = 0;
  uint16_t wait_all_set_mask = 0;
  uint16_t wait_all_clear_mask = 0;
};

struct NativeHomeConfig {
  bool valid = false;
  uint32_t steady_state_mode = 8;
  uint32_t commissioning_mode = 6;
  SdoObjectSpec truth_source{};
  std::vector<NativeHomeOp> transaction{};
};

struct AbsoluteFeedbackFieldSample {
  rosie::rtcore::SnapshotValue<uint8_t> valid = 0;
  rosie::rtcore::SnapshotValue<int32_t> value = 0;
};

struct AbsoluteFeedbackAxis {
  std::array<AbsoluteFeedbackFieldSample, kMaxAbsoluteFeedbackFields> fields{};
  // Phase 1 (2026-04-21): the A6-EC does not cyclically populate
  // U40.20/U40.22 in its TxPDO even though the ESI declares them
  // PDO-mappable (vendor firmware quirk). To still give the canonical-
  // truth shaft-frame gate an apples-to-apples comparison we latch
  // `axis_pos_counts[i]` (the live 0x6064 from the cyclic TxPDO)
  // immediately after each per-axis SDO upload completes and
  // publish it as the "paired" reference. The gate then compares the
  // SDO-sourced multi-turn against this 6064 snapshot (bounded at
  // the mailbox transit, typically 1-5 ms) instead of a 200 ms-skewed
  // live-now 6064.
  //
  // `paired_valid = 0` means the pairing has not yet been established
  // (e.g., before the first SDO poll completes or when the SDO poll is
  // disabled). Python consumers fall back to live-now 6064 in that case.
  rosie::rtcore::SnapshotValue<uint8_t> paired_valid = 0;
  rosie::rtcore::SnapshotValue<int32_t> paired_pos_counts = 0;
  rosie::rtcore::SnapshotValue<uint64_t> paired_sample_time_ns = 0;
};

struct DrivePdoConfig {
  // Unverified opt-in policy: restore the single assignment observed at startup.
  // Must remain off for automatic rescan/preset hand-off; restoration is a
  // separate teardown test and must not undo the physically verified selection.
  bool restore_assignment_on_exit = false;
  std::string label = "custom";
  uint16_t rx_pdo = 0;
  uint16_t tx_pdo = 0;
  uint8_t rx_sync_index = 0;
  uint8_t tx_sync_index = 0;
  std::vector<PdoLayoutEntry> rx_entries;
  std::vector<PdoLayoutEntry> tx_entries;
};

struct Options {
	rosie::rtcore::BusPolicy bus;
  rosie::rtcore::IoConfig cell_io;
  std::string socket_path = "/run/rosie-rt-core/ipc.sock";
  std::string configuration_hash;
  std::string machine_sha256;
  std::string deployment_sha256;
  std::array<uint8_t, 32> machine_digest{};
  // Filesystem group that is allowed to connect to the IPC socket (0660).
  // Default matches the appliance user/group.
  std::string ipc_group = "pi";
  uint64_t cycle_ns = 1000000; // 1 kHz
  // Unqualified fail-closed default; machine configuration supplies its host budget.
  uint64_t max_cycle_lateness_ns = 1000000;
  // Unverified LAN defaults from the native protocol; compiled cell policy may lower or raise them.
  uint64_t control_max_grant_lease_ns = rosie::protocol::fast_control_max_grant_lease_ns;
  uint64_t control_max_jog_input_age_ns = rosie::protocol::fast_control_max_jog_input_age_ns;
  uint32_t num_axes = 6;       // default arm axes for early scaffolding
  uint32_t rt_cpu = 0;
  std::optional<std::string> housekeeping_cpus;
  uint32_t rt_priority = 0;

  // Axis scaling (bring-up defaults; tuned via commissioning).
  // Per-axis lists can be provided via the CLI (comma-separated).
  std::array<AxisConfig, gradient::ipc::v1::GRADIENT_MAX_AXES> axis{};
  std::array<uint16_t, gradient::ipc::v1::GRADIENT_MAX_AXES> slave_position{};

  // Safety: cap commanded motor speed (rpm). 0 disables clamping.
  double max_rpm = 100.0;

  // Unverified EtherCAT bring-up defaults; qualify against drive firmware.
  bool use_dc = true;
  bool disable_output_watchdog = false;
  bool split_domains_per_axis = false;
  bool explicit_pdo_config = false;
  uint32_t wait_before_safeop_ms = 250;
  uint32_t preop_to_safeop_timeout_ms = 5000;
  uint32_t safeop_to_op_timeout_ms = 5000;
  uint32_t startup_passive_ms = 0;
  uint32_t startup_skip_domain_queue_ms = 0;

  // Unverified finite commissioning budget; zero leaves capture disabled.
  uint32_t capture_cycles = 0;
  // Unverified deployment retention defaults: 100000 ms in RAM, 20 dump pairs.
  uint64_t telemetry_retention_ms = rosie::rtcore::kTelemetryDefaultRetentionMs;
  // Unverified count ceiling: 20 x 287204096-byte nine-axis/1 kHz default ring windows
  // is 5744081920 bytes; the byte budget below also limits disk use.
  // Only complete binary/index pairs consume these slots; all indexes consume bytes.
  uint32_t telemetry_dump_count = 20;
  // Unverified index-only ceiling: 64 keeps storm directory scans bounded to
  // 104 files with the default 20 pairs, without consuming their count slots.
  uint32_t telemetry_index_count = 64;
  // Unverified disk budget: 536870912 bytes (512 MiB). Measured default
  // nine-axis/1 kHz dump: 100000*(272+9*288) = 286400000 record bytes,
  // plus header/index metadata; the budget includes retained indexes.
  uint64_t telemetry_dump_bytes = 536870912;
  bool telemetry_dump_on_exit = false;
  // An explicit persistent directory keeps triggered dumps independent of the
  // socket's tmpfs lifetime; an empty value selects runtime/telemetry. Count-only
  // retention at nine axes/1 kHz is bounded above by the 5744081920 ring bytes
  // stated above; the default byte budget further limits retained dumps.
  std::string telemetry_directory;
};

std::string format_pdo_profile_label(uint16_t rx_pdo, uint16_t tx_pdo) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "0x%04x/0x%04x",
                static_cast<unsigned int>(rx_pdo),
                static_cast<unsigned int>(tx_pdo));
  return std::string(buf);
}

// Admit decimal digits within U32 before each conversion step; signs must never
// wrap through strtoul into an apparently valid unsigned configuration value.
bool parse_u32(const char* s, uint32_t* out) {
  if (!s || !*s || !out) {
    return false;
  }
  uint32_t value = 0;
  for (const char* digit = s; *digit; ++digit) {
    if (*digit < '0' || *digit > '9') return false;
    const uint32_t next = static_cast<uint32_t>(*digit - '0');
    if (value > (std::numeric_limits<uint32_t>::max() - next) / 10) return false;
    value = value * 10 + next;
  }
  *out = value;
  return true;
}

bool parse_env_flag(const char* name, bool default_value) {
  if (!name || !*name) {
    return default_value;
  }
  const char* raw = std::getenv(name);
  if (!raw || !*raw) {
    return default_value;
  }
  std::string value(raw);
  for (char& ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  return default_value;
}

bool parse_u64(const char* s, uint64_t* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long long v = std::strtoull(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = static_cast<uint64_t>(v);
  return true;
}

bool parse_double(const char* s, double* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  double v = std::strtod(s, &end);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = v;
  return true;
}

bool parse_i32(const char* s, int* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  long v = std::strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = static_cast<int>(v);
  return true;
}

bool parse_u16_auto(const char* s, uint16_t* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long v = std::strtoul(s, &end, 0);
  if (errno != 0 || end == s || *end != '\0' ||
      v > static_cast<unsigned long>(std::numeric_limits<uint16_t>::max())) {
    return false;
  }
  *out = static_cast<uint16_t>(v);
  return true;
}

bool parse_u32_auto(const char* s, uint32_t* out) {
  if (!s || !*s) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  unsigned long v = std::strtoul(s, &end, 0);
  if (errno != 0 || end == s || *end != '\0' ||
      v > static_cast<unsigned long>(std::numeric_limits<uint32_t>::max())) {
    return false;
  }
  *out = static_cast<uint32_t>(v);
  return true;
}

bool parse_i64_auto(const char* s, int64_t* out) {
  if (!s || !*s || !out) {
    return false;
  }
  char* end = nullptr;
  errno = 0;
  long long v = std::strtoll(s, &end, 0);
  if (errno != 0 || end == s || *end != '\0') {
    return false;
  }
  *out = static_cast<int64_t>(v);
  return true;
}

std::string trim_ascii_ws(const std::string& s) {
  const size_t first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const size_t last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::string ascii_lower_copy(std::string value) {
  for (char& c : value) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return value;
}

std::vector<std::string> split_csv_strict(const std::string& spec) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= spec.size()) {
    size_t comma = spec.find(',', start);
    if (comma == std::string::npos) {
      comma = spec.size();
    }
    std::string tok = trim_ascii_ws(spec.substr(start, comma - start));
    if (tok.empty()) {
      return {}; // invalid (empty token)
    }
    out.push_back(tok);
    if (comma >= spec.size()) {
      break;
    }
    start = comma + 1;
  }
  return out;
}

std::vector<std::string> split_delim_strict(const std::string& spec, char delim) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= spec.size()) {
    size_t pos = spec.find(delim, start);
    if (pos == std::string::npos) {
      pos = spec.size();
    }
    std::string tok = trim_ascii_ws(spec.substr(start, pos - start));
    if (tok.empty()) {
      return {};
    }
    out.push_back(tok);
    if (pos >= spec.size()) {
      break;
    }
    start = pos + 1;
  }
  return out;
}

bool parse_u32_csv_allow_zero(const std::string& spec, std::vector<uint32_t>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    uint32_t v = 0;
    if (!parse_u32_auto(tok.c_str(), &v)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_pdo_layout_spec(const std::string& spec, std::vector<PdoLayoutEntry>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  if (trim_ascii_ws(spec).empty()) {
    return false;
  }
  const auto entries = split_delim_strict(spec, ';');
  if (entries.empty()) {
    return false;
  }
  for (const auto& entry_spec : entries) {
    const auto fields = split_delim_strict(entry_spec, '|');
    if (fields.size() != 4) {
      return false;
    }
    PdoLayoutEntry entry{};
    entry.semantic = trim_ascii_ws(fields[0]);
    uint16_t index = 0;
    uint16_t subindex = 0;
    uint32_t bits = 0;
    if (entry.semantic.empty() ||
        !parse_u16_auto(fields[1].c_str(), &index) ||
        !parse_u16_auto(fields[2].c_str(), &subindex) ||
        !parse_u32_auto(fields[3].c_str(), &bits) ||
        bits == 0 || bits > 255u) {
      return false;
    }
    entry.index = index;
    if (subindex > 255u) return false;
    entry.subindex = static_cast<uint8_t>(subindex & 0xFFu);
    entry.bits = static_cast<uint8_t>(bits);
    out->push_back(entry);
  }
  return !out->empty();
}

bool parse_startup_sdo_config_spec(const std::string& spec,
                                   uint32_t num_axes,
                                   std::vector<StartupSdoConfig>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const std::string trimmed = trim_ascii_ws(spec);
  if (trimmed.empty()) {
    return true;
  }
  const auto descriptors = split_delim_strict(trimmed, ';');
  if (descriptors.empty() ||
      descriptors.size() > static_cast<size_t>(kMaxStartupSdoDescriptors)) {
    return false;
  }

  for (const auto& descriptor_spec : descriptors) {
    const auto fields = split_delim_strict(descriptor_spec, '|');
    if (fields.size() != 5 && fields.size() != 6) {
      return false;
    }
    StartupSdoConfig cfg{};
    cfg.key = trim_ascii_ws(fields[0]);
    std::string type = trim_ascii_ws(fields[1]);
    for (char& c : type) {
      if (c >= 'A' && c <= 'Z') {
        c = static_cast<char>(c - 'A' + 'a');
      }
    }
    cfg.type = startup_sdo_parse_type(type);
    if (cfg.key.empty() || cfg.type == StartupSdoValueType::kNone) {
      return false;
    }
    for (const auto& existing : *out) {
      if (existing.key == cfg.key) {
        return false;
      }
    }
    uint16_t index = 0;
    uint16_t subindex = 0;
    std::vector<int64_t> values{};
    if (!parse_u16_auto(fields[2].c_str(), &index) ||
        !parse_u16_auto(fields[3].c_str(), &subindex) || index == 0) {
      return false;
    }
    for (const auto& token : split_delim_strict(fields[4], ',')) {
      int64_t value = 0;
      if (!startup_sdo_parse_value(cfg.type, trim_ascii_ws(token), &value)) return false;
      values.push_back(value);
    }
    if (values.size() == 1) {
      for (uint32_t i = 0; i < num_axes; ++i) {
        cfg.values[i] = values[0];
      }
    } else if (values.size() == static_cast<size_t>(num_axes)) {
      for (uint32_t i = 0; i < num_axes; ++i) {
        cfg.values[i] = values[i];
      }
    } else {
      return false;
    }
    cfg.valid = true;
    if (fields.size() == 6) {
      if (fields[5] != "power_cycle") return false;
      cfg.effective_after_power_cycle = true;
    }
    cfg.index = index;
    if (subindex > 255u) return false;
    cfg.subindex = static_cast<uint8_t>(subindex & 0xFFu);
    out->push_back(std::move(cfg));
  }
  return !out->empty();
}

bool parse_sdo_scalar_type_token(const std::string& token, SdoScalarType* out) {
  if (!out) {
    return false;
  }
  const std::string lowered = ascii_lower_copy(trim_ascii_ws(token));
  if (lowered == "u8") {
    *out = SdoScalarType::kU8;
    return true;
  }
  if (lowered == "i8") {
    *out = SdoScalarType::kI8;
    return true;
  }
  if (lowered == "u16") {
    *out = SdoScalarType::kU16;
    return true;
  }
  if (lowered == "i16") {
    *out = SdoScalarType::kI16;
    return true;
  }
  if (lowered == "u32") {
    *out = SdoScalarType::kU32;
    return true;
  }
  if (lowered == "i32") {
    *out = SdoScalarType::kI32;
    return true;
  }
  return false;
}

bool parse_absolute_feedback_config_spec(const std::string& spec,
                                        AbsoluteFeedbackConfig* out) {
  if (!out) {
    return false;
  }
  *out = AbsoluteFeedbackConfig{};
  const std::string trimmed = trim_ascii_ws(spec);
  if (trimmed.empty()) {
    return true;
  }
  const auto descriptors = split_delim_strict(trimmed, ';');
  if (descriptors.empty() ||
      descriptors.size() > static_cast<size_t>(kMaxAbsoluteFeedbackFields)) {
    return false;
  }
  AbsoluteFeedbackConfig cfg{};
  for (size_t idx = 0; idx < descriptors.size(); ++idx) {
    const auto fields = split_delim_strict(descriptors[idx], '|');
    if (fields.size() != 4) {
      return false;
    }
    const std::string key = trim_ascii_ws(fields[0]);
    uint16_t index = 0;
    uint16_t subindex = 0;
    SdoScalarType type = SdoScalarType::kNone;
    if (key.empty() ||
        !parse_u16_auto(fields[1].c_str(), &index) ||
        !parse_u16_auto(fields[2].c_str(), &subindex) ||
        !parse_sdo_scalar_type_token(fields[3], &type)) {
      return false;
    }
    for (size_t prev = 0; prev < idx; ++prev) {
      if (cfg.fields[prev].valid && cfg.fields[prev].key == key) {
        return false;
      }
    }
    cfg.fields[idx].valid = true;
    cfg.fields[idx].key = key;
    cfg.fields[idx].object.valid = true;
    cfg.fields[idx].object.index = index;
    cfg.fields[idx].object.subindex = static_cast<uint8_t>(subindex & 0xFFu);
    cfg.fields[idx].object.type = type;
  }
  cfg.field_count = static_cast<uint32_t>(descriptors.size());
  cfg.valid = cfg.field_count > 0;
  *out = cfg;
  return true;
}

bool parse_native_home_config_spec(const std::string& spec, NativeHomeConfig* out) {
  if (!out) {
    return false;
  }
  *out = NativeHomeConfig{};
  const std::string trimmed = trim_ascii_ws(spec);
  if (trimmed.empty()) {
    return true;
  }
  const auto descriptors = split_delim_strict(trimmed, ';');
  if (descriptors.empty()) {
    return false;
  }

  NativeHomeConfig cfg{};
  for (const auto& descriptor : descriptors) {
    const auto fields = split_delim_strict(descriptor, '|');
    if (fields.empty()) {
      return false;
    }
    const std::string entry_kind = ascii_lower_copy(trim_ascii_ws(fields[0]));
    if (entry_kind == "steady_state_mode") {
      uint32_t mode = 0;
      if (fields.size() != 2 || !parse_u32_auto(fields[1].c_str(), &mode)) {
        return false;
      }
      cfg.steady_state_mode = mode;
      continue;
    }
    if (entry_kind == "commissioning_mode") {
      uint32_t mode = 0;
      if (fields.size() != 2 || !parse_u32_auto(fields[1].c_str(), &mode)) {
        return false;
      }
      cfg.commissioning_mode = mode;
      continue;
    }
    if (entry_kind == "truth_source") {
      uint16_t index = 0;
      uint16_t subindex = 0;
      SdoScalarType type = SdoScalarType::kNone;
      if (fields.size() != 4 ||
          !parse_u16_auto(fields[1].c_str(), &index) ||
          !parse_u16_auto(fields[2].c_str(), &subindex) ||
          !parse_sdo_scalar_type_token(fields[3], &type)) {
        return false;
      }
      cfg.truth_source.valid = true;
      cfg.truth_source.index = index;
      if (subindex > 255u) return false;
      cfg.truth_source.subindex = static_cast<uint8_t>(subindex & 0xFFu);
      cfg.truth_source.type = type;
      continue;
    }
    if (entry_kind != "op" || fields.size() < 2) {
      return false;
    }

    const std::string op_name = ascii_lower_copy(trim_ascii_ws(fields[1]));
    NativeHomeOp op{};
    if (op_name == "set_mode") {
      int64_t value = 0;
      if (fields.size() != 3 || !parse_i64_auto(fields[2].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kSetMode;
      op.value_i32 = static_cast<int32_t>(value);
    } else if (op_name == "restore_mode") {
      int64_t value = 0;
      if (fields.size() != 3 || !parse_i64_auto(fields[2].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kRestoreMode;
      op.value_i32 = static_cast<int32_t>(value);
    } else if (op_name == "write_sdo") {
      uint16_t index = 0;
      uint16_t subindex = 0;
      SdoScalarType type = SdoScalarType::kNone;
      int64_t value = 0;
      if (fields.size() != 6 ||
          !parse_u16_auto(fields[2].c_str(), &index) ||
          !parse_u16_auto(fields[3].c_str(), &subindex) ||
          !parse_sdo_scalar_type_token(fields[4], &type) ||
          !parse_i64_auto(fields[5].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kWriteSdo;
      op.object.valid = true;
      op.object.index = index;
      if (subindex > 255u) return false;
      op.object.subindex = static_cast<uint8_t>(subindex & 0xFFu);
      op.object.type = type;
      if (!sdo_scalar_bits(value, type, op.value_i32)) return false;
    } else if (op_name == "controlword_sequence") {
      std::vector<uint32_t> values{};
      if (fields.size() != 3 || !parse_u32_csv_allow_zero(fields[2], &values) || values.empty() ||
          values.size() > op.controlword_values.size()) {
        return false;
      }
      op.kind = NativeHomeOpKind::kControlwordSequence;
      op.controlword_count = static_cast<uint32_t>(values.size());
      for (size_t i = 0; i < values.size(); ++i) {
        if (values[i] > static_cast<uint32_t>(std::numeric_limits<uint16_t>::max())) {
          return false;
        }
        op.controlword_values[i] = static_cast<uint16_t>(values[i] & 0xFFFFu);
      }
    } else if (op_name == "wait_statusword") {
      uint16_t all_set_mask = 0;
      uint16_t all_clear_mask = 0;
      if (fields.size() != 4 ||
          !parse_u16_auto(fields[2].c_str(), &all_set_mask) ||
          !parse_u16_auto(fields[3].c_str(), &all_clear_mask)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kWaitStatusword;
      op.wait_all_set_mask = all_set_mask;
      op.wait_all_clear_mask = all_clear_mask;
    } else if (op_name == "wait_sdo") {
      uint16_t index = 0;
      uint16_t subindex = 0;
      SdoScalarType type = SdoScalarType::kNone;
      int64_t value = 0;
      if (fields.size() != 6 ||
          !parse_u16_auto(fields[2].c_str(), &index) ||
          !parse_u16_auto(fields[3].c_str(), &subindex) ||
          !parse_sdo_scalar_type_token(fields[4], &type) ||
          !parse_i64_auto(fields[5].c_str(), &value)) {
        return false;
      }
      op.kind = NativeHomeOpKind::kWaitSdo;
      op.object.valid = true;
      op.object.index = index;
      if (subindex > 255u) return false;
      op.object.subindex = static_cast<uint8_t>(subindex & 0xFFu);
      op.object.type = type;
      op.value_i32 = static_cast<int32_t>(value);
    } else if (op_name == "release_service_override") {
      if (fields.size() != 2) {
        return false;
      }
      op.kind = NativeHomeOpKind::kReleaseServiceOverride;
    } else if (op_name == "refresh_truth") {
      if (fields.size() != 2) {
        return false;
      }
      op.kind = NativeHomeOpKind::kRefreshTruth;
    } else {
      return false;
    }
    cfg.transaction.push_back(op);
  }

  if (!cfg.truth_source.valid || cfg.transaction.empty()) {
    return false;
  }
  cfg.valid = true;
  *out = cfg;
  return true;
}

bool parse_axis_type_token(const std::string& token, uint8_t* out) {
  if (!out) {
    return false;
  }
  std::string t = token;
  for (char& c : t) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  if (t == "r" || t == "rot" || t == "rotary") {
    *out = gradient::ipc::v1::AXIS_TYPE_ROTARY;
    return true;
  }
  if (t == "l" || t == "lin" || t == "linear") {
    *out = gradient::ipc::v1::AXIS_TYPE_LINEAR;
    return true;
  }
  return false;
}

bool parse_u32_csv(const std::string& spec, std::vector<uint32_t>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  // Decimal U32 lists preserve exact token bytes; the general CSV splitter
  // trims whitespace for other callers and would hide malformed cap tokens.
  if (spec.find_first_not_of("0123456789,") != std::string::npos) return false;
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    uint32_t v = 0;
    if (!parse_u32(tok.c_str(), &v) || v == 0) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_double_csv(const std::string& spec, std::vector<double>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    double v = 0.0;
    if (!parse_double(tok.c_str(), &v)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_sign_csv(const std::string& spec, std::vector<int>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    int v = 0;
    if (!parse_i32(tok.c_str(), &v) || (v != 1 && v != -1)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

bool parse_axis_type_csv(const std::string& spec, std::vector<uint8_t>* out) {
  if (!out) {
    return false;
  }
  out->clear();
  const auto toks = split_csv_strict(spec);
  if (toks.empty()) {
    return false;
  }
  for (const auto& tok : toks) {
    uint8_t v = gradient::ipc::v1::AXIS_TYPE_UNKNOWN;
    if (!parse_axis_type_token(tok, &v)) {
      return false;
    }
    out->push_back(v);
  }
  return true;
}

template <typename T, typename Setter>
bool apply_per_axis(const std::vector<T>& vals, uint32_t num_axes, Setter setter) {
  if (vals.empty()) {
    return true;
  }
  if (vals.size() == 1) {
    for (uint32_t i = 0; i < num_axes; ++i) {
      setter(i, vals[0]);
    }
    return true;
  }
  if (vals.size() != static_cast<size_t>(num_axes)) {
    return false;
  }
  for (uint32_t i = 0; i < num_axes; ++i) {
    setter(i, vals[i]);
  }
  return true;
}

bool finalize_axis_config(Options* opt,
                          const std::string& counts_per_rev_spec,
                          const std::string& gear_ratio_spec,
                          const std::string& sign_spec,
                          const std::string& axis_type_spec,
                          const std::string& lead_m_per_rev_spec) {
  if (!opt) {
    return false;
  }

  // Parse optional per-axis specs (comma-separated). A single value broadcasts to all axes.
  std::vector<uint32_t> cpr{};
  if (!counts_per_rev_spec.empty()) {
    if (!parse_u32_csv(counts_per_rev_spec, &cpr)) {
      logf("ERROR: invalid --counts-per-rev (expected N or N1,N2,...)");
      return false;
    }
    if (!apply_per_axis<uint32_t>(cpr, opt->num_axes, [&](uint32_t i, uint32_t v) {
          opt->axis[i].counts_per_rev = v;
        })) {
      logf("ERROR: --counts-per-rev list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<double> gr{};
  if (!gear_ratio_spec.empty()) {
    if (!parse_double_csv(gear_ratio_spec, &gr)) {
      logf("ERROR: invalid --gear-ratio (expected R or R1,R2,...)");
      return false;
    }
    if (!apply_per_axis<double>(gr, opt->num_axes, [&](uint32_t i, double v) {
          if (v <= 0.0) {
            v = 0.0;
          }
          opt->axis[i].gear_ratio = v;
        })) {
      logf("ERROR: --gear-ratio list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<int> sgn{};
  if (!sign_spec.empty()) {
    if (!parse_sign_csv(sign_spec, &sgn)) {
      logf("ERROR: invalid --sign (expected +1/-1 or S1,S2,...)");
      return false;
    }
    if (!apply_per_axis<int>(sgn, opt->num_axes, [&](uint32_t i, int v) {
          opt->axis[i].sign = v;
        })) {
      logf("ERROR: --sign list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<uint8_t> at{};
  if (!axis_type_spec.empty()) {
    if (!parse_axis_type_csv(axis_type_spec, &at)) {
      logf("ERROR: invalid --axis-type (expected rotary|linear, optionally comma-separated)");
      return false;
    }
    if (!apply_per_axis<uint8_t>(at, opt->num_axes, [&](uint32_t i, uint8_t v) {
          opt->axis[i].axis_type = v;
        })) {
      logf("ERROR: --axis-type list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  std::vector<double> lead{};
  if (!lead_m_per_rev_spec.empty()) {
    if (!parse_double_csv(lead_m_per_rev_spec, &lead)) {
      logf("ERROR: invalid --lead-m-per-rev (expected M or M1,M2,...)");
      return false;
    }
    if (!apply_per_axis<double>(lead, opt->num_axes, [&](uint32_t i, double v) {
          opt->axis[i].lead_m_per_rev = v;
        })) {
      logf("ERROR: --lead-m-per-rev list length must be 1 or --num-axes (%u)", opt->num_axes);
      return false;
    }
  }

  // Validate and derive counts_per_unit and safety clamps.
  for (uint32_t i = 0; i < opt->num_axes; ++i) {
    AxisConfig& a = opt->axis[i];
    if (a.counts_per_rev == 0) {
      logf("ERROR: axis%u counts_per_rev must be > 0", i);
      return false;
    }
    if (a.gear_ratio <= 0.0) {
      logf("ERROR: axis%u gear_ratio must be > 0", i);
      return false;
    }
    if (a.sign != 1 && a.sign != -1) {
      logf("ERROR: axis%u sign must be +1 or -1", i);
      return false;
    }

    if (a.axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY) {
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / kCompletionWrapTwoPi; // counts/rad
    } else if (a.axis_type == gradient::ipc::v1::AXIS_TYPE_LINEAR) {
      if (a.lead_m_per_rev <= 0.0) {
        logf("ERROR: axis%u is linear; provide --lead-m-per-rev (m/rev) for that axis", i);
        return false;
      }
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / a.lead_m_per_rev; // counts/m
    } else {
      logf("ERROR: axis%u has unknown axis_type (use --axis-type rotary|linear)", i);
      return false;
    }
    if (!(a.counts_per_unit > 0.0)) {
      logf("ERROR: axis%u counts_per_unit invalid (check scaling inputs)", i);
      return false;
    }

    // Safety clamp derived from motor rpm limit.
    if (opt->max_rpm <= 0.0) {
      a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
    } else {
      const double max_counts_per_s =
          (opt->max_rpm / 60.0) * static_cast<double>(a.counts_per_rev);
      const long long v = std::llround(max_counts_per_s);
      if (v <= 0) {
        a.max_profile_vel_counts_per_s = 1u;
      } else if (v > static_cast<long long>(std::numeric_limits<uint32_t>::max())) {
        a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
      } else {
        a.max_profile_vel_counts_per_s = static_cast<uint32_t>(v);
      }
    }
  }

  // Populate unused axis slots with sane derived values for tooling.
  for (uint32_t i = opt->num_axes; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
    AxisConfig& a = opt->axis[i];
    if (a.axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY) {
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / kCompletionWrapTwoPi; // counts/rad
    } else {
      // Keep defaults for unused axes.
      a.counts_per_unit =
          (static_cast<double>(a.counts_per_rev) * a.gear_ratio) / kCompletionWrapTwoPi; // counts/rad
    }
    if (opt->max_rpm <= 0.0) {
      a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
    } else {
      const double max_counts_per_s =
          (opt->max_rpm / 60.0) * static_cast<double>(a.counts_per_rev);
      const long long v = std::llround(max_counts_per_s);
      if (v <= 0) {
        a.max_profile_vel_counts_per_s = 1u;
      } else if (v > static_cast<long long>(std::numeric_limits<uint32_t>::max())) {
        a.max_profile_vel_counts_per_s = std::numeric_limits<uint32_t>::max();
      } else {
        a.max_profile_vel_counts_per_s = static_cast<uint32_t>(v);
      }
    }
  }

  return true;
}

bool finalize_slave_positions(Options* opt, const std::string& slave_positions_spec) {
  if (!opt) {
    return false;
  }

  for (uint32_t i = 0; i < gradient::ipc::v1::GRADIENT_MAX_AXES; ++i) {
    opt->slave_position[i] = static_cast<uint16_t>(i);
  }

  if (slave_positions_spec.empty()) {
    return true;
  }

  const auto toks = split_csv_strict(slave_positions_spec);
  if (toks.size() != static_cast<size_t>(opt->num_axes)) {
    logf("ERROR: --slave-positions list length must match --num-axes (%u)", opt->num_axes);
    return false;
  }

  for (uint32_t i = 0; i < opt->num_axes; ++i) {
    uint32_t pos = 0;
    if (!parse_u32(toks[i].c_str(), &pos) ||
        pos > static_cast<uint32_t>(std::numeric_limits<uint16_t>::max())) {
      logf("ERROR: invalid slave position '%s' in --slave-positions", toks[i].c_str());
      return false;
    }
    opt->slave_position[i] = static_cast<uint16_t>(pos);
  }

  for (uint32_t i = 0; i < opt->num_axes; ++i) {
    for (uint32_t j = i + 1; j < opt->num_axes; ++j) {
      if (opt->slave_position[i] == opt->slave_position[j]) {
        logf("ERROR: duplicate slave position %u in --slave-positions",
             static_cast<unsigned int>(opt->slave_position[i]));
        return false;
      }
    }
  }

  return true;
}

#include "axis_profiles.hpp"

// The ordinary Arm brake-opening stage is reviewed only for this A6-EC profile.
inline bool a6_brake_opening_profile_supported(const AxisDriveProfile& profile) {
  return profile.vendor_id == 4194304 && profile.product_code == 1813 &&
      profile.brake.present && profile.brake.hold_displacement_tolerance_counts >= 1 &&
      profile.home.valid && profile.home.steady_state_mode == 8 && profile.home.commissioning_mode == 6 &&
      std::any_of(profile.home.transaction.begin(), profile.home.transaction.end(), [](const NativeHomeOp& step) {
        return step.kind == NativeHomeOpKind::kWriteSdo && step.object.valid &&
            step.object.index == 0x6098 && step.object.subindex == 0 && step.value_i32 == 35;
      });
}
#include "description.hpp"

// Report every parsed flag and its unverified startup default, including
// simulation-only injections, so operators do not depend on hidden switches.
void print_usage(const char* argv0) {
  std::fprintf(
      stderr,
      "Usage: %s [options]\n\n"
      "Options (unverified defaults):\n"
      "  --bus-unknown-slaves POLICY     default: refuse; refuse or hold non-axis slaves in PREOP\n"
      "  --bus-unused-slaves LIST        default: empty; position|vendor|product|revision-or-dash separated by semicolons\n"
      "  --help, -h                     default: off; print usage and exit\n"
      "  --version                      default: off; print build version and exit\n"
      "  --axis-profiles PATH            default: unset (required); compiled profile file\n"
      "  --cell-io TEXT                  default: unset; compiled terminal policy (P6-05, torch unqualified)\n"
      "  --configuration-hash SHA256     default: unset (required); 64 hex digits\n"
      "  --machine-sha256 SHA256         default: recompute; verify 64 hex digits when supplied\n"
      "  --machine-identity PATH         default: adjacent machine.identity if digest supplied and file exists\n"
      "  --deployment-sha256 SHA256      default: recompute from host options; 64 hex digits\n"
      "  --validate-config              default: off; validate and exit\n"
      "  --socket-path PATH             default: /run/rosie-rt-core/ipc.sock\n"
      "  --ipc-group NAME               default: pi; socket group name\n"
      "  --adapter-user NAME            default: unset; restrict admission to resolved user UID\n"
      "  --rt-cpu N                     default: 0; CPU index\n"
      "  --rt-priority N                default: 0 (invalid; required 1..99); FIFO priority\n"
      "  --housekeeping-cpus LIST        default: all remaining allowed CPUs excluding RT CPU; CPU indices\n"
      "  --cycle-ns NS                  default: 1000000 ns; range 250000..10000000 ns\n"
      "  --control-max-grant-lease-ns NS default: 500000000 ns; unverified ceiling 10000000000 ns; whole ms\n"
      "  --control-max-jog-input-age-ns NS default: 250000000 ns; unverified ceiling 2000000000 ns\n"
      "  --max-cycle-lateness-ns NS      default: 1000000 ns; range 1..249999999 ns\n"
      "  --num-axes N                   default: 6 axes; range 1..16 axes\n"
      "  --counts-per-rev N[,N..]       default: 131072 counts/rev\n"
      "  --gear-ratio R[,R..]           default: 1.0 wire revs/axis rev\n"
      "  --sign S[,S..]                 default: +1; mechanical direction (+1 or -1)\n"
      "  --axis-type T[,T..]            default: rotary; rotary (rad) or linear (m)\n"
      "  --lead-m-per-rev M[,M..]        default: 0 m/rev; positive for linear axes\n"
      "  --max-rpm RPM                  default: 100 rev/min; 0 disables clamping\n"
      "  --drive-speed-limit-counts-per-s CSV  default: legacy --max-rpm derivation; explicit positive U32 counts/s per axis overrides it\n"
      "  --slave-positions P[,P..]       default: 0,1,2,...; EtherCAT slave indices\n"
      "  --no-dc                        default: off; distributed clocks enabled\n"
      "  --disable-output-watchdog      default: off; output watchdog enabled\n"
      "  --split-domains-per-axis       default: off; one shared process-data domain\n"
      "  --queue-split-domains-round-robin default: off; unsupported, rejected (fresh full-group exchange required)\n"
      "  --explicit-pdo-config          default: off; use drive PDO mapping\n"
      "  --wait-before-safeop-ms MS      default: 250 ms; 0 leaves master default\n"
      "  --preop-safeop-timeout-ms MS    default: 5000 ms; 0 leaves master default\n"
      "  --safeop-op-timeout-ms MS       default: 5000 ms; 0 leaves master default\n"
      "  --startup-passive-ms MS         default: 0 ms; disabled\n"
      "  --startup-skip-domain-queue-ms MS default: 0 ms; disabled\n"
      "  --capture-cycles N             default: 0 (disabled); 1..65536 cycle records to runtime/cycle-capture.jsonl\n"
      "  --telemetry-retention-ms MS     default: 100000; 100..3600000 ms, at most 2 GiB\n"
      "  --telemetry-dump-count N        default: 20; 1..100 complete pairs (5744081920 bytes count-only at default 9 axes/1 kHz); byte budget also applies\n"
      "  --telemetry-dump-bytes N        default: 536870912 (unverified 512 MiB; one default 9-axis/1 kHz dump); 529680..9007199254740991; minimum reserves two headers and one 16-axis record; oldest deleted first\n"
      "  --telemetry-dump-on-exit        default: off; dump RAM window at shutdown\n"
      "  --telemetry-directory PATH     default: runtime/telemetry; absolute dump directory, 5744081920 bytes count-only / 512 MiB default byte budget\n",
      argv0);
#if ROSIE_RT_SIMULATION
  std::fprintf(stderr,
      "Simulation injections (unverified):\n"
      "  --sim-dio-inputs N                  default: 0; initial raw DI byte, 0..255\n"
      "  --sim-dio-edge-inputs N             default: 0; injected raw DI byte, 0..255\n"
      "  --sim-dio-edge-cycle N              default: 0 (disabled); receive-call index for input edge\n"
      "  --sim-dio-readback-lag-cycles N      default: 0 cycles; independent feedback settling\n"
      "  --sim-dio-loss-cycle N              default: 0 (disabled); first lost terminal receive-call index\n"
      "  --sim-dio-stuck-readback             default: off; hold independent feedback OFF\n"
      "  --sim-velocity-actual-counts-per-s N default: 0 counts/s; signed diagnostic PDO fixture\n"
      "  --sim-following-error-counts N      default: 0 counts; signed diagnostic PDO fixture\n"
      "  --sim-enable-delay-cycles N          default: 0 cycles\n"
      "  --sim-send-fail-from-call N          default: 0 (disabled); first failing send call index\n"
      "  --sim-send-fail-for-calls N          default: 0 calls (persistent once triggered)\n"
      "  --sim-receive-fail-from-call N       default: 0 (disabled); first failing receive call index\n"
      "  --sim-receive-fail-for-calls N       default: 0 calls (persistent once triggered)\n"
      "  --sim-queue-fail-from-call N         default: 0 (disabled); first failing queue call index\n"
      "  --sim-queue-fail-for-calls N         default: 0 calls (persistent once triggered)\n"
      "  --sim-process-fail-from-call N       default: 0 (disabled); first failing process call index\n"
      "  --sim-process-fail-for-calls N       default: 0 calls (persistent once triggered)\n"
      "  --sim-dc-fail-from-call N            default: 0 (disabled); first failing DC call index\n"
      "  --sim-dc-fail-for-calls N            default: 0 calls (persistent once triggered)\n"
      "  --sim-wkc-incomplete-from-call N     default: 0 (disabled); first incomplete receive call index\n"
      "  --sim-wkc-incomplete-for-calls N     default: 0 calls (persistent once triggered)\n"
      "  --sim-continuous-invalid-from-call N default: 0 (disabled); first invalid receive call index\n"
      "  --sim-continuous-invalid-for-calls N default: 0 calls (persistent once triggered)\n"
      "  --sim-continuous-invalid-position N  default: 0; EtherCAT slave index\n"
      "  --sim-disabled-displacement-counts N default: 0 counts\n"
      "  --sim-initial-position-counts N      default: 0 counts; slave 0 before startup\n"
      "  --sim-raw-multi-turn-origin-counts N default: 0 counts; arbitrary command minus raw encoder origin\n"
      "  --sim-home-completion-jitter-counts N  default: 0 counts; HM35 reference jitter\n"
      "  --sim-home-encoder-jitter-counts N   default: 0 counts; HM35 raw encoder jitter\n"
      "  --sim-mapping-upload-failures N      default: 0 uploads; slave 0 assignment timeout\n"
      "  --sim-encoder-fault-state N          default: 0; simulated A6 U41.05 (0x2041:06) value\n"
      "  --sim-multi-turn-overflow N          default: 0; simulated A6 C10.15 (0x2010:16) value\n"
      "  --sim-encoder-alarm-unreadable N     default: 0; nonzero aborts both A6 encoder-alarm uploads\n"
      "  --sim-pdo-mismatch                  default: off; slave 0 mapped descriptor differs\n"
      "  --sim-effective-startup-mismatch    default: off; power-cycle startup readback differs\n"
      "  --sim-sii-revision N                default: unset; override SII without changing CoE revision\n"
      "  --sim-extra-slave SPEC              default: none; repeat position|vendor|product|revision for extra PREOP devices\n"
      "  --sim-sample-zero-crossing-low-read N default: 0 (disabled); cross zero after this low upload\n"
      "  --sim-sample-unavailable-from-upload N default: 0; fail sampled uploads starting at per-axis call N\n"
      "  --sim-sample-unavailable-for-uploads N default: 0; persistent absence, otherwise N failed calls\n"
      "  --sim-sample-contradiction-from-upload N default: 0; offset encoder from per-axis upload N\n"
      "  --sim-sample-contradiction-counts N    default: 0 counts; signed sampled encoder offset\n"
      "  --sim-sampled-upload-delay-ns NS     default: 0 ns; delay each rotation encoder upload\n"
      "  --sim-disabled-displacement-position N default: 0; EtherCAT slave index\n"
      "  --sim-brake-sag-counts N             default: 0 counts\n"
      "  --sim-feedback-offset-counts N       default: 0 counts\n"
      "  --sim-feedback-offset-position N     default: 0; EtherCAT slave index\n"
      "  --sim-mode-mismatch-after-enabled-ns NS default: 0 ns (disabled); slave 0 changes CSP to mode 9\n"
      "  --sim-statusword-fault-after-enabled-ns NS default: 0 ns (disabled); slave 0 sets statusword bit 3\n"
      "  --sim-sdo-block-ns NS                default: 0 ns; block slave 0 Home offset download\n"
      "  --sim-sdo-block-ms MS                default: 0 ms; block slave 0 Home offset download\n"
      "  --sim-mailbox-cycle-sends N          default: 0 (off); each runtime SDO returns only after N more sends\n");
#endif
}

// Parsing preserves the daemon's early-exit status and usage decision so
// callers cannot start threads after invalid configuration or a help request.
struct OptionsParseResult {
  std::optional<int> exit_code;
  bool show_usage = false;
  std::optional<uid_t> adapter_uid;
#if ROSIE_RT_SIMULATION
  SimFaultControls sim_faults{};
#endif
  std::vector<StartupSdoConfig> startup_sdos{};
  std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_profiles{};
  bool have_native_home = false;
  bool have_absolute_feedback = false;

  OptionsParseResult() = default;
  OptionsParseResult(int code, bool usage = false)
      : exit_code(code), show_usage(usage) {}
};

// Validate all startup options before runtime composition. No partially parsed
// profile is used when an error, validation-only request or help ends startup.
OptionsParseResult parse_options(int argc, char** argv, Options* options) {
  Options& opt = *options;
  OptionsParseResult result;
  auto& adapter_uid = result.adapter_uid;
#if ROSIE_RT_SIMULATION
  auto& sim_faults = result.sim_faults;
#endif
  std::string axis_profiles_path;
  std::string machine_identity_path;
  bool validate_config_only = false;
  std::string counts_per_rev_spec;
  std::string gear_ratio_spec;
  std::string sign_spec;
  std::string axis_type_spec;
  std::string lead_m_per_rev_spec;
  std::string slave_positions_spec;
  std::optional<std::vector<uint32_t>> drive_speed_limits;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
#if ROSIE_RT_SIMULATION
    uint64_t* sim_counter = nullptr;
    if (arg == "--sim-extra-slave" && i + 1 < argc) {
      const auto fields = split_delim_strict(argv[++i], '|');
      SimExtraSlave slave;
      uint32_t position = 0;
      if (fields.size() != 4 || !parse_u32(fields[0].c_str(), &position) || position > UINT16_MAX ||
          !parse_u32(fields[1].c_str(), &slave.vendor_id) || !slave.vendor_id ||
          !parse_u32(fields[2].c_str(), &slave.product_code) || !slave.product_code ||
          !parse_u32(fields[3].c_str(), &slave.revision) || sim_faults.extra_slaves.size() == rosie::rtcore::kMaxUnusedSlaves ||
          std::any_of(sim_faults.extra_slaves.begin(), sim_faults.extra_slaves.end(), [&](const auto& s) { return s.position == position; })) return 2;
      slave.position = static_cast<uint16_t>(position);
      sim_faults.extra_slaves.push_back(slave);
      continue;
    }
    if (arg == "--sim-enable-delay-cycles") sim_counter = &sim_faults.enable_delay_cycles;
    else if (arg == "--sim-send-fail-from-call") sim_counter = &sim_faults.send_fail_from_call;
    else if (arg == "--sim-send-fail-for-calls") sim_counter = &sim_faults.send_fail_for_calls;
    else if (arg == "--sim-receive-fail-from-call") sim_counter = &sim_faults.receive_fail_from_call;
    else if (arg == "--sim-receive-fail-for-calls") sim_counter = &sim_faults.receive_fail_for_calls;
    else if (arg == "--sim-queue-fail-from-call") sim_counter = &sim_faults.queue_fail_from_call;
    else if (arg == "--sim-queue-fail-for-calls") sim_counter = &sim_faults.queue_fail_for_calls;
    else if (arg == "--sim-process-fail-from-call") sim_counter = &sim_faults.process_fail_from_call;
    else if (arg == "--sim-process-fail-for-calls") sim_counter = &sim_faults.process_fail_for_calls;
    else if (arg == "--sim-dc-fail-from-call") sim_counter = &sim_faults.dc_fail_from_call;
    else if (arg == "--sim-dc-fail-for-calls") sim_counter = &sim_faults.dc_fail_for_calls;
    else if (arg == "--sim-wkc-incomplete-from-call") sim_counter = &sim_faults.wkc_incomplete_from_call;
    else if (arg == "--sim-wkc-incomplete-for-calls") sim_counter = &sim_faults.wkc_incomplete_for_calls;
    else if (arg == "--sim-continuous-invalid-from-call") sim_counter = &sim_faults.continuous_invalid_from_call;
    else if (arg == "--sim-continuous-invalid-for-calls") sim_counter = &sim_faults.continuous_invalid_for_calls;
    else if (arg == "--sim-sdo-block-ms") sim_counter = &sim_faults.sdo_block_ms;
    else if (arg == "--sim-mailbox-cycle-sends") sim_counter = &sim_faults.mailbox_cycle_sends;
    else if (arg == "--sim-sample-zero-crossing-low-read") sim_counter = &sim_faults.sample_zero_crossing_low_read;
    else if (arg == "--sim-sampled-upload-delay-ns") sim_counter = &sim_faults.sample_upload_delay_ns;
    else if (arg == "--sim-sample-unavailable-from-upload") sim_counter = &sim_faults.sample_unavailable_from_upload;
    else if (arg == "--sim-sample-unavailable-for-uploads") sim_counter = &sim_faults.sample_unavailable_for_uploads;
    else if (arg == "--sim-sample-contradiction-from-upload") sim_counter = &sim_faults.sample_contradiction_from_upload;
    if (sim_counter) {
      if (i + 1 >= argc || !parse_u64(argv[++i], sim_counter)) return 2;
      continue;
    }
    if (arg == "--sim-disabled-displacement-counts") {
      int value = 0;
      if (i + 1 >= argc || !parse_i32(argv[++i], &value)) return 2;
      sim_faults.disabled_displacement_counts = value;
      continue;
    }
    if (arg == "--sim-velocity-actual-counts-per-s" || arg == "--sim-following-error-counts") {
      auto* value = arg == "--sim-velocity-actual-counts-per-s" ? &sim_faults.velocity_actual_counts_per_s : &sim_faults.following_error_counts;
      if (i + 1 >= argc || !parse_i32(argv[++i], value)) return 2;
      continue;
    }
    if (arg == "--sim-initial-position-counts") {
      if (i + 1 >= argc || !parse_i32(argv[++i], &sim_faults.initial_position_counts)) return 2;
      continue;
    }
    if (arg == "--sim-raw-multi-turn-origin-counts" || arg == "--sim-home-completion-jitter-counts" ||
        arg == "--sim-home-encoder-jitter-counts") {
      auto* value = arg == "--sim-raw-multi-turn-origin-counts" ? &sim_faults.raw_multi_turn_origin_counts :
          arg == "--sim-home-completion-jitter-counts" ? &sim_faults.home_completion_jitter_counts : &sim_faults.home_encoder_jitter_counts;
      if (i + 1 >= argc || !parse_i32(argv[++i], value)) return 2;
      continue;
    }
    if (arg == "--sim-sample-contradiction-counts") {
      if (i + 1 >= argc || !parse_i32(argv[++i], &sim_faults.sample_contradiction_counts)) return 2;
      continue;
    }
    if (arg == "--sim-mapping-upload-failures") {
      if (i + 1 >= argc || !parse_u32(argv[++i], &sim_faults.mapping_upload_failures)) return 2;
      continue;
    }
    if (arg == "--sim-encoder-fault-state" || arg == "--sim-multi-turn-overflow" ||
        arg == "--sim-encoder-alarm-unreadable") {
      uint32_t value = 0;
      if (i + 1 >= argc || !parse_u32(argv[++i], &value) || value > UINT16_MAX) return 2;
      (arg == "--sim-encoder-fault-state" ? sim_faults.encoder_fault_state :
          arg == "--sim-multi-turn-overflow" ? sim_faults.multi_turn_overflow :
          sim_faults.encoder_alarm_unreadable) = static_cast<uint16_t>(value);
      continue;
    }
    if (arg == "--sim-dio-inputs" || arg == "--sim-dio-edge-inputs") {
      uint32_t value=0; if(i+1>=argc || !parse_u32(argv[++i],&value) || value>UINT16_MAX) return 2;
      (arg=="--sim-dio-inputs" ? sim_faults.dio_inputs : sim_faults.dio_edge_inputs)=value; continue;
    }
    if (arg == "--sim-dio-readback-lag-cycles" || arg == "--sim-dio-edge-cycle" || arg == "--sim-dio-loss-cycle") {
      uint64_t value=0; if(i+1>=argc || !parse_u64(argv[++i],&value)) return 2;
      if(arg=="--sim-dio-readback-lag-cycles") sim_faults.dio_readback_lag_cycles=value;
      else if(arg=="--sim-dio-edge-cycle") sim_faults.dio_edge_cycle=value;
      else sim_faults.dio_loss_cycle=value;
      continue;
    }
    if(arg=="--sim-dio-stuck-readback") { sim_faults.dio_stuck_readback=true; continue; }
    if (arg == "--sim-pdo-mismatch") {
      sim_faults.pdo_mismatch = true;
      continue;
    }
    if (arg == "--sim-effective-startup-mismatch") {
      sim_faults.effective_startup_mismatch = true;
      continue;
    }
    if (arg == "--sim-sii-revision") {
      if (i + 1 >= argc || !parse_u32(argv[++i], &sim_faults.sii_revision) || !sim_faults.sii_revision) return 2;
      continue;
    }
    if (arg == "--sim-continuous-invalid-position") {
      uint32_t value = 0;
      if (i + 1 >= argc || !parse_u32(argv[++i], &value) || value > UINT16_MAX) return 2;
      sim_faults.continuous_invalid_position = static_cast<uint16_t>(value);
      continue;
    }
    if (arg == "--sim-brake-sag-counts") {
      int value = 0;
      if (i + 1 >= argc || !parse_i32(argv[++i], &value)) return 2;
      sim_faults.brake_sag_counts = value;
      continue;
    }
    if (arg == "--sim-feedback-offset-counts") {
      int value = 0;
      if (i + 1 >= argc || !parse_i32(argv[++i], &value)) return 2;
      sim_faults.feedback_offset_counts = value;
      continue;
    }
    if (arg == "--sim-feedback-offset-position") {
      uint32_t value = 0;
      if (i + 1 >= argc || !parse_u32(argv[++i], &value) || value > UINT16_MAX) return 2;
      sim_faults.feedback_offset_position = static_cast<uint16_t>(value);
      continue;
    }
    if (arg == "--sim-disabled-displacement-position") {
      uint32_t value = 0;
      if (i + 1 >= argc || !parse_u32(argv[++i], &value) || value > UINT16_MAX) return 2;
      sim_faults.disabled_displacement_position = static_cast<uint16_t>(value);
      continue;
    }
    uint64_t* sim_interval = nullptr;
    if (arg == "--sim-mode-mismatch-after-enabled-ns") sim_interval = &sim_faults.mode_mismatch_after_enabled_ns;
    else if (arg == "--sim-statusword-fault-after-enabled-ns") sim_interval = &sim_faults.statusword_fault_after_enabled_ns;
    else if (arg == "--sim-sdo-block-ns") sim_interval = &sim_faults.sdo_block_ns;
    if (sim_interval) {
      if (i + 1 >= argc || !parse_u64(argv[++i], sim_interval)) return 2;
      continue;
    }
#endif
    if (arg == "--help" || arg == "-h") {
      return {0, true};
    }
    if (arg == "--housekeeping-cpus") {
      if (i + 1 >= argc) { logf("ERROR: --housekeeping-cpus requires a CPU list"); return 2; }
      opt.housekeeping_cpus = argv[++i];
      continue;
    }
    if ((arg == "--rt-cpu" || arg == "--rt-priority") && i + 1 < argc) {
      uint32_t value = 0;
      if (!parse_u32(argv[++i], &value)) return 2;
      if (arg == "--rt-cpu") opt.rt_cpu = value; else opt.rt_priority = value;
      continue;
    }
    if (arg == "--cell-io" && i + 1 < argc) {
      try { opt.cell_io = rosie::rtcore::parse_cell_io(argv[++i]); }
      catch (const std::exception& error) { logf("ERROR: %s", error.what()); return 2; }
      continue;
    }
    if (arg == "--axis-profiles" && i + 1 < argc) { axis_profiles_path = argv[++i]; continue; }
    if (arg == "--configuration-hash" && i + 1 < argc) { opt.configuration_hash = argv[++i]; continue; }
    if (arg == "--machine-sha256" && i + 1 < argc) { opt.machine_sha256 = argv[++i]; continue; }
    if (arg == "--machine-identity" && i + 1 < argc) { machine_identity_path = argv[++i]; continue; }
    if (arg == "--deployment-sha256" && i + 1 < argc) { opt.deployment_sha256 = argv[++i]; continue; }
    if (arg == "--validate-config") { validate_config_only = true; continue; }
    if (arg == "--socket-path" && i + 1 < argc) {
      opt.socket_path = argv[++i];
      continue;
    }
    if (arg == "--cycle-ns" && i + 1 < argc) {
      if (!parse_u64(argv[++i], &opt.cycle_ns) || opt.cycle_ns < 250000 || opt.cycle_ns > 10000000) {
        logf("ERROR: invalid --cycle-ns");
        return 2;
      }
      continue;
    }
    if (arg == "--control-max-grant-lease-ns" || arg == "--control-max-jog-input-age-ns") {
      uint64_t value = 0;
      const bool lease = arg == "--control-max-grant-lease-ns";
      // Unverified absolute cell-policy bounds: 10 s lease, 2 s input age.
      if (i + 1 >= argc || !parse_u64(argv[++i], &value) || value == 0 ||
          value > (lease ? UINT64_C(10000000000) : UINT64_C(2000000000)) ||
          (lease && value % 1000000 != 0)) return 2;
      if (lease) opt.control_max_grant_lease_ns = value;
      else opt.control_max_jog_input_age_ns = value;
      continue;
    }
    if (arg == "--max-cycle-lateness-ns" && i + 1 < argc) {
      if (!parse_u64(argv[++i], &opt.max_cycle_lateness_ns) ||
          opt.max_cycle_lateness_ns == 0 || opt.max_cycle_lateness_ns >= 250000000) {
        logf("ERROR: --max-cycle-lateness-ns must be positive and below the 250 ms lease");
        return 2;
      }
      continue;
    }
    if (arg == "--num-axes" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.num_axes) ||
          opt.num_axes == 0 || opt.num_axes > gradient::ipc::v1::GRADIENT_MAX_AXES) {
        logf("ERROR: invalid --num-axes");
        return 2;
      }
      continue;
    }
    if (arg == "--counts-per-rev" && i + 1 < argc) {
      counts_per_rev_spec = argv[++i];
      continue;
    }
    if (arg == "--gear-ratio" && i + 1 < argc) {
      gear_ratio_spec = argv[++i];
      continue;
    }
    if (arg == "--sign" && i + 1 < argc) {
      sign_spec = argv[++i];
      continue;
    }
    if (arg == "--axis-type" && i + 1 < argc) {
      axis_type_spec = argv[++i];
      continue;
    }
    if (arg == "--lead-m-per-rev" && i + 1 < argc) {
      lead_m_per_rev_spec = argv[++i];
      continue;
    }
    if (arg == "--drive-speed-limit-counts-per-s" && i + 1 < argc) {
      std::vector<uint32_t> caps;
      if (drive_speed_limits || !parse_u32_csv(argv[++i], &caps)) {
        logf("ERROR: invalid or duplicate --drive-speed-limit-counts-per-s");
        return 2;
      }
      drive_speed_limits = std::move(caps);
      continue;
    }
    if (arg == "--max-rpm" && i + 1 < argc) {
      if (!parse_double(argv[++i], &opt.max_rpm) || opt.max_rpm < 0.0) {
        logf("ERROR: invalid --max-rpm (must be >= 0)");
        return 2;
      }
      continue;
    }
    if (arg == "--capture-cycles" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.capture_cycles) || !opt.capture_cycles ||
          opt.capture_cycles > rosie::rtcore::kMaxCaptureCycles) {
        logf("ERROR: --capture-cycles must be in [1, 65536]");
        return 2;
      }
      continue;
    }
    if (arg == "--telemetry-retention-ms" && i + 1 < argc) {
      if (!parse_u64(argv[++i], &opt.telemetry_retention_ms)) return 2;
      continue;
    }
    if (arg == "--telemetry-dump-count" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.telemetry_dump_count) ||
          !opt.telemetry_dump_count || opt.telemetry_dump_count > 100) return 2;
      continue;
    }
    if (arg == "--telemetry-dump-on-exit") { opt.telemetry_dump_on_exit = true; continue; }
    if (arg == "--telemetry-dump-bytes" && i + 1 < argc) {
      // Derived minimum: 2*262144 header/index bytes + one 5392-byte record
      // at the maximum 16 axes = 529680; valid independently of option order.
      static_assert(gradient::ipc::v1::TelemetryDumpMinBytes ==
          2 * rosie::rtcore::kTelemetryDumpHeaderBytes + sizeof(rosie::rtcore::CycleCaptureRecord));
      if (!parse_u64(argv[++i], &opt.telemetry_dump_bytes) ||
          opt.telemetry_dump_bytes < gradient::ipc::v1::TelemetryDumpMinBytes ||
          opt.telemetry_dump_bytes > gradient::ipc::v1::TelemetryDumpMaxBytes) {
        std::fprintf(stderr, "telemetry dump_bytes requires 529680..9007199254740991 bytes (two headers and one 16-axis record)\n");
        return 2;
      }
      continue;
    }
    if (arg == "--telemetry-directory" && i + 1 < argc) {
      const std::filesystem::path directory(argv[++i]);
      if (!directory.is_absolute() || directory.lexically_normal() == directory.root_path()) return 2;
      opt.telemetry_directory = directory.lexically_normal().string();
      continue;
    }
    if (arg == "--slave-positions" && i + 1 < argc) {
      slave_positions_spec = argv[++i];
      continue;
    }
    if (arg == "--bus-unknown-slaves" && i + 1 < argc) {
      const std::string policy = argv[++i];
      if (policy != "refuse" && policy != "hold") return 2;
      opt.bus.hold = policy == "hold";
      continue;
    }
    if (arg == "--bus-unused-slaves" && i + 1 < argc) {
      if (!opt.bus.unused.empty()) return 2;
      const auto entries = split_delim_strict(argv[++i], ';');
      for (const auto& entry : entries) {
        const auto parts = split_delim_strict(entry, '|');
        uint32_t position = 0;
        rosie::rtcore::BusSlaveIdentity slave{};
        if (parts.size() != 4 || !parse_u32(parts[0].c_str(), &position) || position > UINT16_MAX ||
            !parse_u32(parts[1].c_str(), &slave.vendor_id) || !slave.vendor_id ||
            !parse_u32(parts[2].c_str(), &slave.product_code) || !slave.product_code) return 2;
        slave.position = static_cast<uint16_t>(position);
        slave.revision_required = parts[3] != "-";
        if (slave.revision_required && !parse_u32(parts[3].c_str(), &slave.revision)) return 2;
        if (opt.bus.unused.size() == rosie::rtcore::kMaxUnusedSlaves) return 2;
        opt.bus.unused.push_back(slave);
      }
      if (opt.bus.unused.empty()) return 2;
      std::sort(opt.bus.unused.begin(), opt.bus.unused.end(), [](const auto& a, const auto& b) { return a.position < b.position; });
      continue;
    }
    if (arg == "--no-dc") {
      opt.use_dc = false;
      continue;
    }
    if (arg == "--disable-output-watchdog") {
      opt.disable_output_watchdog = true;
      continue;
    }
    if (arg == "--split-domains-per-axis") {
      opt.split_domains_per_axis = true;
      continue;
    }
    if (arg == "--queue-split-domains-round-robin") {
      logf("ERROR: --queue-split-domains-round-robin is unsupported: motion requires a fresh full-group exchange every cycle");
      return 2;
    }
    if (arg == "--explicit-pdo-config") {
      opt.explicit_pdo_config = true;
      continue;
    }
    if (arg == "--wait-before-safeop-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.wait_before_safeop_ms)) {
        logf("ERROR: invalid --wait-before-safeop-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--preop-safeop-timeout-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.preop_to_safeop_timeout_ms)) {
        logf("ERROR: invalid --preop-safeop-timeout-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--safeop-op-timeout-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.safeop_to_op_timeout_ms)) {
        logf("ERROR: invalid --safeop-op-timeout-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--startup-passive-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.startup_passive_ms)) {
        logf("ERROR: invalid --startup-passive-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--startup-skip-domain-queue-ms" && i + 1 < argc) {
      if (!parse_u32(argv[++i], &opt.startup_skip_domain_queue_ms)) {
        logf("ERROR: invalid --startup-skip-domain-queue-ms");
        return 2;
      }
      continue;
    }
    if (arg == "--ipc-group" && i + 1 < argc) {
      opt.ipc_group = argv[++i];
      continue;
    }
    if (arg == "--adapter-user") {
      if (i + 1 >= argc || argv[i + 1][0] == '\0') {
        logf("ERROR: --adapter-user requires a nonempty user name");
        return 2;
      }
      const char* name = argv[++i];
      errno = 0;
      const passwd* account = ::getpwnam(name);
      if (!account) {
        logf("ERROR: --adapter-user '%s' could not be resolved: %s", name,
             errno ? std::strerror(errno) : "user does not exist");
        return 2;
      }
      adapter_uid = account->pw_uid;
      continue;
    }

    if (arg == "--version") {
      std::printf("%s\n", ROSIE_RT_VERSION);
      return {0, false};
    }
    logf("ERROR: unknown arg: %s", arg.c_str());
    return {2, true};
  }

  if (!finalize_axis_config(&opt,
                            counts_per_rev_spec,
                            gear_ratio_spec,
                            sign_spec,
                            axis_type_spec,
                            lead_m_per_rev_spec)) {
    return 2;
  }
  for (size_t i = 0; i < opt.bus.unused.size(); ++i) {
    const auto position = opt.bus.unused[i].position;
    if ((opt.cell_io.on_bus() && position == opt.cell_io.position) ||
        (i && position == opt.bus.unused[i-1].position) ||
        std::find(opt.slave_position.begin(), opt.slave_position.begin() + opt.num_axes, position) != opt.slave_position.begin() + opt.num_axes) {
      logf("ERROR: bus unused position=%u overlaps another declaration or axis", unsigned(position));
      return 2;
    }
  }
  if (opt.cell_io.on_bus() && std::find(opt.slave_position.begin(), opt.slave_position.begin() + opt.num_axes,
      opt.cell_io.position) != opt.slave_position.begin() + opt.num_axes) {
    logf("ERROR: cell I/O terminal position overlaps an axis");
    return 2;
  }
  // Require one compiled cap per axis before identity verification, preventing
  // geared position counts/rev from being reused as motor counts/rev.
  if (drive_speed_limits) {
    if (drive_speed_limits->size() != opt.num_axes) {
      logf("ERROR: --drive-speed-limit-counts-per-s requires one cap per axis");
      return 2;
    }
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      opt.axis[i].max_profile_vel_counts_per_s = (*drive_speed_limits)[i];
    }
  }
  if (!finalize_slave_positions(&opt, slave_positions_spec)) {
    return 2;
  }
  if (axis_profiles_path.empty() || opt.rt_priority < 1 || opt.rt_priority > 99 || opt.rt_cpu >= CPU_SETSIZE) {
    logf("ERROR: explicit compiled axis profiles and RT CPU/priority are required"); return 2;
  }
  auto& startup_sdos = result.startup_sdos;
  auto& axis_profiles = result.axis_profiles;
  std::string axis_profiles_diagnostic;
  if (!load_axis_profiles(axis_profiles_path, opt, &axis_profiles, &startup_sdos, &axis_profiles_diagnostic)) {
    logf("ERROR: invalid --axis-profiles file%s%s", axis_profiles_diagnostic.empty() ? "" : " ",
         axis_profiles_diagnostic.c_str()); return 2;
  }
  try {
    namespace identity = rosie::rtcore::identity;
    for (uint32_t axis = 0; axis < opt.num_axes; ++axis) {
      auto& profile = axis_profiles[axis];
      const auto coordinate = identity::coordinate_fields(axis, opt.axis[axis], profile, startup_sdos);
      if (!coordinate.empty()) {
        const auto changed = identity::first_difference(profile.coordinate_identity, coordinate);
        if (!changed.empty()) {
          logf("ERROR: coordinate identity mismatch axis=%u field=%s", axis, changed.c_str()); return 2;
        }
        profile.coordinate_digest = identity::sha256(coordinate);
      }
    }
    const auto actual = identity::canonical(identity::machine_fields(opt, axis_profiles, startup_sdos));
    std::string expected;
    if (machine_identity_path.empty() && !opt.machine_sha256.empty()) {
      const auto adjacent = std::filesystem::path(axis_profiles_path).parent_path() / "machine.identity";
      if (std::filesystem::exists(adjacent)) machine_identity_path = adjacent.string();
    }
    if (!machine_identity_path.empty()) {
      std::ifstream file(machine_identity_path, std::ios::binary);
      // Unverified admission bound: 1 MiB of canonical fields for at most 16 axes.
      constexpr size_t maximum_identity_bytes = 1024 * 1024;
      char ch = 0;
      while (file.get(ch) && expected.size() <= maximum_identity_bytes) expected += ch;
      if (!file.eof() || expected.empty() || expected.size() > maximum_identity_bytes || opt.machine_sha256.empty()) {
        logf("ERROR: machine identity mismatch: machine_sha256 (invalid identity artifact)"); return 2;
      }
    }
    if (!opt.machine_sha256.empty()) {
      const auto difference = identity::verify(opt.machine_sha256, expected, actual);
      if (!difference.empty()) {
        logf("ERROR: machine identity mismatch: %s", difference.c_str()); return 2;
      }
    }
    opt.machine_sha256 = identity::digest(actual);
    opt.machine_digest = identity::sha256(actual);
    if (!rosie::rtcore::TelemetryRing::mapping_bytes(opt.cycle_ns, opt.telemetry_retention_ms, opt.num_axes)) {
      logf("ERROR: telemetry retention must cover 100 ms and fit the 2 GiB resident ceiling");
      return 2;
    }
    if (opt.deployment_sha256.empty()) {
      opt.deployment_sha256 = identity::digest(identity::canonical({
          {"rt_cpu",std::to_string(opt.rt_cpu)}, {"rt_priority",std::to_string(opt.rt_priority)},
          {"housekeeping_cpus",opt.housekeeping_cpus.value_or("")},
          {"socket_path",opt.socket_path}, {"ipc_group",opt.ipc_group},
          {"telemetry_retention_ms",std::to_string(opt.telemetry_retention_ms)},
          {"telemetry_dump_count",std::to_string(opt.telemetry_dump_count)},
          {"telemetry_dump_bytes",std::to_string(opt.telemetry_dump_bytes)},
          {"telemetry_dump_on_exit",opt.telemetry_dump_on_exit ? "1" : "0"},
          {"telemetry_directory",opt.telemetry_directory}}));
    } else if (!identity::valid_digest(opt.deployment_sha256)) {
      logf("ERROR: invalid deployment_sha256"); return 2;
    }
    logf("Machine identity verified: machine_sha256=%s deployment_sha256=%s",
        opt.machine_sha256.c_str(), opt.deployment_sha256.c_str());
  } catch (const std::exception& error) {
    logf("ERROR: machine identity mismatch: %s", error.what()); return 2;
  }
  auto& have_native_home = result.have_native_home;
  auto& have_absolute_feedback = result.have_absolute_feedback;
  for (uint32_t i = 0; i < opt.num_axes; ++i) {
    auto& p = axis_profiles[i];
    p.limits.max_qd = std::min(p.limits.max_qd,
        double(opt.axis[i].max_profile_vel_counts_per_s) / opt.axis[i].counts_per_unit);
    if (p.vendor_id == 0 || p.product_code == 0 || !validate_pdo_layout(p.pdo) ||
        (opt.use_dc && p.dc_quantum_ns && opt.cycle_ns % p.dc_quantum_ns)) {
      logf("ERROR: invalid axis profile or DC period axis=%u", i); return 2;
    }
    have_native_home |= p.home.valid;
    have_absolute_feedback |= p.absolute.valid;
  }
  if (opt.configuration_hash.size() != 64 ||
      !std::all_of(opt.configuration_hash.begin(), opt.configuration_hash.end(),
                   [](unsigned char c) { return std::isxdigit(c); })) return 2;
  if (validate_config_only) { logf("Configuration valid: %u axes", opt.num_axes); return 0; }
  return result;
}
