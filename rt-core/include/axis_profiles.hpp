#pragma once

// Included after the Gradient config parsers, inside their namespace.
// The human-facing JSON compiler emits this private, strict line format.
// It is parsed once at startup; no parsing or allocation occurs in the RT loop.
// Optional qualified coordinate source. Bounds use signed drive-pulse units;
// the independent raw source has a separately Home-bound command-frame origin.
// The physical speed bound includes disabled backdrive and is deliberately
// independent of the commanded velocity limit.
struct CoordinateEvidenceConfig {
  bool enabled = false;
  uint64_t maximum_physical_speed_counts_per_second = 0;
  uint64_t maximum_acquisition_age_ns = 0;
  uint64_t maximum_feedback_error_counts = 0;
  uint64_t maximum_anchor_error_counts = 0;
  uint16_t validity_mask = 0;
  uint16_t validity_value = 0;
  bool sdo_sampled = false;
  uint64_t sample_interval_ns = 0;
};

// An assigned bit observes external enable only; absence never implies enabled.
struct ExternalEnableInputConfig {
  uint32_t bit_index = 0;
  bool active_high = true;
};

// Disabled unless explicitly configured; invalid thresholds never arm a guard.
// Unverified admission bounds: torque 1..1000 raw, error 1..INT32_MAX counts,
// persistence 2..1000 cycles. These are not measured collision limits.
struct CollisionWatchdogConfig {
  bool enabled = false;
  uint32_t torque_abs_max_raw = 0;
  uint32_t following_error_counts_max = 0;
  uint32_t sustained_cycles = 0;
  bool valid() const noexcept {
    return torque_abs_max_raw >= 1 && torque_abs_max_raw <= 1000 &&
        following_error_counts_max >= 1 && following_error_counts_max <= INT32_MAX &&
        sustained_cycles >= 2 && sustained_cycles <= 1000;
  }
};

// One drive tuning object read once per startup epoch and journalled. Never
// written, never a service-write target and never part of config_verified_mask:
// it makes drive-held tuning (auto-tune results, panel edits) visible without a
// second mailbox client competing with rt-core.
struct StartupObserveConfig {
  std::string key;
  StartupSdoValueType type = StartupSdoValueType::kNone;
  uint16_t index = 0;
  uint8_t subindex = 0;
};

// Per-axis capacities shared with the compiler (tools/rtctl/drive_tuning.go).
constexpr size_t kMaxStartupSdosPerAxis = 32;
constexpr size_t kMaxStartupObservesPerAxis = 64;

// "key|type|index|sub;..." with unique keys and objects; empty is refused
// because the compiler omits the line when nothing is observed.
inline bool parse_startup_observe_spec(const std::string& spec, std::vector<StartupObserveConfig>* out) {
  if (!out) return false;
  out->clear();
  const auto entries = split_delim_strict(trim_ascii_ws(spec), ';');
  if (entries.empty() || entries.size() > kMaxStartupObservesPerAxis) return false;
  for (const auto& entry : entries) {
    const auto fields = split_delim_strict(entry, '|');
    if (fields.size() != 4) return false;
    StartupObserveConfig cfg{};
    cfg.key = trim_ascii_ws(fields[0]);
    cfg.type = startup_sdo_parse_type(trim_ascii_ws(fields[1]));
    uint16_t index = 0, subindex = 0;
    if (cfg.key.empty() || cfg.type == StartupSdoValueType::kNone ||
        !std::all_of(cfg.key.begin(), cfg.key.end(), [](unsigned char c) { return std::isalnum(c) || c == '_'; }) ||
        !parse_u16_auto(fields[2].c_str(), &index) || !parse_u16_auto(fields[3].c_str(), &subindex) ||
        index == 0 || subindex > 255u) return false;
    cfg.index = index;
    cfg.subindex = static_cast<uint8_t>(subindex);
    for (const auto& existing : *out)
      if (existing.key == cfg.key || (existing.index == cfg.index && existing.subindex == cfg.subindex)) return false;
    out->push_back(std::move(cfg));
  }
  return true;
}

// A drive object read while running and published per axis for operators
// (docs/decisions/0001), for example the A6-EC heatsink temperature. The value
// is raw / 10^decimals in unit. Diagnostic only: it gates nothing.
struct MonitorConfig {
  std::string key, unit;
  SdoObjectSpec object{};
  uint32_t decimals = 0;
  uint64_t interval_ns = 0;
};

// "key|index|sub|type|decimals|unit|interval_ms;..." with unique keys and
// objects; empty is refused because the compiler omits the line when none.
inline bool parse_monitor_spec(const std::string& spec, std::vector<MonitorConfig>* out) {
  if (!out) return false;
  out->clear();
  auto identifier = [](const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), [](unsigned char c) { return std::isalnum(c) || c == '_'; });
  };
  const auto entries = split_delim_strict(trim_ascii_ws(spec), ';');
  if (entries.empty() || entries.size() > rosie::rtcore::kMaxMonitorPerAxis) return false;
  for (const auto& entry : entries) {
    const auto fields = split_delim_strict(entry, '|');
    if (fields.size() != 7) return false;
    MonitorConfig cfg{};
    cfg.key = trim_ascii_ws(fields[0]);
    cfg.unit = trim_ascii_ws(fields[5]);
    uint16_t index = 0, subindex = 0;
    char* end = nullptr;
    errno = 0;
    const unsigned long decimals = std::strtoul(fields[4].c_str(), &end, 10);
    if (errno || end == fields[4].c_str() || *end) return false;
    errno = 0;
    const unsigned long long interval_ms = std::strtoull(fields[6].c_str(), &end, 10);
    if (errno || end == fields[6].c_str() || *end) return false;
    // Same bounds as the compiler (tools/rtctl/render.go renderMonitor).
    if (!identifier(cfg.key) || !identifier(cfg.unit) || !parse_u16_auto(fields[1].c_str(), &index) ||
        !parse_u16_auto(fields[2].c_str(), &subindex) || index == 0 || subindex > 255u ||
        !parse_sdo_scalar_type_token(fields[3], &cfg.object.type) || decimals > 6 ||
        interval_ms < 1000 || interval_ms > 3600000) return false;
    cfg.object.valid = true;
    cfg.object.index = index;
    cfg.object.subindex = static_cast<uint8_t>(subindex);
    cfg.decimals = static_cast<uint32_t>(decimals);
    cfg.interval_ns = interval_ms * 1000000ull;
    for (const auto& existing : *out)
      if (existing.key == cfg.key ||
          (existing.object.index == cfg.object.index && existing.object.subindex == cfg.object.subindex)) return false;
    out->push_back(std::move(cfg));
  }
  return true;
}

struct AxisDriveProfile {
  bool encoder_battery_absent = false;
  bool encoder_battery_absent_alarm_valid = false;
  uint16_t encoder_battery_absent_error = 0;
  uint32_t encoder_battery_absent_aux = 0;
  // Explicit verified-delta restore of an HM35 v3 anchor. No delta cap: the
  // restored position must lie within the joint's hard limits.
  bool reference_verified_delta_enabled = false;
  // Automatic startup recognition of a persisted HM35 v3 anchor on an independent
  // absolute axis. Absent policy follows A6-EC firmware, which clears 6041 bit
  // 15 at every power-up (vendor reply 2026-04-15, email 4 Q1). A profile that
  // declares bit-15 retention requires it; one that refuses restart trust
  // disables the automatic path. Neither changes explicit restore operations.
  bool retained_home_restore_enabled = true;
  bool retained_home_requires_homing_attained = false;
  // Independent-encoder start-alignment floor, compiled from the drive
  // profile's joint-angle value by this axis's gearing. Required on an
  // independent absolute origin; never a native default.
  uint32_t start_alignment_floor_counts = 0;
  struct AbsoluteAnchorSource {
    uint16_t index;
    uint8_t low_subindex, high_subindex;
    std::string name;
    std::string identity_name;
    uint32_t version;
  };
  AbsoluteAnchorSource anchor_source{};
  // The drive profile owns the continuous source and its anchor identity.
  bool independent_absolute_origin() const noexcept {
    return coordinate_evidence.sdo_sampled && anchor_source.index &&
        anchor_source.low_subindex && anchor_source.high_subindex &&
        !anchor_source.identity_name.empty() && anchor_source.version;
  }
  const AbsoluteAnchorSource& absolute_anchor_source() const noexcept { return anchor_source; }
  bool tolerates_encoder_warning(uint16_t error, uint32_t aux, bool aux_valid) const noexcept {
    return encoder_battery_absent && encoder_battery_absent_alarm_valid && aux_valid &&
        error == encoder_battery_absent_error && aux == encoder_battery_absent_aux;
  }
  // Pending or invalid receipts cannot turn a nonzero alarm into a warning.
  bool drive_error_faulted(uint16_t error, uint32_t aux, bool aux_valid) const noexcept {
    return (error || (aux_valid && aux)) && !tolerates_encoder_warning(error, aux, aux_valid);
  }
  // A warning never masks a DS402 or latched safety fault. A null result lets
  // the publisher describe the remaining coordinate/enable/brake gates.
  const char* alarm_readiness(uint16_t error, uint32_t aux, bool aux_valid,
                              uint16_t statusword, bool safety_fault) const noexcept {
    if (safety_fault || (statusword & 8u) || drive_error_faulted(error, aux, aux_valid)) return "faulted";
    return tolerates_encoder_warning(error, aux, aux_valid) ? "warning_tolerated" : nullptr;
  }
  CollisionWatchdogConfig collision_watchdog{};
  std::string coordinate_identity;
  std::array<uint8_t, 32> coordinate_digest{};
  std::string name;
  uint32_t vendor_id = 0;
  uint32_t product_code = 0;
  uint32_t revision_no = 0;
  uint32_t dc_assign_activate = 0;
  uint64_t dc_quantum_ns = 0;
  int32_t home_truth_sign = 0; // Required explicitly, even when home is disabled.
  DrivePdoConfig pdo{};
  NativeHomeConfig home{};
  AbsoluteFeedbackConfig absolute{};
  std::string absolute_pair_field;
  rosie::rtcore::AxisLimits limits;
  // Absence explicitly opts out of sampled acceleration validation; no fallback.
  std::optional<double> max_acceleration;
  CoordinateEvidenceConfig coordinate_evidence{};
  rosie::rtcore::BrakeConfig brake{};
  // Compiled bench override metadata; kept outside BrakeConfig and the cyclic state.
  std::string brake_override_reason_hex;
  rosie::rtcore::JogTiming jog{};
  // Machine-level bounds compiled per axis; absent leaves supervision disabled.
  rosie::rtcore::DriveSupervisionConfig drive_supervision{};
  // Drive-profile classification of resettable drive alarms; absent resets none.
  rosie::rtcore::DriveAlarmResetConfig drive_alarm_reset{};
  std::string brake_released_semantic;
  uint32_t brake_released_bit = 0;
  uint32_t brake_released_width = 0;
  std::optional<ExternalEnableInputConfig> external_enable_input;
  std::vector<StartupObserveConfig> startup_observe;
  std::vector<MonitorConfig> monitor;
  // parameter_storage: the startup key holding the drive's storage-mode object
  // at its store value, and the value that writes to RAM only. startup_trial:
  // keys written inside a RAM-only window. Both name this axis's startup keys.
  std::string parameter_storage_key;
  int64_t parameter_storage_off = 0;
  std::vector<std::string> startup_trial;
};

bool validate_pdo_layout(const DrivePdoConfig& pdo) {
  const std::vector<std::pair<std::string, unsigned int>> rx{{"cw",16}, {"target_pos",32},
      {"target_vel",32}, {"target_torque",16}, {"mode",8}, {"tp_func",16}, {"max_profile_vel",32}};
  const std::vector<std::pair<std::string, unsigned int>> tx{{"sw",16}, {"pos",32}, {"err",16},
      {"manufacturer_err",32}, {"torque",16}, {"mode_disp",8}, {"tp_status",16},
      {"tp_pos1",32}, {"tp_pos2",32}, {"di",32}, {"bus_voltage",16}, {"load_rate",16},
      {"position_error",32}, {"multi_turn_lo",32}, {"multi_turn_hi",32}, {"igbt_temp",16},
      {"motor_temp",16}, {"drive_not_ready",16}, {"motor_not_rotating",16},
      {"continuous_pos",32}, {"continuous_valid",16},
      {"velocity_actual",32}, {"following_error",32}};
  auto valid = [](const auto& entries, const auto& allowed, const auto& required) {
    std::vector<std::string> seen;
    std::vector<uint32_t> objects;
    for (const auto& entry : entries) {
      bool known = false;
      for (const auto& field : allowed) {
        if (entry.semantic == field.first && entry.bits == field.second) known = true;
      }
      const uint32_t object = (uint32_t(entry.index) << 8) | entry.subindex;
      if (!known || entry.index == 0 ||
          std::find(seen.begin(), seen.end(), entry.semantic) != seen.end() ||
          std::find(objects.begin(), objects.end(), object) != objects.end()) return false;
      seen.push_back(entry.semantic);
      objects.push_back(object);
    }
    for (const auto& field : required) {
      if (std::find(seen.begin(), seen.end(), field) == seen.end()) return false;
    }
    return true;
  };
  return pdo.rx_pdo != 0 && pdo.tx_pdo != 0 && pdo.rx_sync_index < 16 &&
      pdo.tx_sync_index < 16 && pdo.rx_sync_index != pdo.tx_sync_index &&
      pdo.rx_entries.size() + pdo.tx_entries.size() <= 32 &&
      valid(pdo.rx_entries, rx, std::vector<std::string>{"cw", "target_pos", "mode"}) &&
      valid(pdo.tx_entries, tx, std::vector<std::string>{"sw", "pos", "mode_disp"});
}

bool load_axis_profiles(const std::string& path, const Options& opt,
                        std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES>* profiles,
                        std::vector<StartupSdoConfig>* startup, std::string* diagnostic = nullptr) {
  if (diagnostic) diagnostic->clear();
  if (!profiles || !startup || opt.num_axes == 0 ||
      opt.num_axes > gradient::ipc::v1::GRADIENT_MAX_AXES) return false;
  std::ifstream input(path);
  if (!input) return false;
  std::array<bool, gradient::ipc::v1::GRADIENT_MAX_AXES> seen{};
  uint32_t axis = gradient::ipc::v1::GRADIENT_MAX_AXES;
  std::vector<std::string> keys;
  std::string line;
  std::string refused_key;
  const char* refused_reason = nullptr;
  // Retain the first refusal until the section ends, allowing name to occur
  // after the bad declaration without losing the axis identity in the CLI.
  auto refuse = [&](const std::string& key, const char* reason) {
    refused_key = key;
    refused_reason = reason;
  };
  // An explicit empty optional descriptor means disabled. An absent key means
  // incomplete input; it must never inherit another axis or vendor convention.
  constexpr std::array<const char*, 19> required_keys{{
      "name", "vendor_id", "product_code", "revision_no", "rx_pdo", "tx_pdo",
      "rx_sync", "tx_sync", "dc_quantum_ns", "dc_assign_activate", "rx_layout",
      "tx_layout", "home_truth_sign", "native_home", "absolute_feedback",
      "absolute_pair_field", "startup_sdos", "limits", "tracking"}};
  auto section_complete = [&]() {
    if (axis == gradient::ipc::v1::GRADIENT_MAX_AXES) return true;
    if (refused_reason) {
      if (diagnostic) *diagnostic = "axis=" + std::to_string(axis) + " name=" + (*profiles)[axis].name +
          " key=" + refused_key + " reason=" + refused_reason;
      return false;
    }
    const bool have_coordinate_evidence =
        std::find(keys.begin(), keys.end(), "coordinate_evidence") != keys.end();
    const bool have_coordinate_anchor_source =
        std::find(keys.begin(), keys.end(), "coordinate_anchor_source") != keys.end();
    const bool have_brake = std::find(keys.begin(), keys.end(), "brake") != keys.end();
    const bool have_brake_transition = std::find(keys.begin(), keys.end(), "brake_transition") != keys.end();
    const bool have_brake_opening = std::find(keys.begin(), keys.end(), "brake_opening") != keys.end();
    const bool have_supervision = std::find(keys.begin(), keys.end(), "drive_supervision") != keys.end();
    const bool have_alarm_reset = std::find(keys.begin(), keys.end(), "drive_alarm_reset") != keys.end();
    const bool have_lag_allowance = std::find(keys.begin(), keys.end(), "tracking_lag_allowance") != keys.end();
    const bool have_brake_override = std::find(keys.begin(), keys.end(), "brake_override_reason_hex") != keys.end();
    const bool have_jog = std::find(keys.begin(), keys.end(), "jog") != keys.end();
    const bool have_jog_jerk = std::find(keys.begin(), keys.end(), "jog_jerk") != keys.end();
    const bool have_collision = std::find(keys.begin(), keys.end(), "collision_watchdog") != keys.end();
    const bool have_acceleration = std::find(keys.begin(), keys.end(), "max_acceleration") != keys.end();
    const bool have_external_enable = std::find(keys.begin(), keys.end(), "external_enable_input") != keys.end();
    const bool have_restore_assignment = std::find(keys.begin(), keys.end(), "restore_assignment_on_exit") != keys.end();
    const bool have_battery = std::find(keys.begin(), keys.end(), "encoder_battery") != keys.end();
    const bool have_battery_alarm = std::find(keys.begin(), keys.end(), "encoder_battery_absent_alarm") != keys.end();
    const bool have_coordinate_identity = std::find(keys.begin(), keys.end(), "coordinate_identity") != keys.end();
    const bool have_observe = std::find(keys.begin(), keys.end(), "startup_observe") != keys.end();
    const bool have_monitor = std::find(keys.begin(), keys.end(), "monitor") != keys.end();
    const bool have_storage = std::find(keys.begin(), keys.end(), "parameter_storage") != keys.end();
    const bool have_trial = std::find(keys.begin(), keys.end(), "startup_trial") != keys.end();
    if (!have_coordinate_identity || (*profiles)[axis].coordinate_identity.empty()) {
      logf("ERROR: missing coordinate_identity axis=%u name=%s", axis, (*profiles)[axis].name.c_str());
      return false;
    }
    const bool have_delta_restore = std::find(keys.begin(), keys.end(), "reference_verified_delta_restore") != keys.end();
    const bool have_retained_home = std::find(keys.begin(), keys.end(), "retained_home_restore") != keys.end();
    if (have_retained_home && !(*profiles)[axis].independent_absolute_origin()) return false;
    const bool have_alignment_floor = std::find(keys.begin(), keys.end(), "start_alignment_floor") != keys.end();
    if (!have_alignment_floor && (*profiles)[axis].independent_absolute_origin()) {
      logf("ERROR: axis=%u name=%s independent absolute origin requires a configured start_alignment_floor",
           axis, (*profiles)[axis].name.c_str());
      return false;
    }
    if (have_delta_restore) {
      const auto& p = (*profiles)[axis];
      if (!p.independent_absolute_origin() || !p.brake.present || !p.home.truth_source.valid ||
          p.home.truth_source.index != 0x607c || p.home.truth_source.subindex ||
          p.home.truth_source.type != SdoScalarType::kI32 || (p.home_truth_sign != -1 && p.home_truth_sign != 1)) return false;
    }
    if ((*profiles)[axis].coordinate_evidence.sdo_sampled != have_coordinate_anchor_source) return false;
    if (have_battery != have_battery_alarm) return false;
    return keys.size() == required_keys.size() + (have_retained_home ? 1u : 0u) + (have_delta_restore ? 1u : 0u) + (have_coordinate_evidence ? 1u : 0u) +
        (have_coordinate_anchor_source ? 1u : 0u) +
        (have_alignment_floor ? 1u : 0u) +
        (have_brake ? 1u : 0u) + (have_brake_transition ? 1u : 0u) + (have_brake_opening ? 1u : 0u) + (have_supervision ? 1u : 0u) + (have_lag_allowance ? 1u : 0u) + (have_brake_override ? 1u : 0u) + (have_jog ? 1u : 0u) + (have_jog_jerk ? 1u : 0u) + (have_acceleration ? 1u : 0u) + (have_collision ? 1u : 0u) +
        (have_external_enable ? 1u : 0u) + (have_coordinate_identity ? 1u : 0u) + (have_restore_assignment ? 1u : 0u) + (have_battery ? 1u : 0u) +
        (have_battery_alarm ? 1u : 0u) +
        (have_observe ? 1u : 0u) + (have_monitor ? 1u : 0u) + (have_storage ? 1u : 0u) + (have_trial ? 1u : 0u) + (have_alarm_reset ? 1u : 0u) &&
        std::all_of(required_keys.begin(), required_keys.end(), [&](const char* key) {
          return std::find(keys.begin(), keys.end(), key) != keys.end();
        });
  };
  startup->clear();
  while (std::getline(input, line)) {
    line = trim_ascii_ws(line);
    if (line.empty() || line[0] == '#') continue;
    const auto eq = line.find('=');
    if (eq == std::string::npos) return false;
    const std::string key = line.substr(0, eq), value = line.substr(eq + 1);
    if (key == "axis") {
      if (!section_complete()) return false;
      if (!parse_u32(value.c_str(), &axis) || axis >= opt.num_axes || seen[axis]) return false;
      seen[axis] = true;
      (*profiles)[axis] = AxisDriveProfile{};
      keys.clear();
      continue;
    }
    if (axis >= opt.num_axes) return false;
    // A named refusal is irrevocable. Read only this section's validated name;
    // later declarations cannot overwrite the cause or reopen admission.
    if (refused_reason && key != "name") continue;
    if (std::find(keys.begin(), keys.end(), key) != keys.end()) {
      if (key == "coordinate_identity") { refuse(key, "duplicate"); continue; }
      if (key == "restore_assignment_on_exit") {
        refuse(key, value == ((*profiles)[axis].pdo.restore_assignment_on_exit ? "1" : "0")
            ? "duplicate" : "conflict");
        continue;
      }
      return false;
    }
    keys.push_back(key);
    auto& p = (*profiles)[axis];
    uint32_t n = 0;
    if (key == "name") {
      if (value.empty() || value.size() > 63 || !std::all_of(value.begin(), value.end(),
          [](unsigned char c) { return std::isalnum(c) || c == '_' || c == '-' || c == '.'; })) return false;
      p.name = value;
    }
    else if (key == "coordinate_identity") {
      // Unverified admission bound: 4096 decoded bytes per axis.
      if (value.size() > 8192) return false;
      try { p.coordinate_identity = rosie::rtcore::identity::unhex_text(value); }
      catch (const std::exception&) { return false; }
    }
    else if (key == "restore_assignment_on_exit") {
      if ((value != "0" && value != "1") || opt.explicit_pdo_config) { refuse(key, "conflict"); continue; }
      p.pdo.restore_assignment_on_exit = value == "1";
    }
    else if (key == "encoder_battery") {
      if (value != "absent") return false;
      p.encoder_battery_absent = true;
    }
    else if (key == "encoder_battery_absent_alarm") {
      const auto parts = split_delim_strict(value, '|');
      uint32_t error = 0, aux = 0;
      if (parts.size() != 2 || !parse_u32_auto(parts[0].c_str(), &error) || error > 0xffff ||
          !parse_u32_auto(parts[1].c_str(), &aux) || !error || !aux) return false;
      p.encoder_battery_absent_error = static_cast<uint16_t>(error);
      p.encoder_battery_absent_aux = aux;
      p.encoder_battery_absent_alarm_valid = true;
    }
    else if (key == "limits") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 4) return false;
      char* end = nullptr;
      double* outputs[] = {&p.limits.min_q, &p.limits.max_q, &p.limits.max_qd};
      for (size_t k = 0; k < 3; ++k) {
        *outputs[k] = std::strtod(parts[k].c_str(), &end);
        if (end == parts[k].c_str() || *end || !std::isfinite(*outputs[k])) return false;
      }
      if (p.limits.min_q >= p.limits.max_q || p.limits.max_qd <= 0 ||
          (parts[3] != "0" && parts[3] != "1")) return false;
      p.limits.require_home = parts[3] == "1";
      p.limits.valid = true;
    }
    else if (key == "external_enable_input") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 2 || parts[0].empty() ||
          !std::all_of(parts[0].begin(), parts[0].end(), [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
          !parse_u32(parts[0].c_str(), &n) || n >= 32 ||
          (parts[1] != "active_high" && parts[1] != "active_low")) return false;
      p.external_enable_input = ExternalEnableInputConfig{n, parts[1] == "active_high"};
    }
    else if (key == "max_acceleration") {
      char* end = nullptr;
      const double acceleration = std::strtod(value.c_str(), &end);
      if (end == value.c_str() || *end || !std::isfinite(acceleration) || acceleration <= 0) return false;
      p.max_acceleration = acceleration;
    }
    else if (key == "collision_watchdog") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 3) return false;
      for (const auto& part : parts)
        if (part.empty() || !std::all_of(part.begin(), part.end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; })) return false;
      auto& c = p.collision_watchdog;
      if (!parse_u32(parts[0].c_str(), &c.torque_abs_max_raw) ||
          !parse_u32(parts[1].c_str(), &c.following_error_counts_max) ||
          !parse_u32(parts[2].c_str(), &c.sustained_cycles) || !c.valid()) return false;
      c.enabled = true;
    }
    else if (key == "tracking") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 6) return false;
      double* outputs[] = {&p.limits.jog_acceleration, &p.limits.max_target_lead, &p.limits.following_error, &p.limits.completion_tolerance};
      char* end = nullptr;
      for (size_t k = 0; k < 4; ++k) {
        *outputs[k] = std::strtod(parts[k].c_str(), &end);
        if (end == parts[k].c_str() || *end || !std::isfinite(*outputs[k]) || *outputs[k] <= 0) return false;
      }
      if (!parse_u32(parts[4].c_str(), &n) || !n || n > 1000) return false;
      p.limits.following_error_timeout_ns = uint64_t(n) * 1000000;
      if (!parse_u32(parts[5].c_str(), &n) || !n) return false;
      p.limits.completion_timeout_ns = uint64_t(n) * 1000000;
    }
    else if (key == "tracking_lag_allowance") {
      // Milliseconds, 1..1000, mirroring the compiler. Unverified sanity cap:
      // a lag of a second or more is not tracking, and an allowance that large
      // would let the following-error fault ignore real loss of control.
      if (value.empty() || !std::all_of(value.begin(), value.end(),
          [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
          !parse_u32(value.c_str(), &n) || !n || n > 1000) return false;
      p.limits.tracking_lag_allowance_ns = uint64_t(n) * 1000000;
    }
    else if (key == "jog") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 2) return false;
      for (const auto& part : parts)
        if (part.empty() || !std::all_of(part.begin(), part.end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; })) return false;
      if (!parse_u64(parts[0].c_str(), &p.jog.arrest_ns) ||
          !parse_u64(parts[1].c_str(), &p.jog.quick_stop_ns)) return false;
    }
    else if (key == "jog_jerk") {
      // <jerk joint units/s^3>|<smoothing window cycles>, both decimal
      // integers, mirroring the compiler: jerk 1..100000 (unverified sanity
      // cap), window 2..JogSmoother::max_window.
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 2) return false;
      for (const auto& part : parts)
        if (part.empty() || !std::all_of(part.begin(), part.end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; })) return false;
      if (!parse_u32(parts[0].c_str(), &p.jog.jerk) || !p.jog.jerk || p.jog.jerk > 100000 ||
          !parse_u32(parts[1].c_str(), &p.jog.smoothing_cycles) || p.jog.smoothing_cycles < 2 ||
          p.jog.smoothing_cycles > rosie::rtcore::JogSmoother::max_window) return false;
    }
    else if (key == "brake") {
      const auto parts = split_delim_strict(value, '|');
      if ((parts.size() != 6 && parts.size() != 7) || (parts[0] != "0" && parts[0] != "1") ||
          (parts[3] != "0" && parts[3] != "1")) return false;
      uint32_t release_ms = 0, hold_ms = 0, bit = 0;
      for (const auto field : {1u, 2u, 5u})
        if (parts[field].empty() || !std::all_of(parts[field].begin(), parts[field].end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; })) return false;
      if (!parse_u32(parts[1].c_str(), &release_ms) ||
          !parse_u32(parts[2].c_str(), &hold_ms) || !parse_u32(parts[5].c_str(), &bit)) return false;
      p.brake.present = parts[0] == "1";
      p.brake.release_delay_ns = uint64_t(release_ms) * 1000000;
      p.brake.hold_delay_ns = uint64_t(hold_ms) * 1000000;
      p.brake.gravity_axis = parts[3] == "1";
      p.brake.feedback_required = parts[4] != "-";
      if ((!p.brake.present && (p.brake.gravity_axis || p.brake.feedback_required)) ||
          (!p.brake.feedback_required && bit != 0)) return false;
      if (p.brake.feedback_required) p.brake_released_semantic = parts[4];
      p.brake_released_bit = bit;
      if (parts.size() == 7 && (parts[6].empty() ||
          !std::all_of(parts[6].begin(), parts[6].end(),
              [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
          !parse_u32(parts[6].c_str(), &p.brake.hold_displacement_tolerance_counts) ||
          p.brake.hold_displacement_tolerance_counts > INT32_MAX)) return false;
    }
    else if (key == "brake_transition") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 5) return false;
      uint32_t values[5]{};
      for (size_t k = 0; k < 5; ++k) {
        if (parts[k].empty() || !std::all_of(parts[k].begin(), parts[k].end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
            !parse_u32(parts[k].c_str(), &values[k]) || !values[k] || values[k] > INT32_MAX) return false;
      }
      p.brake.transition = {values[0], values[1], uint64_t(values[2]) * 1000000,
          uint64_t(values[3]) * 1000000, uint64_t(values[4]) * 1000000};
    }
    else if (key == "brake_opening") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 3) return false;
      uint32_t values[3]{};
      for (size_t k = 0; k < 3; ++k) {
        if (parts[k].empty() || !std::all_of(parts[k].begin(), parts[k].end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
            !parse_u32(parts[k].c_str(), &values[k]) || !values[k] || values[k] > INT32_MAX) return false;
      }
      p.brake.opening = {values[0], values[1], uint64_t(values[2]) * 1000000};
    }
    else if (key == "start_alignment_floor") {
      if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
          !parse_u32(value.c_str(), &p.start_alignment_floor_counts) || !p.start_alignment_floor_counts ||
          p.start_alignment_floor_counts > INT32_MAX) return false;
    }
    else if (key == "drive_supervision") {
      const auto parts = split_delim_strict(value, '|');
      if (parts.size() != 3) return false;
      uint32_t values[3]{};
      for (size_t k = 0; k < 3; ++k) {
        if (parts[k].empty() || !std::all_of(parts[k].begin(), parts[k].end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
            !parse_u32(parts[k].c_str(), &values[k]) || !values[k] || values[k] > INT32_MAX) return false;
      }
      p.drive_supervision = {uint64_t(values[0]) * 1000000, uint64_t(values[1]) * 1000000, values[2]};
    }
    else if (key == "drive_alarm_reset") {
      if (!rosie::rtcore::parse_drive_alarm_reset_spec(value, &p.drive_alarm_reset)) {
        refuse(key, "invalid");
        continue;
      }
    }
    else if (key == "brake_override_reason_hex") {
      if (value.empty() || value.size() % 2 ||
          !std::all_of(value.begin(), value.end(), [](unsigned char c) {
            return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
          })) return false;
      p.brake_override_reason_hex = value;
    }
    else if (key == "coordinate_evidence") {
      const auto parts = split_delim_strict(value, '|');
      const bool sampled = parts.size() >= 6 && parts[4] == "a6ec_sdo";
      if (parts.size() != 6) return false;
      std::array<uint64_t, 6> values{};
      for (size_t field = 0; field < values.size(); ++field) {
        if (sampled && field == 4) continue;
        if (parts[field].empty() || !std::all_of(parts[field].begin(), parts[field].end(),
            [](unsigned char c) { return c >= '0' && c <= '9'; }) ||
            !parse_u64(parts[field].c_str(), &values[field])) return false;
      }
      if (sampled) {
        if (!values[0] || !values[1] || !values[5] || values[5] >= values[1]) return false;
        // Unverified memory cap, shared with SampledPdoHistory::configure.
        if (!rosie::rtcore::sampled_history_fits(values[1], opt.cycle_ns)) {
          logf("ERROR: sampled PDO history exceeds 131072 entries (ceil(age_ns/cycle_ns)+2)");
          return false;
        }
        if (values[5] > UINT64_MAX / 2) {
          logf("ERROR: two sample cadences exceed the evidence-loss grace range");
          return false;
        }
        p.coordinate_evidence = {true, values[0], values[1], values[2], values[3], 0, 0, true, values[5]};
      } else {
        if (!values[0] || !values[1] || !values[4] || values[4] > UINT16_MAX ||
            values[5] > UINT16_MAX || (values[5] & ~values[4])) return false;
        p.coordinate_evidence = {true, values[0], values[1], values[2], values[3],
            static_cast<uint16_t>(values[4]), static_cast<uint16_t>(values[5])};
      }
    }
    else if (key == "coordinate_anchor_source") {
      const auto parts = split_delim_strict(value, '|');
      uint16_t index = 0, low = 0, high = 0;
      uint32_t version = 0;
      if (parts.size() != 6 || !parse_u16_auto(parts[0].c_str(), &index) || !index ||
          !parse_u16_auto(parts[1].c_str(), &low) || !low || low > 255 ||
          !parse_u16_auto(parts[2].c_str(), &high) || !high || high > 255 ||
          !parse_u32(parts[5].c_str(), &version) || !version ||
          !std::all_of(parts[3].begin(), parts[3].end(), [](unsigned char c) { return std::isalnum(c) || c == '_'; }) ||
          !std::all_of(parts[4].begin(), parts[4].end(), [](unsigned char c) { return std::isalnum(c) || c == '_'; }))
        return false;
      p.anchor_source = {index, static_cast<uint8_t>(low), static_cast<uint8_t>(high),
          parts[3], parts[4], version};
    }
    else if (key == "rx_layout") { if (!parse_pdo_layout_spec(value, &p.pdo.rx_entries)) return false; }
    else if (key == "tx_layout") { if (!parse_pdo_layout_spec(value, &p.pdo.tx_entries)) return false; }
    else if (key == "reference_verified_delta_restore") {
      // The former ",<maximum_delta_counts>" suffix is refused: a capped
      // configuration compiled by an older tool is not the current policy.
      if (value != "independent_absolute_delta_v1") return false;
      p.reference_verified_delta_enabled = true;
    }
    else if (key == "reference_capture") {
      // Removed current-pose zero capture; a configuration carrying it is refused.
      logf("ERROR: axis profile %s declares reference_capture: current-pose reference capture "
           "was removed; establish Home with drive homing (HM35)", p.name.c_str());
      return false;
    }
    else if (key == "retained_home_restore") {
      // Only non-default policies are compiled; the absent key is the A6 default.
      if (value == "disabled") p.retained_home_restore_enabled = false;
      else if (value == "persisted_anchor_agreement_v1,homing_attained_required")
        p.retained_home_requires_homing_attained = true;
      else return false;
    }
    else if (key == "native_home") { if (!parse_native_home_config_spec(value, &p.home)) return false; }
    else if (key == "absolute_feedback") { if (!parse_absolute_feedback_config_spec(value, &p.absolute)) return false; }
    else if (key == "absolute_pair_field") {
      if (!std::all_of(value.begin(), value.end(), [](unsigned char c) { return std::isalnum(c) || c == '_'; })) return false;
      p.absolute_pair_field = value;
    }
    else if (key == "startup_sdos") {
      std::vector<StartupSdoConfig> descriptors;
      if (!parse_startup_sdo_config_spec(value, 1, &descriptors) || descriptors.size() > kMaxStartupSdosPerAxis) return false;
      for (auto& d : descriptors) {
        const auto v = d.values[0];
        d.values.fill(0);
        d.values[axis] = v;
        d.axis_mask = 1u << axis;
        startup->push_back(d);
      }
    }
    else if (key == "startup_observe") {
      if (!parse_startup_observe_spec(value, &p.startup_observe)) return false;
    }
    else if (key == "monitor") {
      if (!parse_monitor_spec(value, &p.monitor)) return false;
    }
    else if (key == "parameter_storage") {
      // "key|storage_off"; the key's own descriptor carries the store value.
      const auto fields = split_delim_strict(value, '|');
      char* end = nullptr;
      errno = 0;
      if (fields.size() != 2 || fields[0].empty() || fields[1].empty()) return false;
      const long long off = std::strtoll(fields[1].c_str(), &end, 10);
      if (errno || end != fields[1].c_str() + fields[1].size()) return false;
      p.parameter_storage_key = fields[0];
      p.parameter_storage_off = off;
    }
    else if (key == "startup_trial") {
      const auto entries = split_delim_strict(value, ';');
      if (entries.empty()) return false;
      for (const auto& entry : entries) {
        if (entry.empty() || std::find(p.startup_trial.begin(), p.startup_trial.end(), entry) != p.startup_trial.end())
          return false;
        p.startup_trial.push_back(entry);
      }
    }
    else if (key == "home_truth_sign") {
      if (value != "1" && value != "-1") return false;
      p.home_truth_sign = value == "1" ? 1 : -1;
    }
    else {
      if (!parse_u32_auto(value.c_str(), &n)) return false;
      if (key == "vendor_id") p.vendor_id = n;
      else if (key == "product_code") p.product_code = n;
      else if (key == "revision_no") p.revision_no = n;
      else if (key == "dc_quantum_ns") p.dc_quantum_ns = n;
      else if (key == "dc_assign_activate" && n <= 65535) p.dc_assign_activate = n;
      else if (key == "rx_pdo" && n <= 65535) p.pdo.rx_pdo = n;
      else if (key == "tx_pdo" && n <= 65535) p.pdo.tx_pdo = n;
      else if (key == "rx_sync" && n < 16) p.pdo.rx_sync_index = n;
      else if (key == "tx_sync" && n < 16) p.pdo.tx_sync_index = n;
      else return false;
    }
  }
  if (!input.eof() || !section_complete()) return false;
  for (uint32_t i = 0; i < opt.num_axes; ++i) {
    auto& p = (*profiles)[i];
    if (!p.jog.configure(opt.cycle_ns)) return false;
    if (!seen[i] || p.name.empty() || !p.limits.valid || !p.limits.following_error_timeout_ns || !p.limits.completion_timeout_ns || p.vendor_id == 0 || p.product_code == 0 || !validate_pdo_layout(p.pdo) ||
        (opt.use_dc && p.dc_quantum_ns && opt.cycle_ns % p.dc_quantum_ns)) return false;
    if (p.limits.require_home && !p.home.valid) return false;
    if (p.encoder_battery_absent && !p.encoder_battery_absent_alarm_valid) return false;
    if (p.external_enable_input) {
      uint32_t width = 0;
      for (const auto& entry : p.pdo.tx_entries)
        if (entry.semantic == "di") width = entry.bits;
      if (p.external_enable_input->bit_index >= width) return false;
    }
    // Metadata must agree with the effective bench policy; no active brake can
    // be published as overridden even if the private record was hand-edited.
    if (!p.brake_override_reason_hex.empty() && (!opt.bus.hold || p.brake.present)) return false;
    if (p.brake.transition.enabled()) {
      bool velocity_mapped = false;
      for (const auto& entry : p.pdo.tx_entries)
        velocity_mapped |= entry.semantic == "velocity_actual" && entry.bits == 32;
      if (!p.brake.present || !velocity_mapped ||
          !p.brake.transition.valid(p.brake.hold_delay_ns, p.brake.hold_displacement_tolerance_counts)) return false;
    }
    if (p.brake.opening.enabled()) {
      if (!p.brake.present || !p.brake.transition.enabled() ||
          !p.brake.opening.valid(p.brake.hold_displacement_tolerance_counts)) return false;
    }
    if (p.brake.feedback_required) {
      for (const auto& entry : p.pdo.tx_entries)
        if (entry.semantic == p.brake_released_semantic)
          p.brake_released_width = entry.bits;
      if (!p.brake_released_width || p.brake_released_bit >= p.brake_released_width) return false;
    }
    const auto mapped = [&](const char* semantic) {
      return std::any_of(p.pdo.tx_entries.begin(), p.pdo.tx_entries.end(),
          [&](const auto& entry) { return entry.semantic == semantic; });
    };
    if (p.collision_watchdog.enabled && !mapped("torque")) return false;
    if (p.coordinate_evidence.enabled) {
      if (p.coordinate_evidence.sdo_sampled) {
        if (opt.explicit_pdo_config || mapped("continuous_pos") || mapped("continuous_valid")) return false;
      } else if (!mapped("continuous_pos") || !mapped("continuous_valid")) return false;
    } else if (mapped("continuous_pos") || mapped("continuous_valid")) {
      // A diagnostic mapping never opts an axis into coordinate recovery.
      return false;
    }
    if (!p.absolute_pair_field.empty()) {
      bool found = false;
      for (uint32_t field = 0; field < p.absolute.field_count; ++field)
        found |= p.absolute.fields[field].valid &&
            p.absolute.fields[field].key == p.absolute_pair_field;
      if (!found) return false;
    }
    for (uint32_t previous = 0; previous < i; ++previous)
      if ((*profiles)[previous].name == p.name) return false;
    // An observed object written on the same axis would journal twice; its
    // startup readback already reports it (the compiler refuses this too).
    for (const auto& observe : p.startup_observe)
      for (const auto& d : *startup)
        if ((d.axis_mask & (1u << i)) && (d.key == observe.key ||
            (d.index == observe.index && d.subindex == observe.subindex))) return false;
    // Storage policy names this axis's own startup keys. The storage-mode
    // descriptor is written first and verified like any other; a trial key is
    // written only between storage off and storage on, so it needs that
    // object, and the storage object itself is never a trial.
    auto own = [&](const std::string& key) -> StartupSdoConfig* {
      for (auto& d : *startup)
        if (d.axis_mask == (1u << i) && d.key == key) return &d;
      return nullptr;
    };
    if (!p.parameter_storage_key.empty()) {
      auto* storage = own(p.parameter_storage_key);
      int64_t off = 0;
      if (!storage || !startup_sdo_parse_value(storage->type, std::to_string(p.parameter_storage_off), &off) ||
          off == storage->values[i]) return false;
      storage->storage_mode = true;
      storage->storage_off = off;
    }
    for (const auto& key : p.startup_trial) {
      auto* trial = own(key);
      if (!trial || trial->storage_mode || p.parameter_storage_key.empty()) return false;
      trial->ram_only = true;
    }
    // One machine-level block: every axis carries it or none does, and the
    // timing bounds are identical (counts differ only by gear).
    const auto& first = (*profiles)[0].drive_supervision;
    if (!p.drive_supervision.valid() || p.drive_supervision.configured() != first.configured() ||
        p.drive_supervision.enable_timeout_ns != first.enable_timeout_ns ||
        p.drive_supervision.release_slack_ns != first.release_slack_ns) {
      logf("ERROR: drive_supervision must be one machine-level block applied to every axis");
      return false;
    }
    // One machine-level time: every axis carries the same allowance or none
    // does. Counts follow from each axis's own commanded velocity.
    if (p.limits.tracking_lag_allowance_ns != (*profiles)[0].limits.tracking_lag_allowance_ns) {
      logf("ERROR: tracking_lag_allowance must be one machine-level value applied to every axis");
      return false;
    }
    // One machine-level jerk and window. The window must bound this axis's
    // jerk: the mean of an a-limited integrator over N cycles changes its
    // acceleration by at most 2*a/(N*cycle) per second (jog and Halt ramps).
    if (p.jog.jerk != (*profiles)[0].jog.jerk ||
        p.jog.smoothing_cycles != (*profiles)[0].jog.smoothing_cycles) {
      logf("ERROR: jog_jerk must be one machine-level value applied to every axis");
      return false;
    }
    if (p.jog.smoothing_cycles) {
      const double acceleration = std::max(p.limits.jog_acceleration, p.max_acceleration.value_or(0.0));
      const double window_s = double(p.jog.smoothing_cycles) * double(opt.cycle_ns) / 1e9;
      if (!(2 * acceleration / window_s <= double(p.jog.jerk) * (1 + 1e-12))) {
        logf("ERROR: jog_jerk window %u cycles does not bound axis=%u jerk to %u", p.jog.smoothing_cycles, i, p.jog.jerk);
        return false;
      }
    }
    p.pdo.label = format_pdo_profile_label(p.pdo.rx_pdo, p.pdo.tx_pdo);
  }
  return startup->size() <= kMaxStartupSdoDescriptors;
}
