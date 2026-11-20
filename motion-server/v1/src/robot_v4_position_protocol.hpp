#pragma once

// Server-only position codecs. Included by nats_protocol after the exact JSON
// scanner definitions, inside its existing non-scanner branch. Prerequisites:
// PositionExecutionRequest, BridgePositionPermit and the daemon's top-level
// JSON/identity helpers. Native scanner consumers never include this file.

bool parse_position_command_id(const std::string& payload,
                               std::string* text, std::string* json) {
  if (text == nullptr || json == nullptr) return false;
  std::string raw, decoded;
  if (!extract_unique_top_level_json_member(payload, "command_id", &raw) || raw.empty()) return false;
  if (raw.front() == '"') {
    if (!parse_exact_json_string_value(raw, &decoded) || decoded.empty()) return false;
  } else {
    // Legacy RTCore command IDs are uint64 (including UnixNano values).
    // Do not route through double, which rounds IDs above 2^53.
    uint64_t value = 0;
    if ((raw.size() > 1 && raw.front() == '0') ||
        !parse_exact_json_u64_value(raw, &value)) return false;
    decoded = std::to_string(value);
  }
  *text = std::move(decoded);
  *json = std::move(raw);
  return true;
}

bool parse_position_execution_request(const std::string& payload,
                                      PositionExecutionRequest* out) {
  if (out == nullptr) return false;
  PositionExecutionRequest request;
  const auto number = [&](const char* key, double* value,
                          bool* supplied = nullptr) {
    std::string raw;
    bool present = false;
    if (!extract_unique_top_level_json_member(payload, key, &raw, &present)) return false;
    if (supplied != nullptr) *supplied = present;
    if (!present) return true;
    // strtod alone accepts hex, NaN and infinities. Admit only decimal JSON
    // tokens, and require complete consumption and a finite result.
    if (raw.empty()) return false;
    std::size_t cursor = raw.front() == '-' ? 1 : 0;
    const auto digits = [&]() {
      const auto start = cursor;
      while (cursor < raw.size() && raw[cursor] >= '0' && raw[cursor] <= '9') ++cursor;
      return cursor != start;
    };
    if (cursor == raw.size()) return false;
    if (raw[cursor] == '0') ++cursor;
    else if (!digits()) return false;
    if (cursor < raw.size() && raw[cursor] == '.') {
      ++cursor;
      if (!digits()) return false;
    }
    if (cursor < raw.size() && (raw[cursor] == 'e' || raw[cursor] == 'E')) {
      ++cursor;
      if (cursor < raw.size() && (raw[cursor] == '+' || raw[cursor] == '-')) ++cursor;
      if (!digits()) return false;
    }
    if (cursor != raw.size()) return false;
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(raw.c_str(), &end);
    if (errno != 0 || end != raw.c_str() + raw.size() || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
  };
  const auto optional_string = [&](const char* key, std::string* value) {
    std::string raw;
    bool present = false;
    return extract_unique_top_level_json_member(payload, key, &raw, &present) &&
           (!present || parse_exact_json_string_value(raw, value));
  };
  std::string control_mode, mastering_action;
  if (!optional_string("control_mode", &control_mode) ||
      !optional_string("runtime_mode", &request.runtime_mode) ||
      !optional_string("servo_v1_action", &mastering_action) ||
      !mastering_action.empty()) return false; // Record-only mastering stays separate.
  control_mode = transport_identity_trim_copy(control_mode);
  std::transform(control_mode.begin(), control_mode.end(), control_mode.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (!control_mode.empty() && control_mode != "csp") return false;
  double axis = 0, timeout_ms = 0, tolerance_deg = 0;
  double target_deg = 0, relative_rad = 0, min_deg = 0, max_deg = 0;
  if (!number("axis", &axis) || axis < 0 || axis > std::numeric_limits<int>::max() ||
      std::floor(axis) != axis || !number("target_rad", &request.target_rad) ||
      !number("target_deg", &target_deg) ||
      !number("relative_jog_rad", &relative_rad, &request.relative) ||
      !number("min_rad", &request.min_rad) || !number("max_rad", &request.max_rad) ||
      !number("min_deg", &min_deg) || !number("max_deg", &max_deg) ||
      !number("timeout_ms", &timeout_ms) || std::floor(timeout_ms) != timeout_ms ||
      !number("tolerance_deg", &tolerance_deg) ||
      !number("max_speed_rad_s", &request.max_speed_rad_s)) return false;
  constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
  if (request.relative && (request.target_rad != 0 || target_deg != 0)) return false;
  request.axis = static_cast<int>(axis);
  if (request.target_rad == 0) request.target_rad = target_deg * radians_per_degree;
  if (request.relative) request.target_rad = relative_rad;
  if (request.min_rad == 0) request.min_rad = min_deg * radians_per_degree;
  if (request.max_rad == 0) request.max_rad = max_deg * radians_per_degree;
  if (tolerance_deg > 0) request.tolerance_rad = tolerance_deg * radians_per_degree;
  if (timeout_ms > 0) request.timeout = std::chrono::milliseconds(
      static_cast<int64_t>(std::min(timeout_ms, 45000.0)));
  *out = std::move(request);
  return true;
}

bool decode_bridge_position_permit(const std::string& body, BridgePositionPermit* out) {
  if (out == nullptr) return false;
  std::string schema, identity;
  BridgePositionPermit permit;
  if (!parse_top_level_robot_command_string(body, "schema", &schema) || schema != "robot.v4.bridge-position-permit.v1" ||
      !parse_position_command_id(body, &identity, &permit.command_id_json)) return false;
  for (auto item : {std::pair<const char*, std::string*>{"challenge_id", &permit.challenge_id},
        {"bridge_boot_id", &permit.bridge_boot_id}, {"run_id", &permit.run_id},
        {"controller_id", &permit.controller_id}, {"controller_boot_id", &permit.controller_boot_id},
        {"leader_lease_id", &permit.leader_lease_id}, {"pair_config_sha256", &permit.pair_config_sha256}}) {
    if (!parse_top_level_robot_command_string(body, item.first, item.second) || item.second->empty()) return false;
  }
  for (auto item : {std::pair<const char*, uint64_t*>{"leader_fence_epoch", &permit.leader_fence_epoch},
        {"bridge_observed_monotonic_ms", &permit.bridge_observed_monotonic_ms},
        {"deadline_monotonic_ms", &permit.deadline_monotonic_ms}, {"remaining_ms", &permit.remaining_ms},
        {"run_deadline_monotonic_ms", &permit.run_deadline_monotonic_ms}, {"run_remaining_ms", &permit.run_remaining_ms},
        {"runtime_mode_epoch", &permit.runtime_mode_epoch}, {"activation_epoch", &permit.activation_epoch},
        {"home_epoch", &permit.home_epoch}, {"pair_revision", &permit.pair_revision}}) {
    if (!parse_top_level_robot_command_u64(body, item.first, item.second)) return false;
  }
  *out = std::move(permit);
  return true;
}
