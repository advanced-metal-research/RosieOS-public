// Private implementation header included once by robot_v4_cartesian_daemon.cpp.
std::string rtcore_status_host_from_subject(const std::string& subject) {
  constexpr const char* prefix = "robot/v4/rtcore.";
  constexpr const char* suffix = ".status";
  const std::string prefix_text(prefix);
  const std::string suffix_text(suffix);
  if (subject.rfind(prefix_text, 0) != 0 ||
      subject.size() <= prefix_text.size() + suffix_text.size() ||
      subject.compare(subject.size() - suffix_text.size(), suffix_text.size(),
                      suffix_text) != 0) {
    return "";
  }
  return subject.substr(
      prefix_text.size(),
      subject.size() - prefix_text.size() - suffix_text.size());
}

void skip_json_whitespace(const std::string& text,
                          std::string::size_type* cursor) {
  while (*cursor < text.size()) {
    const char ch = text[*cursor];
    if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
      return;
    }
    ++(*cursor);
  }
}

int json_hex_value(char ch) {
  if (ch >= '0' && ch <= '9') {
    return ch - '0';
  }
  if (ch >= 'a' && ch <= 'f') {
    return 10 + ch - 'a';
  }
  if (ch >= 'A' && ch <= 'F') {
    return 10 + ch - 'A';
  }
  return -1;
}

bool scan_json_string_token(const std::string& text,
                            std::string::size_type start,
                            std::string::size_type* end,
                            std::string* decoded = nullptr) {
  if (end == nullptr || start >= text.size() || text[start] != '"') {
    return false;
  }
  if (decoded != nullptr) {
    decoded->clear();
  }
  for (std::string::size_type index = start + 1; index < text.size(); ++index) {
    const unsigned char ch = static_cast<unsigned char>(text[index]);
    if (ch == '"') {
      *end = index + 1;
      return true;
    }
    if (ch < 0x20) {
      return false;
    }
    if (ch != '\\') {
      if (decoded != nullptr) {
        decoded->push_back(static_cast<char>(ch));
      }
      continue;
    }
    if (++index >= text.size()) {
      return false;
    }
    const char escaped = text[index];
    char value = '\0';
    switch (escaped) {
      case '"':
      case '\\':
      case '/':
        value = escaped;
        break;
      case 'b':
        value = '\b';
        break;
      case 'f':
        value = '\f';
        break;
      case 'n':
        value = '\n';
        break;
      case 'r':
        value = '\r';
        break;
      case 't':
        value = '\t';
        break;
      case 'u': {
        if (index + 4 >= text.size()) {
          return false;
        }
        unsigned int codepoint = 0;
        for (int digit = 0; digit < 4; ++digit) {
          const int hex = json_hex_value(text[++index]);
          if (hex < 0) {
            return false;
          }
          codepoint = (codepoint << 4u) | static_cast<unsigned int>(hex);
        }
        if (decoded != nullptr) {
          // All authority names are ASCII. Preserve exact escaped ASCII so
          // escaped duplicate keys cannot evade ambiguity rejection; any
          // non-ASCII value cannot equal an accepted authority value.
          decoded->push_back(codepoint <= 0x7fu
                                 ? static_cast<char>(codepoint)
                                 : '?');
        }
        continue;
      }
      default:
        return false;
    }
    if (decoded != nullptr) {
      decoded->push_back(value);
    }
  }
  return false;
}

bool scan_json_compound_token(const std::string& text,
                              std::string::size_type start,
                              std::string::size_type* end) {
  if (end == nullptr || start >= text.size() ||
      (text[start] != '{' && text[start] != '[')) {
    return false;
  }
  std::vector<char> closing;
  closing.push_back(text[start] == '{' ? '}' : ']');
  std::string::size_type cursor = start + 1;
  while (cursor < text.size()) {
    const char ch = text[cursor];
    if (ch == '"') {
      if (!scan_json_string_token(text, cursor, &cursor)) {
        return false;
      }
      continue;
    }
    if (ch == '{' || ch == '[') {
      closing.push_back(ch == '{' ? '}' : ']');
      ++cursor;
      continue;
    }
    if (ch == '}' || ch == ']') {
      if (closing.empty() || ch != closing.back()) {
        return false;
      }
      closing.pop_back();
      ++cursor;
      if (closing.empty()) {
        *end = cursor;
        return true;
      }
      continue;
    }
    ++cursor;
  }
  return false;
}

bool skip_json_value_token(const std::string& text,
                           std::string::size_type start,
                           std::string::size_type* end) {
  if (end == nullptr) {
    return false;
  }
  skip_json_whitespace(text, &start);
  if (start >= text.size()) {
    return false;
  }
  if (text[start] == '"') {
    return scan_json_string_token(text, start, end);
  }
  if (text[start] == '{' || text[start] == '[') {
    return scan_json_compound_token(text, start, end);
  }
  std::string::size_type cursor = start;
  while (cursor < text.size() && text[cursor] != ',' &&
         text[cursor] != '}' && text[cursor] != ']' &&
         text[cursor] != ' ' && text[cursor] != '\t' &&
         text[cursor] != '\r' && text[cursor] != '\n') {
    ++cursor;
  }
  if (cursor == start) {
    return false;
  }
  const std::string token = text.substr(start, cursor - start);
  if (token != "true" && token != "false" && token != "null" &&
      (token[0] != '-' && (token[0] < '0' || token[0] > '9'))) {
    return false;
  }
  *end = cursor;
  return true;
}

// Extracts exactly one direct member from a JSON object. The scanner is
// deliberately structural: strings and nested containers are balanced, the
// outer object must consume the entire payload, and duplicate authority keys
// (including their escaped spellings) fail closed.
bool extract_unique_top_level_json_member(const std::string& object,
                                          const std::string& wanted_key,
                                          std::string* value,
                                          bool* present) {
  if (present != nullptr) *present = false;
  if (value == nullptr) {
    return false;
  }
  std::string::size_type cursor = 0;
  skip_json_whitespace(object, &cursor);
  if (cursor >= object.size() || object[cursor] != '{') {
    return false;
  }
  ++cursor;
  bool found = false;
  for (;;) {
    skip_json_whitespace(object, &cursor);
    if (cursor >= object.size()) {
      return false;
    }
    if (object[cursor] == '}') {
      ++cursor;
      break;
    }
    std::string key;
    if (!scan_json_string_token(object, cursor, &cursor, &key)) {
      return false;
    }
    skip_json_whitespace(object, &cursor);
    if (cursor >= object.size() || object[cursor] != ':') {
      return false;
    }
    ++cursor;
    skip_json_whitespace(object, &cursor);
    const std::string::size_type value_start = cursor;
    std::string::size_type value_end = cursor;
    if (!skip_json_value_token(object, cursor, &value_end)) {
      return false;
    }
    if (key == wanted_key) {
      if (found) {
        return false;
      }
      *value = object.substr(value_start, value_end - value_start);
      found = true;
    }
    cursor = value_end;
    skip_json_whitespace(object, &cursor);
    if (cursor >= object.size()) {
      return false;
    }
    if (object[cursor] == ',') {
      ++cursor;
      continue;
    }
    if (object[cursor] == '}') {
      ++cursor;
      break;
    }
    return false;
  }
  skip_json_whitespace(object, &cursor);
  if (present != nullptr) *present = found;
  return (found || present != nullptr) && cursor == object.size();
}

bool parse_exact_json_string_value(const std::string& raw,
                                   std::string* value) {
  if (value == nullptr) {
    return false;
  }
  std::string::size_type cursor = 0;
  skip_json_whitespace(raw, &cursor);
  if (!scan_json_string_token(raw, cursor, &cursor, value)) {
    return false;
  }
  skip_json_whitespace(raw, &cursor);
  return cursor == raw.size();
}


bool parse_exact_json_string_array_value(
    const std::string& raw,
    std::vector<std::string>* values) {
  if (values == nullptr) {
    return false;
  }
  values->clear();
  std::string::size_type cursor = 0;
  skip_json_whitespace(raw, &cursor);
  if (cursor >= raw.size() || raw[cursor] != '[') {
    return false;
  }
  ++cursor;
  for (;;) {
    skip_json_whitespace(raw, &cursor);
    if (cursor >= raw.size()) {
      return false;
    }
    if (raw[cursor] == ']') {
      ++cursor;
      skip_json_whitespace(raw, &cursor);
      return cursor == raw.size();
    }
    std::string value;
    if (!scan_json_string_token(raw, cursor, &cursor, &value)) {
      return false;
    }
    values->push_back(std::move(value));
    skip_json_whitespace(raw, &cursor);
    if (cursor >= raw.size()) {
      return false;
    }
    if (raw[cursor] == ',') {
      ++cursor;
      skip_json_whitespace(raw, &cursor);
      if (cursor >= raw.size() || raw[cursor] == ']') {
        return false;
      }
      continue;
    }
    if (raw[cursor] == ']') {
      ++cursor;
      skip_json_whitespace(raw, &cursor);
      return cursor == raw.size();
    }
    return false;
  }
}

bool parse_exact_json_u64_value(const std::string& raw, uint64_t* value) {
  if (value == nullptr) {
    return false;
  }
  std::string::size_type cursor = 0;
  skip_json_whitespace(raw, &cursor);
  if (cursor >= raw.size() || raw[cursor] < '0' || raw[cursor] > '9') {
    return false;
  }
  const char* start = raw.c_str() + cursor;
  char* end = nullptr;
  errno = 0;
  const unsigned long long parsed = std::strtoull(start, &end, 10);
  if (errno != 0 || end == start) {
    return false;
  }
  cursor = static_cast<std::string::size_type>(end - raw.c_str());
  skip_json_whitespace(raw, &cursor);
  if (cursor != raw.size()) {
    return false;
  }
  *value = static_cast<uint64_t>(parsed);
  return true;
}

bool parse_exact_json_bool_value(const std::string& raw, bool* value) {
  if (value == nullptr) {
    return false;
  }
  std::string::size_type cursor = 0;
  skip_json_whitespace(raw, &cursor);
  if (raw.compare(cursor, 4, "true") == 0) {
    cursor += 4;
    *value = true;
  } else if (raw.compare(cursor, 5, "false") == 0) {
    cursor += 5;
    *value = false;
  } else {
    return false;
  }
  skip_json_whitespace(raw, &cursor);
  return cursor == raw.size();
}

#ifndef ROBOT_V4_JSON_SCANNER_ONLY
#include "robot_v4_position_protocol.hpp"
#include "robot_v4_bridge_home_protocol.hpp"
bool parse_exact_json_double_array(
    const std::string& raw,
    std::array<double, kLiveAxisCount>* values) {
  if (values == nullptr) {
    return false;
  }
  std::string::size_type cursor = 0;
  skip_json_whitespace(raw, &cursor);
  if (cursor >= raw.size() || raw[cursor] != '[') {
    return false;
  }
  ++cursor;
  std::array<double, kLiveAxisCount> parsed{};
  for (std::size_t index = 0; index < kLiveAxisCount; ++index) {
    skip_json_whitespace(raw, &cursor);
    if (cursor >= raw.size()) {
      return false;
    }
    const char* start = raw.c_str() + cursor;
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(start, &end);
    if (errno != 0 || end == start || !std::isfinite(value)) {
      return false;
    }
    parsed[index] = value;
    cursor = static_cast<std::string::size_type>(
        end - raw.c_str());
    skip_json_whitespace(raw, &cursor);
    if (index + 1 < kLiveAxisCount) {
      if (cursor >= raw.size() || raw[cursor] != ',') {
        return false;
      }
      ++cursor;
    }
  }
  skip_json_whitespace(raw, &cursor);
  if (cursor >= raw.size() || raw[cursor] != ']') {
    return false;
  }
  ++cursor;
  skip_json_whitespace(raw, &cursor);
  if (cursor != raw.size()) {
    return false;
  }
  *values = parsed;
  return true;
}

bool parse_exact_json_u16_array(
    const std::string& raw,
    std::array<uint16_t, kLiveAxisCount>* values,
    bool allow_zero = false) {
  if (values == nullptr) {
    return false;
  }
  std::string::size_type cursor = 0;
  skip_json_whitespace(raw, &cursor);
  if (cursor >= raw.size() || raw[cursor] != '[') {
    return false;
  }
  ++cursor;
  std::array<uint16_t, kLiveAxisCount> parsed{};
  for (std::size_t index = 0; index < kLiveAxisCount; ++index) {
    skip_json_whitespace(raw, &cursor);
    if (cursor >= raw.size() ||
        raw[cursor] < '0' || raw[cursor] > '9') {
      return false;
    }
    const char* start = raw.c_str() + cursor;
    char* end = nullptr;
    errno = 0;
    const unsigned long parsed_value =
        std::strtoul(start, &end, 10);
    if (errno != 0 || end == start ||
        (!allow_zero && parsed_value == 0) ||
        parsed_value > std::numeric_limits<uint16_t>::max()) {
      return false;
    }
    parsed[index] = static_cast<uint16_t>(parsed_value);
    cursor = static_cast<std::string::size_type>(
        end - raw.c_str());
    skip_json_whitespace(raw, &cursor);
    if (index + 1 < kLiveAxisCount) {
      if (cursor >= raw.size() || raw[cursor] != ',') {
        return false;
      }
      ++cursor;
    }
  }
  skip_json_whitespace(raw, &cursor);
  if (cursor >= raw.size() || raw[cursor] != ']') {
    return false;
  }
  ++cursor;
  skip_json_whitespace(raw, &cursor);
  if (cursor != raw.size()) {
    return false;
  }
  *values = parsed;
  return true;
}

bool lowercase_hex_string(const std::string& value,
                          std::size_t width) {
  if (value.size() != width) {
    return false;
  }
  for (const char ch : value) {
    if (!((ch >= '0' && ch <= '9') ||
          (ch >= 'a' && ch <= 'f'))) {
      return false;
    }
  }
  return true;
}

bool parse_rtcore_pair_config_binding(
    const std::string& status,
    RTCorePairConfigBinding* binding) {
  if (binding == nullptr) {
    return false;
  }
  RTCorePairConfigBinding parsed;
  std::string raw;
  std::string activation;
  std::string ethercat_live_raw;
  std::string drive_converged_raw;
  std::string reconciliation_simulated_raw;
  uint64_t mapped_axis_mask = 0;
  uint64_t home_restore_pending_axis_mask = 0;
  uint64_t home_valid_axis_mask = 0;
  uint64_t reconciliation_abort_code = 0;
  uint64_t reconciliation_failed_axis_mask = 0;
  if (!extract_unique_top_level_json_member(
          status, "pair_config_jetson_id", &raw) ||
      !parse_exact_json_string_value(raw, &parsed.jetson_id) ||
      parsed.jetson_id.empty() || parsed.jetson_id.size() >= 32 ||
      !extract_unique_top_level_json_member(
          status, "pair_config_robot_model_id", &raw) ||
      !parse_exact_json_string_value(raw, &parsed.robot_model_id) ||
      parsed.robot_model_id.empty() ||
      parsed.robot_model_id.size() >= 32 ||
      !extract_unique_top_level_json_member(
          status, "pair_config_revision", &raw) ||
      !parse_exact_json_u64_value(raw, &parsed.revision) ||
      parsed.revision == 0 ||
      !extract_unique_top_level_json_member(
          status, "pair_config_sha256", &raw) ||
      !parse_exact_json_string_value(
          raw, &parsed.snapshot_sha256) ||
      !lowercase_hex_string(parsed.snapshot_sha256, 64) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_generated_urdf_sha256", &raw) ||
      !parse_exact_json_string_value(
          raw, &parsed.generated_urdf_sha256) ||
      !lowercase_hex_string(
          parsed.generated_urdf_sha256, 64) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_activation_epoch", &raw) ||
      !parse_exact_json_u64_value(
          raw, &parsed.activation_epoch) ||
      parsed.activation_epoch == 0 ||
      !extract_unique_top_level_json_member(
          status, "pair_config_home_epoch", &raw) ||
      !parse_exact_json_u64_value(raw, &parsed.home_epoch) ||
      parsed.home_epoch == 0 ||
      !extract_unique_top_level_json_member(
          status, "pair_config_home_profile_key", &raw) ||
      !parse_exact_json_string_value(
          raw, &parsed.home_profile_key) ||
      !lowercase_hex_string(parsed.home_profile_key, 16) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_activation", &raw) ||
      !parse_exact_json_string_value(raw, &activation) ||
      (activation != "configuration-only" &&
       activation != "live-qualified") ||
      !extract_unique_top_level_json_member(
          status, "ethercat_live", &ethercat_live_raw) ||
      !parse_exact_json_bool_value(
          ethercat_live_raw, &parsed.ethercat_live) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_mapped_axis_mask", &raw) ||
      !parse_exact_json_u64_value(raw, &mapped_axis_mask) ||
      mapped_axis_mask == 0 ||
      mapped_axis_mask > RTCORE_MAX_LIVE_AXIS_MASK ||
      !extract_unique_top_level_json_member(
          status, "home_valid_axis_mask", &raw) ||
      !parse_exact_json_u64_value(raw, &home_valid_axis_mask) ||
      home_valid_axis_mask > RTCORE_MAX_LIVE_AXIS_MASK ||
      (home_valid_axis_mask & ~mapped_axis_mask) != 0u ||
      !extract_unique_top_level_json_member(
          status, "pair_config_drive_ratio_converged",
          &drive_converged_raw) ||
      !parse_exact_json_bool_value(
          drive_converged_raw, &parsed.drive_ratio_converged) ||
      !extract_unique_top_level_json_member(
          status,
          "pair_config_ratio_reconciliation_simulated",
          &reconciliation_simulated_raw) ||
      !parse_exact_json_bool_value(
          reconciliation_simulated_raw,
          &parsed.ratio_reconciliation_simulated) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_ratio_reconciliation_state", &raw) ||
      !parse_exact_json_string_value(
          raw, &parsed.ratio_reconciliation_state)) {
    return false;
  }
  parsed.configuration_only =
      activation == "configuration-only";
  parsed.mapped_axis_mask =
      static_cast<uint32_t>(mapped_axis_mask);
  std::string pending_home_raw;
  if (extract_unique_top_level_json_member(
          status, "home_restore_pending_axis_mask", &pending_home_raw)) {
    if (!parse_exact_json_u64_value(
            pending_home_raw, &home_restore_pending_axis_mask) ||
        home_restore_pending_axis_mask > RTCORE_MAX_LIVE_AXIS_MASK ||
        (home_restore_pending_axis_mask & ~mapped_axis_mask) != 0u) {
      return false;
    }
    parsed.home_restore_pending_axis_mask_present = true;
    parsed.home_restore_pending_axis_mask =
        static_cast<uint32_t>(home_restore_pending_axis_mask);
  }
  parsed.home_valid_axis_mask =
      static_cast<uint32_t>(home_valid_axis_mask);
  const bool reconciliation_state_known =
      parsed.ratio_reconciliation_state == "uninitialized" ||
      parsed.ratio_reconciliation_state == "pending" ||
      parsed.ratio_reconciliation_state == "succeeded" ||
      parsed.ratio_reconciliation_state == "failed" ||
      parsed.ratio_reconciliation_state == "invalidated" ||
      parsed.ratio_reconciliation_state == "simulated";
  const bool reconciliation_succeeded =
      parsed.ratio_reconciliation_state == "succeeded";
  const bool reconciliation_simulated =
      parsed.ratio_reconciliation_state == "simulated";
  if (!reconciliation_state_known ||
      parsed.ratio_reconciliation_simulated !=
          reconciliation_simulated ||
      (reconciliation_succeeded &&
       !parsed.ethercat_live) ||
      (reconciliation_simulated &&
       parsed.ethercat_live) ||
      parsed.drive_ratio_converged !=
          (reconciliation_succeeded ||
           reconciliation_simulated) ||
      !extract_unique_top_level_json_member(
          status,
          "pair_config_ratio_reconciliation_abort_code",
          &raw) ||
      !parse_exact_json_u64_value(
          raw, &reconciliation_abort_code) ||
      reconciliation_abort_code >
          std::numeric_limits<uint32_t>::max() ||
      !extract_unique_top_level_json_member(
          status,
          "pair_config_ratio_reconciliation_failed_axis_mask",
          &raw) ||
      !parse_exact_json_u64_value(
          raw, &reconciliation_failed_axis_mask) ||
      reconciliation_failed_axis_mask >
          RTCORE_MAX_LIVE_AXIS_MASK ||
      (parsed.drive_ratio_converged &&
       (reconciliation_abort_code != 0 ||
        reconciliation_failed_axis_mask != 0))) {
    return false;
  }
  std::array<uint16_t, kLiveAxisCount> observed_numerator{};
  std::array<uint16_t, kLiveAxisCount> observed_denominator{};
  if (!extract_unique_top_level_json_member(
          status, "pair_config_gear_numerator", &raw) ||
      !parse_exact_json_u16_array(
          raw, &parsed.gear_numerator) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_gear_denominator", &raw) ||
      !parse_exact_json_u16_array(
          raw, &parsed.gear_denominator) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_observed_gear_numerator", &raw) ||
      !parse_exact_json_u16_array(
          raw, &observed_numerator, true) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_observed_gear_denominator", &raw) ||
      !parse_exact_json_u16_array(
          raw, &observed_denominator, true) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_home_position_rad", &raw) ||
      !parse_exact_json_double_array(
          raw, &parsed.home_position_rad) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_speed_limit_rad_s", &raw) ||
      !parse_exact_json_double_array(
          raw, &parsed.speed_limit_rad_s) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_lower_limit_rad", &raw) ||
      !parse_exact_json_double_array(
          raw, &parsed.lower_limit_rad) ||
      !extract_unique_top_level_json_member(
          status, "pair_config_upper_limit_rad", &raw) ||
      !parse_exact_json_double_array(
          raw, &parsed.upper_limit_rad)) {
    return false;
  }
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    const uint32_t bit = 1u << axis;
    if ((parsed.mapped_axis_mask & bit) == 0u) {
      if (observed_numerator[axis] != 0u ||
          observed_denominator[axis] != 0u) {
        return false;
      }
      continue;
    }
    if (parsed.drive_ratio_converged &&
        (observed_numerator[axis] !=
             parsed.gear_numerator[axis] ||
         observed_denominator[axis] !=
             parsed.gear_denominator[axis])) {
      return false;
    }
    if (!(parsed.speed_limit_rad_s[axis] > 0.0) ||
        !(parsed.lower_limit_rad[axis] <
          parsed.upper_limit_rad[axis]) ||
        parsed.home_position_rad[axis] <
            parsed.lower_limit_rad[axis] ||
        parsed.home_position_rad[axis] >
            parsed.upper_limit_rad[axis]) {
      return false;
    }
  }
  parsed.valid = true;
  *binding = parsed;
  return true;
}

bool observe_rtcore_runtime_mode_status(
    const std::string& payload,
    const std::string& expected_subject,
    RTCoreRuntimeModeTracker* tracker) {
  if (tracker == nullptr) {
    return false;
  }
  const std::string expected_host =
      rtcore_status_host_from_subject(expected_subject);
  std::string schema;
  std::string subject;
  std::string host;
  std::string component;
  std::string schema_raw;
  std::string subject_raw;
  std::string host_raw;
  std::string component_raw;
  std::string status;
  if (expected_host.empty() ||
      !extract_unique_top_level_json_member(payload, "schema", &schema_raw) ||
      !parse_exact_json_string_value(schema_raw, &schema) ||
      schema != "robot.v4.rtcore.status.v1" ||
      !extract_unique_top_level_json_member(payload, "subject", &subject_raw) ||
      !parse_exact_json_string_value(subject_raw, &subject) ||
      subject != expected_subject ||
      !extract_unique_top_level_json_member(payload, "host", &host_raw) ||
      !parse_exact_json_string_value(host_raw, &host) ||
      host != expected_host ||
      !extract_unique_top_level_json_member(payload, "component",
                                            &component_raw) ||
      !parse_exact_json_string_value(component_raw, &component) ||
      component != "rtcore" ||
      !extract_unique_top_level_json_member(payload, "status", &status) ||
      status.empty() || status[0] != '{') {
    return false;
  }

  // Parse authority fields only from the nested RTCore status object, never
  // from lookalike top-level metadata.
  std::string name;
  std::string state;
  std::string runtime_mode;
  std::string name_raw;
  std::string state_raw;
  std::string runtime_mode_raw;
  std::string runtime_mode_epoch_raw;
  uint64_t runtime_mode_epoch = 0;
  if (!extract_unique_top_level_json_member(status, "name", &name_raw) ||
      !parse_exact_json_string_value(name_raw, &name) ||
      name != "robot-v4-rtcore" ||
      !extract_unique_top_level_json_member(status, "state", &state_raw) ||
      !parse_exact_json_string_value(state_raw, &state) ||
      state != "running" ||
      !extract_unique_top_level_json_member(status, "rtcore_mode",
                                            &runtime_mode_raw) ||
      !parse_exact_json_string_value(runtime_mode_raw, &runtime_mode) ||
      !extract_unique_top_level_json_member(status, "runtime_mode_epoch",
                                            &runtime_mode_epoch_raw) ||
      !parse_exact_json_u64_value(runtime_mode_epoch_raw,
                                  &runtime_mode_epoch) ||
      runtime_mode_epoch == 0) {
    return false;
  }
  uint32_t runtime_mode_code = RTCORE_RUNTIME_MODE_SIM;
  if (runtime_mode == "sim") {
    runtime_mode_code = RTCORE_RUNTIME_MODE_SIM;
  } else if (runtime_mode == "live") {
    runtime_mode_code = RTCORE_RUNTIME_MODE_LIVE;
  } else {
    return false;
  }
  RTCorePairConfigBinding pair_config;
  if (parse_rtcore_pair_config_binding(status, &pair_config)) {
    bool home_active = false, go_home_active = false;
    bool home_state_known = true;
    for (auto field : {std::pair<const char*, bool*>{"hm35_home_active", &home_active},
                       {"go_home_active", &go_home_active}}) {
      std::string raw;
      bool present = false;
      if (!extract_unique_top_level_json_member(status, field.first, &raw, &present) ||
          (present && !parse_exact_json_bool_value(raw, field.second))) return false;
      home_state_known = home_state_known && present;
    }
    tracker->observe_authority(
        runtime_mode_code, runtime_mode_epoch, pair_config,
        std::chrono::steady_clock::now(), home_active || go_home_active, home_state_known);
  } else {
    // Preserve the legacy mode/epoch binding for the arm-wire compatibility
    // self-test, but never manufacture pair authority from a partial status.
    // All nonzero motion paths use fresh_authority_snapshot and therefore
    // fail closed until every persisted-URDF field is present and exact.
    tracker->observe(runtime_mode_code, runtime_mode_epoch);
  }
  return true;
}

std::string json_escape(const std::string& value);

std::string robot_command_reply_payload(const std::string& command,
                                        const std::string& command_id_json,
                                        bool accepted, bool duplicate,
                                        const std::string& state,
                                        const std::string& error) {
  std::ostringstream payload;
  payload << "{\"schema\":\"robot.v4.command-reply.v1\","
          << "\"component\":\"motion-server\","
          << "\"command\":\"" << json_escape(command) << "\","
          << "\"command_id\":" << command_id_json << ","
          << "\"accepted\":" << (accepted ? "true" : "false") << ","
          << "\"duplicate\":" << (duplicate ? "true" : "false") << ","
          << "\"state\":\"" << json_escape(state) << "\"";
  if (!error.empty()) {
    payload << ",\"error\":\"" << json_escape(error) << "\"";
  }
  payload << "}";
  return payload.str();
}

bool publish_robot_command_reply_body(int fd, const std::string& reply_subject,
                                      const std::string& body) {
  if (fd < 0 || body.empty() || reply_subject.empty() ||
      reply_subject.find_first_of(" \t\r\n") != std::string::npos) return false;
  std::ostringstream frame;
  frame << "PUB " << reply_subject << " " << body.size() << "\r\n"
        << body << "\r\n";
  return send_all(fd, frame.str());
}

bool publish_robot_command_request_reply(int fd,
                                         const std::string& reply_subject,
                                         const std::string& command,
                                         const std::string& command_id,
                                         bool accepted,
                                         bool duplicate,
                                         const std::string& error = "") {
  return publish_robot_command_reply_body(fd, reply_subject,
      robot_command_reply_payload(command, "\"" + json_escape(command_id) + "\"",
          accepted, duplicate, accepted ? "accepted" : "rejected", error));
}

// Terminal-only: queue acceptance cannot manufacture an arrival receipt.
// This encoder is not subscribed or published automatically before cutover.
std::string position_execution_terminal_reply_payload(const PositionExecution& execution) {
  if (!execution.terminal()) return {};
  std::string text, token;
  if (!parse_position_command_id("{\"command_id\":" + execution.command_id_json() + "}",
                                 &text, &token) || text != execution.command_id() ||
      token != execution.command_id_json()) return {};
  const char* state = "failed";
  const char* error = "position execution failed";
  bool accepted = false;
  switch (execution.outcome()) {
    case PositionExecutionOutcome::completed:
      state = "completed"; error = ""; accepted = true; break;
    case PositionExecutionOutcome::cancelled:
      state = "cancelled"; error = "position execution cancelled"; break;
    case PositionExecutionOutcome::timed_out:
      state = "timed_out"; error = "position execution timed out"; break;
    default: break;
  }
  return robot_command_reply_payload("position", token, accepted, false, state, error);
}

bool motion_server_quiescence_request_matches(
    const std::string& payload,
    const MotionServerQuiescenceReceipt& receipt,
    std::string* error) {
  const auto reject = [&](const std::string& reason) {
    if (error != nullptr) {
      *error = reason;
    }
    return false;
  };
  if (!receipt.valid) {
    return reject("motion server has not drained the current pair-authority revoke");
  }
  const RTCoreRuntimeModeBindingSnapshot& authority = receipt.authority;
  const RTCorePairConfigBinding& pair = authority.pair_config;
  std::string runtime_mode;
  std::string robot_model_id;
  std::string pair_config_sha256;
  std::string generated_urdf_sha256;
  std::string home_profile_key;
  uint64_t runtime_mode_epoch = 0;
  uint64_t pair_revision = 0;
  uint64_t activation_epoch = 0;
  uint64_t home_epoch = 0;
  uint64_t mapped_axis_mask = 0;
  uint64_t home_restore_pending_axis_mask = 1;
  uint64_t home_valid_axis_mask = 0;
  if (!parse_top_level_robot_command_string(
          payload, "runtime_mode", &runtime_mode) ||
      !parse_top_level_robot_command_u64(
          payload, "runtime_mode_epoch", &runtime_mode_epoch) ||
      !parse_top_level_robot_command_string(
          payload, "robot_model_id", &robot_model_id) ||
      !parse_top_level_robot_command_u64(
          payload, "pair_revision", &pair_revision) ||
      !parse_top_level_robot_command_string(
          payload, "pair_config_sha256", &pair_config_sha256) ||
      !parse_top_level_robot_command_string(
          payload, "generated_urdf_sha256", &generated_urdf_sha256) ||
      !parse_top_level_robot_command_string(
          payload, "home_profile_key", &home_profile_key) ||
      !parse_top_level_robot_command_u64(
          payload, "activation_epoch", &activation_epoch) ||
      !parse_top_level_robot_command_u64(
          payload, "home_epoch", &home_epoch) ||
      !parse_top_level_robot_command_u64(
          payload, "mapped_axis_mask", &mapped_axis_mask) ||
      !parse_top_level_robot_command_u64(
          payload, "home_restore_pending_axis_mask",
          &home_restore_pending_axis_mask) ||
      !parse_top_level_robot_command_u64(
          payload, "home_valid_axis_mask", &home_valid_axis_mask)) {
    return reject("motion quiescence request is missing exact runtime, pair, or Home authority");
  }
  if (runtime_mode != "live" ||
      authority.runtime_mode != RTCORE_RUNTIME_MODE_LIVE ||
      runtime_mode_epoch != authority.runtime_mode_epoch ||
      robot_model_id != pair.robot_model_id ||
      pair_revision != pair.revision ||
      pair_config_sha256 != pair.snapshot_sha256 ||
      generated_urdf_sha256 != pair.generated_urdf_sha256 ||
      home_profile_key != pair.home_profile_key ||
      activation_epoch != pair.activation_epoch ||
      home_epoch != pair.home_epoch ||
      mapped_axis_mask != pair.mapped_axis_mask ||
      !pair.home_restore_pending_axis_mask_present ||
      home_restore_pending_axis_mask != 0 ||
      pair.home_restore_pending_axis_mask != 0 ||
      home_valid_axis_mask != pair.home_valid_axis_mask) {
    return reject("motion server has not quiesced on the requested live pair/Home generation");
  }
  return true;
}

bool publish_motion_server_quiescence_reply(
    int fd,
    const std::string& reply_subject,
    const std::string& command_id,
    bool accepted,
    const MotionServerQuiescenceReceipt& receipt,
    const std::string& error) {
  if (fd < 0 || reply_subject.empty() ||
      reply_subject.find_first_of(" \t\r\n") != std::string::npos) {
    return false;
  }
  std::ostringstream payload;
  payload << "{\"schema\":\"robot.v4.command-reply.v1\","
          << "\"component\":\"motion-server\","
          << "\"command\":\"motion_quiescence\","
          << "\"command_id\":\"" << json_escape(command_id) << "\","
          << "\"accepted\":" << (accepted ? "true" : "false") << ","
          << "\"duplicate\":false,"
          << "\"state\":\"" << (accepted ? "ready" : "pending") << "\"";
  if (accepted) {
    const RTCoreRuntimeModeBindingSnapshot& authority = receipt.authority;
    const RTCorePairConfigBinding& pair = authority.pair_config;
    payload << ",\"runtime_mode\":\"live\""
            << ",\"runtime_mode_epoch\":" << authority.runtime_mode_epoch
            << ",\"robot_model_id\":\""
            << json_escape(pair.robot_model_id) << "\""
            << ",\"pair_revision\":" << pair.revision
            << ",\"pair_config_sha256\":\""
            << json_escape(pair.snapshot_sha256) << "\""
            << ",\"generated_urdf_sha256\":\""
            << json_escape(pair.generated_urdf_sha256) << "\""
            << ",\"home_profile_key\":\""
            << json_escape(pair.home_profile_key) << "\""
            << ",\"activation_epoch\":" << pair.activation_epoch
            << ",\"home_epoch\":" << pair.home_epoch
            << ",\"mapped_axis_mask\":" << pair.mapped_axis_mask
            << ",\"home_restore_pending_axis_mask\":"
            << pair.home_restore_pending_axis_mask
            << ",\"home_valid_axis_mask\":"
            << pair.home_valid_axis_mask
            << ",\"pair_authority_revoke_count\":"
            << receipt.pair_authority_revoke_count;
  } else if (!error.empty()) {
    payload << ",\"error\":\"" << json_escape(error) << "\"";
  }
  payload << "}";
  const std::string body = payload.str();
  std::ostringstream frame;
  frame << "PUB " << reply_subject << " " << body.size() << "\r\n"
        << body << "\r\n";
  return send_all(fd, frame.str());
}

bool decode_leader_lease_request(const std::string& payload,
                                 LeaderLeaseRequest* out) {
  if (out == nullptr) {
    return false;
  }
  LeaderLeaseRequest decoded;
  if (!parse_json_string(payload, "command", &decoded.command) ||
      !leader_lease_command_kind(decoded.command) ||
      !parse_json_string(payload, "sender_id", &decoded.sender_id) ||
      !parse_json_string(payload, "controller_boot_id",
                         &decoded.controller_boot_id) ||
      !parse_json_string(payload, "request_id", &decoded.request_id) ||
      !parse_json_u64(payload, "campaign_generation",
                      &decoded.campaign_generation) ||
      decoded.campaign_generation == 0) {
    return false;
  }
  const bool has_lease_id =
      parse_json_string(payload, "lease_id", &decoded.lease_id);
  const bool has_fence_epoch =
      parse_json_u64(payload, "fence_epoch", &decoded.fence_epoch);
  const bool has_ttl = parse_json_int(payload, "ttl_ms", &decoded.ttl_ms);
  if (decoded.command == "leader_acquire") {
    if (!has_ttl || decoded.ttl_ms <= 0 || has_lease_id ||
        has_fence_epoch) {
      return false;
    }
  } else if (decoded.command == "leader_renew") {
    if (!has_ttl || decoded.ttl_ms <= 0 || !has_lease_id ||
        decoded.lease_id.empty() || !has_fence_epoch ||
        decoded.fence_epoch == 0) {
      return false;
    }
  } else if (decoded.command == "leader_release") {
    if (has_ttl || !has_lease_id || decoded.lease_id.empty() ||
        !has_fence_epoch || decoded.fence_epoch == 0) {
      return false;
    }
  } else if (decoded.command == "leader_cancel") {
    if (has_ttl || has_lease_id || has_fence_epoch) {
      return false;
    }
  }
  *out = std::move(decoded);
  return true;
}

bool decode_leader_lease_credential(const std::string& payload,
                                    LeaderLeaseCredential* out) {
  if (out == nullptr) {
    return false;
  }
  LeaderLeaseCredential decoded;
  if (!parse_json_string(payload, "controller_boot_id",
                         &decoded.controller_boot_id) ||
      decoded.controller_boot_id.empty() ||
      !parse_json_string(payload, "leader_lease_id", &decoded.lease_id) ||
      decoded.lease_id.empty() ||
      !parse_json_u64(payload, "leader_fence_epoch",
                      &decoded.fence_epoch) ||
      decoded.fence_epoch == 0) {
    return false;
  }
  *out = std::move(decoded);
  return true;
}

// Normalize fail-safe lifecycle requests before queue admission so ordinary
// saturation cannot reject Disarm. Malformed Arm never becomes a Stop alias.
void normalize_rt_core_stop(RobotCommandMessage& message) {
  bool armed = true;
  bool present = false;
  std::string raw;
  if (message.kind == "disarm" ||
      (message.kind == "arm" &&
       extract_unique_top_level_json_member(message.raw_payload, "armed", &raw, &present) &&
       present && parse_exact_json_bool_value(raw, &armed) && !armed)) {
    message.kind = "stop";
  }
}

// Admission invalidates pending execution before queueing or acknowledging Stop,
// including a saturated queue. Receipt waits cannot retain the prior authority.
void admit_rt_core_command(RobotCommandMessage& message, std::atomic<uint64_t>& generation) {
  normalize_rt_core_stop(message);
  if (message.kind == "stop") generation.fetch_add(1);
  message.planning_generation = generation.load();
}

void run_robot_command_subscriber(const Options& options,
                                  LeaderAwareRobotCommandQueue* queue,
                                  MotionServerQuiescenceBarrier* quiescence,
                                  std::atomic<uint64_t>* planning_generation,
                                  LeaderAwareCommandIDDeduper* command_history) {
  auto& deduper = *command_history;
  while (keep_running.load()) {
    std::string resolved_nats_url;
    if (!resolve_daemon_nats_service_ref(options.nats_url, options.daemon_state_url,
                                         &resolved_nats_url)) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    HostPort target;
    if (!parse_nats_url(resolved_nats_url, &target)) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* results = nullptr;
    if (::getaddrinfo(target.host.c_str(), target.port.c_str(), &hints, &results) != 0) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    int fd = -1;
    for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
      fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
      if (fd < 0) {
        continue;
      }
      timeval timeout{};
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;
      (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
      (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
#ifdef SO_NOSIGPIPE
      int no_sigpipe = 1;
      (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe, sizeof(no_sigpipe));
#endif
      if (::connect(fd, item->ai_addr, item->ai_addrlen) == 0) {
        break;
      }
      ::close(fd);
      fd = -1;
    }
    ::freeaddrinfo(results);
    if (fd < 0) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    std::ostringstream handshake;
    handshake << "CONNECT {\"verbose\":false,\"pedantic\":false,\"lang\":\"cpp\","
              << "\"version\":\"robot-v4-cartesiand-command-sub\"}\r\n"
              << "SUB " << options.robot_command_subject << " 71\r\n";
    if (!send_all(fd, handshake.str())) {
      ::close(fd);
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    std::string stream;
    std::string bridge_reply_subject;
    std::string bridge_reply_kind;
    char chunk[4096];
    bool connected = true;
    while (keep_running.load() && connected) {
      const auto bridge_request = deduper.take_bridge_request();
      if (!bridge_request.body.empty()) {
        bridge_reply_subject = bridge_request.reply_subject;
        bridge_reply_kind = bridge_request.reply_kind;
        std::ostringstream request;
        request << "UNSUB 72\r\nSUB " << bridge_reply_subject << " 72\r\nPUB "
                << bridge_request.subject << " " << bridge_reply_subject << " " << bridge_request.body.size()
                << "\r\n" << bridge_request.body << "\r\n";
        if (!send_all(fd, request.str())) { connected = false; break; }
      }
      for (const auto& reply : deduper.pending_replies()) {
        if (!publish_robot_command_reply_body(fd, reply.subject, reply.body)) {
          connected = false;
          break;
        }
        deduper.delivered(reply);
      }
      if (!connected) break;
      const ssize_t received = ::recv(fd, chunk, sizeof(chunk), 0);
      if (received == 0) {
        connected = false;
        break;
      }
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
          continue;
        }
        connected = false;
        break;
      }
      stream.append(chunk, static_cast<size_t>(received));
      for (;;) {
        const std::string::size_type line_end = stream.find("\r\n");
        if (line_end == std::string::npos) {
          break;
        }
        const std::string line = stream.substr(0, line_end);
        if (line == "PING") {
          stream.erase(0, line_end + 2);
          (void)send_all(fd, "PONG\r\n");
          continue;
        }
        if (line.rfind("MSG ", 0) != 0) {
          stream.erase(0, line_end + 2);
          continue;
        }
        // MSG <subject> <sid> [reply-to] <#bytes>
        std::istringstream header(line.substr(4));
        std::vector<std::string> tokens;
        std::string token;
        while (header >> token) {
          tokens.push_back(token);
        }
        if (tokens.size() < 3) {
          stream.erase(0, line_end + 2);
          continue;
        }
        const std::string reply_subject = tokens.size() == 4 ? tokens[2] : "";
        const size_t payload_bytes = static_cast<size_t>(std::strtoul(tokens.back().c_str(), nullptr, 10));
        if (stream.size() < line_end + 2 + payload_bytes + 2) {
          break;  // wait for the full payload
        }
        const std::string payload = stream.substr(line_end + 2, payload_bytes);
        stream.erase(0, line_end + 2 + payload_bytes + 2);
        if (tokens[1] == "72") {
          if (tokens[0] == bridge_reply_subject) {
            RobotCommandMessage reply;
            reply.kind = bridge_reply_kind;
            reply.raw_payload = payload;
            (void)queue->push(std::move(reply));
          }
          continue;
        }
        if (robot_command_is_record_only_position(payload)) continue;
        RobotCommandMessage message;
        if (!parse_robot_command_message(payload, options.robot_cell, &message)) {
          std::string command;
          std::string command_id;
          std::string command_id_json;
          if (options.backend == "rt_core" && !reply_subject.empty() &&
              parse_json_string(payload, "command", &command) && command == "position" &&
              parse_position_command_id(payload, &command_id, &command_id_json)) {
            (void)publish_robot_command_reply_body(fd, reply_subject,
                robot_command_reply_payload(command, command_id_json, false, false,
                    "failed", "rt_core_position_input_unresolved"));
            continue;
          }
          if (!reply_subject.empty() &&
              parse_json_string(payload, "command", &command) &&
              (robot_command_kind_is_motion_server_owned(command) ||
               leader_lease_command_kind(command))) {
            (void)parse_json_string(payload, "command_id", &command_id);
            (void)publish_robot_command_request_reply(
                fd, reply_subject, command, command_id, false, false,
                "invalid robot command envelope");
          }
          continue;
        }
        const std::string deduplication_key = robot_command_deduplication_key(message);
        if (message.kind == "end_run" || message.kind == "end-run") {
          message.kind = "bridge_run_revoke";
          message.raw_payload = payload;
          (void)queue->push(std::move(message));
          continue;
        }
        if (options.backend != "rt_core" && (message.kind == "arm" || message.kind == "disarm")) {
          bool armed = message.kind == "arm";
          bool present = false;
          std::string raw;
          if (extract_unique_top_level_json_member(payload, "armed", &raw, &present) &&
              (!present || parse_exact_json_bool_value(raw, &armed)) && !armed &&
              !deduper.contains(deduplication_key)) {
            planning_generation->fetch_add(1);
            deduper.remember(deduplication_key);
          }
          // Observe cancellation only. The bridge alone executes and replies to Disarm.
          continue;
        }
        const bool rt_core_lifecycle = options.backend == "rt_core" &&
            (message.kind == "arm" || message.kind == "disarm" || message.kind == "home");
        if (!rt_core_lifecycle && !robot_command_kind_is_motion_server_owned(message.kind) &&
            !leader_lease_command_kind(message.kind)) {
          continue;  // bridge-owned or foreign command kind
        }
        if (message.kind == "motion_quiescence") {
          MotionServerQuiescenceReceipt receipt;
          std::string quiescence_error;
          std::string test_session_id;
          const bool request_identity_valid =
              message.bridge_run.valid() &&
              message.sender_id.rfind("steamdeck:", 0) == 0 &&
              message.bridge_run.controller_id == message.sender_id &&
              parse_top_level_robot_command_string(
                  payload, "test_session_id", &test_session_id) &&
              !test_session_id.empty();
          bool accepted = request_identity_valid &&
                          quiescence != nullptr &&
                          quiescence->request(&receipt);
          if (!request_identity_valid) {
            quiescence_error =
                "motion quiescence requires an exact active Steam Deck bridge-run identity";
          } else if (!accepted) {
            quiescence_error =
                "motion server has not drained the current pair-authority revoke";
          } else {
            accepted = motion_server_quiescence_request_matches(
                payload, receipt, &quiescence_error);
          }
          if (!reply_subject.empty()) {
            (void)publish_motion_server_quiescence_reply(
                fd, reply_subject, message.command_id, accepted,
                receipt, quiescence_error);
          }
          continue;
        }
        if (deduper.contains(deduplication_key)) {
          if (!reply_subject.empty()) {
            const std::string terminal = deduper.position_result(deduplication_key);
            if (robot_command_has_terminal_receipt(message.kind) && !terminal.empty()) {
              (void)publish_robot_command_reply_body(fd, reply_subject, terminal);
            } else if (robot_command_has_terminal_receipt(message.kind)) {
              (void)publish_robot_command_reply_body(fd, reply_subject,
                  robot_command_reply_payload(message.kind, message.command_id_json,
                      true, true, "accepted", ""));
            } else (void)publish_robot_command_request_reply(
                fd, reply_subject, message.kind, message.command_id,
                true, true);
          }
          continue;
        }
        // Keep the authenticated envelope until application time. Authority
        // and cold-motion fences are checked by the state-owning UDP loop,
        // never merely when NATS accepts or queues a message.
        message.raw_payload = payload;
        message.reply_subject = reply_subject;
        message.planning_generation = planning_generation->load();
        const std::string reply_command = message.kind;
        const std::string reply_command_id = message.command_id;
        if (options.backend == "rt_core") admit_rt_core_command(message, *planning_generation);
        const std::string position_id_json = message.command_id_json;
        const bool terminal_receipt = robot_command_has_terminal_receipt(reply_command);
        const bool queued = queue->push(std::move(message));
        if (queued) {
          if (terminal_receipt) deduper.remember_position(deduplication_key, reply_subject);
          else deduper.remember(deduplication_key);
        }
        if (!reply_subject.empty() && terminal_receipt && !queued) {
          (void)publish_robot_command_reply_body(fd, reply_subject,
              robot_command_reply_payload(reply_command, position_id_json, false, false,
                  "rejected", "motion-server command queue full"));
        } else if (!reply_subject.empty() && !terminal_receipt) {
          (void)publish_robot_command_request_reply(
              fd, reply_subject, reply_command, reply_command_id,
              queued, false,
              queued ? "" : "motion-server command queue full");
        }
      }
    }
    planning_generation->fetch_add(1);
    RobotCommandMessage lost;
    lost.kind = "bridge_permit_transport_lost";
    (void)queue->push(std::move(lost));
    ::close(fd);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

bool publish_status_to_nats(const Options& options, const std::string& status_json) {
  return publish_payload_to_nats(options.nats_url,
                                 options.daemon_state_url,
                                 options.nats_status_subject,
                                 status_json,
                                 "status");
}

void publish_status_to_nats_async(const Options& options, const std::string& status_json) {
  static std::atomic<bool> in_flight{false};
  static std::atomic<std::int64_t> next_allowed_ns{0};
  const std::int64_t now_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count();
  const std::int64_t next_ns =
      next_allowed_ns.load(std::memory_order_acquire);
  // The control loop rewrites local status after hot feedback and command
  // events, which can be much faster than the intended 250 ms NATS status
  // cadence. Coalesce those writes so the leaf route cannot accumulate old
  // lease snapshots whose source timestamp plus remaining TTL has expired by
  // the time the RevPi bridge observes them.
  if (status_json.empty() || now_ns < next_ns ||
      in_flight.exchange(true)) {
    return;
  }
  next_allowed_ns.store(
      now_ns +
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::milliseconds(250))
              .count(),
      std::memory_order_release);
  std::thread([options, status_json]() {
    (void)publish_status_to_nats(options, status_json);
    in_flight.store(false);
  }).detach();
}

std::string json_escape(const std::string& value) {
  std::ostringstream out;
  for (char ch : value) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

std::string joystick_frame_name(uint8_t frame) {
  switch (frame) {
    case JOYSTICK_INTENT_FRAME_BASE:
      return "base";
    case JOYSTICK_INTENT_FRAME_TOOL:
      return "tool";
    case JOYSTICK_INTENT_FRAME_JOINT:
      return "joint";
    case JOYSTICK_INTENT_FRAME_HOME:
      return "home";
    case JOYSTICK_INTENT_FRAME_TELEMETRY:
      return "telemetry";
    case JOYSTICK_INTENT_FRAME_ARM:
      return "arm";
    case JOYSTICK_INTENT_FRAME_GO_HOME:
      return "go_home";
    case JOYSTICK_INTENT_FRAME_WELD_IO:
      return "weld_io";
    default:
      return "unknown";
  }
}

std::string sockaddr_to_string(const sockaddr_storage& storage, socklen_t len) {
  char host[NI_MAXHOST]{};
  char service[NI_MAXSERV]{};
  const int rc = ::getnameinfo(reinterpret_cast<const sockaddr*>(&storage), len,
                               host, sizeof(host), service, sizeof(service),
                               NI_NUMERICHOST | NI_NUMERICSERV);
  if (rc != 0) {
    return "";
  }
  if (storage.ss_family == AF_INET6) {
    return std::string("[") + host + "]:" + service;
  }
  return std::string(host) + ":" + service;
}

void append_latency_sample(std::vector<double>* samples, double value) {
  samples->push_back(value);
  while (samples->size() > kMaxLiveLatencySamples) {
    samples->erase(samples->begin());
  }
}

bool split_host_port(const std::string& value, HostPort* out) {
  if (out == nullptr) {
    return false;
  }
  if (!value.empty() && value.front() == ':' && value.find(':', 1) == std::string::npos) {
    out->host = "0.0.0.0";
    out->port = value.substr(1);
    return !out->port.empty();
  }
  return network_authority_host_port(value, true, "", out, true);
}

bool parse_positive_int_arg(const char* value, int* out) {
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' ||
      parsed < 1 || parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  *out = static_cast<int>(parsed);
  return true;
}

bool parse_json_int(const std::string& text, const std::string& key, int* out) {
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::string::size_type colon = text.find(':', key_pos + marker.size());
  if (colon == std::string::npos) {
    return false;
  }
  const char* start = text.c_str() + colon + 1;
  char* end = nullptr;
  errno = 0;
  const long parsed = std::strtol(start, &end, 10);
  if (errno != 0 || end == start || parsed < 0 || parsed > std::numeric_limits<int>::max()) {
    return false;
  }
  *out = static_cast<int>(parsed);
  return true;
}

bool parse_json_double(const std::string& text, const std::string& key, double* out) {
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::string::size_type colon = text.find(':', key_pos + marker.size());
  if (colon == std::string::npos) {
    return false;
  }
  const char* start = text.c_str() + colon + 1;
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(start, &end);
  if (errno != 0 || end == start || !std::isfinite(parsed)) {
    return false;
  }
  *out = parsed;
  return true;
}

bool parse_json_u64(const std::string& text, const std::string& key, uint64_t* out) {
  if (out == nullptr) {
    return false;
  }
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::string::size_type colon = text.find(':', key_pos + marker.size());
  if (colon == std::string::npos) {
    return false;
  }
  const std::string::size_type value_pos =
      text.find_first_not_of(" \t\r\n", colon + 1);
  if (value_pos == std::string::npos || text[value_pos] < '0' ||
      text[value_pos] > '9') {
    return false;
  }
  const char* start = text.c_str() + value_pos;
  char* end = nullptr;
  errno = 0;
  const unsigned long long parsed = std::strtoull(start, &end, 10);
  if (errno != 0 || end == start) {
    return false;
  }
  while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') {
    ++end;
  }
  if (*end != ',' && *end != '}') {
    return false;
  }
  *out = static_cast<uint64_t>(parsed);
  return true;
}

bool parse_json_bool(const std::string& text, const std::string& key, bool* out) {
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::string::size_type colon = text.find(':', key_pos + marker.size());
  if (colon == std::string::npos) {
    return false;
  }
  const std::string::size_type value = text.find_first_not_of(" \t\r\n", colon + 1);
  if (value == std::string::npos) {
    return false;
  }
  if (text.compare(value, 4, "true") == 0) {
    *out = true;
    return true;
  }
  if (text.compare(value, 5, "false") == 0) {
    *out = false;
    return true;
  }
  return false;
}

bool parse_json_string(const std::string& text, const std::string& key, std::string* out) {
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::string::size_type colon = text.find(':', key_pos + marker.size());
  if (colon == std::string::npos) {
    return false;
  }
  const std::string::size_type open = text.find('"', colon + 1);
  if (open == std::string::npos) {
    return false;
  }
  std::string value;
  bool escaped = false;
  for (std::string::size_type index = open + 1; index < text.size(); ++index) {
    const char ch = text[index];
    if (escaped) {
      value.push_back(ch);
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      *out = value;
      return true;
    }
    value.push_back(ch);
  }
  return false;
}

std::vector<double> parse_json_number_array(const std::string& text, const std::string& key) {
  std::vector<double> values;
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return values;
  }
  const std::string::size_type open = text.find('[', key_pos + marker.size());
  const std::string::size_type close = text.find(']', open);
  if (open == std::string::npos || close == std::string::npos || close <= open) {
    return values;
  }
  const std::string vector_text = text.substr(open + 1, close - open - 1);
  const char* cursor = vector_text.c_str();
  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r' || *cursor == ',') {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(cursor, &end);
    if (errno != 0 || end == cursor) {
      return {};
    }
    values.push_back(parsed);
    cursor = end;
  }
  return values;
}

bool parse_array6(const std::string& text, const std::string& key, double out[6]) {
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::string::size_type open = text.find('[', key_pos + marker.size());
  const std::string::size_type close = text.find(']', open);
  if (open == std::string::npos || close == std::string::npos || close <= open) {
    return false;
  }
  const std::string vector_text = text.substr(open + 1, close - open - 1);
  const char* cursor = vector_text.c_str();
  for (int i = 0; i < 6; ++i) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == ',') {
      ++cursor;
    }
    char* end = nullptr;
    errno = 0;
    const double parsed = std::strtod(cursor, &end);
    if (errno != 0 || end == cursor) {
      return false;
    }
    out[i] = parsed;
    cursor = end;
  }
  return true;
}

bool parse_cartesian_command_text(const std::string& raw, CartesianCommand* command) {
  int seq = 0;
  if (parse_json_int(raw, "seq", &seq)) {
    command->seq = seq;
  }
  int timeout_ms = 0;
  if (parse_json_int(raw, "timeout_ms", &timeout_ms)) {
    command->timeout_ms = timeout_ms;
  }
  if (command->timeout_ms <= 0) {
    std::cerr << "robot-v4-cartesiand: timeout_ms must be positive\n";
    return false;
  }
  if (!parse_array6(raw, "velocity_vector", command->velocity_vector)) {
    std::cerr << "robot-v4-cartesiand: input must include six-value velocity_vector\n";
    return false;
  }
  double q_seed[6]{};
  if (parse_array6(raw, "joint_position_rad", q_seed)) {
    for (int i = 0; i < 6; ++i) {
      command->joint_position_rad[i] = q_seed[i];
    }
  }
  return true;
}

bool load_cartesian_command(const std::string& path, CartesianCommand* command) {
  std::string raw;
  if (!read_text_file(path, &raw)) {
    return false;
  }
  return parse_cartesian_command_text(raw, command);
}

void put_u16_be(std::vector<unsigned char>* packet, size_t offset, uint16_t value) {
  (*packet)[offset] = static_cast<unsigned char>((value >> 8) & 0xffu);
  (*packet)[offset + 1] = static_cast<unsigned char>(value & 0xffu);
}

void put_u32_be(std::vector<unsigned char>* packet, size_t offset, uint32_t value) {
  (*packet)[offset] = static_cast<unsigned char>((value >> 24) & 0xffu);
  (*packet)[offset + 1] = static_cast<unsigned char>((value >> 16) & 0xffu);
  (*packet)[offset + 2] = static_cast<unsigned char>((value >> 8) & 0xffu);
  (*packet)[offset + 3] = static_cast<unsigned char>(value & 0xffu);
}

void put_u64_be(std::vector<unsigned char>* packet, size_t offset, uint64_t value) {
  for (int byte = 7; byte >= 0; --byte) {
    (*packet)[offset + static_cast<size_t>(7 - byte)] =
        static_cast<unsigned char>((value >> (byte * 8)) & 0xffu);
  }
}

uint16_t read_u16_be(const unsigned char* packet, size_t offset) {
  return static_cast<uint16_t>((static_cast<uint16_t>(packet[offset]) << 8) |
                               static_cast<uint16_t>(packet[offset + 1]));
}

uint32_t read_u32_be(const unsigned char* packet, size_t offset) {
  return (static_cast<uint32_t>(packet[offset]) << 24) |
         (static_cast<uint32_t>(packet[offset + 1]) << 16) |
         (static_cast<uint32_t>(packet[offset + 2]) << 8) |
         static_cast<uint32_t>(packet[offset + 3]);
}

uint64_t read_u64_be(const unsigned char* packet, size_t offset) {
  uint64_t value = 0;
  for (int byte = 0; byte < 8; ++byte) {
    value = (value << 8) | static_cast<uint64_t>(packet[offset + static_cast<size_t>(byte)]);
  }
  return value;
}

uint32_t float32_bits(float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

float float32_from_bits(uint32_t bits) {
  float value = 0.0f;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

void put_f32_be(std::vector<unsigned char>* packet, size_t offset, double value) {
  put_u32_be(packet, offset, float32_bits(static_cast<float>(value)));
}

double read_f32_be(const unsigned char* packet, size_t offset) {
  return static_cast<double>(float32_from_bits(read_u32_be(packet, offset)));
}

uint32_t duration_us_u32(std::chrono::steady_clock::duration duration) {
  const double elapsed_us = std::chrono::duration<double, std::micro>(duration).count();
  if (elapsed_us <= 0.0 || !std::isfinite(elapsed_us)) {
    return 0;
  }
  if (elapsed_us >= static_cast<double>(std::numeric_limits<uint32_t>::max())) {
    return std::numeric_limits<uint32_t>::max();
  }
  return static_cast<uint32_t>(std::llround(elapsed_us));
}

uint32_t timespec_queue_age_us_u32(const timespec& packet_time) {
#if defined(CLOCK_REALTIME)
  timespec now{};
  if (::clock_gettime(CLOCK_REALTIME, &now) != 0) {
    return 0;
  }
  const int64_t sec_delta = static_cast<int64_t>(now.tv_sec) - static_cast<int64_t>(packet_time.tv_sec);
  const int64_t nsec_delta = static_cast<int64_t>(now.tv_nsec) - static_cast<int64_t>(packet_time.tv_nsec);
  const int64_t total_us = sec_delta * 1000000LL + nsec_delta / 1000LL;
  if (total_us <= 0) {
    return 0;
  }
  if (total_us >= static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
    return std::numeric_limits<uint32_t>::max();
  }
  return static_cast<uint32_t>(total_us);
#else
  (void)packet_time;
  return 0;
#endif
}

void enable_udp_receive_timestamps(int fd) {
#if defined(SO_TIMESTAMPNS)
  int enabled = 1;
  (void)::setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled));
#else
  (void)fd;
#endif
}

ssize_t recvfrom_with_metadata(int fd,
                               unsigned char* buffer,
                               size_t buffer_size,
                               sockaddr_storage* peer,
                               socklen_t* peer_len,
                               UdpReceiveMetadata* metadata) {
  if (metadata != nullptr) {
    *metadata = UdpReceiveMetadata{};
  }
#if defined(SO_TIMESTAMPNS) && defined(SCM_TIMESTAMPNS)
  iovec iov{};
  iov.iov_base = buffer;
  iov.iov_len = buffer_size;
  char control[CMSG_SPACE(sizeof(timespec))]{};
  msghdr message{};
  message.msg_name = peer;
  message.msg_namelen = *peer_len;
  message.msg_iov = &iov;
  message.msg_iovlen = 1;
  message.msg_control = control;
  message.msg_controllen = sizeof(control);
  const ssize_t received = ::recvmsg(fd, &message, 0);
  if (received < 0) {
    return received;
  }
  *peer_len = message.msg_namelen;
  for (cmsghdr* cmsg = CMSG_FIRSTHDR(&message); cmsg != nullptr; cmsg = CMSG_NXTHDR(&message, cmsg)) {
    if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SCM_TIMESTAMPNS &&
        cmsg->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
      timespec packet_time{};
      std::memcpy(&packet_time, CMSG_DATA(cmsg), sizeof(packet_time));
      if (metadata != nullptr) {
        metadata->kernel_queue_us = timespec_queue_age_us_u32(packet_time);
        metadata->kernel_timestamp_available = true;
      }
      break;
    }
  }
  return received;
#else
  return ::recvfrom(fd, buffer, buffer_size, 0,
                    reinterpret_cast<sockaddr*>(peer), peer_len);
#endif
}

bool decode_servo_feedback_v1(const unsigned char* data, size_t size, ServoFeedbackV1* feedback) {
  if (data == nullptr || feedback == nullptr || size != sizeof(ServoFeedbackV1)) {
    return false;
  }
  ServoFeedbackV1 decoded{};
  std::memcpy(&decoded, data, sizeof(decoded));
  if (decoded.magic != SERVO_FEEDBACK_PACKET_MAGIC ||
      decoded.version != SERVO_FEEDBACK_PACKET_VERSION_V1 ||
      decoded.axis_count == 0 ||
      decoded.axis_count > kLiveAxisCount) {
    return false;
  }
  *feedback = decoded;
  return true;
}

template <size_t N>
bool servo_feedback_fixed_token(
    const char (&value)[N],
    std::string* out) {
  if (out == nullptr) {
    return false;
  }
  size_t length = 0;
  for (; length < N && value[length] != '\0'; ++length) {
    const unsigned char ch =
        static_cast<unsigned char>(value[length]);
    if (!((ch >= 'a' && ch <= 'z') ||
          (ch >= 'A' && ch <= 'Z') ||
          (ch >= '0' && ch <= '9') ||
          ch == '-' || ch == '_' || ch == '.' ||
          ch == ':' || ch == '/')) {
      return false;
    }
  }
  if (length == 0 || length == N) {
    return false;
  }
  for (size_t index = length; index < N; ++index) {
    if (value[index] != '\0') {
      return false;
    }
  }
  *out = std::string(value, value + length);
  return true;
}

bool servo_feedback_fixed_lowercase_sha256(
    const char (&value)[64],
    std::string* out) {
  if (out == nullptr) {
    return false;
  }
  for (const char ch : value) {
    if (!((ch >= '0' && ch <= '9') ||
          (ch >= 'a' && ch <= 'f'))) {
      return false;
    }
  }
  *out = std::string(value, value + 64);
  return true;
}

bool servo_feedback_home_profile_key_value(
    const std::string& value,
    uint64_t* out) {
  if (out == nullptr || !lowercase_hex_string(value, 16)) {
    return false;
  }
  uint64_t parsed = 0;
  for (const char ch : value) {
    const int nibble = json_hex_value(ch);
    if (nibble < 0) {
      return false;
    }
    parsed =
        (parsed << 4) | static_cast<uint64_t>(nibble);
  }
  if (parsed == 0) {
    return false;
  }
  *out = parsed;
  return true;
}

bool servo_feedback_pair_binding_valid(
    const ServoFeedbackPairBindingV4& binding) {
  std::string jetson_id;
  std::string robot_model_id;
  std::string snapshot_sha256;
  std::string generated_urdf_sha256;
  return binding.revision != 0 &&
         binding.activation_epoch != 0 &&
         binding.home_epoch != 0 &&
         binding.home_profile_key != 0 &&
         binding.runtime_mode_epoch != 0 &&
         binding.mapped_axis_mask != 0 &&
         (binding.mapped_axis_mask &
          ~RTCORE_MAX_LIVE_AXIS_MASK) == 0u &&
         binding.reserved == 0u &&
         servo_feedback_fixed_token(
             binding.jetson_id, &jetson_id) &&
         servo_feedback_fixed_token(
             binding.robot_model_id, &robot_model_id) &&
         servo_feedback_fixed_lowercase_sha256(
             binding.snapshot_sha256, &snapshot_sha256) &&
         servo_feedback_fixed_lowercase_sha256(
             binding.generated_urdf_sha256,
             &generated_urdf_sha256);
}

bool servo_feedback_pair_binding_equal(
    const ServoFeedbackPairBindingV4& left,
    const ServoFeedbackPairBindingV4& right) {
  return left.revision == right.revision &&
         left.activation_epoch == right.activation_epoch &&
         left.home_epoch == right.home_epoch &&
         left.home_profile_key == right.home_profile_key &&
         left.runtime_mode_epoch == right.runtime_mode_epoch &&
         left.mapped_axis_mask == right.mapped_axis_mask &&
         left.reserved == right.reserved &&
         std::memcmp(
             left.jetson_id, right.jetson_id,
             sizeof(left.jetson_id)) == 0 &&
         std::memcmp(
             left.robot_model_id, right.robot_model_id,
             sizeof(left.robot_model_id)) == 0 &&
         std::memcmp(
             left.snapshot_sha256, right.snapshot_sha256,
             sizeof(left.snapshot_sha256)) == 0 &&
         std::memcmp(
             left.generated_urdf_sha256,
             right.generated_urdf_sha256,
             sizeof(left.generated_urdf_sha256)) == 0;
}

bool servo_feedback_pair_binding_matches_authority(
    const ServoFeedbackPairBindingV4& binding,
    const RTCoreRuntimeModeBindingSnapshot& authority) {
  const RTCorePairConfigBinding& pair = authority.pair_config;
  if (!pair.valid ||
      !servo_feedback_pair_binding_valid(binding)) {
    return false;
  }
  std::string jetson_id;
  std::string robot_model_id;
  std::string snapshot_sha256;
  std::string generated_urdf_sha256;
  uint64_t home_profile_key = 0;
  return servo_feedback_fixed_token(
             binding.jetson_id, &jetson_id) &&
         servo_feedback_fixed_token(
             binding.robot_model_id, &robot_model_id) &&
         servo_feedback_fixed_lowercase_sha256(
             binding.snapshot_sha256, &snapshot_sha256) &&
         servo_feedback_fixed_lowercase_sha256(
             binding.generated_urdf_sha256,
             &generated_urdf_sha256) &&
         servo_feedback_home_profile_key_value(
             pair.home_profile_key,
             &home_profile_key) &&
         binding.runtime_mode_epoch ==
             authority.runtime_mode_epoch &&
         binding.revision == pair.revision &&
         binding.activation_epoch ==
             pair.activation_epoch &&
         binding.home_epoch == pair.home_epoch &&
         binding.home_profile_key == home_profile_key &&
         binding.mapped_axis_mask ==
             pair.mapped_axis_mask &&
         jetson_id == pair.jetson_id &&
         robot_model_id == pair.robot_model_id &&
         snapshot_sha256 == pair.snapshot_sha256 &&
         generated_urdf_sha256 ==
             pair.generated_urdf_sha256;
}

bool servo_feedback_packet_valid(const unsigned char* data, size_t size) {
  if (data == nullptr) {
    return false;
  }
  if (size == kServoFeedbackV1PacketBytes) {
    ServoFeedbackV1 feedback{};
    return decode_servo_feedback_v1(data, size, &feedback);
  }
  if (size == kServoFeedbackV2PacketBytes) {
    ServoFeedbackV2 feedback{};
    std::memcpy(&feedback, data, sizeof(feedback));
    return feedback.magic == SERVO_FEEDBACK_PACKET_MAGIC &&
           feedback.version ==
               SERVO_FEEDBACK_PACKET_VERSION_V2 &&
           feedback.axis_count > 0 &&
           feedback.axis_count <= kLiveAxisCount &&
           (feedback.home_valid_axis_mask &
            ~rtcore_live_axis_mask()) == 0u;
  }
  if (size != kServoFeedbackV3PacketBytes &&
      size != kServoFeedbackV4PacketBytes) {
    return false;
  }
  ServoFeedbackV3 feedback{};
  std::memcpy(&feedback, data, sizeof(feedback));
  const uint8_t expected_version =
      size == kServoFeedbackV4PacketBytes
          ? SERVO_FEEDBACK_PACKET_VERSION_V4
          : SERVO_FEEDBACK_PACKET_VERSION_V3;
  if (feedback.magic != SERVO_FEEDBACK_PACKET_MAGIC ||
      feedback.version != expected_version ||
      feedback.axis_count == 0 ||
      feedback.axis_count > kLiveAxisCount ||
      (feedback.home_valid_axis_mask &
       ~rtcore_live_axis_mask()) != 0u ||
      std::any_of(
          std::begin(feedback.reserved),
          std::end(feedback.reserved),
          [](uint8_t value) { return value != 0u; })) {
    return false;
  }
  if (size == kServoFeedbackV3PacketBytes) {
    return true;
  }
  ServoFeedbackV4 bound{};
  std::memcpy(&bound, data, sizeof(bound));
  return servo_feedback_pair_binding_valid(bound.binding) &&
         (feedback.home_valid_axis_mask &
          ~bound.binding.mapped_axis_mask) == 0u;
}

double rtcore_axis_counts_per_rad(size_t axis) {
  if (axis >= kLiveAxisCount) {
    return 0.0;
  }
  RTCoreRuntimeModeBindingSnapshot authority;
  if (!fresh_motion_runtime_authority_snapshot(&authority) ||
      authority.pair_config.configuration_only ||
      !authority.pair_config.drive_ratio_converged) {
    return 0.0;
  }
  const bool exact_live_readback =
      authority.pair_config.ethercat_live &&
      authority.pair_config.ratio_reconciliation_state == "succeeded" &&
      !authority.pair_config.ratio_reconciliation_simulated;
  const bool exact_simulated_readback =
      authority.runtime_mode == RTCORE_RUNTIME_MODE_SIM &&
      !authority.pair_config.ethercat_live &&
      authority.pair_config.ratio_reconciliation_state == "simulated" &&
      authority.pair_config.ratio_reconciliation_simulated;
  const uint16_t numerator =
      authority.pair_config.gear_numerator[axis];
  const uint16_t denominator =
      authority.pair_config.gear_denominator[axis];
  if ((!exact_live_readback && !exact_simulated_readback) ||
      numerator == 0 || denominator == 0) {
    return 0.0;
  }
  const double exact_ratio =
      static_cast<double>(numerator) /
      static_cast<double>(denominator);
  return (static_cast<double>(RTCORE_COUNTS_PER_REV) * exact_ratio) /
         (2.0 * kPi);
}

double rtcore_axis_counts_per_rad_from_authority(
    const RTCoreRuntimeModeBindingSnapshot& authority,
    size_t axis) {
  if (axis >= kLiveAxisCount ||
      !rtcore_runtime_authority_has_exact_ratio(authority) ||
      (authority.pair_config.mapped_axis_mask &
       (1u << static_cast<uint32_t>(axis))) == 0u) {
    return 0.0;
  }
  const uint16_t numerator =
      authority.pair_config.gear_numerator[axis];
  const uint16_t denominator =
      authority.pair_config.gear_denominator[axis];
  if (numerator == 0 || denominator == 0) {
    return 0.0;
  }
  const double exact_ratio =
      static_cast<double>(numerator) /
      static_cast<double>(denominator);
  return (static_cast<double>(RTCORE_COUNTS_PER_REV) * exact_ratio) /
         (2.0 * kPi);
}

double rtcore_position_counts_to_joint_rad_from_authority(
    const RTCoreRuntimeModeBindingSnapshot& authority,
    size_t axis,
    int32_t counts,
    int32_t home_counts) {
  if (axis >= kLiveAxisCount) {
    return 0.0;
  }
  const double counts_per_rad =
      rtcore_axis_counts_per_rad_from_authority(authority, axis);
  const double signed_counts_per_rad =
      counts_per_rad * rtcore_axis_sign(axis);
  const double rotary_frame = counts_per_rad * 2.0 * kPi;
  if (signed_counts_per_rad == 0.0 ||
      !std::isfinite(rotary_frame) || rotary_frame <= 1.0) {
    return 0.0;
  }
  double relative_counts =
      static_cast<double>(counts) -
      static_cast<double>(home_counts);
  if (!std::isfinite(relative_counts)) {
    return 0.0;
  }
  relative_counts = std::fmod(relative_counts, rotary_frame);
  const double half_frame = rotary_frame * 0.5;
  if (relative_counts > half_frame) {
    relative_counts -= rotary_frame;
  } else if (relative_counts < -half_frame) {
    relative_counts += rotary_frame;
  }
  const double rad = relative_counts / signed_counts_per_rad;
  return std::max(-kPi, std::min(kPi, rad));
}

double rtcore_axis_rotary_frame_counts(size_t axis) {
  return rtcore_axis_counts_per_rad(axis) * 2.0 * kPi;
}

double rtcore_shortest_axis_delta_counts(size_t axis, double counts, double home_counts) {
  const double rotary_frame = rtcore_axis_rotary_frame_counts(axis);
  double delta = counts - home_counts;
  if (!std::isfinite(delta) || !std::isfinite(rotary_frame) || rotary_frame <= 1.0) {
    return delta;
  }
  delta = std::fmod(delta, rotary_frame);
  const double half_frame = rotary_frame * 0.5;
  if (delta > half_frame) {
    delta -= rotary_frame;
  } else if (delta < -half_frame) {
    delta += rotary_frame;
  }
  return delta;
}

double rtcore_position_counts_to_joint_rad(size_t axis, int32_t counts, int32_t home_counts) {
  if (axis >= kLiveAxisCount) {
    return 0.0;
  }
  const double signed_counts_per_rad = rtcore_axis_counts_per_rad(axis) * rtcore_axis_sign(axis);
  if (signed_counts_per_rad == 0.0) {
    return 0.0;
  }
  const double relative_counts =
      rtcore_shortest_axis_delta_counts(axis, static_cast<double>(counts), static_cast<double>(home_counts));
  const double rad = relative_counts / signed_counts_per_rad;
  return std::max(-kPi, std::min(kPi, rad));
}

bool decode_servo_feedback_seed(
    const unsigned char* data,
    size_t size,
    const RTCoreRuntimeModeBindingSnapshot& authority,
    ServoFeedbackSeed* seed) {
  if (data == nullptr || seed == nullptr ||
      size != kServoFeedbackV4PacketBytes ||
      !servo_feedback_packet_valid(data, size)) {
    return false;
  }
  ServoFeedbackV4 packet{};
  std::memcpy(&packet, data, sizeof(packet));
  const ServoFeedbackV3& feedback = packet.feedback;
  if (feedback.version !=
          SERVO_FEEDBACK_PACKET_VERSION_V4 ||
      feedback.axis_count < 6 ||
      feedback.axis_count > kLiveAxisCount ||
      feedback.monotonic_ns == 0 ||
      (feedback.home_valid_axis_mask & 0x3fu) != 0x3fu ||
      (feedback.home_valid_axis_mask & ~rtcore_live_axis_mask()) != 0u ||
      !rtcore_runtime_authority_has_exact_ratio(authority) ||
      !servo_feedback_pair_binding_matches_authority(
          packet.binding, authority)) {
    return false;
  }
  if ((0x3fu &
       ~authority.pair_config.mapped_axis_mask) != 0u ||
      feedback.home_valid_axis_mask !=
          authority.pair_config.home_valid_axis_mask) {
    return false;
  }
  ServoFeedbackSeed decoded{};
  decoded.valid = true;
  decoded.axis_count = feedback.axis_count;
  decoded.home_valid_axis_mask = feedback.home_valid_axis_mask & rtcore_live_axis_mask();
  decoded.packet_monotonic_ns = feedback.monotonic_ns;
  decoded.packet_binding = packet.binding;
  decoded.authority_binding = authority;
  for (size_t axis = 0; axis < feedback.axis_count; ++axis) {
    const double frame = rtcore_axis_counts_per_rad_from_authority(authority, axis) * 2 * kPi;
    if (frame > 0 && feedback.axis_pos_counts[axis] >= 0 && feedback.axis_pos_counts[axis] < frame)
      decoded.rotary_frame_valid_axis_mask |= 1u << axis;
    decoded.joint_position_rad[axis] =
        rtcore_position_counts_to_joint_rad_from_authority(
            authority, axis, feedback.axis_pos_counts[axis],
            feedback.axis_home_counts[axis]);
  }
  decoded.fault_fields_valid = true;
  decoded.axis_fault_mask = feedback.axis_fault_mask;
  for (size_t axis = 0; axis < feedback.axis_count; ++axis) {
    // CiA 402 operation-enabled: (statusword & 0x6f) == 0x27. A disarmed
    // drive drops out of operation enabled even with no fault bit set.
    if ((feedback.axis_statusword[axis] & 0x6f) == 0x27) {
      decoded.operation_enabled_axis_mask |= 1u << axis;
    }
  }
  *seed = decoded;
  return true;
}

bool servo_feedback_seed_matches_authority(
    const ServoFeedbackSeed& seed,
    const RTCoreRuntimeModeBindingSnapshot& authority) {
  return seed.valid &&
         seed.packet_monotonic_ns != 0 &&
         servo_feedback_pair_binding_matches_authority(
             seed.packet_binding, authority) &&
         rtcore_runtime_authority_identity_matches(
             seed.authority_binding, authority);
}


// Diagnostic only: the existing admission predicate remains authoritative.
const char* servo_feedback_admission_diagnostic(
    const ServoFeedbackSeed& seed,
    const RTCoreRuntimeModeBindingSnapshot& authority) {
  if (!seed.valid) return "seed_invalid";
  if (seed.packet_monotonic_ns == 0) return "packet_timestamp_missing";
  if (!authority.pair_config.valid) return "authority_pair_invalid";
  const auto& binding = seed.packet_binding;
  const auto& pair = authority.pair_config;
  if (!servo_feedback_pair_binding_valid(binding)) return "packet_binding_invalid";
  if (binding.runtime_mode_epoch != authority.runtime_mode_epoch) return "packet_runtime_epoch";
  if (binding.revision != pair.revision) return "packet_revision";
  if (binding.activation_epoch != pair.activation_epoch) return "packet_activation_epoch";
  if (binding.home_epoch != pair.home_epoch) return "packet_home_epoch";
  uint64_t home_key = 0;
  if (!servo_feedback_home_profile_key_value(pair.home_profile_key, &home_key) ||
      binding.home_profile_key != home_key) return "packet_home_profile_key";
  if (binding.mapped_axis_mask != pair.mapped_axis_mask) return "packet_axis_mask";
  if (!servo_feedback_pair_binding_matches_authority(binding, authority)) return "packet_model_or_snapshot_identity";
  const auto& captured = seed.authority_binding;
  const auto& old = captured.pair_config;
  if (captured.runtime_mode != authority.runtime_mode) return "captured_runtime_mode";
  if (captured.runtime_mode_epoch != authority.runtime_mode_epoch) return "captured_runtime_epoch";
  if (!rtcore_pair_config_identity_matches(old, pair)) return "captured_pair_identity";
  if (old.drive_ratio_converged != pair.drive_ratio_converged) return "captured_drive_ratio_converged";
  if (old.ratio_reconciliation_simulated != pair.ratio_reconciliation_simulated) return "captured_ratio_reconciliation_simulated";
  if (old.ratio_reconciliation_state != pair.ratio_reconciliation_state) return "captured_ratio_reconciliation_state";
  if (old.home_restore_pending_axis_mask_present != pair.home_restore_pending_axis_mask_present) return "captured_home_pending_presence";
  if (old.home_restore_pending_axis_mask != pair.home_restore_pending_axis_mask) return "captured_home_pending_mask";
  if (old.home_valid_axis_mask != pair.home_valid_axis_mask) return "captured_home_valid_mask";
  if (old.ethercat_live != pair.ethercat_live) return "captured_ethercat_live";
  if (!seed.fault_fields_valid) return "fault_fields_invalid";
  return "matched";
}

bool servo_feedback_seed_monotonic_is_newer(
    const ServoFeedbackSeed& candidate,
    bool have_watermark,
    const ServoFeedbackPairBindingV4& watermark_binding,
    uint64_t watermark_monotonic_ns) {
  if (!candidate.valid ||
      candidate.packet_monotonic_ns == 0) {
    return false;
  }
  if (!have_watermark ||
      !servo_feedback_pair_binding_equal(
          candidate.packet_binding,
          watermark_binding)) {
    return true;
  }
  return candidate.packet_monotonic_ns >
         watermark_monotonic_ns;
}

double clamp_value(double value, double low, double high);

uint64_t unix_time_ns() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::vector<unsigned char> encode_steamdeck_joystick_intent(const CartesianCommand& command) {
  const std::string tool_id = "robot-v4-tool";
  std::vector<unsigned char> packet(kJoystickIntentHeaderBytes + tool_id.size(), 0);
  put_u16_be(&packet, 0, JOYSTICK_INTENT_PACKET_MAGIC);
  packet[2] = JOYSTICK_INTENT_PACKET_VERSION;

  size_t offset = 3;
  put_u64_be(&packet, offset, unix_time_ns());
  offset += 8;
  put_u32_be(&packet, offset, static_cast<uint32_t>(command.seq));
  offset += 4;
  packet[offset++] = JOYSTICK_INTENT_FRAME_TOOL;
  packet[offset++] = static_cast<unsigned char>(tool_id.size());

  for (int axis = 0; axis < 6; ++axis) {
    const double limit = axis < 3 ? MAX_JOG_LINEAR_M_S : MAX_JOG_ANGULAR_RAD_S;
    put_f32_be(&packet, offset, clamp_value(command.velocity_vector[axis] / limit, -1.0, 1.0));
    offset += 4;
  }
  put_f32_be(&packet, offset, 1.0);
  offset += 4;
  packet[offset++] = 1;

  for (int axis = 0; axis < 6; ++axis) {
    put_f32_be(&packet, offset, 0.0);
    offset += 4;
  }
  std::memcpy(packet.data() + offset, tool_id.data(), tool_id.size());
  return packet;
}

bool decode_steamdeck_joystick_intent(const unsigned char* packet, size_t size,
                                      SteamDeckJoystickIntent* intent) {
  if (size < kJoystickIntentHeaderBytes || size > kMaxJoystickIntentPacketBytes) {
    return false;
  }
  const uint8_t packet_version = packet[2];
  if (read_u16_be(packet, 0) != JOYSTICK_INTENT_PACKET_MAGIC ||
      (packet_version != JOYSTICK_INTENT_PACKET_VERSION &&
       packet_version != JOYSTICK_INTENT_PACKET_VERSION_V2)) {
    return false;
  }

  size_t offset = 3;
  intent->sample_timestamp_ns = read_u64_be(packet, offset);
  offset += 8;
  intent->sequence = read_u32_be(packet, offset);
  offset += 4;
  intent->frame = packet[offset++];
  const size_t tool_id_len = packet[offset++];
  if ((intent->frame != JOYSTICK_INTENT_FRAME_BASE &&
       intent->frame != JOYSTICK_INTENT_FRAME_TOOL &&
       intent->frame != JOYSTICK_INTENT_FRAME_JOINT &&
       intent->frame != JOYSTICK_INTENT_FRAME_HOME &&
       intent->frame != JOYSTICK_INTENT_FRAME_TELEMETRY &&
       intent->frame != JOYSTICK_INTENT_FRAME_ARM &&
       intent->frame != JOYSTICK_INTENT_FRAME_GO_HOME &&
       intent->frame != JOYSTICK_INTENT_FRAME_WELD_IO) ||
      tool_id_len > kMaxJoystickIntentToolIDBytes) {
    return false;
  }
  const size_t legacy_size = kJoystickIntentHeaderBytes + tool_id_len;
  if ((packet_version == JOYSTICK_INTENT_PACKET_VERSION &&
       size != legacy_size) ||
      (packet_version == JOYSTICK_INTENT_PACKET_VERSION_V2 &&
       size < legacy_size + 10)) {
    return false;
  }

  for (int axis = 0; axis < 6; ++axis) {
    intent->axes[axis] = read_f32_be(packet, offset);
    if (!std::isfinite(intent->axes[axis])) {
      return false;
    }
    offset += 4;
  }
  intent->speed_scale = read_f32_be(packet, offset);
  if (!std::isfinite(intent->speed_scale)) {
    return false;
  }
  offset += 4;
  intent->deadman = packet[offset++] != 0;

  for (int axis = 0; axis < 6; ++axis) {
    intent->tool_tcp[axis] = read_f32_be(packet, offset);
    if (!std::isfinite(intent->tool_tcp[axis])) {
      return false;
    }
    offset += 4;
  }
  intent->tool_id.assign(reinterpret_cast<const char*>(packet + offset), tool_id_len);
  offset += tool_id_len;
  if (packet_version == JOYSTICK_INTENT_PACKET_VERSION_V2) {
    intent->leader_fence_epoch = read_u64_be(packet, offset);
    offset += 8;
    const size_t lease_id_len = packet[offset++];
    if (intent->leader_fence_epoch == 0 || lease_id_len == 0 ||
        lease_id_len > 63 || size != offset + lease_id_len) {
      return false;
    }
    intent->leader_lease_id.assign(
        reinterpret_cast<const char*>(packet + offset), lease_id_len);
  }
  return true;
}

double clamp_value(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

double slew_limited_value(double current, double target, double max_delta) {
  const double delta = clamp_value(target - current, -max_delta, max_delta);
  return current + delta;
}

bool joystick_intent_to_cartesian_command(const SteamDeckJoystickIntent& intent,
                                          CartesianCommand* command) {
  if (intent.sequence > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
    return false;
  }
  command->seq = static_cast<int>(intent.sequence);
  command->timeout_ms = DEFAULT_RTCORE_JOG_TIMEOUT_MS;
  const double speed_scale = clamp_value(intent.speed_scale, 0.0, 1.0);
  for (int axis = 0; axis < 6; ++axis) {
    const double normalized_axis = clamp_value(intent.axes[axis], -1.0, 1.0);
    const double limit = axis < 3 ? MAX_JOG_LINEAR_M_S : MAX_JOG_ANGULAR_RAD_S;
    command->velocity_vector[axis] = normalized_axis * speed_scale * limit;
  }
  return true;
}

CartesianCommand smooth_cartesian_command(const CartesianCommand& command, StreamSmoother* smoother,
                                          int control_frequency_hz) {
  CartesianCommand out = command;
  const auto now = std::chrono::steady_clock::now();
  double dt = 1.0 / static_cast<double>(std::max(1, control_frequency_hz));
  if (smoother->initialized) {
    const std::chrono::duration<double> elapsed = now - smoother->last_update;
    dt = clamp_value(elapsed.count(), 0.001, 0.100);
  }
  smoother->last_update = now;
  smoother->initialized = true;

  for (int axis = 0; axis < 6; ++axis) {
    const bool linear = axis < 3;
    const double velocity_limit = linear ? MAX_JOG_LINEAR_M_S : MAX_JOG_ANGULAR_RAD_S;
    const double accel_limit = linear ? JOG_LINEAR_ACCEL_M_S2 : JOG_ANGULAR_ACCEL_RAD_S2;
    const double requested = clamp_value(command.velocity_vector[axis], -velocity_limit, velocity_limit);
    const double applied = slew_limited_value(smoother->applied_twist[axis], requested, accel_limit * dt);
    smoother->applied_twist[axis] = applied;
    out.velocity_vector[axis] = applied;
  }
  return out;
}

Mat4 identity4() {
  Mat4 out{};
  for (int i = 0; i < 4; ++i) {
    out[i][i] = 1.0;
  }
  return out;
}

Mat4 multiply4(const Mat4& lhs, const Mat4& rhs) {
  Mat4 out{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      for (int k = 0; k < 4; ++k) {
        out[r][c] += lhs[r][k] * rhs[k][c];
      }
    }
  }
  return out;
}

Mat4 urdf_origin_transform(const std::array<double, 3>& xyz,
                           const std::array<double, 3>& rpy) {
  const double cr = std::cos(rpy[0]);
  const double sr = std::sin(rpy[0]);
  const double cp = std::cos(rpy[1]);
  const double sp = std::sin(rpy[1]);
  const double cy = std::cos(rpy[2]);
  const double sy = std::sin(rpy[2]);
  // URDF origin rotation R = Rz(yaw) * Ry(pitch) * Rx(roll).
  return Mat4{{
      {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]}},
      {{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]}},
      {{-sp, cp * sr, cp * cr, xyz[2]}},
      {{0.0, 0.0, 0.0, 1.0}},
  }};
}

Mat4 urdf_origin_transform(const Rosie1400URDFJoint& joint) {
  return urdf_origin_transform(joint.origin_xyz, joint.origin_rpy);
}

Mat4 urdf_axis_rotation_transform(const std::array<double, 3>& axis, double radians) {
  const double len = std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
  if (len <= 1e-12) {
    return identity4();
  }
  const double x = axis[0] / len;
  const double y = axis[1] / len;
  const double z = axis[2] / len;
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  const double one_minus_c = 1.0 - c;
  return Mat4{{
      {{c + x * x * one_minus_c, x * y * one_minus_c - z * s,
        x * z * one_minus_c + y * s, 0.0}},
      {{y * x * one_minus_c + z * s, c + y * y * one_minus_c,
        y * z * one_minus_c - x * s, 0.0}},
      {{z * x * one_minus_c - y * s, z * y * one_minus_c + x * s,
        c + z * z * one_minus_c, 0.0}},
      {{0.0, 0.0, 0.0, 1.0}},
  }};
}

Mat4 urdf_forward_kinematics_j1_base(const double q[6]) {
  // One FK evaluation must never combine the arm chain from one selected
  // pair with the tool frame from another. Copy one immutable, SHA-verified
  // cache snapshot and use it for the complete chain.
  const MotionServerKinematicsSnapshot kinematics =
      motion_server_kinematics_snapshot_for_use();
  const auto& arm_joints = kinematics.arm_joints;
  Mat4 transform = identity4();
  for (int i = 0; i < 6; ++i) {
    const Rosie1400URDFJoint& joint = arm_joints[static_cast<std::size_t>(i)];
    if (i == 0) {
      transform = multiply4(transform, urdf_origin_transform({{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.0}}));
    } else {
      transform = multiply4(transform, urdf_origin_transform(joint));
    }
    transform = multiply4(transform, urdf_axis_rotation_transform(joint.axis, q[i]));
  }
  return multiply4(
      transform,
      urdf_origin_transform(kinematics.end_effector_joint));
}

std::array<double, 3> position_of(const Mat4& transform) {
  return {{transform[0][3], transform[1][3], transform[2][3]}};
}

double wrap_radians_pi(double value);

std::array<double, 3> rotate_direction(const Mat4& transform, const double local[3]) {
  return {{
      transform[0][0] * local[0] + transform[0][1] * local[1] + transform[0][2] * local[2],
      transform[1][0] * local[0] + transform[1][1] * local[1] + transform[1][2] * local[2],
      transform[2][0] * local[0] + transform[2][1] * local[1] + transform[2][2] * local[2],
  }};
}

double dot3(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

std::array<double, 3> cross3(const std::array<double, 3>& a,
                             const std::array<double, 3>& b) {
  return {{a[1] * b[2] - a[2] * b[1],
           a[2] * b[0] - a[0] * b[2],
           a[0] * b[1] - a[1] * b[0]}};
}

double norm3(const std::array<double, 3>& value) {
  return std::sqrt(dot3(value, value));
}

std::array<double, 3> normalized3(std::array<double, 3> value) {
  const double length = norm3(value);
  if (length <= 1.0e-12) {
    return {{0.0, 0.0, 0.0}};
  }
  for (double& component : value) {
    component /= length;
  }
  return value;
}

std::array<double, 3> spreadsheet_constraint_reference_perpendicular(
    const SpreadsheetToolAxisConstraint& constraint) {
  const std::array<double, 3> world = {{constraint.world_axis_xyz[0],
                                        constraint.world_axis_xyz[1],
                                        constraint.world_axis_xyz[2]}};
  std::array<double, 3> reference = {{constraint.offset_reference_axis_xyz[0],
                                      constraint.offset_reference_axis_xyz[1],
                                      constraint.offset_reference_axis_xyz[2]}};
  const double projection = dot3(reference, world);
  for (int axis = 0; axis < 3; ++axis) {
    reference[axis] -= projection * world[axis];
  }
  if (norm3(reference) <= 1.0e-9) {
    reference = std::abs(world[0]) < 0.9 ? std::array<double, 3>{{1.0, 0.0, 0.0}}
                                         : std::array<double, 3>{{0.0, 1.0, 0.0}};
    const double fallback_projection = dot3(reference, world);
    for (int axis = 0; axis < 3; ++axis) {
      reference[axis] -= fallback_projection * world[axis];
    }
  }
  return normalized3(reference);
}

std::array<double, 3> spreadsheet_constraint_target_axis(
    const SpreadsheetToolAxisConstraint& constraint) {
  const double offset_rad = constraint.offset_deg * kPi / 180.0;
  const std::array<double, 3> world = {{constraint.world_axis_xyz[0],
                                        constraint.world_axis_xyz[1],
                                        constraint.world_axis_xyz[2]}};
  const std::array<double, 3> reference =
      spreadsheet_constraint_reference_perpendicular(constraint);
  return normalized3({{std::cos(offset_rad) * world[0] + std::sin(offset_rad) * reference[0],
                       std::cos(offset_rad) * world[1] + std::sin(offset_rad) * reference[1],
                       std::cos(offset_rad) * world[2] + std::sin(offset_rad) * reference[2]}});
}

double spreadsheet_constraint_measured_offset_deg(
    const SpreadsheetToolAxisConstraint& constraint,
    const std::array<double, 3>& actual_axis) {
  const std::array<double, 3> world = {{constraint.world_axis_xyz[0],
                                        constraint.world_axis_xyz[1],
                                        constraint.world_axis_xyz[2]}};
  const std::array<double, 3> reference =
      spreadsheet_constraint_reference_perpendicular(constraint);
  return std::atan2(dot3(actual_axis, reference), dot3(actual_axis, world)) * 180.0 / kPi;
}

double spreadsheet_constraint_axis_error_deg(
    const SpreadsheetToolAxisConstraint& constraint,
    const std::array<double, 3>& actual_axis) {
  const std::array<double, 3> target = spreadsheet_constraint_target_axis(constraint);
  return std::acos(std::clamp(dot3(actual_axis, target), -1.0, 1.0)) * 180.0 / kPi;
}

bool spreadsheet_constraint_sample_satisfied(
    const SpreadsheetToolAxisConstraint& constraint,
    const std::array<double, 3>& actual_axis) {
  const double measured_offset_deg = spreadsheet_constraint_measured_offset_deg(constraint, actual_axis);
  const double signed_error_deg = wrap_radians_pi(
      (measured_offset_deg - constraint.offset_deg) * kPi / 180.0) * 180.0 / kPi;
  const double axis_error_deg = spreadsheet_constraint_axis_error_deg(constraint, actual_axis);
  return signed_error_deg >= -constraint.tolerance_minus_deg - 1.0e-6 &&
         signed_error_deg <= constraint.tolerance_plus_deg + 1.0e-6 &&
         axis_error_deg <= std::max(constraint.tolerance_minus_deg,
                                    constraint.tolerance_plus_deg) + 1.0e-6;
}

std::array<double, 3> small_rotation_vector_between(const Mat4& base, const Mat4& moved) {
  double rel[3][3]{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      for (int k = 0; k < 3; ++k) {
        rel[r][c] += moved[r][k] * base[c][k];
      }
    }
  }
  return {{
      0.5 * (rel[2][1] - rel[1][2]),
      0.5 * (rel[0][2] - rel[2][0]),
      0.5 * (rel[1][0] - rel[0][1]),
  }};
}

std::array<double, 3> rotation_vector_between(const Mat4& current,
                                              const Mat4& target) {
  double relative[3][3]{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      for (int axis = 0; axis < 3; ++axis) {
        relative[row][column] +=
            target[row][axis] * current[column][axis];
      }
    }
  }
  const double angle = std::acos(std::clamp(
      (relative[0][0] + relative[1][1] + relative[2][2] - 1.0) / 2.0,
      -1.0, 1.0));
  const std::array<double, 3> skew = {{
      relative[2][1] - relative[1][2],
      relative[0][2] - relative[2][0],
      relative[1][0] - relative[0][1],
  }};
  if (angle <= 1.0e-9) {
    return {{0.5 * skew[0], 0.5 * skew[1], 0.5 * skew[2]}};
  }
  const double denominator = 2.0 * std::sin(angle);
  if (std::abs(denominator) <= 1.0e-8) {
    return {{0.5 * skew[0], 0.5 * skew[1], 0.5 * skew[2]}};
  }
  const double scale = angle / denominator;
  return {{skew[0] * scale, skew[1] * scale, skew[2] * scale}};
}

Mat6 finite_difference_jacobian(const double q[6]) {
  constexpr double kEps = 1e-6;
  Mat6 jacobian{};
  const Mat4 base_fk = urdf_forward_kinematics_j1_base(q);
  const auto base_pos = position_of(base_fk);
  for (int joint = 0; joint < 6; ++joint) {
    double moved_q[6]{};
    for (int i = 0; i < 6; ++i) {
      moved_q[i] = q[i];
    }
    moved_q[joint] += kEps;
    const Mat4 moved_fk = urdf_forward_kinematics_j1_base(moved_q);
    const auto moved_pos = position_of(moved_fk);
    const auto rot = small_rotation_vector_between(base_fk, moved_fk);
    for (int axis = 0; axis < 3; ++axis) {
      jacobian[axis][joint] = (moved_pos[axis] - base_pos[axis]) / kEps;
      jacobian[axis + 3][joint] = rot[axis] / kEps;
    }
  }
  return jacobian;
}

bool same_joint_position_seed(const double lhs[6], const double rhs[6]) {
  for (int i = 0; i < 6; ++i) {
    if (std::abs(lhs[i] - rhs[i]) > 1e-12) {
      return false;
    }
  }
  return true;
}

Mat6 jacobian_for_command(const CartesianCommand& command, SolverCache* cache) {
  if (cache != nullptr && cache->valid &&
      same_joint_position_seed(cache->joint_position_rad, command.joint_position_rad)) {
    cache->hits++;
    return cache->jacobian;
  }

  Mat6 jacobian = finite_difference_jacobian(command.joint_position_rad);
  if (cache != nullptr) {
    cache->valid = true;
    for (int i = 0; i < 6; ++i) {
      cache->joint_position_rad[i] = command.joint_position_rad[i];
    }
    cache->jacobian = jacobian;
    cache->misses++;
  }
  return jacobian;
}

bool solve6(Mat6 a, Vec6 b, Vec6* x) {
  for (int pivot = 0; pivot < 6; ++pivot) {
    int best = pivot;
    double best_abs = std::abs(a[pivot][pivot]);
    for (int row = pivot + 1; row < 6; ++row) {
      const double candidate = std::abs(a[row][pivot]);
      if (candidate > best_abs) {
        best = row;
        best_abs = candidate;
      }
    }
    if (best_abs < 1e-12) {
      return false;
    }
    if (best != pivot) {
      std::swap(a[pivot], a[best]);
      std::swap(b[pivot], b[best]);
    }
    const double scale = a[pivot][pivot];
    for (int col = pivot; col < 6; ++col) {
      a[pivot][col] /= scale;
    }
    b[pivot] /= scale;
    for (int row = 0; row < 6; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = a[row][pivot];
      for (int col = pivot; col < 6; ++col) {
        a[row][col] -= factor * a[pivot][col];
      }
      b[row] -= factor * b[pivot];
    }
  }
  *x = b;
  return true;
}

void copy_sender_id(char* dest, size_t dest_size, const std::string& sender_id) {
  if (dest == nullptr || dest_size == 0) {
    return;
  }
  std::memset(dest, 0, dest_size);
  size_t out = 0;
  for (char ch : sender_id) {
    if (out + 1 >= dest_size) {
      break;
    }
    const unsigned char value = static_cast<unsigned char>(ch);
    if ((value >= 'a' && value <= 'z') ||
        (value >= 'A' && value <= 'Z') ||
        (value >= '0' && value <= '9') ||
        value == '-' || value == '_' || value == '.' || value == ':' || value == '/') {
      dest[out++] = ch;
    }
  }
  if (out == 0) {
    const char fallback[] = "unknown";
    std::memcpy(dest, fallback, std::min(dest_size - 1, sizeof(fallback) - 1));
  }
}

template <size_t N>
void copy_sender_id(char (&dest)[N], const std::string& sender_id) {
  copy_sender_id(dest, N, sender_id);
}

std::string sender_id_from_intent(const SteamDeckJoystickIntent& intent) {
  if (ControllerLeaderAuthority::concrete_controller_sender(
          intent.tool_id)) {
    return intent.tool_id;
  }
  if (!intent.tool_id.empty()) {
    // Compatibility wire form: existing Deck packets carry only
    // "steamdeck-N" in tool_id. Future controller roles carry their complete
    // canonical "<role>:<instance>" identity in the same bounded field.
    return "steamdeck:" + intent.tool_id;
  }
  return "steamdeck:unknown";
}

std::string lowercase_copy(const std::string& value) {
  std::string out = value;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return out;
}

std::string motion_owner_kind_from_sender(const std::string& sender_id) {
  const std::string lower = lowercase_copy(sender_id);
  if (lower.find("test") != std::string::npos || lower.find("probe") != std::string::npos) {
    return "test";
  }
  if (lower.find("steamdeck:") == 0) {
    return "steamdeck";
  }
  if (lower.empty() || lower == "none") {
    return "none";
  }
  return "other";
}

uint8_t motion_owner_kind_code(const std::string& kind) {
  if (kind == "steamdeck") {
    return 1;
  }
  if (kind == "test") {
    return 2;
  }
  if (kind == "other") {
    return 3;
  }
  return 0;
}

void write_fixed_ascii(std::vector<unsigned char>* packet, size_t offset, size_t width, const std::string& value) {
  if (packet == nullptr || offset + width > packet->size()) {
    return;
  }
  const size_t n = std::min(width, value.size());
  std::memcpy(packet->data() + offset, value.data(), n);
}

std::vector<unsigned char> encode_motion_owner_status_packet(const RuntimeStatus& status) {
  std::vector<unsigned char> packet(kMotionOwnerPacketBytes, 0);
  put_u16_be(&packet, 0, MOTION_OWNER_PACKET_MAGIC);
  packet[2] = MOTION_OWNER_PACKET_VERSION;
  packet[3] = motion_owner_kind_code(status.motion_owner_kind);
  packet[4] = status.motion_owner_fresh ? 1 : 0;
  put_u32_be(&packet, 8, static_cast<uint32_t>(std::max(0, status.motion_owner_age_ms)));
  put_u32_be(&packet, 12, static_cast<uint32_t>(std::max(0, status.rejected_competing_owner_count)));
  write_fixed_ascii(&packet, 16, 64, status.motion_owner_id);
  write_fixed_ascii(&packet, 80, 32, status.motion_owner_command_kind);
  packet[112] = status.plan_playback_active ? 1 : 0;
  packet[113] = status.plan_playback_completed ? 1 : 0;
  put_u16_be(&packet, 114,
             static_cast<uint16_t>(std::max(0, std::min(65535, status.plan_playback_sample_index))));
  put_u32_be(&packet, 116,
             static_cast<uint32_t>(std::max(0.0, status.plan_playback_elapsed_s) * 1000.0));
  put_u32_be(&packet, 120, static_cast<uint32_t>(status.plan_revision & 0xffffffffu));
  packet[124] = status.plan_abort_reason_code;
  // Play epoch: the daemon's accepted-play counter. The Deck captures the
  // epoch at its play press and only honors playback status whose epoch has
  // advanced, so UDP-reordered status from before the play (or a previous
  // run's completion) can never end a new playback.
  put_u16_be(&packet, 125, static_cast<uint16_t>(status.plan_play_commands & 0xffff));
  // Byte 127 was reserved in Motion Owner v4. Publishing Pause here keeps
  // the packet version/size rolling-deploy compatible while distinguishing a
  // frozen, zero-hold plan from one that is actively advancing.
  packet[127] = status.plan_playback_paused ? 1 : 0;
  // v3: exact application acknowledgement for an ordered direction=0
  // path_scrub. The Deck only mutates a row after this equals the intent it
  // published, so connection-level NATS success can never masquerade as a
  // stopped executor.
  put_u64_be(&packet, 128, status.latest_applied_hold_intent_seq);
  put_u64_be(&packet, 136, status.motion_owner_status_seq);
  packet[144] = status.latest_applied_hold_zero_output_sent ? 1 : 0;
  packet[145] = status.current_output_zero_hold ? 1 : 0;
  write_fixed_ascii(&packet, 148, 48, status.latest_applied_hold_command_id);
  write_fixed_ascii(&packet, 196, 48, status.latest_applied_hold_sender_id);
  write_fixed_ascii(&packet, 244, 12, "path_scrub");
  // v4: exact daemon-side application record for the latest accepted NATS
  // cold-motion command. Offsets 0..255 remain byte-for-byte compatible with
  // v3. The full target revision/time/row lets the Deck distinguish applying
  // a command from merely observing another periodic playback update.
  put_u64_be(&packet, 256, status.latest_applied_cold_motion_intent_seq);
  put_u64_be(&packet, 264, status.latest_applied_cold_plan_revision);
  put_u64_be(
      &packet, 272,
      static_cast<uint64_t>(
          std::max(0.0, status.latest_applied_cold_target_time_s) * 1000000.0));
  put_u32_be(
      &packet, 280,
      static_cast<uint32_t>(
          static_cast<int32_t>(status.latest_applied_cold_target_row)));
  packet[284] = status.latest_applied_cold_result_code;
  packet[285] = status.latest_applied_cold_accepted ? 1 : 0;
  packet[286] = status.latest_applied_cold_no_op ? 1 : 0;
  packet[287] = status.latest_applied_cold_zero_output_sent ? 1 : 0;
  write_fixed_ascii(&packet, 288, 96,
                    status.latest_applied_cold_command_id);
  write_fixed_ascii(&packet, 384, 96,
                    status.latest_applied_cold_sender_id);
  write_fixed_ascii(&packet, 480, 32,
                    status.latest_applied_cold_command_kind);
  write_fixed_ascii(&packet, 512, 64,
                    status.latest_applied_cold_result);
  return packet;
}

std::vector<unsigned char> encode_motion_owner_leader_status_packet_v5(
    const RuntimeStatus& status) {
  // Discovery/status contract: v5 is sent to every observed UDP peer, so
  // leader_lease_id is not an adversarial bearer secret. The explicit
  // trusted-LAN live mode coordinates cooperative controllers only; it does
  // not authenticate this grant or extend its fence into final consumers.
  std::vector<unsigned char> packet =
      encode_motion_owner_status_packet(status);
  packet.resize(kMotionOwnerPacketV5Bytes, 0);
  packet[2] = MOTION_OWNER_PACKET_VERSION_V5;
  packet[576] =
      (status.leader_lease_fresh ? 1u : 0u) |
      (status.leader_transition_pending ? 2u : 0u);
  put_u32_be(
      &packet, 580,
      static_cast<uint32_t>(std::max(0, status.leader_expires_in_ms)));
  put_u64_be(&packet, 584, status.leader_fence_epoch);
  write_fixed_ascii(&packet, 592, 64, status.leader_id);
  write_fixed_ascii(&packet, 656, 64, status.leader_boot_id);
  write_fixed_ascii(&packet, 720, 64, status.leader_lease_id);
  write_fixed_ascii(&packet, 784, 64, status.latest_leader_result);
  write_fixed_ascii(&packet, 848, 64,
                    status.latest_leader_request_id);
  put_u64_be(&packet, 912,
             status.latest_leader_campaign_generation);
  // Append-only v5 extension: offsets 0..919 retain their original meaning.
  // The active grant generation is distinct from the latest request outcome
  // generation and lets a client renew after that outcome has been overwritten.
  put_u64_be(&packet, 920, status.leader_campaign_generation);
  return packet;
}

struct RobotV4SingularityMetrics {
  std::array<double, 6> singular_values{};
  double condition_number = std::numeric_limits<double>::infinity();
  double manipulability = 0.0;
  double score = 0.0;
  int singular_direction_joint_index = -1;
  bool valid = false;
};

// Symmetric Jacobi eigensolver for J^T J. Tracking the eigenvectors gives the
// joint direction associated with the smallest singular value without adding
// an Eigen dependency to the realtime daemon.
RobotV4SingularityMetrics robot_v4_singularity_metrics(const Mat6& jacobian) {
  Mat6 symmetric{};
  Mat6 eigenvectors{};
  for (int row = 0; row < 6; ++row) {
    eigenvectors[row][row] = 1.0;
    for (int column = 0; column < 6; ++column) {
      for (int axis = 0; axis < 6; ++axis) {
        symmetric[row][column] += jacobian[axis][row] * jacobian[axis][column];
      }
    }
  }
  for (int iteration = 0; iteration < 96; ++iteration) {
    int pivot_row = 0;
    int pivot_column = 1;
    double pivot_magnitude = 0.0;
    for (int row = 0; row < 6; ++row) {
      for (int column = row + 1; column < 6; ++column) {
        const double magnitude = std::abs(symmetric[row][column]);
        if (magnitude > pivot_magnitude) {
          pivot_magnitude = magnitude;
          pivot_row = row;
          pivot_column = column;
        }
      }
    }
    if (pivot_magnitude <= 1.0e-12) {
      break;
    }
    const double app = symmetric[pivot_row][pivot_row];
    const double aqq = symmetric[pivot_column][pivot_column];
    const double apq = symmetric[pivot_row][pivot_column];
    const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    for (int index = 0; index < 6; ++index) {
      const double aip = symmetric[index][pivot_row];
      const double aiq = symmetric[index][pivot_column];
      symmetric[index][pivot_row] = cosine * aip - sine * aiq;
      symmetric[index][pivot_column] = sine * aip + cosine * aiq;
    }
    for (int index = 0; index < 6; ++index) {
      const double api = symmetric[pivot_row][index];
      const double aqi = symmetric[pivot_column][index];
      symmetric[pivot_row][index] = cosine * api - sine * aqi;
      symmetric[pivot_column][index] = sine * api + cosine * aqi;
    }
    symmetric[pivot_row][pivot_column] = 0.0;
    symmetric[pivot_column][pivot_row] = 0.0;
    for (int index = 0; index < 6; ++index) {
      const double vip = eigenvectors[index][pivot_row];
      const double viq = eigenvectors[index][pivot_column];
      eigenvectors[index][pivot_row] = cosine * vip - sine * viq;
      eigenvectors[index][pivot_column] = sine * vip + cosine * viq;
    }
  }

  struct EigenPair {
    double value = 0.0;
    int source_index = 0;
  };
  std::array<EigenPair, 6> pairs{};
  for (int index = 0; index < 6; ++index) {
    pairs[index] = {std::sqrt(std::max(0.0, symmetric[index][index])), index};
  }
  std::sort(pairs.begin(), pairs.end(), [](const EigenPair& lhs, const EigenPair& rhs) {
    return lhs.value > rhs.value;
  });
  RobotV4SingularityMetrics metrics;
  metrics.manipulability = 1.0;
  for (int index = 0; index < 6; ++index) {
    metrics.singular_values[index] = pairs[index].value;
    metrics.manipulability *= pairs[index].value;
  }
  const double largest = metrics.singular_values.front();
  const double smallest = metrics.singular_values.back();
  metrics.valid = std::isfinite(largest) && std::isfinite(smallest) && largest > 1.0e-12;
  // The motion-server's calibrated warning/hard-stop thresholds are
  // expressed in sigma-min units. Keep the condition number separately;
  // using its reciprocal here would silently change the policy dimension.
  metrics.score = metrics.valid ? smallest : 0.0;
  metrics.condition_number = metrics.valid && smallest > 1.0e-12
                                 ? largest / smallest
                                 : std::numeric_limits<double>::infinity();
  const int singular_vector = pairs.back().source_index;
  double largest_component = 0.0;
  for (int joint = 0; joint < 6; ++joint) {
    const double component = std::abs(eigenvectors[joint][singular_vector]);
    if (component > largest_component) {
      largest_component = component;
      metrics.singular_direction_joint_index = joint;
    }
  }
  return metrics;
}

double robot_v4_singularity_velocity_scale(const RobotV4SingularityMetrics& metrics,
                                            double warning_threshold,
                                            double hard_stop_threshold) {
  if (!metrics.valid || !std::isfinite(warning_threshold) ||
      !std::isfinite(hard_stop_threshold) ||
      warning_threshold <= hard_stop_threshold || metrics.score <= hard_stop_threshold) {
    return 0.0;
  }
  if (metrics.score >= warning_threshold) {
    return 1.0;
  }
  return clamp_value((metrics.score - hard_stop_threshold) /
                         (warning_threshold - hard_stop_threshold),
                     0.0, 1.0);
}

double robot_v4_common_joint_limit_scale(const double* requested_velocity_rad_s,
                                         const double* joint_limits_rad_s,
                                         std::size_t count,
                                         int* limiting_joint_index = nullptr) {
  double scale = 1.0;
  int limiting = -1;
  for (std::size_t axis = 0; axis < count; ++axis) {
    const double requested = std::abs(requested_velocity_rad_s[axis]);
    const double limit = joint_limits_rad_s[axis];
    if (!std::isfinite(requested) || !std::isfinite(limit) || limit <= 0.0) {
      scale = 0.0;
      limiting = static_cast<int>(axis);
      break;
    }
    if (requested > limit && limit / requested < scale) {
      scale = limit / requested;
      limiting = static_cast<int>(axis);
    }
  }
  if (limiting_joint_index != nullptr) {
    *limiting_joint_index = limiting;
  }
  return clamp_value(scale, 0.0, 1.0);
}

std::vector<double> damped_least_squares_joint_velocity(const CartesianCommand& command,
                                                        SolverCache* cache = nullptr) {
  constexpr double kLambda = 0.05;
  const Mat6 jacobian = jacobian_for_command(command, cache);
  Mat6 normal{};
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 6; ++c) {
      for (int joint = 0; joint < 6; ++joint) {
        normal[r][c] += jacobian[r][joint] * jacobian[c][joint];
      }
    }
    normal[r][r] += kLambda * kLambda;
  }
  Vec6 twist{};
  for (int i = 0; i < 6; ++i) {
    twist[i] = command.velocity_vector[i];
  }
  Vec6 y{};
  if (!solve6(normal, twist, &y)) {
    return std::vector<double>(6, 0.0);
  }
  std::vector<double> qd(6, 0.0);
  for (int joint = 0; joint < 6; ++joint) {
    for (int row = 0; row < 6; ++row) {
      qd[joint] += jacobian[row][joint] * y[row];
    }
  }
  return qd;
}

double spreadsheet_plan_tcp_fk_error_m(const double tcp_pose[6],
                                       const double joints_rad[kLiveAxisCount]) {
  double q[6]{};
  for (int axis = 0; axis < 6; ++axis) {
    q[axis] = joints_rad[axis];
  }
  const std::array<double, 3> fk = position_of(urdf_forward_kinematics_j1_base(q));
  const double dx = tcp_pose[0] - fk[0];
  const double dy = tcp_pose[1] - fk[1];
  const double dz = tcp_pose[2] - fk[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double wrap_radians_pi(double value) {
  double wrapped = std::fmod(value + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

bool solve_spreadsheet_tcp_position_ik(const double tcp_pose[6],
                                       const double seed[kLiveAxisCount],
                                       const SpreadsheetToolAxisConstraint* constraint,
                                       double out[kLiveAxisCount]) {
  constexpr int kMaxIterations = 160;
  // Accepted Cartesian samples must resolve inside the pendant's measured
  // endpoint bounds, including sub-millimetre and small-angle requests.
  constexpr double kPositionToleranceM = 0.000005;
  constexpr double kOrientationToleranceRad = 0.001 * kPi / 180.0;
  constexpr double kLambda = 0.02;
  constexpr double kMaxJointStepRad = 0.16;
  const Mat4 target_pose = urdf_origin_transform(
      {{tcp_pose[0], tcp_pose[1], tcp_pose[2]}},
      {{tcp_pose[3], tcp_pose[4], tcp_pose[5]}});
  double q[6]{};
  for (int axis = 0; axis < 6; ++axis) {
    q[axis] = seed[axis];
  }
  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    const Mat4 fk_pose = urdf_forward_kinematics_j1_base(q);
    const std::array<double, 3> fk = position_of(fk_pose);
    Vec6 error{};
    error[0] = tcp_pose[0] - fk[0];
    error[1] = tcp_pose[1] - fk[1];
    error[2] = tcp_pose[2] - fk[2];
    const double error_m =
        std::sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2]);
    const std::array<double, 3> orientation_error =
        rotation_vector_between(fk_pose, target_pose);
    const double orientation_error_rad = norm3(orientation_error);
    const double orientation_weight =
        constraint != nullptr ? constraint->weight : 1.0;
    for (int axis = 0; axis < 3; ++axis) {
      error[axis + 3] = orientation_error[axis] * orientation_weight;
    }
    bool constraint_satisfied = true;
    if (constraint != nullptr && constraint->present && constraint->valid) {
      const std::array<double, 3> actual_axis = normalized3(
          rotate_direction(fk_pose, constraint->tool_axis_xyz));
      constraint_satisfied =
          spreadsheet_constraint_sample_satisfied(*constraint, actual_axis);
    }
    if (error_m <= kPositionToleranceM &&
        orientation_error_rad <= kOrientationToleranceRad &&
        constraint_satisfied) {
      for (int axis = 0; axis < 6; ++axis) {
        out[axis] = q[axis];
      }
      out[6] = seed[6];
      return true;
    }

    const Mat6 jacobian = finite_difference_jacobian(q);
    Mat6 normal{};
    Vec6 rhs{};
    for (int row = 0; row < 6; ++row) {
      for (int col = 0; col < 6; ++col) {
        for (int axis = 0; axis < 6; ++axis) {
          const double row_weight = axis >= 3 && constraint != nullptr ? constraint->weight : 1.0;
          normal[row][col] += (jacobian[axis][row] * row_weight) *
                              (jacobian[axis][col] * row_weight);
        }
      }
      normal[row][row] += kLambda * kLambda;
      for (int axis = 0; axis < 6; ++axis) {
        const double row_weight = axis >= 3 && constraint != nullptr ? constraint->weight : 1.0;
        rhs[row] += (jacobian[axis][row] * row_weight) * error[axis];
      }
    }
    Vec6 delta{};
    if (!solve6(normal, rhs, &delta)) {
      return false;
    }
    double max_delta = 0.0;
    for (double value : delta) {
      max_delta = std::max(max_delta, std::abs(value));
    }
    const double scale = max_delta > kMaxJointStepRad ? kMaxJointStepRad / max_delta : 1.0;
    for (int axis = 0; axis < 6; ++axis) {
      q[axis] = wrap_radians_pi(q[axis] + delta[axis] * scale);
    }
  }
  double candidate[kLiveAxisCount]{};
  for (int axis = 0; axis < 6; ++axis) {
    candidate[axis] = q[axis];
  }
  candidate[6] = seed[6];
  double final_q[6]{};
  for (int axis = 0; axis < 6; ++axis) {
    final_q[axis] = candidate[axis];
  }
  const Mat4 final_fk_pose = urdf_forward_kinematics_j1_base(final_q);
  const double final_orientation_error_rad =
      norm3(rotation_vector_between(final_fk_pose, target_pose));
  bool final_constraint_satisfied = true;
  if (constraint != nullptr && constraint->present && constraint->valid) {
    const std::array<double, 3> actual_axis = normalized3(
        rotate_direction(final_fk_pose, constraint->tool_axis_xyz));
    final_constraint_satisfied =
        spreadsheet_constraint_sample_satisfied(*constraint, actual_axis);
  }
  if (spreadsheet_plan_tcp_fk_error_m(tcp_pose, candidate) <= kPositionToleranceM &&
      final_orientation_error_rad <= kOrientationToleranceRad &&
      final_constraint_satisfied) {
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      out[axis] = candidate[axis];
    }
    return true;
  }
  return false;
}

std::string json_array(const std::vector<double>& values) {
  std::ostringstream out;
  out << "[";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      out << ",";
    }
    out << std::fixed << std::setprecision(6) << values[i];
  }
  out << "]";
  return out.str();
}

std::string double_array_json_fixed(const double* values, int count) {
  std::ostringstream out;
  out << "[";
  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      out << ",";
    }
    out << std::fixed << std::setprecision(6) << values[i];
  }
  out << "]";
  return out.str();
}

std::vector<std::string> json_object_slices_for_array_key(const std::string& text,
                                                          const std::string& key) {
  std::vector<std::string> objects;
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return objects;
  }
  const std::string::size_type array_open = text.find('[', key_pos + marker.size());
  if (array_open == std::string::npos) {
    return objects;
  }
  int array_depth = 0;
  int object_depth = 0;
  std::string::size_type object_start = std::string::npos;
  bool in_string = false;
  bool escaped = false;
  for (std::string::size_type index = array_open; index < text.size(); ++index) {
    const char ch = text[index];
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        in_string = false;
      }
      continue;
    }
    if (ch == '"') {
      in_string = true;
      continue;
    }
    if (ch == '[') {
      ++array_depth;
      continue;
    }
    if (ch == ']') {
      --array_depth;
      if (array_depth <= 0) {
        break;
      }
      continue;
    }
    if (ch == '{') {
      if (object_depth == 0) {
        object_start = index;
      }
      ++object_depth;
      continue;
    }
    if (ch == '}') {
      --object_depth;
      if (object_depth == 0 && object_start != std::string::npos) {
        objects.push_back(text.substr(object_start, index - object_start + 1));
        object_start = std::string::npos;
      }
    }
  }
  return objects;
}

std::vector<std::string> parse_json_string_array(const std::string& text,
                                                 const std::string& key) {
  std::vector<std::string> values;
  const std::string marker = "\"" + key + "\"";
  const std::string::size_type key_pos = text.find(marker);
  if (key_pos == std::string::npos) {
    return values;
  }
  const std::string::size_type array_open = text.find('[', key_pos + marker.size());
  if (array_open == std::string::npos) {
    return values;
  }
  bool in_string = false;
  bool escaped = false;
  std::string current;
  for (std::string::size_type index = array_open + 1; index < text.size(); ++index) {
    const char ch = text[index];
    if (in_string) {
      if (escaped) {
        current.push_back(ch);
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        values.push_back(current);
        current.clear();
        in_string = false;
      } else {
        current.push_back(ch);
      }
      continue;
    }
    if (ch == '"') {
      in_string = true;
      current.clear();
    } else if (ch == ']') {
      break;
    }
  }
  return values;
}
#endif  // ROBOT_V4_JSON_SCANNER_ONLY
