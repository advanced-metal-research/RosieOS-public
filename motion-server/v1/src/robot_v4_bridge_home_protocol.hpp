#pragma once

// Cold coordination only. Home and GoHome validation, packet translation and
// execution remain owned by the bridge and RTCore.
struct BridgeHomeReply {
  std::string challenge_id, ticket_id, bridge_boot_id;
  std::string controller_id, controller_boot_id, run_id;
  std::string kind, command_id_json, state, error;
  std::string prepared_binding;
  bool quiescent = false;
};

bool decode_bridge_home_reply(const std::string& payload, BridgeHomeReply* out) {
  if (out == nullptr) return false;
  BridgeHomeReply result;
  for (auto field : {
           std::pair<const char*, std::string*>{"challenge_id", &result.challenge_id},
           {"ticket_id", &result.ticket_id}, {"bridge_boot_id", &result.bridge_boot_id},
           {"controller_id", &result.controller_id},
           {"controller_boot_id", &result.controller_boot_id},
           {"run_id", &result.run_id}, {"kind", &result.kind}, {"state", &result.state}}) {
    std::string raw;
    if (!extract_unique_top_level_json_member(payload, field.first, &raw) ||
        !parse_exact_json_string_value(raw, field.second)) return false;
  }
  std::string identity;
  if (!parse_position_command_id(payload, &identity, &result.command_id_json) &&
      result.state != "failed" && result.state != "unknown") return false;
  std::string raw;
  bool present = false;
  if (!extract_unique_top_level_json_member(payload, "quiescent", &raw, &present) ||
      (present && !parse_exact_json_bool_value(raw, &result.quiescent))) return false;
  if (!extract_unique_top_level_json_member(payload, "error", &raw, &present) ||
      (present && !parse_exact_json_string_value(raw, &result.error))) return false;
  if (!extract_unique_top_level_json_member(payload, "prepared_binding", &raw, &present)) return false;
  if (present) {
    if (raw.empty() || raw.front() != '{' || raw.back() != '}') return false;
    result.prepared_binding = raw;
  }
  if (result.challenge_id.empty() ||
      (result.kind != "home" && result.kind != "go_home" &&
       result.state != "failed" && result.state != "unknown") ||
      (result.state != "prepared" && result.state != "executing" &&
       result.state != "completed" && result.state != "failed" &&
       result.state != "cancelled" && result.state != "unknown")) return false;
  *out = std::move(result);
  return true;
}
