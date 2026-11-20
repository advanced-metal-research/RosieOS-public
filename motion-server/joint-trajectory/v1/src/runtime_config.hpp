#pragma once

// Runtime configuration for the native dense daemon: every
// deployment-tunable limit, timing, and address lives in a JSON config
// file (config/*.config.json is the canonical, annotated copy), not in
// code. The compiled defaults below mirror those files for an explicitly selected
// backend; selection is required so old configs cannot silently choose native
// execution. A deployment passes --config (or the env var) and the file wins;
// CLI flags override the file for ephemeral ports.
//
// What deliberately does NOT live here: the dense-joint-trajectory format
// constants (record layout, validation tolerances, the 250k sample cap)
// and the 64 MiB TCP frame cap. Those are contracts enforced identically
// by the Go and Python twins -- a C++-only config would let one side
// drift without the tri-language tests noticing until a real plan bounced.
//
// Loading fails loudly: a missing file that was asked for, a wrong schema,
// or a malformed value is a startup error, never a silent fallback --
// silently falling back to compiled numbers is how a site retune gets lost.

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "dense_joint_trajectory_format.hpp"

namespace dense_joint_trajectory {

inline constexpr const char* kDaemonConfigSchema =
    "robot.v4.joint-trajectory-daemon.config.v1";

struct DaemonConfig {
  // Unverified deployment defaults: native motion requires explicit digests and binding.
  std::string backend = "rt_core";
  std::string control_socket = "/run/rosie-rt-core/control.sock";
  std::string controller_identity = "motion-server";
  std::string pair_id;
  std::uint64_t pair_revision = 0;
  std::string machine_sha256;
  std::string deployment_sha256;
  std::string configuration_sha256;
  std::string expected_backend = "simulation";
  std::string home_policy = "home";
  std::string status_subject;
  // Unverified receipt/observation budget in ms; success requires native observations.
  std::uint64_t control_timeout_ms = 10000;
  // ingest
  std::string ingest_listen = "127.0.0.1:8797";
  std::string plan_store_dir;  // empty = $HOME/.rosie/motion-server/joint-trajectory/v1/dense-plans
  // status stream: how often the daemon publishes its status document on
  // NATS. The stream exists at all only when a subject is given on the
  // command line; the TCP `status` request answers either way.
  double status_publish_hz = 20.0;
  // leader leases
  std::uint64_t lease_ttl_min_ms = 500;
  std::uint64_t lease_ttl_max_ms = 2000;
};



// Validate approved native identity before threads, stores, listeners, or grants.
// Placeholder digests cannot silently start an unbound daemon.
inline bool native_binding_configured(const DaemonConfig& config) {
  // SHA-256 identity is 32 bytes encoded as 64 lowercase hexadecimal digits.
  const auto digest = [](const std::string& value) {
    if (value.size() != 64) return false;
    for (char c : value) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    return true;
  };
  return config.backend == "rt_core" && !config.control_socket.empty() &&
      !config.controller_identity.empty() && !config.pair_id.empty() && config.pair_revision > 0 &&
      digest(config.machine_sha256) && digest(config.deployment_sha256) && digest(config.configuration_sha256);
}

namespace config_detail {

inline bool read_file(const std::string& path, std::string* out) {
  std::ifstream input(path);
  if (!input) return false;
  std::ostringstream buffer;
  buffer << input.rdbuf();
  *out = buffer.str();
  return true;
}

inline bool parse_root(const std::string& path, const char* expected_schema,
                       JsonValue* root, std::string* error) {
  std::string text;
  if (!read_file(path, &text)) {
    *error = "cannot read " + path;
    return false;
  }
  JsonParser parser(text.data(), text.size());
  if (!parser.parse(root) || root->kind != JsonValue::Kind::Object) {
    *error = path + ": not a JSON object";
    return false;
  }
  std::string schema;
  if (!json_str(root->find("schema"), &schema) || schema != expected_schema) {
    *error = path + ": schema " + schema + " != " + expected_schema;
    return false;
  }
  return true;
}

// Each getter overwrites the default only when the key exists, and fails
// on a mistyped or out-of-range value.
inline bool take_str(const JsonValue* section, const char* key,
                     std::string* out, std::string* error) {
  if (section == nullptr) return true;
  const JsonValue* value = section->find(key);
  if (value == nullptr) return true;
  if (value->kind != JsonValue::Kind::String || value->string_value.empty()) {
    *error = std::string(key) + " must be a non-empty string";
    return false;
  }
  *out = value->string_value;
  return true;
}

inline bool take_f64(const JsonValue* section, const char* key, double* out,
                     std::string* error) {
  if (section == nullptr) return true;
  const JsonValue* value = section->find(key);
  if (value == nullptr) return true;
  double parsed = 0;
  if (!json_f64(value, &parsed) || !(parsed > 0)) {
    *error = std::string(key) + " must be a positive number";
    return false;
  }
  *out = parsed;
  return true;
}

// A fixed-length array of numbers, every element required: a pose with a
// missing joint is not a pose.
inline bool take_f64_array(const JsonValue* section, const char* key, double* out,
                           std::size_t count, std::string* error) {
  const JsonValue* value = section != nullptr ? section->find(key) : nullptr;
  if (value == nullptr || value->kind != JsonValue::Kind::Array ||
      value->array_items.size() != count) {
    *error = std::string("config: ") + key + " must be an array of " +
             std::to_string(count) + " numbers";
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    if (value->array_items[i].kind != JsonValue::Kind::Number) {
      *error = std::string("config: ") + key + "[" + std::to_string(i) + "] is not a number";
      return false;
    }
    out[i] = value->array_items[i].number_value;
  }
  return true;
}

template <typename Integer>
inline bool take_int(const JsonValue* section, const char* key, Integer* out,
                     std::string* error) {
  if (section == nullptr) return true;
  const JsonValue* value = section->find(key);
  if (value == nullptr) return true;
  std::uint64_t parsed = 0;
  if (!json_u64(value, &parsed) || parsed == 0) {
    *error = std::string(key) + " must be a positive integer";
    return false;
  }
  *out = Integer(parsed);
  return true;
}

}  // namespace config_detail

inline bool load_daemon_config(const std::string& path, DaemonConfig* config,
                               std::string* error, const std::string& backend_override = {}) {
  using namespace config_detail;
  JsonValue root;
  if (!parse_root(path, kDaemonConfigSchema, &root, error)) return false;
  const JsonValue* ingest = root.find("ingest");
  const JsonValue* leader = root.find("leader");
  const JsonValue* status = root.find("status");
  const JsonValue* native = root.find("rt_core");
  if (!root.find("backend") || !take_str(&root, "backend", &config->backend, error) ||
      config->backend != "rt_core") {
    *error = "reason=backend_retired"; return false;
  }
  if (!backend_override.empty()) config->backend = backend_override;
  if (config->backend != "rt_core") {
    *error = "reason=backend_retired"; return false;
  }
  return take_str(native, "socket", &config->control_socket, error) &&
         take_str(native, "controller", &config->controller_identity, error) &&
         take_str(native, "pair_id", &config->pair_id, error) &&
         take_int(native, "pair_revision", &config->pair_revision, error) &&
         take_str(native, "machine_sha256", &config->machine_sha256, error) &&
         take_str(native, "deployment_sha256", &config->deployment_sha256, error) &&
         take_str(native, "configuration_sha256", &config->configuration_sha256, error) &&
         take_str(native, "expected_backend", &config->expected_backend, error) &&
         take_str(native, "home_policy", &config->home_policy, error) &&
         take_str(native, "status_subject", &config->status_subject, error) &&
         take_int(native, "control_timeout_ms", &config->control_timeout_ms, error) &&
         take_str(ingest, "listen", &config->ingest_listen, error) &&
         take_str(ingest, "plan_store_dir", &config->plan_store_dir, error) &&
         take_f64(status, "publish_hz", &config->status_publish_hz, error) &&
         take_int(leader, "ttl_min_ms", &config->lease_ttl_min_ms, error) &&
         take_int(leader, "ttl_max_ms", &config->lease_ttl_max_ms, error);

}

// Resolve selection before startup; CLI order cannot revive a retired file backend.
inline bool load_daemon_config_arguments(int argc, char** argv, DaemonConfig* config,
                                         std::string* error) {
  std::string path, backend;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument != "--config" && argument != "--backend") continue;
    if (i + 1 == argc) { *error = argument + " requires a value"; return false; }
    (argument == "--config" ? path : backend) = argv[++i];
  }
  if (backend.empty()) {
    if (const char* env = std::getenv("JOINT_TRAJECTORY_BACKEND")) backend = env;
  }
  if (!backend.empty() && backend != "rt_core") {
    *error = "reason=backend_retired"; return false;
  }
  if (path.empty()) {
    const char* env = std::getenv("JOINT_TRAJECTORY_DAEMON_CONFIG");
    if (env) path = env;
  }
  if (!path.empty()) return load_daemon_config(path, config, error, backend);
  if (!backend.empty()) config->backend = backend;
  return true;
}

}  // namespace dense_joint_trajectory
