#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "drive_alarm_reset.hpp"
#include "ipc_v1.hpp"

namespace rosie::rtcore::identity {

using Fields = std::map<std::string, std::string>;

// Hash canonical bytes at startup; no file reads or hashing belong in the cycle.
// FIPS 180-4 SHA-256 words and rotations are mathematical constants, not tuning.
inline std::array<uint8_t, 32> sha256(const std::string& text) {
  // Measured against the FIPS empty/abc and million-a known-answer vectors.
  constexpr uint32_t k[] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
  std::array<uint32_t,8> h{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
      0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
  std::vector<uint8_t> bytes(text.begin(),text.end());
  bytes.push_back(0x80);
  while (bytes.size() % 64 != 56) bytes.push_back(0);
  const uint64_t bits = uint64_t(text.size()) * 8;
  for (int shift = 56; shift >= 0; shift -= 8) bytes.push_back(uint8_t(bits >> shift));
  const auto rotate = [](uint32_t x, unsigned n) { return (x >> n) | (x << (32-n)); };
  for (size_t base = 0; base < bytes.size(); base += 64) {
    std::array<uint32_t,64> w{};
    for (unsigned i = 0; i < 16; ++i)
      for (unsigned j = 0; j < 4; ++j) w[i] = (w[i] << 8) | bytes[base+4*i+j];
    for (unsigned i = 16; i < 64; ++i) {
      const auto x = w[i-15], y = w[i-2];
      w[i] = w[i-16] + (rotate(x,7)^rotate(x,18)^(x>>3)) + w[i-7] + (rotate(y,17)^rotate(y,19)^(y>>10));
    }
    auto a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],v=h[7];
    for (unsigned i = 0; i < 64; ++i) {
      const uint32_t t1 = v+(rotate(e,6)^rotate(e,11)^rotate(e,25))+((e&f)^(~e&g))+k[i]+w[i];
      const uint32_t t2 = (rotate(a,2)^rotate(a,13)^rotate(a,22))+((a&b)^(a&c)^(b&c));
      v=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d; h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=v;
  }
  std::array<uint8_t,32> result{};
  for (unsigned i = 0; i < 32; ++i) result[i] = uint8_t(h[i/4] >> (24-8*(i%4)));
  return result;
}

inline std::string digest(const std::string& text) {
  std::string result;
  for (uint8_t b : sha256(text)) { result += "0123456789abcdef"[b>>4]; result += "0123456789abcdef"[b&15]; }
  return result;
}

inline bool valid_digest(const std::string& text) {
  return text.size() == 64 && std::all_of(text.begin(),text.end(),[](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}

// Locale-independent %.17g binary64 formatting rejects nonfinite effective
// values instead of giving NaN or infinity an apparently valid machine identity.
inline std::string number(double value) {
  if (!std::isfinite(value)) throw std::invalid_argument("nonfinite machine field");
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << value;
  return out.str();
}

inline std::string canonical(const Fields& fields) {
  std::string result;
  for (const auto& field : fields) result += field.first + "=" + field.second + "\n";
  return result;
}

// Canonical field tables are bounded at their file reader. Reject duplicate or
// malformed lines so persisted explanations cannot conceal a different digest.
inline Fields parse_fields(const std::string& text) {
  Fields fields;
  std::istringstream input(text);
  std::string line;
  while (std::getline(input, line)) {
    const auto equal = line.find('=');
    if (equal == std::string::npos || equal == 0 ||
        !fields.emplace(line.substr(0, equal), line.substr(equal + 1)).second)
      throw std::invalid_argument("invalid coordinate identity fields");
  }
  if (canonical(fields) != text) throw std::invalid_argument("noncanonical coordinate identity");
  return fields;
}

inline std::string hex_text(const std::string& text) {
  std::string result;
  for (unsigned char c : text) { result += "0123456789abcdef"[c >> 4]; result += "0123456789abcdef"[c & 15]; }
  return result;
}

inline std::string unhex_text(const std::string& text) {
  if (text.size() % 2) throw std::invalid_argument("odd hex text");
  const auto digit = [](char c) -> unsigned {
    if (c >= '0' && c <= '9') return unsigned(c - '0');
    if (c >= 'a' && c <= 'f') return unsigned(c - 'a' + 10);
    throw std::invalid_argument("invalid hex text");
  };
  std::string result;
  for (size_t i = 0; i < text.size(); i += 2) result += char((digit(text[i]) << 4) | digit(text[i+1]));
  return result;
}

// Compare authenticated expected canonical fields in sorted order. Without an
// expected field table a digest cannot identify which input changed.
inline std::string first_difference(const std::string& expected, const std::string& actual) {
  std::istringstream left(expected), right(actual);
  std::string a,b;
  for (;;) {
    const bool have_a = bool(std::getline(left,a)), have_b = bool(std::getline(right,b));
    if (!have_a && !have_b) return {};
    if (a == b && have_a == have_b) continue;
    const auto key_a = a.substr(0,a.find('=')), key_b = b.substr(0,b.find('='));
    if (!have_a) return key_b;
    if (!have_b) return key_a;
    return std::min(key_a,key_b);
  }
}

inline std::string verify(const std::string& expected_digest, const std::string& expected,
                          const std::string& actual) {
  if (!valid_digest(expected_digest)) return "machine_sha256 (invalid digest)";
  if (!expected.empty() && digest(expected) != expected_digest)
    return "machine_sha256 (identity artifact digest mismatch)";
  if (digest(actual) == expected_digest) return {};
  if (!expected.empty()) return first_difference(expected,actual);
  return "machine_sha256 (expected field table unavailable)";
}

template<class Scalar> inline std::string scalar_name(Scalar type) {
  switch (type) {
    case Scalar::kU8: return "u8"; case Scalar::kI8: return "i8";
    case Scalar::kU16: return "u16"; case Scalar::kI16: return "i16";
    case Scalar::kU32: return "u32"; case Scalar::kI32: return "i32";
    default: throw std::invalid_argument("invalid SDO scalar");
  }
}
template<class Object> inline std::string object_text(const Object& object) {
  return std::to_string(object.index)+"|"+std::to_string(object.subindex)+"|"+scalar_name(object.type);
}
template<class Entries> inline std::string pdo_text(const Entries& entries) {
  std::string result;
  for (const auto& entry : entries) {
    if (!result.empty()) result += ';';
    result += entry.semantic+"|"+std::to_string(entry.index)+"|"+std::to_string(entry.subindex)+"|"+std::to_string(entry.bits);
  }
  return result;
}
template<class Home> inline std::string home_text(const Home& home) {
  if (!home.valid) return {};
  std::string result = "steady_state_mode|"+std::to_string(home.steady_state_mode)+
      ";commissioning_mode|"+std::to_string(home.commissioning_mode)+";truth_source|"+object_text(home.truth_source);
  for (const auto& step : home.transaction) {
    using Kind = decltype(step.kind);
    result += ";op|";
    switch (step.kind) {
      case Kind::kSetMode: result += "set_mode|"+std::to_string(step.value_i32); break;
      case Kind::kRestoreMode: result += "restore_mode|"+std::to_string(step.value_i32); break;
      case Kind::kWriteSdo: case Kind::kWaitSdo: {
        using Scalar = decltype(step.object.type);
        const int64_t value = step.object.type == Scalar::kU32 ? int64_t(uint32_t(step.value_i32)) : int64_t(step.value_i32);
        result += (step.kind == Kind::kWriteSdo ? "write_sdo|" : "wait_sdo|")+object_text(step.object)+"|"+std::to_string(value);
        break;
      }
      case Kind::kControlwordSequence:
        result += "controlword_sequence|";
        for (uint32_t i=0; i<step.controlword_count; ++i) { if(i) result += ','; result += std::to_string(step.controlword_values[i]); }
        break;
      case Kind::kWaitStatusword:
        result += "wait_statusword|"+std::to_string(step.wait_all_set_mask)+"|"+std::to_string(step.wait_all_clear_mask); break;
      case Kind::kRefreshTruth: result += "refresh_truth"; break;
      case Kind::kReleaseServiceOverride: result += "release_service_override"; break;
      default: throw std::invalid_argument("invalid home operation");
    }
  }
  return result;
}

template<class Profile> inline std::string coordinate_home(const Profile& p) {
  const auto& home = p.home;
  if (!home.valid) return "none";
  std::string result = "truth|" + object_text(home.truth_source) + ";sign|" + std::to_string(p.home_truth_sign);
  for (const auto& step : home.transaction) {
    using Kind = decltype(step.kind);
    if (step.kind != Kind::kWriteSdo ||
        (step.object.index != 0x6098 && step.object.index != 0x607c)) continue;
    using Scalar = decltype(step.object.type);
    const int64_t value = step.object.type == Scalar::kU32 ? int64_t(uint32_t(step.value_i32)) : int64_t(step.value_i32);
    result += ";write_sdo|" + object_text(step.object) + "|" + std::to_string(value);
  }
  return result;
}

// Rebuild effective count semantics at admission. Only explicitly declared
// physical encoder/gearing declarations survive from the compiler table;
// CLI scaling and startup SDO changes cannot retain an old coordinate identity.
template<class Axis, class Profile, class Startup>
inline std::string coordinate_fields(uint32_t axis, const Axis& a, const Profile& p, const Startup& startup) {
  if (p.coordinate_identity.empty())
    throw std::invalid_argument("missing coordinate_identity axis=" + std::to_string(axis) + " name=" + p.name);
  auto fields = parse_fields(p.coordinate_identity);
  const std::vector<std::string> keys{"anchor.source", "direction.sign", "drive.product_code",
      "drive.revision_no", "drive.vendor_id", "encoder.counts_per_motor_rev", "evidence.mode",
      "gear.denominator", "gear.numerator", "home.policy", "tracking.mode",
      "wire.counts_per_rev", "wire.revs_per_axis_rev"};
  if (fields.size() != keys.size())
    throw std::invalid_argument("incomplete coordinate identity");
  for (const auto& key : keys) if (!fields.count(key) || fields.at(key).empty())
    throw std::invalid_argument("missing coordinate identity field: " + key);
  // Unverified bound: 63 ASCII bytes for the tracking-mode token.
  const auto token = [](const std::string& value) {
    return value.size() <= 63 && std::all_of(value.begin(), value.end(), [](unsigned char c) {
      return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
    });
  };
  if (!token(fields.at("tracking.mode")))
    throw std::invalid_argument("invalid coordinate identity token");
  for (const auto& key : {"encoder.counts_per_motor_rev", "gear.numerator", "gear.denominator"}) {
    const auto& value = fields.at(key);
    if (!std::all_of(value.begin(), value.end(), [](char c) { return c >= '0' && c <= '9'; }) ||
        std::to_string(std::stoull(value)) != value || value == "0")
      throw std::invalid_argument("invalid coordinate identity integer");
  }
  fields["direction.sign"] = std::to_string(a.sign);
  fields["drive.vendor_id"] = std::to_string(p.vendor_id);
  fields["drive.product_code"] = std::to_string(p.product_code);
  fields["drive.revision_no"] = std::to_string(p.revision_no);
  fields["wire.counts_per_rev"] = std::to_string(a.counts_per_rev);
  fields["wire.revs_per_axis_rev"] = number(a.gear_ratio);
  fields["home.policy"] = coordinate_home(p);
  fields["evidence.mode"] = "multi_turn";
  const auto source = p.absolute_anchor_source();
  fields["anchor.source"] = source.identity_name.empty() ? "none|0" :
      std::string(source.identity_name) + "|" + std::to_string(source.version);
  if (p.vendor_id == 4194304 && p.product_code == 1813) {
    for (const auto& item : std::map<std::string, uint32_t>{{"tracking.mode",0x200008}}) {
      bool found = false;
      for (const auto& sdo : startup) if ((sdo.axis_mask & (1u << axis)) &&
          ((uint32_t(sdo.index) << 8) | sdo.subindex) == item.second) {
        fields[item.first] = std::to_string(sdo.values[axis]); found = true;
      }
      if (!found) throw std::invalid_argument("missing coordinate startup field: " + item.first);
    }
  }
  return canonical(fields);
}

// Reconstruct the effective machine, including derived units and velocity caps.
// Requested limits remain included so clipping cannot conceal artifact changes.
template<class Options, class Profiles, class Startup>
inline Fields machine_fields(const Options& opt, const Profiles& profiles, const Startup& startup) {
  Fields fields;
  auto integer = [&](const std::string& key, auto value) { fields[key] = std::to_string(value); };
  if (opt.cell_io.enabled()) fields["cell_io"] = opt.cell_io.canonical;
  integer("num_axes",opt.num_axes); integer("cycle_ns",opt.cycle_ns);
  if (opt.bus.hold || !opt.bus.unused.empty()) {
    fields["bus_unknown_slaves"] = opt.bus.hold ? "hold" : "refuse";
    fields["bus_unused_slaves"] = opt.bus.unused_text();
  }
  // Elide LAN defaults exactly as the compiler does, preserving existing identities.
  if (opt.control_max_grant_lease_ns != rosie::protocol::fast_control_max_grant_lease_ns)
    integer("control_max_grant_lease_ns",opt.control_max_grant_lease_ns);
  if (opt.control_max_jog_input_age_ns != rosie::protocol::fast_control_max_jog_input_age_ns)
    integer("control_max_jog_input_age_ns",opt.control_max_jog_input_age_ns);
  integer("max_cycle_lateness_ns",opt.max_cycle_lateness_ns); fields["max_rpm"] = number(opt.max_rpm);
  // All seven bus_bringup settings are machine semantics: firmware readiness,
  // PDO mapping, watchdog response and DC changes invalidate machine evidence.
  // Keep these effective native fields aligned with the Go compiler's table.
  integer("use_dc",opt.use_dc); integer("disable_output_watchdog",opt.disable_output_watchdog);
  integer("split_domains_per_axis",opt.split_domains_per_axis); integer("explicit_pdo_config",opt.explicit_pdo_config);
  integer("wait_before_safeop_ms",opt.wait_before_safeop_ms); integer("preop_to_safeop_timeout_ms",opt.preop_to_safeop_timeout_ms);
  integer("safeop_to_op_timeout_ms",opt.safeop_to_op_timeout_ms); integer("startup_passive_ms",opt.startup_passive_ms);
  integer("startup_skip_domain_queue_ms",opt.startup_skip_domain_queue_ms);
  for (uint32_t i=0; i<opt.num_axes; ++i) {
    const auto& a = opt.axis[i]; const auto& p = profiles[i]; const auto& l = p.limits;
    const auto prefix = "axis."+std::to_string(i)+".";
    auto n = [&](const char* key, auto value) { integer(prefix+key,value); };
    auto s = [&](const char* key, const std::string& value) { fields[prefix+key] = value; };
    n("slave_position",opt.slave_position[i]); n("counts_per_rev",a.counts_per_rev);
    s("gear_ratio",number(a.gear_ratio)); n("sign",a.sign); s("lead_m_per_rev",number(a.lead_m_per_rev));
    const bool rotary = a.axis_type == gradient::ipc::v1::AXIS_TYPE_ROTARY;
    s("axis_type",rotary ? "rotary" : "linear"); s("position_unit",rotary ? "rad" : "m");
    s("counts_per_unit",number(a.counts_per_unit)); n("max_profile_vel_counts_per_s",a.max_profile_vel_counts_per_s);
    s("name",p.name); n("vendor_id",p.vendor_id); n("product_code",p.product_code); n("revision_no",p.revision_no);
    n("dc_quantum_ns",p.dc_quantum_ns); n("dc_assign_activate",p.dc_assign_activate);
    n("rx_pdo",p.pdo.rx_pdo); n("tx_pdo",p.pdo.tx_pdo); n("rx_sync",p.pdo.rx_sync_index); n("tx_sync",p.pdo.tx_sync_index);
    if (p.pdo.restore_assignment_on_exit) n("restore_assignment_on_exit",1);
    if (p.encoder_battery_absent) {
      s("encoder_battery","absent");
      s("encoder_battery_absent_alarm",
        std::to_string(p.encoder_battery_absent_error)+"|"+std::to_string(p.encoder_battery_absent_aux));
    }
    s("rx_layout",pdo_text(p.pdo.rx_entries)); s("tx_layout",pdo_text(p.pdo.tx_entries));
    n("home_truth_sign",p.home_truth_sign); s("native_home",home_text(p.home));
    if (p.reference_verified_delta_enabled) s("reference_verified_delta_restore", "independent_absolute_delta_v1");
    if (!p.retained_home_restore_enabled) s("retained_home_restore","disabled");
    else if (p.retained_home_requires_homing_attained)
      s("retained_home_restore","persisted_anchor_agreement_v1,homing_attained_required");
    std::string absolute;
    for (uint32_t j=0; j<p.absolute.field_count; ++j) {
      if (j) absolute += ';';
      absolute += p.absolute.fields[j].key+"|"+object_text(p.absolute.fields[j].object);
    }
    s("absolute_feedback",absolute); s("absolute_pair_field",p.absolute_pair_field);
    // Present only when configured, matching the compiler, which emits no line when absent.
    if (!p.monitor.empty()) {
      std::string monitor;
      for (const auto& m : p.monitor) {
        if (!monitor.empty()) monitor += ';';
        monitor += m.key+"|"+object_text(m.object)+"|"+std::to_string(m.decimals)+"|"+m.unit+"|"+
            std::to_string(m.interval_ns/1000000);
      }
      s("monitor",monitor);
    }
    std::string sdos;
    for (const auto& d : startup) if (d.axis_mask & (1u<<i)) {
      if (!sdos.empty()) sdos += ';';
      sdos += d.key+"|"+scalar_name(d.type)+"|"+std::to_string(d.index)+"|"+std::to_string(d.subindex)+"|"+std::to_string(d.values[i]);
      if (d.effective_after_power_cycle) sdos += "|power_cycle";
    }
    s("startup_sdos",sdos);
    // Present only when configured, matching the compiler, which emits no line when absent.
    if (!p.parameter_storage_key.empty())
      s("parameter_storage", p.parameter_storage_key+"|"+std::to_string(p.parameter_storage_off));
    if (!p.startup_trial.empty()) {
      std::string trial;
      for (const auto& key : p.startup_trial) trial += (trial.empty() ? "" : ";") + key;
      s("startup_trial",trial);
    }
    if (!p.startup_observe.empty()) {
      std::string observe;
      for (const auto& o : p.startup_observe) {
        if (!observe.empty()) observe += ';';
        observe += o.key+"|"+scalar_name(o.type)+"|"+std::to_string(o.index)+"|"+std::to_string(o.subindex);
      }
      s("startup_observe",observe);
    }
    s("limits",number(l.min_q)+"|"+number(l.max_q)+"|"+number(l.max_qd)+"|"+(l.require_home ? "1":"0"));
    s("effective_max_velocity",number(std::min(l.max_qd,double(a.max_profile_vel_counts_per_s)/a.counts_per_unit)));
    s("tracking",number(l.jog_acceleration)+"|"+number(l.max_target_lead)+"|"+number(l.following_error)+"|"+
        number(l.completion_tolerance)+"|"+std::to_string(l.following_error_timeout_ns/1000000)+"|"+std::to_string(l.completion_timeout_ns/1000000));
    // Present only when configured, matching the compiler, which emits no line when absent.
    if (l.tracking_lag_allowance_ns)
      s("tracking_lag_allowance", std::to_string(l.tracking_lag_allowance_ns/1000000));
    // Optional per-axis acceleration bound (P4-06): present only when the profile
    // declares it, matching the compiler, which emits no line when absent.
      if (p.max_acceleration) s("max_acceleration",number(*p.max_acceleration));
      if (p.collision_watchdog.enabled)
        s("collision_watchdog", std::to_string(p.collision_watchdog.torque_abs_max_raw) + "|" +
            std::to_string(p.collision_watchdog.following_error_counts_max) + "|" +
            std::to_string(p.collision_watchdog.sustained_cycles));
    if (p.external_enable_input)
      s("external_enable_input", std::to_string(p.external_enable_input->bit_index) + "|" +
          (p.external_enable_input->active_high ? "active_high" : "active_low"));
    s("jog",std::to_string(p.jog.arrest_ns)+"|"+std::to_string(p.jog.quick_stop_ns));
    // Present only when configured, matching the compiler, which emits no line when absent.
    if (p.jog.smoothing_cycles)
      s("jog_jerk", std::to_string(p.jog.jerk)+"|"+std::to_string(p.jog.smoothing_cycles));
    s("brake",std::to_string(p.brake.present)+"|"+std::to_string(p.brake.release_delay_ns/1000000)+"|"+
        std::to_string(p.brake.hold_delay_ns/1000000)+"|"+std::to_string(p.brake.gravity_axis)+"|"+
        (p.brake.feedback_required ? p.brake_released_semantic : "-")+"|"+std::to_string(p.brake_released_bit)+
        (p.brake.hold_displacement_tolerance_counts == 1 ? "" :
            "|"+std::to_string(p.brake.hold_displacement_tolerance_counts)));
    if (p.brake.transition.enabled()) {
      const auto& t = p.brake.transition;
      s("brake_transition", std::to_string(t.max_displacement_counts) + "|" +
          std::to_string(t.max_velocity_counts_per_s) + "|" + std::to_string(t.target_stationary_ns / 1000000) + "|" +
          std::to_string(t.settling_ns / 1000000) + "|" + std::to_string(t.timeout_ns / 1000000));
    }
    if (p.brake.opening.enabled()) {
      const auto& o = p.brake.opening;
      s("brake_opening", std::to_string(o.max_displacement_counts) + "|" +
          std::to_string(o.max_velocity_counts_per_s) + "|" + std::to_string(o.timeout_ns / 1000000));
    }
    if (p.start_alignment_floor_counts) n("start_alignment_floor", p.start_alignment_floor_counts);
    if (p.drive_supervision.configured()) {
      const auto& d = p.drive_supervision;
      s("drive_supervision", std::to_string(d.enable_timeout_ns / 1000000) + "|" +
          std::to_string(d.release_slack_ns / 1000000) + "|" + std::to_string(d.disarm_motion_counts));
    }
    if (p.drive_alarm_reset.configured())
      s("drive_alarm_reset", rosie::rtcore::render_drive_alarm_reset_spec(p.drive_alarm_reset));
    if (!p.brake_override_reason_hex.empty())
      s("brake_override_reason_hex", p.brake_override_reason_hex);
    const auto& c = p.coordinate_evidence;
    if (c.sdo_sampled) {
      const auto& source = p.absolute_anchor_source();
      s("coordinate_anchor_source", std::to_string(source.index) + "|" +
          std::to_string(source.low_subindex) + "|" + std::to_string(source.high_subindex) + "|" +
          source.name + "|" + source.identity_name + "|" + std::to_string(source.version));
    }
    s("coordinate_evidence",c.enabled ? std::to_string(c.maximum_physical_speed_counts_per_second)+"|"+
        std::to_string(c.maximum_acquisition_age_ns)+"|"+std::to_string(c.maximum_feedback_error_counts)+"|"+
        std::to_string(c.maximum_anchor_error_counts)+"|"+
        (c.sdo_sampled ? "a6ec_sdo|"+std::to_string(c.sample_interval_ns) :
         std::to_string(c.validity_mask)+"|"+std::to_string(c.validity_value)) : "");
  }
  return fields;
}

} // namespace rosie::rtcore::identity
