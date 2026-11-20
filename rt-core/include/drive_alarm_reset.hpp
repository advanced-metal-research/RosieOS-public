#pragma once

#include "ds402.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace rosie::rtcore {

// Operator Reset of a resettable drive alarm (a Rosie 1430 arm, 2026-10-11: J5 Er41.1).
//
// The drive leaves CiA402 Fault only on the fault-reset edge (controlword bit 7
// rising, transition 15). rt-core's recovery matrix kept drive_readiness until
// the drive was out of Fault, and sent the edge only after that, so a drive
// alarm the vendor marks resettable could never be cleared without a restart.
// Reset now sends the drive's own CiA402 reset to an axis in Fault whose alarm
// the drive profile classifies resettable, then judges the rt-core fault on the
// drive's answer. Nothing here touches Home, HM35, anchors or any tolerance.
//
// The classification is drive-profile data, compiled by rtctl from the
// profile's fault_reset block into the axis line
// `drive_alarm_reset=<leave_fault_ms>|<min_interval_ms>|<603F>:<203F>;...`.
// Source classifies nothing: an axis without the line resets no drive alarm,
// and an alarm the line does not name, or whose 0x203F detail is unread, is
// not resettable.
struct DriveAlarmCode {
  uint16_t error_code = 0, aux_code = 0;
};
// Native capacity per axis, mirrored by the compiler (the A6-EC manual's
// chapter 10 lists 92 fault codes, 60 of them resettable).
inline constexpr size_t kMaxResettableDriveAlarms = 128;
// Admission bound for leave_fault_ms, mirrored by the compiler. The adapter
// waits 1 s for the reset's recovery publication; an attempt must finish
// inside that with margin, or the press would report no outcome.
inline constexpr uint64_t kMaxDriveAlarmLeaveFaultMs = 500;
// Admission bound for min_interval_ms (one hour), mirrored by the compiler.
inline constexpr uint64_t kMaxDriveAlarmResetIntervalMs = 3600000;

struct DriveAlarmResetConfig {
  // How long after the reset edge the drive may take to leave Fault.
  uint64_t leave_fault_timeout_ns = 0;
  // Least time between two reset attempts on one axis.
  uint64_t min_interval_ns = 0;
  std::array<DriveAlarmCode, kMaxResettableDriveAlarms> codes{};
  uint32_t count = 0;
  bool configured() const noexcept { return leave_fault_timeout_ns != 0 && count != 0; }
  // Exact (0x603F, 0x203F) pair from an admitted 0x203F receipt. A bus code
  // alone is ambiguous (A6-EC 0x3230 is Er40.0, Er41.0 and ALF6.1), so it
  // never qualifies.
  bool resettable(uint16_t error_code, uint32_t aux_code, bool aux_valid) const noexcept {
    if (!aux_valid || !error_code) return false;
    for (uint32_t i = 0; i < count && i < codes.size(); ++i)
      if (codes[i].error_code == error_code && codes[i].aux_code == static_cast<uint16_t>(aux_code)) return true;
    return false;
  }
};

namespace detail {
inline bool parse_drive_alarm_number(const std::string& text, uint64_t max, uint64_t* out) {
  if (text.empty() || text.size() > 20) return false;
  const bool hex = text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X');
  for (size_t i = hex ? 2 : 0; i < text.size(); ++i) {
    const char c = text[i];
    const bool digit = c >= '0' && c <= '9';
    const bool hex_digit = (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
    if (!digit && !(hex && hex_digit)) return false;
  }
  char* end = nullptr;
  const unsigned long long value = std::strtoull(text.c_str(), &end, hex ? 16 : 10);
  if (!end || *end || value > max) return false;
  *out = value;
  return true;
}
} // namespace detail

// Strict mirror of the compiler: both times positive and bounded, 1..128
// unique pairs, a nonzero 0x603F code in each. Anything else refuses the
// configuration rather than resetting a different set of alarms.
inline bool parse_drive_alarm_reset_spec(const std::string& spec, DriveAlarmResetConfig* out) {
  if (!out) return false;
  *out = {};
  const auto first = spec.find('|');
  const auto second = first == std::string::npos ? std::string::npos : spec.find('|', first + 1);
  if (second == std::string::npos || spec.find('|', second + 1) != std::string::npos) return false;
  uint64_t leave_ms = 0, interval_ms = 0;
  if (!detail::parse_drive_alarm_number(spec.substr(0, first), kMaxDriveAlarmLeaveFaultMs, &leave_ms) || !leave_ms ||
      !detail::parse_drive_alarm_number(spec.substr(first + 1, second - first - 1), kMaxDriveAlarmResetIntervalMs,
                                        &interval_ms) || !interval_ms)
    return false;
  DriveAlarmResetConfig parsed{};
  parsed.leave_fault_timeout_ns = leave_ms * 1000000;
  parsed.min_interval_ns = interval_ms * 1000000;
  const std::string list = spec.substr(second + 1);
  size_t begin = 0;
  while (true) {
    const auto end = list.find(';', begin);
    const std::string entry = list.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
    const auto colon = entry.find(':');
    uint64_t error_code = 0, aux_code = 0;
    if (colon == std::string::npos || parsed.count >= kMaxResettableDriveAlarms ||
        !detail::parse_drive_alarm_number(entry.substr(0, colon), 0xffff, &error_code) || !error_code ||
        !detail::parse_drive_alarm_number(entry.substr(colon + 1), 0xffff, &aux_code))
      return false;
    const DriveAlarmCode code{static_cast<uint16_t>(error_code), static_cast<uint16_t>(aux_code)};
    for (uint32_t i = 0; i < parsed.count; ++i)
      if (parsed.codes[i].error_code == code.error_code && parsed.codes[i].aux_code == code.aux_code) return false;
    parsed.codes[parsed.count++] = code;
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  *out = parsed;
  return true;
}

// Canonical text, identical to the compiler's output: the native machine
// identity rebuilds the line from parsed values and must reproduce it.
inline std::string render_drive_alarm_reset_spec(const DriveAlarmResetConfig& config) {
  std::string out = std::to_string(config.leave_fault_timeout_ns / 1000000) + "|" +
      std::to_string(config.min_interval_ns / 1000000) + "|";
  for (uint32_t i = 0; i < config.count && i < config.codes.size(); ++i) {
    char entry[24];
    std::snprintf(entry, sizeof entry, "%s0x%04x:0x%04x", i ? ";" : "", config.codes[i].error_code,
                  config.codes[i].aux_code);
    out += entry;
  }
  return out;
}

// Per-axis state (status) and attempt result (journal, refusal). Wire names
// are stable; append only.
enum class DriveAlarmReset : uint32_t {
  None = 0,            // Drive not in CiA402 Fault.
  Possible,            // Reset sends the drive's fault-reset edge.
  NotConfigured,       // The drive profile classifies no alarm.
  AlarmUnread,         // 0x203F detail not read: unknown, so not resettable.
  NotResettable,       // The drive profile does not classify this alarm resettable.
  RateLimited,         // Within min_interval of this axis's last attempt.
  NotInhibited,        // Armed, enabled, moving, Home, service or commissioning.
  FeedbackStale,       // No fresh statusword for the axis.
  InProgress,          // A reset attempt on this axis is running.
  Cleared,             // The drive left Fault after the edge and stayed out.
  PersistsAfterReset,  // The drive stayed in Fault after the edge.
  Refaulted,           // The drive left Fault, then faulted again.
  Interrupted,         // Stop, authority, configuration, Home or bus changed.
};

inline const char* drive_alarm_reset_name(DriveAlarmReset value) noexcept {
  switch (value) {
    case DriveAlarmReset::None: return "none";
    case DriveAlarmReset::Possible: return "possible";
    case DriveAlarmReset::NotConfigured: return "not_configured";
    case DriveAlarmReset::AlarmUnread: return "alarm_unread";
    case DriveAlarmReset::NotResettable: return "not_resettable";
    case DriveAlarmReset::RateLimited: return "rate_limited";
    case DriveAlarmReset::NotInhibited: return "not_inhibited";
    case DriveAlarmReset::FeedbackStale: return "feedback_stale";
    case DriveAlarmReset::InProgress: return "in_progress";
    case DriveAlarmReset::Cleared: return "cleared";
    case DriveAlarmReset::PersistsAfterReset: return "persists_after_reset";
    case DriveAlarmReset::Refaulted: return "refaulted";
    case DriveAlarmReset::Interrupted: return "interrupted";
  }
  return "unknown";
}

struct DriveAlarmResetObservation {
  uint16_t statusword = 0;
  uint16_t error_code = 0;
  uint32_t aux_code = 0;
  bool aux_valid = false;
  bool fresh = false;
  // Disarmed, nothing enabled, no motion, Home, service or commissioning.
  bool inhibited = false;
  uint64_t now_ns = 0;
  // Start of this axis's last attempt; 0 means none this daemon incarnation.
  uint64_t last_attempt_ns = 0;
};

// The one admission rule, used by Reset and by the published status, so the
// status can never offer a reset that Reset would refuse for another reason
// of this table.
inline DriveAlarmReset classify_drive_alarm_reset(const DriveAlarmResetConfig& config,
                                                  const DriveAlarmResetObservation& in) noexcept {
  if (gradient::ds402::decode_statusword(in.statusword) != gradient::ds402::State::Fault) return DriveAlarmReset::None;
  if (!config.configured()) return DriveAlarmReset::NotConfigured;
  if (!in.fresh) return DriveAlarmReset::FeedbackStale;
  if (!in.aux_valid) return DriveAlarmReset::AlarmUnread;
  if (!config.resettable(in.error_code, in.aux_code, in.aux_valid)) return DriveAlarmReset::NotResettable;
  if (!in.inhibited) return DriveAlarmReset::NotInhibited;
  if (in.last_attempt_ns && in.now_ns >= in.last_attempt_ns && in.now_ns - in.last_attempt_ns < config.min_interval_ns)
    return DriveAlarmReset::RateLimited;
  return DriveAlarmReset::Possible;
}

// Time until a rate-limited axis may be reset again (0 when it may now).
inline uint64_t drive_alarm_reset_retry_after_ns(const DriveAlarmResetConfig& config, uint64_t last_attempt_ns,
                                                 uint64_t now_ns) noexcept {
  if (!last_attempt_ns || now_ns < last_attempt_ns || now_ns - last_attempt_ns >= config.min_interval_ns) return 0;
  return config.min_interval_ns - (now_ns - last_attempt_ns);
}

// One axis's CiA402 fault reset. Bit 7 rises once, only while the drive
// reports Fault (in Fault Reaction Active the drive ignores it, and a bit
// already high when it reaches Fault is no edge). It is held for at most
// `cycles` and falls the cycle the drive leaves Fault. A drive that returns to
// Fault after leaving it gets no second edge in the same attempt.
struct DriveAlarmResetPulse {
  uint16_t cycles_left = 0;
  bool left_fault = false, refaulted = false;
  void begin(uint16_t cycles) noexcept { *this = {}; cycles_left = cycles; }
  uint16_t step(uint16_t statusword) noexcept {
    using State = gradient::ds402::State;
    const auto state = gradient::ds402::decode_statusword(statusword);
    if (state == State::FaultReactionActive) {
      if (left_fault) refaulted = true;
      return 0;
    }
    if (state != State::Fault) {
      left_fault = true;
      cycles_left = 0;
      return 0;
    }
    if (left_fault) refaulted = true;
    if (refaulted || !cycles_left) {
      cycles_left = 0;
      return 0;
    }
    --cycles_left;
    return gradient::ds402::CW_FAULT_RESET;
  }
  bool out_of_fault(uint16_t statusword) const noexcept {
    using State = gradient::ds402::State;
    const auto state = gradient::ds402::decode_statusword(statusword);
    return left_fault && !refaulted && state != State::Fault && state != State::FaultReactionActive &&
        !(statusword & 8u);
  }
};

// Result of one axis in one Reset press; carried inside the recovery snapshot
// so the outcome and its per-axis cause are one coherent publication.
struct alignas(8) DriveAlarmResetEvidence {
  uint64_t sequence = 0;        // Per-daemon attempt sequence; 0 means none.
  uint64_t reset_sequence = 0;  // Native command sequence of the Reset press.
  uint64_t observed_ns = 0;
  uint64_t retry_after_ns = 0;  // RateLimited only.
  DriveAlarmReset result = DriveAlarmReset::None;
  uint32_t statusword = 0, error_code = 0, aux_code = 0, aux_valid = 0;
  uint32_t reserved = 0;
};

// Operator text for one axis's result, naming the joint (J1-based) and the
// drive's alarm. `code`/`text` are the decoded alarm when the profile's drive
// has a decode table, else empty (the raw 0x603F/0x203F codes are named).
inline std::string drive_alarm_reset_message(const DriveAlarmResetEvidence& e, uint32_t axis,
                                             const std::string& code, const std::string& text) {
  char alarm[96];
  if (!code.empty()) std::snprintf(alarm, sizeof alarm, "drive alarm %s", code.c_str());
  else if (e.aux_valid) std::snprintf(alarm, sizeof alarm, "drive fault 0x%04x/0x%04x", e.error_code, e.aux_code & 0xffffu);
  else std::snprintf(alarm, sizeof alarm, "drive fault 0x%04x", e.error_code);
  const std::string detail = text.empty() ? std::string() : ": " + text;
  char head[256] = "";
  const unsigned joint = axis + 1;
  switch (e.result) {
    case DriveAlarmReset::PersistsAfterReset:
      std::snprintf(head, sizeof head, "J%u %s persists after reset", joint, alarm);
      break;
    case DriveAlarmReset::Refaulted:
      std::snprintf(head, sizeof head, "J%u %s returned after reset", joint, alarm);
      break;
    case DriveAlarmReset::Interrupted:
      std::snprintf(head, sizeof head, "J%u %s reset interrupted (Stop, authority, configuration or bus change)",
                    joint, alarm);
      break;
    case DriveAlarmReset::NotResettable:
      std::snprintf(head, sizeof head, "J%u %s is not resettable from rt-core (drive profile)", joint, alarm);
      break;
    case DriveAlarmReset::AlarmUnread:
      std::snprintf(head, sizeof head, "J%u %s not reset: its 0x203F detail is unread, so it is not classified",
                    joint, alarm);
      break;
    case DriveAlarmReset::NotConfigured:
      std::snprintf(head, sizeof head, "J%u %s not reset: the drive profile classifies no alarm as resettable",
                    joint, alarm);
      break;
    case DriveAlarmReset::RateLimited:
      std::snprintf(head, sizeof head, "J%u %s reset refused: retry in %llu ms", joint, alarm,
                    static_cast<unsigned long long>((e.retry_after_ns + 999999) / 1000000));
      break;
    case DriveAlarmReset::FeedbackStale:
      std::snprintf(head, sizeof head, "J%u %s not reset: no fresh drive status", joint, alarm);
      break;
    case DriveAlarmReset::Cleared:
      std::snprintf(head, sizeof head, "J%u %s cleared by reset", joint, alarm);
      break;
    default:
      return "";
  }
  return head + detail;
}

// Whether a result keeps the drive (and so the press) faulted.
constexpr bool drive_alarm_reset_failed(DriveAlarmReset value) noexcept {
  return value != DriveAlarmReset::None && value != DriveAlarmReset::Cleared &&
      value != DriveAlarmReset::Possible && value != DriveAlarmReset::InProgress;
}

} // namespace rosie::rtcore
