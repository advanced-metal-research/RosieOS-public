#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <string>
#include "a6ec_alarm_codes_generated.hpp"

namespace rosie::rtcore {

struct AlarmAuxSnapshot {
  uint64_t event = 0, configuration_epoch = 0;
  uint32_t alarm_aux_code = 0;
  bool alarm_aux_valid = false;
};

// The cycle records every error transition, including clear/reassert while an
// upload blocks. Worker receipts cannot label a different alarm or epoch.
struct DriveAlarmState {
  std::atomic<uint64_t> event{0};
  std::atomic<uint64_t> reply_event{0}, reply_epoch{0};
  std::atomic<uint32_t> alarm_aux_code{0};
  std::atomic<bool> alarm_aux_valid{false};
  uint64_t attempted_event = 0; // Commissioning worker only.

  // One writer invalidates the receipt before replacing its atomic payload;
  // readers accept only the same nonzero event on both sides of the copy.
  void publish(const AlarmAuxSnapshot& value) {
    reply_event.store(0);
    alarm_aux_code.store(value.alarm_aux_code);
    alarm_aux_valid.store(value.alarm_aux_valid);
    reply_epoch.store(value.configuration_epoch);
    reply_event.store(value.event);
  }

  void observe(uint16_t error) {
    const auto previous = event.load();
    if (static_cast<uint16_t>(previous) == error) return;
    // Unverified event capacity: 48-bit transition counter; exhaustion disables
    // readback permanently instead of allowing a stale receipt to match again.
    if ((previous >> 16) == (std::numeric_limits<uint64_t>::max() >> 16)) return;
    event.store((((previous >> 16) + 1) << 16) | error);
  }

  // Clear/change immediately exposes zero and invalid even if the worker still
  // holds older bytes. Only a receipt for this exact cyclic event is observable.
  AlarmAuxSnapshot snapshot(uint64_t expected_event, uint64_t epoch, uint16_t error) const {
    const auto before = reply_event.load();
    const AlarmAuxSnapshot value{before, reply_epoch.load(), alarm_aux_code.load(), alarm_aux_valid.load()};
    if (!error || static_cast<uint16_t>(expected_event) != error ||
        (expected_event >> 16) == (std::numeric_limits<uint64_t>::max() >> 16) ||
        event.load() != expected_event || value.event != expected_event || reply_event.load() != before ||
        value.configuration_epoch != epoch || !value.alarm_aux_valid) return {};
    return value;
  }
};

struct DriveAlarmName { std::string code, text; };

// Exact bus/external-code matches take priority. Missing or unknown auxiliary
// evidence lists every bus candidate, preventing a generic 0x603F from guessing.
inline DriveAlarmName decode_a6ec_alarm(uint16_t bus, uint32_t aux, bool valid) {
  DriveAlarmName result;
  if (!bus) return result;
  bool exact = false;
  for (const auto& row : a6ec_alarm_codes)
    exact |= valid && row.bus_valid && row.bus_code == bus && row.aux_code == static_cast<uint16_t>(aux);
  for (const auto& row : a6ec_alarm_codes) {
    if (!row.bus_valid || row.bus_code != bus || (exact && row.aux_code != static_cast<uint16_t>(aux))) continue;
    if (!result.code.empty()) { result.code += " | "; result.text += " | "; }
    result.code += row.display_code;
    result.text += row.meaning;
  }
  return result;
}
} // namespace rosie::rtcore
