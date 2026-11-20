#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace rosie::rtcore {
// Unverified inventory capacity: at most 256 non-axis devices, independent of
// the 16-axis motion limit. Exceeding it refuses activation, never truncates.
inline constexpr size_t kMaxUnusedSlaves = 256;

struct BusSlaveIdentity {
  uint32_t vendor_id = 0, product_code = 0, revision = 0;
  uint16_t position = 0;
  bool revision_required = false;
  bool matches(const BusSlaveIdentity& other) const {
    return position == other.position && vendor_id == other.vendor_id &&
        product_code == other.product_code && (!revision_required || revision == other.revision);
  }
};

struct BusPolicy {
  bool hold = false;
  std::vector<BusSlaveIdentity> unused;
  // Canonical decimal fields are shared by argv and machine identity; notes
  // belong to the configuration digest so they cannot change motor scaling.
  std::string unused_text() const {
    std::string result;
    for (const auto& s : unused) {
      if (!result.empty()) result += ';';
      result += std::to_string(s.position) + '|' + std::to_string(s.vendor_id) + '|' +
          std::to_string(s.product_code) + '|' + (s.revision_required ? std::to_string(s.revision) : "-");
    }
    return result;
  }
};

enum class BusSlaveDisposition : uint8_t {
  Held, DeclaredUnused, RefusedUnknown, RefusedIdentity, IdentityUnreadable,
  RefusedMissing, IdentityChanged, Disappeared, RefusedState, IoTerminal
};

inline const char* bus_slave_disposition(BusSlaveDisposition disposition) {
  switch (disposition) {
    case BusSlaveDisposition::Held: return "held";
    case BusSlaveDisposition::DeclaredUnused: return "declared_unused";
    case BusSlaveDisposition::RefusedUnknown: return "refused_unknown";
    case BusSlaveDisposition::RefusedIdentity: return "refused_identity";
    case BusSlaveDisposition::IdentityUnreadable: return "identity_unreadable";
    case BusSlaveDisposition::RefusedMissing: return "refused_missing";
    case BusSlaveDisposition::IdentityChanged: return "identity_changed";
    case BusSlaveDisposition::Disappeared: return "disappeared";
    case BusSlaveDisposition::RefusedState: return "refused_state";
    case BusSlaveDisposition::IoTerminal: return "io_terminal";
  }
  return "identity_unreadable";
}

enum class BusAdmissionReason : uint8_t {
  None, IdentityUnreadable, UnknownSlave, DeclaredIdentityMismatch,
  DeclaredSlaveMissing, UnsafeState, CapacityExceeded, IoTerminalNotOperational
};

inline const char* bus_admission_reason(BusAdmissionReason reason) {
  switch (reason) {
    case BusAdmissionReason::None: return "none";
    case BusAdmissionReason::IdentityUnreadable: return "identity_unreadable";
    case BusAdmissionReason::UnknownSlave: return "unknown_slave";
    case BusAdmissionReason::DeclaredIdentityMismatch: return "declared_identity_mismatch";
    case BusAdmissionReason::DeclaredSlaveMissing: return "declared_slave_missing";
    case BusAdmissionReason::UnsafeState: return "unsafe_held_state";
    case BusAdmissionReason::CapacityExceeded: return "capacity_exceeded";
    case BusAdmissionReason::IoTerminalNotOperational: return "io_terminal_not_operational";
  }
  return "identity_unreadable";
}

struct HeldSlave {
  BusSlaveIdentity identity{};
  BusSlaveIdentity observed{};
  BusSlaveIdentity expected{};
  bool io_terminal = false;
  bool declared = false, present = false;
  bool identity_known = false, identity_readable = false;
  BusSlaveDisposition disposition = BusSlaveDisposition::IdentityUnreadable;
  uint8_t al_state = 0;
};

struct alignas(8) BusTopologySnapshot {
  std::array<HeldSlave, kMaxUnusedSlaves> slaves{};
  uint32_t count = 0, responding_expected = 0;
  uint64_t generation = 0;
  bool admitted = false;
  BusAdmissionReason reason = BusAdmissionReason::None;
  uint16_t reason_position = 0;
};

// Publish the retained startup identity and current cached SII observation;
// absence must be explicit rather than silently deleting an unused device.
inline void append_bus_json(std::ostream& out, const BusPolicy& policy, const BusTopologySnapshot& snapshot) {
  out << "{\"unknown_slaves\":\"" << (policy.hold ? "hold" : "refuse")
      << "\",\"held_state_policy\":\"PREOP\",\"unused_change_policy\":\""
      << (policy.hold ? "log_and_continue_axes" : "refuse") << '"'
      << ",\"responding_expected\":" << snapshot.responding_expected
      << ",\"reason\":\"" << bus_admission_reason(snapshot.reason) << '"'
      << ",\"reason_position\":" << (snapshot.reason == BusAdmissionReason::None || snapshot.reason == BusAdmissionReason::CapacityExceeded ? "null" : std::to_string(snapshot.reason_position))
      << ",\"admitted\":" << (snapshot.admitted ? "true" : "false") << ",\"held_slaves\":[";
  for (uint32_t i = 0; i < snapshot.count; ++i) {
    const auto& s = snapshot.slaves[i];
    if (i) out << ',';
    auto number = [](bool known, uint32_t value) { return known ? std::to_string(value) : "null"; };
    out << "{\"slave_position\":" << s.identity.position << ",\"vendor_id\":" << number(s.identity_known, s.identity.vendor_id)
        << ",\"product_code\":" << number(s.identity_known, s.identity.product_code) << ",\"revision\":" << number(s.identity_known, s.identity.revision)
        << ",\"identity_readable\":" << (s.identity_readable ? "true" : "false")
        << ",\"disposition\":\"" << bus_slave_disposition(s.disposition) << '"'
        << ",\"declared_unused\":" << (s.declared ? "true" : "false")
        << ",\"present\":" << (s.present ? "true" : "false") << ",\"al_state\":" << unsigned(s.al_state)
        << ",\"observed_vendor_id\":" << number(s.identity_readable, s.observed.vendor_id)
        << ",\"observed_product_code\":" << number(s.identity_readable, s.observed.product_code)
        << ",\"observed_revision\":" << number(s.identity_readable, s.observed.revision);
    const auto declared = std::find_if(policy.unused.begin(), policy.unused.end(),
        [&](const auto& declaration) { return declaration.position == s.identity.position; });
    if (declared != policy.unused.end())
      out << ",\"expected_vendor_id\":" << declared->vendor_id << ",\"expected_product_code\":" << declared->product_code
          << ",\"expected_revision\":" << number(declared->revision_required, declared->revision);
    if (s.io_terminal)
      out << ",\"expected_vendor_id\":" << s.expected.vendor_id << ",\"expected_product_code\":" << s.expected.product_code
          << ",\"expected_revision\":" << s.expected.revision;
    out << '}';
  }
  out << "]}";
}
} // namespace rosie::rtcore
