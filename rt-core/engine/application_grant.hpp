#pragma once

#include <array>
#include <cstdint>

namespace rosie::rtcore {

// This is native enforcement of the adapter's existing application grant. It
// does not authenticate callers, mint capabilities or create another authority.
enum class ApplicationGrantOperation : uint32_t { Acquire = 1, Renew = 2, Revoke = 3 };
enum class ApplicationGrantReason : uint32_t {
  None = 0,
  Invalid = 1,
  ConnectionMismatch = 2,
  NativeGenerationMismatch = 3,
  Disconnected = 4,
  Busy = 5,
  NotInhibited = 6,
  StaleGeneration = 7,
  IdentityMismatch = 8,
  Expired = 9,
  ClockRegression = 10,
  Inactive = 11,
  AxisMaskMismatch = 12,
};

struct ApplicationGrantIdentity {
  uint64_t connection_id = 0;
  uint64_t native_generation = 0;
  std::array<uint8_t, 16> capability_id{};
  uint64_t app_grant_generation = 0;
};

inline bool same_application_grant_identity(const ApplicationGrantIdentity& a,
                                           const ApplicationGrantIdentity& b) noexcept {
  return a.connection_id == b.connection_id &&
      a.native_generation == b.native_generation &&
      a.capability_id == b.capability_id &&
      a.app_grant_generation == b.app_grant_generation;
}

struct ApplicationGrantRequest {
  ApplicationGrantOperation operation = ApplicationGrantOperation::Acquire;
  ApplicationGrantIdentity identity{};
  uint64_t origin_host_ns = 0;
  uint64_t deadline_host_ns = 0;
  uint32_t axis_mask = 0;
};

// Supplied by the RT owner. connection_id is an authenticated connection's
// nonreused incarnation, never copied from the request. max_lease_ns is trusted
// compiled per-cell ceiling (LAN defaults to 500 ms), not a producer
// claim. Native revoke/Stop changes native_generation independently of this state.
struct ApplicationGrantContext {
  uint64_t connection_id = 0;
  uint64_t native_generation = 0;
  uint64_t now_ns = 0;
  uint64_t max_lease_ns = 0;
  uint32_t configured_mask = 0;
  bool connection_alive = false;
  bool inhibited = false;
};

struct ApplicationGrantState {
  uint64_t connection_id = 0;
  uint64_t highest_app_grant_generation = 0;
  // Last accepted Acquire/Renew, retained while inactive for idempotent Revoke
  // and observation. Retention does not convey permission; active is required.
  ApplicationGrantRequest current{};
  bool active = false;
  uint64_t last_now = 0;
  bool clock_failed = false; // Latched for this runtime; a connection is not a new host clock.
};

struct ApplicationGrantDecision {
  bool accepted = false;
  ApplicationGrantState next{};
  ApplicationGrantReason reason = ApplicationGrantReason::None;
  bool revoke_outputs = false;
};

inline bool valid_application_grant_identity(const ApplicationGrantIdentity& identity) noexcept {
  if (!identity.connection_id || !identity.native_generation || !identity.app_grant_generation)
    return false;
  for (const auto byte : identity.capability_id) if (byte) return true;
  return false;
}

// Call every RT cycle, including cycles with no requests. A transition from
// active to inactive emits exactly one revoke effect; later polls retain the
// generation floor without repeating it. NativeGenerationMismatch means the
// caller has already changed its native fence, so it may clear output permission
// without incrementing that fence a second time. No clock reads or allocation.
inline ApplicationGrantDecision poll_application_grant(ApplicationGrantState state,
    const ApplicationGrantContext& context) noexcept {
  ApplicationGrantDecision result{false, state};
  if (state.clock_failed || context.now_ns < state.last_now) {
    result.reason = ApplicationGrantReason::ClockRegression;
    result.next.clock_failed = true;
  } else {
    result.next.last_now = context.now_ns;
    if (!context.connection_id || context.connection_id != state.connection_id)
      result.reason = ApplicationGrantReason::ConnectionMismatch;
    else if (!context.connection_alive)
      result.reason = ApplicationGrantReason::Disconnected;
    else if (!context.native_generation || !context.max_lease_ns || !context.configured_mask)
      result.reason = ApplicationGrantReason::Invalid;
    else if (state.active) {
      if (state.current.identity.native_generation != context.native_generation)
        result.reason = ApplicationGrantReason::NativeGenerationMismatch;
      else if (state.current.axis_mask != context.configured_mask)
        result.reason = ApplicationGrantReason::AxisMaskMismatch;
      else if (context.now_ns >= state.current.deadline_host_ns)
        result.reason = ApplicationGrantReason::Expired;
      else if (!state.current.origin_host_ns ||
               state.current.deadline_host_ns <= state.current.origin_host_ns ||
               state.current.deadline_host_ns - state.current.origin_host_ns > context.max_lease_ns)
        result.reason = ApplicationGrantReason::Invalid;
    }
  }
  if (result.reason != ApplicationGrantReason::None && state.active) {
    result.next.active = false;
    result.revoke_outputs = true;
  }
  return result;
}

// Call only after the bootstrap owner has authenticated a NEW connection. A
// repeated identifier cannot reset its floor. A mismatching request or ordinary
// context poll never binds a connection. The caller must never reuse connection
// identifiers within this runtime; clock regression still fails closed here.
inline ApplicationGrantDecision bind_application_grant_connection(ApplicationGrantState state,
    const ApplicationGrantContext& context) noexcept {
  auto result = poll_application_grant(state, context);
  if (result.next.clock_failed) return result;
  if (!context.connection_id || !context.native_generation || !context.max_lease_ns ||
      !context.configured_mask || context.connection_id == state.connection_id) {
    result.reason = ApplicationGrantReason::Invalid;
    return result;
  }
  if (!context.connection_alive) {
    result.reason = ApplicationGrantReason::Disconnected;
    return result;
  }
  if (!context.inhibited) {
    result.reason = ApplicationGrantReason::NotInhibited;
    return result;
  }
  result.next = {};
  result.next.connection_id = context.connection_id;
  result.next.last_now = context.now_ns;
  result.accepted = true;
  result.reason = ApplicationGrantReason::None;
  return result;
}

inline ApplicationGrantReason validate_application_grant_interval(const ApplicationGrantRequest& request,
    const ApplicationGrantContext& context) noexcept {
  if (!request.origin_host_ns || request.origin_host_ns > context.now_ns ||
      request.deadline_host_ns <= request.origin_host_ns)
    return ApplicationGrantReason::Invalid;
  if (context.now_ns >= request.deadline_host_ns) return ApplicationGrantReason::Expired;
  // Subtract only after ordering checks. Receipt never restamps origin or adds
  // a fresh relative lease to now, so a delayed request cannot gain lifetime.
  if (request.deadline_host_ns - request.origin_host_ns > context.max_lease_ns)
    return ApplicationGrantReason::Invalid;
  return ApplicationGrantReason::None;
}

// Evaluate at the RT boundary before publishing permission. The bridge owns
// request tickets, duplicate-ticket suppression and correlated receipts. Always
// apply next/revoke_outputs even when accepted is false. A polling invalidation
// prevents Acquire/Renew in that evaluation; the next request must observe the
// resulting native fence. Matching Revoke remains an idempotent cancellation.
inline ApplicationGrantDecision evaluate_application_grant(ApplicationGrantState state,
    const ApplicationGrantContext& context, const ApplicationGrantRequest& request) noexcept {
  auto result = poll_application_grant(state, context);
  if (request.operation != ApplicationGrantOperation::Acquire &&
      request.operation != ApplicationGrantOperation::Renew &&
      request.operation != ApplicationGrantOperation::Revoke) {
    result.reason = ApplicationGrantReason::Invalid;
    return result;
  }
  if (!valid_application_grant_identity(request.identity)) {
    result.reason = ApplicationGrantReason::Invalid;
    return result;
  }
  if (request.operation == ApplicationGrantOperation::Revoke) {
    // Revoke has a canonical identity-only payload, requiring no unexpired
    // lease. It may acknowledge the matching already-expired/revoked identity.
    if (request.origin_host_ns || request.deadline_host_ns || request.axis_mask) {
      result.reason = ApplicationGrantReason::Invalid;
      return result;
    }
    if (!same_application_grant_identity(request.identity, result.next.current.identity)) {
      result.reason = ApplicationGrantReason::IdentityMismatch;
      return result;
    }
    result.revoke_outputs = result.revoke_outputs || result.next.active;
    result.next.active = false;
    result.accepted = true;
    result.reason = ApplicationGrantReason::None;
    return result;
  }
  if (result.reason != ApplicationGrantReason::None) return result;
  if (request.identity.connection_id != context.connection_id) {
    result.reason = ApplicationGrantReason::ConnectionMismatch;
    return result;
  }
  if (request.identity.native_generation != context.native_generation) {
    result.reason = ApplicationGrantReason::NativeGenerationMismatch;
    return result;
  }
  if (request.axis_mask != context.configured_mask) {
    result.reason = ApplicationGrantReason::AxisMaskMismatch;
    return result;
  }
  result.reason = validate_application_grant_interval(request, context);
  if (result.reason != ApplicationGrantReason::None) return result;
  if (request.operation == ApplicationGrantOperation::Acquire) {
    if (result.next.active) {
      result.reason = ApplicationGrantReason::Busy;
      return result;
    }
    if (!context.inhibited) {
      result.reason = ApplicationGrantReason::NotInhibited;
      return result;
    }
    if (request.identity.app_grant_generation <= result.next.highest_app_grant_generation) {
      result.reason = ApplicationGrantReason::StaleGeneration;
      return result;
    }
    result.next.highest_app_grant_generation = request.identity.app_grant_generation;
  } else {
    if (!result.next.active) {
      result.reason = ApplicationGrantReason::Inactive;
      return result;
    }
    if (!same_application_grant_identity(request.identity, result.next.current.identity)) {
      result.reason = ApplicationGrantReason::IdentityMismatch;
      return result;
    }
  }
  result.next.current = request;
  result.next.active = true;
  result.accepted = true;
  result.reason = ApplicationGrantReason::None;
  return result;
}

} // namespace rosie::rtcore
