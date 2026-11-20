#pragma once

#include <cstdint>
#include <limits>

namespace rosie::rtcore {

// Generic SDO readback preserves U32 wire bits in an int32 container. Recover
// the declared numeric value before applying the profile's logical offset sign.
inline bool home_offset_from_scalar_bits(int32_t raw, bool unsigned32, int sign,
                                        int32_t& result) noexcept {
  if (sign != 1 && sign != -1) return false;
  const int64_t value = unsigned32 ? int64_t(uint32_t(raw)) : int64_t(raw);
  const int64_t offset = sign == -1 ? -value : value;
  if (offset < std::numeric_limits<int32_t>::min() ||
      offset > std::numeric_limits<int32_t>::max()) return false;
  result = static_cast<int32_t>(offset);
  return true;
}

// One RT owner evaluates these fixed requests. A mailbox worker retains the
// original job identity throughout every axis/step and owns no output permit.
enum class CommissioningPhase : uint32_t { Idle, Active, Draining };
enum class CommissioningOperation : uint32_t { Reserve, Service, Release, CompleteHome, FailHome, Finish, ReserveReference, CompleteReference, FailReference };
enum class CommissioningReason : uint32_t { None, Invalid, Busy, NotReady, Cancelled, CycleDeadline };

// Stable names preserve the cyclic refusal in worker diagnostics.
inline const char* commissioning_reason_name(CommissioningReason reason) noexcept {
  switch (reason) {
    case CommissioningReason::None: return "none";
    case CommissioningReason::Invalid: return "invalid";
    case CommissioningReason::Busy: return "busy";
    case CommissioningReason::NotReady: return "not_ready";
    case CommissioningReason::Cancelled: return "cancelled";
    case CommissioningReason::CycleDeadline: return "cycle_deadline";
  }
  return "unknown";
}

struct CommissioningIdentity {
  uint64_t job = 0;
  uint64_t generation = 0;
  uint64_t configuration_epoch = 0;
  uint64_t home_epoch = 0;
};

inline bool same_commissioning_identity(const CommissioningIdentity& a,
                                       const CommissioningIdentity& b) noexcept {
  return a.job == b.job && a.generation == b.generation &&
      a.configuration_epoch == b.configuration_epoch && a.home_epoch == b.home_epoch;
}

struct CommissioningRequest {
  uint64_t ticket = 0;
  CommissioningIdentity identity{};
  CommissioningOperation operation = CommissioningOperation::Reserve;
  uint32_t axis_mask = 0;
  int32_t mode = 0;
  uint32_t controlword = 0;
  int32_t home_offset = 0;
  uint32_t abort_code = 0;
  // Reserve for a configuration (SDO) write: the axes need not be verified yet,
  // because the write is how a mismatched parameter is corrected. Every other
  // permission (authority, bus ready, disarmed, no motion) still applies.
  bool configuration_write = false;
};

struct CommissioningContext {
  uint64_t generation = 0;
  uint64_t configuration_epoch = 0;
  uint64_t home_epoch = 0;
  uint32_t configured_mask = 0;
  uint32_t verified_mask = 0;
  bool authority_valid = false;
  bool bus_ready = false;
  bool armed = false;
  bool motion_active = false;
  bool shutdown = false;
  CommissioningReason cancellation_reason = CommissioningReason::Cancelled;
};

struct CommissioningState {
  CommissioningIdentity identity{};
  CommissioningPhase phase = CommissioningPhase::Idle;
  uint32_t axis_mask = 0;
  uint32_t service_mask = 0;
  uint32_t completed_mask = 0;
  CommissioningReason cancellation_reason = CommissioningReason::None;
  // Verified-delta displacement acquisition for an HM35 v3 anchor; never a Home.
  bool reference_acquisition = false;
};

struct CommissioningDecision {
  CommissioningState state{};
  CommissioningReason reason = CommissioningReason::None;
  bool accepted = false;
  bool job_completed = false; // Finish cleanup acceptance alone is not success.
  uint32_t clear_service_mask = 0;
  uint32_t invalidate_home_mask = 0;
  uint32_t set_service_mask = 0;
  uint32_t complete_home_mask = 0;
  uint32_t fail_home_mask = 0;
  uint32_t complete_reference_mask = 0, fail_reference_mask = 0;
};

inline bool commissioning_permission(const CommissioningIdentity& identity,
                                    const CommissioningContext& context) noexcept {
  return identity.job && identity.generation && identity.configuration_epoch && identity.home_epoch &&
      identity.generation == context.generation &&
      identity.configuration_epoch == context.configuration_epoch &&
      identity.home_epoch == context.home_epoch &&
      context.authority_valid && context.bus_ready && !context.shutdown &&
      !context.armed && !context.motion_active;
}

// Cancellation removes permission immediately but never pretends a blocked
// mailbox call has returned. A matching Finish is the only release of Draining.
inline CommissioningDecision poll_commissioning(CommissioningState state,
                                               const CommissioningContext& context) noexcept {
  CommissioningDecision result{state};
  if (state.phase == CommissioningPhase::Active &&
      (!commissioning_permission(state.identity, context) ||
       (context.verified_mask & state.axis_mask) != state.axis_mask)) {
    result.state.phase = CommissioningPhase::Draining;
    result.clear_service_mask = state.service_mask;
    result.state.service_mask = 0;
    result.state.cancellation_reason = context.cancellation_reason;
    result.reason = context.cancellation_reason;
  }
  return result;
}

inline CommissioningDecision evaluate_commissioning(CommissioningState state,
    const CommissioningContext& context, const CommissioningRequest& request) noexcept {
  auto result = poll_commissioning(state, context);
  state = result.state;
  result.reason = CommissioningReason::Invalid;
  if (!request.ticket || !request.identity.job || !request.identity.generation ||
      !request.identity.configuration_epoch || !request.identity.home_epoch) return result;

  if (request.operation == CommissioningOperation::Reserve || request.operation == CommissioningOperation::ReserveReference) {
    if (state.phase != CommissioningPhase::Idle) {
      result.reason = CommissioningReason::Busy;
      return result;
    }
    if (!request.axis_mask || (request.axis_mask & ~context.configured_mask) ||
        request.mode || request.controlword || request.home_offset || request.abort_code ||
        context.home_epoch == std::numeric_limits<uint64_t>::max()) return result;
    if (!commissioning_permission(request.identity, context) ||
        (!request.configuration_write &&
         (context.verified_mask & request.axis_mask) != request.axis_mask)) {
      result.reason = CommissioningReason::NotReady;
      return result;
    }
    result.state = {request.identity, CommissioningPhase::Active, request.axis_mask, 0, 0};
    // Reservation invalidates the previous home incarnation once for the whole
    // job. Its receipt supplies this identity; every later step retains it.
    result.state.reference_acquisition = request.operation == CommissioningOperation::ReserveReference;
    ++result.state.identity.home_epoch;
    result.invalidate_home_mask = request.axis_mask;
  } else {
    if (state.phase == CommissioningPhase::Idle ||
        !same_commissioning_identity(state.identity, request.identity)) {
      result.reason = CommissioningReason::Cancelled;
      return result;
    }
    if (request.operation == CommissioningOperation::Finish) {
      if (request.axis_mask || request.mode || request.controlword || request.home_offset || request.abort_code) return result;
      result.job_completed = state.phase == CommissioningPhase::Active &&
          state.completed_mask == state.axis_mask && state.service_mask == 0;
      result.clear_service_mask |= state.service_mask;
      result.state = {};
    } else {
      // A matching failure is diagnostic only; cancellation must not discard
      // its abort code. It cannot restore service or complete Home while draining.
      if (state.phase != CommissioningPhase::Active && request.operation != CommissioningOperation::FailHome && request.operation != CommissioningOperation::FailReference) {
        result.reason = state.cancellation_reason == CommissioningReason::None
            ? CommissioningReason::Cancelled : state.cancellation_reason;
        return result;
      }
      const uint32_t mask = request.axis_mask;
      if (!mask || (mask & (mask - 1)) || (mask & ~state.axis_mask)) return result;
      switch (request.operation) {
        case CommissioningOperation::Service:
          if (state.reference_acquisition) return result;
          if (request.mode < -128 || request.mode > 127 || request.controlword > 0xffff ||
              request.home_offset || request.abort_code || (mask & state.completed_mask)) return result;
          result.state.service_mask |= mask;
          result.set_service_mask = mask;
          break;
        case CommissioningOperation::Release:
          if (request.mode || request.controlword || request.home_offset || request.abort_code) return result;
          result.state.service_mask &= ~mask;
          result.clear_service_mask |= mask;
          break;
        case CommissioningOperation::CompleteReference:
          if (!state.reference_acquisition || request.mode || request.controlword || request.abort_code ||
              state.service_mask || (mask & state.completed_mask)) return result;
          result.state.completed_mask |= mask; result.complete_reference_mask = mask;
          break;
        case CommissioningOperation::CompleteHome:
          if (state.reference_acquisition) return result;
          if (request.mode || request.controlword || request.abort_code || (mask & state.service_mask) ||
              (mask & state.completed_mask)) return result;
          result.state.completed_mask |= mask;
          result.complete_home_mask = mask;
          break;
        case CommissioningOperation::FailReference:
          if (!state.reference_acquisition || request.mode || request.controlword || request.home_offset ||
              (mask & state.completed_mask)) return result;
          result.state.phase = CommissioningPhase::Draining; result.fail_reference_mask = mask;
          break;
        case CommissioningOperation::FailHome:
          if (state.reference_acquisition) return result;
          if (request.mode || request.controlword || request.home_offset || (mask & state.completed_mask)) return result;
          result.clear_service_mask |= state.service_mask;
          result.state.service_mask = 0;
          result.state.phase = CommissioningPhase::Draining;
          result.fail_home_mask = mask;
          break;
        default:
          return result;
      }
    }
  }
  result.accepted = true;
  result.reason = CommissioningReason::None;
  return result;
}

} // namespace rosie::rtcore
