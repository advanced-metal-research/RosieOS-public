#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rosie::rtcore {

inline constexpr uint32_t jog_session_axes = 16;
inline constexpr uint32_t jog_session_axis_mask = (1u << jog_session_axes) - 1;

struct JogIdentity {
  uint64_t connection_id = 0;
  uint64_t native_generation = 0;
  std::array<uint8_t, 16> capability_id{}; // Mirrored 128-bit application grant.
  uint64_t app_grant_generation = 0;
  uint64_t configuration_epoch = 0;
  uint64_t home_epoch = 0;
  uint64_t clock_mapping_generation = 0;

  bool operator==(const JogIdentity& other) const noexcept {
    return connection_id == other.connection_id && native_generation == other.native_generation &&
        capability_id == other.capability_id && app_grant_generation == other.app_grant_generation &&
        configuration_epoch == other.configuration_epoch && home_epoch == other.home_epoch &&
        clock_mapping_generation == other.clock_mapping_generation;
  }
  bool operator!=(const JogIdentity& other) const noexcept { return !(*this == other); }
  bool valid() const noexcept {
    bool nonzero_grant = false;
    for (const auto byte : capability_id) nonzero_grant |= byte != 0;
    return connection_id && native_generation && nonzero_grant &&
        app_grant_generation && configuration_epoch && home_epoch && clock_mapping_generation;
  }
};

// One snapshot collected by the cyclic owner. ready_mask already combines
// current configuration/home, valid limits, fresh feedback, CSP and enabled
// drive readiness. conflicting_motion excludes this session's own jog mode.
// Timing budgets and clock_mapping_valid come from trusted host qualification,
// never from an update packet or an authority renewal requesting a longer age.
struct JogSessionContext {
  JogIdentity identity{};
  uint64_t now_ns = 0;
  uint64_t grant_deadline_ns = 0; // Trusted mirrored absolute grant expiry.
  uint64_t max_input_age_ns = 0;
  uint64_t max_first_input_wait_ns = 0;
  uint32_t configured_mask = 0;
  uint32_t allowed_mask = 0;
  uint32_t ready_mask = 0;
  bool authority_valid = false;
  bool clock_mapping_valid = false;
  bool armed = false;
  bool conflicting_motion = false;
  bool commissioning_active = false;
};

struct JogBeginRequest {
  JogIdentity identity{};
  uint64_t jog_generation = 0;
  uint32_t axis_mask = 0;
  // Fixed before queueing Begin; native does not restart the first-input wait.
  uint64_t earliest_origin_host_ns = 0;
  uint64_t first_input_deadline_host_ns = 0;
};

struct JogUpdateRequest {
  JogIdentity identity{};
  uint64_t jog_generation = 0;
  uint64_t source_sequence = 0;
  uint64_t earliest_origin_host_ns = 0;
  uint64_t immutable_deadline_host_ns = 0;
  uint32_t axis_mask = 0;
  std::array<double, jog_session_axes> velocity{};
};

struct JogEndRequest {
  JogIdentity identity{};
  uint64_t jog_generation = 0;
};

struct JogSessionState {
  JogIdentity identity{};
  uint64_t highest_generation = 0;
  uint64_t closed_through_generation = 0;
  uint64_t generation = 0;
  uint64_t source_sequence = 0;
  uint64_t begin_origin_ns = 0;
  uint64_t input_origin_ns = 0;
  uint64_t input_deadline_ns = 0;
  uint64_t last_evaluated_ns = 0;
  uint64_t revoked_connection_id = 0;
  uint64_t revoked_native_generation = 0;
  uint32_t axis_mask = 0; // Begin owns this fixed group until it closes.
  uint32_t input_axis_mask = 0;
  std::array<double, jog_session_axes> velocity{};
  bool open = false;
  bool has_input = false;
  bool clock_failed = false;
};

enum class JogSessionReason : uint8_t {
  None, Closed, ModeConflict, NotReady, WrongIdentity, InvalidGeneration,
  StaleSequence, InvalidVector, InvalidTiming, Expired, AuthorityLost,
  ClockReset, GenerationExhausted, Ramping, Limited,
  // Measured native jog code 15, checked against the generated schema in JogLane.
  OutsideLimitsOutward,
  // Path jog (protocol 1.14) codes 16-23, checked against the generated schema
  // in JogLane: sample 0 is not the held command, ring full, chunk or index
  // not contiguous, nonfinite or non-zero unmasked sample, append after the
  // path ended, generator fault (fail closed), path jog unavailable on this
  // cell or session, invalid spacing/ceiling/limits.
  PathStartMismatch, PathCapacity, PathOutOfOrder, PathInvalidSample, PathEnded,
  PathFault, PathUnavailable, PathInvalidConfig,
};

struct JogSessionDecision {
  JogSessionState next{};
  JogSessionReason reason = JogSessionReason::None;
  bool accepted = false;
  bool closed = false;
  bool inhibit = false;
  // This identifies the End/expiry handoff needing qualified stopping. This
  // policy does not implement a halt or authorize an enabled hold.
  bool needs_qualified_halt = false;
};

inline void close_jog_session(JogSessionDecision& out, JogSessionReason reason,
                              bool needs_halt) noexcept {
  if (out.next.open) {
    if (out.next.generation > out.next.closed_through_generation)
      out.next.closed_through_generation = out.next.generation;
    out.closed = true;
    out.inhibit = true;
    out.needs_qualified_halt = needs_halt && out.next.has_input;
  }
  out.next.open = false;
  out.next.has_input = false;
  out.next.axis_mask = 0;
  out.next.input_axis_mask = 0;
  out.next.input_deadline_ns = 0;
  out.next.velocity.fill(0);
  out.reason = reason;
}

inline bool jog_group_ready(const JogSessionContext& context, uint32_t mask) noexcept {
  return mask && !(mask & ~jog_session_axis_mask) &&
      !(context.configured_mask & ~jog_session_axis_mask) &&
      !(mask & ~context.configured_mask) && !(mask & ~context.allowed_mask) &&
      (mask & context.ready_mask) == mask && context.armed &&
      context.max_input_age_ns && context.max_first_input_wait_ns;
}

inline bool valid_jog_deadline(uint64_t origin, uint64_t deadline,
                               uint64_t now, uint64_t budget) noexcept {
  // Ordered subtraction cannot overflow, including hostile wrapped deadlines.
  return origin && origin <= now && now < deadline && budget &&
      deadline - origin <= budget;
}

// Poll every cycle, independently of arriving input or authority renewal. A
// rejected packet cannot postpone any deadline. Clock regression is latched for
// this host-monotonic incarnation; recovery needs a new trusted helper instance.
inline JogSessionDecision evaluate_jog_session(const JogSessionState& state,
                                                const JogSessionContext& context) noexcept {
  JogSessionDecision out{state};
  if (state.clock_failed || !context.now_ns || context.now_ns < state.last_evaluated_ns) {
    close_jog_session(out, JogSessionReason::ClockReset, false);
    out.next.clock_failed = true;
    out.inhibit = true;
    return out;
  }
  out.next.last_evaluated_ns = context.now_ns;
  if (!context.authority_valid || !context.identity.valid() ||
      context.now_ns >= context.grant_deadline_ns) {
    const auto identity = state.open ? state.identity : context.identity;
    out.next.revoked_connection_id = identity.connection_id;
    out.next.revoked_native_generation = identity.native_generation;
    close_jog_session(out, JogSessionReason::AuthorityLost, false);
    return out;
  }
  if (!context.clock_mapping_valid) {
    close_jog_session(out, JogSessionReason::ClockReset, false);
    return out;
  }
  if (!state.open) return out;
  if (state.identity != context.identity) {
    close_jog_session(out, state.identity.clock_mapping_generation != context.identity.clock_mapping_generation
        ? JogSessionReason::ClockReset : JogSessionReason::WrongIdentity, false);
  } else if (context.conflicting_motion || context.commissioning_active) {
    close_jog_session(out, JogSessionReason::ModeConflict, false);
  } else if (!jog_group_ready(context, state.axis_mask)) {
    close_jog_session(out, JogSessionReason::NotReady, false);
  } else if (context.now_ns >= state.input_deadline_ns ||
             (state.has_input && context.now_ns - state.input_origin_ns >= context.max_input_age_ns)) {
    close_jog_session(out, JogSessionReason::Expired, true);
  }
  return out;
}

inline JogSessionDecision evaluate_jog_begin(const JogSessionState& state,
    const JogSessionContext& context, const JogBeginRequest& request) noexcept {
  auto out = evaluate_jog_session(state, context);
  if (out.reason != JogSessionReason::None) return out;
  if (request.identity != context.identity) out.reason = JogSessionReason::WrongIdentity;
  else if (state.open || context.conflicting_motion || context.commissioning_active)
    out.reason = JogSessionReason::ModeConflict;
  else if (context.identity.connection_id == state.revoked_connection_id &&
           context.identity.native_generation == state.revoked_native_generation)
    out.reason = JogSessionReason::AuthorityLost;
  else if (!jog_group_ready(context, request.axis_mask)) out.reason = JogSessionReason::NotReady;
  else if (state.highest_generation == UINT64_MAX || state.closed_through_generation == UINT64_MAX)
    out.reason = JogSessionReason::GenerationExhausted;
  else if (!request.jog_generation || request.jog_generation <= state.highest_generation ||
           request.jog_generation <= state.closed_through_generation)
    out.reason = JogSessionReason::InvalidGeneration;
  // Begin's origin fences source capture; its separate wait starts at admission.
  // Calibration uncertainty must not consume that wait or shift captured input.
  else if (!request.earliest_origin_host_ns || request.earliest_origin_host_ns > context.now_ns ||
           context.now_ns - request.earliest_origin_host_ns >= context.max_input_age_ns ||
           !valid_jog_deadline(context.now_ns, request.first_input_deadline_host_ns,
                               context.now_ns, context.max_first_input_wait_ns) ||
           request.first_input_deadline_host_ns > context.grant_deadline_ns)
    out.reason = JogSessionReason::InvalidTiming;
  else {
    out.next.identity = request.identity;
    out.next.generation = request.jog_generation;
    out.next.highest_generation = request.jog_generation;
    out.next.source_sequence = 0;
    out.next.begin_origin_ns = request.earliest_origin_host_ns;
    out.next.input_origin_ns = 0;
    out.next.input_deadline_ns = request.first_input_deadline_host_ns;
    out.next.axis_mask = request.axis_mask;
    out.next.input_axis_mask = 0;
    out.next.velocity.fill(0);
    out.next.open = true;
    out.next.has_input = false;
    out.accepted = true;
  }
  return out;
}

inline JogSessionDecision evaluate_jog_update(const JogSessionState& state,
    const JogSessionContext& context, const JogUpdateRequest& request) noexcept {
  auto out = evaluate_jog_session(state, context);
  if (out.reason != JogSessionReason::None) return out;
  if (!state.open) out.reason = JogSessionReason::Closed;
  else if (request.identity != state.identity) out.reason = JogSessionReason::WrongIdentity;
  else if (!request.jog_generation || request.jog_generation != state.generation)
    out.reason = JogSessionReason::InvalidGeneration;
  else if (!request.source_sequence || request.source_sequence <= state.source_sequence)
    out.reason = JogSessionReason::StaleSequence;
  else if (!request.axis_mask || (request.axis_mask & ~state.axis_mask))
    out.reason = JogSessionReason::InvalidVector;
  else if (request.earliest_origin_host_ns < state.begin_origin_ns ||
           !valid_jog_deadline(request.earliest_origin_host_ns, request.immutable_deadline_host_ns,
                               context.now_ns, context.max_input_age_ns) ||
           request.immutable_deadline_host_ns > context.grant_deadline_ns ||
           (state.has_input && (request.earliest_origin_host_ns < state.input_origin_ns ||
             (request.earliest_origin_host_ns == state.input_origin_ns &&
              request.immutable_deadline_host_ns > state.input_deadline_ns))))
    out.reason = JogSessionReason::InvalidTiming;
  else {
    for (const auto value : request.velocity) {
      if (!std::isfinite(value)) {
        out.reason = JogSessionReason::InvalidVector;
        return out;
      }
    }
    out.next.source_sequence = request.source_sequence;
    out.next.input_origin_ns = request.earliest_origin_host_ns;
    out.next.input_deadline_ns = request.immutable_deadline_host_ns;
    out.next.input_axis_mask = request.axis_mask;
    for (uint32_t axis = 0; axis < jog_session_axes; ++axis)
      out.next.velocity[axis] = request.axis_mask & (1u << axis) ? request.velocity[axis] : 0;
    out.next.has_input = true;
    out.accepted = true;
  }
  return out;
}

inline JogSessionDecision evaluate_jog_end(const JogSessionState& state,
    const JogSessionContext& context, const JogEndRequest& request) noexcept {
  auto out = evaluate_jog_session(state, context);
  if (request.identity != context.identity || !context.identity.valid()) {
    if (out.reason == JogSessionReason::None) out.reason = JogSessionReason::WrongIdentity;
    return out;
  }
  if (!request.jog_generation) {
    if (out.reason == JogSessionReason::None) out.reason = JogSessionReason::InvalidGeneration;
    return out;
  }
  // Cancellation is safe even while readiness/authority is being revoked. The
  // matching current identity may close a queued Begin that has not arrived.
  if (request.jog_generation > out.next.closed_through_generation)
    out.next.closed_through_generation = request.jog_generation;
  if (out.next.open && out.next.generation <= request.jog_generation)
    close_jog_session(out, JogSessionReason::None, true);
  out.accepted = true;
  return out;
}

// Urgent Stop closes all input and fences queued Begins from this native
// generation, even if no Begin had reached the cyclic owner yet. Floors remain
// monotonic across authority changes; native reserves the next Begin generation.
inline JogSessionDecision revoke_jog_session(const JogSessionState& state,
                                             const JogIdentity& identity) noexcept {
  JogSessionDecision out{state};
  close_jog_session(out, JogSessionReason::AuthorityLost, false);
  out.next.revoked_connection_id = identity.connection_id;
  out.next.revoked_native_generation = identity.native_generation;
  out.inhibit = true;
  out.accepted = true;
  return out;
}

} // namespace rosie::rtcore
