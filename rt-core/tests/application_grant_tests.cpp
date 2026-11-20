// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "engine/application_grant.hpp"

#include <cassert>
#include <iostream>
#include <limits>
#include <type_traits>

using namespace rosie::rtcore;

static_assert(std::is_trivially_copyable_v<ApplicationGrantRequest>);
static_assert(std::is_trivially_copyable_v<ApplicationGrantState>);
static_assert(noexcept(poll_application_grant({}, {})));
static_assert(noexcept(evaluate_application_grant({}, {}, {})));

static ApplicationGrantContext context() {
  return {7, 11, 1000, 500, 3, true, true};
}

static ApplicationGrantRequest acquire() {
  return {ApplicationGrantOperation::Acquire, {7, 11, {0x51}, 1}, 900, 1400, 3};
}

static ApplicationGrantState bound(const ApplicationGrantContext& ctx) {
  const auto result = bind_application_grant_connection({}, ctx);
  assert(result.accepted && !result.next.active && !result.revoke_outputs);
  assert(result.next.connection_id == ctx.connection_id);
  return result.next;
}

static ApplicationGrantState active(const ApplicationGrantContext& ctx) {
  const auto result = evaluate_application_grant(bound(ctx), ctx, acquire());
  assert(result.accepted && result.next.active && !result.revoke_outputs);
  return result.next;
}

static ApplicationGrantRequest revoke(const ApplicationGrantIdentity& identity) {
  return {ApplicationGrantOperation::Revoke, identity, 0, 0, 0};
}

static void acquisition_and_renewal() {
  auto ctx = context();
  auto state = active(ctx);
  assert(state.highest_app_grant_generation == 1);
  assert(state.current.deadline_host_ns == 1400);

  auto renewal = acquire();
  renewal.operation = ApplicationGrantOperation::Renew;
  renewal.origin_host_ns = 1100;
  renewal.deadline_host_ns = 1600;
  ctx.now_ns = 1100;
  ctx.inhibited = false; // Renewal needs existing authority, not motion inhibition.
  auto result = evaluate_application_grant(state, ctx, renewal);
  assert(result.accepted && result.next.active && !result.revoke_outputs);
  state = result.next;
  assert(state.current.deadline_host_ns == 1600 && state.highest_app_grant_generation == 1);

  ctx.now_ns = 1200;
  result = evaluate_application_grant(state, ctx, renewal); // Duplicate content cannot restamp its deadline.
  assert(result.accepted && result.next.current.deadline_host_ns == 1600);
  renewal.origin_host_ns = 1000; // An older but still valid interval may shorten authority.
  renewal.deadline_host_ns = 1300;
  result = evaluate_application_grant(result.next, ctx, renewal);
  assert(result.accepted && result.next.current.deadline_host_ns == 1300);
  assert(result.next.current.origin_host_ns == 1000);

  const auto identity = result.next.current.identity;
  result = evaluate_application_grant(result.next, ctx, revoke(identity));
  assert(result.accepted && !result.next.active && result.revoke_outputs);
  assert(result.next.highest_app_grant_generation == 1);
  assert(result.next.current.deadline_host_ns == 1300);
  result = evaluate_application_grant(result.next, ctx, revoke(identity));
  assert(result.accepted && !result.next.active && !result.revoke_outputs);
}

static void invalid_admission_preserves_existing_grant() {
  auto ctx = context();
  const auto state = active(ctx);
  auto request = acquire();
  request.identity.app_grant_generation = 2;
  auto result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Busy);
  assert(result.next.active && !result.revoke_outputs && result.next.highest_app_grant_generation == 1);

  request.operation = ApplicationGrantOperation::Renew;
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::IdentityMismatch);
  request = acquire();
  request.operation = ApplicationGrantOperation::Renew;
  request.identity.capability_id[15] = 1;
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::IdentityMismatch);
  assert(result.next.active && !result.revoke_outputs);

  request = acquire();
  request.identity.app_grant_generation = 99;
  request.deadline_host_ns = 1401; // A rejected high generation cannot consume the floor.
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Invalid);
  assert(result.next.highest_app_grant_generation == 1 && result.next.active);

  const auto original = state.current.identity;
  result = evaluate_application_grant(state, ctx, revoke(original));
  ++ctx.native_generation; // The output owner applies the accepted revoke.
  auto successor = acquire();
  successor.identity.native_generation = ctx.native_generation;
  successor.identity.app_grant_generation = 2;
  successor.identity.capability_id[0] = 0x71;
  result = evaluate_application_grant(result.next, ctx, successor);
  assert(result.accepted && result.next.active);
  const auto successor_state = result.next;
  result = evaluate_application_grant(successor_state, ctx, revoke(original));
  assert(!result.accepted && result.reason == ApplicationGrantReason::IdentityMismatch);
  assert(result.next.active && !result.revoke_outputs);
  assert(same_application_grant_identity(result.next.current.identity, successor.identity));
}

static void expiry_precedes_queued_requests() {
  auto ctx = context();
  const auto state = active(ctx);
  auto renewal = acquire();
  renewal.operation = ApplicationGrantOperation::Renew;
  renewal.origin_host_ns = 1400;
  renewal.deadline_host_ns = 1900;
  ctx.now_ns = 1400; // The old grant expires at equality, before queued renewal is considered.
  auto result = evaluate_application_grant(state, ctx, renewal);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Expired);
  assert(!result.next.active && result.revoke_outputs);
  assert(result.next.current.deadline_host_ns == 1400 && result.next.highest_app_grant_generation == 1);
  result = evaluate_application_grant(result.next, ctx, renewal);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Inactive && !result.revoke_outputs);

  auto fresh = renewal;
  fresh.operation = ApplicationGrantOperation::Acquire;
  fresh.identity.app_grant_generation = 2;
  result = evaluate_application_grant(state, ctx, fresh);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Expired && result.revoke_outputs);
  ++ctx.native_generation; // New admission must observe the invalidation's native fence.
  fresh.identity.native_generation = ctx.native_generation;
  result = evaluate_application_grant(result.next, ctx, fresh);
  assert(result.accepted && result.next.active && !result.revoke_outputs);

  // An identity-only Revoke can acknowledge an expired identity but cannot revive it.
  ctx = context();
  ctx.now_ns = 1400;
  result = evaluate_application_grant(state, ctx, revoke(state.current.identity));
  assert(result.accepted && !result.next.active && result.revoke_outputs);
  result = evaluate_application_grant(result.next, ctx, revoke(state.current.identity));
  assert(result.accepted && !result.revoke_outputs);

  // Poll invalidation must survive malformed or stale requests too.
  auto malformed = renewal;
  malformed.identity.capability_id = {};
  result = evaluate_application_grant(state, ctx, malformed);
  assert(!result.accepted && !result.next.active && result.revoke_outputs);
  assert(result.next.highest_app_grant_generation == 1);
}

static void stop_renew_interleavings_keep_generation_floor() {
  auto ctx = context();
  auto state = active(ctx);
  auto renewal = acquire();
  renewal.operation = ApplicationGrantOperation::Renew;
  ++ctx.native_generation; // Urgent Stop independently fences native output first.
  auto result = evaluate_application_grant(state, ctx, renewal);
  assert(!result.accepted && result.reason == ApplicationGrantReason::NativeGenerationMismatch);
  assert(!result.next.active && result.revoke_outputs && result.next.highest_app_grant_generation == 1);
  state = result.next;
  result = poll_application_grant(state, ctx);
  assert(!result.next.active && !result.revoke_outputs); // No repeated output fence.
  renewal.identity.native_generation = ctx.native_generation;
  result = evaluate_application_grant(state, ctx, renewal);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Inactive);

  auto new_request = acquire();
  new_request.identity.native_generation = ctx.native_generation;
  result = evaluate_application_grant(state, ctx, new_request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::StaleGeneration);
  new_request.identity.app_grant_generation = 2;
  result = evaluate_application_grant(state, ctx, new_request);
  assert(result.accepted && result.next.highest_app_grant_generation == 2);

  // A renewal accepted just before Stop also loses permission at the next poll.
  ctx = context();
  state = active(ctx);
  renewal = acquire();
  renewal.operation = ApplicationGrantOperation::Renew;
  renewal.origin_host_ns = 1000;
  renewal.deadline_host_ns = 1500;
  result = evaluate_application_grant(state, ctx, renewal);
  assert(result.accepted);
  ++ctx.native_generation;
  result = poll_application_grant(result.next, ctx);
  assert(result.revoke_outputs && !result.next.active);
  assert(result.next.current.deadline_host_ns == 1500); // History is retained, permission is not.
}

static void authenticated_connection_is_the_only_floor_reset() {
  auto ctx = context();
  auto state = active(ctx);
  ctx.connection_alive = false;
  auto result = poll_application_grant(state, ctx);
  assert(result.reason == ApplicationGrantReason::Disconnected && result.revoke_outputs);
  state = result.next;
  result = poll_application_grant(state, ctx);
  assert(!result.revoke_outputs && result.next.highest_app_grant_generation == 1);
  ctx.connection_alive = true;
  ++ctx.native_generation;
  result = bind_application_grant_connection(state, ctx); // Rebinding the same id is not a new connection.
  assert(!result.accepted && result.next.highest_app_grant_generation == 1);

  ++ctx.connection_id;
  result = poll_application_grant(state, ctx);
  assert(result.reason == ApplicationGrantReason::ConnectionMismatch);
  assert(result.next.connection_id == state.connection_id && result.next.highest_app_grant_generation == 1);
  auto request = acquire();
  request.identity.connection_id = ctx.connection_id;
  request.identity.native_generation = ctx.native_generation;
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.next.highest_app_grant_generation == 1);
  result = bind_application_grant_connection(state, ctx);
  assert(result.accepted && !result.next.active && result.next.highest_app_grant_generation == 0);
  result = evaluate_application_grant(result.next, ctx, request);
  assert(result.accepted && result.next.highest_app_grant_generation == 1);

  // Replacing an active authenticated connection still revokes the old grant.
  state = result.next;
  ++ctx.connection_id;
  result = bind_application_grant_connection(state, ctx);
  assert(result.accepted && result.revoke_outputs && !result.next.active);
}

static void clock_regression_latches_until_a_new_runtime() {
  auto ctx = context();
  auto state = active(ctx);
  --ctx.now_ns;
  auto result = poll_application_grant(state, ctx);
  assert(result.reason == ApplicationGrantReason::ClockRegression && result.revoke_outputs);
  assert(result.next.clock_failed && result.next.last_now == 1000);
  state = result.next;
  ctx.now_ns = 1100; // Catching up cannot repair an untrusted clock incarnation.
  auto request = acquire();
  request.identity.app_grant_generation = 2;
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::ClockRegression);
  assert(!result.revoke_outputs && result.next.clock_failed);
  ++ctx.connection_id;
  result = bind_application_grant_connection(state, ctx);
  assert(!result.accepted && result.reason == ApplicationGrantReason::ClockRegression);
  assert(result.next.highest_app_grant_generation == 1 && result.next.connection_id == 7);

  // Inactive clocks are checked too; otherwise a future Acquire could bypass the floor.
  ctx = context();
  state = bound(ctx);
  --ctx.now_ns;
  result = poll_application_grant(state, ctx);
  assert(result.next.clock_failed && !result.revoke_outputs);
}

static void invalid_requests_and_contexts() {
  using Reason = ApplicationGrantReason;
  for (unsigned condition = 0; condition < 20; ++condition) {
    auto ctx = context();
    const auto state = bound(ctx);
    auto request = acquire();
    Reason expected = Reason::Invalid;
    switch (condition) {
      case 0: request.operation = static_cast<ApplicationGrantOperation>(0); break;
      case 1: request.identity.capability_id = {}; break;
      case 2: request.identity.connection_id = 0; break;
      case 3: request.identity.native_generation = 0; break;
      case 4: request.identity.app_grant_generation = 0; break;
      case 5: ++request.identity.connection_id; expected = Reason::ConnectionMismatch; break;
      case 6: ++request.identity.native_generation; expected = Reason::NativeGenerationMismatch; break;
      case 7: request.axis_mask = 0; expected = Reason::AxisMaskMismatch; break;
      case 8: request.axis_mask = 1; expected = Reason::AxisMaskMismatch; break;
      case 9: request.axis_mask = 7; expected = Reason::AxisMaskMismatch; break;
      case 10: request.origin_host_ns = 0; break;
      case 11: request.origin_host_ns = ctx.now_ns + 1; break;
      case 12: request.deadline_host_ns = request.origin_host_ns; break;
      case 13: request.deadline_host_ns = ctx.now_ns; expected = Reason::Expired; break;
      case 14: request.deadline_host_ns = 1401; break;
      case 15: ctx.max_lease_ns = 0; break;
      case 16: ctx.configured_mask = 0; break;
      case 17: ctx.native_generation = 0; break;
      case 18: ctx.connection_alive = false; expected = Reason::Disconnected; break;
      case 19: ctx.inhibited = false; expected = Reason::NotInhibited; break;
    }
    const auto result = evaluate_application_grant(state, ctx, request);
    assert(!result.accepted && result.reason == expected && !result.revoke_outputs);
    assert(!result.next.active && result.next.highest_app_grant_generation == 0);
  }
  auto ctx = context();
  auto state = active(ctx);
  auto malformed_revoke = revoke(state.current.identity);
  malformed_revoke.axis_mask = 3;
  auto result = evaluate_application_grant(state, ctx, malformed_revoke);
  assert(!result.accepted && result.reason == Reason::Invalid && result.next.active);
  ctx.configured_mask = 7;
  result = poll_application_grant(state, ctx);
  assert(result.reason == Reason::AxisMaskMismatch && result.revoke_outputs && !result.next.active);
  ctx = context();
  ctx.max_lease_ns = 499;
  result = poll_application_grant(state, ctx);
  assert(result.reason == Reason::Invalid && result.revoke_outputs && !result.next.active);
}

static void fixed_origin_and_saturation() {
  auto ctx = context();
  auto state = bound(ctx);
  auto request = acquire();
  ctx.now_ns = 1399;
  auto result = evaluate_application_grant(state, ctx, request);
  assert(result.accepted && result.next.current.deadline_host_ns == 1400);
  assert(result.next.current.deadline_host_ns - ctx.now_ns == 1); // Delayed admission gains no fresh 500 ticks.
  ctx.now_ns = 1400;
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Expired);

  constexpr auto maximum = std::numeric_limits<uint64_t>::max();
  ctx = context();
  ctx.now_ns = maximum - 1;
  state = bound(ctx);
  request.origin_host_ns = maximum - 500;
  request.deadline_host_ns = maximum;
  request.identity.app_grant_generation = maximum;
  request.identity.capability_id = {};
  request.identity.capability_id[15] = 1; // Any nonzero capability byte qualifies the fixed identity field.
  result = evaluate_application_grant(state, ctx, request);
  assert(result.accepted && result.next.highest_app_grant_generation == maximum);
  state = result.next;
  ctx.now_ns = maximum;
  result = poll_application_grant(state, ctx);
  assert(result.reason == ApplicationGrantReason::Expired && result.revoke_outputs);
  assert(result.next.last_now == maximum && result.next.highest_app_grant_generation == maximum);
  ctx.now_ns = 0;
  result = poll_application_grant(result.next, ctx);
  assert(result.next.clock_failed && result.reason == ApplicationGrantReason::ClockRegression);

  ctx = context();
  state = bound(ctx);
  request = acquire();
  request.identity.app_grant_generation = maximum;
  result = evaluate_application_grant(state, ctx, request);
  assert(result.accepted);
  result = evaluate_application_grant(result.next, ctx, revoke(request.identity));
  state = result.next;
  ++ctx.native_generation;
  request.identity.native_generation = ctx.native_generation;
  request.identity.app_grant_generation = 1;
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::StaleGeneration);
  assert(result.next.highest_app_grant_generation == maximum);
  request.identity.app_grant_generation = 0;
  result = evaluate_application_grant(state, ctx, request);
  assert(!result.accepted && result.reason == ApplicationGrantReason::Invalid);
}

int main() {
  acquisition_and_renewal();
  invalid_admission_preserves_existing_grant();
  expiry_precedes_queued_requests();
  stop_renew_interleavings_keep_generation_floor();
  authenticated_connection_is_the_only_floor_reset();
  clock_regression_latches_until_a_new_runtime();
  invalid_requests_and_contexts();
  fixed_origin_and_saturation();
  std::cout << "Mirrored application grant policy: passed\n";
}
