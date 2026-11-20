// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "engine/jog_session.hpp"
#include <cassert>
#include <limits>
#include <type_traits>

using namespace rosie::rtcore;

static JogSessionContext context() {
  JogSessionContext c{};
  c.identity.connection_id = 10;
  c.identity.native_generation = 20;
  c.identity.capability_id[0] = 0x31;
  c.identity.capability_id[15] = 0xa5;
  c.identity.app_grant_generation = 30;
  c.identity.configuration_epoch = 40;
  c.identity.home_epoch = 50;
  c.identity.clock_mapping_generation = 60;
  c.now_ns = 1000;
  c.grant_deadline_ns = 2000;
  c.max_input_age_ns = 100;
  c.max_first_input_wait_ns = 50;
  c.configured_mask = c.allowed_mask = c.ready_mask = 3;
  c.authority_valid = c.clock_mapping_valid = c.armed = true;
  return c;
}

static JogBeginRequest begin(const JogSessionContext& c, uint64_t generation = 1) {
  return {c.identity, generation, 3, c.now_ns, c.now_ns + 50};
}

static JogUpdateRequest update(const JogSessionContext& c, uint64_t generation = 1, uint64_t sequence = 1) {
  JogUpdateRequest request{};
  request.identity = c.identity;
  request.jog_generation = generation;
  request.source_sequence = sequence;
  request.earliest_origin_host_ns = c.now_ns;
  request.immutable_deadline_host_ns = c.now_ns + 80;
  request.axis_mask = 3;
  request.velocity[0] = 0.5;
  request.velocity[1] = -0.25;
  return request;
}

static JogSessionState active(JogSessionContext& c) {
  const auto opened = evaluate_jog_begin({}, c, begin(c));
  assert(opened.accepted && opened.next.open && !opened.next.has_input);
  assert(opened.next.velocity == (std::array<double, 16>{}));
  assert(opened.next.source_sequence == 0 && opened.next.input_deadline_ns == 1050);
  c.now_ns = 1010;
  const auto accepted = evaluate_jog_update(opened.next, c, update(c));
  assert(accepted.accepted && accepted.next.has_input);
  return accepted.next;
}

static void unchanged_input(const JogSessionState& state, const JogSessionDecision& rejected,
                             JogSessionReason reason) {
  assert(!rejected.accepted && !rejected.closed && !rejected.inhibit);
  assert(rejected.reason == reason);
  assert(rejected.next.source_sequence == state.source_sequence);
  assert(rejected.next.input_deadline_ns == state.input_deadline_ns);
  assert(rejected.next.input_origin_ns == state.input_origin_ns);
  assert(rejected.next.generation == state.generation);
  assert(rejected.next.closed_through_generation == state.closed_through_generation);
  assert(rejected.next.axis_mask == state.axis_mask);
  assert(rejected.next.input_axis_mask == state.input_axis_mask);
  assert(rejected.next.velocity == state.velocity);
}

static void end_orders_pending_begin() {
  auto c = context();
  auto ended = evaluate_jog_end({}, c, {c.identity, 3});
  assert(ended.accepted && ended.next.closed_through_generation == 3);
  auto delayed = evaluate_jog_begin(ended.next, c, begin(c, 3));
  assert(!delayed.accepted && !delayed.next.open && delayed.reason == JogSessionReason::InvalidGeneration);
  auto opened = evaluate_jog_begin(ended.next, c, begin(c, 4));
  assert(opened.accepted);
  auto stale_end = evaluate_jog_end(opened.next, c, {c.identity, 3});
  assert(stale_end.accepted && !stale_end.closed && stale_end.next.open);
  c.now_ns += 10;
  auto moving = evaluate_jog_update(stale_end.next, c, update(c, 4));
  assert(moving.accepted);
  ended = evaluate_jog_end(moving.next, c, {c.identity, 4});
  assert(ended.accepted && ended.closed && ended.inhibit && ended.needs_qualified_halt);
  assert(!ended.next.open && ended.next.velocity == (std::array<double, 16>{}));
  assert(evaluate_jog_update(ended.next, c, update(c, 4, 2)).reason == JogSessionReason::Closed);
  assert(!evaluate_jog_begin(ended.next, c, begin(c, 4)).accepted);
  opened = evaluate_jog_begin(ended.next, c, begin(c, 5));
  assert(opened.accepted);
  ended = evaluate_jog_end(opened.next, c, {c.identity, 6});
  assert(ended.closed && ended.next.closed_through_generation == 6);
  assert(!evaluate_jog_begin(ended.next, c, begin(c, 6)).accepted);
  assert(evaluate_jog_begin(ended.next, c, begin(c, 7)).accepted);
}

static void invalid_packets_do_not_refresh_or_consume_sequence() {
  auto c = context();
  const auto state = active(c);
  c.now_ns = 1020;
  auto valid = update(c, 1, 2);
  for (unsigned field = 0; field < 8; ++field) {
    auto packet = valid;
    switch (field) {
      case 0: ++packet.identity.connection_id; break;
      case 1: ++packet.identity.native_generation; break;
      case 2: ++packet.identity.capability_id[15]; break;
      case 3: ++packet.identity.app_grant_generation; break;
      case 4: ++packet.identity.configuration_epoch; break;
      case 5: ++packet.identity.home_epoch; break;
      case 6: ++packet.identity.clock_mapping_generation; break;
      case 7: packet.identity.capability_id.fill(0); break;
    }
    unchanged_input(state, evaluate_jog_update(state, c, packet), JogSessionReason::WrongIdentity);
    const auto wrong_end = evaluate_jog_end(state, c, {packet.identity, 999});
    unchanged_input(state, wrong_end, JogSessionReason::WrongIdentity);
  }
  for (uint32_t mask : {0u, 4u, 1u << 16, UINT32_MAX}) {
    auto packet = valid;
    packet.axis_mask = mask;
    unchanged_input(state, evaluate_jog_update(state, c, packet), JogSessionReason::InvalidVector);
  }
  for (double invalid : {std::numeric_limits<double>::infinity(),
                         -std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    auto packet = valid;
    packet.source_sequence = 900;
    packet.velocity[15] = invalid; // Outside the selected group still rejects all input.
    unchanged_input(state, evaluate_jog_update(state, c, packet), JogSessionReason::InvalidVector);
  }
  for (uint64_t sequence : {uint64_t(0), uint64_t(1)}) {
    auto packet = valid;
    packet.source_sequence = sequence;
    unchanged_input(state, evaluate_jog_update(state, c, packet), JogSessionReason::StaleSequence);
  }
  for (uint64_t generation : {uint64_t(0), uint64_t(2)}) {
    auto packet = valid;
    packet.jog_generation = generation;
    unchanged_input(state, evaluate_jog_update(state, c, packet), JogSessionReason::InvalidGeneration);
  }
  const auto accepted = evaluate_jog_update(state, c, valid);
  assert(accepted.accepted && accepted.next.source_sequence == 2);
  valid.axis_mask = 1;
  const auto subset = evaluate_jog_update(state, c, valid);
  assert(subset.accepted && subset.next.axis_mask == 3 && subset.next.input_axis_mask == 1);
  assert(subset.next.velocity[0] == 0.5 && subset.next.velocity[1] == 0);
}

static void fixed_deadlines_survive_delay_and_renewal() {
  auto c = context();
  const auto opened = evaluate_jog_begin({}, c, begin(c));
  c.now_ns = 1050;
  const auto too_late = evaluate_jog_update(opened.next, c, update(c));
  assert(!too_late.accepted && too_late.closed && too_late.inhibit && !too_late.needs_qualified_halt);
  assert(too_late.reason == JogSessionReason::Expired);
  assert(!evaluate_jog_update(too_late.next, c, update(c)).accepted);

  c = context();
  const auto state = active(c); // Accepted input expires at 1090.
  c.now_ns = 1030;
  for (unsigned invalid = 0; invalid < 6; ++invalid) {
    auto packet = update(c, 1, 10);
    switch (invalid) {
      case 0: packet.earliest_origin_host_ns = 1031; break; // Future lower bound.
      case 1: packet.immutable_deadline_host_ns = 1030; break; // Exact expiry.
      case 2: packet.earliest_origin_host_ns = 1009; break; // Reordered origin.
      case 3: packet.immutable_deadline_host_ns = 1131; break; // Beyond age budget.
      case 4: packet.earliest_origin_host_ns = UINT64_MAX; packet.immutable_deadline_host_ns = 10; break;
      case 5: packet.earliest_origin_host_ns = 1010; packet.immutable_deadline_host_ns = 1100; break; // Renew old input.
    }
    unchanged_input(state, evaluate_jog_update(state, c, packet), JogSessionReason::InvalidTiming);
  }
  auto shortened = update(c, 1, 2);
  shortened.earliest_origin_host_ns = 1010;
  shortened.immutable_deadline_host_ns = 1080;
  const auto accepted = evaluate_jog_update(state, c, shortened);
  assert(accepted.accepted && accepted.next.input_deadline_ns == 1080);
  c.now_ns = 1089;
  c.max_input_age_ns = 1000; // A renewed grant/looser future budget cannot extend old input.
  const auto renewed = evaluate_jog_session(state, c);
  assert(renewed.next.open && renewed.next.input_deadline_ns == 1090);
  c.now_ns = 1090;
  auto expired = evaluate_jog_session(renewed.next, c);
  assert(expired.closed && expired.inhibit && expired.needs_qualified_halt && !expired.next.open);
  assert(!evaluate_jog_update(expired.next, c, update(c, 1, 2)).accepted);
  assert(evaluate_jog_begin(expired.next, c, begin(c, 2)).accepted);
}

static void readiness_identity_and_clock_loss_close_input() {
  for (unsigned failure = 0; failure < 8; ++failure) {
    auto c = context();
    const auto state = active(c);
    c.now_ns = 1020;
    switch (failure) {
      case 0: c.ready_mask = 1; break;
      case 1: c.allowed_mask = 1; break;
      case 2: c.armed = false; break;
      case 3: c.conflicting_motion = true; break;
      case 4: c.commissioning_active = true; break;
      case 5: c.authority_valid = false; break;
      case 6: ++c.identity.home_epoch; break;
      case 7: c.clock_mapping_valid = false; break;
    }
    const auto stopped = evaluate_jog_session(state, c);
    assert(stopped.closed && stopped.inhibit && !stopped.next.open && !stopped.needs_qualified_halt);
    assert(stopped.next.velocity == (std::array<double, 16>{}));
  }
  auto c = context();
  const auto state = active(c);
  ++c.identity.clock_mapping_generation;
  c.now_ns = 1020;
  const auto remapped = evaluate_jog_session(state, c);
  assert(remapped.closed && remapped.reason == JogSessionReason::ClockReset && !remapped.next.clock_failed);
  const auto reopened = evaluate_jog_begin(remapped.next, c, begin(c, 2));
  assert(reopened.accepted);
  auto stale = update(c, 2);
  --stale.identity.clock_mapping_generation;
  unchanged_input(reopened.next, evaluate_jog_update(reopened.next, c, stale), JogSessionReason::WrongIdentity);
  c.now_ns = 1019;
  const auto clock_reset = evaluate_jog_session(reopened.next, c);
  assert(clock_reset.closed && clock_reset.inhibit && clock_reset.next.clock_failed);
  c.now_ns = 1030;
  assert(!evaluate_jog_begin(clock_reset.next, c, begin(c, 3)).accepted);
}

static void grant_deadline_independently_fences_input() {
  auto c = context();
  const auto request = begin(c);
  c.grant_deadline_ns = 1049;
  auto rejected = evaluate_jog_begin({}, c, request);
  assert(!rejected.accepted && !rejected.next.open);
  assert(rejected.reason == JogSessionReason::InvalidTiming);
  assert(rejected.next.highest_generation == 0);
  auto capped_begin = request;
  capped_begin.first_input_deadline_host_ns = c.grant_deadline_ns;
  auto opened = evaluate_jog_begin({}, c, capped_begin);
  assert(opened.accepted && opened.next.input_deadline_ns == 1049);
  c.grant_deadline_ns = 2000;
  c.now_ns = 1048;
  const auto renewed_begin = evaluate_jog_session(opened.next, c);
  assert(renewed_begin.next.open && renewed_begin.next.input_deadline_ns == 1049);
  c.now_ns = 1049;
  assert(evaluate_jog_session(renewed_begin.next, c).reason == JogSessionReason::Expired);

  c = context();
  c.grant_deadline_ns = 1060;
  opened = evaluate_jog_begin({}, c, begin(c));
  assert(opened.accepted);
  c.now_ns = 1010;
  auto packet = update(c);
  unchanged_input(opened.next, evaluate_jog_update(opened.next, c, packet),
                  JogSessionReason::InvalidTiming); // No uncapped first input.
  packet.immutable_deadline_host_ns = 1060;
  auto admitted = evaluate_jog_update(opened.next, c, packet);
  assert(admitted.accepted && admitted.next.input_deadline_ns == 1060);
  c.now_ns = 1020;
  auto newer = update(c, 1, 2);
  unchanged_input(admitted.next, evaluate_jog_update(admitted.next, c, newer),
                  JogSessionReason::InvalidTiming); // Nor an uncapped later input.
  c.grant_deadline_ns = 2000;
  const auto renewed = evaluate_jog_session(admitted.next, c);
  assert(renewed.next.open && renewed.next.input_deadline_ns == 1060);
  c.now_ns = 1060;
  const auto expired_input = evaluate_jog_session(renewed.next, c);
  assert(expired_input.closed && expired_input.inhibit && expired_input.needs_qualified_halt);
  assert(expired_input.reason == JogSessionReason::Expired);

  // The mirrored absolute deadline is independently enforced even if an older
  // authority_valid observation still says true. Renewal cannot reopen it.
  c = context();
  const auto state = active(c);
  c.grant_deadline_ns = c.now_ns;
  const auto expired_grant = evaluate_jog_session(state, c);
  assert(c.authority_valid && expired_grant.closed && expired_grant.inhibit);
  assert(expired_grant.reason == JogSessionReason::AuthorityLost);
  assert(!expired_grant.needs_qualified_halt && !expired_grant.next.open);
  c.grant_deadline_ns = 2000;
  assert(!evaluate_jog_update(expired_grant.next, c, update(c, 1, 2)).accepted);
  assert(evaluate_jog_begin(expired_grant.next, c, begin(c, 2)).reason == JogSessionReason::AuthorityLost);
  ++c.identity.native_generation;
  assert(evaluate_jog_begin(expired_grant.next, c, begin(c, 2)).accepted);

  for (uint64_t deadline : {uint64_t(0), uint64_t(999), uint64_t(1000)}) {
    c = context();
    c.grant_deadline_ns = deadline;
    const auto denied = evaluate_jog_begin({}, c, begin(c));
    assert(!denied.accepted && denied.reason == JogSessionReason::AuthorityLost);
  }
}

static void identity_epochs_must_be_positive() {
  for (bool missing_home : {false, true}) {
    auto c = context();
    const auto state = active(c);
    if (missing_home) c.identity.home_epoch = 0;
    else c.identity.configuration_epoch = 0;
    assert(!c.identity.valid());
    const auto denied = evaluate_jog_begin({}, c, begin(c));
    assert(!denied.accepted && denied.reason == JogSessionReason::AuthorityLost);
    const auto closed = evaluate_jog_session(state, c);
    assert(closed.closed && closed.inhibit && closed.reason == JogSessionReason::AuthorityLost);
  }
}

static void stop_fences_active_and_pending_input() {
  auto c = context();
  const auto state = active(c);
  const auto stopped = revoke_jog_session(state, c.identity);
  assert(stopped.closed && stopped.inhibit && !stopped.needs_qualified_halt);
  assert(!evaluate_jog_update(stopped.next, c, update(c, 1, 2)).accepted);
  assert(evaluate_jog_begin(stopped.next, c, begin(c, 2)).reason == JogSessionReason::AuthorityLost);
  ++c.identity.native_generation;
  assert(evaluate_jog_begin(stopped.next, c, begin(c, 2)).accepted);

  c = context();
  const auto pending_stop = revoke_jog_session({}, c.identity);
  assert(!evaluate_jog_begin(pending_stop.next, c, begin(c)).accepted);
  ++c.identity.native_generation;
  assert(evaluate_jog_begin(pending_stop.next, c, begin(c)).accepted);

  c = context();
  const auto before_loss = active(c);
  c.authority_valid = false;
  const auto revoked = evaluate_jog_session(before_loss, c);
  c.authority_valid = true; // Renewal alone cannot undo authority revocation.
  assert(!evaluate_jog_begin(revoked.next, c, begin(c, 2)).accepted);
}

static void sequence_and_generation_wrap_fail_closed() {
  auto c = context();
  auto state = active(c);
  auto final_sequence = update(c, 1, UINT64_MAX);
  final_sequence.immutable_deadline_host_ns = state.input_deadline_ns;
  const auto last = evaluate_jog_update(state, c, final_sequence);
  assert(last.accepted);
  unchanged_input(last.next, evaluate_jog_update(last.next, c, update(c, 1, 0)), JogSessionReason::StaleSequence);
  unchanged_input(last.next, evaluate_jog_update(last.next, c, update(c, 1, 1)), JogSessionReason::StaleSequence);
  auto ended = evaluate_jog_end(last.next, c, {c.identity, UINT64_MAX});
  assert(ended.closed && ended.next.closed_through_generation == UINT64_MAX);
  assert(evaluate_jog_begin(ended.next, c, begin(c, 0)).reason == JogSessionReason::GenerationExhausted);
  assert(!evaluate_jog_begin(ended.next, c, begin(c, 1)).accepted);

  c = context();
  const auto final_generation = evaluate_jog_begin({}, c, begin(c, UINT64_MAX));
  assert(final_generation.accepted);
  ended = evaluate_jog_end(final_generation.next, c, {c.identity, UINT64_MAX});
  assert(!evaluate_jog_begin(ended.next, c, begin(c, 1)).accepted);

  // Valid near-maximum times are handled without addition in the policy;
  // wrapping a deadline or evaluated clock is rejected rather than renewed.
  c = context();
  c.now_ns = UINT64_MAX - 20;
  c.grant_deadline_ns = UINT64_MAX;
  auto request = begin(c);
  request.first_input_deadline_host_ns = UINT64_MAX - 1;
  const auto near_max = evaluate_jog_begin({}, c, request);
  assert(near_max.accepted);
  auto packet = update(c);
  packet.immutable_deadline_host_ns = UINT64_MAX;
  assert(evaluate_jog_update(near_max.next, c, packet).accepted);
  packet.immutable_deadline_host_ns = 60;
  unchanged_input(near_max.next, evaluate_jog_update(near_max.next, c, packet), JogSessionReason::InvalidTiming);
}

static void begin_requires_an_explicit_ready_session() {
  auto c = context();
  assert(evaluate_jog_update({}, c, update(c)).reason == JogSessionReason::Closed);
  for (unsigned failure = 0; failure < 8; ++failure) {
    auto current = c;
    auto request = begin(current);
    switch (failure) {
      case 0: current.conflicting_motion = true; break;
      case 1: current.commissioning_active = true; break;
      case 2: current.ready_mask = 1; break;
      case 3: current.armed = false; break;
      case 4: ++request.identity.app_grant_generation; break;
      case 5: request.axis_mask = 4; break;
      case 6: request.first_input_deadline_host_ns = current.now_ns; break;
      case 7: request.earliest_origin_host_ns = current.now_ns + 1; break;
    }
    const auto rejected = evaluate_jog_begin({}, current, request);
    assert(!rejected.accepted && !rejected.next.open && !rejected.next.has_input);
    assert(rejected.next.highest_generation == 0);
  }
}

// Inject receiver scheduling at the mapped capture ages used by the WSS test.
// Accepted velocity and fixed expiry prove transport delay is not a fresh input.
static void internet_capture_age() {
  for (uint64_t delay : {UINT64_C(120000000), UINT64_C(400000000), UINT64_C(240000000), UINT64_C(630000000)}) {
    auto c = context();
    c.now_ns = 1800000000;
    c.grant_deadline_ns = 6000000000;
    c.max_input_age_ns = c.max_first_input_wait_ns = 750000000;
    auto b = begin(c);
    b.first_input_deadline_host_ns = 2550000000;
    auto opened = evaluate_jog_begin({}, c, b);
    assert(opened.accepted);
    // Keep the session alive with fresh input; refusal below concerns the
    // delayed frame, not expiration of the first-input wait.
    c.now_ns = 1810000000;
    auto fresh = update(c);
    fresh.immutable_deadline_host_ns = 2560000000;
    auto state = evaluate_jog_update(opened.next, c, fresh);
    assert(state.accepted);
    c.now_ns = 2000000000 + delay;
    auto frame = update(c, 1, 2);
    frame.earliest_origin_host_ns = 1879999999;
    frame.immutable_deadline_host_ns = 2629999999;
    auto result = evaluate_jog_update(state.next, c, frame);
    if (delay == 630000000) {
      assert(!result.accepted);
      assert(result.reason == JogSessionReason::Expired || result.reason == JogSessionReason::InvalidTiming);
    } else {
      assert(result.accepted && result.next.source_sequence == 2);
      assert(result.next.velocity[0] == 0.5 && result.next.velocity[1] == -0.25);
      assert(result.next.input_origin_ns == 1879999999);
      assert(result.next.input_deadline_ns == 2629999999);
    }
  }
}

// Absolute source and wait bounds remain independent, preventing calibration
// uncertainty from spending first-input allowance or extending captured input.
// Unverified synthetic clock stimulus: now 1000 ns, cutoff 950 ns, wait 1050 ns;
// the captured input retains origin 975 ns and deadline 1055 ns.
static void calibrated_origin_does_not_shorten_first_input_wait() {
  auto c = context();
  auto request = begin(c);
  request.earliest_origin_host_ns = 950;
  request.first_input_deadline_host_ns = 1050;
  const auto opened = evaluate_jog_begin({}, c, request);
  assert(opened.accepted && opened.next.begin_origin_ns == 950);
  assert(opened.next.input_deadline_ns == 1050 && !opened.next.has_input);
  auto packet = update(c);
  packet.earliest_origin_host_ns = 975;
  packet.immutable_deadline_host_ns = 1055;
  const auto accepted = evaluate_jog_update(opened.next, c, packet);
  assert(accepted.accepted && accepted.next.input_origin_ns == 975);
  assert(accepted.next.input_deadline_ns == 1055);
  packet.earliest_origin_host_ns = 949;
  packet.immutable_deadline_host_ns = 1049;
  assert(evaluate_jog_update(opened.next, c, packet).reason == JogSessionReason::InvalidTiming);
  request.first_input_deadline_host_ns = 1051;
  assert(evaluate_jog_begin({}, c, request).reason == JogSessionReason::InvalidTiming);
  request.first_input_deadline_host_ns = 1050;
  request.earliest_origin_host_ns = 899;
  assert(evaluate_jog_begin({}, c, request).reason == JogSessionReason::InvalidTiming);
  c.now_ns = 1050;
  c.grant_deadline_ns = 3000;
  const auto expired = evaluate_jog_session(opened.next, c);
  assert(expired.reason == JogSessionReason::Expired && !expired.next.open);
}

int main() {
  calibrated_origin_does_not_shorten_first_input_wait();
  internet_capture_age();
  static_assert(std::is_trivially_copyable_v<JogSessionState>);
  static_assert(noexcept(evaluate_jog_update(JogSessionState{}, JogSessionContext{}, JogUpdateRequest{})));
  begin_requires_an_explicit_ready_session();
  end_orders_pending_begin();
  invalid_packets_do_not_refresh_or_consume_sequence();
  fixed_deadlines_survive_delay_and_renewal();
  readiness_identity_and_clock_loss_close_input();
  grant_deadline_independently_fences_input();
  identity_epochs_must_be_positive();
  stop_fences_active_and_pending_input();
  sequence_and_generation_wrap_fail_closed();
}
