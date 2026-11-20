// Makefile appends -UNDEBUG; make test runs the assertion probe first.
#include "engine/commissioning.hpp"
#include <cassert>
#include <iostream>

using namespace rosie::rtcore;

int main() {
  struct TruthCase { int32_t bits; bool unsigned32; int sign; bool accepted; int32_t offset; };
  for (const auto& test : {
      TruthCase{-1, true, 1, false, 0}, {-1, true, -1, false, 0},
      {INT32_MIN, true, 1, false, 0}, {INT32_MIN, true, -1, true, INT32_MIN},
      {INT32_MAX, true, 1, true, INT32_MAX}, {INT32_MAX, true, -1, true, -INT32_MAX},
      {INT32_MIN, false, 1, true, INT32_MIN}, {INT32_MIN, false, -1, false, 0},
      {-1, false, -1, true, 1}, {0, true, 0, false, 0}, {0, false, 2, false, 0}}) {
    int32_t result = 123;
    assert(home_offset_from_scalar_bits(test.bits, test.unsigned32, test.sign, result) == test.accepted);
    assert(result == (test.accepted ? test.offset : 123));
  }
  {
    // A configuration write reserves axes that are not verified yet (that is how
    // a mismatched parameter gets corrected); a Home on the same axes does not.
    CommissioningContext partial{7, 3, 5, 3, 1, true, true, false, false, false};
    CommissioningRequest home{1, {11, 7, 3, 5}, CommissioningOperation::Reserve, 3};
    auto refused = evaluate_commissioning({}, partial, home);
    assert(!refused.accepted && refused.reason == CommissioningReason::NotReady);
    CommissioningRequest write = home;
    write.configuration_write = true;
    auto accepted = evaluate_commissioning({}, partial, write);
    assert(accepted.accepted && accepted.state.phase == CommissioningPhase::Active &&
           accepted.state.axis_mask == 3 && accepted.invalidate_home_mask == 3);
    // Every other permission still applies to a write: armed refuses it.
    CommissioningContext armed = partial;
    armed.armed = true;
    auto blocked = evaluate_commissioning({}, armed, write);
    assert(!blocked.accepted && blocked.reason == CommissioningReason::NotReady);
  }
  CommissioningContext context{7, 3, 5, 3, 3, true, true, false, false, false};
  CommissioningRequest reserve{1, {11, 7, 3, 5}, CommissioningOperation::Reserve, 3};
  auto result = evaluate_commissioning({}, context, reserve);
  assert(result.accepted && result.invalidate_home_mask == 3);
  auto state = result.state;
  assert(state.identity.home_epoch == 6);
  context.home_epoch = state.identity.home_epoch;
  assert(state.phase == CommissioningPhase::Active && state.service_mask == 0);
  assert(evaluate_commissioning(state, context, reserve).reason == CommissioningReason::Busy);

  CommissioningRequest service{2, state.identity, CommissioningOperation::Service, 1, 6, 0x1f};
  result = evaluate_commissioning(state, context, service);
  assert(result.accepted && result.set_service_mask == 1);
  state = result.state;
  auto complete = CommissioningRequest{3, state.identity, CommissioningOperation::CompleteHome, 1, 0, 0, 42};
  assert(!evaluate_commissioning(state, context, complete).accepted); // Override still active.

  auto cancelled_context = context;
  ++cancelled_context.generation; // Stop while the first axis's SDO is blocked.
  result = poll_commissioning(state, cancelled_context);
  assert(result.state.phase == CommissioningPhase::Draining && result.clear_service_mask == 1);
  state = result.state;
  service.axis_mask = 2;
  assert(!evaluate_commissioning(state, cancelled_context, service).accepted);
  service.identity.generation = cancelled_context.generation; // A worker must not adopt a new epoch.
  assert(!evaluate_commissioning(state, cancelled_context, service).accepted);
  assert(!evaluate_commissioning(state, cancelled_context, complete).accepted);
  auto replacement = reserve;
  replacement.identity = {12, cancelled_context.generation, context.configuration_epoch, context.home_epoch};
  assert(evaluate_commissioning(state, cancelled_context, replacement).reason == CommissioningReason::Busy);
  auto finish = CommissioningRequest{4, state.identity, CommissioningOperation::Finish};
  assert(!evaluate_commissioning(state, cancelled_context,
      {5, replacement.identity, CommissioningOperation::Finish}).accepted);
  result = evaluate_commissioning(state, cancelled_context, finish);
  assert(result.accepted && !result.job_completed && result.state.phase == CommissioningPhase::Idle);
  assert(evaluate_commissioning(result.state, cancelled_context, replacement).accepted);

  // Normal completion requires a release and preserves the declared logical offset.
  reserve.identity.home_epoch = context.home_epoch;
  state = evaluate_commissioning({}, context, reserve).state;
  context.home_epoch = state.identity.home_epoch;
  service.identity = state.identity;
  complete.identity = state.identity;
  service.axis_mask = 1;
  state = evaluate_commissioning(state, context, service).state;
  result = evaluate_commissioning(state, context,
      {6, state.identity, CommissioningOperation::Release, 1});
  assert(result.accepted && result.clear_service_mask == 1);
  result = evaluate_commissioning(result.state, context, complete);
  assert(result.accepted && result.complete_home_mask == 1 && result.state.completed_mask == 1);
  assert(!evaluate_commissioning(result.state, context, complete).accepted);
  assert(!evaluate_commissioning(result.state, context, service).accepted);
  state = result.state;
  complete.axis_mask = 2;
  result = evaluate_commissioning(state, context, complete);
  assert(result.accepted && result.state.completed_mask == 3);
  finish.identity = result.state.identity;
  const auto finished = evaluate_commissioning(result.state, context, finish);
  assert(finished.accepted && finished.job_completed);
  auto stopped_before_finish = context;
  ++stopped_before_finish.generation;
  const auto drained = evaluate_commissioning(result.state, stopped_before_finish, finish);
  assert(drained.accepted && !drained.job_completed && drained.state.phase == CommissioningPhase::Idle);

  auto exhausted = context;
  exhausted.home_epoch = std::numeric_limits<uint64_t>::max();
  auto exhausted_reserve = reserve;
  exhausted_reserve.identity.home_epoch = exhausted.home_epoch;
  assert(!evaluate_commissioning({}, exhausted, exhausted_reserve).accepted);

  // Each independent cause removes permission; returning to healthy does not reopen it.
  for (unsigned condition = 0; condition < 9; ++condition) {
    reserve.identity.home_epoch = context.home_epoch;
    auto bad = context;
    switch (condition) {
      case 0: bad.authority_valid = false; break;
      case 1: bad.bus_ready = false; break;
      case 2: bad.armed = true; break;
      case 3: bad.motion_active = true; break;
      case 4: bad.shutdown = true; break;
      case 5: ++bad.generation; break;
      case 6: ++bad.configuration_epoch; break;
      case 7: bad.verified_mask = 1; break;
      case 8: ++bad.home_epoch; break;
    }
    assert(!evaluate_commissioning({}, bad, reserve).accepted);
    state = evaluate_commissioning({}, context, reserve).state;
    if (condition != 8) bad.home_epoch = state.identity.home_epoch;
    else bad.home_epoch = state.identity.home_epoch + 1;
    result = poll_commissioning(state, bad);
    assert(result.state.phase == CommissioningPhase::Draining);
    assert(!evaluate_commissioning(result.state, context, service).accepted);
  }
  // Capture is a typed inhibited reference transaction, not an HM35 service.
  auto reference_request = reserve;
  reference_request.identity.home_epoch = context.home_epoch;
  reference_request.operation = CommissioningOperation::ReserveReference;
  reference_request.axis_mask = 1;
  auto reference = evaluate_commissioning({}, context, reference_request);
  assert(reference.accepted && reference.state.reference_acquisition && reference.invalidate_home_mask == 1);
  auto rc = context; rc.home_epoch = reference.state.identity.home_epoch;
  auto rr = CommissioningRequest{99, reference.state.identity, CommissioningOperation::CompleteReference, 1};
  auto wrong = rr; wrong.operation = CommissioningOperation::CompleteHome;
  assert(!evaluate_commissioning(reference.state, rc, wrong).accepted);
  wrong.operation = CommissioningOperation::Service; wrong.mode = 6; wrong.controlword = 31;
  assert(!evaluate_commissioning(reference.state, rc, wrong).accepted);
  const auto committed = evaluate_commissioning(reference.state, rc, rr);
  assert(committed.accepted && committed.complete_reference_mask == 1 && !committed.complete_home_mask);
  rc.authority_valid = false;
  assert(!evaluate_commissioning(reference.state, rc, rr).accepted);
  rr.operation = CommissioningOperation::FailReference;
  const auto refused = evaluate_commissioning(reference.state, rc, rr);
  assert(refused.accepted && refused.fail_reference_mask == 1 && !refused.fail_home_mask);
  std::cout << "Commissioning identity and draining policy: passed\n";
}
