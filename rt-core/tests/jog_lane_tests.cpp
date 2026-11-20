#include "engine/jog_lane.hpp"
#include <cassert>
#include <iostream>
#include <limits>

using namespace rosie::rtcore;
namespace wire = rosie::protocol;

struct Fixture {
  alignas(8) std::array<uint8_t, fast_control_region_bytes> bytes{};
  FastControlRegion region;
  FastControlRegion::JogControlSlot controls;
  FastControlRegion::JogUpdateSlot updates;
  FastControlRegion::JogObservationSlot observations;
  JogLane lane;
  JogSessionContext context;
  std::array<double, jog_session_axes> limits{};
  wire::JogControl begin{};
  wire::JogUpdate update{};

  Fixture() {
    wire::FastHeader header{};
    header.magic = fast_control_magic;
    header.version_major = wire::version_major;
    header.version_minor = wire::version_minor;
    header.region_bytes = fast_control_region_bytes;
    header.daemon_incarnation[0] = 1;
    header.connection_id = 1;
    header.configuration_digest_lo[0] = 1;
    header.cycle_ns = 1'000'000;
    header.configured_mask = 3;
    assert(FastControlRegion::initialize(bytes.data(), bytes.size(), header, region) == FastControlStatus::Ok);
    assert(FastControlRegion::JogControlSlot::bind(bytes.data() + fast_control_jog_control_offset,
        FastControlRegion::JogControlSlot::region_bytes, controls) == SharedWordsStatus::Ok);
    assert(FastControlRegion::JogUpdateSlot::bind(bytes.data() + fast_control_jog_update_offset,
        FastControlRegion::JogUpdateSlot::region_bytes, updates) == SharedWordsStatus::Ok);
    assert(FastControlRegion::JogObservationSlot::bind(bytes.data() + fast_control_jog_observation_offset,
        FastControlRegion::JogObservationSlot::region_bytes, observations) == SharedWordsStatus::Ok);
    context.identity = {1, 1, {1}, 1, 1, 1, 1};
    context.now_ns = 1'000'000'000;
    context.grant_deadline_ns = 1'250'000'000;
    context.max_input_age_ns = context.max_first_input_wait_ns = 250'000'000;
    context.configured_mask = context.allowed_mask = context.ready_mask = 3;
    context.authority_valid = context.clock_mapping_valid = context.armed = true;
    limits.fill(1);
    begin.ticket = begin.connection_id = begin.native_generation = 1;
    begin.capability_id[0] = 1;
    begin.app_grant_generation = begin.configuration_epoch = begin.home_epoch = begin.clock_mapping_generation = 1;
    begin.origin_host_ns = context.now_ns;
    begin.deadline_host_ns = 1'200'000'000;
    begin.axis_mask = 3;
    begin.operation = wire::jog_operation_begin;
    update.conn_id = update.native_generation = update.app_grant_generation = 1;
    update.capability_id[0] = 1;
    update.config_epoch = update.home_epoch = update.clock_mapping_generation = 1;
    update.jog_generation = update.source_sequence = 1;
    update.axis_mask = 3;
    update.earliest_origin_host_ns = context.now_ns;
    update.immutable_deadline_host_ns = 1'100'000'000;
    update.velocity[0] = .1;
    update.velocity[1] = -.1;
  }
  void control() {
    assert(controls.publish(shared_words_from_le<FastControlRegion::JogControlSlot::payload_words>(
        wire::encode_jog_control(begin))).status == SharedWordsStatus::Ok);
  }
  void input() {
    assert(updates.publish(shared_words_from_le<FastControlRegion::JogUpdateSlot::payload_words>(
        wire::encode_jog_update(update))).status == SharedWordsStatus::Ok);
  }
  wire::JogObservation cycle(bool inhibit = false) {
    assert(lane.cycle(&region, context, limits) == inhibit);
    assert(lane.publish(region, context.identity, context.now_ns).status == SharedWordsStatus::Ok);
    FastControlRegion::JogObservationSlot::Words words{};
    assert(observations.read(words).status == SharedWordsStatus::Ok);
    const auto encoded = shared_words_to_le(words);
    wire::JogObservation value{};
    assert(wire::decode_jog_observation(encoded.data(), encoded.size(), value));
    return value;
  }
  void open() { control(); assert(cycle().control_accepted == 1); }
};

int main() {
  // Current command feedback and signed Home frames must keep continuous
  // exterior updates inward without refreshing a refused input's deadline.
  for (int sign : {-1, 1}) {
    Fixture f;
    const AxisLimits logical = sign > 0 ? AxisLimits{10100, 50100, 1, false, true}
        : AxisLimits{-50100, -10100, 1, false, true};
    f.lane.set_limit_feedback(0, logical, 620000, sign, 100);
    f.open();
    f.update.velocity[0] = -sign;
    f.input();
    auto o = f.cycle();
    assert(o.update_reason == wire::jog_reason_none && o.source_sequence == 1);
    f.update.velocity[0] = sign;
    f.update.source_sequence = 2;
    f.input();
    o = f.cycle();
    assert(o.update_reason == wire::jog_reason_outside_limits_outward && o.source_sequence == 1);
    assert(o.input_deadline_host_ns == 1'100'000'000);
  }
  {
    Fixture f;
    f.lane.set_limit_feedback(0, AxisLimits{.01, .1, 1, false, true}, 0);
    f.open();
    f.update.velocity[0] = -.1;
    f.input();
    auto o = f.cycle();
    assert(o.update_reason == wire::jog_reason_outside_limits_outward && o.open == 1 &&
        o.source_sequence == 0 && o.input_deadline_host_ns == 1'200'000'000);
    f.update.velocity[0] = .1;
    f.input();
    o = f.cycle();
    assert(o.update_reason == wire::jog_reason_none && o.source_sequence == 1);
    f.update.source_sequence = 2;
    f.update.velocity[0] = -.1;
    ++f.context.now_ns;
    ++f.update.earliest_origin_host_ns;
    f.update.immutable_deadline_host_ns = 1'150'000'000;
    f.input();
    o = f.cycle();
    assert(o.update_reason == wire::jog_reason_outside_limits_outward &&
        o.source_sequence == 1 && o.input_deadline_host_ns == 1'100'000'000);
    JogAxis axis;
    axis.reset(0);
    // A lead beyond max_lead is admitted and reported as a warning (2026-10-06).
    assert(f.lane.advance(0, axis, 1000, 0, 1000, 4, .1, 0));
    assert(std::abs(axis.position - 5) < 1e-12 && axis.velocity == 100);
    assert(axis.lead_exceeded && std::abs(axis.lead - 5) < 1e-12);
  }
  {
    Fixture f;
    f.open();
    f.input();
    auto o = f.cycle();
    assert(o.source_sequence == 1 && o.input_deadline_host_ns == 1'100'000'000);
    assert(o.first_observed_host_ns == 1'000'000'000 && o.observed_origin_host_ns == 1'000'000'000);
    f.context.now_ns += 1'000'000;
    o = f.cycle();
    assert(o.first_observed_host_ns == 1'000'000'000);
    // An odd seqlock cannot spin or refresh the accepted deadline.
    auto* sequence = reinterpret_cast<uint64_t*>(f.bytes.data() + fast_control_jog_update_offset);
    __atomic_store_n(sequence, uint64_t{3}, __ATOMIC_RELEASE);
    assert(f.cycle().input_deadline_host_ns == 1'100'000'000);
    f.context.now_ns = 1'100'000'000;
    o = f.cycle(true);
    assert(o.open == 0 && o.has_input == 0 && o.state_reason == wire::jog_reason_expired);
  }
  {
    Fixture f;
    f.open();
    f.update.immutable_deadline_host_ns = f.context.now_ns;
    f.input();
    auto o = f.cycle();
    assert(o.update_reason == wire::jog_reason_expired && o.has_input == 0 && o.source_sequence == 0);
  }
  {
    Fixture f;
    f.open();
    f.input();
    f.update.source_sequence = 5;
    f.update.velocity[0] = .2;
    f.input();
    assert(f.cycle().source_sequence == 5 && f.lane.state().velocity[0] == .2);
    f.update.source_sequence = 4;
    f.input();
    assert(f.cycle().update_reason == wire::jog_reason_stale_sequence);
    assert(f.lane.state().source_sequence == 5);
    f.update.source_sequence = 6;
    f.update.velocity[0] = std::numeric_limits<double>::quiet_NaN();
    f.input();
    assert(f.cycle().update_reason == wire::jog_reason_invalid_vector);
    f.update.velocity[0] = 2;
    f.input();
    assert(f.cycle().update_reason == wire::jog_reason_limited);
    assert(f.lane.state().input_deadline_ns == 1'100'000'000);
    assert(f.lane.expire_before_dispatch(1'100'000'000));
    assert(!f.lane.state().open);
  }
  for (unsigned cause = 0; cause < 5; ++cause) {
    Fixture f;
    f.open(); f.input(); f.cycle();
    if (cause == 0) f.context.identity.native_generation = 2; // Stop fence.
    if (cause == 1) f.context.authority_valid = false;
    if (cause == 2) f.context.identity.connection_id = 0;
    if (cause == 3) f.context.identity.home_epoch = 2;
    if (cause == 4) f.context.commissioning_active = true;
    assert(f.cycle(true).open == 0);
    f.update.source_sequence = 2;
    f.input();
    const auto o = f.cycle();
    assert(o.open == 0 && o.has_input == 0 && o.update_reason == wire::jog_reason_closed);
  }
  {
    Fixture f;
    f.context.conflicting_motion = true;
    f.control();
    auto o = f.cycle();
    assert(o.control_accepted == 0 && o.control_reason == wire::jog_reason_mode_conflict);
    f.context.conflicting_motion = false;
    ++f.begin.ticket;
    f.open();
    f.begin.operation = wire::jog_operation_end;
    f.begin.jog_generation = 1;
    f.begin.axis_mask = 0;
    f.begin.origin_host_ns = f.begin.deadline_host_ns = 0;
    ++f.begin.ticket;
    f.control();
    o = f.cycle(true);
    assert(o.control_accepted == 1 && o.open == 0);
  }
  std::cout << "PASS: JogLane deadlines, seqlocks, modes, validation and generation fences\n";
}
