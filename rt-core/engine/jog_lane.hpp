#pragma once
#include "jog_session.hpp"
#include "fast_control_region.hpp"
#include "protocol_generated.hpp"
#include "jog.hpp"
#include "execution_policy.hpp"
#include "path_jog.hpp"

namespace rosie::rtcore {

// Path jog results map onto the generated jog reasons (protocol 1.14) so the
// adapter and SDK see one reason family for every jog refusal.
inline JogSessionReason path_jog_reason(PathJogResult result) noexcept {
  switch (result) {
    case PathJogResult::Ok: return JogSessionReason::None;
    case PathJogResult::Closed: return JogSessionReason::Closed;
    case PathJogResult::InvalidConfig: return JogSessionReason::PathInvalidConfig;
    case PathJogResult::InvalidSample: return JogSessionReason::PathInvalidSample;
    case PathJogResult::StartMismatch: return JogSessionReason::PathStartMismatch;
    case PathJogResult::Capacity: return JogSessionReason::PathCapacity;
    case PathJogResult::OutOfOrder: return JogSessionReason::PathOutOfOrder;
    case PathJogResult::StaleSequence: return JogSessionReason::StaleSequence;
    case PathJogResult::InvalidTiming: return JogSessionReason::InvalidTiming;
    case PathJogResult::PathEnded: return JogSessionReason::PathEnded;
  }
  return JogSessionReason::PathFault;
}

// One cyclic owner. Local jog frames use the daemon's CLOCK_MONOTONIC domain;
// mapping 1 is negotiated by the versioned local ABI. Remote clock mappings
// require their own qualified adapter and are not accepted by this lane yet.
class JogLane {
 public:
  const JogSessionState& state() const noexcept { return state_; }
  bool ramping() const noexcept { return ramping_; }
  // Publish input permission and its original deadline; a retained stopping
  // reservation and its sentinel deadline must never appear as fresh input.
  bool input_open() const noexcept { return state_.open && !ramping_; }
  uint64_t input_deadline_ns() const noexcept {
    if (ramping_) return ramp_input_deadline_ns_;
    // A begun path session's input expiry is its speed command's deadline,
    // owned by PathJog; the session deadline is parked at UINT64_MAX so the
    // per-axis expiry ramp can never take over a path.
    if (path_begun() && path_->open()) return path_->observe().speed_deadline_ns;
    return state_.input_deadline_ns;
  }

  // Path jog (protocol 1.14). The owner binds one PathJog (its 128 KiB ring
  // lives in the machine object, never on a stack) and, every cycle before
  // cycle(), refreshes the compiled per-axis limits and the held command
  // targets, both in logical joint units. A path session is a jog session in
  // path mode: same identity, generation, authority and first-input admission,
  // and it excludes joint jog, trajectories and Halt the same way. In path
  // mode this lane never runs its per-axis expiry or HaltRamp: PathJog stops
  // along the path on expiry, End and Halt, and the session closes when that
  // stop completes. Machine jog_jerk == 0 leaves path jog unavailable.
  void bind_path(PathJog& path) noexcept { path_ = &path; }
  void set_path_timing(double jerk, double max_sample_step_s, uint64_t cycle_ns,
                       uint64_t max_cycle_ns = 0) noexcept {
    path_config_.jerk = jerk;
    path_config_.max_sample_step_s = max_sample_step_s;
    path_config_.cycle_ns = cycle_ns;
    path_config_.max_cycle_ns = max_cycle_ns;
  }
  void set_path_axis(uint32_t axis, const PathJogAxisConfig& limits, double held) noexcept {
    path_config_.axes[axis] = limits;
    path_held_[axis] = held;
  }
  bool path_available() const noexcept { return path_ && path_config_.jerk > 0; }
  bool path_mode() const noexcept { return state_.open && path_mode_; }
  // The first chunk began PathJog for this session; the owner drives the
  // masked axes from it from then on.
  bool path_begun() const noexcept { return path_mode() && path_begun_; }
  // Public Halt on a path session: a path-preserving stop within the
  // configured limits instead of per-axis ramps. Before any geometry nothing
  // has moved, so the session closes exactly as close_for_halt does.
  void halt_path(uint64_t now) noexcept {
    if (!path_mode()) return;
    if (path_begun_ && path_->open()) {
      path_->halt(now);
      enter_path_stop(JogSessionReason::Closed, now);
    } else {
      close_for_halt();
    }
  }
  // Call once per cycle after PathJog::advance. A stop PathJog started itself
  // (speed deadline) fences input like End; a completed stop closes the
  // session without inhibiting; a generator fault closes it and returns true
  // so the owner fails closed.
  bool path_cycle(const PathJogCycle& cycle, uint64_t now) noexcept {
    if (!path_begun()) return false;
    if (!ramping_ && path_->stopping())
      enter_path_stop(path_->observe().stop_reason == static_cast<uint32_t>(PathStopReason::Expired)
          ? JogSessionReason::Expired : JogSessionReason::Closed, now);
    if (!cycle.closed) return false;
    JogSessionDecision closed{state_};
    close_jog_session(closed, cycle.fault ? JogSessionReason::PathFault : ramp_reason_, false);
    closed.inhibit = cycle.fault;
    ramping_ = false;
    return apply(closed, now);
  }
  // The cyclic owner supplies current command feedback for direction admission;
  // no stale application observation can authorize an outward exterior update.
  void set_limit_feedback(uint32_t axis, const AxisLimits& limits, double position,
                          double scale = 1, double offset = 0) noexcept {
    limits_[axis] = command_limits(limits, scale, offset);
    feedback_[axis] = position;
    scales_[axis] = scale;
    acceleration_[axis] = std::abs(scale * limits.jog_acceleration);
    limit_mask_ |= 1u << axis;
  }
  // Bind the RT integrator once; cancellation must retain the emitted target.
  void configure_axis(uint32_t axis, JogAxis& output, const JogTiming& timing,
                      double acceleration, uint64_t cycle_ns) noexcept {
    axes_[axis] = &output;
    timing_[axis] = timing;
    acceleration_[axis] = acceleration;
    cycle_ns_ = cycle_ns;
  }
  // The lane owns cancellation; the integrator retains the commanded position.
  // The admission step covers one configured cycle; later steps use elapsed
  // monotonic time, capped at the remaining time to zero velocity. Motion still
  // present after abs(initial velocity)/acceleration, rounded up to ns, plus
  // one configured cycle fails closed.
  bool advance(uint32_t axis, JogAxis& output, double requested, double feedback,
               double acceleration, double max_lead, double dt,
               double lag_s = 0) noexcept {
    if (ramping_ && (state_.axis_mask & (1u << axis))) {
      return ramps_[axis].advance(output, feedback, max_lead, ramp_now_ns_, lag_s);
    }
    return output.advance(requested, feedback, acceleration, max_lead, dt, lag_s);
  }
  // Apply one scale to the entire jog request before any axis advances. This
  // preserves the requested vector; planned trajectories are never retimed.
  void govern(uint32_t mask, const std::array<JogAxis, jog_session_axes>& axes,
      std::array<double, jog_session_axes>& requested, double dt) noexcept {
    position_scale_ = 1;
    if (ramping_) return; // Every admitted jog step already reserves its halt.
    for (uint32_t axis = 0; axis < jog_session_axes; ++axis) {
      if (!(mask & limit_mask_ & (1u << axis))) continue;
      position_scale_ = std::min(position_scale_, jog_position_scale(limits_[axis],
          axes[axis], requested[axis], acceleration_[axis], dt));
    }
    for (uint32_t axis = 0; axis < jog_session_axes; ++axis)
      if (mask & (1u << axis)) requested[axis] *= position_scale_;
  }
  // Home cancels the reservation before commissioning checks motion ownership;
  // the retained generation floor prevents a queued update reclaiming the axes.
  bool close_for_home() noexcept {
    if (path_mode() && path_ && path_->open()) path_->abandon();
    JogSessionDecision closed{state_};
    close_jog_session(closed, JogSessionReason::Closed, false);
    ramping_ = false;
    state_ = closed.next;
    state_reason_ = static_cast<uint32_t>(closed.reason);
    return closed.inhibit;
  }
  // Public Halt takes over the integrators and fences the old jog generation.
  // A queued update cannot re-open it or replace the controlled hold targets.
  void close_for_halt() noexcept { (void)close_for_home(); }
  // Recheck immediately before target dispatch. Work earlier in the same cycle
  // cannot turn an immutable deadline into permission to emit an expired target.
  bool expire_before_dispatch(uint64_t now) noexcept {
    ramp_now_ns_ = now;
    if (!state_.open || now < state_.input_deadline_ns) return false;
    JogSessionDecision expired{state_};
    close_jog_session(expired, JogSessionReason::Expired, true);
    return apply(expired, now);
  }
  bool expire_before_dispatch(uint64_t now,
      std::array<double, jog_session_axes>& requested, uint64_t& deadline) noexcept {
    const bool inhibit = expire_before_dispatch(now);
    if (ramping_) {
      requested.fill(0);
      deadline = state_.input_deadline_ns;
    }
    return inhibit;
  }
  // No queue, locks, allocation, mailbox calls or waits. A busy input publisher
  // simply leaves the last accepted input and its original expiry in force.
  bool cycle(FastControlRegion* region, JogSessionContext context,
             const std::array<double, jog_session_axes>& velocity_limits) noexcept {
    ramp_now_ns_ = context.now_ns;
    bool inhibit = false;
    const auto accept = [&](const JogSessionDecision& decision) {
      inhibit |= apply(decision, context.now_ns);
    };
    if (context.identity.connection_id != connection_) {
      connection_ = context.identity.connection_id;
      control_seen_ = update_seen_ = ticket_ = 0;
      control_accepted_ = control_reason_ = update_reason_ = 0;
      observed_sequence_ = observed_origin_ = first_observed_ = 0;
      path_publication_seen_ = path_chunk_seen_ = path_chunk_ack_ = 0;
      path_chunk_result_ = 0;
    }
    // An idle connection before Acquire has no jog authority to revoke. Keep
    // the revoked floor of the session that actually lost authority instead
    // of poisoning a later Acquire's new native generation on idle polls.
    if (state_.open || context.authority_valid) accept(evaluate_jog_session(state_, context));
    if (ramping_ && !path_mode_) inhibit |= poll_ramp(context.now_ns);
    if (inhibit) context.armed = false;
    if (!region) return inhibit;

    protocol::JogControl control{};
    const auto command = region->read_jog_control(control);
    if (command.status == SharedWordsStatus::Ok && command.sequence > control_seen_) {
      control_seen_ = command.sequence;
      if (control.ticket > ticket_) {
        ticket_ = control.ticket;
        JogSessionDecision result{state_};
        result.reason = JogSessionReason::InvalidGeneration;
        const JogIdentity identity{control.connection_id, control.native_generation,
            control.capability_id, control.app_grant_generation, control.configuration_epoch,
            control.home_epoch, control.clock_mapping_generation};
        bool path_begin = false, path_end = false;
        const bool begin = control.operation == protocol::jog_operation_begin ||
            control.operation == protocol::jog_operation_path_begin;
        if (!context.authority_valid && !state_.open) {
          result.reason = JogSessionReason::AuthorityLost;
        } else if (ramping_) {
          result.reason = JogSessionReason::Closed;
        } else if (!control.reserved && begin && !control.jog_generation) {
          path_begin = control.operation == protocol::jog_operation_path_begin;
          // Native reserves every generation; a peer cannot reset its floor.
          const auto generation = state_.highest_generation == UINT64_MAX ? 0 : state_.highest_generation + 1;
          if (path_begin && !path_available()) result.reason = JogSessionReason::PathUnavailable;
          else result = evaluate_jog_begin(state_, context, {identity, generation, control.axis_mask,
              control.origin_host_ns, control.deadline_host_ns});
        } else if (!control.reserved && control.operation == protocol::jog_operation_end &&
                   control.jog_generation && !control.axis_mask && !control.origin_host_ns && !control.deadline_host_ns) {
          result = evaluate_jog_end(state_, context, {identity, control.jog_generation});
          // End of a moving path is a stop along the path; the session (and
          // with it the axes) stays reserved until that stop completes. The
          // generation floor still advances now, fencing queued Begins.
          path_end = result.accepted && result.reason == JogSessionReason::None && state_.open &&
              !result.next.open && path_begun() && path_->open();
        }
        control_accepted_ = result.accepted;
        control_reason_ = static_cast<uint32_t>(result.reason);
        if (path_end) {
          state_.closed_through_generation = result.next.closed_through_generation;
          path_->end(context.now_ns);
          enter_path_stop(JogSessionReason::Closed, context.now_ns);
        } else {
          accept(result);
        }
        if (begin && result.accepted) {
          path_mode_ = path_begin;
          path_begun_ = false;
          path_chunk_seen_ = path_chunk_ack_ = 0;
          path_chunk_result_ = 0;
          path_spacing_ = path_max_speed_ = 0;
          position_scale_ = 1;
        }
      }
    }
    protocol::JogUpdate input{};
    const auto publication = region->read_jog_update(input);
    if (publication.status == SharedWordsStatus::Ok && publication.sequence > update_seen_) {
      update_seen_ = publication.sequence;
      observed_sequence_ = input.source_sequence;
      observed_origin_ = input.earliest_origin_host_ns;
      first_observed_ = context.now_ns;
      if (input.flags & protocol_path_speed_flag) {
        path_speed(input, context);
      } else {
      bool vector_valid = !input.flags && !input.reserved;
      double scale = 1;
      for (uint32_t axis = 0; axis < jog_session_axes; ++axis) {
        vector_valid &= std::isfinite(input.velocity[axis]);
        if (input.axis_mask & (1u << axis)) {
          vector_valid &= std::isfinite(velocity_limits[axis]) && velocity_limits[axis] > 0 &&
              std::isfinite(input.velocity[axis]);
          if (std::abs(input.velocity[axis]) > velocity_limits[axis])
            scale = std::min(scale, velocity_limits[axis] / std::abs(input.velocity[axis]));
        }
        else vector_valid &= input.velocity[axis] == 0;
      }
      JogSessionDecision result{state_};
      result.reason = JogSessionReason::InvalidVector;
      if (!state_.open || ramping_) {
        result.reason = JogSessionReason::Closed;
      } else if (path_mode_) {
        // A plain velocity vector never steers a path session.
        result.reason = JogSessionReason::ModeConflict;
      } else if (vector_valid) {
        JogUpdateRequest request{{input.conn_id, input.native_generation, input.capability_id,
            input.app_grant_generation, input.config_epoch, input.home_epoch, input.clock_mapping_generation},
            input.jog_generation, input.source_sequence, input.earliest_origin_host_ns,
            input.immutable_deadline_host_ns, input.axis_mask, input.velocity};
        for (auto& velocity : request.velocity) velocity *= scale;
        result = evaluate_jog_update(state_, context, request);
        if (result.accepted) {
          for (uint32_t axis = 0; axis < jog_session_axes; ++axis) {
            if ((input.axis_mask & limit_mask_ & (1u << axis)) &&
                !limit_recovery_velocity(limits_[axis], feedback_[axis],
                    request.velocity[axis] * scales_[axis])) {
              result = {state_};
              result.reason = JogSessionReason::OutsideLimitsOutward;
              break;
            }
          }
        }
        if (result.accepted) {
          // Exact wire unit: one million parts represent the requested vector.
          velocity_scale_ppm_ = static_cast<uint32_t>(std::floor(scale * 1'000'000));
          if (scale < 1) result.reason = JogSessionReason::Limited;
        }
        if (result.reason == JogSessionReason::InvalidTiming &&
            input.immutable_deadline_host_ns <= context.now_ns)
          result.reason = JogSessionReason::Expired;
      }
      update_reason_ = static_cast<uint32_t>(result.reason);
      accept(result);
      }
    }
    // Lossless sample handshake: consume only a newly published chunk; the
    // 2296-byte copy is skipped while the peer has nothing new.
    const uint64_t samples = region->path_samples_sequence();
    if (samples && !(samples & 1) && samples != path_publication_seen_) {
      const auto read = region->read_path_samples(path_chunk_);
      if (read.status == SharedWordsStatus::Ok && read.sequence != path_publication_seen_) {
        path_publication_seen_ = read.sequence;
        inhibit |= consume_path_chunk(context);
      }
    }
    return inhibit;
  }

  // Published every cycle with the jog observation. Path fields describe the
  // session whose generation is echoed, including its final state after it
  // closed; a session with no geometry yet publishes zeros for them.
  SharedWordsResult publish_path(FastControlRegion& region, const JogIdentity& identity,
                                 uint64_t now) noexcept {
    protocol::PathObservation o{};
    o.conn_id = identity.connection_id;
    o.native_generation = identity.native_generation;
    o.capability_id = identity.capability_id;
    o.app_grant_generation = identity.app_grant_generation;
    o.config_epoch = identity.configuration_epoch;
    o.home_epoch = identity.home_epoch;
    o.clock_mapping_generation = identity.clock_mapping_generation;
    o.jog_generation = state_.generation;
    o.chunk_sequence_seen = path_chunk_ack_;
    o.chunk_result = path_chunk_result_;
    if (path_ && path_generation_ && path_generation_ == state_.generation) {
      const auto p = path_->observe();
      o.received = p.received;
      o.playable_end_index = p.playable_end_index;
      o.consumed_index = p.consumed_index;
      o.path_s = p.path_s;
      o.path_speed = p.path_speed;
      o.path_acceleration = p.path_acceleration;
      o.underruns = p.underruns;
      o.speed_sequence = p.speed_sequence;
      o.speed_deadline_host_ns = p.speed_deadline_ns;
      o.governing = p.governing;
      o.stop_reason = p.stop_reason;
      o.end_reason = p.end_reason;
      o.flags = (p.open ? path_flag_open : 0u) | (p.stopping ? path_flag_stopping : 0u) |
          (p.ended ? path_flag_ended : 0u) | (p.has_speed ? path_flag_has_speed : 0u) |
          (!p.open && p.stop_reason == static_cast<uint32_t>(PathStopReason::Fault) ? path_flag_fault : 0u);
    }
    o.now_host_ns = now;
    return region.publish_path_observation(o);
  }

  SharedWordsResult publish(FastControlRegion& region, const JogIdentity& identity,
                             uint64_t now) noexcept {
    protocol::JogObservation observation{};
    observation.ticket = ticket_;
    observation.connection_id = identity.connection_id;
    observation.native_generation = identity.native_generation;
    observation.capability_id = identity.capability_id;
    observation.app_grant_generation = identity.app_grant_generation;
    observation.configuration_epoch = identity.configuration_epoch;
    observation.home_epoch = identity.home_epoch;
    observation.clock_mapping_generation = identity.clock_mapping_generation;
    observation.jog_generation = state_.generation;
    observation.source_sequence = state_.source_sequence;
    observation.input_deadline_host_ns = input_deadline_ns();
    observation.now_host_ns = now;
    observation.observed_source_sequence = observed_sequence_;
    observation.observed_origin_host_ns = observed_origin_;
    observation.first_observed_host_ns = first_observed_;
    observation.control_reason = control_reason_;
    observation.control_accepted = control_accepted_;
    observation.update_reason = update_reason_;
    observation.state_reason = input_open() && position_scale_ < 1
        ? static_cast<uint32_t>(JogSessionReason::Limited) : state_reason_;
    observation.axis_mask = state_.axis_mask;
    observation.open = input_open();
    observation.has_input = path_begun() ? path_->observe().has_speed : state_.has_input;
    observation.velocity_scale_ppm = static_cast<uint32_t>(velocity_scale_ppm_ * position_scale_);
    return region.publish_jog_observation(observation);
  }

 private:
  static constexpr uint32_t protocol_path_speed_flag = 1;  // jog_update.flags bit 0
  static constexpr uint32_t path_flag_open = 1, path_flag_stopping = 2, path_flag_ended = 4,
      path_flag_has_speed = 8, path_flag_fault = 16;

  // Input is fenced (observation open = 0) while PathJog stops along the
  // path; the session keeps its axes until the stop completes (path_cycle).
  void enter_path_stop(JogSessionReason reason, uint64_t now) noexcept {
    if (ramping_) return;
    ramp_input_deadline_ns_ = input_deadline_ns();
    ramp_started_ns_ = now;
    ramp_reason_ = reason;
    ramping_ = true;
    state_reason_ = static_cast<uint32_t>(JogSessionReason::Ramping);
  }

  // One path speed command: jog_update flags bit 0, velocity[0] in path
  // units/s, every other entry zero, the session's own mask. Admission is
  // evaluate_jog_update's (identity, generation, increasing sequence, origin
  // never backwards with equal origins unable to extend a deadline, deadline
  // within the input age) against the speed PathJog holds, then
  // PathJog::set_speed. A refusal never closes the session: input expiry and
  // stops belong to PathJog, so a bad frame cannot bend or end the path.
  void path_speed(const protocol::JogUpdate& input, const JogSessionContext& context) noexcept {
    JogSessionReason reason = JogSessionReason::InvalidVector;
    bool valid = input.flags == protocol_path_speed_flag && !input.reserved &&
        std::isfinite(input.velocity[0]) && input.velocity[0] >= 0;
    for (uint32_t axis = 1; axis < jog_session_axes; ++axis) valid &= input.velocity[axis] == 0;
    if (!state_.open || ramping_) {
      reason = JogSessionReason::Closed;
    } else if (!path_mode_) {
      reason = JogSessionReason::ModeConflict;
    } else if (!valid || input.axis_mask != state_.axis_mask) {
      reason = JogSessionReason::InvalidVector;
    } else if (!path_begun_ || !path_->open()) {
      reason = JogSessionReason::PathUnavailable;  // Append geometry before speed.
    } else {
      const auto held = path_->observe();
      JogSessionState current = state_;
      current.has_input = held.has_speed;
      current.input_deadline_ns = held.speed_deadline_ns;
      JogUpdateRequest request{{input.conn_id, input.native_generation, input.capability_id,
          input.app_grant_generation, input.config_epoch, input.home_epoch, input.clock_mapping_generation},
          input.jog_generation, input.source_sequence, input.earliest_origin_host_ns,
          input.immutable_deadline_host_ns, input.axis_mask, {}};
      const auto decision = evaluate_jog_update(current, context, request);
      reason = decision.reason;
      if (decision.accepted) {
        reason = path_jog_reason(path_->set_speed(input.source_sequence, input.velocity[0],
            input.earliest_origin_host_ns, input.immutable_deadline_host_ns, context.now_ns,
            context.max_input_age_ns));
        if (reason == JogSessionReason::None) {
          state_.source_sequence = input.source_sequence;
          state_.input_origin_ns = input.earliest_origin_host_ns;
        }
      }
      if (reason == JogSessionReason::InvalidTiming && input.immutable_deadline_host_ns <= context.now_ns)
        reason = JogSessionReason::Expired;
    }
    update_reason_ = static_cast<uint32_t>(reason);
    state_reason_ = static_cast<uint32_t>(reason);
  }

  // One acknowledged chunk. A chunk for another session or generation (also a
  // stale chunk left in the slot) is ignored without acknowledgement: echoing
  // its sequence could acknowledge a new writer's chunk of the same number.
  // Every other chunk is acknowledged with its sequence and a jog reason; only
  // an accepted chunk advances the expected sequence, so a refused chunk can
  // be corrected and resent. Returns true only when output must be inhibited.
  bool consume_path_chunk(const JogSessionContext& context) noexcept {
    const auto& c = path_chunk_;
    const JogIdentity identity{c.conn_id, c.native_generation, c.capability_id, c.app_grant_generation,
        c.config_epoch, c.home_epoch, c.clock_mapping_generation};
    if (!state_.open || identity != state_.identity || c.jog_generation != state_.generation) return false;
    auto reason = JogSessionReason::None;
    bool inhibit = false;
    if (!path_mode_) {
      reason = JogSessionReason::ModeConflict;
    } else if (c.chunk_sequence != path_chunk_seen_ + 1) {
      reason = JogSessionReason::PathOutOfOrder;
    } else if (c.count > gradient::ipc::v1::PATH_JOG_MAX_CHUNK || (!c.count && !(c.flags & 1)) ||
               (c.flags & ~1u) || c.reserved) {
      reason = JogSessionReason::PathInvalidSample;
    } else if (!std::isfinite(c.spacing) || !(c.spacing > 0) || !std::isfinite(c.max_speed) ||
               !(c.max_speed > 0) || (path_begun_ && (c.spacing != path_spacing_ || c.max_speed != path_max_speed_))) {
      reason = JogSessionReason::PathInvalidConfig;
    } else {
      for (uint32_t i = 0; i < c.count; ++i)
        for (uint32_t axis = 0; axis < path_jog_axes; ++axis)
          if (!(state_.axis_mask & (1u << axis)) && c.samples[i * path_jog_axes + axis] != 0)
            reason = JogSessionReason::PathInvalidSample;
    }
    if (reason == JogSessionReason::None && !path_begun_) {
      if (c.first_index != 0) {
        reason = JogSessionReason::PathOutOfOrder;
      } else {
        auto config = path_config_;
        config.mask = state_.axis_mask;
        PathJogBegin begin{c.spacing, c.max_speed, state_.input_deadline_ns, path_held_};
        reason = path_jog_reason(path_->begin(config, begin, context.now_ns));
        if (reason == JogSessionReason::Closed) reason = JogSessionReason::PathFault;
        if (reason == JogSessionReason::None) {
          path_begun_ = true;
          path_generation_ = state_.generation;
          path_spacing_ = c.spacing;
          path_max_speed_ = c.max_speed;
          state_.input_deadline_ns = UINT64_MAX;
        }
      }
    }
    if (reason == JogSessionReason::None) {
      const auto result = path_->append(c.first_index, c.count, c.samples.data(), c.caps.data(),
                                        (c.flags & 1) != 0, c.end_reason);
      reason = path_jog_reason(result);
      if (result == PathJogResult::StartMismatch) {
        // The path never produced a target other than the held one, so this
        // is a refusal, not a fault: close the session and keep outputs.
        JogSessionDecision closed{state_};
        close_jog_session(closed, JogSessionReason::PathStartMismatch, false);
        closed.inhibit = false;
        inhibit = apply(closed, context.now_ns);
      }
    }
    if (reason == JogSessionReason::None) path_chunk_seen_ = c.chunk_sequence;
    path_chunk_ack_ = c.chunk_sequence;
    path_chunk_result_ = static_cast<uint32_t>(reason);
    return inhibit;
  }

  // Cancellation fences input immediately while retaining RT motion ownership.
  // Only a finite configured halt may keep enable.
  bool apply(const JogSessionDecision& decision, uint64_t now) noexcept {
    // Any close of a path session other than a completed stop (authority
    // loss, inhibit, identity, readiness, conflict) keeps its existing
    // inhibit semantics; the generator is abandoned at the last target.
    if (path_mode() && !decision.next.open && path_ && path_->open()) path_->abandon();
    auto next = decision.next;
    bool inhibit = decision.inhibit;
    bool begin_ramp = decision.needs_qualified_halt && can_ramp(now);
    if (begin_ramp) {
      next.open = true;
      next.axis_mask = state_.axis_mask;
      next.input_deadline_ns = UINT64_MAX; // Input is fenced; poll_ramp owns the halt deadline.
      ramp_input_deadline_ns_ = state_.input_deadline_ns;
      ramp_started_ns_ = now;
      ramp_reason_ = decision.reason == JogSessionReason::None ? JogSessionReason::Closed : decision.reason;
      ramping_ = true;
      inhibit = false;
    } else if (decision.closed || inhibit) {
      ramping_ = false;
    }
    state_ = next;
    if (ramping_)
      state_reason_ = static_cast<uint32_t>(JogSessionReason::Ramping);
    else if (decision.reason != JogSessionReason::None || decision.accepted)
      state_reason_ = static_cast<uint32_t>(decision.reason);
    return inhibit;
  }
  bool can_ramp(uint64_t now) noexcept {
    if (!cycle_ns_) return false;
    for (uint32_t axis = 0; axis < jog_session_axes; ++axis) {
      if (!(state_.axis_mask & (1u << axis))) continue;
      if (!axes_[axis] || !ramps_[axis].begin(*axes_[axis], acceleration_[axis], cycle_ns_, now)) return false;
    }
    return true;
  }
  bool poll_ramp(uint64_t now) noexcept {
    bool stopped = true, failed = now < ramp_started_ns_;
    for (uint32_t axis = 0; axis < jog_session_axes; ++axis) {
      if (!(state_.axis_mask & (1u << axis))) continue;
      stopped &= axes_[axis]->velocity == 0;
      failed |= axes_[axis]->velocity != 0 &&
          now - ramp_started_ns_ > ramps_[axis].bound_ns;
    }
    // Publish at least one ramping cycle, even for an already stationary input.
    if (!failed && (!stopped || now == ramp_started_ns_)) return false;
    JogSessionDecision closed{state_};
    close_jog_session(closed, ramp_reason_, false);
    closed.inhibit = failed;
    ramping_ = false;
    return apply(closed, now);
  }
  std::array<JogAxis*, jog_session_axes> axes_{};
  std::array<AxisLimits, jog_session_axes> limits_{};
  std::array<double, jog_session_axes> feedback_{};
  std::array<double, jog_session_axes> scales_{};
  uint32_t limit_mask_ = 0;
  std::array<JogTiming, jog_session_axes> timing_{};
  std::array<double, jog_session_axes> acceleration_{};
  std::array<HaltRamp, jog_session_axes> ramps_{};
  uint64_t ramp_now_ns_ = 0;
  uint64_t cycle_ns_ = 0, ramp_started_ns_ = 0, ramp_input_deadline_ns_ = 0;
  bool ramping_ = false;
  // Preserve the cancellation cause so a completed halt cannot hide input expiry.
  JogSessionReason ramp_reason_ = JogSessionReason::Closed;
  // Exact wire unit: unscaled input is 1,000,000 parts per million.
  uint32_t velocity_scale_ppm_ = 1'000'000;
  double position_scale_ = 1;
  JogSessionState state_{};
  uint64_t connection_ = 0, control_seen_ = 0, update_seen_ = 0, ticket_ = 0;
  uint64_t observed_sequence_ = 0, observed_origin_ = 0, first_observed_ = 0;
  uint32_t control_reason_ = 0, control_accepted_ = 0, update_reason_ = 0, state_reason_ = 0;
  PathJog* path_ = nullptr;
  PathJogConfig path_config_{};
  std::array<double, path_jog_axes> path_held_{};
  // Copy target for one chunk: a member, so the 2296-byte record never sits
  // on the cyclic thread's stack.
  protocol::PathSamples path_chunk_{};
  bool path_mode_ = false, path_begun_ = false;
  uint64_t path_generation_ = 0, path_publication_seen_ = 0, path_chunk_seen_ = 0, path_chunk_ack_ = 0;
  uint32_t path_chunk_result_ = 0;
  double path_spacing_ = 0, path_max_speed_ = 0;
};

static_assert(static_cast<uint32_t>(JogSessionReason::GenerationExhausted) == protocol::jog_reason_generation_exhausted);
static_assert(static_cast<uint32_t>(JogSessionReason::Ramping) == protocol::jog_reason_ramping);
static_assert(static_cast<uint32_t>(JogSessionReason::Limited) == protocol::jog_reason_limited);
static_assert(static_cast<uint32_t>(JogSessionReason::OutsideLimitsOutward) == protocol::jog_reason_outside_limits_outward);
static_assert(static_cast<uint32_t>(JogSessionReason::PathStartMismatch) == protocol::jog_reason_path_start_mismatch);
static_assert(static_cast<uint32_t>(JogSessionReason::PathCapacity) == protocol::jog_reason_path_capacity);
static_assert(static_cast<uint32_t>(JogSessionReason::PathOutOfOrder) == protocol::jog_reason_path_out_of_order);
static_assert(static_cast<uint32_t>(JogSessionReason::PathInvalidSample) == protocol::jog_reason_path_invalid_sample);
static_assert(static_cast<uint32_t>(JogSessionReason::PathEnded) == protocol::jog_reason_path_ended);
static_assert(static_cast<uint32_t>(JogSessionReason::PathFault) == protocol::jog_reason_path_fault);
static_assert(static_cast<uint32_t>(JogSessionReason::PathUnavailable) == protocol::jog_reason_path_unavailable);
static_assert(static_cast<uint32_t>(JogSessionReason::PathInvalidConfig) == protocol::jog_reason_path_invalid_config);
static_assert(static_cast<uint32_t>(JogSessionReason::StaleSequence) == protocol::jog_reason_stale_sequence);
static_assert(static_cast<uint32_t>(JogSessionReason::InvalidTiming) == protocol::jog_reason_invalid_timing);
static_assert(static_cast<uint32_t>(JogSessionReason::Closed) == protocol::jog_reason_closed);
static_assert(gradient::ipc::v1::PATH_JOG_MAX_SAMPLES == path_jog_capacity);
static_assert(gradient::ipc::v1::PATH_JOG_MAX_CHUNK * path_jog_axes == std::tuple_size<decltype(protocol::PathSamples::samples)>::value);
static_assert(gradient::ipc::v1::PATH_JOG_MAX_CHUNK == std::tuple_size<decltype(protocol::PathSamples::caps)>::value);
static_assert(path_jog_axes == protocol::max_axes);
// Path observation enums are PathJog's, value for value.
static_assert(static_cast<uint32_t>(PathGoverning::Stopping) == 8 && static_cast<uint32_t>(PathStopReason::Fault) == 8);
} // namespace rosie::rtcore
