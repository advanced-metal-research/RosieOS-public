//go:build linux

package main

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Velocity reserve: plan at 75% of the requested cap. Nine-motor bench, 2026-09-14
// (bd-align-all-3.jsonl): the A6-EC position loop overshot a 0.18 rad/s planned
// peak to 0.2029 rad/s observed over one 100 ms status interval, 12.7% above
// plan. Observations enforce the axis limit and record overshoot separately.
// Both move and dense program profiles use this reserve.
const moveVelocityReserve = 0.75

// axisVelocity estimates only between consecutive distinct axis publications.
// Position and time always come from the same AxisStatus; request and motion
// timestamps must not shorten the interval. Invalid or reordered feedback fails
// closed and breaks the pair, preventing a later sample bridging missing data.
type axisVelocity struct {
	previous ipcclient.AxisStatus
}

func (v *axisVelocity) observe(a ipcclient.AxisStatus) (float64, bool, error) {
	prev := v.previous
	if !a.LogicalValid || !finite(a.LogicalPosition) || a.TimeNS == 0 || a.TimeNS < prev.TimeNS ||
		(a.TimeNS == prev.TimeNS && a.LogicalPosition != prev.LogicalPosition) {
		v.previous = ipcclient.AxisStatus{}
		return 0, false, errors.New("invalid or non-consecutive axis feedback")
	}
	if a.TimeNS == prev.TimeNS {
		return 0, false, nil
	}
	v.previous = a
	if prev.TimeNS == 0 {
		return 0, false, nil
	}
	return math.Abs(a.LogicalPosition-prev.LogicalPosition) * float64(time.Second) / float64(a.TimeNS-prev.TimeNS), true, nil
}

// moveProfile keeps the single-axis entry point: one displacement on o.axis.
func moveProfile(d control.Description, o options, origin float64) ([]control.Point, error) {
	origins := map[int]float64{o.axis: origin}
	return moveProfileGroup(d, o, []moveSpec{{axis: o.axis, delta: o.delta}}, origins)
}

// moveProfileGroup samples one bounded trapezoid (triangle for short moves) at
// the daemon cycle for every axis of a simultaneous move. The slowest axis at
// its own velocity and acceleration caps defines the shared time profile; every
// other axis follows the same normalised curve scaled to its own displacement,
// so its peak velocity is lower than its own cap. Rounding stretches time,
// preventing a final partial cycle from exceeding either limit, and every point
// is checked against every axis cap. Refuse excessive point counts before allocating.
func moveProfileGroup(d control.Description, o options, moves []moveSpec, origins map[int]float64) ([]control.Point, error) {
	if len(moves) == 0 || d.CycleNS == 0 || d.CycleNS > uint64(math.MaxInt64) || d.MaxTrajectoryPoints < 3 ||
		!finite(o.velocityFraction) || o.velocityFraction <= 0 || o.velocityFraction > maxFraction ||
		!finite(o.accelFraction) || o.accelFraction <= 0 || o.accelFraction > maxFraction {
		return nil, errors.New("invalid trajectory geometry")
	}
	type plan struct {
		acceleration, velocity, distance, ramp, peak, cruise, total float64
	}
	plans := make([]plan, len(moves))
	dominant := -1
	for i, m := range moves {
		if m.axis < 0 || m.axis >= len(d.Axes) {
			return nil, errors.New("invalid trajectory geometry")
		}
		a := d.Axes[m.axis]
		origin, ok := origins[m.axis]
		if !ok || a.PositionUnit != "rad" || !finite(origin) || !finite(m.delta) || m.delta == 0 ||
			!finite(a.MaxVelocity) || a.MaxVelocity <= 0 || !finite(a.JogAcceleration) || a.JogAcceleration <= 0 ||
			!finite(a.CompletionTolerance) || a.CompletionTolerance <= 0 || !finite(a.MinPosition) || !finite(a.MaxPosition) ||
			origin < a.MinPosition || origin > a.MaxPosition || origin+m.delta < a.MinPosition || origin+m.delta > a.MaxPosition {
			return nil, errors.New("move violates described units, position, velocity or acceleration limits")
		}
		acceleration := a.JogAcceleration
		if a.MaxAcceleration != nil {
			if !finite(*a.MaxAcceleration) || *a.MaxAcceleration <= 0 {
				return nil, errors.New("invalid described acceleration")
			}
			acceleration = math.Min(acceleration, *a.MaxAcceleration)
		}
		acceleration *= o.accelFraction
		velocity := a.MaxVelocity * o.velocityFraction * moveVelocityReserve
		distance := math.Abs(m.delta)
		ramp := math.Min(velocity/acceleration, math.Sqrt(distance/acceleration))
		peak := acceleration * ramp
		cruise := math.Max(0, (distance-acceleration*ramp*ramp)/peak)
		plans[i] = plan{acceleration, velocity, distance, ramp, peak, cruise, 2*ramp + cruise}
		if dominant < 0 || plans[i].total > plans[dominant].total {
			dominant = i
		}
	}
	lead := plans[dominant]
	cycle := float64(d.CycleNS) / float64(time.Second)
	steps := math.Ceil(lead.total / cycle)
	if !finite(steps) || steps < 1 || steps+1 >= float64(d.MaxTrajectoryPoints) || steps > float64(math.MaxInt64)/float64(d.CycleNS) {
		return nil, errors.New("move exceeds max_trajectory_points or time range")
	}
	scale := lead.total / (steps * cycle)
	points := make([]control.Point, int(steps)+1)
	for i := range points {
		t := float64(i) * cycle * scale
		var p float64
		switch {
		case t < lead.ramp:
			p = lead.acceleration * t * t / 2
		case t < lead.ramp+lead.cruise:
			p = lead.acceleration*lead.ramp*lead.ramp/2 + lead.peak*(t-lead.ramp)
		default:
			remaining := math.Max(0, lead.total-t)
			p = lead.distance - lead.acceleration*remaining*remaining/2
		}
		if i == len(points)-1 {
			p = lead.distance
		}
		fraction := p / lead.distance
		positions := make([]float64, len(d.Axes))
		for _, m := range moves {
			positions[m.axis] = origins[m.axis] + m.delta*fraction
		}
		points[i] = control.Point{Time: time.Duration(uint64(i) * d.CycleNS), Position: positions}
		if i > 0 {
			for _, m := range moves {
				a := d.Axes[m.axis]
				if math.Abs(points[i].Position[m.axis]-points[i-1].Position[m.axis])/cycle > a.MaxVelocity*o.velocityFraction {
					return nil, errors.New("profile exceeded velocity cap")
				}
			}
		}
	}
	return points, nil
}

func handleState(s control.Status, handle uint64) (string, uint64) {
	for _, h := range s.Execution.Handles {
		if h.Handle == handle {
			return string(h.State), h.NativeSequence
		}
	}
	return "", 0
}

// move requires the same Enable/Arm checks as hold and observes correlated
// completion and actual travel on every axis of the group. A receipt alone
// never establishes the endpoint.
func (r *runner) move(ctx context.Context) error {
	if err := r.ready(r.last, true); err != nil {
		return err
	}
	moves := r.o.moves
	if len(moves) == 0 {
		moves = []moveSpec{{axis: r.o.axis, delta: r.o.delta}}
	}
	origins := map[int]float64{}
	for _, m := range moves {
		if m.axis < 0 || m.axis >= len(r.last.Core.Axes) {
			return errors.New("move axis outside status")
		}
		// The daemon admits a start only when the first point equals its held
		// wire target within one count (execution_policy.hpp continuous_start).
		// Feedback rests with a 1..3 count spread on the A6-EC bench, so the
		// profile begins at the published held target, not at the feedback.
		a := r.last.Core.Axes[m.axis]
		origins[m.axis] = a.LogicalTarget
	}
	planOptions := r.o
	if r.o.phase == "halt" {
		axis := r.description.Axes[r.o.axis]
		planOptions.velocityFraction = r.o.cruiseVelocity / (axis.MaxVelocity * moveVelocityReserve)
		if planOptions.velocityFraction > maxFraction {
			return r.measurementRefusal("halt_cruise_exceeds_velocity_cap")
		}
		acceleration := axis.JogAcceleration
		if axis.MaxAcceleration != nil {
			acceleration = math.Min(acceleration, *axis.MaxAcceleration)
		}
		// Reserve two cruise windows and the status observation cadence inside
		// a complete accelerate/cruise/decelerate path before any Start request.
		needed := r.o.cruiseVelocity*r.o.cruiseVelocity/(acceleration*r.o.accelFraction) +
			r.o.cruiseVelocity*(2*float64(measurementWindowNS)+float64(pollCadence))/1e9
		if !finite(needed) || needed >= math.Abs(r.o.delta) {
			return r.measurementRefusal("halt_cruise_unreachable_within_travel")
		}
	}
	points, err := moveProfileGroup(r.description, planOptions, moves, origins)
	if err != nil {
		if r.o.measure {
			return errors.Join(err, r.measurementRefusal("measurement_profile_invalid"))
		}
		return err
	}
	var moveMask uint32
	for _, m := range moves {
		moveMask |= 1 << m.axis
	}
	response, err := r.request(ctx, control.Request{Operation: "prepare_trajectory", AxisMask: moveMask, Points: points})
	if err != nil {
		return err
	}
	handle := response.Handle
	if err := r.check("move_prepared", handle != 0); err != nil {
		return err
	}
	_, err = r.observe(ctx, func(s control.Status) (bool, error) {
		if err := r.pollCollision(ctx, "move", moveMask); err != nil {
			return false, err
		}
		if err := r.ready(s, true); err != nil {
			return false, err
		}
		state, _ := handleState(s, handle)
		return state == "prepared", nil
	})
	if err != nil {
		return err
	}
	var measurement *cycleMeasurement
	if r.o.measure {
		measurement, err = r.beginMeasurement(ctx, moves, points)
		if err != nil {
			return err
		}
		measurement.startNS = ipcclient.HostMonotonicNS()
	}
	response, err = r.request(ctx, control.Request{Operation: "start_trajectory", Handle: handle})
	if err != nil {
		return err
	}
	if response.Sequence == 0 {
		return control.ErrProtocol
	}
	velocities := map[int]*axisVelocity{}
	peaks := map[int]*velocityMeasurement{}
	for _, m := range moves {
		peak := 0.0
		for i := 1; i < len(points); i++ {
			peak = math.Max(peak, math.Abs(points[i].Position[m.axis]-points[i-1].Position[m.axis])/(points[i].Time-points[i-1].Time).Seconds())
		}
		peaks[m.axis] = r.beginVelocity("move", m.axis, peak)
		velocities[m.axis] = &axisVelocity{}
		a := r.last.Core.Axes[m.axis]
		if _, _, err := velocities[m.axis].observe(a); err != nil {
			return err
		}
		if err := r.movePosition("move_before", m.axis, a); err != nil {
			return err
		}
	}
	_, err = r.observe(ctx, func(s control.Status) (bool, error) {
		if err := r.pollCollision(ctx, "move", moveMask); err != nil {
			return false, err
		}
		if err := r.ready(s, true); err != nil {
			return false, err
		}
		if measurement != nil {
			if err := measurement.poll(ctx); err != nil {
				return false, err
			}
			if r.c.Fence() != measurement.fence {
				return false, r.measurementRefusal("measurement_generation_changed")
			}
			if r.o.phase == "halt" {
				if measurement.haltSequence == 0 {
					if s.Motion.Done {
						return false, r.measurementRefusal("halt_cruise_not_observed_before_completion")
					}
					if s.Motion.CommandSequence == response.Sequence && measurement.atCruise(r.o.axis, len(measurement.samples)-1) {
						measurement.haltStartNS = ipcclient.HostMonotonicNS()
						halt, err := r.request(ctx, control.Request{Operation: "halt", Fence: measurement.fence})
						measurement.haltReceiptNS = ipcclient.HostMonotonicNS()
						if err != nil {
							return false, err
						}
						if halt.Sequence == 0 {
							return false, control.ErrProtocol
						}
						measurement.haltSequence = halt.Sequence
						if err := r.e.write(record{Kind: "completion", Operation: "halt_receipt", Response: map[string]any{
							"request_host_ns": measurement.haltStartNS, "receipt_host_ns": measurement.haltReceiptNS,
							"native_sequence": halt.Sequence, "fence": measurement.fence,
							"standstill_band_counts": standstillBandCounts, "standstill_window_ns": standstillWindowNS,
							"deceleration_window_ns": measurementWindowNS}}); err != nil {
							return false, err
						}
					}
					return false, nil
				}
				_, _, stable := measurement.stable(r.o.axis)
				state, _ := handleState(s, handle)
				if s.Motion.CommandSequence != measurement.haltSequence || !s.Motion.Done || !stable || (state != "retired" && state != "consumed") {
					return false, nil
				}
				last := measurement.samples[len(measurement.samples)-1].Ax[r.o.axis]
				first := measurement.samples[0].Ax[r.o.axis]
				a := r.description.Axes[r.o.axis]
				heldDelta := (s.Core.Axes[r.o.axis].LogicalTarget - origins[r.o.axis]) * a.CountsPerUnit * float64(a.Sign)
				if math.Abs(heldDelta-float64(int64(last.Target)-int64(first.Target))) > 1 {
					return false, r.measurementRefusal("halt_held_target_changed")
				}
				return true, r.check("halt_armed_idle_same_generation_held_target", s.Core.Armed == 1 && s.Core.CommissioningPhase == 0 && s.Motion.Mode == ipcclient.MOTION_MODE_IDLE && s.Motion.QueueDepth == 0 && r.c.Fence() == measurement.fence)
			}
		}
		settled := true
		for _, m := range moves {
			axis := r.description.Axes[m.axis]
			a := s.Core.Axes[m.axis]
			velocity, fresh, err := velocities[m.axis].observe(a)
			if err != nil {
				return false, err
			}
			if fresh {
				if err := r.observeVelocity(peaks[m.axis], velocity); err != nil {
					return false, err
				}
				if err := r.movePosition("move_sample", m.axis, a); err != nil {
					return false, err
				}
			}
			if math.Abs(a.LogicalPosition-origins[m.axis]-m.delta) > axis.CompletionTolerance {
				settled = false
			}
			if measurement != nil {
				_, _, stable := measurement.stable(m.axis)
				settled = settled && stable
			}
		}
		state, sequence := handleState(s, handle)
		terminal := state == "consumed" || state == "retired"
		complete := s.Motion.TrajectoryID == handle && s.Motion.CommandSequence == response.Sequence && s.Motion.Done && s.Motion.State == 4
		return complete && terminal && sequence == response.Sequence && settled, nil
	})
	if err != nil {
		return err
	}
	if measurement != nil {
		if err := measurement.finish(); err != nil {
			return err
		}
	}
	if r.o.phase == "halt" {
		return r.e.write(record{Kind: "completion", Operation: "halt", Response: map[string]any{
			"armed_idle": true, "same_session_and_generation": true, "held_target_equals_last_commanded_target": true,
			"native_sequence": measurement.haltSequence, "fence": r.c.Fence()}})
	}
	for _, m := range moves {
		axis := r.description.Axes[m.axis]
		end := r.last.Core.Axes[m.axis].LogicalPosition
		if err := r.e.write(record{Kind: "travel", Operation: "move_travel", Response: map[string]any{
			"axis": m.axis, "start_logical_position": origins[m.axis], "end_logical_position": end, "measured_travel_rad": end - origins[m.axis],
			"delta_rad": m.delta, "completion_tolerance": axis.CompletionTolerance, "velocity_bound_rad_s": axis.MaxVelocity,
			"handle": handle}}); err != nil {
			return err
		}
	}
	r.expectations["move_endpoint"], r.expectations["move_velocity_bound"], r.expectations["move_handle_terminal"] = true, true, true
	// Done plus a terminal handle establishes that execution has ended at its
	// held endpoint. Stop is unnecessary for completion, but defaults to explicit
	// inhibition; --keep-armed skips it only until mandatory Release inhibits.
	if err := r.e.write(record{Kind: "completion", Operation: "move", Response: "Trajectory Done and terminal handle observed at endpoint: execution holds the endpoint, so completion needs no Stop. Default Stop follows for explicit inhibition; --keep-armed skips it until mandatory Release."}); err != nil {
		return err
	}
	r.completedMove = true
	return nil
}

func (r *runner) movePosition(label string, axis int, a ipcclient.AxisStatus) error {
	return r.e.write(record{Kind: "position", Operation: label, Response: map[string]any{"axis": axis, "time_ns": a.TimeNS, "logical_position": a.LogicalPosition}})
}

// discard prepares a stationary plan without Enable or Arm and requires a
// discarded handle and unchanged feedback before accepting the non-motion test.
func (r *runner) discard(ctx context.Context) error {
	origin := r.last
	if r.description.CycleNS == 0 || r.description.CycleNS > uint64(math.MaxInt64) || r.description.MaxTrajectoryPoints <= 2 {
		return errors.New("invalid trajectory geometry")
	}
	positions := make([]float64, len(origin.Core.Axes))
	for i, a := range origin.Core.Axes {
		if r.o.mask&(1<<i) != 0 {
			if !a.LogicalValid || !finite(a.LogicalPosition) {
				return fmt.Errorf("axis %d lacks logical position for preparation", i)
			}
			positions[i] = a.LogicalPosition
		}
	}
	points := []control.Point{{Time: 0, Position: positions}, {Time: time.Duration(r.description.CycleNS), Position: positions}}
	response, err := r.request(ctx, control.Request{Operation: "prepare_trajectory", AxisMask: uint32(r.o.mask), Points: points})
	if err != nil {
		return err
	}
	handle := response.Handle
	if err := r.check("discard_prepared", handle != 0); err != nil {
		return err
	}
	_, err = r.observe(ctx, func(s control.Status) (bool, error) {
		state, _ := handleState(s, handle)
		return state == "prepared", r.stationary(s, origin)
	})
	if err != nil {
		return err
	}
	if _, err := r.request(ctx, control.Request{Operation: "discard_trajectory", Handle: handle}); err != nil {
		return err
	}
	_, err = r.observe(ctx, func(s control.Status) (bool, error) {
		state, _ := handleState(s, handle)
		return state == "discarded", r.stationary(s, origin)
	})
	if err != nil {
		return err
	}
	r.expectations["handle_discarded"] = true
	// Unverified unknown-handle probe: the maximum uint64 cannot be allocated
	// by this bounded invocation. It must be refused without reaching motion.
	_, err = r.request(ctx, control.Request{Operation: "discard_trajectory", Handle: math.MaxUint64})
	if err := r.expectRefusal("unknown_discard_refused", "unknown_handle", err); err != nil {
		return err
	}
	return r.noMotion(ctx, origin, "discard_no_motion")
}
