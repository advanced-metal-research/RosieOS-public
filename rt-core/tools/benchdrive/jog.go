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

// Unverified host cadence/budget: 5,000,000 ns updates, 100,000,000 ns input
// lifetime, below the protocol's 250,000,000 ns maximum source age.
const (
	jogCadence = 5 * time.Millisecond
	inputNS    = 100_000_000
)

// Unverified terminal neutralisation window: 25,000,000 ns. Submit zero before
// the last producer slot so a daemon cycle cannot expire the preceding input.
const terminalJogNS = 25_000_000

// jogLimits requires finite, explicit limits and trustworthy coordinates before
// enabling. The conservative swept interval includes a full-duration excursion
// and deceleration distance, preventing a reversal assumption hiding a boundary.
func (r *runner) jogLimits(s control.Status) error {
	a := r.description.Axes[r.o.axis]
	for _, value := range []float64{a.MaxVelocity, a.JogAcceleration, a.CountsPerUnit, a.MaxTargetLead, a.FollowingError, a.CompletionTolerance} {
		if !finite(value) || value <= 0 {
			return errors.New("jog requires explicit positive finite Describe limits and counts_per_unit")
		}
	}
	if a.PositionUnit != "rad" || !finite(a.MinPosition) || !finite(a.MaxPosition) || a.MinPosition >= a.MaxPosition || (a.Sign != 1 && a.Sign != -1) {
		return errors.New("jog requires finite ordered position limits and a valid sign")
	}
	axis := s.Core.Axes[r.o.axis]
	velocity := r.o.fraction * a.MaxVelocity
	travel := velocity*float64(r.o.jogNS)/float64(time.Second) + velocity*velocity/(2*a.JogAcceleration)
	if r.o.direction != 0 {
		// Unverified producer-stall allowance: one 100,000,000 ns frame lifetime.
		travel += velocity * float64(inputNS) / float64(time.Second)
	}
	lo, hi := math.Min(axis.LogicalPosition, axis.LogicalTarget), math.Max(axis.LogicalPosition, axis.LogicalTarget)
	if r.o.direction <= 0 {
		lo -= travel
	}
	if r.o.direction >= 0 {
		hi += travel
	}
	if r.o.direction < -1 || r.o.direction > 1 || !finite(velocity) || velocity <= 0 || r.o.fraction > maxFraction ||
		!axis.HomeValid || !axis.LogicalValid || !finite(axis.LogicalPosition) || !finite(axis.LogicalTarget) || !finite(travel) ||
		!finite(travel*r.o.motorRevsPerAxisRev) || lo < a.MinPosition || hi > a.MaxPosition {
		return errors.New("jog lacks Home/coordinates or conservative travel crosses Describe limits")
	}
	return r.check("jog_limits", true)
}

// jog sends on a dedicated producer, independent of status I/O. Every nonzero
// frame retains its source-age expiry. Directional input stays signed until the
// duration ends; a stalled producer cannot renew it. End precedes Stop.
func (r *runner) jog(ctx context.Context, cancelRun context.CancelCauseFunc) (err error) {
	origin := make([]int32, len(r.last.Core.Axes))
	startPosition := r.last.Core.Axes[r.o.axis].LogicalPosition
	for i, a := range r.last.Core.Axes {
		origin[i] = a.PositionCounts
	}
	if err = r.position("jog_before", r.last, origin); err != nil {
		return err
	}
	begin := ipcclient.HostMonotonicNS()
	fence := r.c.Fence()
	var generation uint64
	end, reversal := begin+uint64(r.o.jogNS), begin+uint64(r.o.jogNS/2)
	deadline := min(end, begin+inputNS)
	if r.o.direction != 0 {
		deadline = begin + inputNS
	}
	mask := uint32(1 << r.o.axis)
	callCtx, callCancel := timeout(ctx, r.o.deadlineNS)
	session, err := r.c.NewJogSession(callCtx, mask, deadline)
	callCancel()
	if err = r.e.receipt("begin_jog", begin, map[string]any{"sdk_method": "NewJogSession", "axis_mask": mask, "direction": r.o.direction, "deadline_host_ns": deadline, "fence": fence}, map[string]any{"created": session != nil}, err); err != nil {
		return err
	}
	ctx, cancel := context.WithCancelCause(ctx)
	producerDone := make(chan error, 1)
	startedProducer := false
	defer func() {
		cancel(nil)
		if startedProducer {
			if producerErr := <-producerDone; producerErr != nil && !errors.Is(err, producerErr) {
				err = errors.Join(err, producerErr)
			}
		}
		endCtx, stop := timeout(context.Background(), r.o.deadlineNS)
		defer stop()
		start := ipcclient.HostMonotonicNS()
		endErr := session.End(endCtx)
		endErr = r.e.receipt("end_jog", start, map[string]any{"fence": fence, "jog_generation": generation}, map[string]any{"ended": endErr == nil}, endErr)
		if endErr == nil {
			r.expectations["jog_end"] = true
		}
		err = errors.Join(err, endErr)
		if err == nil {
			// End may ramp to hold. Observe the final position before outer Stop.
			_, err = r.observe(endCtx, func(s control.Status) (bool, error) {
				if e := r.ready(s, true); e != nil {
					return false, e
				}
				return s.Core.TimeNS >= start && s.Jog != nil && s.Jog.Open == 0 && s.Jog.StateReason != ipcclient.JogReasonRamping, nil
			})
			if err == nil {
				err = r.position("jog_after", r.last, origin)
				r.expectations["jog_after"] = err == nil
				if err == nil {
					travel := r.last.Core.Axes[r.o.axis].LogicalPosition - startPosition
					var motorRevs any
					if r.o.motorRevsPerAxisRev > 0 {
						motorRevs = travel * r.o.motorRevsPerAxisRev / (2 * math.Pi)
					}
					err = r.e.write(record{Kind: "travel", Operation: "jog_travel", Response: map[string]any{
						"axis": r.o.axis, "direction": r.o.direction, "measured_travel_rad": travel,
						"motor_revolutions": motorRevs, "motor_revs_per_axis_rev": r.o.motorRevsPerAxisRev}})
				}
			}
		}
	}()
	callCtx, callCancel = timeout(ctx, r.o.deadlineNS)
	start := ipcclient.HostMonotonicNS()
	initial, err := r.c.JogStatus(callCtx)
	callCancel()
	if err = r.e.receipt("jog_status", start, struct{}{}, initial, err); err != nil {
		return err
	}
	if initial.JogGeneration == 0 || initial.Open != 1 || initial.ControlAccepted != 1 {
		return errors.New("BeginJog lacks an accepted open observation")
	}
	generation = initial.JogGeneration
	if r.o.direction != 0 {
		// Start the requested duration after BeginJog is observed, preventing
		// negotiation latency from consuming the operator's commanded travel.
		end = ipcclient.HostMonotonicNS() + uint64(r.o.jogNS)
	}
	velocity := r.o.fraction * r.description.Axes[r.o.axis].MaxVelocity
	if r.o.direction != 0 {
		velocity *= float64(r.o.direction)
	}
	startedProducer = true
	go func() {
		var producerErr error
		defer func() {
			if value := recover(); value != nil {
				producerErr = fmt.Errorf("jog producer panic: %v", value)
				cancelRun(producerErr)
			}
			if producerErr != nil {
				cancel(producerErr)
			}
			producerDone <- producerErr
		}()
		producerErr = r.sendJog(ctx, session, fence, generation, velocity, end, reversal)
	}()
	// Polls retain native and ingress reasons. A refusal never waits for the
	// requested duration to finish, and a new grant cannot revive this producer.
	ticker := time.NewTicker(pollCadence)
	defer ticker.Stop()
	for {
		callCtx, callCancel := timeout(ctx, inputNS)
		s, statusErr := r.status(callCtx)
		callCancel()
		if statusErr != nil {
			return statusErr
		}
		if err = r.ready(s, true); err != nil {
			return err
		}
		j := s.Jog
		if j == nil || j.JogGeneration != initial.JogGeneration || j.AppGrantGeneration != initial.AppGrantGeneration {
			return errors.New("jog observation identity changed")
		}
		if j.UpdateReason != ipcclient.JogReasonNone || (j.StateReason != ipcclient.JogReasonNone && j.StateReason != ipcclient.JogReasonLimited) || j.Open != 1 {
			rejected := &control.Rejected{Reason: "native_jog_rejected", NativeJogResult: j}
			return r.e.receipt("jog_observation", 0, struct{}{}, j, rejected)
		}
		if j.HasInput == 1 && j.ObservedSourceSequence != 0 {
			r.expectations["jog_input_observed"] = true
		}
		callCtx, callCancel = timeout(ctx, inputNS)
		start = ipcclient.HostMonotonicNS()
		ingress, ingressErr := r.c.JogIngress(callCtx)
		callCancel()
		if ingressErr = r.e.receipt("jog_ingress", start, struct{}{}, ingress, ingressErr); ingressErr != nil {
			return ingressErr
		}
		if ingress.Reason != 0 && ingress.NowHostNS >= begin {
			refused := &control.JogRejection{Reason: fmt.Sprintf("jog_native_rejected_%d", ingress.Reason), NativeReason: ingress.Reason, Sequence: ingress.SourceSequence}
			return r.e.receipt("jog_ingress_rejection", start, struct{}{}, ingress, refused)
		}
		now := ipcclient.HostMonotonicNS()
		if r.o.direction == 0 && !r.expectations["jog_reversal"] && s.Core.TimeNS >= reversal {
			if err = r.position("jog_reversal", s, origin); err != nil {
				return err
			}
			r.expectations["jog_reversal"] = true
		}
		if now >= end {
			if !r.expectations["jog_input_observed"] || (r.o.direction == 0 && !r.expectations["jog_reversal"]) {
				return errors.New("jog input or reversal was never observed")
			}
			return nil
		}
		select {
		case <-ctx.Done():
			return context.Cause(ctx)
		case <-ticker.C:
		}
	}
}

func (r *runner) sendJog(ctx context.Context, session *control.JogSession, fence control.Fence, generation uint64, velocity float64, end, reversal uint64) error {
	ticker := time.NewTicker(jogCadence)
	defer ticker.Stop()
	for sequence := uint64(1); ; sequence++ {
		if ctx.Err() != nil {
			return nil // The observer retains the cancellation cause.
		}
		now := ipcclient.HostMonotonicNS()
		if now >= end && r.o.direction == 0 {
			return nil
		}
		v := velocity
		if r.o.direction == 0 && now >= reversal {
			v = -v
		}
		// Neutralise before the final producer slot so End can be acknowledged
		// without racing input expiry. Only zero input may outlive the motion
		// deadline; all nonzero frames remain capped at that absolute bound.
		terminal := end-now <= terminalJogNS
		if r.o.direction != 0 {
			terminal = now >= end
		}
		if terminal {
			v = 0
		}
		if !finite(v) || math.Abs(v) > r.o.fraction*r.description.Axes[r.o.axis].MaxVelocity {
			return errors.New("jog velocity exceeds the fraction bound")
		}
		vector := make([]float64, len(r.description.Axes))
		vector[r.o.axis] = v
		deadline := min(end, now+inputNS)
		if terminal || r.o.direction != 0 {
			deadline = now + inputNS
		}
		err := session.Update(vector, deadline)
		if err = r.e.receipt("update_jog", now, map[string]any{"source_sequence": sequence, "axis_mask": uint32(1 << r.o.axis), "direction": r.o.direction,
			"fence": fence, "jog_generation": generation,
			"velocity": vector, "deadline_host_ns": deadline, "jog_end_host_ns": end, "reversal_host_ns": reversal, "terminal_zero": terminal},
			map[string]any{"sent": err == nil}, err); err != nil {
			return err
		}
		if terminal {
			return nil
		}
		select {
		case <-ctx.Done():
			return nil
		case <-ticker.C:
		}
	}
}
