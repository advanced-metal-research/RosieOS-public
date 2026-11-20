//go:build linux

package denseexec

import (
	"context"
	"errors"
	"fmt"
	"log"
	"math"
	"slices"
	"strings"

	"rosieos/rt-core/sdk/control"
)

// RecoverToZero runs the one-press recovery (recover_zero_plan.go) for the
// client that pressed it; the HTTP gates have already admitted that client
// as the controller, refused it during a maintenance notice and fenced it to
// the displayed cell. Every step is one journal line naming the client.
// The error is the refusal (typed, with the result) or a transport failure.
func (r *RTCoreController) RecoverToZero(ctx context.Context, in RecoverToZeroRequest) (*RecoverToZeroResult, error) {
	result := &RecoverToZeroResult{Action: RecoverToZeroAction, Steps: []RecoverStep{}}
	journal := stepJournal(ctx, "recover-to-zero", &result.Steps)
	refuse := func(step, code, detail string) (*RecoverToZeroResult, error) {
		journal(RecoverStep{Step: step, Outcome: "refused", Detail: code + ": " + detail})
		result.State, result.Refusal = "refused", &RecoverRefusal{Step: step, Code: code, Detail: detail}
		result.Status = r.Status(context.Background())
		return result, httpError(409, code, "%s", detail)
	}
	if !(in.Fraction > 0 && in.Fraction <= 1) {
		return refuse("start", "recover_fraction_invalid", "Choose a speed between 1 % and 100 % before pressing")
	}
	speeds, err := in.jogSpeeds()
	if err != nil {
		return refuse("start", "recover_jog_speed_invalid", err.Error())
	}
	journal(RecoverStep{Step: "start", Outcome: "done", Detail: fmt.Sprintf("fraction=%.2f confirm_arm=%t", in.Fraction, in.ConfirmArm)})
	if speeds != nil {
		journal(RecoverStep{Step: "speed", Outcome: "done", Detail: fmt.Sprintf("TCP limits %.3f mm/s and %.3f deg/s (selected Jog preset)", speeds.linearMMs*in.Fraction, speeds.angularDegs*in.Fraction)})
	}
	tried := map[string]bool{}
	// Each step runs at most once per press; the bound is the number of steps.
	for range 6 {
		facts, joints, err := r.recoverFacts(ctx)
		if err != nil {
			return refuse("status", "status_unavailable", "No fresh status from the servo host ("+err.Error()+"); recovery stops here")
		}
		facts.Tried = tried
		facts.ArmConfirmed = in.ConfirmArm
		result.Joints = joints
		decision := nextRecoverStep(facts)
		if speeds != nil && (decision.Action == "confirm_arm" || decision.Action == "arm" || decision.Action == "move") {
			// Do not arm for a return whose TCP limits cannot be established.
			// The actual speed/path is still calculated from fresh held targets
			// immediately before trajectory preparation.
			r.mu.Lock()
			geometry := r.robotGeometryLocked()
			d := r.description
			r.mu.Unlock()
			speedErr := returnSpeedPreflight(geometry.Kinematics, d, r.cfg.AxisMask, joints, in.Fraction, *speeds, r.cfg.MoveJerk)
			if speedErr != nil {
				code, detail := denseErrorWords(speedErr)
				return refuse("speed", code, detail)
			}
		}
		tried[decision.Action] = true
		switch decision.Action {
		case "refuse":
			return refuse(decision.Step, decision.Code, decision.Detail)
		case "confirm_stop", "reset":
			if refusal := r.settleStep(ctx, decision, facts, journal); refusal != nil {
				return refuse(refusal.Step, refusal.Code, refusal.Detail)
			}
		case "at_zero":
			journal(RecoverStep{Step: "move", Outcome: "done", Detail: "every joint is already at 0°"})
			result.State, result.Status = "at_zero", r.Status(context.Background())
			return result, nil
		case "confirm_arm":
			journal(RecoverStep{Step: "arm", Outcome: "needs_confirmation", Detail: recoverJointsText(joints)})
			result.State, result.Status = "confirm_arm", r.Status(context.Background())
			return result, nil
		case "arm":
			if stale := recoverConfirmationStale(in.Confirmed, joints); stale != "" {
				return refuse("arm", "confirmation_stale", stale)
			}
			if _, err := r.Arm(ctx, true); err != nil {
				code, detail := denseErrorWords(err)
				return refuse("arm", code, "Arm refused: "+detail)
			}
			journal(RecoverStep{Step: "arm", Outcome: "done", Detail: "operator confirmed: " + recoverJointsText(joints)})
		case "move":
			state, err := r.recoverMove(ctx, joints, in.Fraction, speeds)
			result.Move = &state
			if err != nil {
				code, detail := denseErrorWords(err)
				return refuse("move", code, "Move to zero stopped: "+detail)
			}
			journal(RecoverStep{Step: "move", Outcome: "done", Detail: recoverJointsText(joints)})
			result.State, result.Status = "done", r.Status(context.Background())
			return result, nil
		}
	}
	return refuse("start", "recover_incomplete", "Recovery did not reach a decision; press again")
}

// stepJournal records each step of the recovery press or Arm (action) in
// the answer and as one journal line naming the client that pressed it.
func stepJournal(ctx context.Context, action string, steps *[]RecoverStep) func(RecoverStep) {
	client := ClientOf(ctx).Label
	if client == "" {
		client = "unnamed client"
	}
	return func(step RecoverStep) {
		*steps = append(*steps, step)
		log.Printf("dense-execution: %s client=%s step=%s outcome=%s detail=%s", action, client, step.Step, step.Outcome, step.Detail)
	}
}

// settleStep runs one confirm_stop or reset decision for the recovery press
// or Arm, journaled with the joints, faults and drive alarms it concerned.
// It answers the refusal that ends the press, or nil to read fresh facts and
// decide again: the step cleared, or native refused the reset fault_persists
// and the next decision names the drive alarm still active.
func (r *RTCoreController) settleStep(ctx context.Context, d recoverDecision, f recoverFacts, journal func(RecoverStep)) *recoverDecision {
	switch d.Action {
	case "confirm_stop":
		if why := r.confirmLatchedStop(ctx); why != "" {
			refusal := refuseRecover("confirm_stop", "stop_unconfirmed", "The last Stop is still unconfirmed: "+why+". Press STOP, or use the hardware stop")
			return &refusal
		}
		journal(RecoverStep{Step: "confirm_stop", Outcome: "done", Detail: "fresh status: disarmed, nothing enabled, no lease, no grant"})
	case "reset":
		subject := resetSubject(f)
		if _, err := r.resetFault(ctx, false); err != nil {
			code, detail := denseErrorWords(err)
			if code == "fault_persists" {
				journal(RecoverStep{Step: "reset", Outcome: "persists", Detail: subject + "; " + detail})
				return nil
			}
			refusal := refuseRecover("reset", code, "Reset refused: "+detail)
			return &refusal
		}
		journal(RecoverStep{Step: "reset", Outcome: "done", Detail: subject + "; native faults cleared"})
	}
	return nil
}

// resetSubject is the journal's account of what a reset was sent for.
func resetSubject(f recoverFacts) string {
	var faults, alarms []string
	axes := map[string]bool{}
	for _, fault := range f.Faults {
		faults = append(faults, fault.Name+"("+fault.Recovery+")")
		for _, axis := range fault.Axes {
			axes[axis] = true
		}
	}
	for _, alarm := range f.Alarms {
		what := alarm.Axis + " " + alarm.Code
		if alarm.Text != "" {
			what += " (" + alarm.Text + ")"
		}
		alarms = append(alarms, what)
		axes[alarm.Axis] = true
	}
	names := make([]string, 0, len(axes))
	for axis := range axes {
		names = append(names, axis)
	}
	slices.Sort(names)
	if len(alarms) == 0 {
		alarms = []string{"none"}
	}
	return fmt.Sprintf("axes=%s faults=%s alarms=%s", strings.Join(names, ","), strings.Join(faults, ","), strings.Join(alarms, "; "))
}

// recoverFacts reads one fresh native status and this server's own state.
// joints lists every described rotary joint in the configured mask.
func (r *RTCoreController) recoverFacts(ctx context.Context) (recoverFacts, []RecoverJoint, error) {
	if _, err := r.describe(ctx, true); err != nil {
		return recoverFacts{}, nil, err
	}
	return r.statusFacts(ctx)
}

// statusFacts is recoverFacts without discovery, for Arm: a cold Arm must not
// hold the discovery lock (and with it status) before it queues. Joints are
// named from the cached description, else J1..Jn; faults and drive alarms
// need no description.
func (r *RTCoreController) statusFacts(ctx context.Context) (recoverFacts, []RecoverJoint, error) {
	s, err := rtRequest(r, ctx, "rt_core_status_slow", r.c.Status)
	if err != nil {
		return recoverFacts{}, nil, err
	}
	if s.TimeNS == 0 || s.AdapterIncarnation == "" || s.DaemonIncarnation == "" {
		return recoverFacts{}, nil, rtRefused("rt_core_status_unavailable")
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.held && s.AdapterIncarnation == r.adapter && s.DaemonIncarnation == r.daemon && (r.observation == nil || s.TimeNS >= r.observation.TimeNS) {
		r.setObservationLocked(&s)
	}
	var f recoverFacts
	switch {
	case r.move != nil:
		f.Busy = "A joint move"
	case r.jog != nil:
		f.Busy = "A jog"
	case r.session != nil && r.session.State == "playing":
		f.Busy = "A program"
	case r.loop != nil:
		f.Busy = "A program loop"
	case r.opCancel != nil:
		f.Busy = "Another robot request"
	}
	f.StopLatched = r.stopUncertain || r.inhibited
	grantActive := s.Grant != nil && s.Grant.GrantActive != 0
	f.ForeignHolder = !r.held && !f.StopLatched && (s.Core.LeaseValid != 0 || grantActive || s.Core.Armed != 0)
	f.Armed = r.held && s.Core.Armed == 1
	described := r.description.Axes
	name := func(i int) string {
		if i < len(described) && described[i].ID != "" {
			return described[i].ID
		}
		return fmt.Sprintf("J%d", i+1)
	}
	var ids []string
	var indices []int
	var positions []float64
	for i := range described {
		if r.cfg.AxisMask&(1<<uint(i)) == 0 || described[i].PositionUnit != "rad" {
			continue
		}
		if s.Core.HomeValidMask&(1<<uint(i)) == 0 {
			f.HomeInvalid = append(f.HomeInvalid, name(i))
		}
		if i >= len(s.Axes) || !s.Axes[i].LogicalValid || math.IsNaN(s.Axes[i].LogicalPosition) || math.IsInf(s.Axes[i].LogicalPosition, 0) {
			f.PositionLost = append(f.PositionLost, name(i))
			continue
		}
		ids = append(ids, name(i))
		indices = append(indices, i)
		positions = append(positions, s.Axes[i].LogicalPosition)
	}
	joints := recoverJoints(ids, positions)
	for j := range joints {
		joints[j].Axis = indices[j]
	}
	f.AtZero = len(joints) > 0
	for _, joint := range joints {
		if joint.Moves {
			f.AtZero = false
		}
	}
	if s.Core.Recovery != nil {
		for _, fault := range s.Core.Recovery.Faults {
			var axes, rehome []string
			for i := range 32 {
				if fault.AxisMask&(1<<uint(i)) != 0 {
					axes = append(axes, name(i))
				}
				if fault.RehomeAxisMask&(1<<uint(i)) != 0 {
					rehome = append(rehome, name(i))
				}
			}
			f.Faults = append(f.Faults, recoverFault{Name: fault.Name, Axes: axes, Recovery: fault.Recovery, Rehome: rehome, ResetPreservesHome: fault.ResetPreservesHome})
		}
	}
	if len(f.Faults) == 0 && (s.Core.SafetyFaultMask != 0 || s.Core.ExecutionFaultReasons != 0) {
		// A latched fault the native recovery table does not classify: never
		// guessed resettable.
		f.Faults = append(f.Faults, recoverFault{Name: fmt.Sprintf("fault mask %d/%d", s.Core.SafetyFaultMask, s.Core.ExecutionFaultReasons), Recovery: "unknown"})
	}
	for i, axis := range s.Axes {
		if axis.Readiness != "faulted" {
			continue
		}
		code := strings.TrimSpace(axis.DriveAlarm)
		if code == "" {
			code = "(undecoded)"
		}
		f.Alarms = append(f.Alarms, recoverAlarm{Axis: name(i), Code: code, Text: strings.Join(strings.Fields(axis.DriveAlarmText), " ")})
	}
	return f, joints, nil
}

// confirmLatchedStop settles an unconfirmed Stop (or an unconfirmed Release
// after an acknowledged Stop) by the rule Stop itself now uses: one fresh
// native observation with the reset admission's freshness and identity proof
// shows disarmed, nothing enabled, no Home, trajectory or jog running, no
// lease and no grant, and no acquisition of unknown outcome may have issued
// a grant since (Stop's own rule). It sends nothing to the robot. Answers ""
// when settled, else why not.
func (r *RTCoreController) confirmLatchedStop(ctx context.Context) string {
	if !r.opMu.TryLock() {
		return "another robot request is running"
	}
	defer r.opMu.Unlock()
	if err := r.stopMu.LockContext(ctx); err != nil {
		return "the Stop lane is busy (" + err.Error() + ")"
	}
	defer r.stopMu.Unlock()
	r.mu.Lock()
	acquiring := r.acquiring
	r.mu.Unlock()
	if acquiring {
		return "an acquisition whose outcome is unknown may have issued a grant"
	}
	s, err := r.resetRead(ctx, nil)
	switch {
	case err != nil:
		var rejected *control.Rejected
		if errors.As(err, &rejected) && strings.HasPrefix(rejected.Reason, "reset_requires_inhibited") {
			return "the servo host reports the robot armed, enabled or moving"
		}
		return "no fresh status proves it (" + err.Error() + ")"
	case s.Core.LeaseValid != 0 || s.Grant == nil || s.Grant.GrantActive != 0:
		return "the servo host still shows a lease or grant"
	}
	if client, ok := r.c.(interface{ SetFence(control.Fence) }); ok {
		client.SetFence(control.Fence{})
	}
	r.mu.Lock()
	r.stopUncertain, r.inhibited = false, false
	r.reason = ""
	r.fence = control.Fence{}
	r.acquiring = false
	r.feedbackPending = true
	r.feedbackReceipt++
	r.mu.Unlock()
	log.Printf("dense-execution: stop confirmed by observation: disarmed, no axis enabled, no lease and no grant")
	return ""
}

// recoverMove is All joints to 0°: every joint off zero to exactly 0 rad in
// one JointMove (absolute endpoints from fresh native held state), the same
// prepared trajectory and shared progress the joint table's button runs.
func (r *RTCoreController) recoverMove(ctx context.Context, joints []RecoverJoint, fraction float64, speeds *returnJogSpeeds) (JointMoveState, error) {
	var axes []JointMoveAxis
	for _, joint := range joints {
		if joint.Moves {
			zero := 0.0
			axes = append(axes, JointMoveAxis{Axis: joint.Axis, PositionRad: &zero})
		}
	}
	r.mu.Lock()
	in := JointMoveIntent{TargetID: r.cfg.Binding.PairID, Revision: r.jogRevision, Fraction: fraction, Axes: axes, returnSpeeds: speeds}
	if r.session != nil {
		in.SessionID = r.session.ID
	}
	r.mu.Unlock()
	return r.JointMove(ctx, in)
}

// denseErrorWords is a typed failure's code and detail.
func denseErrorWords(err error) (string, string) {
	var failure *Error
	if errors.As(err, &failure) {
		detail := failure.Detail
		if detail == "" {
			detail = failure.Code
		}
		return failure.Code, detail
	}
	return "rt_core_failure", err.Error()
}

// recoverJointsText is the journal's one-line form of the confirmation:
// the joints that move; the rest stay within the zero tolerance.
func recoverJointsText(joints []RecoverJoint) string {
	var parts []string
	stay := 0
	for _, joint := range joints {
		if !joint.Moves {
			stay++
			continue
		}
		parts = append(parts, fmt.Sprintf("%s %.2f°->0° (delta %+.2f°)", joint.ID, joint.FromDeg, joint.DeltaDeg))
	}
	if stay > 0 {
		parts = append(parts, fmt.Sprintf("%d joint(s) already at 0°", stay))
	}
	return strings.Join(parts, ", ")
}
