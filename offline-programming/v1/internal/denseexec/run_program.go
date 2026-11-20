//go:build linux

package denseexec

import (
	"context"
	"errors"
	"fmt"
	"strings"

	"rosieos/offline-programming/v1/internal/densejointtraj"
)

// RunProgram runs the one-press program run (run_program_plan.go) for the
// client that pressed it; the HTTP gates have already admitted that client
// as the controller, refused it during a maintenance notice and fenced it to
// the displayed cell. Every step is one journal line naming the client.
// The error is the refusal (typed, with the result) or a transport failure.
func (r *RTCoreController) RunProgram(ctx context.Context, in RunProgramRequest) (*RunProgramResult, error) {
	result := &RunProgramResult{Steps: []RecoverStep{}}
	journal := stepJournal(ctx, "run-program", &result.Steps)
	r.mu.Lock()
	pressEpoch := r.epoch
	r.mu.Unlock()
	// Every operation of this press -- Arm, Load, Play -- is fenced by the
	// epoch it started under: a Stop anywhere in the press refuses the rest.
	ctx = context.WithValue(ctx, pressEpochKey{}, pressEpoch)
	// stopped: an operator's STOP or DISARM since the press began. A step's
	// own failure also stops the robot (a new epoch), and keeps its words.
	stopped := func() bool {
		r.mu.Lock()
		defer r.mu.Unlock()
		cause, _, _ := strings.Cut(r.reason, ":")
		return r.epoch != pressEpoch && (cause == "operator_stop" || cause == "stop" || cause == "disarm")
	}
	refuse := func(step, code, detail string) (*RunProgramResult, error) {
		if stopped() && code != "run_stopped" {
			code, detail = "run_stopped", "STOP ended Run program during "+step+"; press Run program to start again ("+detail+")"
		}
		journal(RecoverStep{Step: step, Outcome: "refused", Detail: code + ": " + detail})
		result.State, result.Refusal = "refused", &RecoverRefusal{Step: step, Code: code, Detail: detail}
		result.Status = r.Status(context.Background())
		return result, httpError(409, code, "%s", detail)
	}
	if !(in.Fraction > 0 && in.Fraction <= 1) {
		return refuse("start", "run_fraction_invalid", "Choose a speed between 1 % and 100 % before pressing")
	}
	if in.Source != "" {
		return refuse("start", "run_source_invalid", "Run program plays the kept plan; a test program has its own Play")
	}
	mode, count, why := repeatOf(in)
	if why != "" {
		return refuse("start", "run_repeat_invalid", why)
	}
	if mode != "" && in.Unattended && !r.cfg.UnattendedLoopAllowed {
		return refuse("start", "unattended_loop_not_allowed", "Unattended loops are not allowed on this cell (cell manifest cell_deploy.olp.unattended_loop_allowed, owner decision); a loop runs only while the client that started it stays connected")
	}
	journal(RecoverStep{Step: "start", Outcome: "done", Detail: fmt.Sprintf("plan=%s trajectory=%s dry_run=%t fraction=%.2f confirm_arm=%t confirm_move=%t repeat=%s count=%d confirm_loop=%t",
		in.PlanID, in.TrajectoryDigest, in.DryRun, in.Fraction, in.ConfirmArm, in.ConfirmMove, repeatWord(mode), count, in.ConfirmLoop)})

	fetched, err := r.fetchForLoad(ctx, in.LoadRequest)
	if err != nil {
		code, detail := denseErrorWords(err)
		return refuse("load", code, "Load refused: "+detail)
	}
	plan, err := r.runPreflight(ctx, in.LoadRequest, fetched)
	var start []float64
	if err == nil {
		start = append([]float64(nil), plan.Segments[0].Samples[0].QRad[:]...)
	}
	if err != nil {
		var typed *Error
		if !errors.As(err, &typed) {
			err = rtFailure(err)
		}
		code, detail := denseErrorWords(err)
		return refuse("load", code, "Load refused: "+detail)
	}
	result.Notes = fetched.notes
	if mode != "" {
		ids, tolerances := r.axisFacts()
		if open := loopClosure(plan, ids, r.cfg.AxisMask, tolerances); open != "" {
			return refuse("loop", "loop_not_closed", "This plan ends away from its start ("+open+"): a loop needs the planned return move. Plan it again with the repeat selected; the planner adds a collision-checked return to the start")
		}
		result.Loop = &LoopPlan{Mode: mode, Count: count, ReturnMove: returnMoveOf(plan, ids, r.cfg.AxisMask), Unattended: r.cfg.UnattendedLoopAllowed}
	}

	tried := map[string]bool{}
	// Each step runs at most once per press; the bound is the number of steps.
	for range 10 {
		if stopped() {
			return refuse("stop", "run_stopped", "STOP ended Run program; press Run program to start again")
		}
		facts, joints, err := r.runFacts(ctx, in, start)
		if err != nil {
			code, detail := denseErrorWords(err)
			if code == "rt_core_failure" {
				return refuse("status", "status_unavailable", "No fresh status from the servo host ("+detail+"); nothing moved")
			}
			return refuse("start", code, detail)
		}
		facts.Tried = tried
		facts.Repeat, facts.LoopConfirmed = mode != "", in.ConfirmLoop
		result.Joints = joints
		decision := nextRunStep(facts)
		tried[decision.Action] = true
		switch decision.Action {
		case "refuse":
			return refuse(decision.Step, decision.Code, decision.Detail)
		case "confirm_stop", "reset":
			if refusal := r.settleStep(ctx, decision, facts.recoverFacts, journal); refusal != nil {
				return refuse(refusal.Step, refusal.Code, refusal.Detail)
			}
		case "confirm":
			result.NeedsArm, result.NeedsMove = !facts.Armed, !facts.AtStart
			result.Action = runActionFor(result.NeedsArm, result.NeedsMove)
			if mode != "" {
				result.Action = loopActionFor(result.NeedsArm, result.NeedsMove, mode, count)
			}
			journal(RecoverStep{Step: "confirm", Outcome: "needs_confirmation", Detail: result.Action + ": " + runJointsText(joints)})
			result.State, result.Status = "confirm", r.Status(context.Background())
			return result, nil
		case "arm":
			if stale := runConfirmationStale(in.Confirmed, joints); stale != "" {
				return refuse("arm", "confirmation_stale", stale)
			}
			armed, err := r.Arm(ctx, true)
			if armed != nil {
				result.Steps = append(result.Steps, armed.Steps...)
			}
			if err != nil {
				code, detail := denseErrorWords(err)
				return refuse("arm", code, "Arm refused: "+detail)
			}
			journal(RecoverStep{Step: "arm", Outcome: "done", Detail: "operator confirmed: " + runJointsText(joints)})
		case "move":
			if stale := runConfirmationStale(in.Confirmed, joints); stale != "" {
				return refuse("move", "confirmation_stale", stale)
			}
			state, err := r.runMove(ctx, joints, start, in.Fraction)
			result.Move = &state
			if err != nil {
				code, detail := denseErrorWords(err)
				return refuse("move", code, "Go to plan start stopped: "+detail)
			}
			journal(RecoverStep{Step: "move", Outcome: "done", Detail: runJointsText(joints)})
		case "load":
			loaded, err := r.loadFetched(ctx, in.LoadRequest, fetched)
			if err != nil {
				code, detail := denseErrorWords(err)
				return refuse("load", code, "Load refused: "+detail)
			}
			result.SessionID = loaded.SessionID
			result.Notes = loaded.Notes
			journal(RecoverStep{Step: "load", Outcome: "done", Detail: fmt.Sprintf("session=%s source=%s", loaded.SessionID, loaded.BlobSource)})
		case "play":
			r.mu.Lock()
			session := r.session.ID
			r.mu.Unlock()
			if _, err := r.Play(ctx, session); err != nil {
				code, detail := denseErrorWords(err)
				return refuse("play", code, "Play refused: "+detail)
			}
			journal(RecoverStep{Step: "play", Outcome: "done", Detail: "session=" + session})
			result.SessionID = session
			if mode != "" {
				result.Loop.Running = r.startLoop(ctx, in, fetched, mode, count, result.Loop.ReturnMove != nil, pressEpoch)
			}
			result.State, result.Status = "playing", r.Status(context.Background())
			return result, nil
		}
	}
	return refuse("start", "run_incomplete", "Run program did not reach a decision; press it again")
}

// runPreflight is every check Load makes without the robot, run before
// anything moves: the bytes decode, they are the plan the press names, the
// plan was made for the robot this machine is, and its process outputs are
// allowed in this run mode. It answers the decoded plan.
// Load runs all of them again, with the machine's own checks after them.
func (r *RTCoreController) runPreflight(ctx context.Context, request LoadRequest, fetched loadBytes) (*densejointtraj.Plan, error) {
	plan, err := densejointtraj.Decode(fetched.blob)
	if err != nil {
		return nil, httpError(400, "dense_trajectory_invalid", "%v", err)
	}
	h := plan.Header
	if headerIdentity(h) != request.Identity {
		return nil, rtRefused("dense_identity_mismatch")
	}
	if _, err := r.robotGate(ctx, h.Robot); err != nil {
		return nil, err
	}
	if _, err := r.loadOutputs(ctx, h, request); err != nil {
		return nil, err
	}
	if len(plan.Segments) == 0 || len(plan.Segments[0].Samples) == 0 {
		return nil, httpError(400, "dense_trajectory_invalid", "the plan has no first sample")
	}
	return plan, nil
}

// axisFacts is each described axis's id and completion_tolerance (radians).
func (r *RTCoreController) axisFacts() ([]string, []float64) {
	r.mu.Lock()
	defer r.mu.Unlock()
	ids := make([]string, len(r.description.Axes))
	tolerances := make([]float64, len(r.description.Axes))
	for i, axis := range r.description.Axes {
		ids[i], tolerances[i] = axis.ID, axis.CompletionTolerance
		if axis.PositionUnit != "rad" {
			tolerances[i] = 0
		}
	}
	return ids, tolerances
}

// runFacts reads one fresh native status (statusFacts, after discovery) and
// adds where the arm is against the program's first sample and whether this
// plan is staged in this run mode on the grant this server holds.
func (r *RTCoreController) runFacts(ctx context.Context, in RunProgramRequest, start []float64) (runFacts, []RecoverJoint, error) {
	base, joints, err := r.recoverFacts(ctx)
	if err != nil {
		return runFacts{}, nil, err
	}
	r.mu.Lock()
	tolerances := make([]float64, len(r.description.Axes))
	for i, axis := range r.description.Axes {
		tolerances[i] = axis.CompletionTolerance
	}
	s := r.session
	loaded := r.held && r.program != nil && s != nil && s.Kind == SessionKindTrajectory && s.State == "loaded" &&
		s.Source == "" && s.TrajectoryDigest == in.TrajectoryDigest && s.PlanID == in.PlanID && s.DryRun == in.DryRun
	r.mu.Unlock()
	placed, why := runJoints(joints, start, tolerances)
	if why != "" && len(base.PositionLost) == 0 {
		return runFacts{}, nil, httpError(409, "run_start_unchecked", "%s; nothing moved", why)
	}
	f := runFacts{recoverFacts: base, Loaded: loaded, MoveConfirmed: in.ConfirmMove}
	f.ArmConfirmed = in.ConfirmArm
	f.AtStart = len(placed) > 0
	for _, joint := range placed {
		if joint.Moves {
			f.AtStart = false
		}
	}
	return f, placed, nil
}

// runMove is Go to plan start: every joint outside its tolerance to exactly
// the plan's first sample in one JointMove (absolute endpoints from fresh
// native held state), the same prepared trajectory and shared progress the
// Go to plan start button runs.
func (r *RTCoreController) runMove(ctx context.Context, joints []RecoverJoint, start []float64, fraction float64) (JointMoveState, error) {
	var axes []JointMoveAxis
	for _, joint := range joints {
		if joint.Moves {
			target := start[joint.Axis]
			axes = append(axes, JointMoveAxis{Axis: joint.Axis, PositionRad: &target})
		}
	}
	r.mu.Lock()
	in := JointMoveIntent{TargetID: r.cfg.Binding.PairID, Revision: r.jogRevision, Fraction: fraction, Axes: axes}
	if r.session != nil {
		in.SessionID = r.session.ID
	}
	r.mu.Unlock()
	return r.JointMove(ctx, in)
}
