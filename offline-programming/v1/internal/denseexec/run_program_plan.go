package denseexec

import (
	"context"
	"fmt"
	"math"
	"strings"

	"rosieos/offline-programming/v1/internal/densejointtraj"
)

// Run program is the owner's one press for a planned program (2026-10-11:
// "why do I need to load the plan to the robot if it's already on it?" and
// "after it's run it makes me click Load to robot again"). After a program
// completes native retires it and the arm stands at the program's end, so a
// replay used to need Go to plan start, Load to robot and Play by hand. One
// press now runs, in order and only as far as each step clears:
//
//  1. the plan's bytes are fetched and the checks Load makes without the
//     robot (identity, the robot it was planned for, process outputs) run
//     before anything moves: a plan Load would refuse moves nothing;
//  2. the Stop confirmation and fault reset Arm and recover-to-zero share
//     (settleControlStep, settleFaultStep), while the robot is disarmed;
//  3. one confirmation when the robot must arm or move: "Arm and run",
//     "Move to program start and run" or both, with every joint's angle, the
//     program's first sample and the travel; the joint-space line may sweep
//     the tool, as All joints to 0° says;
//  4. Arm (which itself clears resettable faults first, 4ed9e029);
//  5. Go to plan start, when any joint is outside its completion_tolerance
//     of the first sample: the existing jerk-limited JointMove with its
//     shared progress;
//  6. Load, unless this plan is already staged in this run mode and the arm
//     has not moved since; native preparation checks limits and identity
//     again and a refusal stops the press;
//  7. Play.
//
// It stops at the first step that does not clear and says why in the
// server's sentence. A Stop at any point ends the press: every operation it
// runs is fenced by the press's Stop epoch. It never homes, never resets a
// fault whose recovery is not a reset and never retries a step that failed in
// the same press.
const (
	RunProgramArmAction     = "Arm and run"
	RunProgramMoveAction    = "Move to program start and run"
	RunProgramArmMoveAction = "Arm, move to program start and run"
)

// Unverified: travel since the confirmation beyond recoverConfirmToleranceDeg
// means the pose the operator approved is not the pose that would move; a
// program start that moved by more than this is another plan.
const runStartConfirmToleranceDeg = 0.1

// RunProgramRequest is POST .../dense-execution/run-program: the Load body
// for the kept plan, the Jog speed for Go to plan start, and on the second
// press the confirmation the first press asked for, with the joints the
// operator saw.
type RunProgramRequest struct {
	LoadRequest
	Fraction    float64        `json:"fraction"`
	ConfirmMove bool           `json:"confirm_move"`
	ConfirmArm  bool           `json:"confirm_arm"`
	Confirmed   []RecoverJoint `json:"confirmed,omitempty"`
	// Repeat is "" or "once" (one run), "count" (Count cycles) or "loop"
	// (until stopped or finished); ConfirmLoop answers the one question a
	// repeating press asks before its first cycle (run_loop.go).
	Repeat      string `json:"repeat,omitempty"`
	Count       int    `json:"count,omitempty"`
	ConfirmLoop bool   `json:"confirm_loop,omitempty"`
	// Unattended asks for a loop that does not need the starting client's
	// heartbeat; refused unless the machine allows it (owner decision,
	// RTCoreConfig.UnattendedLoopAllowed, default false). On a machine that
	// allows it every loop is unattended, asked for or not (run_loop.go).
	Unattended bool `json:"unattended,omitempty"`
	// Continue is the server's own check before every further cycle (the
	// maintenance notice): a non-empty answer ends the loop at the program
	// start with that sentence. Set by the HTTP handler, never by a client.
	Continue func() string `json:"-"`
}

// RunProgramResult: State "playing" (started), "confirm" (the operator must
// confirm Action: NeedsArm, NeedsMove) or "refused". Joints carry each
// joint's angle, the program's first sample (ToDeg) and the travel.
type RunProgramResult struct {
	State     string          `json:"state"`
	Action    string          `json:"action"`
	NeedsArm  bool            `json:"needs_arm,omitempty"`
	NeedsMove bool            `json:"needs_move,omitempty"`
	Steps     []RecoverStep   `json:"steps"`
	Joints    []RecoverJoint  `json:"joints,omitempty"`
	Refusal   *RecoverRefusal `json:"refusal,omitempty"`
	Move      *JointMoveState `json:"move,omitempty"`
	SessionID string          `json:"session_id,omitempty"`
	Notes     []string        `json:"notes,omitempty"`
	// Loop is the repeat this press asks about (confirm) or started (playing).
	Loop   *LoopPlan   `json:"loop,omitempty"`
	Status *StatusBody `json:"status,omitempty"`
}

// LoopPlan is what the loop question shows: the repeat, the plan's planned
// return to its start (absent when the program already ends there) and the
// running loop once started.
type LoopPlan struct {
	Mode       string      `json:"mode"`
	Count      int         `json:"count,omitempty"`
	ReturnMove *ReturnMove `json:"return_move,omitempty"`
	Running    *LoopStatus `json:"running,omitempty"`
	// Unattended: the machine allows unattended loops, so this one will keep
	// running without its starter (run_loop.go); the question says so.
	Unattended bool `json:"unattended,omitempty"`
}

// ReturnMove is the planner's loop-return segment (a Home to the plan's first
// configuration, collision-checked like every planned leg): its duration and
// each joint's travel.
type ReturnMove struct {
	DurationS float64        `json:"duration_s"`
	Joints    []RecoverJoint `json:"joints"`
}

// LoopStatus is a running or the last ended loop, in every status body.
type LoopStatus struct {
	Mode             string  `json:"mode"`
	Count            int     `json:"count,omitempty"`
	Cycles           int     `json:"cycles"`
	State            string  `json:"state"` // running, finishing, ended
	CycleElapsedS    float64 `json:"cycle_elapsed_s,omitempty"`
	LastCycleS       float64 `json:"last_cycle_s,omitempty"`
	Owner            string  `json:"owner,omitempty"`
	Attended         bool    `json:"attended"`
	ReturnMove       bool    `json:"return_move"`
	EndReason        string  `json:"end_reason,omitempty"`
	TrajectoryDigest string  `json:"trajectory_digest,omitempty"`
}

// RunProgramBackend is a backend that runs the one-press program run.
type RunProgramBackend interface {
	RunProgram(context.Context, RunProgramRequest) (*RunProgramResult, error)
}

// runFacts is everything one Run program decision reads: the recovery's
// facts from one fresh status, whether the arm is at the program's first
// sample, whether this plan is staged, and what this press confirmed and
// already did.
type runFacts struct {
	recoverFacts
	AtStart       bool
	Loaded        bool
	MoveConfirmed bool
	Repeat        bool
	LoopConfirmed bool
}

var runWords = settleWords{Actor: "Run program", Stops: "nothing moved", Again: "press Run program again"}

// runActionFor names the confirmation for what the press must do.
func runActionFor(arm, move bool) string {
	switch {
	case arm && move:
		return RunProgramArmMoveAction
	case move:
		return RunProgramMoveAction
	}
	return RunProgramArmAction
}

// loopActionFor names a repeating press's one question: what it must do
// first, then the repeat ("Arm, move to program start and loop until stopped").
func loopActionFor(arm, move bool, mode string, count int) string {
	repeat := "loop until stopped"
	if mode == "count" {
		repeat = fmt.Sprintf("run %d times", count)
	}
	switch {
	case arm && move:
		return "Arm, move to program start and " + repeat
	case move:
		return "Move to program start and " + repeat
	case arm:
		return "Arm and " + repeat
	}
	return strings.ToUpper(repeat[:1]) + repeat[1:]
}

func repeatWord(mode string) string {
	if mode == "" {
		return "once"
	}
	return mode
}

// Unverified bound on Run N times: a count past this is a loop.
const maxLoopCount = 10000

// LoopReturnSourceID is the dense segment the planner makes for the program's
// loop-return node (offline-programming ui rails/planRequest.ts appends it,
// weld_planner taught.py plans it as a Home to the plan's first configuration).
const LoopReturnSourceID = "rosie-loop-return"

// repeatOf validates the repeat a press asks for: mode "" (one run), "count"
// or "loop", and the count.
func repeatOf(in RunProgramRequest) (string, int, string) {
	switch in.Repeat {
	case "", "once":
		return "", 0, ""
	case "count":
		if in.Count < 1 || in.Count > maxLoopCount {
			return "", 0, fmt.Sprintf("Choose between 1 and %d cycles", maxLoopCount)
		}
		if in.Count == 1 {
			return "", 0, ""
		}
		return "count", in.Count, ""
	case "loop":
		return "loop", 0, ""
	}
	return "", 0, `repeat must be "once", "count" or "loop"`
}

// loopClosure checks that a plan ends where it starts: every described rotary
// axis of the last sample within its completion_tolerance of the first, so the
// next cycle's Play admits it without a move. It names the joints that do not
// close, or "".
func loopClosure(plan *densejointtraj.Plan, ids []string, mask uint32, tolerances []float64) string {
	if len(plan.Segments) == 0 {
		return "the plan has no samples"
	}
	first := plan.Segments[0].Samples[0].QRad
	lastSegment := plan.Segments[len(plan.Segments)-1]
	last := lastSegment.Samples[len(lastSegment.Samples)-1].QRad
	var open []string
	for i := range tolerances {
		if i >= len(first) || mask&(1<<uint(i)) == 0 || !(tolerances[i] > 0) {
			continue
		}
		if d := math.Abs(last[i] - first[i]); !(d <= tolerances[i]) {
			name := fmt.Sprintf("J%d", i+1)
			if i < len(ids) && ids[i] != "" {
				name = ids[i]
			}
			open = append(open, fmt.Sprintf("%s %.2f°", name, d*180/math.Pi))
		}
	}
	return strings.Join(open, ", ")
}

// returnMoveOf is the plan's loop-return segment, or nil when the planner
// made none (the program already ends at its start).
func returnMoveOf(plan *densejointtraj.Plan, ids []string, mask uint32) *ReturnMove {
	for _, segment := range plan.Segments {
		if segment.SourceID != LoopReturnSourceID || len(segment.Samples) == 0 {
			continue
		}
		from, to := segment.Samples[0].QRad, segment.Samples[len(segment.Samples)-1].QRad
		move := &ReturnMove{DurationS: segment.Samples[len(segment.Samples)-1].TimeS - segment.Samples[0].TimeS}
		for i := range from {
			if mask&(1<<uint(i)) == 0 || i >= len(ids) {
				continue
			}
			f, t := from[i]*180/math.Pi, to[i]*180/math.Pi
			move.Joints = append(move.Joints, RecoverJoint{Axis: i, ID: ids[i], FromDeg: f, ToDeg: t, DeltaDeg: t - f, Moves: math.Abs(t-f) > 1e-6})
		}
		return move
	}
	return nil
}

// nextRunStep is the sequence above as a pure decision over one set of
// facts; the controller re-reads the facts after every step it takes.
// Action: "confirm_stop", "reset", "confirm", "arm", "move", "load", "play"
// or "refuse".
func nextRunStep(f runFacts) recoverDecision {
	if d, ok := settleControlStep(f.recoverFacts, runWords); ok {
		return d
	}
	switch {
	case len(f.HomeInvalid) > 0:
		return refuseRecover("home", "home_invalid", fmt.Sprintf("Home is not valid on %s. Run program never re-homes: an operator runs Home with each joint at its mechanical home pose", strings.Join(f.HomeInvalid, ", ")))
	case len(f.PositionLost) > 0:
		return refuseRecover("position", "position_unknown", fmt.Sprintf("The servo host reports no valid position for %s; nothing moved", strings.Join(f.PositionLost, ", ")))
	}
	if !f.Armed {
		if d, ok := settleFaultStep(f.recoverFacts, runWords); ok {
			return d
		}
	}
	needArm, needMove := !f.Armed, !f.AtStart
	if (needMove && !f.MoveConfirmed) || (needArm && !f.ArmConfirmed) || (f.Repeat && !f.LoopConfirmed) {
		return recoverDecision{Action: "confirm", Step: "confirm"}
	}
	if needMove {
		if f.Tried["move"] {
			return refuseRecover("move", "start_not_reached", "The move to the program's start ended outside its completion tolerance; Play stays refused. Press Run program again to see the angles")
		}
		if needArm {
			if f.Tried["arm"] {
				return refuseRecover("arm", "arm_unconfirmed", "Arm was accepted but the machine does not report armed; nothing moved")
			}
			return recoverDecision{Action: "arm", Step: "arm"}
		}
		return recoverDecision{Action: "move", Step: "move"}
	}
	// A program staged before this press moved the arm is staged again: the
	// prepared program and the pose it starts from are one decision.
	if !f.Loaded || (f.Tried["move"] && !f.Tried["load"]) {
		if f.Tried["load"] {
			return refuseRecover("load", "load_unconfirmed", "Load was accepted but the program is not staged; press Run program again")
		}
		return recoverDecision{Action: "load", Step: "load"}
	}
	if needArm {
		if f.Tried["arm"] {
			return refuseRecover("arm", "arm_unconfirmed", "Arm was accepted but the machine does not report armed; the program did not start")
		}
		return recoverDecision{Action: "arm", Step: "arm"}
	}
	if f.Tried["play"] {
		return refuseRecover("play", "play_unconfirmed", "Play was accepted but the program is not running; press Run program again")
	}
	return recoverDecision{Action: "play", Step: "play"}
}

// runJoints is every rotary joint's angle, the program's first sample and the
// travel; a joint within its completion tolerance (radians) of the start stays.
// joints are statusFacts' joints (FromDeg, Axis); start is the first sample by
// axis index. ok false names the first joint the plan or the description
// cannot place.
func runJoints(joints []RecoverJoint, start []float64, tolerances []float64) ([]RecoverJoint, string) {
	out := make([]RecoverJoint, 0, len(joints))
	for _, joint := range joints {
		if joint.Axis >= len(start) || joint.Axis >= len(tolerances) || math.IsNaN(start[joint.Axis]) || math.IsInf(start[joint.Axis], 0) {
			return nil, joint.ID + " has no position in the plan's first sample"
		}
		tolerance := tolerances[joint.Axis]
		if !(tolerance > 0) || math.IsInf(tolerance, 0) {
			return nil, joint.ID + " describes no completion_tolerance to check the program's start against"
		}
		to := start[joint.Axis] * 180 / math.Pi
		delta := to - joint.FromDeg
		out = append(out, RecoverJoint{Axis: joint.Axis, ID: joint.ID, FromDeg: joint.FromDeg, ToDeg: to, DeltaDeg: delta, Moves: math.Abs(delta) > tolerance*180/math.Pi})
	}
	return out, ""
}

// runConfirmationStale names what changed since the operator confirmed: the
// joints' angles (recoverConfirmationStale) or the program's start, else "".
func runConfirmationStale(confirmed, now []RecoverJoint) string {
	if stale := recoverConfirmationStale(confirmed, now); stale != "" {
		return strings.Replace(stale, "press again", "press Run program again", 1)
	}
	byAxis := map[int]RecoverJoint{}
	for _, joint := range confirmed {
		byAxis[joint.Axis] = joint
	}
	var moved []string
	for _, joint := range now {
		if seen := byAxis[joint.Axis]; joint.Moves && !(math.Abs(seen.ToDeg-joint.ToDeg) <= runStartConfirmToleranceDeg) {
			moved = append(moved, joint.ID)
		}
	}
	if len(moved) == 0 {
		return ""
	}
	return "The program's start changed since the confirmation (" + strings.Join(moved, ", ") + "); press Run program again to see the new start"
}

// runJointsText is the journal's one-line form of the confirmation.
func runJointsText(joints []RecoverJoint) string {
	var parts []string
	stay := 0
	for _, joint := range joints {
		if !joint.Moves {
			stay++
			continue
		}
		parts = append(parts, fmt.Sprintf("%s %.2f°->%.2f° (delta %+.2f°)", joint.ID, joint.FromDeg, joint.ToDeg, joint.DeltaDeg))
	}
	if stay > 0 {
		parts = append(parts, fmt.Sprintf("%d joint(s) at the program's start", stay))
	}
	return strings.Join(parts, ", ")
}
