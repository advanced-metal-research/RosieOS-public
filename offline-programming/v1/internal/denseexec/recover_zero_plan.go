package denseexec

import (
	"context"
	"fmt"
	"math"
	"strings"
)

// Recover to zero is the owner's one-press recovery (2026-10-11: "we should
// also be able to recover by pressing the return all joints to zero button").
// One press runs, in order and only as far as each step clears:
//
//  1. confirm a Stop OLP holds as unconfirmed, by the 51de1050 rule: one
//     fresh native status shows disarmed, nothing enabled, no lease and no
//     grant (stopConfirmedByObservation);
//  2. Reset, when every latched fault's native recovery is a reset; the
//     native reset decides from fresh evidence whether the condition has
//     cleared, and a refusal names the drive alarm still active;
//  3. Arm, only after the operator confirmed the named action with each
//     joint's angle and travel (the joint-space line may sweep the tool);
//  4. the existing All joints to 0° move: one prepared trajectory, one
//     shared progress, jerk-limited, unchanged.
//
// It stops at the first step that does not clear and says why. It never
// disarms, never homes, never resets a fault whose recovery is not a reset,
// and never retries a step that failed in the same press. Arm runs steps 1
// and 2 the same way, by the same code, before it arms (nextArmStep).
const RecoverToZeroAction = "Arm and move all joints to zero"

// Unverified: a disarmed, braked arm reads within hundredths of a degree
// between two samples; travel beyond this since the confirmation means the
// pose the operator approved is not the pose that would move.
const recoverConfirmToleranceDeg = 0.1

// The joint table's zero (JOINT_ZERO_TOLERANCE_DEG in jointJog.ts): two
// hundredths of a degree, under its two-decimal display. A joint inside it
// stays where it is, as All joints to 0° leaves it.
const recoverZeroToleranceDeg = 0.02

// RecoverToZeroRequest is POST .../dense-execution/recover-to-zero. The first
// press sends ConfirmArm false; when the machine must be armed the answer is
// state "confirm_arm" with the joints, and the operator's confirmation sends
// them back with ConfirmArm true.
type RecoverToZeroRequest struct {
	Fraction   float64        `json:"fraction"`
	ConfirmArm bool           `json:"confirm_arm"`
	Confirmed  []RecoverJoint `json:"confirmed,omitempty"`
	// Base Cartesian jog rates; Fraction applies once. Both absent retains
	// the legacy browser contract, both present limits the existing joint path.
	LinearSpeedMMs   *float64 `json:"linear_speed_mm_s,omitempty"`
	AngularSpeedDegs *float64 `json:"angular_speed_deg_s,omitempty"`
}

type returnJogSpeeds struct{ linearMMs, angularDegs float64 }

func (in RecoverToZeroRequest) jogSpeeds() (*returnJogSpeeds, error) {
	if in.LinearSpeedMMs == nil && in.AngularSpeedDegs == nil {
		return nil, nil
	}
	if in.LinearSpeedMMs == nil || in.AngularSpeedDegs == nil ||
		!(*in.LinearSpeedMMs > 0 && *in.LinearSpeedMMs <= cartesianJogMaxLinearMMs) ||
		!(*in.AngularSpeedDegs > 0 && *in.AngularSpeedDegs <= cartesianJogMaxAngularDegS) {
		return nil, fmt.Errorf("Choose valid Jog speeds: tool travel 0–%g mm/s and rotation 0–%g°/s, both greater than zero",
			cartesianJogMaxLinearMMs, cartesianJogMaxAngularDegS)
	}
	return &returnJogSpeeds{*in.LinearSpeedMMs, *in.AngularSpeedDegs}, nil
}

// RecoverJoint is one rotary joint as the confirmation shows it.
type RecoverJoint struct {
	Axis     int     `json:"axis"`
	ID       string  `json:"id"`
	FromDeg  float64 `json:"from_deg"`
	ToDeg    float64 `json:"to_deg"`
	DeltaDeg float64 `json:"delta_deg"`
	// Moves: false for a joint already within the zero tolerance.
	Moves bool `json:"moves"`
}

// RecoverStep is one step as journaled and answered: outcome "done",
// "needs_confirmation" or "refused".
type RecoverStep struct {
	Step    string `json:"step"`
	Outcome string `json:"outcome"`
	Detail  string `json:"detail,omitempty"`
}

// RecoverRefusal is the step that did not clear and why, in operator words.
type RecoverRefusal struct {
	Step   string `json:"step"`
	Code   string `json:"code"`
	Detail string `json:"detail"`
}

// RecoverToZeroResult: State "done" (moved to zero), "at_zero" (every joint
// already there; nothing armed or moved), "confirm_arm" (the operator must
// confirm Action before Arm) or "refused".
type RecoverToZeroResult struct {
	State   string          `json:"state"`
	Action  string          `json:"action"`
	Steps   []RecoverStep   `json:"steps"`
	Joints  []RecoverJoint  `json:"joints,omitempty"`
	Refusal *RecoverRefusal `json:"refusal,omitempty"`
	Move    *JointMoveState `json:"move,omitempty"`
	Status  *StatusBody     `json:"status,omitempty"`
}

// RecoverToZeroBackend is a backend that runs the one-press recovery.
type RecoverToZeroBackend interface {
	RecoverToZero(context.Context, RecoverToZeroRequest) (*RecoverToZeroResult, error)
}

// recoverFault is one latched native fault: its name, the joints it names,
// its native recovery class and the joints whose Home a reset would leave
// invalid (native rehome_axis_mask: untrusted evidence or a fault that
// invalidates Home).
type recoverFault struct {
	Name               string
	Axes               []string
	Recovery           string
	Rehome             []string
	ResetPreservesHome bool
}

// recoverAlarm is one drive in Fault (readiness faulted) and the alarm it
// reports; Code is "(undecoded)" when the drive's alarm was not decoded.
type recoverAlarm struct {
	Axis string
	Code string
	Text string
}

// recoverFacts is everything one decision reads, from one fresh status and
// this server's own state, plus what this press has already tried.
type recoverFacts struct {
	Busy          string // a jog, move, program or request running; "" when idle
	StopLatched   bool   // stop_uncertain or inhibited
	ForeignHolder bool   // a lease, grant or Arm this server does not hold
	Armed         bool   // armed under this server's grant
	HomeInvalid   []string
	PositionLost  []string
	Faults        []recoverFault
	Alarms        []recoverAlarm
	AtZero        bool
	ArmConfirmed  bool
	Tried         map[string]bool
}

// recoverDecision is the next step: Action "confirm_stop", "reset",
// "confirm_arm" (answer and wait for the operator), "arm", "move", "at_zero",
// or "refuse" with the step, code and words.
type recoverDecision struct {
	Action string
	Step   string
	Code   string
	Detail string
}

func refuseRecover(step, code, detail string) recoverDecision {
	return recoverDecision{Action: "refuse", Step: step, Code: code, Detail: detail}
}

// settleWords ends a refusal of the steps the recovery press and Arm share:
// who never re-homes, what stops, and what the operator does next.
type settleWords struct {
	Actor string
	Stops string
	Again string
}

var (
	recoverWords = settleWords{Actor: "recovery", Stops: "recovery stops here", Again: "press again"}
	armWords     = settleWords{Actor: "Arm", Stops: "the robot stays disarmed", Again: "Arm again"}
)

// settleControlStep is the shared first part: nothing running, an
// unconfirmed Stop confirmed once (never retried in the same press), no
// other controller. ok false: nothing to decide here.
func settleControlStep(f recoverFacts, w settleWords) (recoverDecision, bool) {
	switch {
	case f.Busy != "":
		return refuseRecover("start", "motion_active", f.Busy+" is running; let it finish or press STOP, then "+w.Again), true
	case f.StopLatched && f.Tried["confirm_stop"]:
		return refuseRecover("confirm_stop", "stop_unconfirmed", "The last Stop is still unconfirmed; press STOP, or use the hardware stop"), true
	case f.StopLatched:
		return recoverDecision{Action: "confirm_stop", Step: "confirm_stop"}, true
	case f.ForeignHolder:
		return refuseRecover("control", "control_held_elsewhere", "Another controller holds the robot on the servo host; "+w.Stops+". Its operator disarms it, or press STOP"), true
	}
	return recoverDecision{}, false
}

// settleFaultStep is the shared fault part: Reset only when every latched
// fault's native recovery is a reset and no reset would leave a joint's Home
// invalid; a fault that needs a new Home, a restart or an unknown recovery
// refuses before anything is sent. A drive in Fault with nothing latched
// natively (a Rosie 1430 arm's J2 Er41.1, 2026-10-11: masks 0, the alarm only on the
// axis) is reset too: native resets it only when the drive profile classes
// the alarm resettable, else refuses fault_persists. A Reset native refused
// in this press is not retried: the refusal names the drive alarm still
// active. ok false: no latched fault and no drive in Fault.
func settleFaultStep(f recoverFacts, w settleWords) (recoverDecision, bool) {
	if len(f.Faults) == 0 && len(f.Alarms) == 0 {
		return recoverDecision{}, false
	}
	for _, fault := range f.Faults {
		if fault.Name == "coordinate_reference" && fault.ResetPreservesHome && len(fault.Rehome) == 0 {
			return refuseRecover("reset", "fault_not_resettable", fmt.Sprintf("%s: saved Home has recovered; use Clear fault while disarmed, then %s. %s never clears a reference fault automatically", faultSubject(fault), w.Again, w.Actor)), true
		}
		if fault.Recovery != "reset_clears" && fault.Recovery != "reset_after_condition_clears" {
			return refuseRecover("reset", "fault_not_resettable", fmt.Sprintf("%s needs %s, not a reset; %s", faultSubject(fault), recoveryWords(fault.Recovery, w.Actor), w.Stops)), true
		}
	}
	for _, fault := range f.Faults {
		if len(fault.Rehome) > 0 {
			return refuseRecover("reset", "fault_not_resettable", fmt.Sprintf("%s lost the position reference on %s: it needs %s, not a reset; %s", faultSubject(fault), strings.Join(fault.Rehome, ", "), recoveryWords("rehome_required", w.Actor), w.Stops)), true
		}
	}
	if f.Tried["reset"] {
		return refuseRecover("reset", "fault_persists", persistingFaultText(f.Faults, f.Alarms, w.Again)), true
	}
	return recoverDecision{Action: "reset", Step: "reset"}, true
}

// nextArmStep is Arm's part of the sequence (owner 2026-10-11: "re-arming the
// robot should clear faults too if they exist"): the same Stop confirmation
// and fault reset as the recovery press, then Arm itself ("arm"). With no
// latched Stop and no latched fault Arm is exactly what it was. Arm never
// asks for a confirmation of its own here: the client's Arm press is it.
func nextArmStep(f recoverFacts) recoverDecision {
	if !f.StopLatched && len(f.Faults) == 0 && len(f.Alarms) == 0 {
		return recoverDecision{Action: "arm", Step: "arm"}
	}
	if d, ok := settleControlStep(f, armWords); ok {
		return d
	}
	if d, ok := settleFaultStep(f, armWords); ok {
		return d
	}
	return recoverDecision{Action: "arm", Step: "arm"}
}

// nextRecoverStep is the sequence above as a pure decision over one set of
// facts; the controller re-reads the facts after every step it takes.
func nextRecoverStep(f recoverFacts) recoverDecision {
	if d, ok := settleControlStep(f, recoverWords); ok {
		return d
	}
	switch {
	case len(f.HomeInvalid) > 0:
		return refuseRecover("home", "home_invalid", fmt.Sprintf("Home is not valid on %s. Recovery never re-homes: an operator runs Home with each joint at its mechanical home pose", strings.Join(f.HomeInvalid, ", ")))
	case len(f.PositionLost) > 0:
		return refuseRecover("position", "position_unknown", fmt.Sprintf("The servo host reports no valid position for %s; recovery stops here", strings.Join(f.PositionLost, ", ")))
	}
	if d, ok := settleFaultStep(f, recoverWords); ok {
		return d
	}
	switch {
	case f.AtZero:
		return recoverDecision{Action: "at_zero", Step: "move"}
	case f.Armed:
		return recoverDecision{Action: "move", Step: "move"}
	case !f.ArmConfirmed:
		return recoverDecision{Action: "confirm_arm", Step: "arm"}
	case f.Tried["arm"]:
		return refuseRecover("arm", "arm_unconfirmed", "Arm was accepted but the machine does not report armed; recovery stops here")
	}
	return recoverDecision{Action: "arm", Step: "arm"}
}

func faultSubject(fault recoverFault) string {
	if len(fault.Axes) == 0 {
		return "Fault " + fault.Name
	}
	return strings.Join(fault.Axes, ", ") + " " + fault.Name
}

func recoveryWords(recovery, actor string) string {
	switch recovery {
	case "rehome", "rehome_required", "home":
		return "a new Home at the mechanical home pose (" + actor + " never re-homes)"
	case "restart", "restart_required":
		return "a controller restart"
	case "power_cycle", "power_cycle_required":
		return "a drive power cycle"
	case "":
		return "an unknown recovery"
	}
	return recovery
}

// persistingFaultText names what a refused reset left: each drive alarm
// still reported, with what clears it, or the faults when no drive says why;
// again is what the operator presses once it has cleared.
func persistingFaultText(faults []recoverFault, alarms []recoverAlarm, again string) string {
	var parts []string
	for _, alarm := range alarms {
		what := alarm.Axis + " drive alarm " + alarm.Code
		if alarm.Text != "" {
			what += " (" + alarm.Text + ")"
		}
		remedy := "clear its cause at the drive"
		if strings.Contains(strings.ToLower(alarm.Text), "temperature") {
			remedy = "let it cool"
		}
		parts = append(parts, what+" still active — "+remedy)
	}
	if len(parts) == 0 {
		for _, fault := range faults {
			parts = append(parts, faultSubject(fault)+" still active after Reset — clear its cause")
		}
	}
	return strings.Join(parts, "; ") + ", then " + again
}

// recoverJoints lists every rotary joint's angle, target and travel; zero
// within recoverZeroToleranceDeg stays.
func recoverJoints(ids []string, positionsRad []float64) []RecoverJoint {
	joints := make([]RecoverJoint, 0, len(ids))
	for i, id := range ids {
		from := positionsRad[i] * 180 / math.Pi
		joints = append(joints, RecoverJoint{Axis: i, ID: id, FromDeg: from, ToDeg: 0, DeltaDeg: 0 - from, Moves: math.Abs(from) >= recoverZeroToleranceDeg})
	}
	return joints
}

// recoverConfirmationStale names the joints whose angle changed since the
// operator confirmed, or "" when the confirmation still describes the pose.
func recoverConfirmationStale(confirmed, now []RecoverJoint) string {
	byAxis := map[int]RecoverJoint{}
	for _, joint := range confirmed {
		byAxis[joint.Axis] = joint
	}
	var moved []string
	for _, joint := range now {
		seen, ok := byAxis[joint.Axis]
		if !ok || math.IsNaN(seen.FromDeg) || math.IsInf(seen.FromDeg, 0) {
			moved = append(moved, joint.ID+" (not in the confirmation)")
			continue
		}
		if d := math.Abs(seen.FromDeg - joint.FromDeg); d > recoverConfirmToleranceDeg {
			moved = append(moved, fmt.Sprintf("%s by %.2f°", joint.ID, d))
		}
	}
	if len(moved) == 0 {
		return ""
	}
	return "The joints moved since the confirmation (" + strings.Join(moved, ", ") + "); press again to see the current angles"
}
