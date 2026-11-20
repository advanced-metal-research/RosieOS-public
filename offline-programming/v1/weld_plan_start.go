package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
)

// A planning start is a read-only observation, never a request to move to reset.
// It lives in the immutable .weldplan, separate from the model's reset pose.
type measuredPlanStart struct {
	HeldTargets         []float64           `json:"-"`
	AxisTimes           []uint64            `json:"-"`
	Schema              string              `json:"schema"`
	JointNames          []string            `json:"joint_names"`
	JointValues         []float64           `json:"joint_values_rad"`
	Robot               denseexec.RobotCell `json:"robot_cell"`
	CoordinateIdentity  string              `json:"coordinate_identity"`
	ObservationIdentity string              `json:"observation_identity"`
	ObservedTimeNS      uint64              `json:"observed_time_ns,string"`
}

// motionOps are the program.v2 nodes that move the arm. Any enabled one makes
// a connected Plan start from the robot's measured pose (2026-10-11, owner:
// "this is a general purpose robot, we must be able to program it"):
// waypoints, Home and the weld block's approach/weld/transit/retract alike
// (weldplan/measured_start.py).
var motionOps = map[string]bool{"move": true, "weld": true, "approach": true, "transit": true, "retract": true, "home": true}

func hasMotionNodes(raw json.RawMessage) bool {
	var p struct {
		Nodes []struct {
			Op      string `json:"op"`
			Weld    string `json:"weld_transition"`
			Enabled *bool  `json:"enabled"`
		} `json:"nodes"`
	}
	if json.Unmarshal(raw, &p) != nil {
		return false
	}
	for _, n := range p.Nodes {
		if motionOps[n.Op] && (n.Enabled == nil || *n.Enabled) {
			return true
		}
	}
	return false
}

// taughtProgramUnsupported is the refusal for a node the planner cannot plan
// on this path: an `io` row with no cell connected, since the plan's output
// schedule needs the cell's outputs and their expiry (weldplan/io_schedule.py).
// Answered before the pack, naming the rows. Until 2026-10-11 it also named
// Home and the weld block in a program with taught waypoints (a HOME before
// four waypoints reached the Deck as "Planning failed" on a Rosie 1430 arm, 15:45); those
// plan from the measured pose now. The code is kept so journals compare.
const taughtProgramUnsupported = "taught_program_unsupported_node"

// ioNotConfigured: the connected cell's rt-core configuration has no `io`
// block, so no output can be scheduled (a Rosie 1430 arm on 2026-10-11).
const ioNotConfigured = "io_not_configured"

// ioOutputsSchema is weldplan/io_schedule.py SCHEMA.
const ioOutputsSchema = "rosie.cell-io-outputs.v1"

func hasIONodes(raw json.RawMessage) bool {
	var p struct {
		Nodes []struct {
			Op      string `json:"op"`
			Weld    string `json:"weld_transition"`
			Enabled *bool  `json:"enabled"`
		} `json:"nodes"`
	}
	if json.Unmarshal(raw, &p) != nil {
		return false
	}
	for _, n := range p.Nodes {
		if (n.Op == "io" || (n.Op == "move" && n.Weld != "" && n.Weld != "none")) && (n.Enabled == nil || *n.Enabled) {
			return true
		}
	}
	return false
}

// cellIOOutputs is the connected cell's configured outputs as the planner
// schedules them (rosie.cell-io-outputs.v1): name, class and expiry from
// rt-core's Describe `io` block, which the native status carries. Every
// marker the plan then holds names one of these, and is refreshed inside its
// expiry. A cell with no `io` block is refused naming the rows.
func cellIOOutputs(ctx context.Context, target CellTarget, program json.RawMessage) (map[string]any, error) {
	observer, ok := target.(interface {
		Status(context.Context) *denseexec.StatusBody
	})
	var described struct {
		RTCore struct {
			Description struct {
				IO *struct {
					TorchQualified bool `json:"torch_qualified"`
					Inputs         []struct {
						Name string `json:"name"`
					} `json:"inputs"`
					Outputs []struct {
						Name     string `json:"name"`
						Class    string `json:"class"`
						ExpiryNS int64  `json:"expiry_ns"`
					} `json:"outputs"`
				} `json:"io"`
			} `json:"description"`
		} `json:"rt_core"`
	}
	if ok {
		if raw, err := json.Marshal(observer.Status(ctx)); err == nil {
			_ = json.Unmarshal(raw, &described)
		}
	}
	io := described.RTCore.Description.IO
	if io == nil || (len(io.Outputs) == 0 && len(io.Inputs) == 0) {
		names := ioRowNames(program)
		verb := "switches an output"
		if len(names) > 1 {
			verb = "switch outputs"
		}
		return nil, &planStartRefusal{code: ioNotConfigured, message: fmt.Sprintf(
			"%s %s, and the connected cell has no outputs configured (its rt-core configuration has no io block). "+
				"Add and qualify an I/O terminal on the cell, or delete or disable the row, then plan again.",
			strings.Join(names, ", "), verb)}
	}
	outputs := make([]map[string]any, 0, len(io.Outputs))
	for _, o := range io.Outputs {
		// Native status indicators are visible in I/O monitoring but cannot be
		// scheduled or chosen as a weld trigger. Keep them out of the process
		// planner's output contract, which accepts only process/torch outputs.
		if o.Class == "indicator" {
			continue
		}
		outputs = append(outputs, map[string]any{"name": o.Name, "class": o.Class, "expiry_ns": o.ExpiryNS})
	}
	result := map[string]any{"schema": ioOutputsSchema, "torch_qualified": io.TorchQualified, "outputs": outputs}
	if len(io.Inputs) > 0 {
		result["inputs"] = io.Inputs
	}
	return result, nil
}

func ioRowNames(raw json.RawMessage) []string {
	var p struct {
		Nodes []struct {
			Op      string `json:"op"`
			Label   string `json:"label"`
			Enabled *bool  `json:"enabled"`
		} `json:"nodes"`
	}
	if json.Unmarshal(raw, &p) != nil {
		return nil
	}
	var names []string
	for _, n := range p.Nodes {
		if (n.Enabled == nil || *n.Enabled) && n.Op == "io" {
			names = append(names, nodeName(n.Label, n.Op))
		}
	}
	return names
}

// taughtProgramRefusal is nil unless an enabled `io` node is in a program
// planned with no cell connected. The sentence is the planner's
// (io_schedule.io_problems) and the one the pendant and the browser show
// before posting (rails/planRequest.ts measuredStartUnsupported).
func taughtProgramRefusal(raw json.RawMessage) error {
	var p struct {
		Nodes []struct {
			Op      string `json:"op"`
			Label   string `json:"label"`
			Enabled *bool  `json:"enabled"`
		} `json:"nodes"`
	}
	if json.Unmarshal(raw, &p) != nil {
		return nil
	}
	var names []string
	for _, n := range p.Nodes {
		if (n.Enabled != nil && !*n.Enabled) || n.Op != "io" {
			continue
		}
		names = append(names, nodeName(n.Label, n.Op))
	}
	if len(names) == 0 {
		return nil
	}
	return &planStartRefusal{code: taughtProgramUnsupported, message: unsupportedSentence(names)}
}

func nodeName(label, op string) string {
	if label = strings.TrimSpace(label); label == "" || strings.EqualFold(label, op) {
		return strings.ToUpper(op)
	}
	return fmt.Sprintf("%s (%s)", label, op)
}

func unsupportedSentence(names []string) string {
	verb := "switches an output"
	if len(names) > 1 {
		verb = "switch outputs"
	}
	return fmt.Sprintf("%s %s: plan with the cell connected, so the plan knows the cell's outputs and their expiry",
		strings.Join(names, ", "), verb)
}

// Measured-start refusal codes, answered by /weld-plan beside the sentence.
// planStartMoving is the one an operator clears by letting go: a jog still
// open or held, a move still running or queued, or joints that moved inside
// the capture window. planStartUnverified is everything else: feedback,
// Home, identity, a fault or the controller's own refusal.
const (
	planStartMoving     = "planning_start_moving"
	planStartUnverified = "planning_start_unverified"
	// The pendant and the browser show this sentence (or their own words for
	// the same instruction, execution/runControls.ts planMotionInstruction).
	planStartMovingMessage = "Release the jog and wait for the robot to stop before planning"
	planStartFeedback      = "measured planning start requires fresh stationary verified feedback for every described joint; reconnect or establish Home before planning"
)

// planStartRefusal is a measured-start refusal that names its cause, so an
// operator told "reconnect or establish Home" is never really being asked to
// let go of a jog.
type planStartRefusal struct{ code, message string }

func (e *planStartRefusal) Error() string { return e.message }

func planStartMotion(message string) error {
	return &planStartRefusal{code: planStartMoving, message: message}
}

func planStartUnverifiedf(format string, args ...any) error {
	return &planStartRefusal{code: planStartUnverified, message: fmt.Sprintf(format, args...)}
}

// Decode the public JSON envelope so the non-Linux authoring build does not
// depend on SDK runtime types. Missing validity fields fail closed.
//
// A finished jog is admitted while the robot stays armed: mode JOG (3)
// persists after the hold's lane closes, so the terminal state (completed or
// aborted, done, nothing queued, no open lane and no held input) is the end
// of a jog, not motion. Planning is a read-only observation; the measured
// start is fenced again at Load (validatePlanningCapture: configuration,
// Home epoch, controller incarnation and the first sample) and native Play
// checks current positions against the first sample, so requiring Stop,
// disarm and a released lease first added no protection, only an operator
// step that the old refusal text sent people to Home for. Every fault, Home,
// commissioning, identity and multi-sample stillness check still applies.
func measuredStartObservation(raw []byte, cell denseexec.RobotCell) (*measuredPlanStart, error) {
	var s struct {
		Available bool                 `json:"available"`
		Error     string               `json:"daemon_error"`
		Robot     *denseexec.RobotCell `json:"robot_cell"`
		Session   *struct {
			State      string `json:"state"`
			StopReason string `json:"stop_reason"`
		} `json:"session"`
		Native *struct {
			Reason        string `json:"reason"`
			Refusal       string `json:"refusal"`
			RefusalKind   string `json:"refusal_kind"`
			Inhibited     bool   `json:"inhibited"`
			StopUncertain bool   `json:"stop_uncertain"`
			Description   struct {
				Axes []struct {
					ID    string `json:"id"`
					Index int    `json:"index"`
					Unit  string `json:"position_unit"`
				} `json:"axes"`
			} `json:"description"`
			Status *struct {
				Time    uint64 `json:"time_ns"`
				Adapter string `json:"adapter_incarnation"`
				Daemon  string `json:"daemon_incarnation"`
				Grant   *struct {
					Active  *uint32 `json:"GrantActive"`
					StopAck uint64  `json:"StopAckId"`
				} `json:"grant"`
				Motion struct {
					Time  uint64  `json:"time_ns"`
					Mode  *uint32 `json:"mode"`
					State *uint32 `json:"state"`
					Queue *uint32 `json:"queue_depth"`
					Done  bool    `json:"done"`
				} `json:"motion"`
				Jog *struct {
					Open     uint32
					HasInput uint32
				} `json:"jog"`
				Core struct {
					Armed         *uint32 `json:"armed"`
					Lease         *uint32 `json:"lease_valid"`
					Enabled       *uint64 `json:"axis_enable_mask"`
					Config        string  `json:"configuration_sha256"`
					ConfigEpoch   uint64  `json:"configuration_epoch"`
					HomeEpoch     uint64  `json:"home_epoch"`
					Fault         uint64  `json:"safety_fault_mask"`
					Commissioning uint32  `json:"commissioning_phase"`
					HomeActive    uint64  `json:"native_home_active_axis_mask"`
				} `json:"core"`
				Axes []struct {
					Time       uint64   `json:"time_ns"`
					Q          *float64 `json:"logical_position"`
					Target     *float64 `json:"logical_target"`
					Logical    bool     `json:"logical_valid"`
					Home       bool     `json:"home_valid"`
					Calibrated bool     `json:"calibration_valid"`
					State      string   `json:"position_state"`
				} `json:"axes"`
			} `json:"status"`
		} `json:"rt_core"`
	}
	if json.Unmarshal(raw, &s) != nil || !s.Available || s.Native == nil || s.Native.Status == nil || s.Robot == nil {
		return nil, planStartUnverifiedf("%s (the connected machine reported no native status)", planStartFeedback)
	}
	// An old plan rejected after Home has exactly one recovery: capture a new
	// measured start and replan. This retained application refusal is not a
	// native fault. Do not let it deadlock its own read-only recovery path.
	stalePlan := s.Native.RefusalKind == "prepare_program" && s.Native.Refusal == "planning_start_stale"
	// Ordinary acknowledged Stop is a successful nonmotion state, not a fault.
	// A read-only connection has no app session, even after a successful Stop.
	// Require the app receipt plus explicit native acknowledgement/inhibition
	// state; all stationary, coordinate and freshness checks below still apply.
	native := s.Native.Status
	stopped := s.Error == "operator_stop" && s.Native.Reason == "operator_stop" &&
		(s.Session == nil || (s.Session.State == "stopped" && s.Session.StopReason == "operator_stop")) &&
		native.Grant != nil && native.Grant.Active != nil && *native.Grant.Active == 0 && native.Grant.StopAck != 0 &&
		native.Core.Armed != nil && *native.Core.Armed == 0 && native.Core.Enabled != nil && *native.Core.Enabled == 0
	allowedReason := func(reason string) bool {
		return reason == "" || (stopped && reason == "operator_stop") || (stalePlan && (reason == "planning_start_stale" || strings.HasPrefix(reason, "planning_start_stale:")))
	}
	for _, reason := range []string{s.Error, s.Native.Reason} {
		if !allowedReason(reason) {
			return nil, planStartUnverifiedf("the machine reports %q; planning waits until it clears (STOP, or reconnect the cell)", reason)
		}
	}
	if s.Native.StopUncertain {
		return nil, planStartUnverifiedf("the machine's last Stop is unconfirmed (stop_uncertain); STOP again or reconnect before planning")
	}
	if s.Native.Inhibited {
		return nil, planStartUnverifiedf("the machine reports its outputs inhibited; reconnect the cell before planning")
	}
	r := s.Robot
	if !r.Valid || !cell.Valid || r.ModelID != cell.ModelID || r.RobotDescriptionSHA256 != cell.RobotDescriptionSHA256 || r.MachinePlanningCalibrationSHA256 != cell.MachinePlanningCalibrationSHA256 {
		return nil, errors.New("planning start robot/calibration changed while compiling the request")
	}
	n := s.Native.Status
	axes := s.Native.Description.Axes
	switch {
	case len(axes) == 0 || len(axes) != len(n.Axes):
		return nil, planStartUnverifiedf("%s (the machine describes %d joints and reports %d)", planStartFeedback, len(axes), len(n.Axes))
	case n.Time == 0 || n.Motion.Time == 0 || n.Adapter == "" || n.Daemon == "":
		return nil, planStartUnverifiedf("%s (native status has no time or controller identity)", planStartFeedback)
	case n.Core.Config == "" || n.Core.ConfigEpoch == 0 || n.Core.HomeEpoch == 0:
		return nil, planStartUnverifiedf("%s (no configuration or Home epoch)", planStartFeedback)
	case n.Core.Fault != 0:
		return nil, planStartUnverifiedf("the machine reports a safety fault (mask 0x%x); clear it before planning", n.Core.Fault)
	case n.Core.HomeActive != 0:
		return nil, planStartUnverifiedf("Home is running (axis mask 0x%x); plan after it finishes", n.Core.HomeActive)
	case n.Core.Commissioning != 0:
		return nil, planStartUnverifiedf("commissioning phase %d is active; plan after it ends", n.Core.Commissioning)
	case n.Motion.Mode == nil || n.Motion.State == nil || n.Motion.Queue == nil:
		return nil, planStartUnverifiedf("%s (native motion state is missing)", planStartFeedback)
	}
	mode, state := *n.Motion.Mode, *n.Motion.State
	terminal := state == 0 || ((state == 4 || state == 5) && n.Motion.Done)
	jogOpen := n.Jog != nil && (n.Jog.Open != 0 || n.Jog.HasInput != 0)
	switch {
	case jogOpen || (mode == 3 && (!terminal || *n.Motion.Queue != 0)):
		return nil, planStartMotion(planStartMovingMessage)
	case mode == 3 && n.Jog == nil:
		return nil, planStartUnverifiedf("%s (a jog ended but the machine reports no jog state)", planStartFeedback)
	case !terminal || *n.Motion.Queue != 0:
		return nil, planStartMotion("A move is still running; wait for the robot to stop before planning")
	case mode != 0 && mode != 2 && mode != 3:
		return nil, planStartUnverifiedf("%s (unknown motion mode %d)", planStartFeedback, mode)
	}
	if cell.MachineConfigurationSHA256 != "sha256:"+n.Core.Config {
		return nil, errors.New("planning start configuration changed while compiling the request")
	}
	out := &measuredPlanStart{Schema: "rosie.measured-planning-start.v1", Robot: *r, CoordinateIdentity: fmt.Sprintf("%s:%d:%d", n.Core.Config, n.Core.ConfigEpoch, n.Core.HomeEpoch), ObservationIdentity: n.Adapter + ":" + n.Daemon, ObservedTimeNS: n.Motion.Time}
	seen := map[string]bool{}
	for i, a := range axes {
		q := n.Axes[i]
		// Native status does not publish logical_velocity. Admission uses the
		// supported nonmoving state and a bounded window of advancing positions.
		if q.Time == 0 || a.ID == "" || seen[a.ID] || a.Index != i || a.Unit != "rad" {
			return nil, planStartUnverifiedf("%s (axis %d is not a described rotary joint with fresh feedback)", planStartFeedback, i)
		}
		if !q.Logical || !q.Home || !q.Calibrated || q.State != "trusted" || q.Q == nil || math.IsNaN(*q.Q) || math.IsInf(*q.Q, 0) {
			return nil, planStartUnverifiedf("%s (%s: position %s, homed %t, calibrated %t, logical %t)", planStartFeedback, a.ID, q.State, q.Home, q.Calibrated, q.Logical)
		}
		seen[a.ID] = true
		out.AxisTimes = append(out.AxisTimes, q.Time)
		out.JointNames = append(out.JointNames, a.ID)
		out.JointValues = append(out.JointValues, *q.Q)
		if q.Target != nil && !math.IsNaN(*q.Target) && !math.IsInf(*q.Target, 0) {
			out.HeldTargets = append(out.HeldTargets, *q.Target)
		}
	}
	return out, nil
}

func captureMeasuredPlanStart(ctx context.Context, target CellTarget, cell denseexec.RobotCell, held ...bool) (*measuredPlanStart, error) {
	ctx, cancel := context.WithTimeout(ctx, 500*time.Millisecond)
	defer cancel()
	observer, ok := target.(interface {
		Status(context.Context) *denseexec.StatusBody
	})
	if !ok {
		return nil, errors.New("connected planning requires native measured-start observation")
	}
	read := func() (*measuredPlanStart, error) {
		raw, err := json.Marshal(observer.Status(ctx))
		if err != nil {
			return nil, err
		}
		return measuredStartObservation(raw, cell)
	}
	started := time.Now()
	a, err := read()
	if err != nil {
		return nil, err
	}
	useHeld := len(held) > 0 && held[0]
	if useHeld && len(a.HeldTargets) != len(a.JointValues) {
		return nil, planStartUnverifiedf("Trace needs fresh held joint targets")
	}
	// This is planning admission, not a physical standstill certificate. Every
	// sample must remain close to the first; endpoint-only checks miss a return
	// after an intermediate excursion. The public logical rate is unavailable.
	previous := a
	windowStarted := time.Now()
	for samples := 1; samples < 5 || time.Since(windowStarted) < 100*time.Millisecond; {
		timer := time.NewTimer(25 * time.Millisecond)
		select {
		case <-ctx.Done():
			timer.Stop()
			return nil, ctx.Err()
		case <-timer.C:
		}
		b, err := read()
		if err != nil {
			return nil, err
		}
		if time.Since(started) > 500*time.Millisecond || b.ObservedTimeNS < previous.ObservedTimeNS || a.CoordinateIdentity != b.CoordinateIdentity || a.ObservationIdentity != b.ObservationIdentity || len(a.JointNames) != len(b.JointNames) {
			return nil, planStartUnverifiedf("planning start feedback is stale or its native identity changed; retry after fresh stationary feedback")
		}
		advanced := b.ObservedTimeNS > previous.ObservedTimeNS
		if useHeld && len(b.HeldTargets) != len(a.HeldTargets) {
			return nil, planStartUnverifiedf("Trace needs fresh held joint targets")
		}
		for i := range a.JointNames {
			if b.AxisTimes[i] < previous.AxisTimes[i] || a.JointNames[i] != b.JointNames[i] {
				return nil, planStartUnverifiedf("planning start feedback is stale or its native identity changed; retry after fresh stationary feedback")
			}
			if math.Abs(a.JointValues[i]-b.JointValues[i]) > 1e-5 {
				return nil, planStartMotion(fmt.Sprintf("robot moved while capturing the planning start (%s by %.2g rad); wait for the robot to stop, then plan again", a.JointNames[i], math.Abs(a.JointValues[i]-b.JointValues[i])))
			}
			if useHeld && a.HeldTargets[i] != b.HeldTargets[i] {
				return nil, planStartMotion("Joint targets changed while preparing Trace; wait for the robot to stop")
			}
			advanced = advanced && b.AxisTimes[i] > previous.AxisTimes[i]
		}
		if !advanced {
			continue
		}
		samples++
		previous = b
	}
	if useHeld {
		if len(previous.HeldTargets) != len(previous.JointValues) {
			return nil, planStartUnverifiedf("Trace needs fresh held joint targets")
		}
		// Measured feedback still proves standstill; path jog starts at the held
		// command so it does not introduce a discontinuity from servo error.
		previous.JointValues = previous.HeldTargets
	}
	return previous, nil
}
