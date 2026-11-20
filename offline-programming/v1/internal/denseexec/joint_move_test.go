//go:build linux

package denseexec

import (
	"context"
	"math"
	"reflect"
	"slices"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type fakeMoveAdapter struct {
	*fakeRTCore
	points                               []control.Point
	mask                                 uint32
	fences                               []control.Fence
	prepareEntered, prepareWait, started chan struct{}
	prepareOnce, startOnce               sync.Once
	autoComplete                         bool
	refuseMove                           string
	statusReads                          int
	axisLag                              bool
}

type reviewMovingToDone struct {
	*fakeMoveAdapter
	reads int
}

func (f *reviewMovingToDone) Status(ctx context.Context) (control.Status, error) {
	s, err := f.fakeMoveAdapter.Status(ctx)
	f.reads++
	switch f.reads {
	case 1:
		s.Motion.Done, s.Motion.TimeNS = false, 100
		s.Axes[0].TimeNS, s.Axes[0].LogicalTarget = 100, 0.25
	case 2:
		s.Motion.Done, s.Motion.TimeNS = true, 200
		s.Axes[0].TimeNS, s.Axes[0].LogicalTarget = 150, 0.25
	case 3:
		s.Motion.Done, s.Motion.TimeNS = true, 201
		s.Axes[0].TimeNS, s.Axes[0].LogicalTarget = 201, 0.5
	}
	return s, err
}

// TestReviewJointMoveWaitsForDoneAxisPublication preserves the reviewer probe:
// an axis sample after admission but before Done cannot become the held origin.
func TestReviewJointMoveWaitsForDoneAxisPublication(t *testing.T) {
	r, f, in := moveFake(t)
	r.cancel()
	<-r.done
	r.c = &reviewMovingToDone{fakeMoveAdapter: f}
	if _, err := r.JointMove(context.Background(), in); err != nil {
		t.Fatal(err)
	}
	if got := f.points[0].Position[0]; got != 0.5 {
		t.Fatalf("prepared from pre-Done target %g rad; want held target 0.5 rad", got)
	}
}

func moveFake(t *testing.T) (*RTCoreController, *fakeMoveAdapter, JointMoveIntent) {
	t.Helper()
	r, f, _ := rtFake(t)
	f.description.CycleNS, f.description.MaxTrajectoryPoints = 1000000, 100000
	accel := 2.0
	for i := range f.description.Axes {
		f.description.Axes[i] = ipcclient.AxisDescription{PositionUnit: "rad", MinPosition: -2, MaxPosition: 2, MaxVelocity: 2, JogAcceleration: 4, MaxAcceleration: &accel, CompletionTolerance: 0.001, CompletionTimeoutNS: 500000000}
	}
	f.status.Axes[0].LogicalTarget, f.status.Axes[0].LogicalPosition = 0.25, 0.2
	f.status.Motion.Done = true
	m := &fakeMoveAdapter{fakeRTCore: f, autoComplete: true, prepareEntered: make(chan struct{}), started: make(chan struct{})}
	r.c = m
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	return r, m, JointMoveIntent{TargetID: "bench", SessionID: r.Status(context.Background()).Session.ID, Axis: 0, Degrees: 30, Fraction: 0.5}
}

func TestJointMoveAbsoluteStart(t *testing.T) {
	r, f, in := moveFake(t)
	for _, target := range []float64{0.00001, 0.00001, -0.4} {
		in.Axes = []JointMoveAxis{{Axis: 0, PositionRad: &target}}
		in.Revision = r.JointJogState().Revision
		if _, err := r.JointMove(context.Background(), in); err != nil {
			t.Fatal(err)
		}
		if got := f.points[len(f.points)-1].Position[0]; math.Abs(got-target) > 1e-12 {
			t.Fatalf("absolute endpoint %g, want %g despite distinct measured/held positions", got, target)
		}
	}
	outside := 3.0
	in.Axes = []JointMoveAxis{{Axis: 0, PositionRad: &outside}}
	in.Revision = r.JointJogState().Revision
	if _, err := r.JointMove(context.Background(), in); err == nil {
		t.Fatal("absolute endpoint escaped joint limits")
	}
}

func (f *fakeMoveAdapter) Status(context.Context) (control.Status, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	f.statusReads++
	f.status.Motion.TimeNS++
	if !f.axisLag {
		for i := range f.status.Axes {
			f.status.Axes[i].TimeNS = f.status.Motion.TimeNS
		}
	}
	s := f.status
	s.Axes = append([]ipcclient.LogicalAxisStatus(nil), s.Axes...)
	s.Execution.Handles = append([]control.HandleRecord(nil), s.Execution.Handles...)
	return s, f.statusErr
}
func (f *fakeMoveAdapter) Command(ctx context.Context, in *control.Request) (control.Response, error) {
	if in.Operation != "prepare_trajectory" && in.Operation != "start_trajectory" && in.Operation != "discard_trajectory" {
		return f.fakeRTCore.Command(ctx, in)
	}
	f.mu.Lock()
	f.record(in.Operation)
	f.fences = append(f.fences, in.Fence)
	refused := f.refuseMove == in.Operation
	if in.Operation == "prepare_trajectory" {
		f.points, f.mask = in.Points, in.AxisMask
	}
	entered, wait := f.prepareEntered, f.prepareWait
	f.mu.Unlock()
	if in.Operation == "prepare_trajectory" {
		f.prepareOnce.Do(func() { close(entered) })
		if wait != nil {
			<-wait
		} // Deliberately return a late receipt after Stop.
	}
	if refused {
		return control.Response{}, rtRefused("not_ready")
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	switch in.Operation {
	case "prepare_trajectory":
		f.status.Execution.Handles = []control.HandleRecord{{Handle: 17, State: "prepared", Generation: 1}}
	case "start_trajectory":
		if err := ctx.Err(); err != nil {
			return control.Response{}, err
		}
		f.status.Execution.Handles = []control.HandleRecord{{Handle: 17, State: "started", Generation: 1, NativeSequence: 42}}
		f.status.Motion = ipcclient.MotionState{TrajectoryID: 17, CommandSequence: 42, TimeNS: 10, State: 2}
		if f.autoComplete {
			f.completeLocked()
		}
		f.startOnce.Do(func() { close(f.started) })
	case "discard_trajectory":
		f.status.Execution.Handles = []control.HandleRecord{{Handle: 17, State: "discarded"}}
	}
	return control.Response{Operation: in.Operation, Handle: 17, Sequence: 42}, nil
}
func (f *fakeMoveAdapter) completeLocked() {
	f.status.Axes = append([]ipcclient.LogicalAxisStatus(nil), f.status.Axes...)
	// Every commanded joint of the prepared trajectory settles on its endpoint.
	for i := range f.status.Axes {
		if i == 0 || f.mask&(1<<uint(i)) != 0 {
			f.status.Axes[i].LogicalPosition = f.points[len(f.points)-1].Position[i]
			f.status.Axes[i].LogicalTarget = f.status.Axes[i].LogicalPosition
		}
	}
	f.status.Motion.Done, f.status.Motion.State = true, 4
	f.status.Execution.Handles = []control.HandleRecord{{Handle: 17, State: "consumed", Generation: 1, NativeSequence: 42}}
}
func moveCalls(f *fakeMoveAdapter) []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	var calls []string
	for _, c := range f.calls {
		if c == "prepare_trajectory" || c == "start_trajectory" || c == "discard_trajectory" || c == "stop" || c == "release" {
			calls = append(calls, c)
		}
	}
	return calls
}

func TestJointMovePlanningBounds(t *testing.T) {
	r, _, in := moveFake(t)
	d := r.description
	for _, degrees := range []float64{30, -30, 0.001} {
		in.Degrees = degrees
		p, err := jointMoveProfile(d, in, 0.25)
		if err != nil {
			t.Fatal(err)
		}
		if p[0].Position[0] != 0.25 || p[len(p)-1].Position[0] != 0.25+degrees*math.Pi/180 {
			t.Fatal("profile changed start or endpoint")
		}
		previousVelocity := 0.0
		for i, point := range p {
			if point.Time != time.Duration(i)*time.Millisecond {
				t.Fatal("not on daemon cycle")
			}
			for _, pos := range point.Position[1:] {
				if pos != 0 {
					t.Fatal("unselected joint moved")
				}
			}
			if i == 0 {
				continue
			}
			v := (point.Position[0] - p[i-1].Position[0]) / 0.001
			if math.Abs(v) > 1 {
				t.Fatalf("velocity=%g > 1 rad/s", v)
			}
			if math.Abs(v-previousVelocity)/0.001 > 1.000001 {
				t.Fatalf("acceleration=%g > 1.000001 rad/s²", math.Abs(v-previousVelocity)/0.001)
			}
			previousVelocity = v
		}
	}
	for _, tc := range []struct {
		name string
		edit func(*control.Description, *JointMoveIntent)
		code string
	}{
		{"soft_limit", func(_ *control.Description, i *JointMoveIntent) { i.Degrees = 120 }, "joint_move_limits"},
		{"range", func(_ *control.Description, i *JointMoveIntent) { i.Degrees = 250 }, "joint_move_limits"},
		{"zero", func(_ *control.Description, i *JointMoveIntent) { i.Degrees = 0 }, "joint_move_invalid"},
		{"nan", func(_ *control.Description, i *JointMoveIntent) { i.Degrees = math.NaN() }, "joint_move_invalid"},
		{"fraction", func(_ *control.Description, i *JointMoveIntent) { i.Fraction = 1.01 }, "joint_move_invalid"},
		{"points", func(d *control.Description, _ *JointMoveIntent) { d.MaxTrajectoryPoints = 3 }, "joint_move_too_many_points"},
		{"units", func(d *control.Description, _ *JointMoveIntent) { d.Axes[0].PositionUnit = "m" }, "joint_move_units"},
		{"acceleration", func(d *control.Description, _ *JointMoveIntent) { v := 0.0; d.Axes[0].MaxAcceleration = &v }, "joint_move_description_invalid"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			copy := d
			copy.Axes = append([]ipcclient.AxisDescription(nil), d.Axes...)
			input := in
			input.Degrees = 30
			tc.edit(&copy, &input)
			_, err := jointMoveProfile(copy, input, 0.25)
			jointCode(t, jointJogFailure(err), tc.code)
		})
	}
}

func TestJointMovePrepareStartCompletion(t *testing.T) {
	r, f, in := moveFake(t)
	state, err := r.JointMove(context.Background(), in)
	if err != nil || state.State != "done" || state.Revision != 1 {
		t.Fatalf("state=%+v error=%v", state, err)
	}
	if got := moveCalls(f); !reflect.DeepEqual(got, []string{"prepare_trajectory", "start_trajectory"}) {
		t.Fatal(got)
	}
	if f.points[0].Position[0] != 0.25 {
		t.Fatal("move did not begin at held target")
	}
	for _, fence := range f.fences {
		if fence.Session != "grant" || fence.Generation != 1 {
			t.Fatal(fence)
		}
	}
	_, err = r.JointMove(context.Background(), in)
	jointCode(t, err, "jog_session_stale")
}

func TestJointMoveWaitsForAxisPublication(t *testing.T) {
	for _, phase := range []string{"held_target", "completion"} {
		t.Run(phase, func(t *testing.T) {
			r, f, in := moveFake(t)
			f.mu.Lock()
			f.autoComplete = phase == "held_target"
			if phase == "held_target" {
				f.axisLag = true
				f.status.Motion.TimeNS, f.status.Axes[0].TimeNS = 100, 1
				f.statusReads = 0
			}
			f.mu.Unlock()
			result := make(chan error, 1)
			go func() { _, err := r.JointMove(context.Background(), in); result <- err }()
			if phase == "completion" {
				<-f.started
				f.mu.Lock()
				f.axisLag = true
				f.completeLocked()
				f.status.Motion.TimeNS, f.status.Axes[0].TimeNS = 100, 1
				f.statusReads = 0
				f.mu.Unlock()
			}
			rtAwait(t, func() bool { f.mu.Lock(); defer f.mu.Unlock(); return f.statusReads >= 3 })
			select {
			case err := <-result:
				t.Fatalf("stale axis publication completed move: %v", err)
			default:
			}
			if phase == "held_target" && len(moveCalls(f)) != 0 {
				t.Fatal("prepared from stale held target")
			}
			f.mu.Lock()
			if phase == "held_target" {
				f.status.Axes[0].LogicalTarget = 0.5
			}
			f.axisLag = false
			f.mu.Unlock()
			select {
			case err := <-result:
				if err != nil {
					t.Fatal(err)
				}
			case <-time.After(3000 * time.Millisecond):
				t.Fatal("fresh axis publication did not complete move")
			}
			if phase == "held_target" && f.points[0].Position[0] != 0.5 {
				t.Fatal("first point did not use the fresh held target")
			}
		})
	}
}

func TestJointMoveSlowProfileRetainsItsCompletionWindow(t *testing.T) {
	r, f, in := moveFake(t)
	f.autoComplete = false
	r.mu.Lock()
	r.description.Axes = append([]ipcclient.AxisDescription(nil), r.description.Axes...)
	r.description.Axes[0].MaxVelocity = 0.02
	r.mu.Unlock()
	result := make(chan error, 1)
	go func() { _, err := r.JointMove(context.Background(), in); result <- err }()
	<-f.started
	r.mu.Lock()
	window := r.move.completionBudget
	r.mu.Unlock()
	// Unverified absolute test window in ns for this described slow profile.
	if window < 60000000000 || window > 80000000000 {
		t.Fatalf("completion window=%d ns", window)
	}
	f.mu.Lock()
	f.completeLocked()
	f.mu.Unlock()
	select {
	case err := <-result:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(3000 * time.Millisecond):
		t.Fatal("completion not observed")
	}
}

func TestJointMoveRefusalDiscards(t *testing.T) {
	for _, op := range []string{"prepare_trajectory", "start_trajectory"} {
		t.Run(op, func(t *testing.T) {
			r, f, in := moveFake(t)
			f.refuseMove = op
			state, err := r.JointMove(context.Background(), in)
			jointCode(t, err, "not_ready")
			// A refused upload accepted nothing and moved nothing: it is a refusal
			// of this step and the machine stays armed. A refused Start still
			// Stops and discards, since a handle exists on the adapter.
			wantState, want := "refused", []string{"prepare_trajectory"}
			if op == "start_trajectory" {
				wantState, want = "stopped", []string{"prepare_trajectory", "start_trajectory", "stop", "release", "discard_trajectory"}
			}
			if state.State != wantState || state.Reason != "not_ready" {
				t.Fatal(state)
			}
			if got := moveCalls(f); !reflect.DeepEqual(got, want) {
				t.Fatal(got)
			}
		})
	}
}

func TestJointMoveStopDisarmGrantLossAndLatePrepare(t *testing.T) {
	for _, action := range []string{"stop", "disarm", "grant_loss", "late_prepare", "request_cancel"} {
		t.Run(action, func(t *testing.T) {
			r, f, in := moveFake(t)
			f.autoComplete = false
			if action == "late_prepare" {
				f.prepareWait = make(chan struct{})
			}
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			result := make(chan error, 1)
			go func() { _, err := r.JointMove(ctx, in); result <- err }()
			if action == "late_prepare" {
				<-f.prepareEntered
			} else {
				<-f.started
			}
			switch action {
			case "disarm":
				_, _ = r.Arm(context.Background(), false)
			case "grant_loss":
				f.updates <- control.Renewal{Err: rtRefused("no_grant")}
			case "request_cancel":
				cancel()
			default:
				_, _ = r.Stop(context.Background())
			}
			if action == "late_prepare" {
				close(f.prepareWait)
			}
			select {
			case err := <-result:
				if err == nil {
					t.Fatal("stopped move succeeded")
				}
			case <-time.After(3000 * time.Millisecond):
				t.Fatal("Stop blocked behind move")
			}
			got := moveCalls(f)
			if action == "late_prepare" {
				if !reflect.DeepEqual(got, []string{"prepare_trajectory", "stop", "release", "discard_trajectory"}) {
					t.Fatal(got)
				}
			} else if len(got) != 5 || got[0] != "prepare_trajectory" || got[1] != "start_trajectory" ||
				slices.Index(got, "stop") < 2 || slices.Index(got, "release") != slices.Index(got, "stop")+1 ||
				slices.Index(got, "discard_trajectory") < 2 {
				t.Fatal(got)
			}
			if r.JointJogState().Move.State == "done" {
				t.Fatal("Stop became completion")
			}
			if f.fences[len(f.fences)-1].Generation != 1 {
				t.Fatal("discard adopted new authority")
			}
		})
	}
}

// A running loop is between cycles for moments, not "playing": a move by
// angle or a Cartesian step there is refused before anything is prepared.
func TestJointMoveRefusedWhileALoopRuns(t *testing.T) {
	r, f, in := moveFake(t)
	r.mu.Lock()
	r.loop = &loopRun{mode: "loop"}
	r.session.State = "done"
	r.mu.Unlock()
	in.Revision = r.JointJogState().Revision
	_, err := r.JointMove(context.Background(), in)
	jointCode(t, err, "jog_mode_conflict")
	if len(f.points) != 0 {
		t.Fatalf("prepared %d points while a loop runs", len(f.points))
	}
	r.mu.Lock()
	r.loop = nil
	r.mu.Unlock()
	if _, err := r.JointMove(context.Background(), in); err != nil {
		t.Fatalf("after the loop ended: %v", err)
	}
}

func TestJointMoveNeedsCorrelatedCompletionAndEndpoint(t *testing.T) {
	r, f, in := moveFake(t)
	f.autoComplete = false
	result := make(chan error, 1)
	go func() { _, err := r.JointMove(context.Background(), in); result <- err }()
	<-f.started
	_, err := r.JointMove(context.Background(), in)
	jointCode(t, err, "jog_mode_conflict")
	jog := JointJogIntent{TargetID: in.TargetID, SessionID: in.SessionID, Revision: r.JointJogState().Revision, Axis: 0, Direction: 1, Fraction: 0.1}
	_, err = r.JointJog(context.Background(), "begin", jog)
	jointCode(t, err, "jog_mode_conflict")
	for _, wrong := range []string{"endpoint", "sequence", "handle"} {
		f.mu.Lock()
		f.completeLocked()
		switch wrong {
		case "endpoint":
			f.status.Axes[0].LogicalPosition += 0.01
		case "sequence":
			f.status.Motion.CommandSequence = 43
		case "handle":
			f.status.Motion.TrajectoryID = 18
		}
		reads := f.statusReads
		f.mu.Unlock()
		rtAwait(t, func() bool { f.mu.Lock(); defer f.mu.Unlock(); return f.statusReads >= reads+2 })
		select {
		case err := <-result:
			t.Fatalf("%s completed: %v", wrong, err)
		default:
		}
	}
	f.mu.Lock()
	f.completeLocked()
	f.status.Motion.CommandSequence = 42
	f.status.Motion.TrajectoryID = 17
	f.mu.Unlock()
	select {
	case err := <-result:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(3000 * time.Millisecond):
		t.Fatal("correlated completion not observed")
	}
}

// The several-axis profile: every joint follows one shared progress, so every
// joint starts and finishes on the same sample and none is faster than alone.
func TestJointMovesProfileSynchronisesAxes(t *testing.T) {
	// moveFake describes every axis alike: 2 rad/s, 2 rad/s² (the lower of jog
	// and max acceleration), a 1 ms cycle.
	r, _, _ := moveFake(t)
	d := r.description
	if len(d.Axes) < 3 {
		t.Fatalf("the fake describes %d axes; the test needs three", len(d.Axes))
	}
	long, err := jointMoveProfile(d, JointMoveIntent{Axis: 0, Degrees: 30, Fraction: 0.5}, 0.25)
	if err != nil {
		t.Fatal(err)
	}
	short, err := jointMoveProfile(d, JointMoveIntent{Axis: 1, Degrees: -5, Fraction: 0.5}, 0.1)
	if err != nil {
		t.Fatal(err)
	}
	if len(short) >= len(long) {
		t.Fatalf("the -5° move should be shorter alone: %d vs %d samples", len(short), len(long))
	}
	both, _, err := jointMovesProfile(d, []JointMoveAxis{{Axis: 0, Degrees: 30}, {Axis: 1, Degrees: -5}}, 0.5, []float64{0.25, 0.1}, 0)
	if err != nil {
		t.Fatal(err)
	}
	if len(both) != len(long) {
		t.Fatalf("the joint move lasts as long as its longest joint: %d vs %d samples", len(both), len(long))
	}
	last := both[len(both)-1].Position
	if math.Abs(last[0]-(0.25+30*math.Pi/180)) > 1e-12 || math.Abs(last[1]-(0.1-5*math.Pi/180)) > 1e-12 || last[2] != 0 {
		t.Fatalf("endpoints %v", last)
	}
	if both[0].Position[0] != 0.25 || both[0].Position[1] != 0.1 {
		t.Fatalf("origins %v", both[0].Position)
	}
	// The stretched joint moves on every sample (it is not parked and then
	// rushed) and never faster than it would alone.
	cycle := float64(d.CycleNS) / 1e9
	var peakAlone, peakTogether float64
	for i := 1; i < len(short); i++ {
		peakAlone = math.Max(peakAlone, math.Abs(short[i].Position[1]-short[i-1].Position[1])/cycle)
	}
	stalled := 0
	for i := 1; i < len(both); i++ {
		step := both[i].Position[1] - both[i-1].Position[1]
		if step > 1e-15 {
			t.Fatalf("sample %d: J2 reversed (%g)", i, step)
		}
		if step == 0 {
			stalled++
		}
		peakTogether = math.Max(peakTogether, math.Abs(step)/cycle)
	}
	if peakTogether > peakAlone*(1+1e-9) {
		t.Fatalf("J2 faster together (%g rad/s) than alone (%g rad/s)", peakTogether, peakAlone)
	}
	if stalled > 2 {
		t.Fatalf("J2 parked on %d samples of %d", stalled, len(both))
	}
	// Absolute check on J1's peak: 0.5 of 2 rad/s with the 0.75 reserve is 0.75 rad/s.
	var peakJ1 float64
	for i := 1; i < len(both); i++ {
		peakJ1 = math.Max(peakJ1, math.Abs(both[i].Position[0]-both[i-1].Position[0])/cycle)
	}
	if peakJ1 > 0.75*(1+1e-6) || peakJ1 < 0.7 {
		t.Fatalf("J1 peak %g rad/s", peakJ1)
	}
	for _, bad := range []struct {
		name string
		axes []JointMoveAxis
		code string
	}{
		{"duplicate", []JointMoveAxis{{Axis: 0, Degrees: 10}, {Axis: 0, Degrees: 5}}, "joint_move_invalid"},
		{"empty", nil, "joint_move_invalid"},
		{"zero delta", []JointMoveAxis{{Axis: 0, Degrees: 10}, {Axis: 1, Degrees: 0}}, "joint_move_invalid"},
		{"outside", []JointMoveAxis{{Axis: 0, Degrees: 10}, {Axis: 99, Degrees: 1}}, "invalid_axis_mask"},
	} {
		origins := make([]float64, len(bad.axes))
		_, _, err := jointMovesProfile(d, bad.axes, 0.5, origins, 0)
		jointCode(t, jointJogFailure(err), bad.code)
	}
}

// Several joints in one intent prepare one trajectory under the union mask,
// begin at every joint's held target, complete on every endpoint, and report
// the list in their state.
func TestJointMoveSeveralAxesPrepareOneTrajectory(t *testing.T) {
	r, f, in := moveFake(t)
	f.mu.Lock()
	f.status.Axes[1].LogicalTarget, f.status.Axes[1].LogicalPosition = 0.1, 0.1
	f.mu.Unlock()
	in.Axis, in.Degrees = 0, 0
	in.Axes = []JointMoveAxis{{Axis: 0, Degrees: 30}, {Axis: 1, Degrees: -10}}
	state, err := r.JointMove(context.Background(), in)
	if err != nil || state.State != "done" || state.Axis != 0 || !reflect.DeepEqual(state.Axes, []int{0, 1}) {
		t.Fatalf("state=%+v error=%v", state, err)
	}
	if got := moveCalls(f); !reflect.DeepEqual(got, []string{"prepare_trajectory", "start_trajectory"}) {
		t.Fatal(got)
	}
	if f.mask != 0b11 {
		t.Fatalf("mask %b", f.mask)
	}
	if f.points[0].Position[0] != 0.25 || f.points[0].Position[1] != 0.1 {
		t.Fatalf("origins %v", f.points[0].Position)
	}
	last := f.points[len(f.points)-1].Position
	if math.Abs(last[0]-(0.25+30*math.Pi/180)) > 1e-12 || math.Abs(last[1]-(0.1-10*math.Pi/180)) > 1e-12 {
		t.Fatalf("endpoints %v", last)
	}
	// A joint outside the held mask, or listed twice, is refused before anything moves.
	stale := r.Status(context.Background())
	for _, bad := range [][]JointMoveAxis{{{Axis: 0, Degrees: 5}, {Axis: 9, Degrees: 5}}, {{Axis: 0, Degrees: 5}, {Axis: 0, Degrees: 5}}} {
		again := in
		again.Revision = r.JointJogState().Revision
		again.Axes = bad
		_, err = r.JointMove(context.Background(), again)
		jointCode(t, err, "invalid_axis_mask")
	}
	if r.Status(context.Background()).Session.ID != stale.Session.ID {
		t.Fatal("refusals must not touch the session")
	}
	if got := moveCalls(f); len(got) != 2 {
		t.Fatalf("a refused several-axis move must not prepare: %v", got)
	}
}

// The fraction runs to 100 % of the described maximum (owner decision
// 2026-09-21); the measured 0.75 reserve keeps the profile under the limit
// itself, and anything above 1 is refused before a profile is planned.
func TestJointMoveFractionRunsToTheDescribedMaximum(t *testing.T) {
	r, _, _ := moveFake(t)
	d := r.description
	full, err := jointMoveProfile(d, JointMoveIntent{Axis: 0, Degrees: 30, Fraction: 1}, 0.25)
	if err != nil {
		t.Fatal(err)
	}
	half, err := jointMoveProfile(d, JointMoveIntent{Axis: 0, Degrees: 30, Fraction: 0.5}, 0.25)
	if err != nil {
		t.Fatal(err)
	}
	if len(full) >= len(half) {
		t.Fatalf("100 %% should be quicker than 50 %%: %d vs %d samples", len(full), len(half))
	}
	// moveFake describes 2 rad/s on a 1 ms cycle: no sample step may exceed
	// the maximum under the reserve, 1.5 rad/s.
	limit := d.Axes[0].MaxVelocity * jointMoveVelocityReserve
	for i := 1; i < len(full); i++ {
		if v := math.Abs(full[i].Position[0]-full[i-1].Position[0]) / 0.001; v > limit+1e-9 {
			t.Fatalf("sample %d moves at %.4f rad/s; the reserve bound is %.4f", i, v, limit)
		}
	}
	if _, err := jointMoveProfile(d, JointMoveIntent{Axis: 0, Degrees: 30, Fraction: 1.01}, 0.25); err == nil {
		t.Fatal("a fraction above 1 must be refused")
	} else {
		jointCode(t, jointJogFailure(err), "joint_move_invalid")
	}
}
