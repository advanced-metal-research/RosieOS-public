//go:build linux

package denseexec

import (
	"context"
	"errors"
	"math"
	"slices"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// runClient is the move fake whose native preparation can refuse, as rt-core
// refuses a program outside the machine's limits.
type runClient struct {
	*fakeMoveAdapter
	prepareRefusal string
}

func (c *runClient) PrepareProgram(ctx context.Context, blob []byte) (control.Program, error) {
	if c.prepareRefusal != "" {
		c.mu.Lock()
		c.record("prepare_program")
		c.mu.Unlock()
		return control.Program{}, &control.Rejected{Reason: c.prepareRefusal}
	}
	return c.fakeMoveAdapter.PrepareProgram(ctx, blob)
}

// runFixture: a described machine, Homed, disarmed, every joint at the
// reference plan's first sample; completion tolerance 0.001 rad (0.057°).
func runFixture(t *testing.T) (*RTCoreController, *runClient, RunProgramRequest, []float64) {
	t.Helper()
	r, f, id := rtFake(t)
	_, plan := fixtureBlob(t)
	start := append([]float64(nil), plan.Segments[0].Samples[0].QRad[:]...)
	accel := 2.0
	f.mu.Lock()
	f.description.CycleNS, f.description.MaxTrajectoryPoints = 1000000, 100000
	for i := range f.description.Axes {
		f.description.Axes[i] = ipcclient.AxisDescription{ID: f.description.Axes[i].ID, PositionUnit: "rad", MinPosition: -4, MaxPosition: 4, MaxVelocity: 2, JogAcceleration: 4, MaxAcceleration: &accel, CompletionTolerance: 0.001, CompletionTimeoutNS: 500000000}
	}
	f.status.Motion.Done = true
	for i := range f.status.Axes {
		f.status.Axes[i].LogicalPosition, f.status.Axes[i].LogicalTarget = start[i], start[i]
	}
	f.mu.Unlock()
	c := &runClient{fakeMoveAdapter: &fakeMoveAdapter{fakeRTCore: f, autoComplete: true, prepareEntered: make(chan struct{}), started: make(chan struct{})}}
	r.c = c
	return r, c, RunProgramRequest{LoadRequest: LoadRequest{Identity: id}, Fraction: 0.1}, start
}

func runCalls(c *runClient) []string {
	c.mu.Lock()
	defer c.mu.Unlock()
	var calls []string
	for _, call := range c.calls {
		if slices.Contains([]string{"enable", "arm", "prepare_trajectory", "start_trajectory", "prepare_program", "start_program", "stop"}, call) {
			calls = append(calls, call)
		}
	}
	return calls
}

func runSteps(result *RunProgramResult) []string {
	var steps []string
	for _, step := range result.Steps {
		steps = append(steps, step.Step+":"+step.Outcome)
	}
	return steps
}

func runRefused(t *testing.T, result *RunProgramResult, err error, step, code string) {
	t.Helper()
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != code || result == nil || result.State != "refused" || result.Refusal == nil || result.Refusal.Step != step || result.Refusal.Code != code {
		t.Fatalf("want refusal %s/%s, got result=%+v err=%v", step, code, result, err)
	}
}

// confirmRun is the operator's answer to a confirm: the flags it asked for
// and the joints it showed.
func confirmRun(in RunProgramRequest, asked *RunProgramResult) RunProgramRequest {
	in.ConfirmArm, in.ConfirmMove, in.Confirmed = asked.NeedsArm, asked.NeedsMove, asked.Joints
	return in
}

// Armed at the plan's start with nothing staged: one press loads and plays,
// asks nothing and moves nothing on the way.
func TestRunProgramNotLoadedLoadsAndPlays(t *testing.T) {
	r, c, in, _ := runFixture(t)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	result, err := r.RunProgram(ctx, in)
	if err != nil || result.State != "playing" || result.SessionID == "" {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := runSteps(result); !slices.Equal(got, []string{"start:done", "load:done", "play:done"}) {
		t.Fatal(got)
	}
	if got := runCalls(c); !slices.Equal(got, []string{"enable", "arm", "prepare_program", "start_program"}) {
		t.Fatalf("calls %v", got)
	}
	if s := r.Status(ctx).Session; s.State != "playing" || s.ID != result.SessionID {
		t.Fatalf("session %+v", s)
	}
}

// Disarmed at the start: the first press asks "Arm and run" and sends
// nothing that energises; the confirmed press stages the program before it
// arms (a Load refusal leaves the robot disarmed), then arms and plays.
func TestRunProgramFromDisarmedAsksOnceThenArmsAndPlays(t *testing.T) {
	r, c, in, _ := runFixture(t)
	ctx := context.Background()
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.State != "confirm" || asked.Action != RunProgramArmAction || !asked.NeedsArm || asked.NeedsMove {
		t.Fatalf("result=%+v err=%v", asked, err)
	}
	for _, joint := range asked.Joints {
		if joint.Moves {
			t.Fatalf("%s moves although it is at the start: %+v", joint.ID, joint)
		}
	}
	if got := runCalls(c); len(got) != 0 {
		t.Fatalf("the question sent %v", got)
	}
	result, err := r.RunProgram(ctx, confirmRun(in, asked))
	if err != nil || result.State != "playing" {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := runCalls(c); !slices.Equal(got, []string{"prepare_program", "enable", "arm", "start_program"}) {
		t.Fatalf("calls %v", got)
	}
}

// The owner's replay: the program ran to completion, native retired it and
// the arm stands at the program's end. The next press asks once to move to
// the start (the robot is still armed: no Arm in the question), then moves
// to exactly the first sample, stages the program again and plays it.
func TestRunProgramReplayAfterCompletionMovesReloadsAndPlays(t *testing.T) {
	r, c, in, start := runFixture(t)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	first, err := r.RunProgram(ctx, in)
	if err != nil || first.State != "playing" {
		t.Fatalf("first run: %+v %v", first, err)
	}
	c.mu.Lock()
	c.status.Execution.State = "completed"
	c.mu.Unlock()
	rtAwait(t, func() bool { return r.Status(ctx).Session.State == "done" })
	end := []float64{start[0] + 0.3, start[1] - 0.2}
	c.mu.Lock()
	c.status.Axes[0].LogicalPosition, c.status.Axes[0].LogicalTarget = end[0], end[0]
	c.status.Axes[1].LogicalPosition, c.status.Axes[1].LogicalTarget = end[1], end[1]
	c.calls = nil
	c.mu.Unlock()

	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.State != "confirm" || asked.Action != RunProgramMoveAction || asked.NeedsArm || !asked.NeedsMove {
		t.Fatalf("replay question: %+v %v", asked, err)
	}
	moving := map[string]RecoverJoint{}
	for _, joint := range asked.Joints {
		if joint.Moves {
			moving[joint.ID] = joint
		}
	}
	if len(moving) != 2 || math.Abs(moving["J1"].DeltaDeg-(-0.3*180/math.Pi)) > 1e-9 || math.Abs(moving["J2"].ToDeg-start[1]*180/math.Pi) > 1e-9 {
		t.Fatalf("the question shows %+v", asked.Joints)
	}
	if got := runCalls(c); len(got) != 0 {
		t.Fatalf("the question sent %v", got)
	}
	result, err := r.RunProgram(ctx, confirmRun(in, asked))
	if err != nil || result.State != "playing" || result.SessionID == first.SessionID {
		t.Fatalf("replay: %+v %v", result, err)
	}
	if got := runSteps(result); !slices.Equal(got, []string{"start:done", "move:done", "load:done", "play:done"}) {
		t.Fatal(got)
	}
	if got := runCalls(c); !slices.Equal(got, []string{"prepare_trajectory", "start_trajectory", "prepare_program", "start_program"}) {
		t.Fatalf("calls %v", got)
	}
	c.mu.Lock()
	endpoint := c.points[len(c.points)-1].Position
	c.mu.Unlock()
	for axis := 0; axis < 2; axis++ {
		if math.Abs(endpoint[axis]-start[axis]) > 1e-12 {
			t.Fatalf("J%d moved to %g rad, the program starts at %g rad", axis+1, endpoint[axis], start[axis])
		}
	}
}

// Disarmed away from the start: one question names Arm and the move; the
// confirmed press arms, moves, loads and plays, in that order.
func TestRunProgramDisarmedAwayFromStartArmsMovesLoadsPlays(t *testing.T) {
	r, c, in, start := runFixture(t)
	ctx := context.Background()
	c.mu.Lock()
	c.status.Axes[0].LogicalPosition, c.status.Axes[0].LogicalTarget = start[0]+0.1, start[0]+0.1
	c.mu.Unlock()
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.Action != RunProgramArmMoveAction || !asked.NeedsArm || !asked.NeedsMove {
		t.Fatalf("question: %+v %v", asked, err)
	}
	result, err := r.RunProgram(ctx, confirmRun(in, asked))
	if err != nil || result.State != "playing" {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := runCalls(c); !slices.Equal(got, []string{"enable", "arm", "prepare_trajectory", "start_trajectory", "prepare_program", "start_program"}) {
		t.Fatalf("calls %v", got)
	}
}

// At the start already: no question about a move and no move, even when
// confirm_move is sent.
func TestRunProgramAtStartNeverMoves(t *testing.T) {
	r, c, in, _ := runFixture(t)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	in.ConfirmMove = true
	if result, err := r.RunProgram(ctx, in); err != nil || result.State != "playing" || result.Move != nil {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := runCalls(c); slices.Contains(got, "prepare_trajectory") || slices.Contains(got, "start_trajectory") {
		t.Fatalf("calls %v", got)
	}
}

// A plan Load refuses moves nothing. The identity check runs before the
// question, even away from the start; native preparation refusing the
// program at the start arms nothing and plays nothing.
func TestRunProgramLoadRefusalMovesNothing(t *testing.T) {
	r, c, in, start := runFixture(t)
	ctx := context.Background()
	c.mu.Lock()
	c.status.Axes[0].LogicalPosition = start[0] + 0.5
	c.mu.Unlock()
	wrong := in
	wrong.PlanRevision++
	wrong.ConfirmArm, wrong.ConfirmMove = true, true
	result, err := r.RunProgram(ctx, wrong)
	runRefused(t, result, err, "load", "dense_identity_mismatch")
	if got := runCalls(c); len(got) != 0 {
		t.Fatalf("a refused plan sent %v", got)
	}

	c.mu.Lock()
	c.status.Axes[0].LogicalPosition = start[0]
	c.mu.Unlock()
	c.prepareRefusal = "trajectory_limits: J3 velocity"
	in.ConfirmArm = true
	result, err = r.RunProgram(ctx, in)
	if result == nil || result.State != "refused" || result.Refusal.Step != "load" || err == nil {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	// Load's own failure handling releases the grant it took (stop); nothing
	// energises and nothing moves.
	for _, call := range runCalls(c) {
		if call != "prepare_program" && call != "stop" {
			t.Fatalf("a refused Load went on to %v", runCalls(c))
		}
	}
	if detail := result.Refusal.Detail; !strings.Contains(detail, "Load refused") || !strings.Contains(detail, "trajectory_limits") {
		t.Fatalf("the refusal does not carry the machine's words: %q", detail)
	}
	if r.Status(ctx).Armed != nil && *r.Status(ctx).Armed {
		t.Fatal("a refused Load left the robot armed")
	}
}

// STOP during Go to plan start ends the press: the move is cancelled and
// nothing is loaded or played afterwards.
func TestRunProgramStopMidSequence(t *testing.T) {
	r, c, in, start := runFixture(t)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	c.mu.Lock()
	c.status.Axes[0].LogicalPosition, c.status.Axes[0].LogicalTarget = start[0]+0.4, start[0]+0.4
	c.prepareWait = make(chan struct{})
	c.mu.Unlock()
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.Action != RunProgramMoveAction {
		t.Fatalf("question: %+v %v", asked, err)
	}
	type answer struct {
		result *RunProgramResult
		err    error
	}
	done := make(chan answer, 1)
	go func() {
		result, err := r.RunProgram(ctx, confirmRun(in, asked))
		done <- answer{result, err}
	}()
	<-c.prepareEntered
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	close(c.prepareWait)
	got := <-done
	if got.err == nil || got.result == nil || got.result.State != "refused" || got.result.Refusal.Code != "run_stopped" {
		t.Fatalf("result=%+v err=%v", got.result, got.err)
	}
	calls := runCalls(c)
	if slices.Contains(calls, "prepare_program") || slices.Contains(calls, "start_program") || slices.Contains(calls, "start_trajectory") {
		t.Fatalf("after STOP the press went on: %v", calls)
	}
}

// A confirmation answers the pose it showed: travel since then refuses
// before Arm or the move.
func TestRunProgramStaleConfirmationRefuses(t *testing.T) {
	r, c, in, start := runFixture(t)
	ctx := context.Background()
	c.mu.Lock()
	c.status.Axes[0].LogicalPosition = start[0] + 0.2
	c.mu.Unlock()
	asked, err := r.RunProgram(ctx, in)
	if err != nil || asked.State != "confirm" {
		t.Fatalf("question: %+v %v", asked, err)
	}
	c.mu.Lock()
	c.status.Axes[0].LogicalPosition = start[0] + 0.25
	c.mu.Unlock()
	result, err := r.RunProgram(ctx, confirmRun(in, asked))
	runRefused(t, result, err, "arm", "confirmation_stale")
	if got := runCalls(c); len(got) != 0 {
		t.Fatalf("a stale confirmation sent %v", got)
	}
}
