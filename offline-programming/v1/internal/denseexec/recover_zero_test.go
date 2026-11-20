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

// recoverClient is the move fake with the reset fake's native evidence: an
// advancing same-host clock, lease and grant from the fake's own grant, and
// a reset that native refuses fault_persists while the drive alarm stays.
type recoverClient struct {
	*fakeMoveAdapter
	persists bool
	foreign  bool
}

func (c *recoverClient) Status(ctx context.Context) (control.Status, error) {
	c.mu.Lock()
	c.status.TimeNS += 1000000
	c.status.Motion.TimeNS = c.status.TimeNS
	for i := range c.status.Axes {
		c.status.Axes[i].TimeNS = c.status.TimeNS
	}
	s := c.status
	s.Axes = append([]ipcclient.LogicalAxisStatus(nil), s.Axes...)
	s.Execution.Handles = append([]control.HandleRecord(nil), s.Execution.Handles...)
	s.Grant = &ipcclient.GrantObservation{NowHostNs: s.TimeNS}
	if c.grant.Session != "" || c.foreign {
		s.Grant.GrantActive, s.Core.LeaseValid = 1, 1
	} else {
		s.Core.LeaseValid = 0
	}
	c.mu.Unlock()
	return s, nil
}

func (c *recoverClient) Command(ctx context.Context, q *control.Request) (control.Response, error) {
	if q.Operation == "reset_fault" {
		c.mu.Lock()
		persists := c.persists
		if persists {
			c.record("reset_fault")
		}
		c.grant = control.Grant{}
		if !persists {
			for i := range c.status.Axes {
				c.status.Axes[i].DriveAlarm, c.status.Axes[i].DriveAlarmText = "", ""
			}
		}
		c.mu.Unlock()
		if persists {
			return control.Response{Operation: "reset_fault", Sequence: 1}, &control.Rejected{Reason: "fault_persists"}
		}
	}
	return c.fakeMoveAdapter.Command(ctx, q)
}

// recoverFixture: the 1430 cell after the 12:19 stall, disarmed, OLP latched on an
// unconfirmed Stop, J5 faulted with Er41.1 (drive_readiness,
// reset_after_condition_clears), J1 at +17.19°, J2 at -11.46°, J5 at +37.4°.
func recoverFixture(t *testing.T) (*RTCoreController, *recoverClient) {
	t.Helper()
	r, f, _ := rtFake(t)
	pin := strings.Repeat("d", 64)
	r.cfg.Binding.ConfigurationSHA256 = pin
	accel := 2.0
	f.mu.Lock()
	f.description.ConfigurationSHA256 = pin
	f.description.CycleNS, f.description.MaxTrajectoryPoints = 1000000, 100000
	for i := range f.description.Axes {
		f.description.Axes[i] = ipcclient.AxisDescription{ID: f.description.Axes[i].ID, PositionUnit: "rad", MinPosition: -2, MaxPosition: 2, MaxVelocity: 2, JogAcceleration: 4, MaxAcceleration: &accel, CompletionTolerance: 0.001, CompletionTimeoutNS: 500000000}
	}
	f.status.Core.ConfigurationSHA256, f.status.Core.ConfigurationEpoch, f.status.Core.HomeEpoch = pin, 1, 7
	f.status.TimeNS = 1000000000
	f.status.Motion.TimeNS, f.status.Motion.Done = f.status.TimeNS, true
	for axis, rad := range map[int]float64{0: 0.3, 1: -0.2, 4: 37.4 * math.Pi / 180} {
		f.status.Axes[axis].LogicalPosition, f.status.Axes[axis].LogicalTarget = rad, rad
	}
	f.status.Core.SafetyFaultMask, f.status.Core.ExecutionFaultReasons = 16, 128
	f.status.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 4, Name: "drive_readiness", AxisMask: 16, Recovery: "reset_after_condition_clears"}}}
	f.status.Axes[4].Readiness, f.status.Axes[4].DriveAlarm, f.status.Axes[4].DriveAlarmText = "faulted", "Er41.1", "Motor over-temperature due to locked-  rotor"
	f.mu.Unlock()
	m := &fakeMoveAdapter{fakeRTCore: f, autoComplete: true, prepareEntered: make(chan struct{}), started: make(chan struct{})}
	c := &recoverClient{fakeMoveAdapter: m}
	r.c = c
	r.mu.Lock()
	r.stopUncertain, r.reason = true, "operator_stop: control_session_stale"
	r.mu.Unlock()
	return r, c
}

func recoverCalls(c *recoverClient, names ...string) []string {
	c.mu.Lock()
	defer c.mu.Unlock()
	var calls []string
	for _, call := range c.calls {
		if slices.Contains(names, call) {
			calls = append(calls, call)
		}
	}
	return calls
}

func recoverRefusal(t *testing.T, result *RecoverToZeroResult, err error, step, code string) string {
	t.Helper()
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != code || result == nil || result.State != "refused" || result.Refusal == nil || result.Refusal.Step != step || result.Refusal.Code != code {
		t.Fatalf("want refusal %s/%s, got result=%+v err=%v", step, code, result, err)
	}
	return result.Refusal.Detail
}

var motionOps = []string{"enable", "arm", "home", "prepare_trajectory", "start_trajectory", "prepare_program", "start_program"}

// The the 1430 cell incident state, J5 cooled: the first press confirms the latched
// Stop from a fresh status, resets the drive fault and stops at the
// confirmation without enabling anything; the confirmed second press arms
// and runs All joints to 0° as one move to exact zero.
func TestRecoverToZeroFromIncidentState(t *testing.T) {
	r, c := recoverFixture(t)
	ctx := context.Background()
	result, err := r.RecoverToZero(ctx, RecoverToZeroRequest{Fraction: 0.1})
	if err != nil || result.State != "confirm_arm" || result.Action != "Arm and move all joints to zero" {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	var steps []string
	for _, step := range result.Steps {
		steps = append(steps, step.Step+":"+step.Outcome)
	}
	if !slices.Equal(steps, []string{"start:done", "confirm_stop:done", "reset:done", "arm:needs_confirmation"}) {
		t.Fatal(steps)
	}
	if got := recoverCalls(c, append(motionOps, "stop")...); len(got) != 0 {
		t.Fatal("first press moved, enabled or stopped:", got)
	}
	if got := recoverCalls(c, "reset_fault"); len(got) != 1 {
		t.Fatal("reset count", got)
	}
	if body := r.Status(ctx); body.RTCore.StopUncertain || body.RTCore.Inhibited || body.RTCore.RecoveryAction != "" {
		t.Fatalf("latch not cleared: %+v", body.RTCore)
	}
	j5 := result.Joints[4]
	if j5.ID != "J5" || math.Abs(j5.FromDeg-37.4) > 1e-9 || j5.DeltaDeg != -j5.FromDeg || j5.ToDeg != 0 || !j5.Moves || result.Joints[2].Moves {
		t.Fatalf("confirmation joints %+v", result.Joints)
	}

	result, err = r.RecoverToZero(ctx, RecoverToZeroRequest{Fraction: 0.1, ConfirmArm: true, Confirmed: result.Joints})
	if err != nil || result.State != "done" || result.Move == nil || result.Move.State != "done" {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := recoverCalls(c, "reset_fault", "enable", "arm", "prepare_trajectory", "start_trajectory", "home"); !slices.Equal(got, []string{"reset_fault", "enable", "arm", "prepare_trajectory", "start_trajectory"}) {
		t.Fatal(got)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.mask != 1|2|16 {
		t.Fatalf("move axis mask %b; want J1, J2 and J5 only", c.mask)
	}
	end := c.points[len(c.points)-1].Position
	for _, axis := range []int{0, 1, 4} {
		if math.Abs(end[axis]) > 1e-12 {
			t.Fatalf("J%d ends at %g rad, not 0", axis+1, end[axis])
		}
	}
}

// J5 still hot: the reset is refused, the press stops and says so in the
// drive's words, never enables or arms, and leaves no Stop latch behind.
func TestRecoverToZeroStopsOnPersistingDriveAlarm(t *testing.T) {
	r, c := recoverFixture(t)
	c.persists = true
	result, err := r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: 0.1, ConfirmArm: true})
	detail := recoverRefusal(t, result, err, "reset", "fault_persists")
	if detail != "J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then press again" {
		t.Fatal(detail)
	}
	if got := recoverCalls(c, motionOps...); len(got) != 0 {
		t.Fatal(got)
	}
	if got := recoverCalls(c, "reset_fault"); len(got) != 1 {
		t.Fatal("reset retried in one press:", got)
	}
	if body := r.Status(context.Background()); body.RTCore.StopUncertain || body.RTCore.Inhibited {
		t.Fatalf("refused reset latched recovery: %+v", body.RTCore)
	}
}

// Faults whose recovery is a new Home or a restart, invalid Home, another
// controller's lease and a Stop the servo host cannot confirm all stop the
// press before any reset, Arm or Home.
func TestRecoverToZeroRefusals(t *testing.T) {
	for _, tc := range []struct {
		name, step, code, words string
		edit                    func(*RTCoreController, *recoverClient)
	}{
		{"rehome fault", "reset", "fault_not_resettable", "J1 coordinate_reference needs a new Home", func(_ *RTCoreController, c *recoverClient) {
			c.status.Core.Recovery.Faults = append(c.status.Core.Recovery.Faults, ipcclient.FaultRecovery{Bit: 1, Name: "coordinate_reference", AxisMask: 1, Recovery: "rehome_required"})
		}},
		{"home invalid", "home", "home_invalid", "Home is not valid on J3. Recovery never re-homes", func(_ *RTCoreController, c *recoverClient) {
			c.status.Core.HomeValidMask &^= 4
		}},
		{"stop not confirmable", "confirm_stop", "stop_unconfirmed", "the servo host still shows a lease or grant", func(_ *RTCoreController, c *recoverClient) {
			c.foreign = true
		}},
		{"foreign holder", "control", "control_held_elsewhere", "Another controller holds the robot", func(r *RTCoreController, c *recoverClient) {
			r.stopUncertain = false
			c.foreign = true
		}},
		{"speed missing", "start", "recover_fraction_invalid", "Choose a speed", nil},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, c := recoverFixture(t)
			fraction := 0.1
			if tc.edit != nil {
				c.mu.Lock()
				r.mu.Lock()
				tc.edit(r, c)
				r.mu.Unlock()
				c.mu.Unlock()
			} else {
				fraction = 0
			}
			result, err := r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: fraction, ConfirmArm: true})
			if detail := recoverRefusal(t, result, err, tc.step, tc.code); !strings.Contains(detail, tc.words) {
				t.Fatal(detail)
			}
			if got := recoverCalls(c, append(motionOps, "reset_fault")...); len(got) != 0 {
				t.Fatal(got)
			}
		})
	}
}

// A confirmation that no longer describes the pose does not arm; an armed
// machine moves without a confirmation; a disarmed machine at zero is left
// disarmed.
func TestRecoverToZeroConfirmationArmedAndAtZero(t *testing.T) {
	r, c := recoverFixture(t)
	first, err := r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: 0.1})
	if err != nil || first.State != "confirm_arm" {
		t.Fatal(first, err)
	}
	stale := append([]RecoverJoint(nil), first.Joints...)
	stale[1].FromDeg += 1
	result, err := r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: 0.1, ConfirmArm: true, Confirmed: stale})
	if detail := recoverRefusal(t, result, err, "arm", "confirmation_stale"); !strings.Contains(detail, "J2 by 1.00°") {
		t.Fatal(detail)
	}
	if got := recoverCalls(c, motionOps...); len(got) != 0 {
		t.Fatal(got)
	}

	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	result, err = r.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: 0.25})
	if err != nil || result.State != "done" {
		t.Fatalf("armed press: %+v %v", result, err)
	}

	zero, z := recoverFixture(t)
	z.mu.Lock()
	for i := range z.status.Axes {
		z.status.Axes[i].LogicalPosition, z.status.Axes[i].LogicalTarget = 0.0001, 0.0001
	}
	z.mu.Unlock()
	result, err = zero.RecoverToZero(context.Background(), RecoverToZeroRequest{Fraction: 0.1})
	if err != nil || result.State != "at_zero" {
		t.Fatalf("at zero: %+v %v", result, err)
	}
	if got := recoverCalls(z, motionOps...); len(got) != 0 {
		t.Fatal("arming a robot already at zero:", got)
	}
}
