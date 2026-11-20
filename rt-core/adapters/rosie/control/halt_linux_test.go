//go:build linux

package control

import (
	"context"
	"errors"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

func waitHalt(t *testing.T, f lifecycleFixture, sequence uint64) {
	t.Helper()
	// Unverified harness timeout: positive completion requires the observed receipt and hold.
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	_, err := f.client.Wait(ctx, func(s ipcclient.Status) bool {
		motion, err := f.client.Motion()
		return err == nil && motion.CommandSequence == sequence && motion.Done &&
			s.Armed == 1 && s.AxisEnableMask == 511 && ipcclient.AllOperationEnabled(s)
	})
	if err != nil {
		t.Fatal(err)
	}
}

// Halt receipts retire the executing handle without invalidating the grant;
// a new Start remains possible without another Enable or Arm.
func TestPublicHaltTrajectoryResumeAndRefusals(t *testing.T) {
	f := startLifecycleCore(t)
	api := serveHandleAPI(t, f.controller, nil)
	if capability, ok := Capability(OperationHalt); !ok || capability.State != "implemented" {
		t.Fatal(capability)
	}
	for _, tc := range []struct {
		fence  Fence
		reason string
	}{
		{Fence{}, ReasonNoGrant}, {Fence{f.fence.Session, f.fence.Generation + 1}, ReasonWrongGeneration},
	} {
		api.call(t, controlRequest(tc.fence, OperationHalt), tc.reason)
	}
	_, err := f.client.Halt(context.Background(), f.fence.Generation+1)
	var rejected *ipcclient.RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonWrongGeneration {
		t.Fatal(err)
	}
	program := mustPrepareProgram(t, f, "halt-first")
	if _, err := f.controller.StartProgram(f.fence, program.Identity); err != nil {
		t.Fatal(err)
	}
	response := api.call(t, controlRequest(f.fence, OperationHalt), "")
	if response.Sequence == 0 {
		t.Fatalf("halt receipt: %+v", response)
	}
	waitHalt(t, f, response.Sequence)
	f.controller.mu.Lock()
	state := f.controller.handles[program.handle].State
	grant := f.controller.grant
	f.controller.mu.Unlock()
	if state != HandleRetired || grant.Generation != f.fence.Generation || grant.Session != f.fence.Session {
		t.Fatal(state, grant)
	}
	assertNativeUnknown(t, f.client, program.handle)
	// Idle receipts must not replace the motion command sequence. Wait for the
	// published idle cursor; the independent cyclic observation clock stays live.
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	var before ipcclient.MotionState
	_, err = f.client.Wait(ctx, func(ipcclient.Status) bool {
		before, err = f.client.Motion()
		return err == nil && before.Mode == 0 && before.Done
	})
	if err != nil {
		t.Fatal(err)
	}
	sequence, err := f.controller.Halt(f.fence)
	if err != nil {
		t.Fatal(err)
	}
	after, err := f.client.Motion()
	if after.TimeNS < before.TimeNS {
		t.Fatal("idle observation clock regressed")
	}
	before.TimeNS = after.TimeNS // Compare motion content at the same observation time.
	if err != nil || sequence == 0 || sequence == before.CommandSequence || after != before {
		t.Fatalf("idle receipt changed motion: receipt=%d before=%+v after=%+v err=%v", sequence, before, after, err)
	}
	program = mustPrepareProgram(t, f, "halt-resume")
	if _, err := f.controller.StartProgram(f.fence, program.Identity); err != nil {
		t.Fatal(err)
	}
	stopped, err := f.controller.Stop(f.fence, false)
	if err != nil {
		t.Fatal(err)
	}
	_, err = f.controller.Halt(Fence{stopped.Session, stopped.Generation})
	if err == nil || err.Error() != ReasonInhibited {
		t.Fatalf("inhibited: %v", err)
	}
}

// A cancelled jog generation cannot supply another target; Halt leaves the
// drives holding and a fresh BeginJog can subsequently be stopped urgently.
func TestPublicHaltJogGenerationAndStop(t *testing.T) {
	f := startLifecycleCore(t)
	origin := ipcclient.HostMonotonicNS()
	generation, err := f.controller.BeginJog(f.fence, 3, origin, origin+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	velocity := make([]float64, 9)
	velocity[0] = .02
	velocity[1] = -.01
	if err := f.controller.UpdateJog(f.fence, generation, 1, origin, origin+200_000_000, 3, velocity); err != nil {
		t.Fatal(err)
	}
	// Unverified harness timeout: motion must cross the absolute encoder-count thresholds.
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	_, err = f.client.Wait(ctx, func(s ipcclient.Status) bool {
		return len(s.Axes) == 9 && s.Axes[0].PositionCounts >= 20 && s.Axes[1].PositionCounts <= -10
	})
	if err != nil {
		t.Fatal(err)
	}
	sequence, err := f.controller.Halt(f.fence)
	if err != nil {
		t.Fatal(err)
	}
	sequence, err = f.controller.Halt(f.fence)
	if err != nil {
		t.Fatal(err)
	}
	waitHalt(t, f, sequence)
	if err := f.controller.UpdateJog(f.fence, generation, 2, origin, origin+200_000_000, 3, velocity); err == nil {
		t.Fatal("old jog accepted")
	}
	origin = ipcclient.HostMonotonicNS()
	next, err := f.controller.BeginJog(f.fence, 3, origin, origin+200_000_000)
	if err != nil || next != generation+1 {
		t.Fatal(next, err)
	}
	if _, err := f.controller.Stop(f.fence, false); err != nil {
		t.Fatal(err)
	}
	_, err = f.client.Wait(ctx, func(s ipcclient.Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	if err != nil {
		t.Fatal(err)
	}
}

// A live two-axis trajectory transfers its sampled velocity into controlled
// hold; waiting for encoder displacement prevents an idle Halt from passing.
func TestPublicHaltMovingTrajectory(t *testing.T) {
	f := startLifecycleCore(t)
	end := make([]float64, 9)
	// Command units: 0.04 and -0.02 radians over 2000000000 ns.
	end[0], end[1] = .04, -.02
	handle, err := f.controller.Prepare(f.fence, 3, []ipcclient.Point{
		{Time: 0, Position: make([]float64, 9)}, {Time: 2 * time.Second, Position: end},
	})
	if err != nil {
		t.Fatal(err)
	}
	start, err := f.controller.Start(f.fence, handle)
	if err != nil {
		t.Fatal(err)
	}
	// Unverified harness timeout: both axes must cross absolute count thresholds.
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	before, err := f.client.Wait(ctx, func(s ipcclient.Status) bool {
		motion, err := f.client.Motion()
		return err == nil && motion.CommandSequence == start && motion.TrajectoryID == handle && !motion.Done &&
			len(s.Axes) == 9 && s.Axes[0].PositionCounts >= 40 && s.Axes[1].PositionCounts <= -20
	})
	if err != nil {
		t.Fatal(err)
	}
	sequence, err := f.controller.Halt(f.fence)
	if err != nil {
		t.Fatal(err)
	}
	waitHalt(t, f, sequence)
	_, err = f.client.Wait(ctx, func(s ipcclient.Status) bool {
		return s.Axes[0].PositionCounts-before.Axes[0].PositionCounts >= 1 &&
			s.Axes[1].PositionCounts-before.Axes[1].PositionCounts <= -1
	})
	if err != nil {
		t.Fatal("ramp did not continue both axes toward hold:", err)
	}
	assertNativeUnknown(t, f.client, handle)
}
