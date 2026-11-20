//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"testing"
	"time"
)

func TestDiscardPreparedHandleIsIdempotentAndReclaimsCapacity(t *testing.T) {
	c := startCore(t, "simulation.json").client
	arm(t, c, 3)
	for id := uint64(1301); id < 1310; id++ {
		handle := prepareStationary(t, c, id)
		for attempt := 0; attempt < 2; attempt++ {
			sequence, err := c.DiscardTrajectory(context.Background(), handle)
			if err != nil || sequence == 0 {
				t.Fatalf("discard %d attempt %d: seq=%d err=%v", handle, attempt, sequence, err)
			}
		}
		requireUnknownHandle(t, c, handle)
	}
	state, err := c.Status()
	if err != nil || state.Armed != 1 || state.AxisEnableMask != 3 {
		t.Fatalf("Discard revoked independent authority: %+v %v", state, err)
	}
}

func TestDiscardActiveIsRejectedWithoutCancellingEitherPlan(t *testing.T) {
	c := startCore(t, "simulation.json").client
	arm(t, c, 3)
	wait(t, c, allOperationEnabled)
	handle := prepareStationary(t, c, 1401)
	other := prepareStationary(t, c, 1402)
	sequence, err := c.StartTrajectory(context.Background(), handle)
	if err != nil {
		t.Fatal(err)
	}
	_, err = c.DiscardTrajectory(context.Background(), handle)
	var rejected *RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonModeConflict || rejected.Result.Handle != handle || rejected.Result.Operation != MsgTrajectoryDiscard {
		t.Fatalf("active Discard result: %v", err)
	}
	if _, err := c.DiscardTrajectory(context.Background(), 9999); err != nil {
		t.Fatalf("unknown Discard must be idempotent: %v", err)
	}
	after := monotonicNS()
	for deadline := time.Now().Add(500 * time.Millisecond); ; {
		motion, err := c.Motion()
		if err == nil && motion.TrajectoryID == handle && motion.CommandSequence == sequence && motion.TimeNS >= after {
			if motion.Done {
				t.Fatalf("Discard changed active execution: %+v", motion)
			}
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("missing correlated post-Discard observation: %+v %v", motion, err)
		}
		time.Sleep(5 * time.Millisecond)
	}
	if _, err := legacyAbort(t, c, handle, 0, 0); err != nil {
		t.Fatal(err)
	}
	requireAbortedMotion(t, c, handle)
	if _, err := c.StartTrajectory(context.Background(), other); err != nil {
		t.Fatalf("Discard invalidated another preparation: %v", err)
	}
}

func TestMalformedDiscardAndStartFramesAreRejected(t *testing.T) {
	c := startCore(t, "simulation.json").client
	arm(t, c, 3)
	handle := prepareStationary(t, c, 1501)
	// Independent ABI fixture: extra words from the retired 16-byte Start
	// layout must not be accepted by either generated handle operation.
	for _, operation := range []uint16{MsgTrajectoryDiscard, MsgTrajectoryStart} {
		payload := make([]byte, 16)
		wire.PutUint64(payload, handle)
		seq, err := c.command(context.Background(), operation, payload)
		mustSubmit(t, seq, err)
		result, err := c.WaitResult(context.Background(), seq)
		var rejected *RejectedError
		if !errors.As(err, &rejected) || result.Reason != ReasonInvalidCommand {
			t.Fatalf("oversized handle operation %#x: %+v %v", operation, result, err)
		}
	}
	// The current private command-envelope policy revokes on malformed size.
	// Verify its inhibition explicitly rather than treating rejection as an
	// unchanged authority grant. Generated valid frames still work afterward.
	wait(t, c, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	arm(t, c, 3)
	requireUnknownHandle(t, c, handle)
	fresh := prepareStationary(t, c, 1502)
	if _, err := c.StartTrajectory(context.Background(), fresh); err != nil {
		t.Fatalf("valid generated Start frame rejected after recovery: %v", err)
	}
}
