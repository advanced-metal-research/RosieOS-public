//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"testing"
	"time"
)

// These frames intentionally exercise legacy native commands that have no
// convenience method. Their layout is an independent regression fixture for
// the current v1 ABI, not an additional product encoder.
func legacyAbort(t *testing.T, c *Client, handle uint64, flags, reserved uint32) (CommandResult, error) {
	t.Helper()
	payload := make([]byte, 16)
	wire.PutUint64(payload, handle)
	wire.PutUint32(payload[8:], flags)
	wire.PutUint32(payload[12:], reserved)
	sequence, err := c.command(context.Background(), 0x0123, payload)
	if err != nil {
		t.Fatal(err)
	}
	return c.WaitResult(context.Background(), sequence)
}

func prepareStationary(t *testing.T, c *Client, id uint64) uint64 {
	t.Helper()
	handle, err := c.PrepareTrajectory(context.Background(), id, 3, []Point{
		{Time: 0, Position: []float64{0, 0}},
		{Time: time.Second, Position: []float64{0, 0}},
	})
	if err != nil {
		t.Fatal(err)
	}
	return handle
}

func requireUnknownHandle(t *testing.T, c *Client, handle uint64) {
	t.Helper()
	_, err := c.StartTrajectory(context.Background(), handle)
	var rejected *RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonUnknownHandle {
		t.Fatalf("cancelled handle %d must be unstartable: %v", handle, err)
	}
}

func requireAbortedMotion(t *testing.T, c *Client, handle uint64) {
	t.Helper()
	deadline := time.Now().Add(750 * time.Millisecond)
	for {
		motion, err := c.Motion()
		if err == nil && motion.TrajectoryID == handle && motion.Done && motion.State == 5 {
			return
		}
		if time.Now().After(deadline) {
			t.Fatalf("active Abort did not reach the cyclic owner: %+v %v", motion, err)
		}
		time.Sleep(5 * time.Millisecond)
	}
}

func TestAbortPreparedHandleCannotStartAndReclaimsStorage(t *testing.T) {
	c := startCore(t, "simulation.json").client
	arm(t, c, 3)
	// More than the three resident slots proves cancellation reclaims storage,
	// without a global Stop or a completed execution supplying the reclamation.
	for id := uint64(701); id < 708; id++ {
		handle := prepareStationary(t, c, id)
		result, err := legacyAbort(t, c, handle, 0, 0)
		if err != nil || result.Result != ResultAccepted || result.Handle != handle {
			t.Fatalf("prepared Abort result: %+v %v", result, err)
		}
		requireUnknownHandle(t, c, handle)
		state, err := c.Status()
		if err != nil || state.Armed != 1 || state.AxisEnableMask != 3 {
			t.Fatalf("prepared Abort must not revoke independent motion authority: %+v %v", state, err)
		}
	}
}

func TestAbortAllPreparedHandlesIsIdempotent(t *testing.T) {
	c := startCore(t, "simulation.json").client
	arm(t, c, 3)
	for id := uint64(801); id < 804; id++ {
		prepareStationary(t, c, id)
	}
	for attempt := 0; attempt < 2; attempt++ {
		result, err := legacyAbort(t, c, 0, 0, 0)
		if err != nil || result.Result != ResultAccepted {
			t.Fatalf("Abort-all result: %+v %v", result, err)
		}
	}
	for id := uint64(801); id < 804; id++ {
		requireUnknownHandle(t, c, id)
		prepareStationary(t, c, id+100)
	}
}

func TestMalformedAbortPreservesPreparedHandle(t *testing.T) {
	for _, field := range []string{"flags", "reserved"} {
		t.Run(field, func(t *testing.T) {
			c := startCore(t, "simulation.json").client
			arm(t, c, 3)
			handle := prepareStationary(t, c, 901)
			var flags, reserved uint32
			if field == "flags" {
				flags = 1
			} else {
				reserved = 1
			}
			result, err := legacyAbort(t, c, handle, flags, reserved)
			var rejected *RejectedError
			if !errors.As(err, &rejected) || result.Reason != ReasonInvalidCommand {
				t.Fatalf("malformed Abort must be explicitly rejected: %+v %v", result, err)
			}
			if _, err := c.StartTrajectory(context.Background(), handle); err != nil {
				t.Fatalf("malformed Abort changed the prepared handle: %v", err)
			}
		})
	}
}

func TestAbortActivePreservesOtherPreparedPlanAndAllowsIDReuse(t *testing.T) {
	c := startCore(t, "simulation.json").client
	arm(t, c, 3)
	handle := prepareStationary(t, c, 1101)
	other := prepareStationary(t, c, 1102)
	if _, err := c.StartTrajectory(context.Background(), handle); err != nil {
		t.Fatal(err)
	}
	if _, err := legacyAbort(t, c, handle, 0, 0); err != nil {
		t.Fatal(err)
	}
	requireAbortedMotion(t, c, handle)
	if _, err := c.StartTrajectory(context.Background(), other); err != nil {
		t.Fatalf("targeted Abort cancelled another prepared plan: %v", err)
	}
	if _, err := legacyAbort(t, c, other, 0, 0); err != nil {
		t.Fatal(err)
	}
	requireAbortedMotion(t, c, other)
	// Retired storage is reclaimed by the helper; a reused handle must not
	// inherit the original slot's pending cancellation.
	prepareStationary(t, c, handle)
	if _, err := c.StartTrajectory(context.Background(), handle); err != nil {
		t.Fatal(err)
	}
	time.Sleep(150 * time.Millisecond)
	motion, err := c.Motion()
	if err != nil || motion.TrajectoryID != handle || motion.Done {
		t.Fatalf("a cancelled incarnation leaked into the reused handle: %+v %v", motion, err)
	}
}

func TestLegacyJogStopAcknowledgesGlobalCancellation(t *testing.T) {
	for _, tc := range []struct {
		name  string
		flags uint32
	}{{"stop", 2}, {"stop_with_quick_stop", 6}} {
		t.Run(tc.name, func(t *testing.T) {
			c := startCore(t, "simulation.json").client
			arm(t, c, 3)
			handle := prepareStationary(t, c, 1001)
			payload := make([]byte, 144)
			wire.PutUint32(payload[4:], tc.flags)
			sequence, err := c.command(context.Background(), 0x0130, payload)
			mustSubmit(t, sequence, err)
			result, err := c.WaitResult(context.Background(), sequence)
			if err != nil || result.Result != ResultAccepted || result.Reason != ReasonNone {
				t.Fatalf("legacy jog Stop was applied but not acknowledged: %+v %v", result, err)
			}
			wait(t, c, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
			arm(t, c, 3)
			requireUnknownHandle(t, c, handle)
		})
	}
}

func TestLegacyJogRejectsUnsupportedStopEncodings(t *testing.T) {
	for _, tc := range []struct {
		name  string
		flags uint32
	}{{"quick_stop_without_stop", 4}, {"active_and_stop", 3}, {"unknown_bit", 10}} {
		t.Run(tc.name, func(t *testing.T) {
			c := startCore(t, "simulation.json").client
			arm(t, c, 3)
			payload := make([]byte, 144)
			wire.PutUint32(payload[4:], tc.flags)
			sequence, err := c.command(context.Background(), 0x0130, payload)
			mustSubmit(t, sequence, err)
			result, err := c.WaitResult(context.Background(), sequence)
			var rejected *RejectedError
			if !errors.As(err, &rejected) || result.Reason != ReasonInvalidCommand {
				t.Fatalf("unsupported jog flags need an invalid-command result: %+v %v", result, err)
			}
			wait(t, c, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
		})
	}
}
