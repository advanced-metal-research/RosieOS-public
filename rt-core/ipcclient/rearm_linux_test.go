//go:build linux

package ipcclient

import (
	"context"
	"math"
	"testing"
	"time"
)

// This test exercises the production cyclic output/rate guard with a simulator
// plant displacement, rather than resetting guard state in the test itself.
func TestDisabledDisplacementRearmsAtObservedPosition(t *testing.T) {
	f := startCoreWithArgs(t, "simulation.json", "--sim-disabled-displacement-counts", "2000")
	c := f.client
	arm(t, c, 3)
	wait(t, c, func(s Status) bool { return s.Axes[0].Statusword&0x6f == 0x27 })
	seq, err := c.Stop(context.Background())
	mustSubmit(t, seq, err)
	wait(t, c, func(s Status) bool { return s.Armed == 0 && s.Axes[0].PositionCounts == 2000 })

	for attempt := 1; attempt <= 3; attempt++ {
		seq, err = c.Enable(context.Background(), 3)
		mustSubmit(t, seq, err)
		seq, err = c.Arm(context.Background())
		mustSubmit(t, seq, err)
		rearmed := wait(t, c, func(s Status) bool {
			return s.SafetyFaultMask != 0 || (s.Armed == 1 && s.Axes[0].Statusword&0x6f == 0x27)
		})
		if rearmed.SafetyFaultMask != 0 || rearmed.Axes[0].PositionCounts != 2000 {
			t.Errorf("re-arm attempt %d after valid disabled displacement failed: %+v", attempt, rearmed)
		}
		if attempt < 3 {
			seq, err = c.Stop(context.Background())
			mustSubmit(t, seq, err)
			// Reset must not cause output motion.
			seq, err = c.FaultReset(context.Background(), 3)
			mustSubmit(t, seq, err)
			wait(t, c, func(s Status) bool { return s.Armed == 0 && s.SafetyFaultMask == 0 })
		}
	}
	if t.Failed() {
		return
	}
	// Enabling alone must not snap back or resume prior motion. Observe over
	// multiple published status samples, not just the accepted Arm command.
	for deadline := time.Now().Add(250 * time.Millisecond); time.Now().Before(deadline); {
		s, err := c.Status()
		if err != nil || s.SafetyFaultMask != 0 || s.Axes[0].PositionCounts != 2000 {
			t.Fatalf("re-armed hold moved or faulted: %+v, %v", s, err)
		}
		time.Sleep(10 * time.Millisecond)
	}
	q := float64(2000) * 2 * math.Pi / 131072
	seq, err = c.Trajectory(context.Background(), 8404, 3, []Point{
		{Time: 0, Position: []float64{q, 0}},
		{Time: time.Second, Position: []float64{q + 0.01, 0}},
	})
	mustSubmit(t, seq, err)
	moved := wait(t, c, func(s Status) bool { return s.SafetyFaultMask != 0 || s.Axes[0].PositionCounts >= 2208 })
	if moved.SafetyFaultMask != 0 {
		t.Fatalf("fresh trajectory after re-arm faulted: %+v", moved)
	}
}

// A trustworthy exterior displacement may re-arm at its observed count; only
// an explicit inward request can change the held target toward the interval.
func TestDisabledDisplacementOutsideLimitsRearmsForInwardMotion(t *testing.T) {
	f := startCoreWithArgs(t, "simulation.json", "--sim-disabled-displacement-counts", "60000")
	c := f.client
	arm(t, c, 3)
	wait(t, c, func(s Status) bool { return s.Axes[0].Statusword&0x6f == 0x27 })
	seq, err := c.Stop(context.Background())
	mustSubmit(t, seq, err)
	wait(t, c, func(s Status) bool { return s.Armed == 0 && s.Axes[0].PositionCounts == 60000 })
	seq, err = c.Enable(context.Background(), 3)
	mustSubmit(t, seq, err)
	seq, err = c.Arm(context.Background())
	mustSubmit(t, seq, err)
	held := wait(t, c, func(s Status) bool { return s.Armed == 1 && s.Axes[0].Statusword&0x6f == 0x27 })
	if held.ExecutionFaultReasons != 0 || held.Axes[0].PositionCounts != 60000 {
		t.Fatalf("exterior feedback was not held: %+v", held)
	}
	seq, err = c.Jog(context.Background(), 1, []float64{.02, 0}, 200*time.Millisecond)
	if r := limitHoldResult(t, c, seq, err); r.Reason != ReasonOutsideLimitsOutward {
		t.Fatal(r)
	}
	seq, err = c.Jog(context.Background(), 1, []float64{-.02, 0}, 200*time.Millisecond)
	if r := limitHoldResult(t, c, seq, err); r.Result != ResultAccepted {
		t.Fatal(r)
	}
	inward := wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts <= 59990 })
	if inward.Armed != 1 || inward.ExecutionFaultReasons != 0 {
		t.Fatal(inward)
	}
}
