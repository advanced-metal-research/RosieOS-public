//go:build linux

package denseexec

import (
	"context"
	"math"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// TestJointJogComposed requires native position and closed-generation evidence;
// an SDK send alone cannot pass the integration contract.
func TestJointJogComposed(t *testing.T) {
	c, d, _, socket := rtComposedFixture(t)
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:joint-jog"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	before, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	in := JointJogIntent{TargetID: "bench", SessionID: r.Status(ctx).Session.ID, Axis: 0, Direction: 1, Fraction: 0.1}
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	// Unverified absolute observation bound; J1 must move 0.001 logical units
	// and remain within 0.5 logical units of the starting position.
	deadline := time.NewTimer(3000 * time.Millisecond)
	defer deadline.Stop()
	for {
		s, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		distance := s.Axes[0].LogicalPosition - before.Axes[0].LogicalPosition
		if distance > 0.5 {
			t.Fatalf("J1 exceeded bound: %f", distance)
		}
		if distance >= 0.001 {
			break
		}
		select {
		case <-deadline.C:
			t.Fatal("J1 did not move")
		case <-ticker.C:
			if _, err := r.JointJog(ctx, "update", in); err != nil {
				t.Fatal(err)
			}
		}
	}
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
	var settledNS uint64
	var settledCounts int32
	rtAwait(t, func() bool {
		s, err := c.Status(ctx)
		if err != nil || s.Jog == nil || s.Jog.Open != 0 || !s.Motion.Done {
			return false
		}
		_, _ = r.Heartbeat(in.SessionID)
		a := s.Axes[0]
		// Unverified simulation standstill contract: unchanged counts across
		// 100000000 ns of distinct native publications, with the generation closed.
		if settledNS == 0 || a.PositionCounts != settledCounts {
			settledNS, settledCounts = a.TimeNS, a.PositionCounts
		}
		return a.TimeNS >= settledNS+100000000
	})
	after, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(after.Axes[0].LogicalPosition-before.Axes[0].LogicalPosition) > 0.5 {
		t.Fatal("release exceeded distance bound")
	}
	for i := 1; i < len(after.Axes); i++ {
		if after.Axes[i].PositionCounts != before.Axes[i].PositionCounts {
			t.Fatalf("axis %d moved", i)
		}
	}
	if state := r.JointJogState(); state.Active || state.Reason != "" {
		t.Fatal(state)
	}
}
