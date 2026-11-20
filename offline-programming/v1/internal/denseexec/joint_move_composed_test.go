//go:build linux

package denseexec

import (
	"context"
	"math"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// TestJointMoveComposed checks +30 degrees and return through the public SDK
// against a simulation-only description; no bench profile or endpoint is used.
func TestJointMoveComposed(t *testing.T) {
	c, d, _, socket := rtComposedFixture(t, func(config map[string]any) {
		limits := config["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
		// Unverified simulation geometry for a 30-degree test, in rad and rad/s.
		limits["min"], limits["max"], limits["max_velocity"] = -1.0, 1.0, 1.0
	})
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:joint-move"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	session := r.Status(ctx).Session.ID
	heartbeatDone := make(chan struct{})
	heartbeatCtx, heartbeatCancel := context.WithCancel(ctx)
	defer func() { heartbeatCancel(); <-heartbeatDone }()
	go func() {
		defer close(heartbeatDone)
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-heartbeatCtx.Done():
				return
			case <-ticker.C:
				_, _ = r.Heartbeat(session)
			}
		}
	}()
	before, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	for _, degrees := range []float64{30, -30} {
		start, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		in := JointMoveIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Axis: 0, Degrees: degrees, Fraction: 0.5}
		state, err := r.JointMove(ctx, in)
		if err != nil || state.State != "done" {
			observed, _ := c.Status(ctx)
			t.Logf("public core=%+v motion=%+v execution=%+v", observed.Core, observed.Motion, observed.Execution)
			t.Fatalf("move=%+v error=%v", state, err)
		}
		after, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		if math.Abs(after.Axes[0].LogicalPosition-start.Axes[0].LogicalTarget-degrees*math.Pi/180) > 0.0002 {
			t.Fatalf("endpoint=%g", after.Axes[0].LogicalPosition)
		}
		if !after.Motion.Done || after.Motion.State != 4 || after.Core.Armed != 1 {
			t.Fatalf("motion=%+v", after.Motion)
		}
		for i := 1; i < len(after.Axes); i++ {
			if after.Axes[i].PositionCounts != before.Axes[i].PositionCounts {
				t.Fatalf("axis %d moved", i)
			}
		}
	}
	after, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(after.Axes[0].LogicalPosition-before.Axes[0].LogicalTarget) > 0.0002 {
		t.Fatal("return endpoint not observed")
	}
}
