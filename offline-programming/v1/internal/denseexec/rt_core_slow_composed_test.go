//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// TestReviewSingleSlowStatusComposed expires exactly one HTTP status request
// while native renewals succeed; retry and fresh motion retain the same fence.
func TestReviewSingleSlowStatusComposed(t *testing.T) {
	c, d, _, socket, counts := delayedJointFixture(t, true)
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:slow"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	// Unverified test watchdog: 30000 ms; expired request and native receipts pass.
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	fence := c.Fence()
	session := r.statusSnapshot().Session.ID
	done := make(chan struct{})
	go func() {
		defer close(done)
		// Review browser heartbeat cadence: 1000 ms.
		tick := time.NewTicker(time.Second)
		defer tick.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-tick.C:
				_, _ = r.Heartbeat(session)
			}
		}
	}()
	defer func() { cancel(); <-done }()
	counts.mu.Lock()
	counts.delayStatus = true
	counts.slowFinished = make(chan struct{})
	finished := counts.slowFinished
	before := counts.renewals
	counts.mu.Unlock()
	observed := make(chan *StatusBody, 1)
	go func() { observed <- r.Status(ctx) }()
	select {
	case <-finished:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	s := <-observed
	if s.RTCore.Note != "rt_core_status_slow" {
		s = r.Status(ctx)
	}
	if s.RTCore.Note != "rt_core_status_slow" || !s.Session.Lease.Held || s.Armed == nil || !*s.Armed {
		t.Fatalf("status=%+v rt=%+v", s, s.RTCore)
	}
	// Refresh the controller's motion observation before testing a new intent.
	if err := r.observe(ctx); err != nil {
		t.Fatal(err)
	}
	state, err := r.JointMove(ctx, JointMoveIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Axis: 0, Degrees: 1, Fraction: 0.5})
	if err != nil || state.State != "done" {
		t.Fatalf("move=%+v err=%v", state, err)
	}
	if c.Fence() != fence {
		t.Fatalf("fence changed: %+v", c.Fence())
	}
	counts.mu.Lock()
	defer counts.mu.Unlock()
	if counts.renewals < before+3 {
		t.Fatalf("accepted renewals=%d", counts.renewals-before)
	}
	for _, path := range counts.coldRequests {
		// Only reservation repair may reconnect after the cancelled Status.
		if path != "/v1/jog/ingress" {
			t.Fatalf("cold operation after deadline: %s", path)
		}
	}
	t.Logf("delayed_status=1 accepted_renewals=%d generation=%d cold_status=0", counts.renewals-before, c.Fence().Generation)
}
