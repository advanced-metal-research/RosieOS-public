//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// TestReviewStatusDuringArmComposed holds the native ARM request while four
// 250 ms status polls complete, proving no operation lock delays observation.
func TestReviewStatusDuringArmComposed(t *testing.T) {
	c, d, _, socket, counts := delayedJointFixture(t, false)
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:poll"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	counts.mu.Lock()
	counts.armEntered, counts.armRelease = make(chan struct{}), make(chan struct{})
	counts.mu.Unlock()
	released := false
	defer func() {
		if !released {
			close(counts.armRelease)
		}
	}()
	// Unverified test watchdog: 10000 ms; status receipts precede ARM release.
	ctx, cancel := context.WithTimeout(context.Background(), 10000*time.Millisecond)
	defer cancel()
	armed := make(chan error, 1)
	go func() { _, err := r.Arm(ctx, true); armed <- err }()
	select {
	case <-counts.armEntered:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	for i := 0; i < 4; i++ {
		// Unverified per-poll ceiling: 1000 ms, independent of the ARM watchdog.
		call, stop := context.WithTimeout(ctx, 1000*time.Millisecond)
		s := r.Status(call)
		stop()
		if !s.Available || s.DaemonError != "" {
			t.Fatalf("poll %d queued: %s", i, s.DaemonError)
		}
	}
	close(counts.armRelease)
	released = true
	if err := <-armed; err != nil {
		t.Fatal(err)
	}
}
