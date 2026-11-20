//go:build linux

package denseexec

import (
	"context"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// TestReviewWarmRequestMixComposed runs the browser cadence, keeper, ARM and
// joint move against native simulation. Every new connection costs 900 ms;
// zero cold requests after Acquire prevents handshake tails from retiring it.
func TestReviewWarmRequestMixComposed(t *testing.T) {
	c, d, _, socket, counts := delayedJointFixture(t, true)
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:warm"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	// Unverified test watchdog: 30000 ms; native receipts establish completion.
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	var workers sync.WaitGroup
	defer func() { cancel(); workers.Wait() }()
	var polls, heartbeats atomic.Int32
	workers.Add(2)
	go func() {
		defer workers.Done()
		// Measured UI request cadence from review: 250 ms.
		tick := time.NewTicker(250 * time.Millisecond)
		defer tick.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-tick.C:
				workers.Add(1)
				go func() {
					defer workers.Done()
					if s := r.Status(ctx); s.Available {
						polls.Add(1)
					}
				}()
			}
		}
	}()
	go func() {
		defer workers.Done()
		// Review heartbeat cadence: 1000 ms.
		tick := time.NewTicker(time.Second)
		defer tick.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-tick.C:
				if s := r.statusSnapshot().Session; s != nil {
					if _, err := r.Heartbeat(s.ID); err == nil {
						heartbeats.Add(1)
					}
				}
			}
		}
	}()
	started := time.Now()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	t.Logf("ARM elapsed_ms=%d", time.Since(started).Milliseconds())
	session := r.statusSnapshot().Session.ID
	state, err := r.JointMove(ctx, JointMoveIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Axis: 0, Degrees: 1, Fraction: 0.5})
	if err != nil || state.State != "done" {
		t.Fatalf("move=%+v err=%v", state, err)
	}
	rtAwait(t, func() bool {
		counts.mu.Lock()
		defer counts.mu.Unlock()
		return polls.Load() >= 12 && heartbeats.Load() >= 3 && counts.renewals >= 3
	})
	s, err := c.Status(ctx)
	if err != nil || s.Core.Armed != 1 || s.Grant == nil || s.Grant.GrantActive != 1 {
		t.Fatalf("native=%+v err=%v", s, err)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	counts.mu.Lock()
	defer counts.mu.Unlock()
	if len(counts.coldRequests) != 0 || counts.opened.Load() != 7 {
		t.Fatalf("connections=%d cold_after_acquire=%v", counts.opened.Load(), counts.coldRequests)
	}
	t.Logf("connections=%d peak_in_flight=%d peak_held=%d cold_after_acquire=%d polls=%d heartbeats=%d renewals=%d", counts.opened.Load(), counts.peak, counts.peakHeld, len(counts.coldRequests), polls.Load(), heartbeats.Load(), counts.renewals)
}
