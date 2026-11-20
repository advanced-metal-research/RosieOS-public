//go:build linux

package denseexec

import (
	"context"
	"errors"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// TestNothingHeldClearsEmptyLocalLatch requires owned fence evidence for a
// recovery latch; a stale local flag alone must not force anonymous adapter Stop.
func TestNothingHeldClearsEmptyLocalLatch(t *testing.T) {
	r, f, _ := rtFake(t)
	r.mu.Lock()
	r.inhibited, r.stopUncertain = true, true
	r.mu.Unlock()
	result, err := r.Stop(context.Background())
	if err != nil || !result.OK || result.Note != "nothing_held" || result.Attempts != 0 || result.Status.RTCore.Inhibited || result.Status.RTCore.StopUncertain {
		t.Fatalf("empty local latch: %+v %v", result, err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if len(f.calls) != 0 {
		t.Fatalf("anonymous Stop was sent: %v", f.calls)
	}
}

// TestNothingHeldCannotHideUnknownAcquire distinguishes no authority from a
// missing receipt, preventing the no-op path from clearing uncertain issuance.
func TestNothingHeldCannotHideUnknownAcquire(t *testing.T) {
	for _, cause := range []error{control.ErrProtocol, control.ErrTransport} {
		r, f, _ := rtFake(t)
		f.acquireErr, f.stopErr = cause, rtRefused("control_session_stale")
		if _, err := r.Arm(context.Background(), true); err == nil {
			t.Fatal("unknown acquisition accepted")
		}
		status := r.statusSnapshot().RTCore
		if !status.StopUncertain || status.Inhibited {
			t.Fatalf("unknown issuance hidden: %+v", status)
		}
		var failure *Error
		if _, err := r.Arm(context.Background(), true); !errors.As(err, &failure) || failure.Code != "rt_core_inhibited" {
			t.Fatalf("unknown issuance admitted ARM: %v", err)
		}
	}
}

func retirementController(t *testing.T) (*RTCoreController, *control.Client) {
	t.Helper()
	c, d, _, socket := rtComposedFixture(t)
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:retirement"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	return r, c
}

// TestNothingHeldComposed requires acknowledged no-ops and a fresh native ARM;
// anonymous adapter Stop must never turn absence of authority into a latch.
func TestNothingHeldComposed(t *testing.T) {
	for _, action := range []string{"stop", "disarm"} {
		t.Run(action, func(t *testing.T) {
			r, c := retirementController(t)
			ctx := context.Background()
			if action == "stop" {
				result, err := r.Stop(ctx)
				if err != nil || !result.OK || !result.Stopped || result.Note != "nothing_held" || result.Attempts != 0 {
					t.Fatalf("nothing held Stop: %+v %v", result, err)
				}
			} else {
				result, err := r.Arm(ctx, false)
				if err != nil || !result.Accepted || result.Note != "nothing_held" {
					t.Fatalf("nothing held Disarm: %+v %v", result, err)
				}
			}
			if _, err := r.Arm(ctx, true); err != nil {
				t.Fatal(err)
			}
			state, err := c.Status(ctx)
			if err != nil || state.Core.Armed != 1 {
				t.Fatalf("fresh ARM: %+v %v", state.Core, err)
			}
		})
	}
}

// TestHeartbeatRetirementComposed retires a live, renewed session by the browser
// deadline, then repeats Stop and ARM on that controller without a restart.
func TestHeartbeatRetirementComposed(t *testing.T) {
	r, c := retirementController(t)
	ctx := context.Background()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	old := c.Grant()
	r.mu.Lock()
	// Unverified deadline stimulus: browser age 6000 ms, independent of lease age.
	r.lastHeartbeat = time.Now().Add(-6000 * time.Millisecond)
	r.mu.Unlock()
	rtAwait(t, func() bool {
		r.stopMu.Lock()
		defer r.stopMu.Unlock()
		r.mu.Lock()
		defer r.mu.Unlock()
		return !r.held && r.session.State == "stopped" && !r.inhibited && c.Grant().Session == ""
	})
	state, err := c.Status(ctx)
	if err != nil || state.Core.Armed != 0 || state.Core.AxisEnableMask != 0 {
		t.Fatalf("retirement: %+v %v", state.Core, err)
	}
	for i := 0; i < 2; i++ {
		if result, err := r.Stop(ctx); err != nil || !result.OK {
			t.Fatalf("already retired Stop: %+v %v", result, err)
		}
	}
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	fresh := c.Grant()
	if fresh.Session == old.Session || fresh.Session == "" || fresh.Generation <= old.Generation {
		t.Fatalf("fresh grant: %+v", fresh)
	}
}
