//go:build linux

package control

import (
	"context"
	"errors"
	"testing"

	"rosieos/rt-core/ipcclient"
)

// Keep native observations independent of the adapter's historical execution
// summary: acquiring ownership must never reset native faults or establish Home.
type homeAuthorityNative struct {
	leaseNative
	acquireError error
	renewError   error
	state        ipcclient.Status
	homes, stops int
}

func (n *homeAuthorityNative) MirrorAcquire(ctx context.Context, id ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	if n.acquireError != nil {
		return ipcclient.GrantReceipt{}, n.acquireError
	}
	return n.leaseNative.MirrorAcquire(ctx, id, deadline)
}

func (n *homeAuthorityNative) MirrorRenew(ctx context.Context, id ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	if n.renewError != nil {
		return ipcclient.GrantReceipt{}, n.renewError
	}
	return n.leaseNative.MirrorRenew(ctx, id, deadline)
}
func (n *homeAuthorityNative) Stop(context.Context) (uint64, error) {
	n.stops++
	return uint64(n.stops), nil
}
func (n *homeAuthorityNative) Status() (ipcclient.Status, error) { return n.state, nil }
func (n *homeAuthorityNative) Motion() (ipcclient.MotionState, error) {
	return ipcclient.MotionState{}, nil
}
func (n *homeAuthorityNative) Home(context.Context, uint32) (uint64, error) {
	n.homes++
	return 10, nil
}
func (n *homeAuthorityNative) WaitResult(context.Context, uint64) (ipcclient.CommandResult, error) {
	return ipcclient.CommandResult{Sequence: 10}, nil
}

func TestFreshAcquisitionRetiresPreviousAuthorityErrorBeforeHome(t *testing.T) {
	for _, faulted := range []bool{false, true} {
		t.Run(map[bool]string{false: "healthy", true: "native_fault_retained"}[faulted], func(t *testing.T) {
			n := &homeAuthorityNative{state: ipcclient.Status{TimeNS: 1}}
			if faulted {
				n.state.SafetyFaultMask = 1
			}
			binding := Binding{PairID: "cell", Revision: 1, ConfigurationSHA256: "config"}
			c := &Controller{core: n, binding: binding, description: ipcclient.Description{MaxGrantLeaseNS: 500000000}, done: make(chan struct{})}
			c.eventObservation.core.TimeNS = 1
			first, err := c.Acquire("old-owner", binding)
			if err != nil {
				t.Fatal(err)
			}
			defer func() { c.cancel() }()
			old := Fence{first.Session, first.Generation}
			n.renewError = &ipcclient.GrantRejectedError{RequestSequence: 11, Reason: 3}
			if _, err = c.Renew(old); !errors.Is(err, n.renewError) {
				t.Fatalf("renew failure: %v", err)
			}
			if _, err = c.Stop(old, true); err != nil {
				t.Fatal(err)
			}
			if c.execution.Error == "" {
				t.Fatal("historical failure disappeared before new ownership")
			}
			history := c.events.sequence
			firstEvent := c.events.entries[0]
			n.renewError = nil
			fresh, err := c.Acquire("new-owner", binding)
			if err != nil {
				t.Fatal(err)
			}
			state, _, execution, err := c.Snapshot()
			if err != nil {
				t.Fatal(err)
			}
			if execution.Error != "" || execution.State != "" || execution.NativeSequence != 0 || execution.Identity != (Identity{}) {
				t.Fatalf("new authority inherited old execution: %+v", execution)
			}
			if state.HomeValidMask != 0 || state.SafetyFaultMask != n.state.SafetyFaultMask {
				t.Fatalf("acquire changed native evidence: %+v", state)
			}
			if c.events.sequence <= history || c.events.entries[0].Sequence != firstEvent.Sequence || c.events.entries[0].Type != firstEvent.Type {
				t.Fatal("acquisition discarded event history")
			}
			if !faulted {
				if _, err = c.Home(Fence{fresh.Session, fresh.Generation}, 1); err != nil {
					t.Fatal(err)
				}
				if n.homes != 1 || c.execution.Error != "" {
					t.Fatalf("home retained stale failure: %d %+v", n.homes, c.execution)
				}
			}
			// A failure of the new ownership still becomes visible.
			n.renewError = &ipcclient.GrantRejectedError{RequestSequence: 12, Reason: 3}
			if _, err = c.Renew(Fence{fresh.Session, fresh.Generation}); !errors.Is(err, n.renewError) || c.execution.Error != n.renewError.Error() {
				t.Fatalf("new failure hidden: %v %+v", err, c.execution)
			}
		})
	}
}

func TestUnsuccessfulAcquisitionDoesNotClearExecutionFailure(t *testing.T) {
	for _, owned := range []bool{false, true} {
		t.Run(map[bool]string{false: "native_rejection", true: "existing_owner"}[owned], func(t *testing.T) {
			n := &homeAuthorityNative{}
			binding := Binding{PairID: "cell", Revision: 1, ConfigurationSHA256: "config"}
			c := &Controller{core: n, binding: binding, description: ipcclient.Description{MaxGrantLeaseNS: 500000000}, done: make(chan struct{})}
			if owned {
				if _, err := c.Acquire("current", binding); err != nil {
					t.Fatal(err)
				}
			} else {
				n.acquireError = &ipcclient.GrantRejectedError{RequestSequence: 12, Reason: 3}
			}
			defer func() {
				if c.cancel != nil {
					c.cancel()
				}
			}()
			c.execution = Execution{State: "faulted", Error: "current_execution_failed", NativeSequence: 9}
			if _, err := c.Acquire("other", binding); err == nil {
				t.Fatal("unexpected acquisition")
			}
			want := "current_execution_failed"
			if !owned {
				want = n.acquireError.Error()
			}
			if c.execution.Error != want {
				t.Fatalf("current rejection hidden: %+v", c.execution)
			}
		})
	}
}
