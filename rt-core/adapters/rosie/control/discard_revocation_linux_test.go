//go:build linux

package control

import (
	"context"
	"errors"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

type revokingDiscardResult struct {
	nativeClient
	onRevoked func()
}

func (c *revokingDiscardResult) DiscardTrajectory(ctx context.Context, handle uint64) (uint64, error) {
	// Native malformed-envelope rejection revokes outputs before returning
	// InvalidCommand. Use real native revocation to exercise that result/state
	// combination at the adapter boundary; this is distinct from InUse.
	if _, err := c.nativeClient.Stop(ctx); err != nil {
		return 0, err
	}
	if c.onRevoked != nil {
		c.onRevoked()
	}
	return 0, &ipcclient.RejectedError{Result: ipcclient.CommandResult{
		Handle: handle, Operation: ipcclient.MsgTrajectoryDiscard,
		Result: ipcclient.ResultRejected, Reason: ipcclient.ReasonInvalidCommand,
	}}
}

func TestRevokingDiscardRejectionInvalidatesPreparedPublication(t *testing.T) {
	f := startLifecycleCore(t)
	program := mustPrepareProgram(t, f, "revoked-discard")
	native := &gatedHandleStop{countedHandleNative: &countedHandleNative{nativeClient: f.client}}
	gate := &handleStopGate{entered: make(chan struct{}), release: make(chan struct{})}
	defer gate.open()
	retired := make(chan error, 1)
	replaceNativeTestClient(f.controller, &revokingDiscardResult{nativeClient: native, onRevoked: func() {
		// Retire authority while its independent Stop acknowledgement is still
		// pending; the execution summary must already match the retired program.
		native.next.Store(gate)
		go func() { retired <- f.controller.failAuthority(f.fence, ipcclient.ErrRevoked) }()
		select {
		case <-gate.entered:
		case <-time.After(3 * time.Second):
			t.Fatal("authority retirement did not reach Stop")
		}
	}})
	_, err := f.controller.Discard(f.fence, program.handle)
	var rejected *ipcclient.RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonInvalidCommand {
		t.Fatalf("missing revocation rejection: %v", err)
	}
	f.controller.mu.Lock()
	retained, execution := f.controller.program, f.controller.execution
	f.controller.mu.Unlock()
	if retained != nil || execution.State != "faulted" {
		t.Fatalf("InvalidCommand was treated as unchanged InUse: %+v", execution)
	}
	gate.open()
	select {
	case <-retired:
	case <-time.After(3 * time.Second):
		t.Fatal("authority retirement did not finish")
	}
	// Native revocation now also removes the mirrored application grant. Neither
	// Enable nor renewal of that identity may silently restore it.
	if _, err := f.controller.Enable(f.fence, 511); err == nil {
		t.Fatal("Enable restored a natively revoked application grant")
	}
	if _, err := f.controller.Renew(f.fence); err == nil {
		t.Fatal("renewal restored a natively revoked application grant")
	}
	grant, err := f.controller.Acquire("revoked-discard-recovery", f.controller.binding)
	if err != nil {
		t.Fatal(err)
	}
	armLifecycle(t, f.controller, f.client, Fence{grant.Session, grant.Generation})
	assertNativeUnknown(t, f.client, program.handle)
}
