//go:build linux

package control

import (
	"context"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

type pausedAuthorityRenew struct {
	nativeClient
	pauseNext atomic.Bool
	entered   chan error
	release   chan struct{}
	stopAck   chan error
}

func (c *pausedAuthorityRenew) MirrorRenew(ctx context.Context, identity ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	receipt, err := c.nativeClient.MirrorRenew(ctx, identity, deadline)
	if c.pauseNext.CompareAndSwap(true, false) {
		c.entered <- err
		if err == nil {
			// Native has actually accepted the renewal. Delay its return to the
			// adapter even if its context expires, exercising a late completion.
			<-c.release
		}
	}
	return receipt, err
}

func (c *pausedAuthorityRenew) Stop(ctx context.Context) (uint64, error) {
	sequence, err := c.nativeClient.Stop(ctx)
	select {
	case c.stopAck <- err:
	default:
	}
	return sequence, err
}

func TestStopReachesNativeWhileRenewReceiptIsPaused(t *testing.T) {
	f := startLifecycleCore(t)
	wrapper := &pausedAuthorityRenew{nativeClient: f.client, entered: make(chan error, 1),
		release: make(chan struct{}), stopAck: make(chan error, 1)}
	wrapper.pauseNext.Store(true)
	var releaseOnce sync.Once
	release := func() { releaseOnce.Do(func() { close(wrapper.release) }) }
	t.Cleanup(release) // Runs before fixture cleanup, including on the failing-before case.

	// The fixture already renews every 100 ms. Intercept exactly its next
	// completed native renewal, without racing replacement of the test boundary.
	f.controller.authorityMu.Lock()
	f.controller.mu.Lock()
	f.controller.core = wrapper
	f.controller.mu.Unlock()
	f.controller.authorityMu.Unlock()
	select {
	case err := <-wrapper.entered:
		if err != nil {
			t.Fatalf("native renewal failed before the controlled pause: %v", err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("fixture renewal never reached the controlled return pause")
	}

	type stopResult struct {
		grant Grant
		err   error
	}
	stopped := make(chan stopResult, 1)
	go func() {
		grant, err := f.controller.Stop(f.fence, false)
		stopped <- stopResult{grant, err}
	}()
	select {
	case err := <-wrapper.stopAck:
		if err != nil {
			t.Fatalf("native urgent Stop did not acknowledge inhibition: %v", err)
		}
	case <-time.After(time.Second):
		t.Fatal("Stop waited behind the paused grant receipt before reaching native")
	}
	// The pause has not been released. The first Stop must already have removed
	// the previously accepted native grant; adapter completion may await fencing.
	for limit := time.Now().Add(time.Second); ; {
		observation, err := f.client.FastObservation()
		if err == nil && observation.GrantActive == 0 {
			break
		}
		if time.Now().After(limit) {
			t.Fatalf("native grant remained active while its old receipt was paused: %+v, %v", observation, err)
		}
		time.Sleep(time.Millisecond)
	}
	release()
	var result stopResult
	select {
	case result = <-stopped:
		if result.err != nil {
			t.Fatal(result.err)
		}
	case <-time.After(3 * time.Second):
		t.Fatal("Stop failed to finish after the old renewal returned")
	}
	if result.grant.Session == "" || result.grant.Generation <= f.fence.Generation {
		t.Fatalf("Stop did not publish fresh application authority: %+v", result.grant)
	}
	// The old renewal must not commit over or revoke the replacement grant.
	observation, err := f.client.FastObservation()
	if err != nil || observation.GrantActive != 1 || observation.AppGrantGeneration != result.grant.Generation {
		t.Fatalf("late renewal damaged replacement authority: %+v, %v", observation, err)
	}
	f.controller.mu.Lock()
	current, stopping := f.controller.grant, f.controller.stopping
	f.controller.mu.Unlock()
	if stopping || current.Session != result.grant.Session || current.Generation != result.grant.Generation {
		t.Fatalf("late renewal replaced local authority: current=%+v stopping=%v", current, stopping)
	}
}

type pausedAuthorityAcquire struct {
	nativeClient
	entered chan struct{}
	release chan struct{}
}

func (c *pausedAuthorityAcquire) MirrorAcquire(ctx context.Context, identity ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	close(c.entered)
	<-c.release
	// Deliberately enter the real SDK only after Close has completed its Stop.
	// The SDK's local revocation snapshot is now new; the original application
	// transaction's canceled session context must prevent fresh publication.
	return c.nativeClient.MirrorAcquire(ctx, identity, deadline)
}

func TestCloseCancelsPendingStopReacquisitionBeforeNativePublication(t *testing.T) {
	f := startLifecycleCore(t)
	wrapper := &pausedAuthorityAcquire{nativeClient: f.client, entered: make(chan struct{}), release: make(chan struct{})}
	var releaseOnce sync.Once
	release := func() { releaseOnce.Do(func() { close(wrapper.release) }) }
	t.Cleanup(release)
	f.controller.authorityMu.Lock()
	f.controller.mu.Lock()
	f.controller.core = wrapper
	f.controller.mu.Unlock()
	f.controller.authorityMu.Unlock()

	stopped := make(chan error, 1)
	go func() { _, err := f.controller.Stop(f.fence, false); stopped <- err }()
	select {
	case <-wrapper.entered:
	case <-time.After(3 * time.Second):
		t.Fatal("Stop did not reach replacement grant publication")
	}
	closed := make(chan error, 1)
	go func() { closed <- f.controller.Close() }()
	select {
	case err := <-closed:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(time.Second):
		t.Fatal("Close waited for the pending replacement grant")
	}
	release()
	select {
	case err := <-stopped:
		if err == nil {
			t.Fatal("Stop published application authority after Close")
		}
	case <-time.After(3 * time.Second):
		t.Fatal("pending Stop did not finish after its publication pause was released")
	}
	observation, err := f.client.FastObservation()
	if err != nil || observation.GrantActive != 0 {
		t.Fatalf("pending replacement republished native authority after Close: %+v, %v", observation, err)
	}
	f.controller.mu.Lock()
	current, stopping := f.controller.grant, f.controller.stopping
	f.controller.mu.Unlock()
	if current.Session != "" || !stopping {
		t.Fatalf("Close's terminal authority state was overwritten: grant=%+v stopping=%v", current, stopping)
	}
}
