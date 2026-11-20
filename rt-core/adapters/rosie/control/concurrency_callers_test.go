//go:build linux

package control

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

type handleStopGate struct {
	entered chan struct{}
	release chan struct{}
	once    sync.Once
}

func (g *handleStopGate) open() { g.once.Do(func() { close(g.release) }) }

type gatedHandleStop struct {
	*countedHandleNative
	next atomic.Pointer[handleStopGate]
}

// Stop executes real native inhibition before pausing its return. The gate
// exposes the adapter's in-flight ordering without mocking daemon acceptance.
func (n *gatedHandleStop) Stop(ctx context.Context) (uint64, error) {
	gate := n.next.Swap(nil)
	sequence, err := n.nativeClient.Stop(ctx)
	if gate != nil {
		close(gate.entered)
		<-gate.release
	}
	return sequence, err
}

type handleCallResult struct {
	response Response
	err      error
}

func asyncHandleCall(ctx context.Context, a *handleAPI, request Request, start <-chan struct{}) <-chan handleCallResult {
	done := make(chan handleCallResult, 1)
	go func() {
		if start != nil {
			<-start
		}
		response, _, err := a.do(ctx, request)
		done <- handleCallResult{response, err}
	}()
	return done
}

func awaitHandleCall(t *testing.T, ctx context.Context, done <-chan handleCallResult) Response {
	t.Helper()
	select {
	case result := <-done:
		if result.err != nil {
			t.Fatal(result.err)
		}
		return result.response
	case <-ctx.Done():
		t.Fatal("concurrent caller deadline exceeded")
		return Response{}
	}
}

func TestConcurrentRenewStopStartRace(t *testing.T) {
	f := newHandleFixture(t)
	native := &gatedHandleStop{countedHandleNative: &countedHandleNative{nativeClient: f.client}}
	replaceNativeTestClient(f.controller, native)
	a := serveHandleAPI(t, f.controller, nil)
	ctx, cancel := context.WithTimeout(context.Background(), 180*time.Second)
	defer cancel()
	renews := make(chan Grant, 512)
	renewErrors := make(chan error, 1)
	renewCtx, stopRenew := context.WithCancel(ctx)
	renewDone := make(chan struct{})
	go func() {
		defer close(renewDone)
		ticker := time.NewTicker(2 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-renewCtx.Done():
				return
			case <-ticker.C:
			}
			response, _, err := a.do(renewCtx, controlRequest(f.fence, "renew"))
			if renewCtx.Err() != nil {
				return
			}
			if err != nil || response.Error != "" {
				renewErrors <- fmt.Errorf("renew: %+v %v", response, err)
				return
			}
			raw, _ := json.Marshal(response.Data)
			var grant Grant
			if err := json.Unmarshal(raw, &grant); err != nil {
				renewErrors <- err
				return
			}
			if grant.Session != f.fence.Session || grant.DeadlineHostNS <= ipcclient.HostMonotonicNS() {
				renewErrors <- fmt.Errorf("expired keepalive: %+v", grant)
				return
			}
			select {
			case renews <- grant:
			case <-renewCtx.Done():
				return
			}
		}
	}()
	t.Cleanup(func() { stopRenew(); <-renewDone })
	fence := f.fence
	for iteration := 0; iteration < 200; iteration++ {
		if ctx.Err() != nil {
			t.Fatal("200-iteration race exceeded test deadline")
		}
		enableHandles(t, a, fence)
		handle := prepareHandle(t, a, fence, time.Second)
		_, _, before := a.snapshot(t)
		accepted := native.accepted.Load()
		gate := &handleStopGate{entered: make(chan struct{}), release: make(chan struct{})}
		t.Cleanup(gate.open)
		native.next.Store(gate)
		start := make(chan struct{})
		request := controlRequest(fence, "start_trajectory")
		request.Handle = handle
		started := asyncHandleCall(ctx, a, request, start)
		stopped := asyncHandleCall(ctx, a, controlRequest(fence, "stop"), start)
		close(start)
		select {
		case <-gate.entered:
		case <-ctx.Done():
			t.Fatal("Stop deadlocked before native inhibition")
		}
		overlapped := false
		for !overlapped {
			select {
			case grant := <-renews:
				overlapped = grant.Stopping && grant.Generation == fence.Generation+1
			case err := <-renewErrors:
				t.Fatal(err)
			case <-ctx.Done():
				t.Fatal("no overlapping Stop keepalive")
			}
		}
		awaitHandleCondition(t, func() bool { state, _, _ := a.snapshot(t); return state.Armed == 0 && state.AxisEnableMask == 0 })
		// A stale fast observation is not evidence either way: under host load the
		// RT publication can be older than the freshness window when this reads it.
		// Poll for a fresh observation within the test deadline, then assert on it.
		var observation ipcclient.GrantObservation
		var err error
		for {
			observation, err = f.client.FastObservation()
			if err == nil || ctx.Err() != nil {
				break
			}
			time.Sleep(time.Millisecond)
		}
		if err != nil || observation.GrantActive != 0 {
			t.Fatalf("iteration %d: keepalive restored native authority: %+v %v", iteration, observation, err)
		}
		gate.open()
		stop := awaitHandleCall(t, ctx, stopped)
		if stop.Error != "" {
			t.Fatalf("iteration %d Stop: %+v", iteration, stop)
		}
		grant := grantFrom(t, stop)
		if grant.Session != fence.Session || grant.Generation != fence.Generation+1 || grant.Stopping {
			t.Fatalf("iteration %d grant: %+v", iteration, grant)
		}
		result := awaitHandleCall(t, ctx, started)
		switch result.Error {
		case "", "fence", "control_session_stale", "handle_retired":
		default:
			t.Fatalf("iteration %d unexpected Start reason: %+v", iteration, result)
		}
		_, _, after := a.snapshot(t)
		if after.Generation < before.Generation || after.Generation > before.Generation+1 || native.accepted.Load() > accepted+1 {
			t.Fatalf("iteration %d double execution: %d -> %d; native %d -> %d", iteration, before.Generation, after.Generation, accepted, native.accepted.Load())
		}
		requireHandle(t, a, handle, HandleRetired)
		late := controlRequest(fence, "prepare_trajectory")
		late.AxisMask, late.Points = 511, handlePoints(time.Second)
		a.call(t, late, ReasonControlSessionStale)
		fence = Fence{grant.Session, grant.Generation}
	}
	select {
	case err := <-renewErrors:
		t.Fatal(err)
	default:
	}
}

func TestExpiryVersusRenewRace(t *testing.T) {
	f := newHandleFixture(t)
	a := serveHandleAPI(t, f.controller, nil)
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	fence := f.fence
	for iteration := 0; iteration < 32; iteration++ {
		grant := grantFrom(t, a.call(t, controlRequest(fence, "renew"), ""))
		// Schedule ingress on either side of the actual monotonic expiry.
		// The timer creates the race; only the returned grant/reason can pass.
		offset := int64(-500000)
		if iteration%2 != 0 {
			offset = 500000
		}
		wait := time.Duration(int64(grant.DeadlineHostNS) + offset - int64(ipcclient.HostMonotonicNS()))
		timer := time.NewTimer(max(wait, 0))
		select {
		case <-timer.C:
		case <-ctx.Done():
			timer.Stop()
			t.Fatal("expiry race deadline exceeded")
		}
		response, _, err := a.do(ctx, controlRequest(fence, "renew"))
		if err != nil {
			t.Fatal(err)
		}
		if response.Error == "" {
			renewed := grantFrom(t, response)
			if renewed.Session != fence.Session || renewed.DeadlineHostNS <= ipcclient.HostMonotonicNS() {
				t.Fatalf("renew returned an expired grant: %+v", renewed)
			}
			a.call(t, controlRequest(fence, "release"), "")
		} else if response.Error != "expired" {
			t.Fatalf("expiry race returned %q", response.Error)
		}
		awaitHandleCondition(t, func() bool {
			f.controller.mu.Lock()
			defer f.controller.mu.Unlock()
			return f.controller.grant.Session == "" && !f.controller.stopping
		})
		acquire := controlRequest(Fence{}, "acquire")
		acquire.Controller, acquire.Binding = "expiry-race", f.controller.binding
		fresh := grantFrom(t, a.call(t, acquire, ""))
		fence = Fence{fresh.Session, fresh.Generation}
	}
}

func TestReleaseDuringStop(t *testing.T) {
	f := newHandleFixture(t)
	native := &gatedHandleStop{countedHandleNative: &countedHandleNative{nativeClient: f.client}}
	replaceNativeTestClient(f.controller, native)
	a := serveHandleAPI(t, f.controller, nil)
	stopRenew := keepHandleGrant(t, a, f.fence)
	enableHandles(t, a, f.fence)
	active := prepareHandle(t, a, f.fence, 2*time.Second)
	prepared := prepareHandle(t, a, f.fence, 2*time.Second)
	request := controlRequest(f.fence, "start_trajectory")
	request.Handle = active
	a.call(t, request, "")
	gate := &handleStopGate{entered: make(chan struct{}), release: make(chan struct{})}
	t.Cleanup(gate.open)
	native.next.Store(gate)
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	stopped := asyncHandleCall(ctx, a, controlRequest(f.fence, "stop"), nil)
	select {
	case <-gate.entered:
	case <-ctx.Done():
		t.Fatal("Stop did not reach native")
	}
	released := asyncHandleCall(ctx, a, controlRequest(f.fence, "release"), nil)
	awaitHandleCondition(t, func() bool {
		f.controller.mu.Lock()
		defer f.controller.mu.Unlock()
		return f.controller.stopFlight != nil && f.controller.stopFlight.release
	})
	select {
	case <-released:
		t.Fatal("Release completed before Stop")
	default:
	}
	awaitHandleCondition(t, func() bool { state, _, _ := a.snapshot(t); return state.Armed == 0 && state.AxisEnableMask == 0 })
	f.controller.mu.Lock()
	session := f.controller.grant.Session
	f.controller.mu.Unlock()
	if session != f.fence.Session {
		t.Fatal("session released before Stop completed")
	}
	stopRenew()
	gate.open()
	for _, result := range []Response{awaitHandleCall(t, ctx, stopped), awaitHandleCall(t, ctx, released)} {
		if result.Error != "" || grantFrom(t, result).Session != "" {
			t.Fatalf("release ordering: %+v", result)
		}
	}
	for _, id := range []uint64{active, prepared} {
		requireHandle(t, a, id, HandleRetired)
	}
	a.call(t, controlRequest(f.fence, "renew"), "control_session_stale")
	observation, err := f.client.FastObservation()
	if err != nil || observation.GrantActive != 0 {
		t.Fatalf("Release left native authority: %+v %v", observation, err)
	}
}

func TestExpiryDuringStopReacquisition(t *testing.T) {
	f := newHandleFixture(t)
	a := serveHandleAPI(t, f.controller, nil)
	id := prepareHandle(t, a, f.fence, time.Second)
	request := controlRequest(f.fence, "renew")
	key := "expires-with-session"
	request.RequestID = &key
	a.call(t, request, "")
	paused := &pausedAuthorityAcquire{nativeClient: f.client, entered: make(chan struct{}), release: make(chan struct{})}
	replaceNativeTestClient(f.controller, paused)
	var once sync.Once
	release := func() { once.Do(func() { close(paused.release) }) }
	t.Cleanup(release)
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	stopped := asyncHandleCall(ctx, a, controlRequest(f.fence, "stop"), nil)
	select {
	case <-paused.entered:
	case <-ctx.Done():
		t.Fatal("Stop never reached reacquisition")
	}
	awaitHandleCondition(t, func() bool {
		f.controller.mu.Lock()
		defer f.controller.mu.Unlock()
		return f.controller.grant.Session == "" && len(f.controller.requests.entries) == 0
	})
	release()
	if result := awaitHandleCall(t, ctx, stopped); result.Error != "expired" {
		t.Fatalf("expired Stop: %+v", result)
	}
	requireHandle(t, a, id, HandleRetired)
	replaceNativeTestClient(f.controller, f.client)
	acquire := controlRequest(Fence{}, "acquire")
	acquire.Controller, acquire.Binding = "after-stop-expiry", f.controller.binding
	a.call(t, acquire, "")
}

type pendingHandleRenew struct {
	nativeClient
	entered chan struct{}
	release chan struct{}
}

func (n *pendingHandleRenew) MirrorRenew(ctx context.Context, identity ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	close(n.entered)
	<-n.release
	return n.nativeClient.MirrorRenew(ctx, identity, deadline)
}

func TestRenewQueuedPastExpiryReturnsExpired(t *testing.T) {
	f := newHandleFixture(t)
	a := serveHandleAPI(t, f.controller, nil)
	paused := &pendingHandleRenew{nativeClient: f.client, entered: make(chan struct{}), release: make(chan struct{})}
	replaceNativeTestClient(f.controller, paused)
	var once sync.Once
	release := func() { once.Do(func() { close(paused.release) }) }
	t.Cleanup(release)
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	result := asyncHandleCall(ctx, a, controlRequest(f.fence, "renew"), nil)
	select {
	case <-paused.entered:
	case <-ctx.Done():
		t.Fatal("renew never entered native transaction")
	}
	awaitHandleCondition(t, func() bool {
		f.controller.mu.Lock()
		defer f.controller.mu.Unlock()
		return f.controller.expiredSession == f.fence.Session
	})
	release()
	if response := awaitHandleCall(t, ctx, result); response.Error != "expired" {
		t.Fatalf("renew lost expiry classification: %+v", response)
	}
	a.call(t, controlRequest(f.fence, "renew"), "expired")
}
