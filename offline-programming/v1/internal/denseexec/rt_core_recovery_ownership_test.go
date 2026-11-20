//go:build linux

package denseexec

import (
	"context"
	"errors"
	api "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/sdk/control"
	"sync/atomic"
	"testing"
	"time"
)

type delayedHomeStatus struct {
	RTCoreClient
	first            atomic.Bool
	entered, release chan struct{}
	old              control.Status
	oldErr           error
}

func (c *delayedHomeStatus) Status(ctx context.Context) (control.Status, error) {
	if c.first.CompareAndSwap(true, false) {
		close(c.entered)
		select {
		case <-c.release:
			return c.old, c.oldErr
		case <-ctx.Done():
			return control.Status{}, ctx.Err()
		}
	}
	return c.RTCoreClient.Status(ctx)
}
func TestRTCoreHomeRecoveryDiscardsOldFaultBeforeInterpretation(t *testing.T) {
	r, f, _ := rtFake(t)
	f.mu.Lock()
	referenceFault(f, 4)
	old := f.status
	f.status.Core.SafetyFaultMask = 0
	f.status.Core.ExecutionFaultReasons = 0
	f.status.Core.Recovery = nil
	f.status.TimeNS = 2
	f.status.Motion.TimeNS = 2
	f.mu.Unlock()
	wrapped := &delayedHomeStatus{RTCoreClient: f, entered: make(chan struct{}), release: make(chan struct{}), old: old}
	wrapped.first.Store(true)
	r.c = wrapped
	finished := make(chan error, 1)
	go func() { finished <- r.observe(context.Background()) }()
	<-wrapped.entered
	if err := r.observe(context.Background()); err != nil {
		t.Fatal(err)
	}
	close(wrapped.release)
	if err := <-finished; err != nil {
		t.Fatalf("older reference fault overruled newer clear: %v", err)
	}
	r.mu.Lock()
	observed := r.observation.TimeNS
	r.mu.Unlock()
	if observed != 2 {
		t.Fatalf("observation regressed: %d", observed)
	}
	f.mu.Lock()
	referenceFault(f, 4)
	f.status.TimeNS = 3
	f.status.Motion.TimeNS = 3
	f.mu.Unlock()
	if err := r.observe(context.Background()); err == nil {
		t.Fatal("new fault was hidden")
	}
}
func TestRTCoreHomeRecoveryDiscardsPriorOwnerStatusError(t *testing.T) {
	r, f, _ := rtFake(t)
	wrapped := &delayedHomeStatus{RTCoreClient: f, entered: make(chan struct{}), release: make(chan struct{}), oldErr: control.ErrTransport}
	wrapped.first.Store(true)
	r.c = wrapped
	finished := make(chan error, 1)
	go func() { finished <- r.observe(context.Background()) }()
	<-wrapped.entered
	r.mu.Lock()
	r.fence = control.Fence{Session: "replacement", Generation: 2}
	r.mu.Unlock()
	close(wrapped.release)
	if err := <-finished; err != nil {
		t.Fatalf("previous owner transport failure reached successor: %v", err)
	}
}
func TestRTCoreHomeRecoveryIgnoresReplacedKeeperOnly(t *testing.T) {
	r, _, _ := rtFake(t)
	if err := r.acquire(context.Background()); err != nil {
		t.Fatal(err)
	}
	old, next := make(chan control.Renewal), make(chan control.Renewal)
	r.mu.Lock()
	r.renewals = old
	r.mu.Unlock()
	deadline := time.Now().Add(time.Second)
	for {
		r.mu.Lock()
		select {
		case old <- control.Renewal{Err: &control.Rejected{Reason: "control_session_stale"}}:
			// The old result has been received, but its current-owner check is blocked
			// on this mutex. Install replacement ownership before releasing that check.
			r.renewals = next
			r.fence = control.Fence{Session: "replacement", Generation: 2}
			r.mu.Unlock()
			goto replaced
		default:
			r.mu.Unlock()
		}
		if time.Now().After(deadline) {
			t.Fatal("watcher did not capture prior keeper")
		}
		time.Sleep(time.Millisecond)
	}
replaced:
	select {
	case next <- control.Renewal{Grant: control.Grant{Session: "replacement", Generation: 2}}:
	case <-time.After(time.Second):
		t.Fatal("old keeper retired replacement")
	}
	r.mu.Lock()
	held := r.held
	r.mu.Unlock()
	if !held {
		t.Fatal("replacement was stopped")
	}
	select {
	case next <- control.Renewal{Err: control.ErrTransport}:
	case <-time.After(time.Second):
		t.Fatal("current keeper not observed")
	}
	for deadline := time.Now().Add(time.Second); ; {
		r.mu.Lock()
		held := r.held
		r.mu.Unlock()
		if !held {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("current keeper failure ignored")
		}
		time.Sleep(time.Millisecond)
	}
}

type delayedHomeEvents struct {
	RTCoreClient
	block            atomic.Bool
	returned         atomic.Bool
	entered, release chan struct{}
	next             chan uint64
}

func (c *delayedHomeEvents) EventsContext(ctx context.Context, after uint64) (control.EventBatch, error) {
	if c.block.CompareAndSwap(true, false) {
		close(c.entered)
		select {
		case <-c.release:
		case <-ctx.Done():
			return control.EventBatch{}, ctx.Err()
		}
		c.returned.Store(true)
		return control.EventBatch{AdapterIncarnation: "adapter", NextSequence: 1, Events: []control.Event{{Type: "fault_latched", Fault: &api.FaultEvent{Bit: 64, AxisMask: 4}}}}, nil
	}
	if c.returned.Load() {
		select {
		case c.next <- after:
		default:
		}
	}
	return c.RTCoreClient.EventsContext(ctx, after)
}
func TestRTCoreHomeRecoveryDiscardsPriorOwnerEvents(t *testing.T) {
	r, f, _ := rtFake(t)
	wrapped := &delayedHomeEvents{RTCoreClient: f, entered: make(chan struct{}), release: make(chan struct{}), next: make(chan uint64, 1)}
	r.c = wrapped
	if err := r.acquire(context.Background()); err != nil {
		t.Fatal(err)
	}
	wrapped.block.Store(true)
	select {
	case <-wrapped.entered:
	case <-time.After(time.Second):
		t.Fatal("event request not reached")
	}
	r.mu.Lock()
	r.fence = control.Fence{Session: "replacement", Generation: 2}
	r.cursor = 99
	r.mu.Unlock()
	close(wrapped.release)
	select {
	case after := <-wrapped.next:
		if after != 99 {
			t.Fatalf("old event response regressed successor cursor to %d", after)
		}
	case <-time.After(time.Second):
		t.Fatal("old fault event stopped successor")
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if !r.held || r.cursor != 99 {
		t.Fatalf("successor changed: held=%v cursor=%d", r.held, r.cursor)
	}
}

func TestRTCoreHomeRecoveryConditionalStopPreservesSuccessor(t *testing.T) {
	r, f, _ := rtFake(t)
	if err := r.acquire(context.Background()); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	old := observedAuthority{epoch: r.epoch, fence: r.fence, renewals: r.renewals}
	r.fence = control.Fence{Session: "replacement", Generation: 2}
	r.renewals = make(chan control.Renewal)
	r.feedbackCode = "new_feedback"
	r.statusNote = "new_note"
	current := observedAuthority{epoch: r.epoch, fence: r.fence, renewals: r.renewals}
	r.mu.Unlock()
	// Exercise the guard at Stop's mutation boundary, independently of the
	// watcher's earlier current-owner check: the replacement already won.
	if _, err := r.stopObserved("old_fault", &old); !errors.Is(err, errObservationSuperseded) {
		t.Fatalf("old Stop result: %v", err)
	}
	for _, err := range []error{control.ErrTransport, control.ErrConnectionUnsent, context.DeadlineExceeded} {
		if failure := r.failObserved(err, old); failure != nil {
			t.Fatalf("old observer affected successor: %v", failure)
		}
	}
	r.mu.Lock()
	if !r.held || r.epoch != current.epoch || r.stopping != 0 || r.feedbackCode != "new_feedback" || r.statusNote != "new_note" {
		t.Fatal("old observer mutated replacement state")
	}
	r.mu.Unlock()
	f.mu.Lock()
	stops := rtCallIndex(f.calls, "stop")
	f.mu.Unlock()
	if stops >= 0 {
		t.Fatal("old error sent Stop")
	}
	if err := r.failObserved(&rtAuthorityFailure{control.ErrTransport}, current); err == nil {
		t.Fatal("current failure ignored")
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if rtCallIndex(f.calls, "stop") < 0 {
		t.Fatal("current error did not Stop")
	}
}
