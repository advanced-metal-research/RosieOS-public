//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// TestHomeCompletedRenewalStall establishes renewal cadence, then requires six
// successful renewals inside the held HTTP receipt after all nine native axes
// finish, before withholding renewal. The keeper must report typed
// deadline/transport loss, terminate, and leave native outputs inhibited even
// when the already successful Home receipt subsequently arrives.
// StartRenewal's 250 ms request context runs in its own goroutine/urgent pool;
// scheduler starvation counts against that receipt budget. Go's -p=1 limits
// package concurrency, not renewal goroutine scheduling or GOMAXPROCS.
func TestHomeCompletedRenewalStall(t *testing.T) {
	testHomeCompletedRenewalStall(t, false)
}

// TestHomeCompletedRenewalStallPendingBaseline queues a successful renewal before
// observation resumes, preventing a later baseline from rejecting its earlier start.
func TestHomeCompletedRenewalStallPendingBaseline(t *testing.T) {
	testHomeCompletedRenewalStall(t, true)
}

// testHomeCompletedRenewalStall checks cadence and expiry with an optional queued
// first receipt, so observer scheduling cannot conceal a baseline ordering error.
func testHomeCompletedRenewalStall(t *testing.T, pendingBaseline bool) {
	// Unverified isolation budget: 200,000,000 ns excludes incidental cycle
	// jitter from this injected HTTP-deadline test; no live configuration changes.
	f := startFixtureAxesWithCycleBudget(t, false, false, 9, 200000000)
	upstream := control.NewUnixHTTPClient(f.socket)
	t.Cleanup(upstream.CloseIdleConnections)
	homeCompleted, releaseHome := make(chan struct{}), make(chan struct{})
	var releaseOnce sync.Once
	unblock := func() { releaseOnce.Do(func() { close(releaseHome) }) }
	t.Cleanup(unblock)
	var stallRenewal atomic.Bool
	var heldRenewals, stalledRenewals atomic.Uint32
	var holdStarted atomic.Uint64
	// Unverified observation buffer: 16 receipts cover the six-sample hold;
	// only requests that start inside that hold can enter this channel.
	heldReceipts := make(chan [2]uint64, 16)
	proxy := unixServer(t, func(w http.ResponseWriter, req *http.Request) {
		started := ipcclient.HostMonotonicNS()
		body, err := io.ReadAll(req.Body)
		if err != nil {
			t.Error(err)
			return
		}
		var command control.Request
		if req.URL.Path == "/v1/control" {
			if err = json.Unmarshal(body, &command); err != nil {
				t.Error(err)
				return
			}
		}
		if command.Operation == "renew" && stallRenewal.Load() {
			stalledRenewals.Add(1)
			// Request cancellation is the injected stall's release condition.
			<-req.Context().Done()
			return
		}
		forward, err := http.NewRequestWithContext(req.Context(), req.Method, "http://localhost"+req.URL.RequestURI(), bytes.NewReader(body))
		if err != nil {
			t.Error(err)
			return
		}
		forward.Header = req.Header.Clone()
		response, err := upstream.Do(forward)
		if err != nil {
			t.Error(err)
			return
		}
		raw, err := io.ReadAll(response.Body)
		response.Body.Close()
		if err != nil {
			t.Error(err)
			return
		}
		if command.Operation == "home" {
			holdStarted.Store(ipcclient.HostMonotonicNS())
			close(homeCompleted)
			select {
			case <-releaseHome:
			case <-req.Context().Done():
				return
			}
		}
		w.WriteHeader(response.StatusCode)
		_, _ = w.Write(raw)
		if command.Operation == "renew" && response.StatusCode == http.StatusOK {
			if boundary := holdStarted.Load(); boundary != 0 && started >= boundary {
				heldRenewals.Add(1)
				select {
				case heldReceipts <- [2]uint64{started, ipcclient.HostMonotonicNS()}:
				case <-req.Context().Done():
				}
			}
		}
	})
	c, err := control.Dial(control.UnixPath(proxy))
	simfixture.Must(t, err)
	t.Cleanup(func() { c.Close() })
	ctx, cancel := context.WithTimeout(context.Background(), fixtureDeadline)
	defer cancel()
	grant, err := c.Acquire(ctx, "home-renewal-stall", control.Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: f.hash})
	simfixture.Must(t, err)
	// Capture the origin before StartRenewal can queue an earlier request start.
	previous := ipcclient.HostMonotonicNS()
	// Unverified test cadence: 50,000,000 ns, independent of Home's duration.
	updates, err := c.StartRenewal(ctx, 50*time.Millisecond)
	simfixture.Must(t, err)
	defer c.StopRenewal()
	// Unverified cadence bound: 150,000,000 ns between successful receipts
	// and request starts; require three baseline samples before starting Home.
	const cadence = 150 * time.Millisecond
	if pendingBaseline {
		// Unverified regression fixture: one successful receipt queued before
		// the observer resumes; channel synchronization replaces scheduler luck.
		sourceUpdates := updates
		pendingUpdates := make(chan control.Renewal, 1)
		select {
		case first, open := <-sourceUpdates:
			if !open || first.Err != nil {
				t.Fatalf("pending baseline renewal failed: open=%v update=%+v", open, first)
			}
			pendingUpdates <- first
		case <-ctx.Done():
			t.Fatal(ctx.Err())
		}
		go func() {
			defer close(pendingUpdates)
			for update := range sourceUpdates {
				select {
				case pendingUpdates <- update:
				case <-ctx.Done():
					return
				}
			}
		}()
		updates = pendingUpdates
	}
	for count := 0; count < 3; count++ {
		timer := time.NewTimer(cadence)
		select {
		case update, open := <-updates:
			timer.Stop()
			startGap := int64(update.StartedHostMonotonicNS) - int64(previous)
			receiptGap := int64(ipcclient.HostMonotonicNS()) - int64(update.StartedHostMonotonicNS)
			if !open || update.Err != nil || startGap < 0 || startGap > int64(cadence) ||
				receiptGap < 0 || receiptGap > int64(cadence) {
				t.Fatalf("baseline renewal %d missed cadence: open=%v update=%+v", count+1, open, update)
			}
			previous = update.StartedHostMonotonicNS
		case <-timer.C:
			t.Fatal("baseline renewal receipt exceeded 150000000 ns")
		case <-ctx.Done():
			timer.Stop()
			t.Fatal(ctx.Err())
		}
	}
	homeResult := make(chan error, 1)
	go func() { _, err := c.Home(ctx, 511); homeResult <- err }()
	defer unblock()
	select {
	case <-homeCompleted:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	homed := f.poll(t, func(s control.Status) bool { return s.Core.HomeValidMask == 511 && s.Core.CommissioningPhase == 0 })
	if homed.Core.Armed != 0 || homed.Core.AxisEnableMask != 0 || homed.Core.ExecutionFaultReasons != 0 {
		t.Fatalf("native Home failed or energized after completion: %+v", homed.Core)
	}
	select {
	case err := <-homeResult:
		t.Fatalf("Home receipt bypassed stall: %v", err)
	default:
	}
	previous = holdStarted.Load()
	// Unverified sample count: six successful renewals must start and complete
	// after native Home finishes, with its HTTP receipt still held throughout.
	for count := 0; count < 6; count++ {
		timer := time.NewTimer(cadence)
		select {
		case receipt := <-heldReceipts:
			timer.Stop()
			startGap := int64(receipt[0]) - int64(previous)
			receiptGap := int64(receipt[1]) - int64(receipt[0])
			if receipt[0] < holdStarted.Load() || startGap < 0 || startGap > int64(cadence) ||
				receiptGap < 0 || receiptGap > int64(cadence) {
				t.Fatalf("held Home renewal %d missed cadence: previous=%d start=%d receipt=%d ns", count+1, previous, receipt[0], receipt[1])
			}
			previous = receipt[0]
			t.Logf("held Home renewal %d: start=%d receipt=%d ns", count+1, receipt[0], receipt[1])
		case <-timer.C:
			t.Fatal("held Home renewal receipt exceeded 150000000 ns")
		case <-ctx.Done():
			timer.Stop()
			t.Fatal(ctx.Err())
		}
	}
	select {
	case err := <-homeResult:
		t.Fatalf("Home receipt escaped hold during renewal cadence check: %v", err)
	default:
	}
	stallRenewal.Store(true)
	var failed control.Renewal
	for failed.Err == nil {
		select {
		case update, open := <-updates:
			if !open {
				t.Fatal("keeper closed without terminal failure")
			}
			failed = update
		case <-ctx.Done():
			t.Fatal(ctx.Err())
		}
	}
	if !errors.Is(failed.Err, context.DeadlineExceeded) || !errors.Is(failed.Err, control.ErrTransport) {
		t.Fatalf("renewal lost typed reason: %v", failed.Err)
	}
	if code, _ := outcome(failed.Err); code != 4 {
		t.Fatalf("renewal exit=%d", code)
	}
	if heldRenewals.Load() < 6 || stalledRenewals.Load() != 1 {
		t.Fatalf("renewals inside completed Home hold=%d stalled=%d", heldRenewals.Load(), stalledRenewals.Load())
	}
	select {
	case _, open := <-updates:
		if open {
			t.Fatal("keeper retried failed lease")
		}
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	expired := f.poll(t, func(s control.Status) bool {
		return s.Core.ControlGeneration > homed.Core.ControlGeneration && s.Core.LeaseValid == 0
	})
	// Native expiry and the adapter's subsequent revoke can each fence the
	// connection. Require retirement of the original identity, not one event.
	if expired.Core.Armed != 0 || expired.Core.AxisEnableMask != 0 || expired.Core.NativeHomeActiveMask != 0 || expired.Core.ExecutionFaultReasons != 0 {
		t.Fatalf("stalled renewal restored outputs: %+v", expired.Core)
	}
	for axis, state := range expired.Core.Axes {
		// CiA402 masked statusword 0x27 denotes Operation Enabled.
		if state.Statusword&0x6f == 0x27 {
			t.Fatalf("axis %d remained Operation Enabled after expiry", axis)
		}
	}
	unblock()
	select {
	case err := <-homeResult:
		simfixture.Must(t, err)
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	if c.Grant().Generation != grant.Generation {
		t.Fatal("late Home receipt silently reacquired authority")
	}
	t.Logf("renewal after native Home completion: ErrTransport + DeadlineExceeded, exit 4, successful held-response renewals=%d, stalled requests=1, outputs inhibited", heldRenewals.Load())
}
