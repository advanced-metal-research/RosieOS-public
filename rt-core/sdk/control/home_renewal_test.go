//go:build linux

package control

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// TestHomeCompletedRenewalStall requires successful renewal receipts at a fixed
// cadence while Home's successful response is held. Limiting each transport to
// one connection prevents a shared lifecycle pool from hiding behind extra dials.
// The benchdrive test with this name separately covers native Home and expiry.
func TestHomeCompletedRenewalStall(t *testing.T) {
	// Unverified test budgets: 50 ms renewal interval, 150 ms maximum receipt
	// gap, 5 s cancellation ceiling; none are derived from observed performance.
	const interval, receiptGap, ceiling = 50 * time.Millisecond, 150 * time.Millisecond, 5 * time.Second
	homeHeld, releaseHome := make(chan struct{}), make(chan struct{})
	var releaseOnce sync.Once
	unblock := func() { releaseOnce.Do(func() { close(releaseHome) }) }
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Method == http.MethodGet && r.URL.Path == "/v1/describe" {
			// Unverified fixture ceilings: LAN's 500 ms lease and 250 ms input age.
			_ = json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: "describe",
				Data: json.RawMessage(`{"max_grant_lease_ns":500000000,"max_jog_input_age_ns":250000000}`)})
			return
		}
		var request Request
		if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
			t.Error(err)
			return
		}
		// Unverified grant fixture: session owner, generation 1, lease 500 ms.
		grant, err := json.Marshal(Grant{Session: "owner", Generation: 1, LeaseMS: 500})
		if err != nil {
			t.Error(err)
			return
		}
		response := Response{Schema: "rosie.rt-control.response.v1", Operation: request.Operation, Data: grant}
		if request.Operation == "home" {
			close(homeHeld)
			select {
			case <-releaseHome:
			case <-r.Context().Done():
				return
			}
		}
		_ = json.NewEncoder(w).Encode(response)
	}))
	defer server.Close()
	defer unblock()
	// Unverified contention fixture: one active connection per independent pool.
	c := newClient(server.URL, &http.Transport{MaxConnsPerHost: 1})
	defer c.Close()
	ctx, cancel := context.WithTimeout(context.Background(), ceiling)
	defer cancel()
	_, err := c.Acquire(ctx, "home-renewal", Binding{})
	must(t, err)
	// Start the receipt budget before the keeper can queue its first renewal.
	previousReceipt := ipcclient.HostMonotonicNS()
	updates, err := c.StartRenewal(ctx, interval)
	must(t, err)
	var previousStart uint64
	observe := func(phase string, count int, after uint64) {
		t.Helper()
		for received := 0; received < count; {
			remaining := receiptGap - time.Duration(int64(ipcclient.HostMonotonicNS())-int64(previousReceipt))
			if remaining <= 0 {
				t.Fatalf("%s: renewal receipt gap exceeded 150000000 ns", phase)
			}
			timer := time.NewTimer(remaining)
			select {
			case update, open := <-updates:
				timer.Stop()
				now := ipcclient.HostMonotonicNS()
				if !open || update.Err != nil {
					t.Fatalf("%s: renewal failed: open=%v err=%v", phase, open, update.Err)
				}
				receiptDelta := int64(now) - int64(previousReceipt)
				requestDelta := int64(now) - int64(update.StartedHostMonotonicNS)
				startDelta := int64(update.StartedHostMonotonicNS) - int64(previousStart)
				if receiptDelta < 0 || receiptDelta > int64(receiptGap) || requestDelta < 0 || requestDelta > int64(receiptGap) ||
					(previousStart != 0 && (startDelta < 0 || startDelta > int64(receiptGap))) {
					t.Fatalf("%s: renewal cadence exceeded 150000000 ns: previous=%d start=%d receipt=%d", phase, previousStart, update.StartedHostMonotonicNS, now)
				}
				previousReceipt, previousStart = now, update.StartedHostMonotonicNS
				if update.StartedHostMonotonicNS >= after {
					received++
					t.Logf("%s renewal %d: start=%d receipt=%d ns", phase, received, previousStart, now)
				}
			case <-timer.C:
				t.Fatalf("%s: renewal receipt gap exceeded 150000000 ns while Home response remained held", phase)
			case <-ctx.Done():
				timer.Stop()
				t.Fatal(ctx.Err())
			}
		}
	}
	// Unverified sample counts: three baseline receipts and six inside the hold.
	observe("baseline", 3, 0)
	homeResult := make(chan error, 1)
	go func() { _, err := c.Home(ctx, 511); homeResult <- err }()
	select {
	case <-homeHeld:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	observe("held Home", 6, ipcclient.HostMonotonicNS())
	select {
	case err := <-homeResult:
		t.Fatalf("Home response escaped hold: %v", err)
	default:
	}
	unblock()
	select {
	case err := <-homeResult:
		must(t, err)
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
}
