//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"
)

func TestManualIOSessionDoesNotArmOrRevive(t *testing.T) {
	r, f, _ := rtFake(t)
	ctx := context.Background()
	token, err := r.BeginManualIO(ctx)
	if err != nil {
		t.Fatal(err)
	}
	request := ManualIORequest{Session: token, Sequence: 1, Samples: []ManualIOSample{{Channel: "O1", Value: true, Begin: true}}}
	if err = r.ManualIO(ctx, request); err != nil {
		t.Fatal(err)
	}
	// Delayed duplicates never produce another native intent.
	if err = r.ManualIO(ctx, request); errorCode(err) != "io_test_expired" {
		t.Fatal(err)
	}
	r.mu.Lock()
	r.manualDeadline = time.Now().Add(-time.Millisecond)
	r.mu.Unlock()
	request.Sequence = 2
	request.Samples[0].Begin = false
	if err = r.ManualIO(ctx, request); errorCode(err) != "io_test_expired" {
		t.Fatal(err)
	}
	request.Sequence = 3
	if err = r.ManualIO(ctx, request); errorCode(err) != "io_test_expired" {
		t.Fatal(err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	count := 0
	for _, c := range f.calls {
		if c == "io_test" {
			count++
		}
		if c == "arm" || c == "enable" || c == "stop" {
			t.Fatal("manual testing changed motion authority", f.calls)
		}
	}
	if count != 1 {
		t.Fatal("expired/duplicate session reached native", f.calls)
	}
}
func TestManualIORefusalDoesNotStopL2(t *testing.T) {
	r, f, _ := rtFake(t)
	ctx := context.Background()
	token, err := r.BeginManualIO(ctx)
	if err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.refuse = "io_test"
	f.mu.Unlock()
	err = r.ManualIO(ctx, ManualIORequest{Session: token, Sequence: 1, Samples: []ManualIOSample{{Channel: "O1", Value: true, Begin: true}}})
	if err == nil {
		t.Fatal("refusal lost")
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	for _, c := range f.calls {
		if c == "stop" || c == "arm" || c == "enable" {
			t.Fatal(f.calls)
		}
	}
}

// A setup token can survive a delayed empty HTTP reply; that token must not
// create native output renewals. The native 200 ms per-output expiry is separate.
func TestManualIOEmptySetupBudgetDoesNotRenewOutputs(t *testing.T) {
	r, f, _ := rtFake(t)
	token, err := r.BeginManualIO(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	time.Sleep(200 * time.Millisecond)
	if err = r.ManualIO(context.Background(), ManualIORequest{Session: token, Sequence: 1}); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	for _, call := range f.calls {
		if call == "io_test" || call == "arm" || call == "enable" {
			t.Fatal("empty setup changed outputs", f.calls)
		}
	}
	f.mu.Unlock()
	r.mu.Lock()
	r.manualDeadline = time.Now().Add(-time.Millisecond)
	r.mu.Unlock()
	if err = r.ManualIO(context.Background(), ManualIORequest{Session: token, Sequence: 2}); errorCode(err) != "io_test_expired" {
		t.Fatal(err)
	}
}
