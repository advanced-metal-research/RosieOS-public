//go:build linux

package denseexec

import (
	"context"
	"errors"
	"testing"

	"rosieos/rt-core/sdk/control"
)

// TestReviewRequestDeadlineRetryPreservesFence verifies exactly one retry and
// distinguishes the identical deadline error when it originates in renewal.
func TestReviewRequestDeadlineRetryPreservesFence(t *testing.T) {
	r, _, _ := rtFake(t)
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	calls := 0
	_, err := rtRequest(r, context.Background(), "rt_core_status_slow", func(context.Context) (control.Status, error) {
		calls++
		return control.Status{}, errors.Join(control.ErrTransport, context.DeadlineExceeded)
	})
	if calls != 2 || !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("calls=%d err=%v", calls, err)
	}
	_ = r.fail(err)
	if !r.statusSnapshot().Session.Lease.Held || r.statusSnapshot().RTCore.Note != "rt_core_status_slow" {
		t.Fatal("request deadline retired authority or lost note")
	}
	_ = r.fail(&rtAuthorityFailure{err})
	if r.statusSnapshot().Session.Lease.Held {
		t.Fatal("renewal deadline retained authority")
	}
}

// TestConnectionReopenedUnsentStatus preserves observed authority for a proven
// unsent command while a keeper failure still retires that same authority.
func TestConnectionReopenedUnsentStatus(t *testing.T) {
	r, f, _ := rtFake(t)
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	before := len(f.calls)
	f.mu.Unlock()
	failure := r.failKind(control.ErrConnectionUnsent, "arm")
	var api *Error
	if !errors.As(failure, &api) || api.Code != "rt_core_connection_reopened" {
		t.Fatal(failure)
	}
	s := r.statusSnapshot()
	if !s.Session.Lease.Held || s.RTCore.StopUncertain || s.RTCore.Note != "rt_core_connection_reopened" {
		t.Fatalf("status=%+v", s)
	}
	f.mu.Lock()
	after := len(f.calls)
	f.mu.Unlock()
	if after != before {
		t.Fatalf("unsent request issued cleanup: before=%d after=%d", before, after)
	}
	calls := 0
	_, err := rtRequest(r, context.Background(), "rt_core_status_slow", func(context.Context) (control.Status, error) {
		calls++
		return control.Status{}, control.ErrConnectionUnsent
	})
	if !errors.Is(err, control.ErrConnectionUnsent) || calls != 1 {
		t.Fatalf("additional retries=%d error=%v", calls, err)
	}
	_ = r.fail(&rtAuthorityFailure{control.ErrConnectionUnsent})
	if r.statusSnapshot().Session.Lease.Held {
		t.Fatal("unsent renewal retained authority")
	}
}
