//go:build linux

package denseexec

import (
	"context"
	"errors"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

type delayedHomeClient struct {
	*fakeRTCore
	delay time.Duration
	calls int
}

func (c *delayedHomeClient) Command(ctx context.Context, request *control.Request) (control.Response, error) {
	c.calls++
	select {
	case <-time.After(c.delay):
		return control.Response{Operation: request.Operation, Sequence: 1}, nil
	case <-ctx.Done():
		return control.Response{}, ctx.Err()
	}
}

func TestHomeReceiptOutlivesNetworkBudgetWithoutReplay(t *testing.T) {
	r, f, _ := rtFake(t)
	r.roundTrip = time.Millisecond
	r.effectiveLease = 500 * time.Millisecond
	c := &delayedHomeClient{fakeRTCore: f, delay: r.callBudget() + 100*time.Millisecond}
	r.c = c
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	if err := r.commandMasked(ctx, "home", control.Identity{}, 63); err != nil {
		t.Fatalf("commissioning receipt was clipped to network latency: %v", err)
	}
	if c.calls != 1 {
		t.Fatalf("Home calls = %d, want one", c.calls)
	}
}

func TestHomeReceiptRetainsCallerCancellation(t *testing.T) {
	r, f, _ := rtFake(t)
	c := &delayedHomeClient{fakeRTCore: f, delay: time.Second}
	r.c = c
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Millisecond)
	defer cancel()
	if err := r.commandMasked(ctx, "home", control.Identity{}, 63); !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("Home ignored operation cancellation: %v", err)
	}
	if c.calls != 1 {
		t.Fatalf("cancelled Home replayed: %d calls", c.calls)
	}
}
