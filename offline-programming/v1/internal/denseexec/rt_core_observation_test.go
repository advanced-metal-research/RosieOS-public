//go:build linux

package denseexec

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

type currentStatusClient struct {
	RTCoreClient
	mu               sync.Mutex
	full, compact    int
	entered, release chan struct{}
}

func (c *currentStatusClient) Status(ctx context.Context) (control.Status, error) {
	c.mu.Lock()
	c.full++
	c.mu.Unlock()
	return c.RTCoreClient.Status(ctx)
}

func (c *currentStatusClient) StatusObservation(ctx context.Context) (control.Status, error) {
	c.mu.Lock()
	c.compact++
	entered, release := c.entered, c.release
	c.mu.Unlock()
	if entered != nil {
		close(entered)
		select {
		case <-release:
		case <-ctx.Done():
			return control.Status{}, ctx.Err()
		}
	}
	s, err := c.RTCoreClient.Status(ctx)
	s.Execution.Handles = nil
	return s, err
}

func (c *currentStatusClient) counts() (int, int) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.full, c.compact
}

func currentStatusRefusal(t *testing.T, err error, reason string) {
	t.Helper()
	var refused *control.Rejected
	if !errors.As(err, &refused) || refused.Reason != reason {
		t.Fatalf("refusal = %v; want %s", err, reason)
	}
}

func currentStatusFake(t *testing.T) (*RTCoreController, *fakeRTCore, *currentStatusClient) {
	t.Helper()
	r, f, _ := rtFake(t)
	r.cancel()
	<-r.done // Explicit reads below exercise the real observer without a timer race.
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.status.TimeNS, f.status.Motion.TimeNS = 10, 10
	f.status.Execution.Handles = []control.HandleRecord{{Handle: 17, State: "prepared"}}
	f.mu.Unlock()
	if !r.Status(context.Background()).Available {
		t.Fatal("fixture did not obtain a full initial observation")
	}
	c := &currentStatusClient{RTCoreClient: f}
	r.c = c
	return r, f, c
}

func TestRTCoreCurrentObservationRouting(t *testing.T) {
	r, _, c := currentStatusFake(t)
	ctx := context.Background()
	if err := r.observe(ctx); err != nil {
		t.Fatal(err)
	}
	if got := r.ControlStatus(ctx); !got.Available || len(got.RTCore.Status.Execution.Handles) != 0 {
		t.Fatal("control poll did not use current-only evidence")
	}
	if got := r.Status(ctx); !got.Available || len(got.RTCore.Status.Execution.Handles) != 1 {
		t.Fatal("diagnostic Status lost full history")
	}
	if full, compact := c.counts(); full != 1 || compact != 2 {
		t.Fatalf("idle readers: full=%d compact=%d", full, compact)
	}
	r.mu.Lock()
	r.move = &jointMoveRun{}
	r.mu.Unlock()
	if err := r.observe(ctx); err != nil {
		t.Fatal(err)
	}
	if got := r.ControlStatus(ctx); len(got.RTCore.Status.Execution.Handles) != 1 {
		t.Fatal("active move lost its prepared handle")
	}
	if full, compact := c.counts(); full != 3 || compact != 2 {
		t.Fatalf("move readers: full=%d compact=%d", full, compact)
	}
	r.mu.Lock()
	r.move = nil
	r.mu.Unlock()
}

func TestRTCoreCurrentObservationLegacyFallback(t *testing.T) {
	r, _, c := currentStatusFake(t)
	r.c = c.RTCoreClient // An alternate client without StatusObservation.
	if err := r.observe(context.Background()); err != nil {
		t.Fatal(err)
	}
	if got := r.ControlStatus(context.Background()); len(got.RTCore.Status.Execution.Handles) != 1 {
		t.Fatal("legacy client's full read was not retained")
	}
}

// A control read can reach the adapter after a joint move has already obtained
// its full receipt. Even a newer compact publication must not erase that receipt
// or refresh nativeSeen on its behalf. Faults must nevertheless remain visible.
func TestRTCoreLateCurrentObservationPreservesMoveReceipts(t *testing.T) {
	for _, poll := range []bool{false, true} {
		for _, fault := range []bool{false, true} {
			name := "observer"
			if poll {
				name = "control-poll"
			}
			if fault {
				name += "/fault"
			}
			t.Run(name, func(t *testing.T) {
				r, f, c := currentStatusFake(t)
				c.entered, c.release = make(chan struct{}), make(chan struct{})
				ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
				defer cancel()
				result := make(chan error, 1)
				var body *StatusBody
				go func() {
					if poll {
						body = r.ControlStatus(ctx)
						result <- nil
					} else {
						result <- r.observe(ctx)
					}
				}()
				select {
				case <-c.entered:
				case <-ctx.Done():
					t.Fatal("compact read did not enter")
				}
				r.mu.Lock()
				r.move = &jointMoveRun{}
				r.mu.Unlock()
				_ = r.Status(ctx) // The move has a full prepared-handle publication.
				r.mu.Lock()
				before, seen := r.observation, time.Now().Add(-time.Second)
				r.nativeSeen = seen
				r.mu.Unlock()
				f.mu.Lock()
				f.status.TimeNS++
				f.status.Motion.TimeNS++
				if fault {
					f.status.Core.SafetyFaultMask = 1
				}
				f.mu.Unlock()
				close(c.release)
				err := <-result
				if fault && !poll {
					currentStatusRefusal(t, err, "rt_core_fault: 1")
				} else if err != nil {
					t.Fatal(err)
				}
				if poll && (body.RTCore.Status.TimeNS != 11 || (body.RTCore.Status.Core.SafetyFaultMask != 0) != fault) {
					t.Fatal("late UI poll hid current fault/time evidence")
				}
				r.mu.Lock()
				preserved := r.observation == before && r.nativeSeen.Equal(seen)
				r.move = nil
				r.mu.Unlock()
				if !preserved {
					t.Fatal("late compact response replaced move receipts or refreshed nativeSeen")
				}
			})
		}
	}
}

func TestRTCoreCurrentObservationPreservesFreshnessAndFaultChecks(t *testing.T) {
	for _, test := range []struct {
		name, reason string
		change       func(*control.Status)
	}{
		{"clock", "", func(s *control.Status) { s.Motion.TimeNS = 0 }},
		{"incarnation", "rt_core_incarnation_changed", func(s *control.Status) { s.DaemonIncarnation = "replacement" }},
		{"fault", "rt_core_fault: 4", func(s *control.Status) { s.Core.SafetyFaultMask = 4 }},
		{"axis", "rt_core_fault: axis faulted", func(s *control.Status) { s.Axes[0].Readiness = "faulted" }},
		{"execution", "native_execution_failed", func(s *control.Status) { s.Execution.Error = "native_execution_failed" }},
	} {
		t.Run(test.name, func(t *testing.T) {
			r, f, _ := currentStatusFake(t)
			f.mu.Lock()
			test.change(&f.status)
			f.mu.Unlock()
			err := r.observe(context.Background())
			if test.reason == "" {
				if !errors.Is(err, control.ErrProtocol) {
					t.Fatalf("invalid clock accepted: %v", err)
				}
			} else {
				currentStatusRefusal(t, err, test.reason)
			}
		})
	}
	t.Run("unchanged-publication", func(t *testing.T) {
		r, f, _ := currentStatusFake(t)
		r.mu.Lock()
		seen := time.Now().Add(-r.timingLocked().StatusAge - time.Second)
		r.nativeSeen = seen
		r.mu.Unlock()
		_ = r.ControlStatus(context.Background())
		currentStatusRefusal(t, r.observe(context.Background()), "rt_core_status_stale")
		r.mu.Lock()
		unchanged := r.nativeSeen.Equal(seen)
		r.mu.Unlock()
		if !unchanged {
			t.Fatal("identical compact publication rejuvenated readiness")
		}
		f.mu.Lock()
		f.status.TimeNS++
		f.status.Motion.TimeNS++
		f.mu.Unlock()
		if err := r.observe(context.Background()); err != nil {
			t.Fatal(err)
		}
	})
}

func TestRTCoreCurrentObservationCancellation(t *testing.T) {
	r, _, c := currentStatusFake(t)
	c.entered, c.release = make(chan struct{}), make(chan struct{})
	r.mu.Lock()
	before, seen := r.observation, r.nativeSeen
	r.mu.Unlock()
	ctx, cancel := context.WithCancel(context.Background())
	result := make(chan error, 1)
	go func() { result <- r.observe(ctx) }()
	<-c.entered
	cancel()
	if err := <-result; !errors.Is(err, context.Canceled) {
		t.Fatalf("cancellation: %v", err)
	}
	r.mu.Lock()
	unchanged := r.observation == before && r.nativeSeen.Equal(seen)
	r.mu.Unlock()
	if !unchanged {
		t.Fatal("cancelled read changed retained evidence")
	}
}

func TestJointMoveRetainsHandleReceiptsWithCurrentStatusClient(t *testing.T) {
	r, f, in := moveFake(t)
	r.cancel()
	<-r.done
	c := &currentStatusClient{RTCoreClient: f}
	r.c = c
	if _, err := r.JointMove(context.Background(), in); err != nil {
		t.Fatalf("joint move with compact-capable client: %v", err)
	}
	if full, _ := c.counts(); full == 0 {
		t.Fatal("move completed without full native receipts")
	}
}
