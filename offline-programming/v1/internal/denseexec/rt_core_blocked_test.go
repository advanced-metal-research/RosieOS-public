//go:build linux

package denseexec

import (
	"context"
	"errors"
	"runtime"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

type blockedPreparation struct {
	*fakeRTCore
	entered chan struct{}
}

type blockedDiscovery struct {
	*fakeRTCore
	entered chan struct{}
}

func (c *blockedDiscovery) Describe(ctx context.Context) (control.Description, error) {
	close(c.entered)
	<-ctx.Done()
	return control.Description{}, ctx.Err()
}

// TestBlockedArmDiscoveryLeavesStatusAvailable permits observation before the
// first description completes, preventing a cold Arm from hiding native status.
func TestBlockedArmDiscoveryLeavesStatusAvailable(t *testing.T) {
	r, f, _ := rtFake(t)
	r.cancel()
	<-r.done
	discovery := &blockedDiscovery{f, make(chan struct{})}
	r.c = discovery
	done := make(chan error, 1)
	go func() { _, err := r.Arm(context.Background(), true); done <- err }()
	<-discovery.entered
	// Unverified absolute observation watchdog: 500 ms, with discovery withheld.
	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()
	status := r.Status(ctx)
	if ctx.Err() != nil || !status.Available || status.Armed == nil || *status.Armed {
		t.Fatalf("discovery blocked status: %+v", status)
	}
	if _, err := r.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	select {
	case err := <-done:
		if err == nil {
			t.Fatal("cancelled discovery accepted")
		}
	case <-ctx.Done():
		t.Fatal("discovery did not cancel")
	}
}

type blockedRenewalStartup struct {
	*fakeRTCore
	entered chan struct{}
}

func (c *blockedRenewalStartup) StartRenewal(ctx context.Context, _ time.Duration) (<-chan control.Renewal, error) {
	close(c.entered)
	<-ctx.Done()
	return nil, ctx.Err()
}

// TestBlockedArmKeeperStartupCancelsWithStop exercises the real acquisition
// lock order, preventing detached keeper startup from ignoring Arm cancellation.
func TestBlockedArmKeeperStartupCancelsWithStop(t *testing.T) {
	r, f, _ := rtFake(t)
	r.cancel()
	<-r.done
	startup := &blockedRenewalStartup{f, make(chan struct{})}
	r.c = startup
	done := make(chan error, 1)
	go func() { _, err := r.Arm(context.Background(), true); done <- err }()
	<-startup.entered
	stopped := make(chan error, 1)
	go func() { _, err := r.Stop(context.Background()); stopped <- err }()
	// Unverified absolute startup cancellation watchdog: 500 ms for each receipt.
	select {
	case err := <-stopped:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(500 * time.Millisecond):
		t.Fatal("Stop blocked by keeper startup")
	}
	select {
	case err := <-done:
		if err == nil {
			t.Fatal("cancelled startup accepted")
		}
	case <-time.After(500 * time.Millisecond):
		t.Fatal("Arm ignored startup cancellation")
	}
	if f.Grant().Session != "" {
		t.Fatal("startup cancellation retained authority")
	}
}

func (c *blockedPreparation) PrepareAcquisition(ctx context.Context) (control.AcquisitionSnapshot, error) {
	close(c.entered)
	<-ctx.Done()
	return control.AcquisitionSnapshot{}, ctx.Err()
}

// TestBlockedArmStopDisarmAndStatus requires observation and cancellation to
// complete while acquisition owns the operation lane and the adapter owns none.
func TestBlockedArmStopDisarmAndStatus(t *testing.T) {
	for _, disarm := range []bool{false, true} {
		name := "stop"
		if disarm {
			name = "disarm"
		}
		t.Run(name, func(t *testing.T) {
			r, f, _ := rtFake(t)
			r.cancel()
			<-r.done
			blocked := &blockedPreparation{fakeRTCore: f, entered: make(chan struct{})}
			r.c = blocked
			done := make(chan error, 1)
			go func() { _, err := r.Arm(context.Background(), true); done <- err }()
			<-blocked.entered
			stack := make([]byte, 1<<20)
			n := runtime.Stack(stack, true)
			if !strings.Contains(string(stack[:n]), "(*blockedPreparation).PrepareAcquisition") {
				t.Fatal("missing blocked Arm stack")
			}
			t.Logf("blocked Arm stack:\n%s", stack[:n])
			poll := make(chan *StatusBody, 1)
			go func() { poll <- r.Status(context.Background()) }()
			// Unverified absolute status and breakout watchdogs: 500 ms each.
			select {
			case status := <-poll:
				if !status.Available || status.Armed == nil || *status.Armed || f.Grant().Session != "" {
					t.Fatalf("no-grant observation: %+v", status)
				}
			case <-time.After(500 * time.Millisecond):
				t.Fatal("status waited behind Arm")
			}
			stopped := make(chan error, 1)
			go func() {
				if disarm {
					_, err := r.Arm(context.Background(), false)
					stopped <- err
				} else {
					_, err := r.Stop(context.Background())
					stopped <- err
				}
			}()
			select {
			case err := <-stopped:
				if err != nil {
					t.Fatal(err)
				}
			case <-time.After(500 * time.Millisecond):
				t.Fatal("breakout blocked")
			}
			select {
			case err := <-done:
				if err == nil {
					t.Fatal("cancelled Arm accepted")
				}
			case <-time.After(500 * time.Millisecond):
				t.Fatal("Arm retained busy")
			}
			if !r.opMu.TryLock() {
				t.Fatal("operation lane retained")
			}
			r.opMu.Unlock()
		})
	}
}

// TestBlockedArmStopLockCancellation prevents the stopMu/opCancel cycle: Stop
// must cancel the lock owner before waiting to serialize authority retirement.
func TestBlockedArmStopLockCancellation(t *testing.T) {
	r, _, _ := rtFake(t)
	entered, done := make(chan struct{}), make(chan error, 1)
	go func() {
		done <- r.operationKind(context.Background(), "arm", func(ctx context.Context) error {
			r.stopMu.Lock()
			defer r.stopMu.Unlock()
			close(entered)
			<-ctx.Done()
			return ctx.Err()
		})
	}()
	<-entered
	stopped := make(chan error, 1)
	go func() { _, err := r.Stop(context.Background()); stopped <- err }()
	// Unverified absolute cancellation watchdog: 500 ms, proven by both receipts.
	select {
	case err := <-stopped:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(500 * time.Millisecond):
		t.Fatal("Stop waited before cancellation")
	}
	select {
	case err := <-done:
		if err == nil {
			t.Fatal("cancelled operation accepted")
		}
	case <-time.After(500 * time.Millisecond):
		t.Fatal("operation still blocked")
	}
}

type unconfirmedArm struct{ *fakeRTCore }

type unconfirmedCleanup struct{ unconfirmedArm }

func (c *unconfirmedCleanup) Stop(ctx context.Context) (control.Grant, error) {
	<-ctx.Done()
	return control.Grant{}, ctx.Err()
}

// TestArmTimeoutBoundsUncertainStopCleanup includes a silent urgent lane in the
// overall Arm window, preventing cleanup from retaining busy after the timeout.
func TestArmTimeoutBoundsUncertainStopCleanup(t *testing.T) {
	r, f, _ := rtFake(t)
	r.cancel()
	<-r.done
	r.c = &unconfirmedCleanup{unconfirmedArm{f}}
	// Unverified fixture timing: 100 ms lease and 1 ns RTT at Arm entry.
	r.effectiveLease, r.roundTrip = 100*time.Millisecond, time.Nanosecond
	started := time.Now()
	result, err := r.Arm(context.Background(), true)
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "rt_core_arm_timeout" || !result.Status.RTCore.StopUncertain {
		t.Fatalf("unconfirmed cleanup: %+v %v", result, err)
	}
	// Unverified absolute result watchdog: 1000 ms including urgent cleanup.
	if elapsed := time.Since(started); elapsed > 1000*time.Millisecond {
		t.Fatalf("cleanup escaped Arm window: elapsed_ns=%d", elapsed)
	}
	if !r.opMu.TryLock() {
		t.Fatal("cleanup retained busy")
	}
	r.opMu.Unlock()
	// Restore the fake's acknowledged Stop for fixture teardown.
	r.c = f
}

type busyArm struct{ *fakeRTCore }

func (c *busyArm) Command(context.Context, *control.Request) (control.Response, error) {
	return control.Response{}, &control.Rejected{Reason: "busy"}
}

// TestArmCompletedBusyIsTerminal prevents a refused Arm or status read from
// leaving the UI's ARMING transition latched after the operation lane is free.
func TestArmCompletedBusyIsTerminal(t *testing.T) {
	r, f, _ := rtFake(t)
	r.cancel()
	<-r.done
	r.c = &busyArm{f}
	result, err := r.Arm(context.Background(), true)
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "rt_core_arm_refused" || result.Status.RTCore.Reason == "busy" {
		t.Fatalf("terminal Arm refusal: %+v %v", result, err)
	}
	r.mu.Lock()
	r.reason = ""
	r.mu.Unlock()
	f.mu.Lock()
	f.statusErr = &control.Rejected{Reason: "busy"}
	f.mu.Unlock()
	if status := r.Status(context.Background()); status.DaemonError != "rt_core_request_refused: busy" {
		t.Fatalf("idle status still signals an operation: %s", status.DaemonError)
	}
}

// TestArmTimeoutQueuedWaits releases expired Arm and status callers even when
// another operation retains the operation or discovery lock.
func TestArmTimeoutQueuedWaits(t *testing.T) {
	for _, discovery := range []bool{false, true} {
		r, _, _ := rtFake(t)
		r.cancel()
		<-r.done
		lock := &r.opMu
		if discovery {
			lock = &r.describeMu
		}
		lock.Lock()
		// Unverified caller deadline: 100 ms; absolute completion watchdog: 500 ms.
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		done := make(chan error, 1)
		go func() { _, err := r.Arm(ctx, true); done <- err }()
		select {
		case err := <-done:
			lock.Unlock()
			cancel()
			var failure *Error
			if !errors.As(err, &failure) || failure.Code != "rt_core_arm_timeout" {
				t.Fatalf("queued deadline: %v", err)
			}
		case <-time.After(500 * time.Millisecond):
			lock.Unlock()
			cancel()
			<-done
			t.Fatal("expired Arm retained a queued lock wait")
		}
	}
}

func (c *unconfirmedArm) Command(ctx context.Context, req *control.Request) (control.Response, error) {
	if req.Operation == "arm" {
		return control.Response{Operation: "arm", Sequence: 1}, nil
	}
	return c.fakeRTCore.Command(ctx, req)
}
func (c *unconfirmedArm) Status(ctx context.Context) (control.Status, error) {
	c.mu.Lock()
	c.status.TimeNS++
	c.status.Motion.TimeNS++
	c.mu.Unlock()
	return c.fakeRTCore.Status(ctx)
}

// TestArmTimeoutIncludesConfirmationAndCleanup requires a typed terminal result
// and a free operation lane when native confirmation never arrives.
func TestArmTimeoutIncludesConfirmationAndCleanup(t *testing.T) {
	r, f, _ := rtFake(t)
	r.cancel()
	<-r.done
	r.c = &unconfirmedArm{f}
	// Unverified fixture timing: 100 ms effective lease and 1 ns warm RTT.
	r.effectiveLease, r.roundTrip = 100*time.Millisecond, time.Nanosecond
	started := time.Now()
	_, err := r.Arm(context.Background(), true)
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "rt_core_arm_timeout" {
		t.Fatalf("confirmation timeout: %v", err)
	}
	// Unverified absolute result watchdog: 1000 ms for the 800000024 ns window.
	if elapsed := time.Since(started); elapsed > 1000*time.Millisecond {
		t.Fatalf("Arm elapsed_ns=%d", elapsed)
	}
	if !r.opMu.TryLock() {
		t.Fatal("timeout retained operation lane")
	}
	r.opMu.Unlock()
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.opCancel != nil || r.held || r.feedbackCode != "rt_core_arm_timeout" || r.reason == "busy" || f.Grant().Session != "" {
		t.Fatalf("timeout state: held=%v reason=%s feedback=%s", r.held, r.reason, r.feedbackCode)
	}
}
