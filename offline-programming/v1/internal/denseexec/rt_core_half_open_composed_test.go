//go:build linux

package denseexec

import (
	"context"
	"errors"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

func halfOpenController(t *testing.T) (*RTCoreController, *control.Client, *jitterConnections) {
	t.Helper()
	c, d, _, socket, counts := delayedJointFixture(t, true)
	if _, err := c.Describe(context.Background()); err != nil {
		t.Fatal(err)
	}
	if _, err := c.PrepareAcquisition(context.Background()); err != nil {
		t.Fatal(err)
	}
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:half-open"}, c)
	if err != nil {
		t.Fatal(err)
	}
	r.cancel()
	<-r.done
	t.Cleanup(func() { r.Close(context.Background()) })
	observer, err := control.Dial(control.UnixPath(filepath.Join(filepath.Dir(socket), "control.sock")))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { observer.Close() })
	return r, observer, counts
}

func blockReservedStatus(counts *jitterConnections) <-chan struct{} {
	counts.mu.Lock()
	defer counts.mu.Unlock()
	counts.halfOpenPath, counts.halfOpenRemaining = "/v1/status", 1
	counts.halfOpenEntered = make(chan struct{}, 1)
	return counts.halfOpenEntered
}

func awaitHalfOpen(t *testing.T, entered <-chan struct{}, r *RTCoreController, observer *control.Client) {
	t.Helper()
	// Unverified absolute admission watchdog: 5000 ms under the delayed fixture.
	select {
	case <-entered:
	case <-time.After(5000 * time.Millisecond):
		t.Fatal("reserved status not reached")
	}
	stack := make([]byte, 1<<20)
	n := runtime.Stack(stack, true)
	if !strings.Contains(string(stack[:n]), "(*Client).warmConnections") || !strings.Contains(string(stack[:n]), "(*RTCoreController).Arm") {
		t.Fatalf("missing Arm/reservation stack:\n%s", stack[:n])
	}
	t.Logf("reserved half-open Arm stack:\n%s", stack[:n])
	s, err := observer.Status(context.Background())
	if err != nil || s.Core.Armed != 0 || (s.Grant != nil && s.Grant.GrantActive != 0) {
		t.Fatalf("adapter held authority before Acquire: core=%+v grant=%+v err=%v", s.Core, s.Grant, err)
	}
	// Unverified poll watchdog: 1000 ms. A receipt, not elapsed time, passes.
	pollCtx, cancel := context.WithTimeout(context.Background(), 1000*time.Millisecond)
	defer cancel()
	status := r.Status(pollCtx)
	if !status.Available || status.Armed == nil || *status.Armed || pollCtx.Err() != nil {
		t.Fatalf("status blocked by Arm: %+v", status)
	}
}

// TestHalfOpenReservedArmComposed requires one deadline-driven reopen to finish
// acquisition after a relay withholds a reserved response before adapter admission.
func TestHalfOpenReservedArmComposed(t *testing.T) {
	r, observer, counts := halfOpenController(t)
	entered := blockReservedStatus(counts)
	done := make(chan error, 1)
	go func() { _, err := r.Arm(context.Background(), true); done <- err }()
	awaitHalfOpen(t, entered, r, observer)
	// Unverified absolute completion watchdog: 10000 ms including cold reopen.
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(10000 * time.Millisecond):
		t.Fatal("Arm retained busy")
	}
	s, err := observer.Status(context.Background())
	if err != nil || s.Core.Armed != 1 || s.Grant == nil || s.Grant.GrantActive != 1 {
		t.Fatalf("Arm not confirmed: %+v %v", s.Core, err)
	}
	counts.mu.Lock()
	defer counts.mu.Unlock()
	if counts.opened.Load() != 8 {
		t.Fatalf("connections=%d want=8", counts.opened.Load())
	}
}

// TestBlockedArmBreakoutComposed requires Stop and Disarm to cancel reservation
// preparation without grant traffic, then permits a fresh Arm on the same client.
func TestBlockedArmBreakoutComposed(t *testing.T) {
	for _, disarm := range []bool{false, true} {
		name := "stop"
		if disarm {
			name = "disarm"
		}
		t.Run(name, func(t *testing.T) {
			r, observer, counts := halfOpenController(t)
			entered := blockReservedStatus(counts)
			done := make(chan error, 1)
			go func() { _, err := r.Arm(context.Background(), true); done <- err }()
			awaitHalfOpen(t, entered, r, observer)
			if disarm {
				if _, err := r.Arm(context.Background(), false); err != nil {
					t.Fatal(err)
				}
			} else {
				if stopped, err := r.Stop(context.Background()); err != nil || stopped.Note != "nothing_held" {
					t.Fatalf("Stop: %+v %v", stopped, err)
				}
			}
			// Unverified absolute cancellation watchdog: 1000 ms.
			select {
			case err := <-done:
				if err == nil {
					t.Fatal("cancelled Arm accepted")
				}
			case <-time.After(1000 * time.Millisecond):
				t.Fatal("cancelled Arm retained busy")
			}
			if _, err := r.Arm(context.Background(), true); err != nil {
				t.Fatal(err)
			}
		})
	}
}

// TestHeartbeatStopRearmHalfOpenComposed preserves the reported Stop/re-Arm gap
// and rejects old browser sessions without cancelling the replacement authority.
func TestHeartbeatStopRearmHalfOpenComposed(t *testing.T) {
	r, observer, counts := halfOpenController(t)
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	old := r.statusSnapshot().Session.ID
	oldGeneration := r.c.Grant().Generation
	if _, err := r.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	// Reported stimulus: Arm starts 4000 ms after Stop; success requires receipts.
	timer := time.NewTimer(4000 * time.Millisecond)
	defer timer.Stop()
	<-timer.C
	entered := blockReservedStatus(counts)
	done := make(chan error, 1)
	go func() { _, err := r.Arm(context.Background(), true); done <- err }()
	awaitHalfOpen(t, entered, r, observer)
	for i := 0; i < 2; i++ {
		_, err := r.Heartbeat(old)
		var failure *Error
		if !errors.As(err, &failure) || failure.Code != "dense_session_mismatch" {
			t.Fatalf("old heartbeat: %v", err)
		}
	}
	// Unverified absolute result watchdog: 10000 ms, including reservation retry.
	select {
	case err := <-done:
		if err != nil {
			t.Fatal(err)
		}
	case <-time.After(10000 * time.Millisecond):
		t.Fatal("replacement Arm retained busy")
	}
	fresh := r.statusSnapshot().Session.ID
	if fresh == old || r.c.Grant().Generation <= oldGeneration {
		t.Fatal("replacement session or generation was reused")
	}
	if _, err := r.Heartbeat(fresh); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Heartbeat(old); err == nil {
		t.Fatal("retired session heartbeat accepted")
	}
	s, err := observer.Status(context.Background())
	if err != nil || s.Core.Armed != 1 || s.Grant == nil || s.Grant.GrantActive != 1 {
		t.Fatalf("old heartbeat retired new grant: %+v %v", s.Core, err)
	}
}
