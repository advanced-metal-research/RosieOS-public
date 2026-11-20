//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"testing"
	"time"
)

// Idle bus failures require explicit recovery. A disabled send failure with
// matching fresh position retains Home; missing exchange evidence fails closed.
// Stop submission alone cannot clear either latch.
type idleBusReviewObservation struct {
	Status
	CycleCount   uint64 `json:"rt_cycle_counter"`
	StartupReady uint32 `json:"startup_ready"`
	WKCActual    uint32 `json:"wkc_actual"`
	WKCExpected  uint32 `json:"wkc_expected"`
	WCState      uint32 `json:"domain_wc_state"`
}

func readIdleBusReviewObservation(path string) (idleBusReviewObservation, error) {
	var observation idleBusReviewObservation
	raw, err := os.ReadFile(path)
	if err == nil {
		err = json.Unmarshal(raw, &observation)
	}
	if err == nil {
		now := monotonicNS()
		if observation.TimeNS == 0 || observation.TimeNS > now || now-observation.TimeNS > uint64(time.Second) {
			err = errors.New("stale idle-bus observation")
		}
	}
	return observation, err
}

func waitIdleBusReviewObservation(t *testing.T, f fixture, predicate func(idleBusReviewObservation) bool) idleBusReviewObservation {
	t.Helper()
	deadline := time.Now().Add(8 * time.Second)
	path := filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json")
	var pollResult idleBusReviewObservation
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		observation, err := readIdleBusReviewObservation(path)
		if err != nil {
			t.Fatal(err)
		}
		if observation.ConfigurationSHA256 != f.options.ConfigurationSHA256 || observation.Backend != "simulation" {
			t.Fatalf("idle-bus observation identity mismatch: %+v", observation)
		}
		if predicate(observation) {
			pollResult = observation
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("idle-bus observation deadline: %+v", observation)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func TestHealthyDisarmedSendFailureLatchesFaultUntilExplicitRecovery(t *testing.T) {
	requireIdleBusFailureRecovery(t, false, "--sim-send-fail-from-call", "5000", "--sim-send-fail-for-calls", "600")
}

func TestHealthyDisarmedWKCLossLatchesFaultUntilExplicitRecovery(t *testing.T) {
	// WKC incompleteness has successful API return codes. It is loss of fresh
	// process data (bit 7), not a fabricated negative send/receive error (bit 8).
	requireIdleBusFailureRecovery(t, true, "--sim-wkc-incomplete-from-call", "5000", "--sim-wkc-incomplete-for-calls", "600")
}

func requireIdleBusFailureRecovery(t *testing.T, incompleteWKC bool, args ...string) {
	t.Helper()
	// Avoid unrelated startup retries reading the simulator's not-yet-written
	// home object. The actual Home transaction still writes and reads it.
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	f := startCoreWithArgs(t, "simulation-home.json", args...)
	c := f.client
	homeSequence, err := c.Home(context.Background(), 1)
	mustSubmit(t, homeSequence, err)
	if _, err := c.WaitResult(context.Background(), homeSequence); err != nil {
		t.Fatal(err)
	}
	healthy := waitIdleBusReviewObservation(t, f, func(s idleBusReviewObservation) bool {
		return s.HomeValidMask == 1 && s.ConfigVerifiedMask == 1 && s.CommissioningPhase == 0 &&
			s.Armed == 0 && s.AxisEnableMask == 0 && s.StartupReady == 1 && s.WCState == 2
	})
	if healthy.ExecutionFaultReasons != 0 || healthy.SafetyFaultMask != 0 || healthy.WKCActual != healthy.WKCExpected ||
		len(healthy.Axes) != 1 || healthy.Axes[0].Statusword&0x4f != 0x40 {
		t.Fatalf("failure must follow a healthy, homed, disarmed state: %+v", healthy)
	}

	wantedFault := faultBusTransport
	wantedHome := uint32(1)
	if incompleteWKC {
		wantedFault = 1 << 7 // FaultDriveReadiness in include/execution_policy.hpp.
		wantedHome = 0
	}
	faulted := waitIdleBusReviewObservation(t, f, func(s idleBusReviewObservation) bool {
		return s.ExecutionFaultReasons&wantedFault != 0 && s.StartupReady == 0 && s.ConfigVerifiedMask == 0
	})
	if faulted.Armed != 0 || faulted.AxisEnableMask != 0 || faulted.HomeValidMask != wantedHome ||
		faulted.SafetyFaultMask != 1 || faulted.ControlGeneration <= healthy.ControlGeneration {
		t.Fatalf("idle failure did not revoke and invalidate the affected coordinates: %+v", faulted)
	}
	if incompleteWKC {
		if faulted.WCState == 2 || faulted.WKCActual >= faulted.WKCExpected ||
			faulted.ExecutionFaultReasons&faultBusTransport != 0 || faulted.LastBusFailureOperation != 0 || faulted.LastBusFailureCode != 0 {
			t.Fatalf("incomplete WKC must remain distinguishable from a failed bus API call: %+v", faulted)
		}
	} else if faulted.LastBusFailureOperation != 5 || faulted.LastBusFailureCode != -5 {
		t.Fatalf("idle send failure lost its operation/errno: %+v", faulted)
	}
	for deadline := time.Now().Add(time.Second); ; {
		motion, err := c.Motion()
		if err == nil && motion.TimeNS >= faulted.TimeNS && motion.State == 6 && motion.Done {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("idle fault must be client-visible as FAULTED: %+v %v", motion, err)
		}
		time.Sleep(10 * time.Millisecond)
	}

	// The finite injection ends and the existing metrics worker re-verifies
	// configured startup objects. Neither event silently restores home or clears
	// the execution latch, even though communication is healthy again.
	recovered := waitIdleBusReviewObservation(t, f, func(s idleBusReviewObservation) bool {
		return s.TimeNS > faulted.TimeNS && s.StartupReady == 1 && s.WCState == 2 &&
			s.WKCActual == s.WKCExpected && s.ConfigVerifiedMask == 1
	})
	if recovered.HomeValidMask != wantedHome || recovered.Armed != 0 || recovered.AxisEnableMask != 0 ||
		recovered.SafetyFaultMask != 1 || recovered.ExecutionFaultReasons&wantedFault == 0 {
		t.Fatalf("bus recovery silently cleared the idle fault/home gate: %+v", recovered)
	}
	requireIdleBusNotReady(t, c, c.Enable, 1)
	armSequence, err := c.Arm(context.Background())
	mustSubmit(t, armSequence, err)
	requireIdleBusNotReadyResult(t, c, armSequence)
	stopContext, stopCancel := context.WithTimeout(context.Background(), time.Second)
	_, err = c.Stop(stopContext)
	stopCancel()
	if err != nil {
		t.Fatalf("restored bus could not acknowledge an inhibited Stop frame: %v", err)
	}
	stoppedAt := monotonicNS()
	stopped := waitIdleBusReviewObservation(t, f, func(s idleBusReviewObservation) bool { return s.TimeNS >= stoppedAt })
	if stopped.HomeValidMask != wantedHome || stopped.SafetyFaultMask != 1 || stopped.ExecutionFaultReasons&wantedFault == 0 {
		t.Fatalf("Stop acknowledgement was incorrectly treated as fault/home recovery: %+v", stopped)
	}

	// Wait for the reset's correlated result and subsequently published effect.
	resetSequence, err := c.FaultReset(context.Background(), 1)
	mustSubmit(t, resetSequence, err)
	if _, err := c.WaitResult(context.Background(), resetSequence); err != nil {
		t.Fatal(err)
	}
	resetAt := monotonicNS()
	reset := waitIdleBusReviewObservation(t, f, func(s idleBusReviewObservation) bool {
		return s.TimeNS >= resetAt && s.ExecutionFaultReasons == 0 && s.SafetyFaultMask == 0 && s.ConfigVerifiedMask == 1
	})
	if reset.HomeValidMask != wantedHome || reset.Armed != 0 || reset.AxisEnableMask != 0 {
		t.Fatalf("ResetFault restored authority/home without a new Home: %+v", reset)
	}
	if incompleteWKC {
		requireIdleBusNotReady(t, c, c.Enable, 1)
		homeSequence, err = c.Home(context.Background(), 1)
		mustSubmit(t, homeSequence, err)
		if _, err := c.WaitResult(context.Background(), homeSequence); err != nil {
			t.Fatal(err)
		}
	}
	wait(t, c, func(s Status) bool { return s.HomeValidMask == 1 && s.CommissioningPhase == 0 && s.Armed == 0 })
	arm(t, c, 1)
	wait(t, c, allOperationEnabled)
	sequence, err := c.Trajectory(context.Background(), 9801, 1, []Point{
		{Time: 0, Position: []float64{0}},
		{Time: 100 * time.Millisecond, Position: []float64{0}},
	})
	mustSubmit(t, sequence, err)
	for deadline := time.Now().Add(time.Second); ; {
		motion, err := c.Motion()
		if err == nil && motion.TrajectoryID == 9801 && motion.CommandSequence == sequence && motion.State == 4 && motion.Done {
			break
		}
		if time.Now().After(deadline) {
			t.Fatalf("fresh plan did not execute after explicit reset/Home/Enable/Arm: %+v %v", motion, err)
		}
		time.Sleep(10 * time.Millisecond)
	}
	final, err := c.Status()
	if err != nil || final.SafetyFaultMask != 0 || final.ExecutionFaultReasons != 0 || final.HomeValidMask != 1 {
		t.Fatalf("explicit idle-bus recovery remained faulted: %+v %v", final, err)
	}
}

func requireIdleBusNotReady(t *testing.T, c *Client, command func(context.Context, uint32) (uint64, error), mask uint32) {
	t.Helper()
	sequence, err := command(context.Background(), mask)
	mustSubmit(t, sequence, err)
	requireIdleBusNotReadyResult(t, c, sequence)
}

func requireIdleBusNotReadyResult(t *testing.T, c *Client, sequence uint64) {
	t.Helper()
	result, err := c.WaitResult(context.Background(), sequence)
	var rejected *RejectedError
	if !errors.As(err, &rejected) || result.Reason != ReasonNotReady {
		t.Fatalf("admission bypassed the remaining fault/home gate: %+v %v", result, err)
	}
}

func TestStartupIncompleteWKCDoesNotLatchIdleExecutionFault(t *testing.T) {
	// The ordinary fixture waits for verified configuration before returning,
	// which would conceal startup. Use its compiler and launch the same daemon
	// while reading the public metrics from the first incomplete cycle onward.
	compiled := compiledForValidation(t, "simulation.json")
	directory, err := os.MkdirTemp("", "iwkc-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(directory) })

	args := append(append([]string(nil), compiled.args...), "--socket-path", filepath.Join(directory, "ipc.sock"),
		"--sim-wkc-incomplete-from-call", "1", "--sim-wkc-incomplete-for-calls", "1000")
	process := exec.Command(compiled.binary, args...)
	simfixture.SpawnLogged(t, process, filepath.Join(directory, "core.log"), 3*time.Second, 0, nil)

	sawIncomplete := false
	deadline := time.Now().Add(5 * time.Second)
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		observation, readErr := readIdleBusReviewObservation(filepath.Join(directory, "metrics.json"))
		if readErr == nil {
			if observation.ExecutionFaultReasons != 0 || observation.SafetyFaultMask != 0 ||
				observation.Armed != 0 || observation.AxisEnableMask != 0 || observation.HomeValidMask != 0 {
				t.Fatalf("initial incomplete WKC was treated as loss of an established healthy state: %+v", observation)
			}
			if observation.CycleCount > 0 && observation.WKCExpected != 0 && observation.WCState != 2 &&
				observation.WKCActual < observation.WKCExpected {
				sawIncomplete = true
				if observation.StartupReady != 0 || observation.ConfigVerifiedMask != 0 {
					t.Fatalf("incomplete startup process data was admitted as ready: %+v", observation)
				}
			}
			if observation.StartupReady == 1 && observation.ConfigVerifiedMask == 3 &&
				observation.WCState == 2 && observation.WKCActual == observation.WKCExpected {
				if !sawIncomplete {
					t.Fatal("test never observed the injected incomplete startup WKC")
				}
				return true // Startup recovers without ResetFault or any client authority.
			}
		} else if !errors.Is(readErr, os.ErrNotExist) {
			t.Fatal(readErr)
		}
		if time.Now().After(deadline) {
			t.Fatalf("startup did not recover automatically: %+v %v", observation, readErr)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
}
