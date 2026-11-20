//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"testing"
	"time"
)

// These tests launch the ordinary daemon with explicit simulator-only plant
// controls. The same tests also run against 895e249 plus only those controls;
// the fixture never changes the engine's readiness, clock or output decisions.
const faultBusTransport = uint32(1 << 8)

func allOperationEnabled(s Status) bool {
	if len(s.Axes) == 0 {
		return false
	}
	return AllOperationEnabled(s)
}

func busFailureObservation(t *testing.T, f fixture, operation uint32) {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
	if err != nil {
		t.Fatal(err)
	}
	var observed struct {
		Operation uint32 `json:"last_bus_failure_operation"`
		Code      int64  `json:"last_bus_failure_code"`
	}
	if err := json.Unmarshal(raw, &observed); err != nil {
		t.Fatal(err)
	}
	if observed.Operation != operation || observed.Code != -5 {
		t.Errorf("bus failure must retain failed operation %d and errno -5: %+v", operation, observed)
	}
}

func TestTrajectoryStartWaitsForOperationEnabled(t *testing.T) {
	// Unverified simulator injection: 450 enable cycles outlast the 350 ms
	// observation below yet stay inside the 500 ms drive-state enable bound.
	f := startCoreWithArgs(t, "simulation.json", "--sim-enable-delay-cycles", "450")
	c := f.client
	// Preparing before Arm leaves the whole injected enable delay available for
	// the test. Returning to the initial position reproduces the old false
	// COMPLETED result even though the drive never moved during the plan.
	handle, err := c.PrepareTrajectory(context.Background(), 9101, 3, []Point{
		{Time: 0, Position: []float64{0, 0}},
		{Time: 100 * time.Millisecond, Position: []float64{0.002, 0.002}},
		{Time: 200 * time.Millisecond, Position: []float64{0, 0}},
	})
	if err != nil {
		t.Fatal(err)
	}
	arm(t, c, 3)
	before, err := c.Status()
	if err != nil || allOperationEnabled(before) {
		t.Fatalf("enable-delay precondition was not exercised: %+v %v", before, err)
	}
	_, err = c.StartTrajectory(context.Background(), handle)
	var rejected *RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonNotReady {
		t.Errorf("Start before OperationEnabled must reject with ReasonNotReady: %v", err)
	}
	// Observe beyond the whole rejected plan, while the drive still cannot
	// enable. An admission rejection cannot secretly start its realtime clock.
	for deadline := time.Now().Add(350 * time.Millisecond); time.Now().Before(deadline); {
		motion, motionErr := c.Motion()
		if motionErr == nil && motion.TrajectoryID == handle && motion.State == 4 && motion.Done {
			t.Errorf("plan completed without OperationEnabled: %+v", motion)
			break
		}
		time.Sleep(10 * time.Millisecond)
	}
	ready := wait(t, c, allOperationEnabled)
	if ready.SafetyFaultMask != 0 || ready.Axes[0].PositionCounts != 0 || ready.Axes[1].PositionCounts != 0 {
		t.Fatalf("rejected Start must leave the held position unchanged: %+v", ready)
	}
	sequence, err := c.Trajectory(context.Background(), 9102, 3, []Point{
		{Time: 0, Position: []float64{0, 0}},
		{Time: 200 * time.Millisecond, Position: []float64{0.003, 0.003}},
	})
	mustSubmit(t, sequence, err)
	deadline := time.Now().Add(2 * time.Second)
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		motion, motionErr := c.Motion()
		if motionErr == nil && motion.TrajectoryID == 9102 && motion.State == 4 && motion.Done {
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("fresh plan did not execute after the enable barrier: %+v %v", motion, motionErr)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	moved := wait(t, c, func(s Status) bool {
		return s.Axes[0].PositionCounts >= 62 && s.Axes[1].PositionCounts <= -62
	})
	if moved.SafetyFaultMask != 0 {
		t.Fatalf("fresh plan faulted after readiness: %+v", moved)
	}
}

func TestFailedBusSendCannotAcknowledgeStop(t *testing.T) {
	requireFailedSubmissionCannotAcknowledgeStop(t, "send", "--sim-send-fail-from-call", 5)
}

func TestFailedDomainQueueCannotAcknowledgeStop(t *testing.T) {
	requireFailedSubmissionCannotAcknowledgeStop(t, "domain queue", "--sim-queue-fail-from-call", 7)
}

func TestFailedSplitDomainQueueCannotAcknowledgeStop(t *testing.T) {
	// Call 2000 fails on the second domain of a cycle, after the first domain
	// was queued. A successful first queue cannot stand for the whole group.
	requireFailedSubmissionCannotAcknowledgeStop(t, "split domain queue", "--sim-queue-fail-from-call", 7,
		"--split-domains-per-axis")
}

func TestStopSubmitsAllConfiguredDomains(t *testing.T) {
	f := startCoreWithArgs(t, "simulation.json", "--split-domains-per-axis")
	arm(t, f.client, 3)
	wait(t, f.client, allOperationEnabled)
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	if _, err := f.client.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	state := wait(t, f.client, func(s Status) bool {
		return s.Armed == 0 && s.Axes[0].Statusword&0x4f == 0x40 && s.Axes[1].Statusword&0x4f == 0x40
	})
	if state.SafetyFaultMask != 0 {
		t.Fatalf("successful all-domain Stop introduced a fault: %+v", state)
	}
}

func requireFailedSubmissionCannotAcknowledgeStop(t *testing.T, operation string, flag string, operationCode uint32, extraArgs ...string) {
	t.Helper()
	f := startCoreWithArgs(t, "simulation.json", append([]string{flag, "2000"}, extraArgs...)...)
	c := f.client
	arm(t, c, 3)
	ready := wait(t, c, allOperationEnabled)
	if ready.ExecutionFaultReasons != 0 {
		t.Fatalf("%s failure test must reach a healthy enabled state first: %+v", operation, ready)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	faulted, err := c.Wait(ctx, func(s Status) bool { return s.ExecutionFaultReasons&faultBusTransport != 0 })
	cancel()
	if err != nil {
		t.Errorf("failed %s was not latched as a typed bus transport fault: %+v %v", operation, faulted, err)
	} else if faulted.Armed != 0 || faulted.AxisEnableMask != 0 || faulted.SafetyFaultMask != 3 {
		t.Errorf("failed %s did not inhibit the whole controlled group: %+v", operation, faulted)
	}
	// BusOperation::Send is 5 and Queue is 7. A send without the inhibited
	// domain queued can succeed; it still cannot acknowledge that Stop frame.
	busFailureObservation(t, f, operationCode)
	stopContext, stopCancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer stopCancel()
	if _, err := c.Stop(stopContext); err == nil {
		t.Errorf("Stop acknowledged an inhibited process image while every %s failed", operation)
	}
	observationContext, observationCancel := context.WithTimeout(context.Background(), time.Second)
	defer observationCancel()
	state, err := c.Wait(observationContext, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	if err != nil || state.Armed != 0 || state.AxisEnableMask != 0 {
		t.Errorf("unacknowledged Stop must still revoke output authority: %+v %v", state, err)
	}
}

func TestReceiveFailureFaultsActiveTrajectory(t *testing.T) {
	requireBusCycleFailureFaultsActiveTrajectory(t, "receive", "--sim-receive-fail-from-call", 2)
}

func TestDomainProcessFailureFaultsActiveTrajectory(t *testing.T) {
	requireBusCycleFailureFaultsActiveTrajectory(t, "domain process", "--sim-process-fail-from-call", 6)
}

func TestDCSyncFailureFaultsActiveTrajectory(t *testing.T) {
	// DC is enabled by default for the simulation fixture, just as in the
	// normal daemon. Failed clock synchronization is a cycle failure even when
	// the domain's position feedback continues to look correct.
	requireBusCycleFailureFaultsActiveTrajectory(t, "DC slave sync", "--sim-dc-fail-from-call", 4)
}

func requireBusCycleFailureFaultsActiveTrajectory(t *testing.T, operation string, flag string, operationCode uint32) {
	t.Helper()
	f := startCoreWithArgs(t, "simulation.json", flag, "2000")
	c := f.client
	arm(t, c, 3)
	ready := wait(t, c, allOperationEnabled)
	if ready.ExecutionFaultReasons != 0 {
		t.Fatalf("%s failure test must reach a healthy enabled state first: %+v", operation, ready)
	}
	// A stationary dwell exposes the unvalidated-feedback defect: the old
	// engine can falsely complete against a matching position without a
	// successful receive/process/clock cycle establishing execution validity.
	sequence, err := c.Trajectory(context.Background(), 9301, 3, []Point{
		{Time: 0, Position: []float64{0, 0}},
		{Time: 3 * time.Second, Position: []float64{0, 0}},
	})
	mustSubmit(t, sequence, err)
	deadline := time.Now().Add(4 * time.Second)
	var faulted Status
	sawExecuting, sawBusFault, sawFaultedMotion, reportedCompletion := false, false, false, false
	for time.Now().Before(deadline) {
		status, statusErr := c.Status()
		if statusErr != nil {
			t.Fatal(statusErr)
		}
		if status.ExecutionFaultReasons&faultBusTransport != 0 {
			sawBusFault, faulted = true, status
		}
		motion, motionErr := c.Motion()
		if motionErr == nil && motion.TrajectoryID == 9301 {
			sawExecuting = sawExecuting || motion.State == 3
			sawFaultedMotion = sawFaultedMotion || (motion.State == 6 && motion.Done)
			if motion.State == 4 && motion.Done && !reportedCompletion {
				t.Errorf("trajectory completed using unvalidated feedback after %s failure: %+v", operation, motion)
				reportedCompletion = true
			}
		}
		time.Sleep(10 * time.Millisecond)
	}
	if !sawExecuting {
		t.Errorf("%s failure test never observed an active trajectory", operation)
	}
	if !sawBusFault || !sawFaultedMotion {
		t.Errorf("%s failure must fault active execution: typed_bus_fault=%t faulted_motion=%t", operation, sawBusFault, sawFaultedMotion)
	}
	if sawBusFault && (faulted.Armed != 0 || faulted.AxisEnableMask != 0 || faulted.SafetyFaultMask != 3) {
		t.Errorf("%s failure did not inhibit every controlled axis: %+v", operation, faulted)
	}
	// Input failure must remain observable even though the inhibited output
	// frame can still be queued and submitted successfully afterward.
	busFailureObservation(t, f, operationCode)
}
