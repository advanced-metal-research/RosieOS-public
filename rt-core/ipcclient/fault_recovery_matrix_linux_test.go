//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

// Require the receipt and matching native recovery publication; a successful
// write or an older status cannot establish that this reset cleared anything.
func recoveryReset(t *testing.T, c *Client, mask uint32, persists bool) Status {
	t.Helper()
	// Unverified test interaction budget: 3 seconds.
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	sequence, err := c.FaultReset(ctx, mask)
	if err != nil {
		t.Fatal(err)
	}
	result, err := c.WaitResult(ctx, sequence)
	if persists {
		var rejected *RejectedError
		if !errors.As(err, &rejected) || result.Result != ResultRejected {
			t.Fatalf("reset accepted persistent fault: %+v %v", result, err)
		}
	} else if err != nil {
		t.Fatal(err)
	}
	status := wait(t, c, func(s Status) bool { return s.Recovery != nil && s.Recovery.ResetSequence == sequence })
	if persists && status.Recovery.Reason != "fault_persists" {
		t.Fatalf("missing native fault_persists: %+v", status.Recovery)
	}
	if !persists && status.Recovery.Reason != "" {
		t.Fatalf("unexpected recovery rejection: %+v", status.Recovery)
	}
	return status
}

// Suspend the simulated daemon as in the frozen deadline oracle. The timer is
// fault injection only; acceptance requires an independently observed bit 2.
func recoveryDeadlineFault(t *testing.T) fixture {
	t.Helper()
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	f := startCoreWithArgs(t, "simulation-home.json")
	seq, err := f.client.Home(context.Background(), 1)
	mustSubmit(t, seq, err)
	if _, err := f.client.WaitResult(context.Background(), seq); err != nil {
		t.Fatal(err)
	}
	wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.CommissioningPhase == 0 })
	arm(t, f.client, 1)
	wait(t, f.client, allOperationEnabled)
	raw, err := f.client.conn.SyscallConn()
	if err != nil {
		t.Fatal(err)
	}
	var credential *unix.Ucred
	var socketErr error
	if err := raw.Control(func(fd uintptr) {
		credential, socketErr = unix.GetsockoptUcred(int(fd), unix.SOL_SOCKET, unix.SO_PEERCRED)
	}); err != nil {
		t.Fatal(err)
	}
	if socketErr != nil {
		t.Fatal(socketErr)
	}
	pid := int(credential.Pid)
	if err := unix.Kill(pid, unix.SIGSTOP); err != nil {
		t.Fatal(err)
	}
	defer unix.Kill(pid, unix.SIGCONT)
	// Unverified injected suspension: 80 ms, exceeding the 20 ms cycle budget.
	<-time.After(80 * time.Millisecond)
	if err := unix.Kill(pid, unix.SIGCONT); err != nil {
		t.Fatal(err)
	}
	faulted := wait(t, f.client, func(s Status) bool { return s.ExecutionFaultReasons&4 != 0 && s.Armed == 0 })
	if faulted.HomeValidMask != 1 {
		t.Fatalf("deadline miss invalidated trustworthy Home: %+v", faulted)
	}
	return f
}

func TestResetAfterDeadlineMissKeepsHomeValid(t *testing.T) {
	f := recoveryDeadlineFault(t)
	reset := recoveryReset(t, f.client, 1, false)
	if reset.HomeValidMask != 1 || reset.ExecutionFaultReasons != 0 || reset.SafetyFaultMask != 0 {
		t.Fatalf("deadline reset discarded Home or retained faults: %+v", reset)
	}
	if len(reset.Recovery.Outcomes) != 1 || reset.Recovery.Outcomes[0].Bit != 2 || reset.Recovery.Outcomes[0].Outcome != "cleared" {
		t.Fatalf("wrong deadline outcome: %+v", reset.Recovery)
	}
}

// AMR-720 / 2026-10-06: a jog target leading feedback beyond max_target_lead
// is a warning, not a fault. On a homed, qualified axis it neither disarms,
// latches a fault nor touches Home.
func TestTargetLeadWarnsAndKeepsHome(t *testing.T) {
	// Same lag as TestLimitHoldTrackingGuards/target_lead: a 60-count feedback
	// offset against a 0.002 rad lead bound.
	machine, profile := limitHoldInputs(t, true, true)
	machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)["max_target_lead"] = .002
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err, digest)
	}
	f, _ := coordinateModelDaemon(t, compiled, digest, "--sim-feedback-offset-counts", "60")
	coordinateHome(t, f, 1)
	wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.CommissioningPhase == 0 })
	arm(t, f.client, 1)
	wait(t, f.client, allOperationEnabled)
	seq, err := f.client.Jog(context.Background(), 1, []float64{.02}, 200*time.Millisecond)
	if r := limitHoldResult(t, f.client, seq, err); r.Result != ResultAccepted {
		t.Fatal(r)
	}
	// Unverified observation window: twice the 200 ms jog input.
	time.Sleep(400 * time.Millisecond)
	s := wait(t, f.client, func(Status) bool { return true })
	if s.Armed != 1 || s.HomeValidMask != 1 || s.ExecutionFaultReasons != 0 || s.SafetyFaultMask != 0 {
		t.Fatalf("target lead disarmed, faulted or discarded Home: %+v", s)
	}
}

func TestResetWhileBusStillFailingIsRejectedFaultPersists(t *testing.T) {
	f := startCoreWithArgs(t, "simulation.json", "--sim-send-fail-from-call", "2500", "--sim-send-fail-for-calls", "2000")
	faulted := wait(t, f.client, func(s Status) bool { return s.ExecutionFaultReasons&256 != 0 })
	reset := recoveryReset(t, f.client, 3, true)
	if reset.ExecutionFaultReasons != faulted.ExecutionFaultReasons || reset.SafetyFaultMask != faulted.SafetyFaultMask || reset.HomeValidMask != faulted.HomeValidMask {
		t.Fatalf("persistent reset cleared latched state: before=%+v after=%+v", faulted, reset)
	}
}

func TestResetNeverRestoresArmOrMotion(t *testing.T) {
	f := recoveryDeadlineFault(t)
	reset := recoveryReset(t, f.client, 1, false)
	last := reset.TimeNS
	for samples := 0; samples < 12; samples++ {
		observed := wait(t, f.client, func(s Status) bool { return s.TimeNS > last })
		last = observed.TimeNS
		if observed.Armed != 0 || observed.AxisEnableMask != 0 || observed.Axes[0].PositionCounts-reset.Axes[0].PositionCounts != 0 {
			t.Fatalf("reset restored authority or caused nonzero encoder motion: %+v", observed)
		}
	}
}

func TestResetAfterDisabledBusTransportKeepsHomeValid(t *testing.T) {
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	f := startCoreWithArgs(t, "simulation-home.json", "--sim-send-fail-from-call", "3500", "--sim-send-fail-for-calls", "600")
	seq, err := f.client.Home(context.Background(), 1)
	mustSubmit(t, seq, err)
	if _, err := f.client.WaitResult(context.Background(), seq); err != nil {
		t.Fatal(err)
	}
	wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.CommissioningPhase == 0 })
	wait(t, f.client, func(s Status) bool { return s.ExecutionFaultReasons&256 != 0 && s.ConfigVerifiedMask == 0 })
	wait(t, f.client, func(s Status) bool { return s.ExecutionFaultReasons&256 != 0 && s.ConfigVerifiedMask == 1 })
	reset := recoveryReset(t, f.client, 1, false)
	if reset.HomeValidMask != 1 || reset.Armed != 0 || reset.AxisEnableMask != 0 {
		t.Fatalf("disabled matching bus recovery lost Home: %+v", reset)
	}
}

func TestLegacyZeroMaskResetRetainsBlanketInvalidation(t *testing.T) {
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	f := startCoreWithArgs(t, "simulation-home.json")
	sequence, err := f.client.Home(context.Background(), 1)
	mustSubmit(t, sequence, err)
	if _, err := f.client.WaitResult(context.Background(), sequence); err != nil {
		t.Fatal(err)
	}
	wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.CommissioningPhase == 0 })
	// Measured legacy payload: eight zero bytes select all axes and no flags.
	sequence, err = f.client.command(context.Background(), MsgFaultReset, []byte{0, 0, 0, 0, 0, 0, 0, 0})
	mustSubmit(t, sequence, err)
	if _, err := f.client.WaitResult(context.Background(), sequence); err != nil {
		t.Fatal(err)
	}
	after := HostMonotonicNS()
	reset := wait(t, f.client, func(s Status) bool { return s.TimeNS >= after && s.HomeValidMask == 0 })
	if reset.Armed != 0 || reset.AxisEnableMask != 0 || reset.ExecutionFaultReasons != 0 || reset.SafetyFaultMask != 0 {
		t.Fatalf("legacy reset changed its inhibition/clear contract: %+v", reset)
	}
}
