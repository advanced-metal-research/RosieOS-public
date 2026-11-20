//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"
)

// Exercise the production adapter role, not the simulation-only oracle role.
// The simulator derives DS402 status and position from its last successfully
// queued/sent process image. These are submitted-output effects in the simulated
// plant, not a claim about physical standstill or a prepared target field.
func TestApplicationGrantExpiryInhibitsMovingSubmittedTrajectory(t *testing.T) {
	c := applicationGrantNativeClient(t)
	ctx, cancel := context.WithTimeout(context.Background(), 12*time.Second)
	defer cancel()
	identity := GrantIdentity{CapabilityID: [16]byte{0x61, 0x75, 0x74, 0x68}, Generation: 1}
	receipt, err := c.MirrorAcquire(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil {
		t.Fatal(err)
	}
	renew := func() GrantReceipt {
		t.Helper()
		current, err := c.MirrorRenew(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease)
		if err != nil {
			t.Fatal(err)
		}
		return current
	}
	waitMotion := func(predicate func(MotionState) bool) MotionState {
		t.Helper()
		deadline := time.Now().Add(time.Second)
		for {
			motion, err := c.Motion()
			if err == nil && predicate(motion) {
				return motion
			}
			if time.Now().After(deadline) {
				t.Fatalf("motion observation did not converge: %+v, %v", motion, err)
			}
			time.Sleep(time.Millisecond)
		}
	}
	const oldHandle = 9801
	handle, err := c.PrepareTrajectory(ctx, oldHandle, 3, []Point{
		{Position: []float64{0, 0}},
		{Time: 3 * time.Second, Position: []float64{0.12, 0.12}},
	})
	if err != nil {
		t.Fatal(err)
	}
	arm(t, c, 3)
	renew()
	ready := wait(t, c, allOperationEnabled)
	if ready.SafetyFaultMask != 0 || ready.ExecutionFaultReasons != 0 {
		t.Fatalf("moving-expiry precondition faulted: %+v", ready)
	}
	renew()
	startNS := HostMonotonicNS()
	sequence, err := c.StartTrajectory(ctx, handle)
	if err != nil {
		t.Fatal(err)
	}
	waitMotion(func(m MotionState) bool {
		return m.TimeNS >= startNS && m.TrajectoryID == handle && m.CommandSequence == sequence && m.State == 3 && !m.Done
	})
	first := wait(t, c, func(s Status) bool {
		return s.TimeNS >= startNS && allOperationEnabled(s) && s.Axes[0].PositionCounts >= 20 && s.Axes[1].PositionCounts <= -20
	})
	renew()
	second := wait(t, c, func(s Status) bool {
		return s.TimeNS > first.TimeNS && allOperationEnabled(s) &&
			s.Axes[0].PositionCounts > first.Axes[0].PositionCounts+20 && s.Axes[1].PositionCounts < first.Axes[1].PositionCounts-20
	})
	// Both axes moved over two fresh observations, rather than merely accepting
	// Start. Keep the ordinary native heartbeat service alive throughout; only
	// shorten the mirrored application authority's absolute expiry.
	expiryNS := HostMonotonicNS() + uint64(100*time.Millisecond)
	short, err := c.MirrorRenew(ctx, identity, expiryNS)
	if err != nil || short.DeadlineNS != expiryNS || short.NativeGeneration != receipt.NativeGeneration {
		t.Fatalf("could not expire an otherwise healthy moving grant: %+v, %v", short, err)
	}
	expired := waitGrantObservation(t, c, func(o GrantObservation) bool {
		return o.GrantActive == 0 && o.NativeGeneration > receipt.NativeGeneration
	})
	if expired.NowHostNs < expiryNS || expired.NowHostNs >= startNS+uint64(3*time.Second) {
		t.Fatalf("grant did not expire during the running plan: %+v, deadline=%d start=%d", expired, expiryNS, startNS)
	}
	aborted := waitMotion(func(m MotionState) bool {
		return m.TimeNS >= expired.NowHostNs && m.TrajectoryID == handle && m.CommandSequence == sequence && m.State == 5 && m.Done
	})
	inhibited := func(s Status) bool {
		if s.Armed != 0 || s.AxisEnableMask != 0 {
			return false
		}
		for _, axis := range s.Axes {
			if axis.Statusword&0x4f != 0x40 {
				return false
			}
		}
		return true
	}
	held := wait(t, c, func(s Status) bool { return s.TimeNS >= expired.NowHostNs && inhibited(s) })
	if held.SafetyFaultMask != 0 || held.ExecutionFaultReasons != 0 || held.ConfigVerifiedMask != 3 || held.LeaseValid != 0 {
		t.Fatalf("application expiry must inhibit without inventing a drive fault: %+v", held)
	}
	if held.Axes[0].PositionCounts < second.Axes[0].PositionCounts || held.Axes[1].PositionCounts > second.Axes[1].PositionCounts {
		t.Fatalf("inhibition replaced measured position with the old plan origin: moving=%+v held=%+v", second, held)
	}
	// Observe beyond the 250 ms transport lease with the same client heartbeat
	// service running. Fresh native observations must retain the abort and the
	// simulated disabled drives must not follow any subsequent old plan target.
	for until := HostMonotonicNS() + uint64(350*time.Millisecond); ; {
		state, err := c.Status()
		if err != nil {
			t.Fatal(err)
		}
		if state.TimeNS > held.TimeNS {
			if !inhibited(state) || state.Axes[0].PositionCounts != held.Axes[0].PositionCounts || state.Axes[1].PositionCounts != held.Axes[1].PositionCounts {
				t.Fatalf("expired trajectory resumed submitted motion: held=%+v observed=%+v", held, state)
			}
		}
		motion, err := c.Motion()
		if err != nil || motion.TimeNS < aborted.TimeNS || motion.TrajectoryID != handle || motion.CommandSequence != sequence || motion.State != 5 || !motion.Done {
			t.Fatalf("expired execution was replayed or completed: %+v, %v", motion, err)
		}
		if state.TimeNS >= until {
			break
		}
		if HostMonotonicNS() > until+uint64(time.Second) {
			t.Fatal("no fresh inhibited feedback beyond the native transport lease")
		}
		time.Sleep(10 * time.Millisecond)
	}
	_, err = c.MirrorRenew(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease)
	var grantRejected *GrantRejectedError
	if !errors.As(err, &grantRejected) || grantRejected.Reason != GrantReasonNativeGenerationMismatch {
		t.Fatalf("old grant renewed after expiry: %v", err)
	}
	_, err = c.MirrorAcquire(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease)
	if !errors.As(err, &grantRejected) || grantRejected.Reason != GrantReasonStaleGeneration {
		t.Fatalf("old grant generation reopened after expiry: %v", err)
	}
	identity = GrantIdentity{CapabilityID: [16]byte{0x6e, 0x65, 0x77}, Generation: 2}
	fresh, err := c.MirrorAcquire(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease)
	if err != nil || fresh.NativeGeneration != expired.NativeGeneration || c.Err() != nil {
		t.Fatalf("same live heartbeat connection could not acquire fresh authority: %+v, %v, connection=%v", fresh, err, c.Err())
	}
	acquired := wait(t, c, func(s Status) bool { return s.TimeNS > held.TimeNS && s.LeaseValid == 1 })
	if !inhibited(acquired) || acquired.Axes[0].PositionCounts != held.Axes[0].PositionCounts || acquired.Axes[1].PositionCounts != held.Axes[1].PositionCounts {
		t.Fatalf("acquiring authority implicitly enabled or resumed motion: %+v", acquired)
	}
	renew()
	arm(t, c, 3)
	renew()
	rearmed := wait(t, c, func(s Status) bool { return s.TimeNS > acquired.TimeNS && allOperationEnabled(s) })
	if rearmed.Axes[0].PositionCounts != held.Axes[0].PositionCounts || rearmed.Axes[1].PositionCounts != held.Axes[1].PositionCounts || rearmed.SafetyFaultMask != 0 {
		t.Fatalf("fresh Enable/Arm replayed the old plan instead of holding: %+v", rearmed)
	}
	_, err = c.StartTrajectory(ctx, handle)
	var planRejected *RejectedError
	if !errors.As(err, &planRejected) || planRejected.Result.Reason != ReasonUnknownHandle {
		t.Fatalf("expired execution handle remained startable under fresh authority: %v", err)
	}
	// Replan from the stable measured position of this ideal, zero-offset plant.
	// simulation.json explicitly has 131072 counts/revolution and signs +1/-1.
	const radiansPerCount = 2 * math.Pi / 131072
	q0 := float64(held.Axes[0].PositionCounts) * radiansPerCount
	q1 := -float64(held.Axes[1].PositionCounts) * radiansPerCount
	newHandle, err := c.PrepareTrajectory(ctx, 9802, 3, []Point{
		{Position: []float64{q0, q1}},
		{Time: 200 * time.Millisecond, Position: []float64{q0 + 100*radiansPerCount, q1 + 100*radiansPerCount}},
	})
	if err != nil {
		t.Fatal(err)
	}
	renew()
	newSequence, err := c.StartTrajectory(ctx, newHandle)
	if err != nil {
		t.Fatal(err)
	}
	waitMotion(func(m MotionState) bool {
		return m.TrajectoryID == newHandle && m.CommandSequence == newSequence && m.State == 4 && m.Done
	})
	renew()
	recovered := wait(t, c, func(s Status) bool {
		return s.TimeNS > rearmed.TimeNS && s.Axes[0].PositionCounts >= held.Axes[0].PositionCounts+99 &&
			s.Axes[1].PositionCounts <= held.Axes[1].PositionCounts-99
	})
	if recovered.Armed != 1 || !allOperationEnabled(recovered) || recovered.SafetyFaultMask != 0 || recovered.ExecutionFaultReasons != 0 {
		t.Fatalf("fresh authority and a new plan did not recover healthy motion: %+v", recovered)
	}
	t.Logf("submitted-image simulator feedback moved %d/%d -> %d/%d counts; application expiry inhibited at %d/%d, retired handle %d, and new handle %d reached %d/%d with the heartbeat connection intact",
		first.Axes[0].PositionCounts, first.Axes[1].PositionCounts, second.Axes[0].PositionCounts, second.Axes[1].PositionCounts,
		held.Axes[0].PositionCounts, held.Axes[1].PositionCounts, handle, newHandle, recovered.Axes[0].PositionCounts, recovered.Axes[1].PositionCounts)
}
