//go:build linux

package faultmatrix

import (
	"context"
	"errors"
	ctl "rosieos/rt-core/adapters/rosie/control"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

func grantFixture(t *testing.T) (*fixture, ipc.GrantIdentity) {
	f := compileFixture(t, "simulation.json", nil)
	f.launch(false)
	id := ipc.GrantIdentity{CapabilityID: [16]byte{1}, Generation: 1}
	_, err := f.c.MirrorAcquire(context.Background(), id, ipc.HostMonotonicNS()+500_000_000)
	must(t, err)
	return f, id
}
func (f *fixture) grantUntil(after, bound uint64, predicate func(ipc.GrantObservation) bool) ipc.GrantObservation {
	f.t.Helper()
	deadline := time.Now().Add(watchdog)
	for {
		s, err := f.c.FastObservation()
		if err == nil && s.NowHostNs >= after {
			if s.NowHostNs-after > bound {
				f.t.Fatal("grant observation exceeded ns bound")
			}
			if predicate(s) {
				return s
			}
		}
		if time.Now().After(deadline) {
			f.t.Fatal("grant observation watchdog")
		}
		pause()
	}
}
func staleGrant(t *testing.T, r row) {
	f, id := grantFixture(t)
	before := f.grantUntil(0, ^uint64(0), func(s ipc.GrantObservation) bool { return s.GrantActive == 1 })
	f.grantUntil(before.DeadlineHostNs, r.boundNS, func(s ipc.GrantObservation) bool { return s.GrantActive == 0 })
	_, err := f.c.MirrorAcquire(context.Background(), id, ipc.HostMonotonicNS()+500_000_000)
	var rejected *ipc.GrantRejectedError
	if !errors.As(err, &rejected) || rejected.Reason != ipc.GrantReasonStaleGeneration {
		t.Fatalf("stale grant: %v", err)
	}
	s := f.until(ipc.HostMonotonicNS(), r.boundNS, func(s ipc.Status) bool { return s.Armed == 0 })
	f.hold(s, 200_000_000, 0)
}
func staleNative(t *testing.T, r row) {
	f, id := grantFixture(t)
	before := f.grantUntil(0, ^uint64(0), func(s ipc.GrantObservation) bool { return s.GrantActive == 1 })
	after := f.grantUntil(before.DeadlineHostNs, r.boundNS, func(s ipc.GrantObservation) bool { return s.GrantActive == 0 })
	if after.NativeGeneration != before.NativeGeneration+1 {
		t.Fatalf("expiry generation=%d want=%d", after.NativeGeneration, before.NativeGeneration+1)
	}
	_, err := f.c.MirrorRenew(context.Background(), id, ipc.HostMonotonicNS()+500_000_000)
	var rejected *ipc.GrantRejectedError
	if !errors.As(err, &rejected) || rejected.Reason != ipc.GrantReasonNativeGenerationMismatch {
		t.Fatalf("old native renewal: %v", err)
	}
	s := f.until(ipc.HostMonotonicNS(), r.boundNS, func(s ipc.Status) bool { return s.Armed == 0 })
	f.hold(s, 200_000_000, 0)
}
func staleJog(t *testing.T, r row) {
	f, id := grantFixture(t)
	f.arm()
	origin := ipc.HostMonotonicNS()
	first, err := f.c.BeginLocalJog(context.Background(), id, 3, origin, origin+200_000_000)
	must(t, err)
	must(t, f.c.EndLocalJog(context.Background(), first))
	_, err = f.c.Stop(context.Background())
	must(t, err)
	id.Generation++
	_, err = f.c.MirrorAcquire(context.Background(), id, ipc.HostMonotonicNS()+500_000_000)
	must(t, err)
	f.arm()
	origin = ipc.HostMonotonicNS()
	second, err := f.c.BeginLocalJog(context.Background(), id, 3, origin, origin+200_000_000)
	must(t, err)
	if second.Identity.JogGeneration <= first.Identity.JogGeneration {
		t.Fatal("jog generation did not advance")
	}
	stale := second
	stale.Identity.JogGeneration = first.Identity.JogGeneration
	origin = ipc.HostMonotonicNS()
	must(t, f.c.UpdateLocalJog(context.Background(), stale, 1, origin, origin+100_000_000, 3, []float64{.01, .01}))
	deadline := time.Now().Add(watchdog)
	for {
		s, err := f.c.JogObservation()
		if err == nil && s.ObservedSourceSequence == 1 {
			if s.UpdateReason != ipc.JogReasonInvalidGeneration || s.HasInput != 0 || s.NowHostNs-origin > r.boundNS {
				t.Fatalf("stale jog accepted/misdiagnosed: %+v", s)
			}
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("jog observation watchdog")
		}
		pause()
	}
	s := f.until(origin, r.boundNS, func(s ipc.Status) bool { return true })
	if s.Axes[0].PositionCounts != 0 || s.Axes[1].PositionCounts != 0 {
		t.Fatal("stale jog moved")
	}
}
func wrongSession(t *testing.T, r row) {
	f, a, fence := adapterFixture(t)
	armAdapter(t, f, a, fence)
	h, err := a.Prepare(fence, 3, []ipc.Point{{Position: []float64{0, 0}}, {Time: time.Second, Position: []float64{.01, .01}}})
	must(t, err)
	_, err = a.Start(ctl.Fence{Session: "wrong", Generation: fence.Generation}, h)
	if err == nil || err.Error() != "control_session_stale" {
		t.Fatalf("wrong session: %v", err)
	}
	f.hold(f.status(), 200_000_000, 0)
}
