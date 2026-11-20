//go:build linux

package faultmatrix

import (
	"context"
	ctl "rosieos/rt-core/adapters/rosie/control"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

func restart(t *testing.T, r row) {
	f, a, fence := adapterFixture(t)
	armAdapter(t, f, a, fence)
	clock, err := f.c.LocalJogClock()
	must(t, err)
	h, err := a.Prepare(fence, 3, []ipc.Point{{Position: []float64{0, 0}}, {Time: 4 * time.Second, Position: []float64{.02, .02}}})
	must(t, err)
	_, err = a.Start(fence, h)
	must(t, err)
	inert, err := a.Prepare(fence, 3, []ipc.Point{{Position: []float64{0, 0}}, {Time: time.Second, Position: []float64{0, 0}}})
	must(t, err)
	f.shutdown()
	_, _, view, _ := a.Snapshot()
	retired := 0
	for _, record := range view.Handles {
		if (record.Handle == h || record.Handle == inert) && record.State == ctl.HandleRetired {
			retired++
		}
	}
	if retired != 2 {
		t.Fatalf("adapter did not retire both handles: %+v", view)
	}
	_, err = a.Start(fence, inert)
	if err == nil {
		t.Fatal("adapter replayed stale handle")
	}
	t.Logf("adapter stale Start refusal=%v; retired handles=%d", err, retired)
	f.launch(false)
	next, err := f.c.LocalJogClock()
	must(t, err)
	if clock.Incarnation == next.Incarnation {
		t.Fatal("daemon incarnation restored")
	}
	g := f.grantUntil(0, ^uint64(0), func(g ipc.GrantObservation) bool { return g.GrantActive == 0 })
	if g.AppGrantGeneration != 0 {
		t.Fatalf("restart restored grant: %+v", g)
	}
	s := f.until(ipc.HostMonotonicNS(), r.boundNS, func(s ipc.Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	if s.HomeValidMask != 0 || s.SafetyFaultMask != 0 || s.CommissioningPhase != 0 {
		t.Fatalf("restart restored process state: %+v", s)
	}
	for _, handle := range []uint64{h, inert} {
		_, err = f.c.StartTrajectory(context.Background(), handle)
		reject(t, err, ipc.ReasonUnknownHandle)
	}
	f.hold(s, 400_000_000, 0)
}
