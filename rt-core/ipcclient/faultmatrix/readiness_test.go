//go:build linux

package faultmatrix

import (
	"context"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

func delayedEnable(t *testing.T, r row) {
	// Unverified simulator injection inside the 500 ms drive-state enable bound.
	f := start(t, "--sim-enable-delay-cycles", "300")
	h := f.prepare(101, 0.002, 200*time.Millisecond)
	at := ipc.HostMonotonicNS()
	f.command(f.c.Enable(context.Background(), 3))
	f.command(f.c.Arm(context.Background()))
	_, err := f.c.StartTrajectory(context.Background(), h)
	reject(t, err, ipc.ReasonNotReady)
	ready := f.until(at, r.boundNS, enabled)
	if ready.ExecutionFaultReasons != 0 || ready.Axes[0].PositionCounts != 0 || ready.Axes[1].PositionCounts != 0 {
		t.Fatalf("rejected plan moved/faulted: %+v", ready)
	}
	h = f.prepare(102, 0.003, 200*time.Millisecond)
	f.execute(h)
	moved := f.until(ipc.HostMonotonicNS(), 200_000_000+r.boundNS, func(s ipc.Status) bool { return s.Axes[0].PositionCounts >= 62 && s.Axes[1].PositionCounts <= -62 })
	if moved.SafetyFaultMask != 0 {
		t.Fatalf("fresh ready Start faulted: %+v", moved)
	}
}
