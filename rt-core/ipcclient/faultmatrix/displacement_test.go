//go:build linux

package faultmatrix

import (
	"context"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
)

func continuousDisplacement(t *testing.T, r row) {
	f := start(t, "--sim-disabled-displacement-counts", "2000")
	f.arm()
	at := ipc.HostMonotonicNS()
	_, err := f.c.Stop(context.Background())
	must(t, err)
	s := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.Armed == 0 && s.Axes[0].PositionCounts == 2000 })
	inhibited(t, s, 0)
	rearmed := f.arm()
	if rearmed.Axes[0].PositionCounts < 1999 || rearmed.Axes[0].PositionCounts > 2001 || rearmed.SafetyFaultMask != 0 {
		t.Fatalf("re-arm lost observed anchor: %+v", rearmed)
	}
	f.hold(rearmed, 400_000_000, 1)
}
