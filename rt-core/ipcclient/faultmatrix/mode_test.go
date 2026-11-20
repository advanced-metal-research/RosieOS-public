//go:build linux

package faultmatrix

import (
	ipc "rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

func modeMismatch(t *testing.T, r row) {
	f := start(t, "--sim-mode-mismatch-after-enabled-ns", "1000000000")
	f.arm()
	f.execute(f.prepare(301, 0.02, 3*time.Second))
	at, count := f.event("sim_mode_mismatch")
	s := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.ExecutionFaultReasons&r.fault != 0 })
	inhibited(t, s, r.fault)
	if s.Axes[0].PositionCounts != count {
		t.Fatalf("mode fault moved axis 0: injection=%d after=%d", count, s.Axes[0].PositionCounts)
	}
	f.hold(s, 400_000_000, 0)
}
