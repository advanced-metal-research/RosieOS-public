//go:build linux

package faultmatrix

import (
	"encoding/json"
	"os"
	"path/filepath"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

func statuswordFault(t *testing.T, r row) {
	f := start(t, "--sim-statusword-fault-after-enabled-ns", "1000000000")
	f.arm()
	f.execute(f.prepare(501, 0.02, 3*time.Second))
	at, _ := f.event("sim_statusword_fault")
	s := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.ExecutionFaultReasons&r.fault != 0 })
	inhibited(t, s, r.fault)
	if s.Axes[0].Statusword&8 == 0 {
		t.Fatal("statusword injection absent")
	}
	f.hold(s, 400_000_000, 0)
}
func untypedStatusword(t *testing.T, r row) {
	t.Skip("P4-04: CONTRACTS.md section 1 Untyped output fault: exhaustive unique fault classification is not enforced")
}
func stalePDO(t *testing.T, r row) {
	f := start(t, "--sim-receive-fail-from-call", "2000", "--sim-receive-fail-for-calls", "1000")
	f.arm()
	f.execute(f.prepare(503, 0, 4*time.Second))
	at, _ := f.event("sim_receive_failure")
	s := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.ExecutionFaultReasons&r.fault == r.fault })
	inhibited(t, s, r.fault)
	read := func() []uint64 {
		raw, err := os.ReadFile(filepath.Join(f.socketDir, "metrics.json"))
		must(t, err)
		var m struct {
			Axes []struct {
				Time uint64 `json:"pdo_observed_time_ns"`
			} `json:"axes"`
		}
		must(t, json.Unmarshal(raw, &m))
		v := []uint64{}
		for _, a := range m.Axes {
			v = append(v, a.Time)
		}
		return v
	}
	before := read()
	f.until(s.TimeNS+1, r.boundNS, func(next ipc.Status) bool { return true })
	after := read()
	if len(before) != 2 || len(after) != 2 || before[0] == 0 || before[0] != after[0] || before[1] != after[1] {
		t.Fatalf("stale PDO manufactured timestamps: %v -> %v", before, after)
	}
}
