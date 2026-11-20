//go:build linux

package control

import "testing"

// testdata/deploy-tool-notice.json is the servo host's notice exactly as the
// deploy tool's one writer produces it (steamdeck/real/v4
// rt_core_maintenance_contract_test.go regenerates and compares it). The
// guard must read it as an active, well-formed notice with the deploy's own
// reason, phase, times, author and release, and refuse authority with it.
func TestMaintenanceReadsTheDeployToolNotice(t *testing.T) {
	state := readMaintenanceFile("testdata/deploy-tool-notice.json")
	want := maintenanceState{Active: true, Reason: "rt-core full deploy on example-arm-1420", Phase: "installing on the servo host",
		StartedAt: "2026-10-07T19:50:00Z", ETA: "2026-10-07T20:30:00Z", SetBy: "realtime-core deploy (operator@mac)", Release: "bc423a86ead5"}
	if state != want {
		t.Fatalf("read %+v, want %+v", state, want)
	}
	if !maintenanceRefuses(OperationArm) || maintenanceRefuses(OperationStop) {
		t.Fatal("an active notice refuses Arm and never Stop")
	}
}
