//go:build linux

package faultmatrix

import (
	"encoding/json"
	"os"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
)

// Unverified outside simulation; CONTRACTS.md section 1 fixes these bit assignments.
const (
	followingFault  uint32 = 1 << 5
	coordinateFault uint32 = 1 << 6
	readinessFault  uint32 = 1 << 7
	transportFault  uint32 = 1 << 8
)

type row struct {
	name, injection, before, after, reason, faultName string
	fault                                             uint32
	boundNS                                           uint64
	contract                                          string
	run                                               func(*testing.T, row)
}

// TestMatrix keeps the independent contract, injection and absolute observation
// budget together so a regression cannot silently disappear from the matrix.
func TestMatrix(t *testing.T) {
	raw, err := os.ReadFile("../../config/machines/simulation/simulation.json")
	must(t, err)
	var config struct {
		Cycle uint64 `json:"cycle_ns"`
	}
	must(t, json.Unmarshal(raw, &config))
	bound := config.Cycle + publicationNS
	rows := []row{
		{"a_delayed_enable", "enable delayed 1000 cycles", "E", "E then P", "not_ready then none", "none", 0, 1_000*config.Cycle + bound, "section 1 start I/E; start R", delayedEnable},
		{"b_continuous_displacement", "disabled displacement +2000 counts", "R -> I", "R at observed count", "none", "none", 0, bound, "section 1 Disabled coordinate reconciliation", continuousDisplacement},
		{"c_mode_mismatch", "mode 9 after 1000000000 enabled ns", "P", "F inhibited; 0 counts after fault", "no command reason", "FaultDriveReadiness", readinessFault, bound, "section 1 FaultDriveReadiness (motion)", modeMismatch},
		{"d_send", "send -5 from call 2000 for 1000 calls", "P", "F; ACK only after successful submission", "no command reason", "FaultBusTransport", transportFault, bound, "section 1 FaultBusTransport; stop All", busFailure},
		{"d_queue", "queue -5 from call 2000 for 1000 calls", "P", "F; ACK only after successful submission", "no command reason", "FaultBusTransport", transportFault, bound, "section 1 FaultBusTransport; stop All", busFailure},
		{"d_receive", "receive -5 from call 2000 for 1000 calls", "P", "F; inhibited submission can ACK", "no command reason", "FaultBusTransport", transportFault, bound, "section 1 FaultBusTransport; Applied to bus", busFailure},
		{"d_dc", "DC -5 from call 2000 for 1000 calls", "P", "F; inhibited submission can ACK", "no command reason", "FaultBusTransport", transportFault, bound, "section 1 FaultBusTransport; Applied to bus", busFailure},
		{"d_process", "process -5 from call 2000 for 1000 calls", "P", "F; inhibited submission can ACK", "no command reason", "FaultBusTransport", transportFault, bound, "section 1 FaultBusTransport; Applied to bus", busFailure},
		{"e_statusword", "statusword bit 3 after 1000000000 enabled ns", "P", "F inhibited", "no command reason", "FaultDriveReadiness", readinessFault, bound, "section 1 FaultDriveReadiness (motion)", statuswordFault},
		{"e_statusword_unique_name", "statusword bit 3 while idle", "I", "untyped safety inhibition", "not enforced", "untyped", 0, bound, "section 1 Untyped output fault: exhaustive unique fault classification is not enforced (P4-04)", untypedStatusword},
		{"e_stale_pdo", "receive not exchanged for 1000 calls", "P", "F; receipt timestamp frozen", "no command reason", "FaultBusTransport", transportFault, bound, "section 1 FaultBusTransport (receive failure precedes readiness); Physically observed", stalePDO},
		{"f_blocked_sdo", "Home offset download blocked 2000000000 ns", "C", "D until worker Finish, then I", "not_ready (terminal Home)", "none", 0, bound, "section 1 Stop while service Active; Finish C/D; Final Home result", blockedSDO},
		{"g_output_lag", "actuator feedback offset -1000 counts exceeds 834.431 counts for 100000000 ns", "R", "F inhibited", "no command reason", "FaultFollowingError", followingFault, 100_000_000 + bound, "section 1 FaultFollowingError", outputLag},
		{"h_one_connection", "concurrent native Start and Stop", "R", "I; old handle retired", "unknown_handle (late Start)", "none", 0, bound, "section 1 stop All; Native generation change", concurrentOne},
		{"h_two_connections", "concurrent public HTTP Start and Stop", "R", "I; generation fences late Start", "control_session_stale", "none", 0, bound, "section 1 Public Stop versus Release; Public session", concurrentTwo},
		{"i_old_grant", "Acquire generation at retained floor", "I", "I", "stale_generation", "none", 0, bound, "section 1 Grant guard Acquire generation at/below floor", staleGrant},
		{"i_old_native", "Renew with old native generation after grant expiry", "I", "I", "native_generation_mismatch", "none", 0, bound, "section 1 Grant guard Acquire/Renew native generation differs", staleNative},
		{"i_old_jog", "old jog generation on current identity", "J", "J with no admitted input", "invalid_generation", "none", 0, bound, "section 1 Jog guard Update identity/generation/sequence/group differs", staleJog},
		{"i_wrong_session", "wrong public session on Start", "R", "R no motion", "control_session_stale", "none", 0, bound, "section 1 Public session authorize", wrongSession},
		{"j_digest", "modified compiled artifact with unchanged digest", "startup", "exit 2", "machine identity mismatch: axis.0.limits", "none", 0, bound, "P1-02/P1-03: effective machine digest verification", corruptDigest},
		{"j_missing_field", "remove compiled tracking descriptor", "startup", "exit 2", "invalid --axis-profiles file", "none", 0, bound, "section 3 R10: explicit native descriptors; TestNativeCompiledProfilesRequireEveryField", corruptMissing},
		{"j_unknown_field", "add unknown compiled field", "startup", "exit 2", "invalid --axis-profiles file", "none", 0, bound, "section 3 R10: explicit native descriptors and dead defaults", corruptUnknown},
		{"k_restart", "restart daemon with prepared and executing work", "P", "I; new incarnation, no grant or plan", "unknown_handle; retired (adapter)", "none", 0, bound, "section 1 Process restart; No replay; Daemon incarnation", restart},
	}
	for _, r := range rows {
		t.Run(r.name, func(t *testing.T) {
			t.Logf("injection=%s; %s -> %s; reason=%s fault=%s/%#x bound_ns=%d; CONTRACTS.md %s", r.injection, r.before, r.after, r.reason, r.faultName, r.fault, r.boundNS, r.contract)
			r.run(t, r)
		})
	}
}

func TestMatrixPublication(t *testing.T) {
	f := start(t)
	previous := f.status()
	var max uint64
	for i := 0; i < 5; i++ {
		s := f.until(previous.TimeNS+1, publicationNS, func(s ipc.Status) bool { return true })
		gap := s.TimeNS - previous.TimeNS
		if gap > max {
			max = gap
		}
		previous = s
	}
	t.Logf("metrics samples=6 maximum_publication_gap_ns=%d fixed_allowance_ns=%d", max, publicationNS)
}
