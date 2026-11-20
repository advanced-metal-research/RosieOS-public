package main

import (
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

// rtCoreMaintenanceNoticeGolden is the servo host's notice as this tool
// writes it, fixed times aside. rt-control's reader is tested against the
// same file (rt-core/adapters/rosie/control maintenance_contract_test.go), so
// the one writer and the reader cannot drift apart unnoticed: change both
// and regenerate with ROSIE_UPDATE_GOLDEN=1.
const rtCoreMaintenanceNoticeGolden = "rt-core/adapters/rosie/control/testdata/deploy-tool-notice.json"

func TestRTCoreMaintenanceWriterMatchesTheRTControlReaderGolden(t *testing.T) {
	if _, err := exec.LookPath("python3"); err != nil {
		t.Skip("python3")
	}
	dir := t.TempDir()
	script := filepath.Join(dir, "writer.py")
	if err := os.WriteFile(script, []byte(rtCoreMaintenanceWriterPy), 0o600); err != nil {
		t.Fatal(err)
	}
	// The arguments maint_set passes: file, run, prefix, reason, phase, eta s,
	// set-by, release, owner pid, lock.
	out, err := exec.Command("python3", script, filepath.Join(dir, "absent.json"), "realtime-core-deploy-20261007T195000Z-example-arm-1420",
		rtCoreMaintenanceRunPrefix, "rt-core full deploy on example-arm-1420", "installing on the servo host", "2400",
		"realtime-core deploy (operator@mac)", "bc423a86ead5", "4242", "/home/operator/.rosie/locks/example-arm-1420.lock").Output()
	if err != nil {
		t.Fatal(err)
	}
	var notice map[string]any
	if err := json.Unmarshal(out, &notice); err != nil {
		t.Fatal(err)
	}
	// maint_servo_write: the servo host's copy is the relay notice under rt-control's schema.
	notice["schema"] = rtCoreMaintenanceServoSchema
	notice["started_at"], notice["eta"], notice["updated_at"] = "2026-10-07T19:50:00Z", "2026-10-07T20:30:00Z", "2026-10-07T19:50:00Z"
	if owner, ok := notice["owner"].(map[string]any); ok {
		owner["host"] = "example-relay"
	}
	got, err := json.MarshalIndent(notice, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	got = append(got, '\n')
	repo, err := repoRoot()
	if err != nil {
		t.Fatal(err)
	}
	golden := filepath.Join(repo, filepath.FromSlash(rtCoreMaintenanceNoticeGolden))
	if os.Getenv("ROSIE_UPDATE_GOLDEN") == "1" {
		if err := os.WriteFile(golden, got, 0o644); err != nil {
			t.Fatal(err)
		}
	}
	want, err := os.ReadFile(golden)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(want) {
		t.Fatalf("the writer's notice drifted from the rt-control reader's golden %s:\n%s\nwant\n%s", rtCoreMaintenanceNoticeGolden, got, want)
	}
}
