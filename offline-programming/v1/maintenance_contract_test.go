package main

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// The relay's notice is the deploy tool's one writer's output under this
// server's schema; the servo host's copy (rt-core's golden
// adapters/rosie/control/testdata/deploy-tool-notice.json) differs only in its
// schema. This server reads it as an active, well-formed notice with the
// deploy's reason, phase, author, release and run, and neither overrun nor
// stale before its ETA.
func TestMaintenanceReadsTheDeployToolNotice(t *testing.T) {
	raw, err := os.ReadFile("../../rt-core/adapters/rosie/control/testdata/deploy-tool-notice.json")
	if err != nil {
		t.Fatal(err)
	}
	relay := strings.Replace(string(raw), `"rosie.rt-control.maintenance.v1"`, `"`+maintenanceFileSchema+`"`, 1)
	path := filepath.Join(t.TempDir(), "maintenance.json")
	if err := os.WriteFile(path, []byte(relay), 0o644); err != nil {
		t.Fatal(err)
	}
	notice, err := readMaintenanceNotice(path)
	if err != nil {
		t.Fatal(err)
	}
	now, _ := time.Parse(time.RFC3339, "2026-10-07T20:00:00Z")
	view := maintenanceViewOf(notice, nil, now, defaultMaintenanceStaleAfter)
	if !view.Active || view.Malformed || view.Overrun || view.Stale || view.Reason != "rt-core full deploy on example-arm-1420" ||
		view.Phase != "installing on the servo host" || view.SetBy != "realtime-core deploy (operator@mac)" || view.Release != "bc423a86ead5" ||
		view.RunID != "realtime-core-deploy-20261007T195000Z-example-arm-1420" || view.ETA != "2026-10-07T20:30:00Z" {
		t.Fatalf("view %+v", view)
	}
}
