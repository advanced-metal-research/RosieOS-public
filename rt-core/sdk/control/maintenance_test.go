//go:build linux

package control

import (
	"context"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"testing"

	adapter "rosieos/rt-core/adapters/rosie/control"
)

// The SDK decodes the adapter's own handler: the notice the guard enforces,
// with no session, grant or Describe needed first.
func TestSDKMaintenanceReadsTheAdapterNotice(t *testing.T) {
	path := filepath.Join(t.TempDir(), "maintenance.json")
	controller := &adapter.Controller{}
	controller.SetMaintenanceFile(path)
	server := httptest.NewServer(adapter.NewHandler(controller))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	observation, err := client.Maintenance(context.Background())
	if err != nil || observation.Active || observation.Holder != nil {
		t.Fatalf("no notice: %+v %v", observation, err)
	}
	if err := os.WriteFile(path, []byte(`{"schema":"rosie.rt-control.maintenance.v1","reason":"rt-core full deploy on rosie_1420_v1","phase":"waiting for operators",
"started_at":"2026-10-07T19:50:00Z","eta":"2026-10-07T20:35:00Z","set_by":"realtime-core deploy","release":"abc123"}`), 0o644); err != nil {
		t.Fatal(err)
	}
	observation, err = client.Maintenance(context.Background())
	want := MaintenanceObservation{Active: true, Reason: "rt-core full deploy on rosie_1420_v1", Phase: "waiting for operators",
		StartedAt: "2026-10-07T19:50:00Z", ETA: "2026-10-07T20:35:00Z", SetBy: "realtime-core deploy", Release: "abc123"}
	if err != nil || observation != want {
		t.Fatalf("notice:\n got %+v %v\nwant %+v", observation, err, want)
	}
	if client.Grant().Session != "" {
		t.Fatal("reading the notice must not take a grant")
	}
}
