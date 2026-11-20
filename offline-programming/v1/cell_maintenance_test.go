package main

import (
	"context"
	"errors"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/rt-core/sdk/control"
)

// fakeCellRTControl is rt-control's GET /v1/maintenance as the SDK returns it.
type fakeCellRTControl struct {
	observation control.MaintenanceObservation
	err         error
	reads       int
	closed      bool
}

func (f *fakeCellRTControl) Maintenance(context.Context) (control.MaintenanceObservation, error) {
	f.reads++
	return f.observation, f.err
}

func (f *fakeCellRTControl) Close() error { f.closed = true; return nil }

var cellNotice = control.MaintenanceObservation{Active: true, Reason: "rt-core full deploy on example-arm-1420",
	Phase: "installing on the servo host", StartedAt: "2026-10-07T19:45:00Z", ETA: "2026-10-07T20:40:00Z",
	SetBy: "realtime-core deploy (operator@mac)", Release: "9a48c367",
	Holder: &control.MaintenanceHolder{Controller: "offline-programming:server-abc", Remote: true, Identity: "steamdeck-pendant-olp", Generation: 4, Since: "2026-10-07T19:30:00Z"}}

type cellMaintenanceFixture struct {
	reader   *CellMaintenance
	rtc      *fakeCellRTControl
	now      time.Time
	selected *cellRTControl
	dials    []denseexec.RTCoreConfig
}

func newCellMaintenanceFixture() *cellMaintenanceFixture {
	f := &cellMaintenanceFixture{rtc: &fakeCellRTControl{}, now: maintenanceTestNow,
		selected: &cellRTControl{CellID: "example-arm-1420", Host: "example-servo:8443", Config: denseexec.RTCoreConfig{Address: "https://example-servo:8443"}}}
	f.reader = newCellMaintenance(func() (cellRTControl, bool) {
		if f.selected == nil {
			return cellRTControl{}, false
		}
		return *f.selected, true
	})
	f.reader.now = func() time.Time { return f.now }
	f.reader.dial = func(cfg denseexec.RTCoreConfig) (cellMaintenanceClient, error) {
		f.dials = append(f.dials, cfg)
		return f.rtc, nil
	}
	return f
}

// A notice on the servo host alone blocks: this host's file is absent, as on a
// Steam Deck running OLP locally.
func TestCellNoticeAloneMakesTheViewActive(t *testing.T) {
	local := maintenanceViewOf(nil, nil, maintenanceTestNow, 2*time.Hour)
	cell := &MaintenanceCell{Cell: "example-arm-1420", Reachable: true, Supported: true, MaintenanceObservation: cellNotice}
	view := mergeMaintenance(local, cell, maintenanceTestNow, 2*time.Hour)
	if !view.Active || view.Reason != cellNotice.Reason || view.Phase != cellNotice.Phase || view.ETA != cellNotice.ETA ||
		view.StartedAt != cellNotice.StartedAt || view.SetBy != cellNotice.SetBy || view.Release != cellNotice.Release || view.Overrun || view.Stale || view.Cell != cell {
		t.Fatalf("cell-only notice: %+v", view)
	}
	inactive := mergeMaintenance(local, &MaintenanceCell{Reachable: true, Supported: true}, maintenanceTestNow, 2*time.Hour)
	if inactive.Active || inactive.Reason != "" || inactive.Cell == nil {
		t.Fatalf("inactive cell notice: %+v", inactive)
	}
}

// Both notices: active, both reasons when they differ, the later ETA, the
// earlier start; this host's set_by and release win when present.
func TestBothNoticesMergeReasonsAndTakeTheLaterETA(t *testing.T) {
	notice, err := readMaintenanceNoticeBytes(t, activeNotice)
	if err != nil {
		t.Fatal(err)
	}
	local := maintenanceViewOf(notice, nil, maintenanceTestNow, 2*time.Hour)
	cell := &MaintenanceCell{Cell: "example-arm-1420", Reachable: true, Supported: true, MaintenanceObservation: cellNotice}
	view := mergeMaintenance(local, cell, maintenanceTestNow, 2*time.Hour)
	if !view.Active || view.ETA != "2026-10-07T20:40:00Z" || view.StartedAt != "2026-10-07T19:45:00Z" || view.SetBy != local.SetBy || view.Release != "fd9af899" {
		t.Fatalf("merged times/source: %+v", view)
	}
	for _, want := range []string{"rt-core full deploy fd9af899", "servo host (example-arm-1420): rt-core full deploy on example-arm-1420"} {
		if !strings.Contains(view.Reason, want) {
			t.Fatalf("reason %q lacks %q", view.Reason, want)
		}
	}
	same := *cell
	same.Reason, same.Phase = local.Reason, local.Phase
	if view := mergeMaintenance(local, &same, maintenanceTestNow, 2*time.Hour); view.Reason != local.Reason || view.Phase != local.Phase {
		t.Fatalf("identical reasons must read once: %q / %q", view.Reason, view.Phase)
	}
	overrun := *cell
	overrun.ETA = "2026-10-07T17:00:00Z"
	overrun.StartedAt = "2026-10-07T16:00:00Z"
	if view := mergeMaintenance(maintenanceViewOf(nil, nil, maintenanceTestNow, 2*time.Hour), &overrun, maintenanceTestNow, 2*time.Hour); !view.Overrun || !view.Stale || !view.Active {
		t.Fatalf("a cell notice 3 h past its ETA stays active, overrun and stale: %+v", view)
	}
}

func readMaintenanceNoticeBytes(t *testing.T, raw string) (*maintenanceNoticeFile, error) {
	t.Helper()
	path := filepath.Join(t.TempDir(), "notice.json")
	if err := os.WriteFile(path, []byte(raw), 0o644); err != nil {
		return nil, err
	}
	return readMaintenanceNotice(path)
}

// One read per period whoever asks; the last active notice stays while
// rt-control does not answer (a deploy restarts it under that notice); an
// rt-control without the route answers, it just cannot say.
func TestCellMaintenanceReaderCachesAndKeepsAnActiveNoticeThroughARestart(t *testing.T) {
	f := newCellMaintenanceFixture()
	f.rtc.observation = cellNotice
	first := f.reader.Current(context.Background())
	if first == nil || !first.Active || !first.Reachable || !first.Supported || first.Holder == nil || first.Holder.Identity != "steamdeck-pendant-olp" || first.AnsweredAt != "2026-10-07T20:00:00Z" {
		t.Fatalf("first read: %+v", first)
	}
	f.now = f.now.Add(time.Second)
	if again := f.reader.Current(context.Background()); f.rtc.reads != 1 || !again.Active {
		t.Fatalf("read again within the period: reads=%d", f.rtc.reads)
	}
	f.now = f.now.Add(2 * time.Second)
	f.rtc.err = errors.New("dial tcp: connection refused")
	down := f.reader.Current(context.Background())
	if f.rtc.reads != 2 || !down.Active || down.Reachable || down.Reason != cellNotice.Reason || down.ETA != cellNotice.ETA || down.Holder != nil ||
		down.AnsweredAt != "2026-10-07T20:00:00Z" || !strings.Contains(down.Error, "connection refused") {
		t.Fatalf("unreachable after an active notice: %+v", down)
	}
	if view := mergeMaintenance(maintenanceViewOf(nil, nil, f.now, 2*time.Hour), down, f.now, 2*time.Hour); !view.Active || !strings.Contains(view.Error, "not answering") {
		t.Fatalf("merged view while rt-control restarts: %+v", view)
	}
	f.now = f.now.Add(3 * time.Second)
	f.rtc.err, f.rtc.observation = nil, control.MaintenanceObservation{}
	if back := f.reader.Current(context.Background()); back.Active || !back.Reachable {
		t.Fatalf("rt-control back with no notice: %+v", back)
	}
	f.now = f.now.Add(3 * time.Second)
	f.rtc.err = errors.New("i/o timeout")
	if down := f.reader.Current(context.Background()); down.Active || down.Reachable {
		t.Fatalf("unreachable with no notice seen is the ordinary disconnected state: %+v", down)
	}
	f.now = f.now.Add(3 * time.Second)
	f.rtc.err = &control.Rejected{Reason: "unknown_endpoint"}
	if old := f.reader.Current(context.Background()); old.Active || !old.Reachable || old.Supported || !strings.Contains(old.Error, "no maintenance route") {
		t.Fatalf("rt-control older than the route: %+v", old)
	}
	if len(f.dials) != 1 {
		t.Fatalf("one client per selected cell, got %d dials", len(f.dials))
	}
}

// A new selection reads its own rt-control from scratch; no selection reads none.
func TestCellMaintenanceFollowsTheSelection(t *testing.T) {
	f := newCellMaintenanceFixture()
	f.rtc.observation = cellNotice
	if view := f.reader.Current(context.Background()); view == nil || !view.Active {
		t.Fatalf("first cell: %+v", view)
	}
	first := f.rtc
	f.rtc = &fakeCellRTControl{}
	f.selected = &cellRTControl{CellID: "local-simulation", Host: "localhost", Config: denseexec.RTCoreConfig{Socket: "/run/rt/control.sock"}}
	view := f.reader.Current(context.Background())
	if !first.closed || view == nil || view.Active || view.Cell != "local-simulation" || len(f.dials) != 2 || f.dials[1].Socket != "/run/rt/control.sock" {
		t.Fatalf("reselect: closed=%t view=%+v dials=%+v", first.closed, view, f.dials)
	}
	f.selected = nil
	if view := f.reader.Current(context.Background()); view != nil || !f.rtc.closed {
		t.Fatalf("no selection: %+v closed=%t", view, f.rtc.closed)
	}
	var none *CellMaintenance
	if none.Current(context.Background()) != nil {
		t.Fatal("a nil reader reads nothing")
	}
}

// Through the server: the route shows the cell's notice and its holder, robot
// actions are refused with it, protective requests pass, and a select is
// refused only by this host's notice.
func TestServerMergesTheCellNoticeIntoRouteAndGate(t *testing.T) {
	f := newCellMaintenanceFixture()
	f.rtc.observation = cellNotice
	source := NewMaintenanceSource(filepath.Join(t.TempDir(), "absent.json"), 2*time.Hour)
	source.now = func() time.Time { return maintenanceTestNow }
	backend := &maintenanceBackend{}
	server := NewServer(ServerOptions{DenseExecution: backend, Maintenance: source, CellMaintenance: f.reader})
	response, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, "")
	cell, _ := body["cell"].(map[string]any)
	holder, _ := cell["holder"].(map[string]any)
	if response.Code != http.StatusOK || body["active"] != true || body["reason"] != cellNotice.Reason || body["eta"] != cellNotice.ETA ||
		cell["active"] != true || cell["reachable"] != true || holder["controller"] != "offline-programming:server-abc" || holder["remote"] != true {
		t.Fatalf("route: %s", response.Body.String())
	}
	for _, request := range maintenanceRefusedRequests {
		if strings.HasSuffix(request.path, "/targets/select") {
			continue
		}
		response, body := maintenanceRequest(t, server, request.method, request.path, request.body)
		if response.Code != http.StatusServiceUnavailable || !strings.Contains(body["detail"].(string), cellNotice.Reason) {
			t.Fatalf("%s %s: %d %s", request.method, request.path, response.Code, response.Body.String())
		}
	}
	for _, request := range maintenanceProtectiveRequests {
		if _, body := maintenanceRequest(t, server, request.method, request.path, request.body); body["error"] == "maintenance" {
			t.Fatalf("%s %s refused by the cell notice", request.method, request.path)
		}
	}
	if _, body := maintenanceRequest(t, server, "POST", "/api/offline-programming/v1/targets/select", `{"cell_id":"c","model_id":"m"}`); body["error"] == "maintenance" {
		t.Fatal("the selected cell's notice refused selecting a cell")
	}
	if backend.arms != 1 || backend.homes != 0 || backend.plays != 0 {
		t.Fatalf("only the protective disarm reaches the controller: %+v", backend)
	}
	if f.rtc.reads != 1 {
		t.Fatalf("every request re-read rt-control within one period: %d reads", f.rtc.reads)
	}
}
