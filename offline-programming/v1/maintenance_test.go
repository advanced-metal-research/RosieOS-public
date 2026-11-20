package main

import (
	"bytes"
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
)

var maintenanceTestNow = time.Date(2026, 10, 7, 20, 0, 0, 0, time.UTC)

// maintenanceBackend counts what reached the controller, Pause and the
// session deadman's heartbeat included.
type maintenanceBackend struct {
	identityBackend
	pauses, heartbeats int
}

func (b *maintenanceBackend) Pause(ctx context.Context, _ string) (*denseexec.StatusBody, error) {
	b.pauses++
	return b.Status(ctx), nil
}

func (b *maintenanceBackend) Heartbeat(string) (*denseexec.HeartbeatResult, error) {
	b.heartbeats++
	return &denseexec.HeartbeatResult{}, nil
}

func maintenanceTestServer(t *testing.T, notice string) (*Server, *maintenanceBackend, string, *MaintenanceSource) {
	t.Helper()
	path := filepath.Join(t.TempDir(), "maintenance.json")
	if notice != "" {
		if err := os.WriteFile(path, []byte(notice), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	source := NewMaintenanceSource(path, 2*time.Hour)
	source.now = func() time.Time { return maintenanceTestNow }
	backend := &maintenanceBackend{}
	return NewServer(ServerOptions{DenseExecution: backend, Maintenance: source}), backend, path, source
}

func maintenanceRequest(t *testing.T, server *Server, method, path, body string) (*httptest.ResponseRecorder, map[string]any) {
	t.Helper()
	response := httptest.NewRecorder()
	server.Handler().ServeHTTP(response, httptest.NewRequest(method, path, strings.NewReader(body)))
	var decoded map[string]any
	_ = json.Unmarshal(response.Body.Bytes(), &decoded)
	return response, decoded
}

const activeNotice = `{"schema":"rosie.olp.maintenance.v1","active":true,"reason":"rt-core full deploy fd9af899",
"phase":"installing package on the servo host","started_at":"2026-10-07T19:50:00Z","eta":"2026-10-07T20:30:00Z",
"set_by":"realtime-core deploy (operator@example-relay)","release":"fd9af899","run_id":"run-1"}`

// The robot actions a notice must refuse: anything that arms, starts or
// continues motion, Homes, clears faults, loads or plays, or takes a target.
var maintenanceRefusedRequests = []struct{ method, path, body string }{
	{"POST", denseExecutionPath + "/arm", `{"armed":true}`},
	{"POST", denseExecutionPath + "/jog/begin", `{}`},
	{"POST", denseExecutionPath + "/trace/begin", `{}`},
	{"POST", denseExecutionPath + "/cartesian/start", `{}`},
	{"POST", denseExecutionPath + "/cartesian/move", `{}`},
	{"POST", denseExecutionPath + "/move", `{}`},
	{"POST", denseExecutionPath + "/load", `{}`},
	{"POST", denseExecutionPath + "/play", `{"session_id":"s"}`},
	{"POST", denseExecutionPath + "/home", `{}`},
	{"POST", denseExecutionPath + "/go-home", `{}`},
	{"POST", denseExecutionPath + "/reset-fault", `{}`},
	{"POST", denseExecutionPath + "/recover-to-zero", `{"fraction":0.1,"confirm_arm":true}`},
	{"POST", denseExecutionPath + "/run-program", `{"fraction":0.1,"confirm_arm":true,"confirm_move":true}`},
	{"POST", denseExecutionPath + "/target", `{"cell":"example-arm-1420"}`},
	{"POST", denseExecutionPath + "/cells", `{}`},
	{"DELETE", denseExecutionPath + "/cells/other", ``},
	{"POST", denseExecutionPath + "/some-future-motion-route", `{}`},
	{"POST", "/api/offline-programming/v1/targets/select", `{"cell_id":"c","model_id":"m"}`},
	{"POST", executionPath + "/start", `{}`},
	{"POST", executionPath + "/review", `{}`},
	{"POST", executionGatewayPath + "/review", `{}`},
}

// Protective requests a notice must never refuse: they stop, disarm,
// release, keep a session's deadman fed, or end/continue a running hold.
var maintenanceProtectiveRequests = []struct{ method, path, body string }{
	{"POST", denseExecutionPath + "/stop", `{}`},
	{"POST", denseExecutionPath + "/pause", `{"session_id":"s"}`},
	{"POST", denseExecutionPath + "/heartbeat", `{"session_id":"s"}`},
	{"POST", denseExecutionPath + "/run-program/finish", `{}`},
	{"POST", denseExecutionPath + "/arm", `{"armed":false}`},
	{"POST", denseExecutionPath + "/target", `{"cell":""}`},
	{"POST", denseExecutionPath + "/jog/end", `{}`},
	{"POST", denseExecutionPath + "/jog/update", `{}`},
	{"POST", denseExecutionPath + "/trace/update", `{}`},
	{"POST", denseExecutionPath + "/trace/end", `{}`},
	{"POST", denseExecutionPath + "/cartesian/halt", `{}`},
	{"POST", denseExecutionPath + "/cartesian/stop", `{}`},
	{"POST", denseExecutionPath + "/cartesian/intent", `{}`},
	{"POST", executionPath + "/exec-1/stop", `{}`},
}

func TestMaintenanceNoticeRefusesRobotActionsWithReasonAndETA(t *testing.T) {
	server, backend, _, _ := maintenanceTestServer(t, activeNotice)
	for _, request := range maintenanceRefusedRequests {
		response, body := maintenanceRequest(t, server, request.method, request.path, request.body)
		if response.Code != http.StatusServiceUnavailable || body["error"] != "maintenance" {
			t.Fatalf("%s %s: status=%d body=%s", request.method, request.path, response.Code, response.Body.String())
		}
		detail, _ := body["detail"].(string)
		for _, want := range []string{"rt-core full deploy fd9af899", "installing package on the servo host", "2026-10-07T20:30:00Z", "in 30m00s", "Stop, Disarm and Disconnect still work"} {
			if !strings.Contains(detail, want) {
				t.Fatalf("%s detail %q lacks %q", request.path, detail, want)
			}
		}
		if retry := response.Header().Get("Retry-After"); retry != "1800" {
			t.Fatalf("%s Retry-After=%q, want the 1800 s to the ETA", request.path, retry)
		}
		notice, _ := body["maintenance"].(map[string]any)
		if notice["active"] != true || notice["run_id"] != "run-1" {
			t.Fatalf("%s carries no notice: %v", request.path, body)
		}
	}
	if backend.arms != 0 || backend.homes != 0 || backend.plays != 0 {
		t.Fatalf("a refused action reached the controller: %+v", backend)
	}
}

func TestMaintenanceNoticeNeverRefusesProtectiveOrReadRequests(t *testing.T) {
	server, backend, _, _ := maintenanceTestServer(t, activeNotice)
	for _, request := range maintenanceProtectiveRequests {
		response, body := maintenanceRequest(t, server, request.method, request.path, request.body)
		if body["error"] == "maintenance" {
			t.Fatalf("%s %s refused by the notice: %s", request.method, request.path, response.Body.String())
		}
	}
	if backend.stops != 1 || backend.arms != 1 || backend.pauses != 1 || backend.heartbeats != 1 {
		t.Fatalf("Stop, Disarm, Pause and the heartbeat must reach the controller during maintenance: %+v", backend)
	}
	for _, path := range []string{maintenanceAPIPath, "/api/offline-programming/v1/health", denseExecutionPath + "/status"} {
		response, _ := maintenanceRequest(t, server, "GET", path, "")
		if response.Code != http.StatusOK {
			t.Fatalf("GET %s status=%d during maintenance: %s", path, response.Code, response.Body.String())
		}
	}
	// Planning, authoring and the local simulator are not robot actions.
	for _, path := range []string{programsPath, "/api/offline-programming/v1/local-simulator/jog/start", WeldPlanAPIPath} {
		_, body := maintenanceRequest(t, server, "POST", path, `{}`)
		if body["error"] == "maintenance" {
			t.Fatalf("POST %s refused by the notice", path)
		}
	}
}

func TestMaintenanceRouteReportsNoticeAndClearAllowsActions(t *testing.T) {
	server, backend, path, _ := maintenanceTestServer(t, activeNotice)
	response, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, "")
	if response.Code != http.StatusOK || body["active"] != true || body["phase"] != "installing package on the servo host" ||
		body["eta"] != "2026-10-07T20:30:00Z" || body["now"] != "2026-10-07T20:00:00Z" || body["overrun"] != false || body["stale"] != false {
		t.Fatalf("notice view: %s", response.Body.String())
	}
	if response.Header().Get("Cache-Control") != "no-store" {
		t.Fatal("a notice must not be cached by a client or proxy")
	}
	if server, _ := body["server"].(map[string]any); server["build_id"] == "" || server["build_id"] == nil {
		t.Fatalf("no build id: %s", response.Body.String())
	}
	_, health := maintenanceRequest(t, server, "GET", "/api/offline-programming/v1/health", "")
	if health["maintenance"].(map[string]any)["active"] != true || health["build_id"] == "" {
		t.Fatalf("health: %v", health)
	}

	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	response, body = maintenanceRequest(t, server, "GET", maintenanceAPIPath, "")
	if body["active"] != false {
		t.Fatalf("cleared notice still active: %s", response.Body.String())
	}
	response, body = maintenanceRequest(t, server, "POST", denseExecutionPath+"/arm", `{"armed":true}`)
	if body["error"] == "maintenance" || backend.arms != 1 {
		t.Fatalf("arm after clear: %d %s arms=%d", response.Code, response.Body.String(), backend.arms)
	}

	// The file is re-read when it changes: a new notice applies at once.
	if err := os.WriteFile(path, []byte(activeNotice), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, body = maintenanceRequest(t, server, "POST", denseExecutionPath+"/arm", `{"armed":true}`); body["error"] != "maintenance" {
		t.Fatalf("re-set notice not enforced: %v", body)
	}
	if err := os.WriteFile(path, []byte(strings.Replace(activeNotice, `"active":true`, `"active":false`, 1)), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, body = maintenanceRequest(t, server, "GET", maintenanceAPIPath, ""); body["active"] != false {
		t.Fatalf("active:false is not a notice: %v", body)
	}
}

func TestMaintenanceUnreadableNoticeFailsClosed(t *testing.T) {
	for name, notice := range map[string]string{
		"not json":       `{"active":true,`,
		"no started_at":  `{"reason":"deploy"}`,
		"bad eta":        `{"reason":"deploy","started_at":"2026-10-07T19:50:00Z","eta":"in twenty minutes"}`,
		"foreign schema": `{"schema":"other.v9","reason":"deploy","started_at":"2026-10-07T19:50:00Z"}`,
	} {
		server, backend, _, _ := maintenanceTestServer(t, notice)
		_, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, "")
		if body["active"] != true || body["malformed"] != true || body["error"] == "" {
			t.Fatalf("%s: %v", name, body)
		}
		if _, body = maintenanceRequest(t, server, "POST", denseExecutionPath+"/arm", `{"armed":true}`); body["error"] != "maintenance" || backend.arms != 0 {
			t.Fatalf("%s: unreadable notice did not refuse arm: %v", name, body)
		}
		if detail, _ := body["detail"].(string); !strings.Contains(detail, "no expected finish time") && name != "foreign schema" {
			t.Fatalf("%s: detail %q", name, detail)
		}
	}
}

func TestMaintenanceOverrunAndStaleNeverClearTheNotice(t *testing.T) {
	cases := []struct {
		eta            string
		overrun, stale bool
		detail         string
	}{
		{"2026-10-07T19:45:00Z", true, false, "overrunning by 15m00s"},
		{"2026-10-07T17:59:00Z", true, true, "overrunning by 2h01m"},
	}
	for _, c := range cases {
		notice := strings.Replace(activeNotice, "2026-10-07T20:30:00Z", c.eta, 1)
		notice = strings.Replace(notice, "2026-10-07T19:50:00Z", "2026-10-07T17:00:00Z", 1)
		server, _, _, _ := maintenanceTestServer(t, notice)
		_, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, "")
		if body["active"] != true || body["overrun"] != c.overrun || body["stale"] != c.stale {
			t.Fatalf("eta %s: %v", c.eta, body)
		}
		response, refused := maintenanceRequest(t, server, "POST", denseExecutionPath+"/jog/begin", `{}`)
		if refused["error"] != "maintenance" || !strings.Contains(refused["detail"].(string), c.detail) || response.Header().Get("Retry-After") != "30" {
			t.Fatalf("eta %s: %d %v %q", c.eta, response.Code, refused, response.Header().Get("Retry-After"))
		}
	}
	// Without an ETA, staleness counts from the start.
	notice := `{"reason":"manual","started_at":"2026-10-07T17:30:00Z"}`
	server, _, _, _ := maintenanceTestServer(t, notice)
	if _, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, ""); body["active"] != true || body["stale"] != true || body["overrun"] != false {
		t.Fatalf("no-eta notice: %v", body)
	}
}

// The deploy run's progress (steps, gate suite, fraction) reaches clients as
// written; a progress that is not a JSON object is dropped and the notice
// still blocks.
func TestMaintenanceRoutePassesTheProgressThrough(t *testing.T) {
	progress := `{"steps":[{"name":"build+test","label":"Build and test gate","state":"running","estimate_s":900,"elapsed_s":300,"remaining_s":600}],` +
		`"current":1,"total":1,"suite":{"name":"ui_test","index":3,"total":12,"done":["cells"],"running":["ui_test"]},"fraction":0.333,"updated_at":"2026-10-07T20:00:00Z"}`
	notice := strings.TrimSuffix(activeNotice, "}") + `,"progress":` + progress + "}"
	server, _, _, _ := maintenanceTestServer(t, notice)
	_, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, "")
	p, _ := body["progress"].(map[string]any)
	if body["active"] != true || p == nil || p["fraction"] != 0.333 || p["suite"].(map[string]any)["name"] != "ui_test" || body["phase"] != "installing package on the servo host" {
		t.Fatalf("progress not passed through: %v", body)
	}
	server, _, _, _ = maintenanceTestServer(t, strings.TrimSuffix(activeNotice, "}")+`,"progress":"step 3"}`)
	if _, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, ""); body["active"] != true || body["progress"] != nil || body["malformed"] != false {
		t.Fatalf("a non-object progress must be dropped, the notice kept: %v", body)
	}
	server, _, _, _ = maintenanceTestServer(t, activeNotice)
	if _, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, ""); body["progress"] != nil {
		t.Fatalf("a notice without progress serves none: %v", body)
	}
}

func TestMaintenanceWithoutNoticeChangesNothing(t *testing.T) {
	server, backend, _, _ := maintenanceTestServer(t, "")
	_, body := maintenanceRequest(t, server, "GET", maintenanceAPIPath, "")
	if body["active"] != false || body["malformed"] != false {
		t.Fatalf("no file: %v", body)
	}
	if _, body = maintenanceRequest(t, server, "POST", denseExecutionPath+"/arm", `{"armed":true}`); body["error"] == "maintenance" || backend.arms != 1 {
		t.Fatalf("arm without a notice: %v", body)
	}
	// A server built without a source behaves the same.
	plain := NewServer(ServerOptions{})
	response, body := maintenanceRequest(t, plain, "GET", maintenanceAPIPath, "")
	if response.Code != http.StatusOK || body["active"] != false {
		t.Fatalf("nil source: %s", response.Body.String())
	}
}

func TestMaintenanceCommandSetShowClear(t *testing.T) {
	path := filepath.Join(t.TempDir(), "nested", "maintenance.json")
	var stdout, stderr bytes.Buffer
	if code := Run([]string{"maintenance", "set", "--file", path, "--reason", "OLP update", "--eta", "20m", "--phase", "copying", "--run-id", "r1"}, &stdout, &stderr); code != 0 {
		t.Fatalf("set exit %d: %s", code, stderr.String())
	}
	info, err := os.Stat(path)
	if err != nil || info.Mode().Perm() != 0o644 {
		t.Fatalf("notice file must be world-readable: %v %v", info, err)
	}
	notice, err := readMaintenanceNotice(path)
	if err != nil || notice.Schema != maintenanceFileSchema || notice.Active == nil || !*notice.Active || notice.Reason != "OLP update" || notice.RunID != "r1" {
		t.Fatalf("written notice: %+v %v", notice, err)
	}
	started, _ := time.Parse(time.RFC3339, notice.StartedAt)
	eta, _ := time.Parse(time.RFC3339, notice.ETA)
	if d := eta.Sub(started); d < 19*time.Minute || d > 21*time.Minute {
		t.Fatalf("eta %s is not 20 minutes after %s", notice.ETA, notice.StartedAt)
	}
	view := NewMaintenanceSource(path, 0).Current()
	if !view.Active || view.Malformed || view.Phase != "copying" {
		t.Fatalf("server view of the CLI notice: %+v", view)
	}

	// A phase update keeps the original start, so elapsed time stays true.
	time.Sleep(1100 * time.Millisecond)
	if code := Run([]string{"maintenance", "set", "--file", path, "--reason", "OLP update", "--phase", "restarting"}, &stdout, &stderr); code != 0 {
		t.Fatalf("update exit %d: %s", code, stderr.String())
	}
	if updated, _ := readMaintenanceNotice(path); updated.StartedAt != notice.StartedAt || updated.Phase != "restarting" {
		t.Fatalf("update moved the start or lost the phase: %+v", updated)
	}

	if code := Run([]string{"maintenance", "clear", "--file", path, "--run-id", "someone-else"}, &stdout, &stderr); code != 0 {
		t.Fatal(stderr.String())
	}
	if _, err := os.Stat(path); err != nil {
		t.Fatal("clear --run-id of another run removed the notice")
	}
	if code := Run([]string{"maintenance", "clear", "--file", path}, &stdout, &stderr); code != 0 {
		t.Fatal(stderr.String())
	}
	if _, err := os.Stat(path); !os.IsNotExist(err) {
		t.Fatalf("clear left the notice: %v", err)
	}
	stdout.Reset()
	if code := Run([]string{"maintenance", "show", "--file", path}, &stdout, &stderr); code != 0 || !strings.Contains(stdout.String(), `"active": false`) {
		t.Fatalf("show after clear: %d %s", code, stdout.String())
	}
	if code := Run([]string{"maintenance", "set", "--file", path}, &stdout, &stderr); code == 0 {
		t.Fatal("set without --reason must refuse")
	}
}
