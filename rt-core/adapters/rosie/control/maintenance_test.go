//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/json"
	"log"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
	"unicode/utf8"

	"rosieos/rt-core/ipcclient"
)

type maintenanceNative struct{ nativeClient }

func (n *maintenanceNative) Err() error { return nil }
func (n *maintenanceNative) MirrorAcquire(_ context.Context, _ ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	return ipcclient.GrantReceipt{DeadlineNS: deadline}, nil
}

const maintenanceActiveNotice = `{"schema":"rosie.rt-control.maintenance.v1","active":true,"reason":"rt-core full deploy on cell",
"phase":"installing on the servo host","started_at":"2026-10-07T19:50:00Z","eta":"2026-10-07T20:30:00Z","run_id":"realtime-core-deploy-1"}`

func maintenanceController(t *testing.T, notice string) (*Controller, string) {
	t.Helper()
	path := filepath.Join(t.TempDir(), "maintenance.json")
	if notice != "" {
		if err := os.WriteFile(path, []byte(notice), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	binding := Binding{PairID: "cell", Revision: 1, ConfigurationSHA256: "config"}
	c := &Controller{core: &maintenanceNative{}, binding: binding, description: ipcclient.Description{MaxGrantLeaseNS: 3000000000}}
	c.SetMaintenanceFile(path)
	t.Cleanup(func() {
		if c.cancel != nil {
			c.cancel()
		}
	})
	return c, path
}

func maintenanceAcquire(t *testing.T, c *Controller) Response {
	t.Helper()
	body, err := json.Marshal(Request{Schema: RequestSchema, Operation: OperationAcquire, Controller: "olp-rosie_1420_v1", Binding: c.binding})
	if err != nil {
		t.Fatal(err)
	}
	w := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(w, httptest.NewRequest(http.MethodPost, "/v1/control", bytes.NewReader(body)))
	var response Response
	if err := json.Unmarshal(w.Body.Bytes(), &response); err != nil {
		t.Fatalf("%d %s", w.Code, w.Body.String())
	}
	return response
}

// An active notice refuses new authority with the catalogued "busy" reason
// carrying the notice's reason; once it is removed the same request is admitted.
func TestMaintenanceNoticeRefusesAcquireWithBusy(t *testing.T) {
	c, path := maintenanceController(t, maintenanceActiveNotice)
	if response := maintenanceAcquire(t, c); response.Error != ReasonBusy+": rt-core full deploy on cell" || c.grant.Session != "" {
		t.Fatalf("refusal: %q grant=%+v", response.Error, c.grant)
	}
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	var journal bytes.Buffer
	log.SetOutput(&journal)
	t.Cleanup(func() { log.SetOutput(os.Stderr) })
	if response := maintenanceAcquire(t, c); response.Error != "" || c.grant.Session == "" {
		t.Fatalf("Acquire after the notice cleared: %+v", response)
	}
	if line := journal.String(); !strings.Contains(line, `rt-control: grant_acquired generation=1 controller="olp-rosie_1420_v1" principal=local lease_ms=500`) || strings.Contains(line, c.grant.Session) {
		t.Fatalf("holder journal line: %q", line)
	}
}

func TestMaintenanceNoticeRefusesOnlyAuthorityAndMotionStarts(t *testing.T) {
	c, _ := maintenanceController(t, maintenanceActiveNotice)
	for _, op := range []string{OperationAcquire, OperationArm, OperationEnable, OperationHome, OperationRestoreAnchor, OperationRestoreReferenceVerifiedDelta,
		OperationResetFault, OperationJog, OperationBeginJog, OperationBeginPathJog, OperationStartTrajectory, OperationStartProgram, OperationIoArm} {
		if err := c.checkMaintenance(op); err == nil || err.Error() != ReasonBusy+": rt-core full deploy on cell" {
			t.Errorf("%s admitted during maintenance: %v", op, err)
		}
	}
	// A notice never stops, disarms or retires a session by itself, and never
	// prevents an operator from doing so.
	for _, op := range []string{OperationStop, OperationRelease, OperationHalt, OperationEndJog, OperationRenew, OperationIoDisarm,
		OperationDiscardTrajectory, OperationPrepareTrajectory, OperationMarkTelemetry, OperationRecoveryStatus} {
		if err := c.checkMaintenance(op); err != nil {
			t.Errorf("%s refused during maintenance: %v", op, err)
		}
	}
	c.SetMaintenanceFile("")
	if err := c.checkMaintenance(OperationArm); err != nil {
		t.Fatalf("no notice source refused: %v", err)
	}
}

func TestMaintenanceUnreadableNoticeFailsClosedAndOverrunStays(t *testing.T) {
	for name, test := range map[string]struct{ notice, want string }{
		"not json":      {`{"active":`, ReasonBusy},
		"no started_at": {`{"reason":"deploy"}`, ReasonBusy + ": deploy"},
		"foreign":       {`{"schema":"rosie.olp.maintenance.v1","started_at":"2026-10-07T19:50:00Z"}`, ReasonBusy},
		// Long past its ETA: still active. Nothing clears a notice by itself.
		"overrun": {`{"reason":"deploy","started_at":"2026-10-07T10:00:00Z","eta":"2026-10-07T11:00:00Z"}`, ReasonBusy + ": deploy"},
	} {
		c, _ := maintenanceController(t, test.notice)
		if err := c.checkMaintenance(OperationArm); err == nil || err.Error() != test.want {
			t.Errorf("%s: %v", name, err)
		}
	}
	c, path := maintenanceController(t, `{"active":false,"started_at":"2026-10-07T10:00:00Z"}`)
	if err := c.checkMaintenance(OperationArm); err != nil {
		t.Fatalf("active:false refused: %v", err)
	}
	// A change on disk is seen at the next check.
	if err := os.WriteFile(path, []byte(maintenanceActiveNotice+"\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	if err := c.checkMaintenance(OperationArm); err == nil {
		t.Fatal("a notice written after the first check was not seen")
	}
}

func maintenanceRoute(t *testing.T, c *Controller) (Response, MaintenanceObservation, string) {
	t.Helper()
	w := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(w, httptest.NewRequest(http.MethodGet, "/v1/maintenance", nil))
	var response RawResponse
	if err := json.Unmarshal(w.Body.Bytes(), &response); err != nil || w.Code != http.StatusOK || response.Error != "" {
		t.Fatalf("GET /v1/maintenance: %d %s %v", w.Code, w.Body.String(), err)
	}
	var observation MaintenanceObservation
	if err := json.Unmarshal(response.Data, &observation); err != nil {
		t.Fatal(err)
	}
	return Response{Schema: response.Schema, Operation: response.Operation}, observation, w.Body.String()
}

// GET /v1/maintenance is the guard's own state: the notice's display fields
// while it refuses, its holder by name and principal, never the session token.
func TestMaintenanceRouteServesNoticeAndHolderWithoutSession(t *testing.T) {
	c, path := maintenanceController(t, "")
	response, observation, _ := maintenanceRoute(t, c)
	if response.Operation != OperationMaintenance || response.Schema != ResponseSchema || observation.Active || observation.Holder != nil {
		t.Fatalf("no notice, no grant: %+v %+v", response, observation)
	}
	before := time.Now().UTC().Truncate(time.Second)
	if response := maintenanceAcquire(t, c); response.Error != "" {
		t.Fatalf("Acquire: %+v", response)
	}
	if err := os.WriteFile(path, []byte(`{"schema":"rosie.rt-control.maintenance.v1","active":true,"reason":"rt-core full deploy on cell",
"phase":"installing on the servo host","started_at":"2026-10-07T12:50:00-07:00","eta":"2026-10-07T20:30:00Z","set_by":"realtime-core deploy (operator@mac)",
"release":"9a48c367","run_id":"realtime-core-deploy-1"}`), 0o644); err != nil {
		t.Fatal(err)
	}
	_, observation, body := maintenanceRoute(t, c)
	want := MaintenanceObservation{Active: true, Reason: "rt-core full deploy on cell", Phase: "installing on the servo host",
		StartedAt: "2026-10-07T19:50:00Z", ETA: "2026-10-07T20:30:00Z", SetBy: "realtime-core deploy (operator@mac)", Release: "9a48c367"}
	holder := observation.Holder
	observation.Holder = nil
	if observation != want {
		t.Fatalf("notice:\n got %+v\nwant %+v", observation, want)
	}
	if holder == nil || holder.Controller != "olp-rosie_1420_v1" || holder.Remote || holder.Identity != "" || holder.Generation != 1 {
		t.Fatalf("holder: %+v", holder)
	}
	if since, err := time.Parse(time.RFC3339, holder.Since); err != nil || since.Before(before) || since.After(time.Now().Add(time.Second)) {
		t.Fatalf("since %q (acquired after %s): %v", holder.Since, before, err)
	}
	if strings.Contains(body, c.grant.Session) || strings.Contains(body, "session") || strings.Contains(body, "run_id") {
		t.Fatalf("route leaks the session or deploy-private fields: %s", body)
	}
	// The route answers while the guard refuses, and the guard still refuses.
	if err := c.checkMaintenance(OperationArm); err == nil || err.Error() != ReasonBusy+": rt-core full deploy on cell" {
		t.Fatalf("guard: %v", err)
	}
	if err := os.WriteFile(path, []byte(`{"active":`), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, observation, _ := maintenanceRoute(t, c); !observation.Active || !observation.Malformed || observation.Reason != "" {
		t.Fatalf("unreadable notice must read active and malformed: %+v", observation)
	}
	if err := os.WriteFile(path, []byte(`{"active":false,"reason":"done","started_at":"2026-10-07T10:00:00Z"}`), 0o644); err != nil {
		t.Fatal(err)
	}
	if _, observation, _ := maintenanceRoute(t, c); observation.Active || observation.Reason != "" || observation.Holder == nil {
		t.Fatalf("active:false: %+v", observation)
	}
}

func TestMaintenanceRouteNamesRemoteHolderAndClipsText(t *testing.T) {
	c, _ := maintenanceController(t, `{"reason":"`+strings.Repeat("λ", 300)+`","started_at":"2026-10-07T19:50:00Z","eta":"soon"}`)
	if _, err := c.acquire("steamdeck-pendant olp", c.binding, sessionPrincipal{ID: "steamdeck-pendant-olp", Pair: "cell"}); err != nil {
		t.Fatal(err)
	}
	observation := c.Maintenance()
	if observation.Holder == nil || !observation.Holder.Remote || observation.Holder.Identity != "steamdeck-pendant-olp" || observation.Holder.Controller != "steamdeck-pendant olp" {
		t.Fatalf("remote holder: %+v", observation.Holder)
	}
	if !observation.Active || observation.ETA != "" || observation.StartedAt != "2026-10-07T19:50:00Z" {
		t.Fatalf("unparseable eta must read empty, notice still active: %+v", observation)
	}
	if len(observation.Reason) > maintenanceTextMaxBytes || !utf8.ValidString(observation.Reason) || !strings.HasSuffix(observation.Reason, "…") {
		t.Fatalf("clipped reason: %d bytes valid=%t", len(observation.Reason), utf8.ValidString(observation.Reason))
	}
	// The refusal keeps the deploy's whole text: it is the cause, not a display.
	if err := c.checkMaintenance(OperationArm); err == nil || err.Error() != ReasonBusy+": "+strings.Repeat("λ", 300) {
		t.Fatalf("guard: %v", err)
	}
	c.SetMaintenanceFile("")
	if observation := c.Maintenance(); observation.Active || observation.Holder == nil {
		t.Fatalf("no notice source: %+v", observation)
	}
}
