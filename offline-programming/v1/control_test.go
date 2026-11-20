package main

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/rt-core/sdk/control"
)

// controlBackend is a machine with a grant and an arm state: Arm and Home take
// the grant, Stop and Disarm release it, as the rt-core controller does.
type controlBackend struct {
	maintenanceBackend
	mu                  sync.Mutex
	armed, held         bool
	controllerID        string
	homeEntered, homeGo chan struct{}
}

func (b *controlBackend) Status(context.Context) *denseexec.StatusBody {
	b.mu.Lock()
	defer b.mu.Unlock()
	armed := b.armed
	body := &denseexec.StatusBody{Backend: "rt_core", Available: true, Armed: &armed,
		Target: &denseexec.TargetIdentity{CellID: "example-arm-1420", Label: "Rosie 1430 rosie_1420_v1", Host: "example-servo"}}
	if b.held {
		body.Session = &denseexec.SessionInfo{ID: "ds-1", Kind: "authority", State: "idle", Lease: denseexec.LeaseInfo{Held: true}}
	}
	return body
}

func (b *controlBackend) Arm(ctx context.Context, armed bool) (*denseexec.ArmResult, error) {
	b.maintenanceBackend.Arm(ctx, armed)
	b.mu.Lock()
	b.armed, b.held = armed, armed
	b.mu.Unlock()
	return &denseexec.ArmResult{OK: true, Accepted: true}, nil
}

func (b *controlBackend) Home(ctx context.Context) error {
	if b.homeEntered != nil {
		close(b.homeEntered)
		<-b.homeGo
	}
	b.mu.Lock()
	b.held = true
	b.mu.Unlock()
	return b.maintenanceBackend.Home(ctx)
}

func (b *controlBackend) Stop(ctx context.Context) (*denseexec.StopResult, error) {
	b.mu.Lock()
	b.armed, b.held = false, false
	b.mu.Unlock()
	return b.maintenanceBackend.Stop(ctx)
}

func (b *controlBackend) ControllerID() string { return b.controllerID }

// A browser tab and a Steam Deck pendant, each with its own client id.
var (
	browserTab = map[string]string{"User-Agent": macChrome, "Origin": "http://127.0.0.1:5189", clientIDHeader: "tab-6f1c2a9e"}
	deckClient = map[string]string{"X-Rosie-Client": "rosie-pendant-v5 (steamdeck-pendant)", clientIDHeader: "deck-0b7e44d1"}
)

func controlTestServer(t *testing.T) (*Server, *controlBackend, *time.Time) {
	t.Helper()
	backend := &controlBackend{controllerID: "offline-programming:server-olp001"}
	server := NewServer(ServerOptions{DenseExecution: backend})
	clock := time.Date(2026, 10, 7, 22, 0, 0, 0, time.UTC)
	server.control.now = func() time.Time { return clock }
	server.clients.now = func() time.Time { return clock }
	return server, backend, &clock
}

func controlRequest(t *testing.T, server *Server, client map[string]string, method, path, body string) (*httptest.ResponseRecorder, map[string]any) {
	t.Helper()
	request := httptest.NewRequest(method, path, strings.NewReader(body))
	request.RemoteAddr = "127.0.0.1:50000"
	if client["X-Rosie-Client"] != "" {
		request.RemoteAddr = "100.64.0.15:41000"
	}
	for key, value := range client {
		request.Header.Set(key, value)
	}
	response := httptest.NewRecorder()
	server.Handler().ServeHTTP(response, request)
	var decoded map[string]any
	_ = json.Unmarshal(response.Body.Bytes(), &decoded)
	return response, decoded
}

func controlOf(t *testing.T, body map[string]any) ControlView {
	t.Helper()
	raw, _ := json.Marshal(body["control"])
	var view ControlView
	if err := json.Unmarshal(raw, &view); err != nil {
		t.Fatal(err)
	}
	return view
}

// Requests a second controller must not get through while the first uses
// the robot: everything that takes control or starts motion, Connect included
// (a select Stops the selected backend).
var controlTakingRequests = []struct{ method, path, body string }{
	{"POST", denseExecutionPath + "/arm", `{"armed":true}`},
	{"POST", denseExecutionPath + "/jog/begin", `{}`},
	{"POST", denseExecutionPath + "/cartesian/start", `{}`},
	{"POST", denseExecutionPath + "/cartesian/move", `{}`},
	{"POST", denseExecutionPath + "/move", `{}`},
	{"POST", denseExecutionPath + "/load", `{}`},
	{"POST", denseExecutionPath + "/play", `{"session_id":"s"}`},
	{"POST", denseExecutionPath + "/home", `{}`},
	{"POST", denseExecutionPath + "/reset-fault", `{}`},
	{"POST", denseExecutionPath + "/recover-to-zero", `{"fraction":0.1}`},
	{"POST", denseExecutionPath + "/run-program", `{"fraction":0.1}`},
	{"POST", denseExecutionPath + "/target", `{"cell":"example-arm-1420"}`},
	{"POST", denseExecutionPath + "/some-future-motion-route", `{}`},
	{"POST", "/api/offline-programming/v1/targets/select", `{"cell_id":"c","model_id":"m"}`},
	{"POST", executionPath + "/start", `{}`},
}

func TestSecondControllerIsRefusedNamingTheHolderAndStopAlwaysPasses(t *testing.T) {
	server, backend, _ := controlTestServer(t)
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK {
		t.Fatalf("first controller's arm: %d %s", response.Code, response.Body.String())
	}
	for _, request := range controlTakingRequests {
		response, body := controlRequest(t, server, deckClient, request.method, request.path, request.body)
		if response.Code != http.StatusConflict || body["error"] != controlRefusal {
			t.Fatalf("%s %s from the Deck while the browser is armed: %d %s", request.method, request.path, response.Code, response.Body.String())
		}
		view := controlOf(t, body)
		if view.State != "other" || view.Owner == nil || view.Owner.Label != "Browser (Chrome on macOS) on http://127.0.0.1:5189" ||
			view.Owner.Since != "2026-10-07T22:00:00Z" || !view.Armed || view.TakeControl || !view.StopAvailable {
			t.Fatalf("%s refusal must name the armed holder and offer only Stop: %+v", request.path, view)
		}
		detail, _ := body["detail"].(string)
		for _, want := range []string{"Browser (Chrome on macOS)", "ARMED", "Stop always works"} {
			if !strings.Contains(detail, want) {
				t.Fatalf("%s detail %q lacks %q", request.path, detail, want)
			}
		}
	}
	if backend.arms != 1 || backend.homes != 0 || backend.plays != 0 || backend.stops != 0 {
		t.Fatalf("a refused request reached the controller: %+v", backend.identityBackend)
	}
	// The holder keeps working.
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/play", `{"session_id":"s"}`); response.Code != http.StatusOK {
		t.Fatalf("the holder's own play: %d %s", response.Code, response.Body.String())
	}
	// Protective requests pass from anyone: the heartbeat, ending a hold,
	// Pause, Disarm, Disconnect and Stop.
	for _, request := range []struct{ path, body string }{
		{"/heartbeat", `{"session_id":"s"}`}, {"/jog/end", `{}`}, {"/cartesian/halt", `{}`}, {"/pause", `{"session_id":"s"}`},
	} {
		if _, body := controlRequest(t, server, deckClient, "POST", denseExecutionPath+request.path, request.body); body["error"] == controlRefusal {
			t.Fatalf("%s refused for another controller", request.path)
		}
	}
	if response, _ := controlRequest(t, server, deckClient, "POST", denseExecutionPath+"/stop", `{}`); response.Code != http.StatusOK || backend.stops != 1 {
		t.Fatalf("Stop from the other controller: %d %s (stops %d)", response.Code, response.Body.String(), backend.stops)
	}
	// Stop released the grant: the robot is free and the Deck may arm; the
	// browser is now the one refused.
	if response, _ := controlRequest(t, server, deckClient, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK {
		t.Fatalf("arm after Stop freed the robot: %d %s", response.Code, response.Body.String())
	}
	response, body := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":true}`)
	if view := controlOf(t, body); response.Code != http.StatusConflict || view.Owner == nil || view.Owner.Label != "rosie-pendant-v5 (steamdeck-pendant)" || view.Owner.Kind != "pendant" {
		t.Fatalf("browser after the Deck armed: %d %+v", response.Code, view)
	}
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":false}`); response.Code != http.StatusOK || backend.armed {
		t.Fatalf("Disarm from the other controller: %d armed=%v", response.Code, backend.armed)
	}
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/home", `{}`); response.Code != http.StatusOK {
		t.Fatalf("home after Disarm freed the robot: %d %s", response.Code, response.Body.String())
	}
}

func TestTakeControlOnlyFromADisarmedIdleHolder(t *testing.T) {
	server, backend, clock := controlTestServer(t)
	// Home leaves the grant held and the machine disarmed.
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/home", `{}`); response.Code != http.StatusOK {
		t.Fatal(response.Code, response.Body.String())
	}
	*clock = clock.Add(10 * time.Second)
	response, body := controlRequest(t, server, deckClient, "POST", controlAPIPath+"/take", `{}`)
	view := controlOf(t, body)
	if response.Code != http.StatusConflict || view.TakeControl || !view.LeaseHeld || view.Armed || !strings.Contains(view.TakeControlNote, "20s more") {
		t.Fatalf("take 10 s after the holder's Home: %d %+v", response.Code, view)
	}
	*clock = clock.Add(21 * time.Second)
	_, body = controlRequest(t, server, deckClient, "GET", maintenanceAPIPath, "")
	if view := controlOf(t, body); view.State != "other" || !view.TakeControl {
		t.Fatalf("the poll must offer Take control after 31 s idle: %+v", view)
	}
	response, body = controlRequest(t, server, deckClient, "POST", controlAPIPath+"/take", `{}`)
	if response.Code != http.StatusOK || body["state"] != "yours" {
		t.Fatalf("take from a disarmed idle holder: %d %s", response.Code, response.Body.String())
	}
	if backend.stops != 0 || backend.arms != 0 {
		t.Fatalf("Take control moved or stopped something: %+v", backend.identityBackend)
	}
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusConflict {
		t.Fatalf("the replaced holder must now be refused: %d", response.Code)
	}
	// Armed: never taken, however long idle.
	if response, _ := controlRequest(t, server, deckClient, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK {
		t.Fatal(response.Code, response.Body.String())
	}
	*clock = clock.Add(time.Hour)
	response, body = controlRequest(t, server, browserTab, "POST", controlAPIPath+"/take", `{}`)
	if view := controlOf(t, body); response.Code != http.StatusConflict || view.TakeControl || !strings.Contains(view.TakeControlNote, "never taken from an armed controller") {
		t.Fatalf("take from an armed holder: %d %+v", response.Code, view)
	}
	if !backend.armed || backend.stops != 0 {
		t.Fatal("a refused take must leave the armed holder untouched")
	}
}

func TestARunningRequestIsInUseBeforeTheGrantIs(t *testing.T) {
	server, backend, _ := controlTestServer(t)
	backend.homeEntered, backend.homeGo = make(chan struct{}), make(chan struct{})
	done := make(chan *httptest.ResponseRecorder)
	go func() {
		response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/home", `{}`)
		done <- response
	}()
	<-backend.homeEntered
	response, body := controlRequest(t, server, deckClient, "POST", denseExecutionPath+"/arm", `{"armed":true}`)
	if view := controlOf(t, body); response.Code != http.StatusConflict || view.Owner == nil || view.Owner.Running != "home" || view.TakeControl {
		t.Fatalf("arm while another client's Home runs: %d %+v", response.Code, view)
	}
	if response, _ := controlRequest(t, server, deckClient, "POST", denseExecutionPath+"/stop", `{}`); response.Code != http.StatusOK {
		t.Fatal("Stop must pass while another client's request runs")
	}
	close(backend.homeGo)
	if response := <-done; response.Code != http.StatusOK {
		t.Fatal(response.Code, response.Body.String())
	}
}

func TestAGrantHeldByAnotherServerIsNamedAndNotStoppable(t *testing.T) {
	server, backend, _ := controlTestServer(t)
	rtc := &fakeCellRTControl{observation: control.MaintenanceObservation{Holder: &control.MaintenanceHolder{
		Controller: "offline-programming:server-deck01", Remote: true, Identity: "steamdeck-pendant-olp", Generation: 7, Since: "2026-10-07T21:58:00Z"}}}
	server.cellMaintenance = newCellMaintenance(func() (cellRTControl, bool) {
		return cellRTControl{CellID: "example-arm-1420", Host: "example-servo"}, true
	})
	server.cellMaintenance.dial = func(denseexec.RTCoreConfig) (cellMaintenanceClient, error) { return rtc, nil }
	response, body := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":true}`)
	view := controlOf(t, body)
	if response.Code != http.StatusConflict || view.State != "other_server" || view.StopAvailable || view.TakeControl ||
		view.ServoHolder == nil || view.ServoHolder.Identity != "steamdeck-pendant-olp" || !strings.Contains(view.Detail, "hardware stop") {
		t.Fatalf("arm while another server holds the cell: %d %+v", response.Code, view)
	}
	if backend.arms != 0 {
		t.Fatal("refused arm reached the controller")
	}
	// Connect only observes the cell: it is not refused for the holder.
	if _, body := controlRequest(t, server, browserTab, "POST", "/api/offline-programming/v1/targets/select", `{"cell_id":"c","model_id":"m"}`); body["error"] == controlRefusal {
		t.Fatal("select refused for another server's grant")
	}
	// This server's own grant is not "another controller".
	server.cellMaintenance.Close()
	rtc.observation.Holder.Controller = backend.controllerID
	if response, _ := controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":true}`); response.Code != http.StatusOK {
		t.Fatalf("own grant refused: %d %s", response.Code, response.Body.String())
	}
}

func TestControlViewsMarkTheControllerAndTheRequester(t *testing.T) {
	server, _, _ := controlTestServer(t)
	_, body := controlRequest(t, server, deckClient, "GET", controlAPIPath, "")
	if body["state"] != "free" || body["named"] != true || body["stop_available"] != true {
		t.Fatalf("free view: %v", body)
	}
	controlRequest(t, server, browserTab, "POST", denseExecutionPath+"/arm", `{"armed":true}`)
	_, mine := controlRequest(t, server, browserTab, "GET", controlAPIPath, "")
	if mine["state"] != "yours" {
		t.Fatalf("holder's view: %v", mine)
	}
	response, _ := controlRequest(t, server, deckClient, "GET", maintenanceAPIPath, "")
	var view MaintenanceView
	if err := json.Unmarshal(response.Body.Bytes(), &view); err != nil || view.Control == nil || view.Control.State != "other" {
		t.Fatalf("maintenance view carries no control: %v %s", err, response.Body.String())
	}
	marks := map[string]MaintenanceClient{}
	for _, client := range view.Clients {
		marks[client.Kind] = client
	}
	if !marks["browser"].Controller || marks["browser"].You || !marks["pendant"].You || marks["pendant"].Controller {
		t.Fatalf("clients list marks: %+v", view.Clients)
	}
}

// Clients that send no id (today's UI and pendant) are told apart by what
// their requests carry, and one such client is never refused against itself.
func TestClientsWithoutAnIdKeepTheirOwnControl(t *testing.T) {
	server, _, _ := controlTestServer(t)
	oldBrowser := map[string]string{"User-Agent": macChrome, "Origin": "http://127.0.0.1:5189"}
	oldDeck := map[string]string{"X-Rosie-Client": "rosie-pendant-v5 (steamdeck-pendant)"}
	for _, path := range []string{"/arm", "/jog/begin", "/move", "/home"} {
		if response, body := controlRequest(t, server, oldBrowser, "POST", denseExecutionPath+path, `{"armed":true}`); body["error"] == controlRefusal {
			t.Fatalf("old browser's own %s refused: %s", path, response.Body.String())
		}
	}
	if response, _ := controlRequest(t, server, oldDeck, "POST", denseExecutionPath+"/jog/begin", `{}`); response.Code != http.StatusConflict {
		t.Fatalf("old Deck jog while the old browser is armed: %d", response.Code)
	}
	if !validClientID("tab-6f1c2a9e") || validClientID("short") || validClientID("has space in it") || validClientID(strings.Repeat("x", 65)) {
		t.Fatal("client id validation")
	}
}
