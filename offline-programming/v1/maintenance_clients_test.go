package main

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
)

type postureBackend struct{ maintenanceBackend }

func (*postureBackend) Status(context.Context) *denseexec.StatusBody {
	armed := true
	return &denseexec.StatusBody{Backend: "rt_core", Available: true, Armed: &armed,
		Target:  &denseexec.TargetIdentity{CellID: "example-arm-1420", Label: "Rosie 1430 rosie_1420_v1", Host: "example-servo"},
		Session: &denseexec.SessionInfo{State: "loaded", HeartbeatAgeMS: 400}}
}

func clientRequest(server *Server, method, path, remote string, headers map[string]string) {
	request := httptest.NewRequest(method, path, strings.NewReader(`{}`))
	request.RemoteAddr = remote
	for key, value := range headers {
		request.Header.Set(key, value)
	}
	server.Handler().ServeHTTP(httptest.NewRecorder(), request)
}

const macChrome = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/141.0.0.0 Safari/537.36"

// The overlay and the deploy tool name who is connected from what requests
// already carry; nothing identifies a person, and an unnamed client says so.
func TestMaintenanceListsConnectedClientsHonestly(t *testing.T) {
	server := NewServer(ServerOptions{DenseExecution: &postureBackend{}})
	clock := time.Date(2026, 10, 7, 21, 0, 0, 0, time.UTC)
	server.clients.now = func() time.Time { return clock }
	clientRequest(server, "GET", maintenanceAPIPath, "127.0.0.1:50001", map[string]string{"User-Agent": macChrome, "Referer": "http://127.0.0.1:5189/offline-programming/v1/ui/"})
	clientRequest(server, "POST", denseExecutionPath+"/arm", "127.0.0.1:50002", map[string]string{"User-Agent": macChrome, "Origin": "http://127.0.0.1:5189"})
	clientRequest(server, "POST", denseExecutionPath+"/heartbeat", "192.168.1.113:40000", map[string]string{"X-Rosie-Client": "rosie-pendant-v5 (steamdeck-pendant)"})
	clientRequest(server, "GET", "/api/offline-programming/v1/health", "127.0.0.1:50003", map[string]string{"User-Agent": "curl/8.5.0"})
	clientRequest(server, "GET", denseExecutionPath+"/status", "10.0.0.9:1", nil)
	clientRequest(server, "GET", "/offline-programming/v1/ui/", "10.0.0.8:1", map[string]string{"User-Agent": macChrome})

	response := httptest.NewRecorder()
	server.Handler().ServeHTTP(response, httptest.NewRequest("GET", maintenanceAPIPath, nil))
	var view MaintenanceView
	if err := json.Unmarshal(response.Body.Bytes(), &view); err != nil {
		t.Fatal(err)
	}
	byLabel := map[string]MaintenanceClient{}
	for _, client := range view.Clients {
		byLabel[client.Label] = client
	}
	browser := byLabel["Browser (Chrome on macOS) on http://127.0.0.1:5189"]
	if browser.Kind != "browser" || !browser.Loopback || browser.Address != "127.0.0.1" || browser.LastAction != "arm" || browser.LastActionAt != "2026-10-07T21:00:00Z" {
		t.Fatalf("browser (Referer and Origin are the same client): %+v\n%+v", browser, view.Clients)
	}
	pendant := byLabel["rosie-pendant-v5 (steamdeck-pendant)"]
	if pendant.Kind != "pendant" || pendant.Loopback || !pendant.SessionHolder || pendant.Address != "192.168.1.113" {
		t.Fatalf("pendant: %+v", pendant)
	}
	if byLabel["curl (deploy tool or a script)"].Kind != "tool" || byLabel["Unknown client (sends no User-Agent)"].Kind != "unknown" {
		t.Fatalf("tool/unknown: %+v", view.Clients)
	}
	if len(view.Clients) != 5 {
		t.Fatalf("page loads outside the API are not clients; want 5 (and the overlay's own poll), got %+v", view.Clients)
	}
	if view.Machine == nil || view.Machine.Cell != "example-arm-1420" || view.Machine.Armed == nil || !*view.Machine.Armed ||
		view.Machine.SessionState != "loaded" || view.Machine.HeartbeatAgeMS == nil || *view.Machine.HeartbeatAgeMS != 400 {
		t.Fatalf("machine: %+v", view.Machine)
	}

	// The heartbeat holder lapses with the server's 5 s deadman; a client
	// silent for a minute leaves the list.
	clock = clock.Add(6 * time.Second)
	for _, client := range server.clients.list() {
		if client.SessionHolder {
			t.Fatalf("stale heartbeat still holds the session: %+v", client)
		}
	}
	clock = clock.Add(maintenanceClientWindow)
	if clients := server.clients.list(); len(clients) != 0 {
		t.Fatalf("silent clients listed: %+v", clients)
	}
}

func TestMaintenanceClientLabelsAreBounded(t *testing.T) {
	request := httptest.NewRequest("GET", "/api/x", nil)
	request.Header.Set("X-Rosie-Client", "evil\x1b[2Jname"+strings.Repeat("x", 200))
	kind, label, _ := classifyClient(request)
	if kind != "tool" || strings.ContainsRune(label, 0x1b) || len(label) > 84 {
		t.Fatalf("%s %q", kind, label)
	}
	registry := newClientRegistry()
	for i := 0; i < maintenanceClientLimit+10; i++ {
		r := httptest.NewRequest(http.MethodGet, "/api/x", nil)
		r.Header.Set("X-Rosie-Client", "tool-"+strings.Repeat("y", i))
		registry.observe(r)
	}
	if n := len(registry.list()); n != maintenanceClientLimit {
		t.Fatalf("registry grew to %d", n)
	}
}
