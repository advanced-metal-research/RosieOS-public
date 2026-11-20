package main

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/simulator"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
)

type facadeBackend struct {
	identityBackend
	failStop bool
	closed   int
}

func (*facadeBackend) Capabilities(context.Context) denseexec.Capabilities {
	return denseexec.Capabilities{Available: true, Backend: "rt_core"}
}
func (b *facadeBackend) Stop(ctx context.Context) (*denseexec.StopResult, error) {
	if b.failStop {
		return &denseexec.StopResult{Stopped: false}, nil
	}
	return b.identityBackend.Stop(ctx)
}
func (b *facadeBackend) Close(context.Context) { b.closed++ }

// Both selection contracts share authenticated discovery and generation
// admission; listing and registration cannot create motion authority.
func TestDenseExecutionCellsFacadeSharesNativeAuthority(t *testing.T) {
	for _, selection := range []struct{ route, body string }{
		{"dense-execution/target", `{"cell":"sim"}`},
		{"targets/select", `{"cell_id":"sim","model_id":"rosie_1420_v1"}`},
	} {
		t.Run(selection.route, func(t *testing.T) {
			backend := &facadeBackend{}
			describes := 0
			mismatch := ""
			picker := targets.NewPicker(targets.PickerOptions{
				Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"}}},
				Describe: func(_ context.Context, cfg denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
					describes++
					if mismatch != "" {
						return cfg, control.Description{}, &denseexec.Error{Status: 409, Code: mismatch, Detail: mismatch}
					}
					description := control.Description{}
					description.Backend = "simulation"
					return cfg, description, nil
				},
				NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) { return backend, nil },
			})
			h := NewServer(ServerOptions{DenseExecution: picker}).Handler()
			request := func(method, path, body string) *httptest.ResponseRecorder {
				w := httptest.NewRecorder()
				h.ServeHTTP(w, httptest.NewRequest(method, "/api/offline-programming/v1/"+path, strings.NewReader(body)))
				return w
			}
			for _, code := range []string{"rt_core_identity_mismatch", "cell_configuration_mismatch"} {
				mismatch = code
				w := request("POST", selection.route, selection.body)
				if w.Code != 409 || !strings.Contains(w.Body.String(), code) || picker.SelectedCell() != "" {
					t.Fatalf("selection bypass: %d %s", w.Code, w.Body.String())
				}
			}
			mismatch = ""
			if w := request("POST", selection.route, selection.body); w.Code != 200 {
				t.Fatal(w.Code, w.Body.String())
			}
			if backend.arms != 0 || backend.plays != 0 {
				t.Fatal("connect armed or played")
			}
			before := describes
			if w := request("GET", "dense-execution/cells", ""); w.Code != 200 || !strings.Contains(w.Body.String(), `"active_cell":"sim"`) {
				t.Fatal(w.Code, w.Body.String())
			}
			if describes != before+1 {
				t.Fatalf("refresh describes=%d want=%d", describes, before+1)
			}
			for _, input := range []string{`{"id":"unknown","address":"https://unknown:8443"}`, `{"id":"sim","address":"https://redirect:8443"}`} {
				if w := request("POST", "dense-execution/cells", input); w.Code != 409 || !strings.Contains(w.Body.String(), "not a commissioned cell") {
					t.Fatal(w.Code, w.Body.String())
				}
			}
			if w := request("DELETE", "dense-execution/cells/sim", ""); w.Code != 409 {
				t.Fatal(w.Code, w.Body.String())
			}
			for _, path := range []string{"arm", "home", "go-home", "load", "play", "move", "jog/begin", "jog/update", "cartesian/start", "cartesian/intent", "cartesian/move", "cartesian/stop", "cartesian/halt"} {
				w := request("POST", "dense-execution/"+path, `{"armed":true}`)
				if w.Code != 409 || !strings.Contains(w.Body.String(), "target_changed") {
					t.Fatalf("unfenced %s: %d %s", path, w.Code, w.Body.String())
				}
			}
			backend.failStop = true
			if w := request("POST", "dense-execution/target", `{"cell":""}`); w.Code != 409 || picker.SelectedCell() != "sim" || backend.closed != 0 {
				t.Fatal("failed Stop lost selection", w.Code, w.Body.String())
			}
			backend.failStop = false
			if w := request("POST", "dense-execution/target", `{"cell":""}`); w.Code != 200 || picker.SelectedCell() != "" || backend.closed != 1 || backend.stops != 1 {
				t.Fatal("disconnect did not retire controller", w.Code, w.Body.String())
			}
			if w := request("POST", "dense-execution/arm", `{"armed":true}`); w.Code != 409 || !strings.Contains(w.Body.String(), "target_changed") {
				t.Fatal("offline motion admitted", w.Code, w.Body.String())
			}
		})
	}
}

func TestReviewDisabledJogHasTypedReason(t *testing.T) {
	response := httptest.NewRecorder()
	NewServer(ServerOptions{}).Handler().ServeHTTP(response, httptest.NewRequest(http.MethodPost, "/api/offline-programming/v1/local-simulator/jog/start", nil))
	var body map[string]any
	if err := json.Unmarshal(response.Body.Bytes(), &body); err != nil {
		t.Fatal(err)
	}
	if response.Code != 503 || body["code"] != "local_simulation_jog_unavailable" {
		t.Fatalf("disabled jog reason: status=%d body=%s", response.Code, response.Body.String())
	}
}

// Unconfigured (nil manager, and a manager missing one of its three
// settings) is the normal dev case: capabilities and status answer 200
// with available:false, every action answers 503 dense_target_unavailable
// in the {"error","detail"} shape.
func TestDenseExecutionUnconfiguredFailsClosed(t *testing.T) {
	for name, server := range map[string]*Server{
		"nil": NewServer(ServerOptions{}),
	} {
		handler := server.Handler()
		for _, path := range []string{"capabilities", "status"} {
			response := httptest.NewRecorder()
			handler.ServeHTTP(response, httptest.NewRequest(http.MethodGet,
				"/api/offline-programming/v1/dense-execution/"+path, nil))
			if response.Code != http.StatusOK {
				t.Fatalf("%s %s status=%d body=%s", name, path, response.Code, response.Body.String())
			}
			var body map[string]any
			if err := json.Unmarshal(response.Body.Bytes(), &body); err != nil {
				t.Fatalf("%s %s: %v", name, path, err)
			}
			if body["available"] != false {
				t.Fatalf("%s %s must report available:false: %s", name, path, response.Body.String())
			}
		}
		// The target's machine_planning_calibration.json is a read, but of a target: none, no answer.
		cell := httptest.NewRecorder()
		handler.ServeHTTP(cell, httptest.NewRequest(http.MethodGet,
			"/api/offline-programming/v1/dense-execution/cell", nil))
		if cell.Code != http.StatusServiceUnavailable || !strings.Contains(cell.Body.String(), "dense_target_unavailable") {
			t.Fatalf("%s cell status=%d body=%s", name, cell.Code, cell.Body.String())
		}
		for _, action := range []string{"load", "play", "pause", "stop", "arm", "heartbeat", "go-home"} {
			response := httptest.NewRecorder()
			handler.ServeHTTP(response, httptest.NewRequest(http.MethodPost,
				"/api/offline-programming/v1/dense-execution/"+action,
				strings.NewReader(`{"session_id":"x","armed":true}`)))
			if response.Code != http.StatusServiceUnavailable {
				t.Fatalf("%s %s status=%d body=%s", name, action, response.Code, response.Body.String())
			}
			var body struct {
				Error  string `json:"error"`
				Detail string `json:"detail"`
			}
			if err := json.Unmarshal(response.Body.Bytes(), &body); err != nil {
				t.Fatalf("%s %s: %v", name, action, err)
			}
			if body.Error != "dense_target_unavailable" || body.Detail == "" {
				t.Fatalf("%s %s body=%s", name, action, response.Body.String())
			}
		}
	}
}

func TestRTCoreBackendSelectionFailsClosed(t *testing.T) {
	for _, input := range []struct {
		backend, config string
		connected       bool
	}{
		{"unknown", "", false}, {"rt_core", "", false}, {"rt_core", "config.json", true}, {"legacy_v4", "config.json", false},
	} {
		if err := validateExecutionBackend(input.backend, input.config, input.connected); err == nil {
			t.Fatalf("accepted %+v", input)
		}
	}
	if err := validateExecutionBackend("legacy_v4", "", false); err == nil || !strings.Contains(err.Error(), "backend_retired") {
		t.Fatalf("legacy selection: %v", err)
	}
	if err := validateExecutionBackend("rt_core", "config.json", false); err != nil {
		t.Fatal(err)
	}
}

// Explicit retired jog selections fail before UI/configuration reads or startup.
func TestServeRetiredJogBackendFailsBeforeStartup(t *testing.T) {
	for _, backend := range []string{"legacy_v4", "unknown"} {
		t.Run(backend, func(t *testing.T) {
			t.Setenv("OFFLINE_PROGRAMMING_JOG_BACKEND", backend)
			err := runServe([]string{"--execution-backend", "rt_core", "--rt-core-config", "/absent/native.json", "--dist", "/absent/ui", "--enable-local-simulator", "--listen", "127.0.0.1:0"}, io.Discard, io.Discard)
			var retired *simulator.JogBackendError
			if !errors.As(err, &retired) || retired.Backend != backend || !strings.Contains(err.Error(), "backend_retired") {
				t.Fatalf("startup refusal: %v", err)
			}
		})
	}
}

// unattendedBackend reports its cell manifest's unattended_loop_allowed.
type unattendedBackend struct {
	facadeBackend
	allowed bool
}

func (b *unattendedBackend) UnattendedLoopAllowed() bool { return b.allowed }

// Owner decision 2026-11-13: the pendant's program arm (latched with no held
// L2) is admitted only on a cell whose manifest sets
// cell_deploy.olp.unattended_loop_allowed. A hand-made {"program": true}
// cannot program-arm any other cell; an ordinary arm and every disarm pass.
func TestProgramArmNeedsAnUnattendedLoopCell(t *testing.T) {
	for _, allowed := range []bool{false, true} {
		backend := &unattendedBackend{allowed: allowed}
		h := NewServer(ServerOptions{DenseExecution: backend}).Handler()
		post := func(body string) *httptest.ResponseRecorder {
			w := httptest.NewRecorder()
			h.ServeHTTP(w, httptest.NewRequest("POST", "/api/offline-programming/v1/dense-execution/arm", strings.NewReader(body)))
			return w
		}
		w := post(`{"armed":true,"program":true}`)
		if allowed && (w.Code != 200 || backend.arms != 1) {
			t.Fatalf("allowed cell refused program arm: %d %s arms=%d", w.Code, w.Body.String(), backend.arms)
		}
		if !allowed && (w.Code != 409 || !strings.Contains(w.Body.String(), "program_arm_not_allowed") || backend.arms != 0) {
			t.Fatalf("program arm reached a cell without unattended loops: %d %s arms=%d", w.Code, w.Body.String(), backend.arms)
		}
		before := backend.arms
		for _, body := range []string{`{"armed":true}`, `{"armed":false,"program":true}`, `{"armed":false}`} {
			if w := post(body); w.Code != 200 {
				t.Fatalf("allowed=%t %s: %d %s", allowed, body, w.Code, w.Body.String())
			}
		}
		if backend.arms != before+3 {
			t.Fatalf("allowed=%t: arms=%d want %d", allowed, backend.arms, before+3)
		}
	}
	// A backend that cannot say fails closed.
	backend := &facadeBackend{}
	h := NewServer(ServerOptions{DenseExecution: backend}).Handler()
	w := httptest.NewRecorder()
	h.ServeHTTP(w, httptest.NewRequest("POST", "/api/offline-programming/v1/dense-execution/arm", strings.NewReader(`{"armed":true,"program":true}`)))
	if w.Code != 409 || backend.arms != 0 {
		t.Fatalf("unknown cell admitted program arm: %d %s", w.Code, w.Body.String())
	}
}

// The picker reports the selected backend's flag, and nothing once released.
func TestPickerReportsTheSelectedCellsUnattendedLoopFlag(t *testing.T) {
	backend := &unattendedBackend{allowed: true}
	picker := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"}}},
		Describe: func(_ context.Context, cfg denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			description := control.Description{}
			description.Backend = "simulation"
			return cfg, description, nil
		},
		NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) { return backend, nil },
	})
	if picker.UnattendedLoopAllowed() {
		t.Fatal("no selection reported unattended loops")
	}
	h := NewServer(ServerOptions{DenseExecution: picker}).Handler()
	request := func(path, body string) *httptest.ResponseRecorder {
		w := httptest.NewRecorder()
		h.ServeHTTP(w, httptest.NewRequest("POST", "/api/offline-programming/v1/"+path, strings.NewReader(body)))
		return w
	}
	if w := request("targets/select", `{"cell_id":"sim","model_id":"rosie_1420_v1"}`); w.Code != 200 {
		t.Fatal(w.Code, w.Body.String())
	}
	if !picker.UnattendedLoopAllowed() {
		t.Fatal("selected cell's flag lost")
	}
	if w := request("dense-execution/target", `{"cell":""}`); w.Code != 200 || picker.UnattendedLoopAllowed() {
		t.Fatal("released selection kept the flag", w.Code, w.Body.String())
	}
}
