package main

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"strconv"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
)

type identityBackend struct {
	denseexec.Backend
	arms, homes, plays, stops int
}

func (*identityBackend) Available() bool       { return true }
func (*identityBackend) Close(context.Context) {}
func (*identityBackend) Status(context.Context) *denseexec.StatusBody {
	return &denseexec.StatusBody{Backend: "rt_core", Available: true}
}
func (b *identityBackend) Arm(context.Context, bool) (*denseexec.ArmResult, error) {
	b.arms++
	return &denseexec.ArmResult{Accepted: true}, nil
}
func (b *identityBackend) Home(context.Context) error { b.homes++; return nil }
func (b *identityBackend) GoHome(context.Context) (*denseexec.GoHomeResult, error) {
	b.homes++
	return &denseexec.GoHomeResult{}, nil
}
func (b *identityBackend) Play(context.Context, string) (*denseexec.StatusBody, error) {
	b.plays++
	return b.Status(context.Background()), nil
}
func (b *identityBackend) Stop(context.Context) (*denseexec.StopResult, error) {
	b.stops++
	return &denseexec.StopResult{Stopped: true}, nil
}

// TestReviewTwoClientTargetIdentity preserves the stale SIM badge probe: client B
// selecting REAL cannot turn client A's old ARM or motion into physical authority.
func TestReviewTwoClientTargetIdentity(t *testing.T) {
	ctx := context.Background()
	var backends []*identityBackend
	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{
			{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"},
			{CellID: "real", Models: []string{"rosie_1420_v1"}, RequestedMode: "real", Execution: targets.CellExecution{RTCoreConfig: denseexec.RTCoreConfig{ExpectedBackend: "ethercat"}}},
		}},
		Describe: func(_ context.Context, cfg denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			d := control.Description{}
			d.Backend = cfg.ExpectedBackend
			if d.Backend == "" {
				d.Backend = "simulation"
			}
			return cfg, d, nil
		},
		NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) {
			b := &identityBackend{}
			backends = append(backends, b)
			return b, nil
		},
	})
	h := NewServer(ServerOptions{DenseExecution: p}).Handler()
	request := func(path, body string, target *denseexec.TargetIdentity) *httptest.ResponseRecorder {
		t.Helper()
		r := httptest.NewRequest(http.MethodPost, "/api/offline-programming/v1/"+path, strings.NewReader(body))
		if target != nil {
			r.Header.Set("X-RT-Target-Generation", strconv.FormatUint(target.SelectionGeneration, 10))
			r.Header.Set("X-RT-Target-Cell", target.CellID)
		}
		w := httptest.NewRecorder()
		h.ServeHTTP(w, r)
		return w
	}
	selectCell := func(cell string) {
		t.Helper()
		w := request("targets/select", `{"cell_id":"`+cell+`","model_id":"rosie_1420_v1"}`, nil)
		if w.Code != 200 {
			t.Fatal(w.Code, w.Body.String())
		}
	}
	observe := func() *denseexec.TargetIdentity {
		t.Helper()
		w := httptest.NewRecorder()
		h.ServeHTTP(w, httptest.NewRequest(http.MethodGet, denseExecutionPath+"/status", nil))
		var s denseexec.StatusBody
		if err := json.Unmarshal(w.Body.Bytes(), &s); err != nil || s.Target == nil {
			t.Fatal(w.Body.String(), err)
		}
		return s.Target
	}
	selectCell("sim")
	clientA := observe()
	if clientA.SelectionGeneration != 1 || clientA.Simulation == nil || !*clientA.Simulation {
		t.Fatalf("client A: %+v", clientA)
	}
	selectCell("real")
	clientB := observe()
	if clientB.SelectionGeneration != 2 || clientB.Simulation == nil || *clientB.Simulation {
		t.Fatalf("client B: %+v", clientB)
	}
	wrongCell := *clientB
	wrongCell.CellID = "sim"
	wrongGeneration := *clientB
	wrongGeneration.SelectionGeneration = 1
	for _, identity := range []*denseexec.TargetIdentity{nil, clientA, &wrongCell, &wrongGeneration} {
		for _, path := range []string{"arm", "enable", "reset-fault", "home", "go-home", "play", "load", "jog/start", "jog/update", "jog/move", "jog/joint/move"} {
			w := request("dense-execution/"+path, `{"armed":true,"session_id":"s"}`, identity)
			var failure struct {
				Error string `json:"error"`
			}
			_ = json.Unmarshal(w.Body.Bytes(), &failure)
			if w.Code != 409 || failure.Error != "target_changed" {
				t.Fatalf("%s: %d %s", path, w.Code, w.Body.String())
			}
		}
	}
	if b := backends[1]; b.arms != 0 || b.homes != 0 || b.plays != 0 || b.stops != 0 {
		t.Fatalf("stale requests touched real backend: %+v", b)
	}
	for _, path := range []string{"arm", "home", "go-home", "play"} {
		w := request("dense-execution/"+path, `{"armed":true,"session_id":"s"}`, clientB)
		if w.Code != 200 {
			t.Fatalf("fresh %s: %d %s", path, w.Code, w.Body.String())
		}
	}
	if b := backends[1]; b.arms != 1 || b.homes != 2 || b.plays != 1 {
		t.Fatalf("fresh request counts: %+v", b)
	}
	for _, path := range []string{"arm", "stop"} {
		w := request("dense-execution/"+path, `{"armed":false}`, clientA)
		if w.Code != 200 {
			t.Fatal(w.Code, w.Body.String())
		}
	}
	// Admission remains held until the backend operation finishes; no gap exists
	// between comparing identity and forwarding an energising command.
	if err := p.WithTarget(2, "real", func() {
		_, err := p.Select(ctx, targets.Selection{CellID: "sim", ModelID: "rosie_1420_v1"})
		var failure *denseexec.Error
		if !errors.As(err, &failure) || failure.Code != "cell_switch_busy" {
			t.Fatalf("selection inside admission: %v", err)
		}
	}); err != nil {
		t.Fatal(err)
	}
	selectCell("real")
	if observe().SelectionGeneration != 3 {
		t.Fatal("same-cell reselection reused generation")
	}
	if w := request("dense-execution/arm", `{"armed":true}`, clientB); w.Code != 409 {
		t.Fatal(w.Code, w.Body.String())
	}
}
