package main

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
)

func TestCellTargetHTTPSelectionRememberedByServer(t *testing.T) {
	p := targets.NewPicker(targets.PickerOptions{Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "bench-relay", Label: "Detached bench", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"}}}, Describe: func(_ context.Context, c denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
		d := control.Description{}
		d.Backend = "simulation"
		return c, d, nil
	}, NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) {
		return &selectionBackend{}, nil
	}})
	h := NewServer(ServerOptions{DenseExecution: p}).Handler()
	post := httptest.NewRecorder()
	h.ServeHTTP(post, httptest.NewRequest(http.MethodPost, "/api/offline-programming/v1/targets/select", strings.NewReader(`{"cell_id":"bench-relay","model_id":"rosie_1420_v1"}`)))
	if post.Code != 200 {
		t.Fatal(post.Code, post.Body.String())
	}
	get := httptest.NewRecorder()
	h.ServeHTTP(get, httptest.NewRequest(http.MethodGet, "/api/offline-programming/v1/targets", nil))
	var result targets.TargetList
	if err := json.Unmarshal(get.Body.Bytes(), &result); err != nil {
		t.Fatal(err)
	}
	if result.Selected == nil || result.Selected.CellID != "bench-relay" || result.Selected.ModelID != "rosie_1420_v1" {
		t.Fatal(get.Body.String())
	}
}

// selectionBackend exercises target identity without acquiring a motion resource.
type selectionBackend struct{ denseexec.Backend }

func (*selectionBackend) Available() bool       { return false }
func (*selectionBackend) Close(context.Context) {}
func (*selectionBackend) Capabilities(context.Context) denseexec.Capabilities {
	return denseexec.Capabilities{Backend: "rt_core"}
}
func (*selectionBackend) Status(context.Context) *denseexec.StatusBody {
	return &denseexec.StatusBody{Backend: "rt_core"}
}
