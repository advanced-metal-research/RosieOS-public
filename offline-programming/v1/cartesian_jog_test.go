package main

import (
	"context"
	"net/http"
	"net/http/httptest"
	"strconv"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
)

type cartesianRouteBackend struct {
	identityBackend
	actions []string
}

func (b *cartesianRouteBackend) CartesianJog(_ context.Context, action string, in denseexec.CartesianJogIntent) (denseexec.JointJogState, error) {
	b.actions = append(b.actions, action)
	return denseexec.JointJogState{Revision: in.Revision, Active: action == "start" || action == "intent"}, nil
}
func (*cartesianRouteBackend) JointJogState() denseexec.JointJogState {
	return denseexec.JointJogState{}
}

func (b *cartesianRouteBackend) CartesianMove(_ context.Context, in denseexec.CartesianMoveIntent) (denseexec.JointMoveState, error) {
	b.actions = append(b.actions, "move")
	return denseexec.JointMoveState{State: "done", Axis: in.Axis, Revision: in.Revision + 1}, nil
}

func TestCartesianRoutesIdentityMutationAndTargetSwitch(t *testing.T) {
	var backends []*cartesianRouteBackend
	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{
			{CellID: "first", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"},
			{CellID: "second", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"},
		}},
		Describe: func(_ context.Context, c denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			d := control.Description{}
			d.Backend = "simulation"
			return c, d, nil
		},
		NewBackend: func(c denseexec.RTCoreConfig, _ denseexec.Config) (denseexec.Backend, error) {
			if c.CartesianModel != "rosie_1420_v1" {
				t.Fatalf("model=%s", c.CartesianModel)
			}
			b := &cartesianRouteBackend{}
			backends = append(backends, b)
			return b, nil
		},
	})
	t.Cleanup(func() { p.Close(context.Background()) })
	if _, err := p.Select(context.Background(), targets.Selection{CellID: "first", ModelID: "rosie_1420_v1"}); err != nil {
		t.Fatal(err)
	}
	identity := p.Status(context.Background()).Target
	h := NewServer(ServerOptions{DenseExecution: p}).Handler()
	for _, action := range []string{"start", "intent", "home", "step", "set-zero"} {
		r := httptest.NewRequest(http.MethodPost, "/api/offline-programming/v1/local-simulator/jog/"+action, strings.NewReader(`{}`))
		w := httptest.NewRecorder()
		h.ServeHTTP(w, r)
		if w.Code != 409 || !strings.Contains(w.Body.String(), "target_changed") {
			t.Fatal("selected machine admitted preview request", action, w.Code)
		}
	}
	send := func(action, generation, cell string) *httptest.ResponseRecorder {
		r := httptest.NewRequest(http.MethodPost, denseExecutionPath+"/cartesian/"+action, strings.NewReader(`{"target_id":"first","session_id":"s","revision":0,"frame":"tool","twist":[0.05,0,0,0,0,0],"fraction":0.1}`))
		r.Header.Set("X-RT-Target-Generation", generation)
		r.Header.Set("X-RT-Target-Cell", cell)
		w := httptest.NewRecorder()
		h.ServeHTTP(w, r)
		return w
	}
	gen := strconv.FormatUint(identity.SelectionGeneration, 10)
	for _, action := range []string{"start", "intent", "stop", "halt", "move"} {
		for _, bad := range [][2]string{{"", "first"}, {"0", "first"}, {gen, "second"}, {gen + "0", "first"}} {
			w := send(action, bad[0], bad[1])
			if w.Code != 409 || !strings.Contains(w.Body.String(), "target_changed") {
				t.Fatal(w.Code, w.Body.String())
			}
		}
	}
	if len(backends[0].actions) != 0 {
		t.Fatal(backends[0].actions)
	}
	if w := send("start", gen, "first"); w.Code != 200 {
		t.Fatal(w.Body.String())
	}
	if w := send("move", gen, "first"); w.Code != 200 || !strings.Contains(w.Body.String(), `"state":"done"`) {
		t.Fatal(w.Code, w.Body.String())
	}
	if _, err := p.Select(context.Background(), targets.Selection{CellID: "second", ModelID: "rosie_1420_v1"}); err != nil {
		t.Fatal(err)
	}
	if backends[0].stops != 1 {
		t.Fatal("previous backend was not stopped")
	}
	if w := send("intent", gen, "first"); w.Code != 409 {
		t.Fatal(w.Code)
	}
	if len(backends[1].actions) != 0 {
		t.Fatal(backends[1].actions)
	}
}
