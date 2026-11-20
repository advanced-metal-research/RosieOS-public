package main

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"reflect"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
)

type traceHTTPBackend struct {
	jointHTTPBackend
	traceIntent denseexec.TraceIntent
}

// The deployed server wraps native control in the cell picker. Test the complete
// HTTP route through that wrapper, including stale identities and release.
func TestTracePickerHTTPIdentity(t *testing.T) {
	var backends []*traceHTTPBackend
	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"}}},
		Describe: func(_ context.Context, c denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			d := control.Description{}
			d.Backend = "simulation"
			return c, d, nil
		},
		NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) {
			b := &traceHTTPBackend{jointHTTPBackend: jointHTTPBackend{Backend: &identityBackend{}}}
			backends = append(backends, b)
			return b, nil
		},
	})
	t.Cleanup(func() { p.Close(context.Background()) })
	h := NewServer(ServerOptions{DenseExecution: p}).Handler()
	request := func(path, generation, cell string) *httptest.ResponseRecorder {
		t.Helper()
		method := http.MethodPost
		if path == "/trace/state" {
			method = http.MethodGet
		}
		body := `{"target_id":"bench","session_id":"s","revision":3,"input_sequence":4,"trajectory_digest":"digest","reverse":true,"fraction":0.1,"time_scale":4,"first_segment":1,"last_segment":3}`
		r := httptest.NewRequest(method, denseExecutionPath+path, strings.NewReader(body))
		r.Header.Set("X-RT-Target-Generation", generation)
		r.Header.Set("X-RT-Target-Cell", cell)
		w := httptest.NewRecorder()
		h.ServeHTTP(w, r)
		return w
	}
	for i := 0; i < 2; i++ {
		if _, err := p.Select(context.Background(), targets.Selection{CellID: "sim", ModelID: "rosie_1420_v1"}); err != nil {
			t.Fatal(err)
		}
	}
	for _, identity := range [][2]string{{"", ""}, {"1", "sim"}, {"2", "other"}, {"invalid", "sim"}} {
		for _, path := range []string{"/trace/prepare", "/trace/begin", "/trace/update"} {
			w := request(path, identity[0], identity[1])
			if w.Code != 409 || !strings.Contains(w.Body.String(), `"error":"target_changed"`) {
				t.Fatalf("%s: %d %s", path, w.Code, w.Body.String())
			}
		}
	}
	if len(backends[0].calls) != 0 || len(backends[1].calls) != 0 {
		t.Fatal("stale requests reached joint backend")
	}
	for _, path := range []string{"/trace/state", "/trace/prepare", "/trace/begin", "/trace/update", "/trace/end"} {
		w := request(path, "2", "sim")
		if w.Code != 200 {
			t.Fatalf("%s: %d %s", path, w.Code, w.Body.String())
		}
		if path == "/trace/state" {
			var state denseexec.JointJogState
			if err := json.Unmarshal(w.Body.Bytes(), &state); err != nil || state.CadenceMS != 50 {
				t.Fatal(w.Body.String())
			}
		}
	}
	if !reflect.DeepEqual(backends[1].calls, []string{"state", "prepare", "begin", "update", "end"}) || len(backends[0].calls) != 0 {
		t.Fatal("incorrect joint forwarding", backends)
	}
	in := backends[1].traceIntent
	if in.TargetID != "bench" || in.SessionID != "s" || in.Revision != 3 || in.InputSequence != 4 || in.Digest != "digest" || !in.Reverse || in.Fraction != 0.1 || in.TimeScale != 4 || in.FirstSegment != 1 || in.LastSegment != 3 {
		t.Fatalf("Trace intent changed in transit: %+v", in)
	}
	// Release remains available with an old selection identity.
	if w := request("/trace/end", "1", "sim"); w.Code != 200 {
		t.Fatal(w.Code, w.Body.String())
	}
	backends[1].err = &denseexec.Error{Status: 409, Code: "trace_not_ready", Detail: "Trace not ready"}
	if w := request("/trace/begin", "2", "sim"); w.Code != 409 || !strings.Contains(w.Body.String(), "trace_not_ready") {
		t.Fatal(w.Code, w.Body.String())
	}
}

func (b *traceHTTPBackend) Trace(_ context.Context, action string, in denseexec.TraceIntent) (denseexec.JointJogState, error) {
	b.calls = append(b.calls, action)
	b.traceIntent = in
	return denseexec.JointJogState{Active: action != "end", Generation: 17, CadenceMS: 50}, b.err
}
