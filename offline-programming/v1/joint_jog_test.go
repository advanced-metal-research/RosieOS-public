package main

import (
	"context"
	"encoding/json"
	"math"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"syscall"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/targets"
	"rosieos/rt-core/sdk/control"
	"rosieos/rt-core/simfixture"
)

type jointHTTPBackend struct {
	denseexec.Backend
	action string
	intent denseexec.JointJogIntent
	err    error
	calls  []string
}

// TestJointPickerHTTPIdentity exercises the actual joint routes through selection;
// stale, missing or malformed identities must never reach either session.
func TestJointPickerHTTPIdentity(t *testing.T) {
	var backends []*jointHTTPBackend
	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"}}},
		Describe: func(_ context.Context, c denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			d := control.Description{}
			d.Backend = "simulation"
			return c, d, nil
		},
		NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) {
			b := &jointHTTPBackend{Backend: &identityBackend{}}
			backends = append(backends, b)
			return b, nil
		},
	})
	t.Cleanup(func() { p.Close(context.Background()) })
	h := NewServer(ServerOptions{DenseExecution: p}).Handler()
	request := func(path, generation, cell string) *httptest.ResponseRecorder {
		t.Helper()
		method := http.MethodPost
		if path == "/jog/state" {
			method = http.MethodGet
		}
		// Jog and move bodies have distinct strict schemas.
		body := `{"target_id":"bench","session_id":"s","revision":3,"axis":0,"direction":1,"fraction":0.1}`
		if path == "/move" {
			body = `{"target_id":"bench","session_id":"s","revision":3,"axis":0,"degrees":30,"fraction":0.1}`
		}
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
		for _, path := range []string{"/jog/begin", "/jog/update", "/move"} {
			w := request(path, identity[0], identity[1])
			if w.Code != 409 || !strings.Contains(w.Body.String(), `"error":"target_changed"`) {
				t.Fatalf("%s: %d %s", path, w.Code, w.Body.String())
			}
		}
	}
	if len(backends[0].calls) != 0 || len(backends[1].calls) != 0 {
		t.Fatal("stale requests reached joint backend")
	}
	for _, path := range []string{"/jog/state", "/jog/begin", "/jog/update", "/jog/end", "/move"} {
		w := request(path, "2", "sim")
		if w.Code != 200 {
			t.Fatalf("%s: %d %s", path, w.Code, w.Body.String())
		}
		if path == "/jog/state" {
			var state denseexec.JointJogState
			if err := json.Unmarshal(w.Body.Bytes(), &state); err != nil || state.CadenceMS != 50 {
				t.Fatal(w.Body.String())
			}
		}
	}
	if !reflect.DeepEqual(backends[1].calls, []string{"state", "begin", "update", "end", "move"}) || len(backends[0].calls) != 0 {
		t.Fatal("incorrect joint forwarding", backends)
	}
	// Release remains available with an old selection identity.
	if w := request("/jog/end", "1", "sim"); w.Code != 200 {
		t.Fatal(w.Code, w.Body.String())
	}
	backends[1].err = &denseexec.Error{Status: 409, Code: "jog_not_ready", Detail: "Jog not ready"}
	if w := request("/jog/begin", "2", "sim"); w.Code != 409 || !strings.Contains(w.Body.String(), "jog_not_ready") {
		t.Fatal(w.Code, w.Body.String())
	}
}

// TestJointPickerUnsupportedFailsClosed prevents optional-interface forwarding
// from manufacturing an active lane when selection or joint support is absent.
func TestJointPickerUnsupportedFailsClosed(t *testing.T) {
	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation"}}},
		Describe: func(_ context.Context, c denseexec.RTCoreConfig) (denseexec.RTCoreConfig, control.Description, error) {
			d := control.Description{}
			d.Backend = "simulation"
			return c, d, nil
		},
		NewBackend: func(denseexec.RTCoreConfig, denseexec.Config) (denseexec.Backend, error) {
			return &identityBackend{}, nil
		},
	})
	for _, phase := range []string{"unselected", "unsupported", "closed"} {
		if phase == "unsupported" {
			if _, err := p.Select(context.Background(), targets.Selection{CellID: "sim", ModelID: "rosie_1420_v1"}); err != nil {
				t.Fatal(err)
			}
		}
		if phase == "closed" {
			p.Close(context.Background())
		}
		if state := p.JointJogState(); state.Active || state.Reason == "" {
			t.Fatal(phase, state)
		}
		if _, err := p.JointJog(context.Background(), "begin", denseexec.JointJogIntent{}); err == nil {
			t.Fatal(phase, "jog admitted")
		}
		if _, err := p.JointMove(context.Background(), denseexec.JointMoveIntent{}); err == nil {
			t.Fatal(phase, "move admitted")
		}
		if _, err := p.Trace(context.Background(), "begin", denseexec.TraceIntent{}); err == nil {
			t.Fatal(phase, "trace admitted")
		}
	}
}

// TestJointBrowserRequestIdentity runs the shipped builder and motion handlers;
// every joint request must retain the target displayed at the start of the action.
func TestJointBrowserRequestIdentity(t *testing.T) {
	runBrowserContract(t, `
import assert from 'node:assert/strict';
import { jointJogAPI, JointJogHold, JointAngleMove } from './ui/src/execution/jointJog.ts';
const sent = [];
let resolveState;
const api = jointJogAPI(async (url, init) => {
  sent.push({url, init});
  if (url.endsWith('/state')) await new Promise(resolve => { resolveState = resolve; });
  return Response.json(url.endsWith('/move') ? {axis: 0, state: 'done', revision: 4} : {active: false, revision: 3, reason: ''});
});
let tick;
const hold = new JointJogHold(api, () => {}, {every: fn => {tick = fn; return 1;}, clear: () => {}});
const displayed = {cell_id: 'sim', selection_generation: '7'};
const begin = hold.begin('bench', 's', 0, 1, 0.1, displayed);
displayed.cell_id = 'replacement'; displayed.selection_generation = '8';
resolveState(); await begin;
tick(); await new Promise(resolve => setImmediate(resolve));
await hold.end();
const move = new JointAngleMove(api, () => {});
const moving = move.begin('bench', 's', 0, 30, 0.5, {cell_id: 'sim', selection_generation: '7'});
resolveState(); await moving;
assert.equal(move.state.state, 'done');
assert.deepEqual(sent.map(r => r.url.split('/').at(-1)), ['state', 'begin', 'update', 'end', 'state', 'move']);
for (const {init} of sent) {
  assert.equal(init.headers['X-RT-Target-Cell'], 'sim');
  assert.equal(init.headers['X-RT-Target-Generation'], '7');
}
`)
}

func (b *jointHTTPBackend) JointJog(_ context.Context, action string, in denseexec.JointJogIntent) (denseexec.JointJogState, error) {
	b.action, b.intent = action, in
	b.calls = append(b.calls, action)
	return denseexec.JointJogState{Active: action != "end", Generation: 17, CadenceMS: 50}, b.err
}
func (b *jointHTTPBackend) JointJogState() denseexec.JointJogState {
	b.calls = append(b.calls, "state")
	return denseexec.JointJogState{CadenceMS: 50}
}

func (b *jointHTTPBackend) JointMove(_ context.Context, in denseexec.JointMoveIntent) (denseexec.JointMoveState, error) {
	b.action = "move"
	b.calls = append(b.calls, "move")
	b.intent = denseexec.JointJogIntent{TargetID: in.TargetID, SessionID: in.SessionID, Axis: in.Axis, Fraction: in.Fraction, Revision: in.Revision}
	return denseexec.JointMoveState{Axis: in.Axis, State: "done", Revision: in.Revision + 1}, b.err
}

func TestJointMoveHTTPRouteAndTypedRefusal(t *testing.T) {
	b := &jointHTTPBackend{}
	h := NewServer(ServerOptions{DenseExecution: b}).Handler()
	for _, code := range []string{"", "joint_move_limits"} {
		b.err = nil
		if code != "" {
			b.err = &denseexec.Error{Status: 409, Code: code, Detail: "outside soft limits"}
		}
		w := httptest.NewRecorder()
		h.ServeHTTP(w, httptest.NewRequest(http.MethodPost, denseExecutionPath+"/move", strings.NewReader(`{"target_id":"bench","session_id":"s","revision":3,"axis":0,"degrees":30,"fraction":0.5}`)))
		if code == "" {
			if w.Code != 200 || !strings.Contains(w.Body.String(), `"state":"done"`) || b.action != "move" || b.intent.Revision != 3 {
				t.Fatal(w.Body.String())
			}
		} else if w.Code != 409 || !strings.Contains(w.Body.String(), code) {
			t.Fatal(w.Body.String())
		}
	}
}

func TestJointJogHTTPRoutes(t *testing.T) {
	b := &jointHTTPBackend{}
	h := NewServer(ServerOptions{DenseExecution: b}).Handler()
	for _, action := range []string{"begin", "update", "end", "state"} {
		method := http.MethodPost
		if action == "state" {
			method = http.MethodGet
		}
		w := httptest.NewRecorder()
		h.ServeHTTP(w, httptest.NewRequest(method, denseExecutionPath+"/jog/"+action, strings.NewReader(`{"target_id":"bench","session_id":"s","revision":3,"axis":0,"direction":1,"fraction":0.1}`)))
		if w.Code != 200 {
			t.Fatalf("%s: %d %s", action, w.Code, w.Body.String())
		}
		if action != "state" && (b.action != action || b.intent.TargetID != "bench" || b.intent.Revision != 3) {
			t.Fatalf("route lost fence: %+v", b)
		}
	}
	b.err = &denseexec.Error{Status: 409, Code: "jog_not_ready", Detail: "Jog not ready"}
	w := httptest.NewRecorder()
	h.ServeHTTP(w, httptest.NewRequest(http.MethodPost, denseExecutionPath+"/jog/begin", strings.NewReader(`{}`)))
	if w.Code != 409 || !strings.Contains(w.Body.String(), `"error":"jog_not_ready"`) {
		t.Fatal(w.Body.String())
	}
	w = httptest.NewRecorder()
	NewServer(ServerOptions{}).Handler().ServeHTTP(w, httptest.NewRequest(http.MethodPost, denseExecutionPath+"/jog/begin", strings.NewReader(`{}`)))
	if w.Code != 409 || !strings.Contains(w.Body.String(), "dense_target_unavailable") {
		t.Fatal(w.Body.String())
	}
}

// TestJointPickerComposed selects and arms a local simulation through HTTP, then
// proves J1 jog and signed moves reach that session without moving other axes.
func TestJointPickerComposed(t *testing.T) {
	if os.Getenv("RTCORE_COMPOSED") != "1" {
		t.Skip("set RTCORE_COMPOSED=1 after make control sim in rt-core")
	}
	root, err := filepath.Abs("../../rt-core")
	if err != nil {
		t.Fatal(err)
	}
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "joint-picker-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "jp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-program.json"))
	if err != nil {
		t.Fatal(err)
	}
	var config map[string]any
	if err := json.Unmarshal(raw, &config); err != nil {
		t.Fatal(err)
	}
	limits := config["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
	// Unverified simulation geometry, in rad and rad/s, for the signed-angle probe.
	limits["min"], limits["max"], limits["max_velocity"] = -1.0, 1.0, 1.0
	raw, err = json.Marshal(config)
	if err != nil {
		t.Fatal(err)
	}
	machine := filepath.Join(build, "machine.json")
	if err := os.WriteFile(machine, raw, 0600); err != nil {
		t.Fatal(err)
	}
	compiled, args, err := simfixture.Compile(root, machine, filepath.Join(build, "config"))
	if err != nil {
		t.Fatal(err)
	}
	spawn := func(name string, args ...string) {
		t.Helper()
		log, err := os.Create(filepath.Join(build, name+".log"))
		if err != nil {
			t.Fatal(err)
		}
		cmd := exec.Command(filepath.Join(root, "build", name), args...)
		cmd.Dir, cmd.Stdout, cmd.Stderr = build, log, log
		if err := cmd.Start(); err != nil {
			log.Close()
			t.Fatal(err)
		}
		t.Cleanup(func() {
			_ = cmd.Process.Signal(syscall.SIGTERM)
			done := make(chan error, 1)
			go func() { done <- cmd.Wait() }()
			select {
			case err := <-done:
				if err != nil {
					t.Errorf("%s exit: %v", name, err)
				}
			case <-time.After(5000 * time.Millisecond):
				_ = cmd.Process.Kill()
				<-done
				t.Errorf("%s failed to terminate", name)
			}
			log.Close()
			if t.Failed() {
				raw, _ := os.ReadFile(log.Name())
				t.Logf("%s: %s", name, raw)
			}
		})
	}
	// Unverified observation ceiling; only published state can satisfy a wait.
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	await := func(predicate func() bool) {
		t.Helper()
		ticker := time.NewTicker(time.Millisecond)
		defer ticker.Stop()
		for !predicate() {
			select {
			case <-ctx.Done():
				t.Fatal("joint picker observation missing", ctx.Err())
			case <-ticker.C:
			}
		}
	}
	coreSocket, socket := filepath.Join(dir, "i.sock"), filepath.Join(dir, "c.sock")
	spawn("rosie-rt-core-sim", append(args, "--socket-path", coreSocket)...)
	await(func() bool {
		raw, _ := os.ReadFile(filepath.Join(dir, "metrics.json"))
		var m struct {
			ConfigVerifiedMask uint32 `json:"config_verified_mask"`
		}
		return json.Unmarshal(raw, &m) == nil && m.ConfigVerifiedMask == 511
	})
	spawn("rt-control", "--core-socket", coreSocket, "--socket", socket, "--configuration-sha256", filepath.Base(compiled), "--backend", "simulation", "--pair-id", "bench", "--pair-revision", "7")
	c, err := control.Dial(control.UnixPath(socket))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { c.Close() })
	await(func() bool { s, err := c.Status(ctx); return err == nil && s.Motion.TimeNS > 0 })
	p := targets.NewPicker(targets.PickerOptions{
		Catalogue: targets.CellCatalogue{Cells: []targets.Cell{{CellID: "sim-local", Label: "Local simulation", Models: []string{"rosie_1420_v1"}, RequestedMode: "simulation", Execution: targets.CellExecution{Local: true}}}},
		Local:     denseexec.RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7}, AxisMask: 511},
		Source:    denseexec.Config{ControllerID: "offline-programming:joint-picker"},
	})
	t.Cleanup(func() { p.Close(context.Background()) })
	h := NewServer(ServerOptions{DenseExecution: p}).Handler()
	request := func(method, path string, body any, out any) {
		t.Helper()
		raw, err := json.Marshal(body)
		if err != nil {
			t.Fatal(err)
		}
		r := httptest.NewRequest(method, "/api/offline-programming/v1/"+path, strings.NewReader(string(raw))).WithContext(ctx)
		r.Header.Set("X-RT-Target-Generation", "1")
		r.Header.Set("X-RT-Target-Cell", "sim-local")
		w := httptest.NewRecorder()
		h.ServeHTTP(w, r)
		if w.Code != 200 {
			t.Fatalf("%s: %d %s", path, w.Code, w.Body.String())
		}
		if out != nil {
			if err := json.Unmarshal(w.Body.Bytes(), out); err != nil {
				t.Fatal(err)
			}
		}
	}
	request(http.MethodPost, "targets/select", targets.Selection{CellID: "sim-local", ModelID: "rosie_1420_v1"}, nil)
	request(http.MethodPost, "dense-execution/arm", map[string]bool{"armed": true}, nil)
	var status denseexec.StatusBody
	request(http.MethodGet, "dense-execution/status", nil, &status)
	if status.Target == nil || status.Target.CellID != "sim-local" || status.Target.SelectionGeneration != 1 || status.Session == nil {
		t.Fatalf("selected session: %+v", status)
	}
	session := status.Session.ID
	heartbeatCtx, heartbeatCancel := context.WithCancel(ctx)
	heartbeatDone := make(chan struct{})
	defer func() { heartbeatCancel(); <-heartbeatDone }()
	go func() {
		defer close(heartbeatDone)
		ticker := time.NewTicker(100 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-heartbeatCtx.Done():
				return
			case <-ticker.C:
				_, _ = p.Heartbeat(session)
			}
		}
	}()
	observe := func() control.Status {
		t.Helper()
		s, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		return s
	}
	before := observe()
	var jog denseexec.JointJogState
	request(http.MethodGet, "dense-execution/jog/state", nil, &jog)
	in := denseexec.JointJogIntent{TargetID: "bench", SessionID: session, Revision: jog.Revision, Axis: 0, Direction: 1, Fraction: 0.1}
	request(http.MethodPost, "dense-execution/jog/begin", in, &jog)
	if !jog.Active || jog.Generation == 0 || jog.Sequence != 1 {
		t.Fatal(jog)
	}
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	for {
		distance := observe().Axes[0].LogicalPosition - before.Axes[0].LogicalPosition
		if distance > 0.5 {
			t.Fatal("jog exceeded 0.5 rad")
		}
		if distance >= 0.001 {
			break
		}
		select {
		case <-ctx.Done():
			t.Fatal("J1 did not jog")
		case <-ticker.C:
			request(http.MethodPost, "dense-execution/jog/update", in, nil)
		}
	}
	request(http.MethodPost, "dense-execution/jog/end", in, &jog)
	if jog.Active || jog.Revision != in.Revision+1 {
		t.Fatal(jog)
	}
	var heldAfter uint64
	await(func() bool {
		s := observe()
		if s.Jog == nil || s.Jog.Open != 0 || !s.Motion.Done {
			return false
		}
		if heldAfter == 0 {
			heldAfter = s.Motion.TimeNS
		}
		return s.Axes[0].TimeNS >= heldAfter
	})
	held := observe().Axes[0].LogicalTarget
	if held < 0.001 || held > 0.5 {
		t.Fatalf("held J1=%g rad", held)
	}
	for _, degrees := range []float64{30, -30} {
		start := observe().Axes[0].LogicalTarget
		request(http.MethodGet, "dense-execution/jog/state", nil, &jog)
		var move denseexec.JointMoveState
		request(http.MethodPost, "dense-execution/move", denseexec.JointMoveIntent{TargetID: "bench", SessionID: session, Revision: jog.Revision, Axis: 0, Degrees: degrees, Fraction: 0.5}, &move)
		if move.State != "done" || move.Revision != jog.Revision+1 {
			t.Fatal(move)
		}
		after := observe()
		if !after.Motion.Done || after.Motion.State != 4 || after.Core.Armed != 1 || math.Abs(after.Axes[0].LogicalPosition-start-degrees*math.Pi/180) > 0.0002 {
			t.Fatalf("move endpoint: %+v %+v", after.Motion, after.Axes[0])
		}
		for i := 1; i < len(after.Axes); i++ {
			if after.Axes[i].PositionCounts != before.Axes[i].PositionCounts {
				t.Fatalf("axis %d moved", i)
			}
		}
	}
	if math.Abs(observe().Axes[0].LogicalPosition-held) > 0.0002 {
		t.Fatal("return to held position missing")
	}
	request(http.MethodPost, "dense-execution/stop", nil, nil)
	await(func() bool { s := observe(); return s.Core.Armed == 0 && s.Core.AxisEnableMask == 0 })
}
