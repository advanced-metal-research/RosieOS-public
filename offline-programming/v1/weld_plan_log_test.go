package main

import (
	"bytes"
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"log"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
	"rosieos/offline-programming/v1/internal/seam"
	"rosieos/rt-core/sdk/control"
)

// Every /weld-plan and /weld-plan/export answer is one journal line naming
// the route, program, client, status and code, with the stage durations: a
// refused plan used to leave nothing on the server.
func TestWeldPlanLogsEveryOutcomeOnOneLine(t *testing.T) {
	var lines bytes.Buffer
	previous := log.Writer()
	log.SetOutput(&lines)
	t.Cleanup(func() { log.SetOutput(previous) })

	worker := seamWorkerFunc(func(context.Context, []byte) (seam.Result, error) {
		t.Fatal("a refused request must not reach the seam worker")
		return seam.Result{}, nil
	})
	server := &Server{seams: worker, motionOrigin: "http://127.0.0.1:1"}
	body := `{"require_measured_start":true,"workpiece_absent":true,"step_base64":"","program_id":"deck-teach","program":{"nodes":[{"op":"move"}]}}`
	for _, route := range []struct {
		path    string
		handler http.HandlerFunc
	}{{WeldPlanAPIPath, server.weldPlan}, {WeldPlanExportAPIPath, server.weldPlanExport}} {
		lines.Reset()
		request := httptest.NewRequest(http.MethodPost, route.path, strings.NewReader(body))
		request.Header.Set("X-Rosie-Client", "rosie-pendant-v5 (steamdeck)")
		recorder := httptest.NewRecorder()
		route.handler(recorder, request)
		if recorder.Code != http.StatusBadRequest {
			t.Fatalf("%s: status %d %s", route.path, recorder.Code, recorder.Body)
		}
		line := strings.TrimSpace(lines.String())
		if strings.Count(line, "\n") != 0 {
			t.Fatalf("%s: want one line, got %q", route.path, line)
		}
		for _, want := range []string{"program deck-teach", "from rosie-pendant-v5 (steamdeck)", "-> 400 invalid_request",
			"no model-reset fallback", "pack ", "plan 0 ms", "fetch 0 ms", "total "} {
			if !strings.Contains(line, want) {
				t.Fatalf("%s: %q lacks %q", route.path, line, want)
			}
		}
	}

	lines.Reset()
	recorder := httptest.NewRecorder()
	(&Server{}).weldPlan(recorder, httptest.NewRequest(http.MethodPost, WeldPlanAPIPath, strings.NewReader("{}")))
	if !strings.Contains(lines.String(), "weld-plan: program - from ") || !strings.Contains(lines.String(), "-> 503 seam_worker_unavailable") {
		t.Fatalf("refusal before decode: %q", lines.String())
	}
}

// The Deck's program on the 1430 cell at 15:45 2026-10-11 (a HOME, then taught
// waypoints) and a weld-only block, against a connected cell: both go to the
// measured-start capture now (2026-10-11, owner: "this is a general purpose
// robot, we must be able to program it"). The fake target has no native
// status, so reaching the capture is what its refusal proves; before, the
// first was refused by name at 400 and the second planned from reset.
func TestConnectedPlanOfAnyMotionNodeCapturesTheMeasuredStart(t *testing.T) {
	root, _ := cellRepo(t)
	for name, program := range map[string]string{
		"home and taught": `{"nodes":[{"op":"home","label":"HOME"},{"op":"move","label":"P1"},{"op":"dwell"},{"op":"move","label":"P2"}]}`,
		"weld block only": `{"nodes":[{"op":"home"},{"op":"approach"},{"op":"weld"},{"op":"retract"},{"op":"home"}]}`,
		"home only":       `{"nodes":[{"op":"home"}]}`,
	} {
		t.Run(name, func(t *testing.T) {
			sent, err := packProgram(t, root, targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`),
				weldPlanRequest{CellID: "cell_a", WorkpieceAbsent: true, Program: json.RawMessage(program)})
			if err == nil || !strings.Contains(err.Error(), "native measured-start observation") || sent != nil {
				t.Fatalf("wanted the capture attempted before packing, got %v (sent %v)", err, sent)
			}
		})
	}
}

// Plan from program start is the operator's explicit alternative: no capture,
// the program's own start (model reset), then Go to plan start. It is never
// combined with a required measured start.
func TestPlanFromProgramStartSkipsTheCaptureAndIsNeverCombined(t *testing.T) {
	root, _ := cellRepo(t)
	target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
	program := json.RawMessage(`{"nodes":[{"op":"home"},{"op":"move","label":"P1"},{"op":"approach"},{"op":"weld"},{"op":"retract"}]}`)
	sent, err := packProgram(t, root, target, weldPlanRequest{CellID: "cell_a", WorkpieceAbsent: true, Program: program, PlanFromProgramStart: true})
	if err != nil {
		t.Fatal(err)
	}
	if start, present := sent["start_state"]; !present || start != nil {
		t.Fatalf("a program-start plan carried start_state %v", start)
	}
	if _, err := packProgram(t, root, target, weldPlanRequest{CellID: "cell_a", WorkpieceAbsent: true, Program: program,
		PlanFromProgramStart: true, RequireMeasuredStart: true}); err == nil || !strings.Contains(err.Error(), "not both") {
		t.Fatalf("both starts accepted: %v", err)
	}
}

// packProgram packs through the real seam.Worker, strict transport decode
// included, and returns what reached the Python builder. A fake in place of
// the worker let every field packWeldPlan sends skip DisallowUnknownFields:
// that is how `io_outputs` reached the 1430 cell (4ed9e029, 2026-10-11) and refused
// every plan with `invalid JSON: json: unknown field "io_outputs"`.
func packProgram(t *testing.T, root string, target CellTarget, body weldPlanRequest) (map[string]any, error) {
	t.Helper()
	runner := &stdinRunner{response: []byte(`{"weldplan_base64":"` +
		base64.StdEncoding.EncodeToString([]byte("pk")) + `","sha256":"ab"}`)}
	worker, err := seam.New(seam.Config{WeldPlannerRoot: t.TempDir(), Runner: runner})
	if err != nil {
		t.Fatal(err)
	}
	server := NewServer(ServerOptions{RepoRoot: root, CellTarget: target,
		RobotDescriptionCache: filepath.Join(root, "cache"), Seams: worker})
	_, _, err = server.packWeldPlan(context.Background(), body)
	var sent map[string]any
	if runner.stdin != nil {
		if decodeErr := json.Unmarshal(runner.stdin, &sent); decodeErr != nil {
			t.Fatal(decodeErr)
		}
	}
	return sent, err
}

// stdinRunner stands in for the seam_worker subprocess only.
type stdinRunner struct {
	response []byte
	stdin    []byte
}

func (runner *stdinRunner) Run(_ context.Context, command seam.Command) ([]byte, []byte, error) {
	runner.stdin = command.Stdin
	return runner.response, nil, nil
}

// the 1430 cell has no outputs configured: a taught program with no `io` row must
// pack (no io_outputs reaches the builder), from the measured start and from
// the program's start, through the strict seam transport.
func TestPackWeldPlanOnACellWithoutOutputsPassesTheStrictSeamTransport(t *testing.T) {
	root, _ := cellRepo(t)
	target := targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`)
	sent, err := packProgram(t, root, target, weldPlanRequest{CellID: "cell_a", WorkpieceAbsent: true, PlanFromProgramStart: true,
		Program: json.RawMessage(`{"nodes":[{"op":"move","label":"P1"},{"op":"move","label":"P2"}]}`)})
	if err != nil {
		t.Fatalf("a taught program on a cell without outputs did not pack: %v", err)
	}
	if sent["operation"] != "build_plan_request" {
		t.Fatalf("nothing reached the builder: %v", sent)
	}
	if _, present := sent["io_outputs"]; present {
		t.Fatalf("no output row, yet io_outputs reached the builder: %v", sent["io_outputs"])
	}
}

// An `io` row is scheduled against the connected cell's outputs. With no
// cell it is refused by name with the planner's own sentence; a connected
// cell with no outputs configured (the 1430 cell today) is refused with its own code.
// Both before the cell is read or the seam worker packs.
func TestWeldPlanRefusesIOThatNoCellOutputCanCarry(t *testing.T) {
	worker := seamWorkerFunc(func(context.Context, []byte) (seam.Result, error) {
		t.Fatal("a refused request must not reach the seam worker")
		return seam.Result{}, nil
	})
	for _, c := range []struct {
		name, code, want string
		target           CellTarget
	}{
		{"offline", taughtProgramUnsupported,
			"Gas on (io) switches an output: plan with the cell connected, so the plan knows the cell's outputs and their expiry", nil},
		{"connected without outputs", ioNotConfigured,
			"Gas on (io) switches an output, and the connected cell has no outputs configured (its rt-core configuration has no io block). " +
				"Add and qualify an I/O terminal on the cell, or delete or disable the row, then plan again.",
			&fakeCellTarget{reachable: true, err: errors.New("the cell must not be read for a refused program")}},
	} {
		t.Run(c.name, func(t *testing.T) {
			server := &Server{seams: worker, motionOrigin: "http://127.0.0.1:1", cellTarget: c.target}
			body := `{"workpiece_absent":true,"step_base64":"","program_id":"a031defa","program":{"nodes":[` +
				`{"op":"home","label":"HOME"},{"op":"io","label":"Gas on","channel":"gas","state":true},` +
				`{"op":"move","label":"P1"}]}}`
			recorder := httptest.NewRecorder()
			server.weldPlan(recorder, httptest.NewRequest(http.MethodPost, WeldPlanAPIPath, strings.NewReader(body)))
			var answer struct{ Code, Error string }
			if err := json.Unmarshal(recorder.Body.Bytes(), &answer); err != nil || recorder.Code != http.StatusBadRequest || answer.Code != c.code {
				t.Fatalf("status %d %s", recorder.Code, recorder.Body)
			}
			if answer.Error != c.want {
				t.Fatalf("sentence %q", answer.Error)
			}
		})
	}
}

// ioCellTarget is a connected cell whose native status describes outputs.
type ioCellTarget struct {
	*fakeCellTarget
	io string
}

func (f *ioCellTarget) Status(context.Context) *denseexec.StatusBody {
	var described control.Description
	described.IO = json.RawMessage(f.io)
	return &denseexec.StatusBody{RTCore: &denseexec.RTCoreObservation{Description: described}}
}

// The connected cell's outputs travel to the seam worker as the planner
// reads them (weldplan/io_schedule.py rosie.cell-io-outputs.v1), with
// program-start planning so no measured capture is involved.
func TestPackWeldPlanCarriesTheCellsOutputs(t *testing.T) {
	root, _ := cellRepo(t)
	target := &ioCellTarget{fakeCellTarget: targetServing(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`),
		io: `{"profile":"dio_terminal","slave_position":9,"torch_qualified":false,"inputs":[],` +
			`"outputs":[{"name":"green","bit":2,"class":"indicator","expiry_ns":200000000},{"name":"gas","bit":0,"safe_state":false,"expiry_ns":200000000,"class":"process","readback_bit":1,"readback_polarity":"active_high"}]}`}
	sent, err := packProgram(t, root, target, weldPlanRequest{CellID: "cell_a", WorkpieceAbsent: true, PlanFromProgramStart: true,
		Program: json.RawMessage(`{"nodes":[{"op":"move","label":"P1"},{"op":"io","label":"Gas on","channel":"gas","state":true}]}`)})
	if err != nil {
		t.Fatal(err)
	}
	got, _ := json.Marshal(sent["io_outputs"])
	want := `{"outputs":[{"class":"process","expiry_ns":200000000,"name":"gas"}],"schema":"rosie.cell-io-outputs.v1","torch_qualified":false}`
	if string(got) != want {
		t.Fatalf("io_outputs %s", got)
	}
	plain, err := packProgram(t, root, target, weldPlanRequest{CellID: "cell_a", WorkpieceAbsent: true, PlanFromProgramStart: true,
		Program: json.RawMessage(`{"nodes":[{"op":"move","label":"P1"}]}`)})
	if err != nil || plain["io_outputs"] != nil {
		t.Fatalf("a program with no output row carried io_outputs %v (%v)", plain["io_outputs"], err)
	}
}

func TestTaughtProgramRefusalNamesEveryEnabledIONodeOnly(t *testing.T) {
	for _, c := range []struct{ program, want string }{
		{`{"nodes":[{"op":"move"},{"op":"dwell"}]}`, ""},
		{`{"nodes":[{"op":"home"},{"op":"move"},{"op":"approach"},{"op":"weld"},{"op":"transit"},{"op":"weld"},{"op":"retract"}]}`, ""},
		{`{"nodes":[{"op":"io","enabled":false},{"op":"move"}]}`, ""},
		{`{"nodes":[{"op":"move","label":"P1"},{"op":"io","label":"Gas"},{"op":"io","label":""}]}`,
			"Gas (io), IO switch outputs: plan with the cell connected"},
	} {
		err := taughtProgramRefusal(json.RawMessage(c.program))
		if (c.want == "") != (err == nil) || (err != nil && !strings.Contains(err.Error(), c.want)) {
			t.Fatalf("%s: %v", c.program, err)
		}
	}
	if status, code := weldPlanFailure(taughtProgramRefusal(json.RawMessage(`{"nodes":[{"op":"io"}]}`))); status != http.StatusBadRequest || code != taughtProgramUnsupported {
		t.Fatalf("%d %s", status, code)
	}
}

// A jog or move still under way answers 409 planning_start_moving, so a
// client can say "let go and plan again"; the rest stay 400 under their code.
func TestWeldPlanFailureCodesForMeasuredStartRefusals(t *testing.T) {
	for _, c := range []struct {
		err    error
		status int
		code   string
	}{
		{planStartMotion(planStartMovingMessage), http.StatusConflict, planStartMoving},
		{planStartUnverifiedf("the machine reports a safety fault (mask 0x8); clear it before planning"), http.StatusBadRequest, planStartUnverified},
		{&seam.WorkerError{Code: seam.ErrorInvalidRequest, Message: "x"}, http.StatusBadRequest, string(seam.ErrorInvalidRequest)},
	} {
		status, code := weldPlanFailure(c.err)
		if status != c.status || code != c.code {
			t.Fatalf("%v: %d %s", c.err, status, code)
		}
	}
}
