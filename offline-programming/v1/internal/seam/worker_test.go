package seam

import (
	"archive/zip"
	"bytes"
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/robotconfig"
)

// fakeRunner answers without a subprocess, so the bounds and the cache are
// tested at their own speed rather than OCCT's.
type fakeRunner struct {
	calls    atomic.Int64
	response []byte
	stderr   []byte
	err      error
	seen     atomic.Value // last Command
}

func (runner *fakeRunner) Run(_ context.Context, command Command) ([]byte, []byte, error) {
	runner.calls.Add(1)
	runner.seen.Store(command)
	return runner.response, runner.stderr, runner.err
}

func newWorker(t *testing.T, runner CommandRunner) *Worker {
	t.Helper()
	worker, err := New(Config{WeldPlannerRoot: t.TempDir(), Runner: runner})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	return worker
}

func TestARequestWithNoSTEPIsRefusedRatherThanResolved(t *testing.T) {
	t.Parallel()
	// The point of D24: this worker holds nothing between calls, so there is
	// no workspace to look a hash up in and no answer to give without bytes.
	worker := newWorker(t, &fakeRunner{response: []byte(`{"joints":{}}`)})
	_, err := worker.DetectJoints(context.Background(), JointsRequest{})
	var workerErr *WorkerError
	if !errors.As(err, &workerErr) || workerErr.Code != ErrorInvalidRequest {
		t.Fatalf("want invalid_request, got %v", err)
	}
	if !strings.Contains(workerErr.Message, "workspace") {
		t.Errorf("the message should say why there is nothing to resolve: %q", workerErr.Message)
	}
}

func TestAnOversizedSTEPIsRefusedBeforeItIsEncoded(t *testing.T) {
	t.Parallel()
	runner := &fakeRunner{response: []byte(`{"joints":{}}`)}
	worker, err := New(Config{WeldPlannerRoot: t.TempDir(), MaxSTEPBytes: 16, Runner: runner})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	_, err = worker.DetectJoints(context.Background(), JointsRequest{STEP: make([]byte, 17)})
	var workerErr *WorkerError
	if !errors.As(err, &workerErr) || workerErr.Code != ErrorPayloadTooLarge {
		t.Fatalf("want payload_too_large, got %v", err)
	}
	if runner.calls.Load() != 0 {
		t.Error("the subprocess should not have been started")
	}
}

func TestEveryRequestReachesTheWorker(t *testing.T) {
	t.Parallel()
	// Nothing is cached. A response cache is only correct keyed by the whole
	// request, and keyed that way it hits on a verbatim repeat, which is not
	// what authoring does -- the next click is the next seam. Measured, it
	// would save a 50 ms parse over a local pipe and none of the 0.34 s
	// `import OCP` that dominates a request.
	runner := &fakeRunner{response: []byte(`{"joints":{}}`)}
	worker := newWorker(t, runner)
	step := []byte("ISO-10303-21;")

	first, err := worker.DetectJoints(context.Background(), JointsRequest{STEP: step})
	if err != nil {
		t.Fatalf("first: %v", err)
	}
	if _, err := worker.DetectJoints(context.Background(), JointsRequest{STEP: step}); err != nil {
		t.Fatalf("second: %v", err)
	}
	if runner.calls.Load() != 2 {
		t.Fatalf("want both calls to reach the worker, got %d", runner.calls.Load())
	}
	if first.Fingerprint != FingerprintSTEP(step) {
		t.Errorf("fingerprint %q does not name the part", first.Fingerprint)
	}
}

func TestTheCallerHoldsTheDerivedDocumentAndHandsItBack(t *testing.T) {
	t.Parallel()
	// What stands in for a cache: the expensive half travels in the request,
	// so the worker does not rebuild what the caller already has.
	runner := &fakeRunner{response: []byte(`{"joints":{}}`)}
	worker := newWorker(t, runner)
	if _, err := worker.DetectJoints(context.Background(), JointsRequest{
		STEP:     []byte("ISO-10303-21;"),
		Topology: []byte(`{"solids":[{"id":"solid_0"}]}`),
	}); err != nil {
		t.Fatalf("DetectJoints: %v", err)
	}
	var body struct {
		Topology map[string]any `json:"topology"`
	}
	if err := json.Unmarshal(runner.seen.Load().(Command).Stdin, &body); err != nil {
		t.Fatalf("stdin: %v", err)
	}
	if body.Topology == nil {
		t.Fatal("the topology the caller supplied did not travel")
	}
}

func TestTheWorkersOwnErrorCodeSurvivesAsACode(t *testing.T) {
	t.Parallel()
	// The worker answers with a code rather than a traceback, including when
	// it exits non-zero, so the body is read before the exit status is trusted.
	runner := &fakeRunner{
		response: []byte(`{"error":{"code":"invalid_request","detail":"joint is required"}}`),
		err:      errors.New("exit status 2"),
	}
	worker := newWorker(t, runner)
	_, err := worker.DetectJoints(context.Background(), JointsRequest{STEP: []byte("x")})
	var workerErr *WorkerError
	if !errors.As(err, &workerErr) || workerErr.Code != ErrorInvalidRequest {
		t.Fatalf("want invalid_request from the body, got %v", err)
	}
	if workerErr.Message != "joint is required" {
		t.Errorf("detail lost: %q", workerErr.Message)
	}
}

func TestACrashWithNoBodyIsClassifiedFromStderr(t *testing.T) {
	t.Parallel()
	runner := &fakeRunner{stderr: []byte("Killed"), err: errors.New("signal: killed")}
	worker := newWorker(t, runner)
	_, err := worker.DetectJoints(context.Background(), JointsRequest{STEP: []byte("x")})
	var workerErr *WorkerError
	if !errors.As(err, &workerErr) || workerErr.Code != ErrorWorkerFailed {
		t.Fatalf("want worker_failed, got %v", err)
	}
	if !strings.Contains(workerErr.Message, "Killed") {
		t.Errorf("stderr should reach the caller: %q", workerErr.Message)
	}
}

func TestCheckTorchFitsNamesTheDocumentItIsMissing(t *testing.T) {
	t.Parallel()
	worker := newWorker(t, &fakeRunner{response: []byte(`{"clearance":{}}`)})
	_, err := worker.CheckTorchFits(context.Background(), TorchFitRequest{
		STEP:  []byte("x"),
		Joint: []byte(`{"id":"weld_joint_0001"}`),
		Seam:  []byte(`{"id":"seam_0001"}`),
	})
	var workerErr *WorkerError
	if !errors.As(err, &workerErr) || !strings.Contains(workerErr.Message, "torch") {
		t.Fatalf("want the missing document named, got %v", err)
	}
}

func TestTheCommandRunsTheWorkerModuleInTheWeldPlannerRoot(t *testing.T) {
	t.Parallel()
	runner := &fakeRunner{response: []byte(`{"joints":{}}`)}
	root := t.TempDir()
	worker, err := New(Config{WeldPlannerRoot: root, PythonExecutable: "python3", Runner: runner})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	if _, err := worker.DetectJoints(context.Background(),
		JointsRequest{STEP: []byte("ISO-10303-21;")}); err != nil {
		t.Fatalf("run: %v", err)
	}
	command := runner.seen.Load().(Command)
	if command.Dir != root || command.Name != "python3" {
		t.Fatalf("ran %q in %q", command.Name, command.Dir)
	}
	if strings.Join(command.Args, " ") != "-m seam_worker.workers --stdin" {
		t.Fatalf("args: %v", command.Args)
	}
	var body struct {
		Operation string `json:"operation"`
		STEP      string `json:"step_base64"`
	}
	if err := json.Unmarshal(command.Stdin, &body); err != nil {
		t.Fatalf("stdin is not JSON: %v", err)
	}
	if body.Operation != string(OperationDetectJoints) || body.STEP == "" {
		t.Fatalf("the request does not carry its own STEP: %+v", body)
	}
}

func TestPlanTorchPathRefusesWhatItWouldOtherwiseGuess(t *testing.T) {
	t.Parallel()
	worker := newWorker(t, &fakeRunner{response: []byte(`{"plan":{}}`)})
	base := TorchPathRequest{
		STEP:  []byte("ISO-10303-21;"),
		Joint: []byte(`{"id":"weld_joint_0001"}`),
		Seam:  []byte(`{"id":"seam_0001"}`),
		Torch: []byte(`{"id":"mig_gas_nozzle_16"}`),
		Search: SearchKnobs{WorkDeviationDeg: 15, TravelDeviationDeg: 40,
			StandoffCostDegPerMM: 3},
		DefaultStandoffMM: 10,
	}
	for name, mutate := range map[string]func(*TorchPathRequest){
		// A gun this worker guessed at would be a coverage number about
		// hardware nobody has.
		"no torch":      func(r *TorchPathRequest) { r.Torch = nil },
		"no standoff":   func(r *TorchPathRequest) { r.DefaultStandoffMM = 0 },
		"bad direction": func(r *TorchPathRequest) { r.Direction = "sideways" },
	} {
		request := base
		mutate(&request)
		if _, err := worker.PlanTorchPath(context.Background(), request); err == nil {
			t.Errorf("%s was accepted", name)
		}
	}
}

func TestPlanTorchPathAsksForOneDirectionAtATime(t *testing.T) {
	t.Parallel()
	// Both ways is two calls. Mirroring one wide table answers both at once
	// but only pays when the authored angle sits inside the deviation, and
	// outside it the bin count runs away.
	runner := &fakeRunner{response: []byte(`{"plan":{}}`)}
	worker := newWorker(t, runner)
	request := TorchPathRequest{
		STEP:  []byte("ISO-10303-21;"),
		Joint: []byte(`{"id":"j"}`), Seam: []byte(`{"id":"s"}`), Torch: []byte(`{"id":"t"}`),
		Search:            SearchKnobs{WorkDeviationDeg: 15, TravelDeviationDeg: 40},
		DefaultStandoffMM: 10,
	}
	for _, direction := range []string{"", "reverse"} {
		request.Direction = direction
		if _, err := worker.PlanTorchPath(context.Background(), request); err != nil {
			t.Fatalf("%q: %v", direction, err)
		}
		var body struct {
			Direction string `json:"direction"`
		}
		if err := json.Unmarshal(runner.seen.Load().(Command).Stdin, &body); err != nil {
			t.Fatalf("stdin: %v", err)
		}
		want := direction
		if want == "" {
			want = "forward"
		}
		if body.Direction != want {
			t.Errorf("asked for %q, sent %q", want, body.Direction)
		}
	}
}

func TestSampleSeamFramesIsTheOneRequestWithNoSTEP(t *testing.T) {
	t.Parallel()
	// A curve and the rail its work angle is measured from are the whole
	// input. Nothing about the part is measured, so there is nothing to send.
	runner := &fakeRunner{response: []byte(`{"frames":{"frames":[]}}`)}
	worker := newWorker(t, runner)
	if _, err := worker.SampleSeamFrames(context.Background(), SeamFramesRequest{
		Curve:     []byte(`{"segments":[]}`),
		Reference: []byte(`{"mode":"rail_curve"}`),
		Samples:   160,
	}); err != nil {
		t.Fatalf("SampleSeamFrames: %v", err)
	}
	var body map[string]any
	if err := json.Unmarshal(runner.seen.Load().(Command).Stdin, &body); err != nil {
		t.Fatalf("stdin: %v", err)
	}
	if _, present := body["step_base64"]; present {
		t.Error("a STEP travelled for a request that does not measure the part")
	}
	if body["samples"] != float64(160) {
		t.Errorf("samples: %v", body["samples"])
	}

	// The exception is this operation, not the rule.
	if _, err := worker.DetectJoints(context.Background(), JointsRequest{}); err == nil {
		t.Error("detect_joints was allowed to run without a STEP")
	}
	if _, err := worker.SampleSeamFrames(context.Background(),
		SeamFramesRequest{Curve: []byte(`{"segments":[]}`)}); err == nil {
		t.Error("frames without a reference were accepted")
	}
}

func TestBuildPlanRequestWillNotNameARobotItWasNotGiven(t *testing.T) {
	t.Parallel()
	// Naming one would resolve differently on another machine, and the
	// placement is measured from that cell's work frame.
	worker := newWorker(t, &fakeRunner{response: []byte(`{"weldplan_base64":""}`)})
	_, err := worker.BuildPlanRequest(context.Background(), PlanRequestBuild{
		STEP:    []byte("ISO-10303-21;"),
		Program: []byte(`{"schema":"robot.v4.program.v2"}`),
	})
	var workerErr *WorkerError
	if !errors.As(err, &workerErr) || !strings.Contains(workerErr.Message, "work frame") {
		t.Fatalf("want the reason, got %v", err)
	}
}

// ---- against the real worker -------------------------------------------------

func weldPlannerRoot(t *testing.T) string {
	t.Helper()
	root, err := filepath.Abs(filepath.Join("..", "..", "..", "..", "weld_planner", "v1"))
	if err != nil {
		t.Fatalf("root: %v", err)
	}
	python := filepath.Join(root, ".pixi", "envs", "default", "bin", "python")
	if _, err := os.Stat(python); err != nil {
		t.Skipf("weld planner environment not provisioned at %s", python)
	}
	t.Setenv("SEAM_WORKER_PYTHON", python)
	return root
}

func TestTheRealWorkerDetectsJointsAndTheSeamsTheyDefaultTo(t *testing.T) {
	root := weldPlannerRoot(t)
	step, err := os.ReadFile(filepath.Join(root, "data", "fixtures", "bracket_b.step"))
	if err != nil {
		t.Skipf("fixture: %v", err)
	}
	worker, err := New(Config{WeldPlannerRoot: root})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Minute)
	defer cancel()

	result, err := worker.DetectJoints(ctx, JointsRequest{STEP: step})
	if err != nil {
		t.Fatalf("DetectJoints: %v", err)
	}
	var answer struct {
		Joints struct {
			WeldJoints []map[string]any `json:"weld_joints"`
			Seams      []struct {
				ID           string `json:"id"`
				WeldJointRef string `json:"weld_joint_ref"`
			} `json:"seams"`
		} `json:"joints"`
	}
	if err := json.Unmarshal(result.JSON, &answer); err != nil {
		t.Fatalf("response: %v", err)
	}
	if len(answer.Joints.WeldJoints) == 0 {
		t.Fatal("no weld joints on a part that has them")
	}
	if len(answer.Joints.Seams) == 0 {
		t.Fatal("joints with no default seams are places a weld could go, not welds")
	}
	// Every seam names a joint, and after D24 that name has to resolve inside
	// whatever document carries it.
	byID := map[string]bool{}
	for _, joint := range answer.Joints.WeldJoints {
		if id, ok := joint["id"].(string); ok {
			byID[id] = true
		}
	}
	for _, seam := range answer.Joints.Seams {
		if !byID[seam.WeldJointRef] {
			t.Errorf("seam %q names joint %q, which is not in this answer", seam.ID, seam.WeldJointRef)
		}
	}
}

func TestTheRealWorkerPlansATorchPathBothWaysRound(t *testing.T) {
	root := weldPlannerRoot(t)
	step, err := os.ReadFile(filepath.Join(root, "data", "fixtures", "bracket_b.step"))
	if err != nil {
		t.Skipf("fixture: %v", err)
	}
	worker, err := New(Config{WeldPlannerRoot: root})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 8*time.Minute)
	defer cancel()

	detected, err := worker.DetectJoints(ctx, JointsRequest{STEP: step})
	if err != nil {
		t.Fatalf("DetectJoints: %v", err)
	}
	var found struct {
		Joints struct {
			WeldJoints []json.RawMessage `json:"weld_joints"`
			Seams      []json.RawMessage `json:"seams"`
		} `json:"joints"`
	}
	if err := json.Unmarshal(detected.JSON, &found); err != nil {
		t.Fatalf("joints: %v", err)
	}
	var jointID struct {
		ID string `json:"id"`
	}
	_ = json.Unmarshal(found.Joints.WeldJoints[0], &jointID)
	var seam json.RawMessage
	for _, candidate := range found.Joints.Seams {
		var ref struct {
			WeldJointRef string `json:"weld_joint_ref"`
		}
		if err := json.Unmarshal(candidate, &ref); err == nil && ref.WeldJointRef == jointID.ID {
			seam = candidate
			break
		}
	}
	if seam == nil {
		t.Fatal("no seam names the first joint")
	}

	// The reason a caller asks twice: the same metal answers differently run
	// the other way, and nothing but measuring both says which is better.
	coverage := map[string]float64{}
	for _, direction := range []string{"forward", "reverse"} {
		result, err := worker.PlanTorchPath(ctx, TorchPathRequest{
			STEP:  step,
			Joint: found.Joints.WeldJoints[0],
			Seam:  seam,
			Torch: []byte(`{"id":"mig_gas_nozzle_16","body_radius_mm":8,"body_length_mm":150,` +
				`"standoff_range_mm":{"min":6,"max":25}}`),
			Direction:         direction,
			Search:            SearchKnobs{WorkDeviationDeg: 15, TravelDeviationDeg: 40, StandoffCostDegPerMM: 3},
			DefaultStandoffMM: 10,
		})
		if err != nil {
			t.Fatalf("PlanTorchPath %s: %v", direction, err)
		}
		var answer struct {
			Direction string `json:"direction"`
			RaysCast  int    `json:"rays_cast"`
			Plan      struct {
				ClearFraction float64 `json:"clear_fraction"`
				StandoffMM    float64 `json:"standoff_mm"`
			} `json:"plan"`
		}
		if err := json.Unmarshal(result.JSON, &answer); err != nil {
			t.Fatalf("response: %v", err)
		}
		if answer.Direction != direction {
			t.Errorf("asked %q, answered for %q", direction, answer.Direction)
		}
		if answer.RaysCast <= 0 {
			t.Error("a lattice that cast no rays measured nothing")
		}
		if answer.Plan.StandoffMM <= 0 {
			t.Error("the plan settled on no standoff")
		}
		coverage[direction] = answer.Plan.ClearFraction
	}
	if coverage["forward"] == coverage["reverse"] {
		t.Logf("both directions cover %.3f; this seam does not distinguish them", coverage["forward"])
	}
}

func TestTheRealWorkerPacksAPlanRequestThePlannerCanOpen(t *testing.T) {
	root := weldPlannerRoot(t)
	step, err := os.ReadFile(filepath.Join(root, "data", "fixtures", "bracket_b.step"))
	if err != nil {
		t.Skipf("fixture: %v", err)
	}
	// The robot as the OLP compiles it: the one compiler, against the real
	// description, with the model's empty cell document.
	description, err := robotconfig.At(filepath.Join(root, "..", "..", "robot_description", "robots")).Load("rosie_1400_v3")
	if err != nil {
		t.Skipf("cell: %v", err)
	}
	settings, err := robotconfig.Resolve(description, nil, robotconfig.SourceEmpty)
	if err != nil {
		t.Fatalf("Resolve: %v", err)
	}
	compiled, err := robotconfig.Compile(description, settings)
	if err != nil {
		t.Fatalf("Compile: %v", err)
	}
	planningContext, err := json.Marshal(compiled.Wire())
	if err != nil {
		t.Fatal(err)
	}
	worker, err := New(Config{WeldPlannerRoot: root})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Minute)
	defer cancel()

	// A program.v2 with one seam weld, minimal but real: the container refuses
	// a document its own validator will not take.
	program := []byte(`{
	  "schema":"robot.v4.program.v2","program_id":"p","program_name":"p",
	  "program_revision":1,"base_program_revision":0,"program_state":"draft",
	  "execution_mode":"simulation","created_utc":"2026-09-10T00:00:00Z",
	  "units":{"length":"m","angle":"rad","time":"s","joint_order":["J1","J2","J3","J4","J5","J6"]},
	  "producer":{"component":"offline-programming-v1","source_revision":"test","inputs":[]},
	  "workpiece":{"frame_id":"workpiece"},
	  "placement":{"cell_id":"rosie_1400_v3","anchor_xyz_m":[0,0,0],"offset_xyz_m":[0,0,0],"rpy_rad":[0,0,0]},
	  "robot":{"model_id":"rosie_1400_v3","robot_description_sha256":"sha256:0000000000000000000000000000000000000000000000000000000000000000"},
	  "defaults":{"clearance_min_mm":20,"speed_scale":1,"torch_policy":"free","positioner_policy":"free"},
	  "planner_owns":["positioner joint values"],
	  "nodes":[
	    {"id":"a","op":"approach","enabled":true,"label":"a","comment":"","row_hash":"sha256:0000000000000000000000000000000000000000000000000000000000000000"},
	    {"id":"w","op":"weld","enabled":true,"label":"w","comment":"","row_hash":"sha256:0000000000000000000000000000000000000000000000000000000000000000",
	     "geometry":{"kind":"seam","id":"seam_0001",
	       "curve":{"segments":[{"degree":1,"control_points_xyz_m":[[0,0,0],[0.1,0,0]],"knots_u":[0,0,1,1]}]},
	       "reference":{"mode":"rail"},
	       "orientation":{"work_angle_deg":{"kind":"constant","value":0},"travel_angle_deg":{"kind":"constant","value":10}}},
	     "travel_speed_mm_s":{"kind":"constant","value":4},"standoff_mm":{"kind":"constant","value":17},"weave":{"shape":"none"},"weld_parameters":{}},
	    {"id":"r","op":"retract","enabled":true,"label":"r","comment":"","row_hash":"sha256:0000000000000000000000000000000000000000000000000000000000000000"}
	  ]}`)

	result, err := worker.BuildPlanRequest(ctx, PlanRequestBuild{
		STEP: step, STEPFilename: "bracket_b.step", Program: program,
		PlanningContext: planningContext,
		Tooling:         []byte(`{"id":"mig_gas_nozzle_16","body_radius_mm":8,"body_length_mm":150,"default_standoff_mm":17}`),
		RequestID:       "packed-by-a-test",
	})
	if err != nil {
		t.Fatalf("BuildPlanRequest: %v", err)
	}
	var packed struct {
		WeldplanBase64 string `json:"weldplan_base64"`
		Bytes          int    `json:"bytes"`
		SHA256         string `json:"sha256"`
	}
	if err := json.Unmarshal(result.JSON, &packed); err != nil {
		t.Fatalf("response: %v", err)
	}
	blob, err := base64.StdEncoding.DecodeString(packed.WeldplanBase64)
	if err != nil || len(blob) != packed.Bytes {
		t.Fatalf("blob: %v, %d bytes against %d declared", err, len(blob), packed.Bytes)
	}
	archive, err := zip.NewReader(bytes.NewReader(blob), int64(len(blob)))
	if err != nil {
		t.Fatalf("not a zip: %v", err)
	}
	present := map[string]bool{}
	for _, file := range archive.File {
		present[file.Name] = true
	}
	for _, name := range []string{"mimetype", "manifest.json", "program.json", "cell.json", "tooling.json",
		"context/robot.urdf", "context/planner.json", "context/identity.json", "context/spheres.json"} {
		if !present[name] {
			t.Errorf("%s is missing from the container", name)
		}
	}
	// The placement travels inside the program now; a second copy beside it
	// could only ever disagree.
	if present["placement.json"] {
		t.Error("placement.json is packed as well as carried in the program")
	}
	if len(packed.SHA256) != 64 {
		t.Errorf("no digest to name the request by: %q", packed.SHA256)
	}
}

func TestAnUnsetRailOffsetIsAbsentRatherThanZero(t *testing.T) {
	t.Parallel()
	// Zero is the one value that breaks the rail. The bisector rail is the
	// weld line moved along the groove bisector, and the offset is the only
	// thing that gives it a DIRECTION: `frames.py` reads the work-angle
	// reference as (rail - point), so a rail lying on the seam yields the
	// zero vector, the reference falls back to world up, and every 90 degree
	// fillet is measured 45 degrees off its own bisector. Measured on
	// `bracket_b_2x.step`, that took every seam from 99-100% weldable to
	// 0-12%.
	//
	// A Go zero value is indistinguishable from "unset" once it is on the
	// wire, so the field has to be left out for the worker's own default to
	// apply -- the same convention `min_length_m` already follows.
	runner := &fakeRunner{response: []byte(`{"joints":{}}`)}
	worker := newWorker(t, runner)
	if _, err := worker.DetectJoints(context.Background(), JointsRequest{
		STEP: []byte("ISO-10303-21;"),
	}); err != nil {
		t.Fatalf("DetectJoints: %v", err)
	}
	command, _ := runner.seen.Load().(Command)
	var body map[string]any
	if err := json.Unmarshal(command.Stdin, &body); err != nil {
		t.Fatalf("request body is not JSON: %v", err)
	}
	if _, present := body["rail_offset_m"]; present {
		t.Errorf("an unset offset must not reach the worker at all, got %v", body["rail_offset_m"])
	}
}

func TestAnAskedForRailOffsetIsSent(t *testing.T) {
	t.Parallel()
	runner := &fakeRunner{response: []byte(`{"joints":{}}`)}
	worker := newWorker(t, runner)
	if _, err := worker.DetectJoints(context.Background(), JointsRequest{
		STEP: []byte("ISO-10303-21;"), RailOffsetM: 0.035,
	}); err != nil {
		t.Fatalf("DetectJoints: %v", err)
	}
	command, _ := runner.seen.Load().(Command)
	var body map[string]any
	if err := json.Unmarshal(command.Stdin, &body); err != nil {
		t.Fatalf("request body is not JSON: %v", err)
	}
	if body["rail_offset_m"] != 0.035 {
		t.Errorf("want the offset the caller asked for, got %v", body["rail_offset_m"])
	}
}

// packWeldPlan always sends `io_outputs` (null when the program switches no
// output). The strict transport must know it: on 2026-10-11 it did not, and
// every the 1430 cell plan was refused with `json: unknown field "io_outputs"`. The
// cell's outputs reach the builder verbatim; null or absent reach it as
// absent ("no outputs packed").
func TestExecuteJSONForwardsTheCellsOutputs(t *testing.T) {
	outputs := `{"schema":"rosie.cell-io-outputs.v1","torch_qualified":false,"outputs":[{"name":"gas","class":"process","expiry_ns":200000000}]}`
	for _, sent := range []string{"", "null", outputs} {
		t.Run(sent, func(t *testing.T) {
			runner := &fakeRunner{response: []byte(`{"weldplan_base64":"eA=="}`)}
			worker := newWorker(t, runner)
			body := map[string]any{"operation": "build_plan_request", "workpiece_absent": true,
				"program": json.RawMessage(`{"nodes":[]}`), "planning_context": json.RawMessage(`{"files":[]}`)}
			if sent != "" {
				body["io_outputs"] = json.RawMessage(sent)
			}
			raw, _ := json.Marshal(body)
			if _, err := worker.ExecuteJSON(context.Background(), raw); err != nil {
				t.Fatalf("strict transport rejected io_outputs: %v", err)
			}
			var received map[string]json.RawMessage
			if err := json.Unmarshal(runner.seen.Load().(Command).Stdin, &received); err != nil {
				t.Fatal(err)
			}
			got, present := received["io_outputs"]
			if sent != outputs {
				if present {
					t.Fatalf("no outputs, yet io_outputs reached the builder: %s", got)
				}
				return
			}
			var want, have any
			json.Unmarshal([]byte(outputs), &want)
			json.Unmarshal(got, &have)
			a, _ := json.Marshal(want)
			b, _ := json.Marshal(have)
			if !bytes.Equal(a, b) {
				t.Fatalf("outputs dropped or changed: %s", got)
			}
		})
	}
}

// Exercise the same strict JSON boundary used by packWeldPlan, not only the
// lower Python builder. A captured start must survive both Go request structs.
func TestExecuteJSONForwardsMeasuredPlanningStart(t *testing.T) {
	for _, start := range []string{"", "null", `{"schema":"rosie.measured-planning-start.v1","joint_names":["J1","J2","J3","J4","J5","J6"],"joint_values_rad":[0.01,0,0,0,0,-0.02],"coordinate_identity":"config:2:3","observation_identity":"adapter:daemon"}`} {
		t.Run(start, func(t *testing.T) {
			runner := &fakeRunner{response: []byte(`{"weldplan_base64":"eA=="}`)}
			worker := newWorker(t, runner)
			body := map[string]any{"operation": "build_plan_request", "workpiece_absent": true, "program": json.RawMessage(`{"nodes":[]}`), "planning_context": json.RawMessage(`{"files":[]}`)}
			if start != "" {
				body["start_state"] = json.RawMessage(start)
			}
			raw, _ := json.Marshal(body)
			if _, err := worker.ExecuteJSON(context.Background(), raw); err != nil {
				t.Fatalf("strict transport rejected start: %v", err)
			}
			var received map[string]json.RawMessage
			if err := json.Unmarshal(runner.seen.Load().(Command).Stdin, &received); err != nil {
				t.Fatal(err)
			}
			if start != "" && start != "null" {
				var want, got any
				json.Unmarshal([]byte(start), &want)
				json.Unmarshal(received["start_state"], &got)
				a, _ := json.Marshal(want)
				b, _ := json.Marshal(got)
				if !bytes.Equal(a, b) {
					t.Fatalf("capture dropped or changed: %s", received["start_state"])
				}
			}
			body["invented_field"] = true
			raw, _ = json.Marshal(body)
			before := runner.calls.Load()
			if _, err := worker.ExecuteJSON(context.Background(), raw); err == nil || runner.calls.Load() != before {
				t.Fatal("unrelated unknown JSON field bypassed strict decoder")
			}
		})
	}
}
