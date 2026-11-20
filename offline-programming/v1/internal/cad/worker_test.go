package cad

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"
)

type fakeRunner struct {
	mu      sync.Mutex
	calls   int
	started chan struct{}
	release chan struct{}
	run     func(context.Context, Command) ([]byte, []byte, error)
}

func (runner *fakeRunner) Run(ctx context.Context, command Command) ([]byte, []byte, error) {
	runner.mu.Lock()
	runner.calls++
	runner.mu.Unlock()
	if runner.started != nil {
		select {
		case runner.started <- struct{}{}:
		default:
		}
	}
	if runner.release != nil {
		select {
		case <-runner.release:
		case <-ctx.Done():
			return nil, nil, ctx.Err()
		}
	}
	if runner.run != nil {
		return runner.run(ctx, command)
	}
	return successfulResponse(command)
}

func (runner *fakeRunner) callCount() int {
	runner.mu.Lock()
	defer runner.mu.Unlock()
	return runner.calls
}

func TestWorkerCachesOperationVariantsUnderSTEPFingerprint(t *testing.T) {
	runner := &fakeRunner{}
	worker := newTestWorker(t, runner, Config{})
	step := []byte("ISO-10303-21; SAME STEP")

	first, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: step},
	})
	if err != nil {
		t.Fatalf("first topology: %v", err)
	}
	if first.CacheHit {
		t.Fatal("first result unexpectedly came from cache")
	}
	if first.Fingerprint != FingerprintSTEP(step) {
		t.Fatalf("fingerprint = %q", first.Fingerprint)
	}
	first.JSON[0] = '['

	second, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: append([]byte(nil), step...)},
	})
	if err != nil {
		t.Fatalf("cached topology: %v", err)
	}
	if !second.CacheHit {
		t.Fatal("second result was not a cache hit")
	}
	if len(second.JSON) == 0 || second.JSON[0] != '{' {
		t.Fatal("caller mutation changed cached JSON")
	}

	_, err = worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP:        STEPInput{Filename: "part.step", Bytes: step},
		SampleCount: 32,
	})
	if err != nil {
		t.Fatalf("sample-count variant: %v", err)
	}
	_, err = worker.BuildSeam(context.Background(), SeamRequest{
		STEP:   STEPInput{Filename: "part.step", Bytes: step},
		EdgeID: "part_0:edge_00001",
		Window: &SeamWindow{Start: 0.1, End: 0.9},
	})
	if err != nil {
		t.Fatalf("seam variant: %v", err)
	}
	if calls := runner.callCount(); calls != 3 {
		t.Fatalf("runner calls = %d, want 3", calls)
	}
	if stats := worker.CacheStats(); stats.Entries != 3 || stats.Bytes <= 0 {
		t.Fatalf("cache stats = %+v", stats)
	}

	worker.PurgeFingerprint(first.Fingerprint)
	if stats := worker.CacheStats(); stats.Entries != 0 || stats.Bytes != 0 {
		t.Fatalf("cache after purge = %+v", stats)
	}
}

func TestWorkerUsesRegisteredFixtureWithoutProductBranching(t *testing.T) {
	fixturePath := filepath.Join(t.TempDir(), "fixture.step")
	if err := os.WriteFile(fixturePath, []byte("fixture-step"), 0o600); err != nil {
		t.Fatal(err)
	}
	runner := &fakeRunner{}
	worker := newTestWorker(t, runner, Config{
		Fixtures: map[string]Fixture{
			"test_fixture": {Filename: "generic_fixture.step", Path: fixturePath},
		},
	})
	result, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{FixtureID: "test_fixture"},
	})
	if err != nil {
		t.Fatalf("fixture topology: %v", err)
	}
	if result.Fingerprint != FingerprintSTEP([]byte("fixture-step")) {
		t.Fatalf("fixture fingerprint = %q", result.Fingerprint)
	}
	if runner.callCount() != 1 {
		t.Fatalf("runner calls = %d", runner.callCount())
	}
}

func TestWorkerEnforcesPayloadAndSequenceBounds(t *testing.T) {
	worker := newTestWorker(t, &fakeRunner{}, Config{
		MaxSTEPBytes:     4,
		MaxSequenceSeams: 1,
	})
	_, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{Filename: "large.step", Bytes: []byte("12345")},
	})
	assertWorkerError(t, err, ErrorPayloadTooLarge, ErrPayloadTooLarge)

	_, err = worker.BuildSequence(context.Background(), SequenceRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
		Segments: []SequenceSegment{
			{EdgeID: "edge-a", Start: 0, End: 1},
			{EdgeID: "edge-b", Start: 0, End: 1},
		},
	})
	assertWorkerError(t, err, ErrorInvalidRequest, ErrInvalidRequest)

	_, err = worker.BuildSeam(context.Background(), SeamRequest{
		STEP:   STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
		Window: &SeamWindow{Start: 0.5, End: 0.5},
	})
	assertWorkerError(t, err, ErrorInvalidRequest, ErrInvalidRequest)
}

func TestWorkerRejectsConcurrentWorkAboveLimit(t *testing.T) {
	runner := &fakeRunner{
		started: make(chan struct{}, 1),
		release: make(chan struct{}),
	}
	worker := newTestWorker(t, runner, Config{MaxConcurrent: 1})
	firstDone := make(chan error, 1)
	go func() {
		_, err := worker.ExtractTopology(context.Background(), TopologyRequest{
			STEP: STEPInput{Filename: "first.step", Bytes: []byte("first")},
		})
		firstDone <- err
	}()
	select {
	case <-runner.started:
	case <-time.After(time.Second):
		t.Fatal("first request did not reach runner")
	}
	_, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{Filename: "second.step", Bytes: []byte("second")},
	})
	assertWorkerError(t, err, ErrorBusy, ErrBusy)
	close(runner.release)
	if err := <-firstDone; err != nil {
		t.Fatalf("first request: %v", err)
	}
}

func TestWorkerAppliesTimeout(t *testing.T) {
	runner := &fakeRunner{
		release: make(chan struct{}),
	}
	worker := newTestWorker(t, runner, Config{Timeout: 20 * time.Millisecond})
	_, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
	})
	assertWorkerError(t, err, ErrorTimeout, ErrTimeout)
}

func TestWorkerRejectsAlreadyCanceledContextBeforeRunning(t *testing.T) {
	runner := &fakeRunner{}
	worker := newTestWorker(t, runner, Config{})
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	_, err := worker.ExtractTopology(ctx, TopologyRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
	})
	assertWorkerError(t, err, ErrorCanceled, ErrCanceled)
	if runner.callCount() != 0 {
		t.Fatalf("runner calls = %d, want 0", runner.callCount())
	}
}

func TestExecuteJSONSupportsPythonAliasesAndValidatesInput(t *testing.T) {
	runner := &fakeRunner{}
	worker := newTestWorker(t, runner, Config{})
	raw, err := json.Marshal(map[string]any{
		"operation":       "build_seam",
		"filename":        "part.step",
		"stepBase64":      base64.StdEncoding.EncodeToString([]byte("STEP")),
		"sampleCount":     32,
		"edgeId":          "part_0:edge_00001",
		"startS":          0.2,
		"endS":            0.8,
		"seamSampleCount": 24,
	})
	if err != nil {
		t.Fatal(err)
	}
	result, err := worker.ExecuteJSON(context.Background(), raw)
	if err != nil {
		t.Fatalf("execute JSON: %v", err)
	}
	if result.Operation != OperationBuildSeam {
		t.Fatalf("operation = %q", result.Operation)
	}

	_, err = worker.ExecuteJSON(context.Background(), []byte(`{"operation":"extract_topology","unknown":true}`))
	assertWorkerError(t, err, ErrorInvalidRequest, ErrInvalidRequest)

	_, err = worker.ExecuteJSON(context.Background(), []byte(`{} {}`))
	assertWorkerError(t, err, ErrorInvalidRequest, ErrInvalidRequest)
}

func TestWorkerRejectsMismatchedResponseFingerprint(t *testing.T) {
	runner := &fakeRunner{
		run: func(context.Context, Command) ([]byte, []byte, error) {
			return []byte(`{"fingerprint":"wrong","unit":"mm","edges":[]}`), nil, nil
		},
	}
	worker := newTestWorker(t, runner, Config{})
	_, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
	})
	assertWorkerError(t, err, ErrorInvalidResponse, ErrInvalidResponse)
}

func TestWorkerDogBoxFixtureIntegration(t *testing.T) {
	if os.Getenv("OFFLINE_PROGRAMMING_CAD_INTEGRATION") != "1" {
		t.Skip("set OFFLINE_PROGRAMMING_CAD_INTEGRATION=1 when the CadQuery Python environment is available")
	}
	repositoryRoot, err := filepath.Abs(filepath.Join("..", "..", "..", ".."))
	if err != nil {
		t.Fatal(err)
	}
	config := DefaultConfig(repositoryRoot)
	config.PythonExecutable = os.Getenv("OFFLINE_PROGRAMMING_CAD_PYTHON")
	if config.PythonExecutable == "" {
		config.PythonExecutable = "python3"
	}
	config.Timeout = 4 * time.Minute
	t.Setenv("CADQUERY_SKIP_COLLISION_CHECK", "1")
	worker, err := NewWorker(config)
	if err != nil {
		t.Fatalf("NewWorker: %v", err)
	}
	result, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP:        STEPInput{FixtureID: DogBoxFixtureID},
		SampleCount: 16,
	})
	if err != nil {
		t.Fatalf("Dog Box topology: %v", err)
	}
	var topology struct {
		Fingerprint        string `json:"fingerprint"`
		SeamCandidateCount int    `json:"seam_candidate_count"`
	}
	if err := json.Unmarshal(result.JSON, &topology); err != nil {
		t.Fatal(err)
	}
	if topology.Fingerprint != result.Fingerprint {
		t.Fatalf("response fingerprint = %q, worker = %q", topology.Fingerprint, result.Fingerprint)
	}
	if topology.SeamCandidateCount != 10 {
		t.Fatalf("Dog Box seam candidates = %d, want 10", topology.SeamCandidateCount)
	}
	cached, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP:        STEPInput{FixtureID: DogBoxFixtureID},
		SampleCount: 16,
	})
	if err != nil {
		t.Fatalf("cached Dog Box topology: %v", err)
	}
	if !cached.CacheHit {
		t.Fatal("second Dog Box request was not a cache hit")
	}
}

func TestWorkerTessellateCachesUnderTheSameFingerprintAsTopology(t *testing.T) {
	runner := &fakeRunner{}
	worker := newTestMeshWorker(t, runner, Config{})
	step := []byte("ISO-10303-21; MESHABLE STEP")

	topology, err := worker.ExtractTopology(context.Background(), TopologyRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: step},
	})
	if err != nil {
		t.Fatalf("topology: %v", err)
	}
	mesh, err := worker.Tessellate(context.Background(), MeshRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: step},
	})
	if err != nil {
		t.Fatalf("tessellate: %v", err)
	}
	if mesh.Operation != OperationTessellate {
		t.Fatalf("operation = %q", mesh.Operation)
	}
	if mesh.CacheHit {
		t.Fatal("first mesh unexpectedly came from cache")
	}
	if mesh.Fingerprint != topology.Fingerprint {
		t.Fatalf("mesh fingerprint = %q, topology = %q", mesh.Fingerprint, topology.Fingerprint)
	}

	// A repeat request must hit the LRU, and a different sample count must not
	// split the mesh cache: tessellation does not sample edges.
	repeat, err := worker.Tessellate(context.Background(), MeshRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: step},
	})
	if err != nil {
		t.Fatalf("cached tessellate: %v", err)
	}
	if !repeat.CacheHit {
		t.Fatal("second mesh request was not a cache hit")
	}
	if calls := runner.callCount(); calls != 2 {
		t.Fatalf("runner calls = %d, want 2 (one topology, one mesh)", calls)
	}

	// A coarser deflection is a distinct mesh and must run the worker again.
	coarse, err := worker.Tessellate(context.Background(), MeshRequest{
		STEP:               STEPInput{Filename: "part.step", Bytes: step},
		LinearDeflectionMM: 2,
	})
	if err != nil {
		t.Fatalf("coarse tessellate: %v", err)
	}
	if coarse.CacheHit {
		t.Fatal("coarse mesh unexpectedly came from cache")
	}
	if stats := worker.CacheStats(); stats.Entries != 3 {
		t.Fatalf("cache entries = %d, want 3", stats.Entries)
	}
	worker.PurgeFingerprint(mesh.Fingerprint)
	if stats := worker.CacheStats(); stats.Entries != 0 {
		t.Fatalf("cache after purge = %+v", stats)
	}
}

func TestWorkerTessellateRunsTheMeshScriptWithItsOwnTimeout(t *testing.T) {
	var meshCommand Command
	runner := &fakeRunner{
		run: func(_ context.Context, command Command) ([]byte, []byte, error) {
			meshCommand = command
			return successfulResponse(command)
		},
	}
	worker := newTestMeshWorker(t, runner, Config{
		Timeout:          20 * time.Millisecond,
		MeshTimeout:      2 * time.Minute,
		PythonExecutable: "python3",
	})
	if !worker.MeshAvailable() {
		t.Fatal("MeshAvailable = false for a configured mesh script")
	}
	if _, err := worker.Tessellate(context.Background(), MeshRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
	}); err != nil {
		t.Fatalf("tessellate: %v", err)
	}
	if len(meshCommand.Args) == 0 ||
		!strings.HasSuffix(filepath.ToSlash(meshCommand.Args[0]), "tessellate_step.py") {
		t.Fatalf("mesh script args = %v", meshCommand.Args)
	}
	if meshCommand.Args[len(meshCommand.Args)-1] != "--stdin" {
		t.Fatalf("mesh command does not use --stdin: %v", meshCommand.Args)
	}
	if meshCommand.Dir != worker.cadQueryRoot {
		t.Fatalf("mesh command dir = %q, want the CadQuery root %q", meshCommand.Dir, worker.cadQueryRoot)
	}
	var payload struct {
		Operation            Operation `json:"operation"`
		LinearDeflectionMM   float64   `json:"linear_deflection_mm"`
		AngularDeflectionRad float64   `json:"angular_deflection_rad"`
		MaxTriangles         int       `json:"max_triangles"`
	}
	if err := json.Unmarshal(meshCommand.Stdin, &payload); err != nil {
		t.Fatal(err)
	}
	if payload.Operation != OperationTessellate ||
		payload.LinearDeflectionMM != DefaultLinearDeflectionMM ||
		payload.AngularDeflectionRad != DefaultAngularDeflectionRad ||
		payload.MaxTriangles != DefaultMaxMeshTriangles {
		t.Fatalf("mesh request payload = %+v", payload)
	}

	// The sampled-topology timeout is 20 ms here; the mesh ceiling is separate,
	// so a slow mesh run is bounded by MeshTimeout rather than Timeout.
	if got := worker.operationTimeout(OperationTessellate); got != 2*time.Minute {
		t.Fatalf("mesh timeout = %s", got)
	}
	if got := worker.operationTimeout(OperationExtractTopology); got != 20*time.Millisecond {
		t.Fatalf("topology timeout = %s", got)
	}
}

func TestWorkerTessellateEnforcesMeshBounds(t *testing.T) {
	worker := newTestMeshWorker(t, &fakeRunner{}, Config{MaxMeshTriangles: 5_000})
	step := STEPInput{Filename: "part.step", Bytes: []byte("STEP")}
	for _, test := range []struct {
		name    string
		request MeshRequest
	}{
		{"linear too fine", MeshRequest{STEP: step, LinearDeflectionMM: 0.0001}},
		{"linear too coarse", MeshRequest{STEP: step, LinearDeflectionMM: 100}},
		{"angular too fine", MeshRequest{STEP: step, AngularDeflectionRad: 0.001}},
		{"angular too coarse", MeshRequest{STEP: step, AngularDeflectionRad: 9}},
		{"triangles below floor", MeshRequest{STEP: step, MaxTriangles: 10}},
		{"triangles above worker cap", MeshRequest{STEP: step, MaxTriangles: 6_000}},
	} {
		_, err := worker.Tessellate(context.Background(), test.request)
		assertWorkerError(t, err, ErrorInvalidRequest, ErrInvalidRequest)
	}
}

func TestWorkerReportsTessellationUnavailableWithoutAMeshScript(t *testing.T) {
	runner := &fakeRunner{}
	worker := newTestWorker(t, runner, Config{})
	if worker.MeshAvailable() {
		t.Fatal("MeshAvailable = true without a configured mesh script")
	}
	_, err := worker.Tessellate(context.Background(), MeshRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
	})
	assertWorkerError(t, err, ErrorUnavailable, ErrUnavailable)
	if runner.callCount() != 0 {
		t.Fatalf("runner calls = %d, want 0", runner.callCount())
	}
}

func TestWorkerRejectsMeshResponseWithoutMeshResult(t *testing.T) {
	runner := &fakeRunner{
		run: func(_ context.Context, command Command) ([]byte, []byte, error) {
			var request struct {
				STEPBase64 string `json:"step_base64"`
			}
			if err := json.Unmarshal(command.Stdin, &request); err != nil {
				return nil, nil, err
			}
			step, err := base64.StdEncoding.DecodeString(request.STEPBase64)
			if err != nil {
				return nil, nil, err
			}
			// Correct fingerprint and unit, but the edges of a topology reply.
			return []byte(`{"fingerprint":"` + FingerprintSTEP(step) + `","unit":"mm","edges":[]}`), nil, nil
		},
	}
	worker := newTestMeshWorker(t, runner, Config{})
	_, err := worker.Tessellate(context.Background(), MeshRequest{
		STEP: STEPInput{Filename: "part.step", Bytes: []byte("STEP")},
	})
	assertWorkerError(t, err, ErrorInvalidResponse, ErrInvalidResponse)
}

func TestExecuteJSONRoutesTessellateWithAliases(t *testing.T) {
	var stdin []byte
	runner := &fakeRunner{
		run: func(_ context.Context, command Command) ([]byte, []byte, error) {
			stdin = append([]byte(nil), command.Stdin...)
			return successfulResponse(command)
		},
	}
	worker := newTestMeshWorker(t, runner, Config{})
	raw, err := json.Marshal(map[string]any{
		"operation":            "tessellate",
		"filename":             "part.step",
		"stepBase64":           base64.StdEncoding.EncodeToString([]byte("STEP")),
		"linearDeflectionMm":   0.25,
		"angularDeflectionRad": 0.4,
		"maxTriangles":         12_000,
	})
	if err != nil {
		t.Fatal(err)
	}
	result, err := worker.ExecuteJSON(context.Background(), raw)
	if err != nil {
		t.Fatalf("execute JSON: %v", err)
	}
	if result.Operation != OperationTessellate {
		t.Fatalf("operation = %q", result.Operation)
	}
	var payload struct {
		LinearDeflectionMM   float64 `json:"linear_deflection_mm"`
		AngularDeflectionRad float64 `json:"angular_deflection_rad"`
		MaxTriangles         int     `json:"max_triangles"`
	}
	if err := json.Unmarshal(stdin, &payload); err != nil {
		t.Fatal(err)
	}
	if payload.LinearDeflectionMM != 0.25 ||
		payload.AngularDeflectionRad != 0.4 ||
		payload.MaxTriangles != 12_000 {
		t.Fatalf("camelCase aliases were not forwarded: %+v", payload)
	}
}

func TestNewWorkerRejectsAMissingConfiguredMeshScript(t *testing.T) {
	root := t.TempDir()
	script := filepath.Join(root, "python", "extract_weld_edges.py")
	if err := os.MkdirAll(filepath.Dir(script), 0o700); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(script, []byte("# test fixture\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	_, err := NewWorker(Config{
		CadQueryRoot:   root,
		MeshScriptPath: filepath.Join(root, "python", "absent_tessellate.py"),
		Runner:         &fakeRunner{},
	})
	if err == nil {
		t.Fatal("NewWorker accepted a mesh script path that does not exist")
	}
}

func TestDefaultConfigPointsAtThisProductsMeshScript(t *testing.T) {
	repositoryRoot, err := filepath.Abs(filepath.Join("..", "..", "..", ".."))
	if err != nil {
		t.Fatal(err)
	}
	config := DefaultConfig(repositoryRoot)
	if info, err := os.Stat(config.MeshScriptPath); err != nil || !info.Mode().IsRegular() {
		t.Fatalf("default mesh script %s is not a regular file: %v", config.MeshScriptPath, err)
	}
}

func TestWorkerDogBoxMeshIntegration(t *testing.T) {
	if os.Getenv("OFFLINE_PROGRAMMING_CAD_INTEGRATION") != "1" {
		t.Skip("set OFFLINE_PROGRAMMING_CAD_INTEGRATION=1 when the CadQuery Python environment is available")
	}
	repositoryRoot, err := filepath.Abs(filepath.Join("..", "..", "..", ".."))
	if err != nil {
		t.Fatal(err)
	}
	config := DefaultConfig(repositoryRoot)
	config.PythonExecutable = os.Getenv("OFFLINE_PROGRAMMING_CAD_PYTHON")
	if config.PythonExecutable == "" {
		config.PythonExecutable = "python3"
	}
	config.MeshTimeout = 6 * time.Minute
	worker, err := NewWorker(config)
	if err != nil {
		t.Fatalf("NewWorker: %v", err)
	}
	result, err := worker.Tessellate(context.Background(), MeshRequest{
		STEP: STEPInput{FixtureID: DogBoxFixtureID},
	})
	if err != nil {
		t.Fatalf("Dog Box mesh: %v", err)
	}
	var payload struct {
		Fingerprint string `json:"fingerprint"`
		Unit        string `json:"unit"`
		Mesh        struct {
			Positions     []float64 `json:"positions"`
			Indices       []int     `json:"indices"`
			Normals       []float64 `json:"normals"`
			VertexCount   int       `json:"vertex_count"`
			TriangleCount int       `json:"triangle_count"`
			SolidCount    int       `json:"solid_count"`
			Truncated     bool      `json:"truncated"`
		} `json:"mesh"`
	}
	if err := json.Unmarshal(result.JSON, &payload); err != nil {
		t.Fatal(err)
	}
	if payload.Fingerprint != result.Fingerprint || payload.Unit != "mm" {
		t.Fatalf("fingerprint = %q (worker %q), unit = %q", payload.Fingerprint, result.Fingerprint, payload.Unit)
	}
	if payload.Mesh.TriangleCount <= 0 || len(payload.Mesh.Indices) != payload.Mesh.TriangleCount*3 {
		t.Fatalf("triangle count = %d, indices = %d", payload.Mesh.TriangleCount, len(payload.Mesh.Indices))
	}
	if payload.Mesh.VertexCount <= 0 || len(payload.Mesh.Positions) != payload.Mesh.VertexCount*3 {
		t.Fatalf("vertex count = %d, positions = %d", payload.Mesh.VertexCount, len(payload.Mesh.Positions))
	}
	if len(payload.Mesh.Normals) != len(payload.Mesh.Positions) {
		t.Fatalf("normals = %d, positions = %d", len(payload.Mesh.Normals), len(payload.Mesh.Positions))
	}
	for index, value := range payload.Mesh.Indices {
		if value < 0 || value >= payload.Mesh.VertexCount {
			t.Fatalf("indices[%d] = %d is outside [0, %d)", index, value, payload.Mesh.VertexCount)
		}
	}
	if payload.Mesh.Truncated {
		t.Fatalf("Dog Box mesh was truncated at %d triangles", payload.Mesh.TriangleCount)
	}
	t.Logf(
		"Dog Box mesh: %d triangles, %d vertices, %d solids, %d bytes of JSON",
		payload.Mesh.TriangleCount, payload.Mesh.VertexCount, payload.Mesh.SolidCount, len(result.JSON),
	)
}

func newTestWorker(t *testing.T, runner CommandRunner, overrides Config) *Worker {
	t.Helper()
	root := t.TempDir()
	script := filepath.Join(root, "python", "extract_weld_edges.py")
	if err := os.MkdirAll(filepath.Dir(script), 0o700); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(script, []byte("# test fixture\n"), 0o600); err != nil {
		t.Fatal(err)
	}
	config := overrides
	config.CadQueryRoot = root
	config.Runner = runner
	worker, err := NewWorker(config)
	if err != nil {
		t.Fatalf("NewWorker: %v", err)
	}
	return worker
}

// newTestMeshWorker adds a stand-in tessellation script so mesh requests reach
// the runner instead of failing as unavailable.
func newTestMeshWorker(t *testing.T, runner CommandRunner, overrides Config) *Worker {
	t.Helper()
	root := t.TempDir()
	for _, name := range []string{"extract_weld_edges.py", "tessellate_step.py"} {
		script := filepath.Join(root, "python", name)
		if err := os.MkdirAll(filepath.Dir(script), 0o700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(script, []byte("# test fixture\n"), 0o600); err != nil {
			t.Fatal(err)
		}
	}
	config := overrides
	config.CadQueryRoot = root
	config.MeshScriptPath = filepath.Join(root, "python", "tessellate_step.py")
	config.Runner = runner
	worker, err := NewWorker(config)
	if err != nil {
		t.Fatalf("NewWorker: %v", err)
	}
	return worker
}

func successfulResponse(command Command) ([]byte, []byte, error) {
	var request struct {
		Operation  Operation `json:"operation"`
		STEPBase64 string    `json:"step_base64"`
	}
	if err := json.Unmarshal(command.Stdin, &request); err != nil {
		return nil, nil, err
	}
	step, err := base64.StdEncoding.DecodeString(request.STEPBase64)
	if err != nil {
		return nil, nil, err
	}
	response := map[string]any{
		"fingerprint": FingerprintSTEP(step),
		"unit":        "mm",
	}
	switch request.Operation {
	case OperationExtractTopology:
		response["edges"] = []any{}
	case OperationBuildSeam:
		response["seam"] = map[string]any{}
	case OperationBuildSequence:
		response["weld_sequence"] = map[string]any{}
	case OperationTessellate:
		response["mesh"] = map[string]any{
			"positions":      []float64{0, 0, 0, 1, 0, 0, 0, 1, 0},
			"indices":        []int{0, 1, 2},
			"vertex_count":   3,
			"triangle_count": 1,
		}
	}
	raw, err := json.Marshal(response)
	return raw, nil, err
}

func assertWorkerError(t *testing.T, err error, code ErrorCode, sentinel error) {
	t.Helper()
	if err == nil {
		t.Fatalf("expected %s error", code)
	}
	var workerErr *WorkerError
	if !errors.As(err, &workerErr) {
		t.Fatalf("error type = %T, want *WorkerError: %v", err, err)
	}
	if workerErr.Code != code {
		t.Fatalf("error code = %q, want %q: %v", workerErr.Code, code, err)
	}
	if !errors.Is(err, sentinel) {
		t.Fatalf("errors.Is(%v) = false for %v", err, sentinel)
	}
}
