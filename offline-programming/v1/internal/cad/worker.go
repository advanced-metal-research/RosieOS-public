package cad

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"time"
)

const (
	DefaultTimeout          = 90 * time.Second
	DefaultMaxRequestBytes  = int64(12 << 20)
	DefaultMaxSTEPBytes     = int64(8 << 20)
	DefaultMaxOutputBytes   = int64(64 << 20)
	DefaultMaxConcurrent    = 1
	DefaultMaxCacheEntries  = 24
	DefaultMaxCacheBytes    = int64(96 << 20)
	DefaultSampleCount      = 64
	DefaultSeamSampleCount  = 40
	MaxSampleCount          = 256
	MaxSeamSampleCount      = 512
	DefaultMaxSequenceSeams = 64
	DogBoxFixtureID         = "dog_box"

	// Tessellation is dominated by OCCT meshing on large assemblies, so it gets
	// a longer ceiling than the sampled-topology operations.
	DefaultMeshTimeout              = 240 * time.Second
	DefaultLinearDeflectionMM       = 0.5
	DefaultAngularDeflectionRad     = 0.5
	MinLinearDeflectionMM           = 0.01
	MaxLinearDeflectionMM           = 10.0
	MinAngularDeflectionRad         = 0.05
	MaxAngularDeflectionRad         = 1.5
	DefaultMaxMeshTriangles         = 400_000
	MaxMeshTriangles                = 2_000_000
	MinMeshTriangles                = 1_000
	defaultMeshScriptRelativeToRepo = "offline-programming/v1/internal/cad/python/tessellate_step.py"
)

// Config controls every resource bound and external path used by Worker.
type Config struct {
	CadQueryRoot string
	ScriptPath   string
	// MeshScriptPath implements the tessellate operation. It is a separate
	// script from ScriptPath so the mesh path lives inside this product rather
	// than in the shared cadquery/v1 worker; it still runs with CadQueryRoot as
	// its working directory so it resolves the same Python environment.
	MeshScriptPath   string
	PythonExecutable string
	PixiExecutable   string
	Timeout          time.Duration
	MeshTimeout      time.Duration
	MaxRequestBytes  int64
	MaxSTEPBytes     int64
	MaxOutputBytes   int64
	MaxConcurrent    int
	MaxSequenceSeams int
	MaxMeshTriangles int
	MaxCacheEntries  int
	MaxCacheBytes    int64
	Fixtures         map[string]Fixture
	Runner           CommandRunner
}

// DefaultConfig returns repository-layout defaults and registers Dog Box as a
// normal fixture. Callers may replace or extend Fixtures without changing the
// worker.
func DefaultConfig(repositoryRoot string) Config {
	cadQueryRoot := filepath.Join(repositoryRoot, "cadquery", "v1")
	return Config{
		CadQueryRoot:   cadQueryRoot,
		MeshScriptPath: filepath.Join(repositoryRoot, filepath.FromSlash(defaultMeshScriptRelativeToRepo)),
		Fixtures: map[string]Fixture{
			DogBoxFixtureID: {
				Filename: "dog_box.step",
				Path:     filepath.Join(cadQueryRoot, "step", "dog_box.step"),
			},
		},
	}
}

// Worker invokes the existing CadQuery v1 Python implementation behind
// explicit payload, timeout, concurrency, output, and cache bounds.
type Worker struct {
	cadQueryRoot     string
	scriptPath       string
	meshScriptPath   string
	pythonExecutable string
	pixiExecutable   string
	timeout          time.Duration
	meshTimeout      time.Duration
	maxRequestBytes  int64
	maxSTEPBytes     int64
	maxOutputBytes   int64
	maxSequenceSeams int
	maxMeshTriangles int
	fixtures         map[string]Fixture
	gate             chan struct{}
	runner           CommandRunner
	cache            *responseCache
}

// NewWorker validates configuration without starting a subprocess.
func NewWorker(config Config) (*Worker, error) {
	if strings.TrimSpace(config.CadQueryRoot) == "" {
		return nil, errors.New("cad: CadQueryRoot is required")
	}
	config.CadQueryRoot = filepath.Clean(config.CadQueryRoot)
	if config.ScriptPath == "" {
		config.ScriptPath = filepath.Join("python", "extract_weld_edges.py")
	}
	scriptPath := config.ScriptPath
	if !filepath.IsAbs(scriptPath) {
		scriptPath = filepath.Join(config.CadQueryRoot, scriptPath)
	}
	if info, err := os.Stat(scriptPath); err != nil || !info.Mode().IsRegular() {
		if err == nil {
			err = errors.New("not a regular file")
		}
		return nil, fmt.Errorf("cad: worker script %s: %w", scriptPath, err)
	}
	// An empty MeshScriptPath disables tessellation with a stable "unavailable"
	// code; a configured one must exist, so a packaging mistake fails loudly at
	// construction rather than on a user's first import.
	meshScriptPath := strings.TrimSpace(config.MeshScriptPath)
	if meshScriptPath != "" {
		if !filepath.IsAbs(meshScriptPath) {
			meshScriptPath = filepath.Join(config.CadQueryRoot, meshScriptPath)
		}
		if info, err := os.Stat(meshScriptPath); err != nil || !info.Mode().IsRegular() {
			if err == nil {
				err = errors.New("not a regular file")
			}
			return nil, fmt.Errorf("cad: mesh worker script %s: %w", meshScriptPath, err)
		}
	}
	if config.Timeout <= 0 {
		config.Timeout = DefaultTimeout
	}
	if config.MeshTimeout <= 0 {
		config.MeshTimeout = DefaultMeshTimeout
	}
	if config.MaxMeshTriangles <= 0 {
		config.MaxMeshTriangles = DefaultMaxMeshTriangles
	}
	if config.MaxMeshTriangles > MaxMeshTriangles {
		config.MaxMeshTriangles = MaxMeshTriangles
	}
	if config.MaxMeshTriangles < MinMeshTriangles {
		config.MaxMeshTriangles = MinMeshTriangles
	}
	if config.MaxRequestBytes <= 0 {
		config.MaxRequestBytes = DefaultMaxRequestBytes
	}
	if config.MaxSTEPBytes <= 0 {
		config.MaxSTEPBytes = DefaultMaxSTEPBytes
	}
	if config.MaxOutputBytes <= 0 {
		config.MaxOutputBytes = DefaultMaxOutputBytes
	}
	if config.MaxConcurrent <= 0 {
		config.MaxConcurrent = DefaultMaxConcurrent
	}
	if config.MaxSequenceSeams <= 0 {
		config.MaxSequenceSeams = DefaultMaxSequenceSeams
	}
	if config.MaxCacheEntries < 0 {
		return nil, errors.New("cad: MaxCacheEntries cannot be negative")
	}
	if config.MaxCacheEntries == 0 {
		config.MaxCacheEntries = DefaultMaxCacheEntries
	}
	if config.MaxCacheBytes < 0 {
		return nil, errors.New("cad: MaxCacheBytes cannot be negative")
	}
	if config.MaxCacheBytes == 0 {
		config.MaxCacheBytes = DefaultMaxCacheBytes
	}
	if config.Runner == nil {
		config.Runner = ExecRunner{}
	}
	if config.PythonExecutable == "" {
		config.PythonExecutable = strings.TrimSpace(os.Getenv("CADQUERY_TOPOLOGY_PYTHON"))
	}
	if config.PixiExecutable == "" {
		config.PixiExecutable = "pixi"
	}
	fixtures := make(map[string]Fixture, len(config.Fixtures))
	for id, fixture := range config.Fixtures {
		id = strings.TrimSpace(id)
		if id == "" {
			return nil, errors.New("cad: fixture ID cannot be empty")
		}
		if strings.TrimSpace(fixture.Filename) == "" || strings.TrimSpace(fixture.Path) == "" {
			return nil, fmt.Errorf("cad: fixture %q requires filename and path", id)
		}
		fixtures[id] = Fixture{
			Filename: strings.TrimSpace(fixture.Filename),
			Path:     filepath.Clean(fixture.Path),
		}
	}
	return &Worker{
		cadQueryRoot:     config.CadQueryRoot,
		scriptPath:       scriptPath,
		meshScriptPath:   meshScriptPath,
		pythonExecutable: config.PythonExecutable,
		pixiExecutable:   config.PixiExecutable,
		timeout:          config.Timeout,
		meshTimeout:      config.MeshTimeout,
		maxRequestBytes:  config.MaxRequestBytes,
		maxSTEPBytes:     config.MaxSTEPBytes,
		maxOutputBytes:   config.MaxOutputBytes,
		maxSequenceSeams: config.MaxSequenceSeams,
		maxMeshTriangles: config.MaxMeshTriangles,
		fixtures:         fixtures,
		gate:             make(chan struct{}, config.MaxConcurrent),
		runner:           config.Runner,
		cache:            newResponseCache(config.MaxCacheEntries, config.MaxCacheBytes),
	}, nil
}

// ExtractTopology invokes the extract_topology operation.
func (worker *Worker) ExtractTopology(ctx context.Context, request TopologyRequest) (Result, error) {
	return worker.execute(ctx, invocation{
		operation:   OperationExtractTopology,
		step:        request.STEP,
		sampleCount: request.SampleCount,
	})
}

// BuildSeam invokes the build_seam operation.
func (worker *Worker) BuildSeam(ctx context.Context, request SeamRequest) (Result, error) {
	return worker.execute(ctx, invocation{
		operation:       OperationBuildSeam,
		step:            request.STEP,
		sampleCount:     request.SampleCount,
		edgeID:          request.EdgeID,
		window:          request.Window,
		seamSampleCount: request.SeamSampleCount,
		reversed:        request.Reversed,
		geometryMethod:  request.GeometryMethod,
		pathMethod:      request.PathMethod,
	})
}

// BuildSequence invokes the build_sequence operation.
func (worker *Worker) BuildSequence(ctx context.Context, request SequenceRequest) (Result, error) {
	return worker.execute(ctx, invocation{
		operation:       OperationBuildSequence,
		step:            request.STEP,
		sampleCount:     request.SampleCount,
		seamSampleCount: request.SeamSampleCount,
		segments:        request.Segments,
	})
}

// Tessellate invokes the tessellate operation, producing surface triangles for
// the workstation viewport under the same STEP fingerprint identity as
// ExtractTopology.
func (worker *Worker) Tessellate(ctx context.Context, request MeshRequest) (Result, error) {
	return worker.execute(ctx, invocation{
		operation:            OperationTessellate,
		step:                 request.STEP,
		linearDeflectionMM:   request.LinearDeflectionMM,
		angularDeflectionRad: request.AngularDeflectionRad,
		maxTriangles:         request.MaxTriangles,
	})
}

// MeshAvailable reports whether this worker was configured with a tessellation
// script. Callers use it to degrade without provoking an error round trip.
func (worker *Worker) MeshAvailable() bool {
	return worker.meshScriptPath != ""
}

// ExecuteJSON validates a transport request before routing it through the typed
// worker. It is the integration hook for a later HTTP adapter.
func (worker *Worker) ExecuteJSON(ctx context.Context, raw []byte) (Result, error) {
	if len(raw) == 0 {
		return Result{}, workerError(ErrorInvalidRequest, "", "request body is empty", ErrInvalidRequest)
	}
	if int64(len(raw)) > worker.maxRequestBytes {
		return Result{}, workerError(
			ErrorPayloadTooLarge,
			"",
			fmt.Sprintf("request exceeds the %d byte limit", worker.maxRequestBytes),
			ErrPayloadTooLarge,
		)
	}
	var request transportRequest
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&request); err != nil {
		return Result{}, workerError(ErrorInvalidRequest, "", fmt.Sprintf("invalid JSON: %v", err), ErrInvalidRequest)
	}
	if err := requireJSONEOF(decoder); err != nil {
		return Result{}, workerError(ErrorInvalidRequest, "", err.Error(), ErrInvalidRequest)
	}
	step, err := request.stepInput()
	if err != nil {
		return Result{}, workerError(ErrorInvalidRequest, request.Operation, err.Error(), ErrInvalidRequest)
	}
	switch request.Operation {
	case "", OperationExtractTopology:
		return worker.ExtractTopology(ctx, TopologyRequest{
			STEP:        step,
			SampleCount: firstInt(request.SampleCount, request.SampleCountCamel),
		})
	case OperationBuildSeam:
		window := &SeamWindow{
			Start: firstFloat(request.Start, request.StartCamel, 0),
			End:   firstFloat(request.End, request.EndCamel, 1),
		}
		return worker.BuildSeam(ctx, SeamRequest{
			STEP:            step,
			SampleCount:     firstInt(request.SampleCount, request.SampleCountCamel),
			EdgeID:          firstString(request.EdgeID, request.EdgeIDCamel),
			Window:          window,
			SeamSampleCount: firstInt(request.SeamSampleCount, request.SeamSampleCountCamel),
			Reversed:        firstBool(request.Reversed, request.Reverse),
			GeometryMethod:  firstString(request.GeometryMethod, request.GeometryMethodCamel),
			PathMethod:      firstString(request.PathMethod, request.PathMethodCamel),
		})
	case OperationBuildSequence:
		segments, err := request.normalizedSegments()
		if err != nil {
			return Result{}, workerError(ErrorInvalidRequest, request.Operation, err.Error(), ErrInvalidRequest)
		}
		return worker.BuildSequence(ctx, SequenceRequest{
			STEP:            step,
			SampleCount:     firstInt(request.SampleCount, request.SampleCountCamel),
			SeamSampleCount: firstInt(request.SeamSampleCount, request.SeamSampleCountCamel),
			Segments:        segments,
		})
	case OperationTessellate:
		return worker.Tessellate(ctx, MeshRequest{
			STEP:                 step,
			LinearDeflectionMM:   firstFloat(request.LinearDeflectionMM, request.LinearDeflectionMMCamel, 0),
			AngularDeflectionRad: firstFloat(request.AngularDeflectionRad, request.AngularDeflectionRadCamel, 0),
			MaxTriangles:         firstInt(request.MaxTriangles, request.MaxTrianglesCamel),
		})
	default:
		return Result{}, workerError(
			ErrorInvalidRequest,
			request.Operation,
			fmt.Sprintf("unsupported operation %q", request.Operation),
			ErrInvalidRequest,
		)
	}
}

// CacheStats returns current bounded-cache usage.
func (worker *Worker) CacheStats() CacheStats {
	return worker.cache.stats()
}

// PurgeFingerprint removes every cached operation variant for one STEP SHA-256.
func (worker *Worker) PurgeFingerprint(fingerprint string) {
	worker.cache.purgeFingerprint(strings.ToLower(strings.TrimSpace(fingerprint)))
}

// FingerprintSTEP computes the cache identity used by Worker.
func FingerprintSTEP(step []byte) string {
	sum := sha256.Sum256(step)
	return hex.EncodeToString(sum[:])
}

type invocation struct {
	operation            Operation
	step                 STEPInput
	sampleCount          int
	edgeID               string
	window               *SeamWindow
	seamSampleCount      int
	reversed             bool
	geometryMethod       string
	pathMethod           string
	segments             []SequenceSegment
	linearDeflectionMM   float64
	angularDeflectionRad float64
	maxTriangles         int
}

type pythonRequest struct {
	Operation       Operation         `json:"operation"`
	Filename        string            `json:"filename"`
	STEPBase64      string            `json:"step_base64"`
	SampleCount     int               `json:"sample_count"`
	EdgeID          string            `json:"edge_id,omitempty"`
	Start           *float64          `json:"start_s,omitempty"`
	End             *float64          `json:"end_s,omitempty"`
	SeamSampleCount int               `json:"seam_sample_count,omitempty"`
	Reversed        bool              `json:"reversed,omitempty"`
	GeometryMethod  string            `json:"geometry_method,omitempty"`
	PathMethod      string            `json:"path_method,omitempty"`
	Segments        []SequenceSegment `json:"segments,omitempty"`

	LinearDeflectionMM   float64 `json:"linear_deflection_mm,omitempty"`
	AngularDeflectionRad float64 `json:"angular_deflection_rad,omitempty"`
	MaxTriangles         int     `json:"max_triangles,omitempty"`
}

func (worker *Worker) execute(ctx context.Context, request invocation) (Result, error) {
	if ctx == nil {
		ctx = context.Background()
	}
	if err := ctx.Err(); err != nil {
		return Result{}, worker.contextError(request.operation, err)
	}
	payload, fingerprint, key, err := worker.prepare(request)
	if err != nil {
		return Result{}, err
	}
	if cached, ok := worker.cache.get(key); ok {
		return Result{
			Operation:   request.operation,
			Fingerprint: fingerprint,
			JSON:        cached,
			CacheHit:    true,
		}, nil
	}
	select {
	case worker.gate <- struct{}{}:
		defer func() { <-worker.gate }()
	default:
		return Result{}, workerError(ErrorBusy, request.operation, "concurrency limit reached", ErrBusy)
	}
	if cached, ok := worker.cache.get(key); ok {
		return Result{
			Operation:   request.operation,
			Fingerprint: fingerprint,
			JSON:        cached,
			CacheHit:    true,
		}, nil
	}
	runCtx, cancel := context.WithTimeout(ctx, worker.operationTimeout(request.operation))
	defer cancel()
	command := worker.command(request.operation, payload)
	stdout, stderr, runErr := worker.runner.Run(runCtx, command)
	if runErr != nil {
		return Result{}, worker.classifyRunError(request.operation, runCtx, runErr, stderr)
	}
	result := bytes.TrimSpace(stdout)
	if int64(len(result)) > worker.maxOutputBytes {
		return Result{}, workerError(
			ErrorPayloadTooLarge,
			request.operation,
			fmt.Sprintf("response exceeds the %d byte limit", worker.maxOutputBytes),
			ErrPayloadTooLarge,
		)
	}
	if err := validateResponse(request.operation, fingerprint, result); err != nil {
		return Result{}, err
	}
	worker.cache.put(key, result)
	return Result{
		Operation:   request.operation,
		Fingerprint: fingerprint,
		JSON:        append([]byte(nil), result...),
	}, nil
}

func (worker *Worker) prepare(request invocation) ([]byte, string, cacheKey, error) {
	switch request.operation {
	case OperationExtractTopology, OperationBuildSeam, OperationBuildSequence:
	case OperationTessellate:
		if worker.meshScriptPath == "" {
			return nil, "", cacheKey{}, workerError(
				ErrorUnavailable,
				request.operation,
				"tessellation is not configured for this worker",
				ErrUnavailable,
			)
		}
	default:
		return nil, "", cacheKey{}, workerError(
			ErrorInvalidRequest,
			request.operation,
			fmt.Sprintf("unsupported operation %q", request.operation),
			ErrInvalidRequest,
		)
	}
	filename, step, err := worker.resolveSTEP(request.operation, request.step)
	if err != nil {
		return nil, "", cacheKey{}, err
	}
	sampleCount := request.sampleCount
	if sampleCount == 0 || request.operation == OperationTessellate {
		// Tessellation does not sample edges. Pinning the field keeps one mesh
		// per STEP fingerprint instead of one per incidental sample count.
		sampleCount = DefaultSampleCount
	}
	if sampleCount < 8 || sampleCount > MaxSampleCount {
		return nil, "", cacheKey{}, workerError(
			ErrorInvalidRequest,
			request.operation,
			fmt.Sprintf("sample count must be between 8 and %d", MaxSampleCount),
			ErrInvalidRequest,
		)
	}
	wire := pythonRequest{
		Operation:   request.operation,
		Filename:    filename,
		STEPBase64:  base64.StdEncoding.EncodeToString(step),
		SampleCount: sampleCount,
	}
	switch request.operation {
	case OperationExtractTopology:
	case OperationTessellate:
		linear := request.linearDeflectionMM
		if linear == 0 {
			linear = DefaultLinearDeflectionMM
		}
		angular := request.angularDeflectionRad
		if angular == 0 {
			angular = DefaultAngularDeflectionRad
		}
		if math.IsNaN(linear) || math.IsInf(linear, 0) ||
			linear < MinLinearDeflectionMM || linear > MaxLinearDeflectionMM {
			return nil, "", cacheKey{}, workerError(
				ErrorInvalidRequest,
				request.operation,
				fmt.Sprintf("linear deflection must be between %g and %g mm", MinLinearDeflectionMM, MaxLinearDeflectionMM),
				ErrInvalidRequest,
			)
		}
		if math.IsNaN(angular) || math.IsInf(angular, 0) ||
			angular < MinAngularDeflectionRad || angular > MaxAngularDeflectionRad {
			return nil, "", cacheKey{}, workerError(
				ErrorInvalidRequest,
				request.operation,
				fmt.Sprintf("angular deflection must be between %g and %g rad", MinAngularDeflectionRad, MaxAngularDeflectionRad),
				ErrInvalidRequest,
			)
		}
		maxTriangles := request.maxTriangles
		if maxTriangles <= 0 {
			maxTriangles = worker.maxMeshTriangles
		}
		if maxTriangles < MinMeshTriangles || maxTriangles > worker.maxMeshTriangles {
			return nil, "", cacheKey{}, workerError(
				ErrorInvalidRequest,
				request.operation,
				fmt.Sprintf("max triangles must be between %d and %d", MinMeshTriangles, worker.maxMeshTriangles),
				ErrInvalidRequest,
			)
		}
		wire.LinearDeflectionMM = linear
		wire.AngularDeflectionRad = angular
		wire.MaxTriangles = maxTriangles
	case OperationBuildSeam:
		seamSampleCount := request.seamSampleCount
		if seamSampleCount == 0 {
			seamSampleCount = DefaultSeamSampleCount
		}
		if seamSampleCount < 2 || seamSampleCount > MaxSeamSampleCount {
			return nil, "", cacheKey{}, workerError(
				ErrorInvalidRequest,
				request.operation,
				fmt.Sprintf("seam sample count must be between 2 and %d", MaxSeamSampleCount),
				ErrInvalidRequest,
			)
		}
		window := request.window
		if window == nil {
			window = &SeamWindow{Start: 0, End: 1}
		}
		if err := validateWindow(*window); err != nil {
			return nil, "", cacheKey{}, workerError(ErrorInvalidRequest, request.operation, err.Error(), ErrInvalidRequest)
		}
		wire.EdgeID = strings.TrimSpace(request.edgeID)
		wire.Start = floatPointer(window.Start)
		wire.End = floatPointer(window.End)
		wire.SeamSampleCount = seamSampleCount
		wire.Reversed = request.reversed
		wire.GeometryMethod = strings.TrimSpace(request.geometryMethod)
		wire.PathMethod = strings.TrimSpace(request.pathMethod)
	case OperationBuildSequence:
		seamSampleCount := request.seamSampleCount
		if seamSampleCount == 0 {
			seamSampleCount = DefaultSeamSampleCount
		}
		if seamSampleCount < 2 || seamSampleCount > MaxSeamSampleCount {
			return nil, "", cacheKey{}, workerError(
				ErrorInvalidRequest,
				request.operation,
				fmt.Sprintf("seam sample count must be between 2 and %d", MaxSeamSampleCount),
				ErrInvalidRequest,
			)
		}
		if len(request.segments) > worker.maxSequenceSeams {
			return nil, "", cacheKey{}, workerError(
				ErrorInvalidRequest,
				request.operation,
				fmt.Sprintf("sequence exceeds the %d seam limit", worker.maxSequenceSeams),
				ErrInvalidRequest,
			)
		}
		wire.SeamSampleCount = seamSampleCount
		wire.Segments = make([]SequenceSegment, len(request.segments))
		for index, segment := range request.segments {
			segment.ID = strings.TrimSpace(segment.ID)
			segment.EdgeID = strings.TrimSpace(segment.EdgeID)
			if segment.EdgeID == "" {
				return nil, "", cacheKey{}, workerError(
					ErrorInvalidRequest,
					request.operation,
					fmt.Sprintf("segment %d requires edge_id", index),
					ErrInvalidRequest,
				)
			}
			if err := validateWindow(SeamWindow{Start: segment.Start, End: segment.End}); err != nil {
				return nil, "", cacheKey{}, workerError(
					ErrorInvalidRequest,
					request.operation,
					fmt.Sprintf("segment %d: %s", index, err),
					ErrInvalidRequest,
				)
			}
			wire.Segments[index] = segment
		}
	}
	payload, err := json.Marshal(wire)
	if err != nil {
		return nil, "", cacheKey{}, workerError(ErrorInvalidRequest, request.operation, err.Error(), ErrInvalidRequest)
	}
	if int64(len(payload)) > worker.maxRequestBytes {
		return nil, "", cacheKey{}, workerError(
			ErrorPayloadTooLarge,
			request.operation,
			fmt.Sprintf("encoded request exceeds the %d byte limit", worker.maxRequestBytes),
			ErrPayloadTooLarge,
		)
	}
	fingerprint := FingerprintSTEP(step)
	variant := sha256.Sum256(payloadWithoutSTEP(wire))
	return payload, fingerprint, cacheKey{fingerprint: fingerprint, variant: variant}, nil
}

func (worker *Worker) resolveSTEP(operation Operation, input STEPInput) (string, []byte, error) {
	hasBytes := input.Bytes != nil
	hasFixture := strings.TrimSpace(input.FixtureID) != ""
	if hasBytes == hasFixture {
		return "", nil, workerError(
			ErrorInvalidRequest,
			operation,
			"provide exactly one of STEP bytes or fixture_id",
			ErrInvalidRequest,
		)
	}
	filename := strings.TrimSpace(input.Filename)
	var step []byte
	if hasFixture {
		fixtureID := strings.TrimSpace(input.FixtureID)
		fixture, ok := worker.fixtures[fixtureID]
		if !ok {
			return "", nil, workerError(
				ErrorInvalidRequest,
				operation,
				fmt.Sprintf("unknown fixture %q", fixtureID),
				ErrInvalidRequest,
			)
		}
		if filename == "" {
			filename = fixture.Filename
		}
		file, err := os.Open(fixture.Path)
		if err != nil {
			return "", nil, workerError(
				ErrorUnavailable,
				operation,
				fmt.Sprintf("fixture %q is unavailable: %v", fixtureID, err),
				ErrUnavailable,
			)
		}
		defer file.Close()
		step, err = readBounded(file, worker.maxSTEPBytes)
		if err != nil {
			if errors.Is(err, ErrPayloadTooLarge) {
				return "", nil, workerError(
					ErrorPayloadTooLarge,
					operation,
					fmt.Sprintf("fixture %q exceeds the %d byte STEP limit", fixtureID, worker.maxSTEPBytes),
					ErrPayloadTooLarge,
				)
			}
			return "", nil, workerError(ErrorUnavailable, operation, fmt.Sprintf("read fixture %q: %v", fixtureID, err), ErrUnavailable)
		}
	} else {
		step = append([]byte(nil), input.Bytes...)
	}
	if len(step) == 0 {
		return "", nil, workerError(ErrorInvalidRequest, operation, "STEP payload is empty", ErrInvalidRequest)
	}
	if int64(len(step)) > worker.maxSTEPBytes {
		return "", nil, workerError(
			ErrorPayloadTooLarge,
			operation,
			fmt.Sprintf("STEP payload exceeds the %d byte limit", worker.maxSTEPBytes),
			ErrPayloadTooLarge,
		)
	}
	if filename == "" {
		filename = "upload.step"
	}
	if len(filename) > 255 || strings.ContainsAny(filename, "\x00\r\n") {
		return "", nil, workerError(ErrorInvalidRequest, operation, "invalid STEP filename", ErrInvalidRequest)
	}
	lower := strings.ToLower(filename)
	if !strings.HasSuffix(lower, ".step") && !strings.HasSuffix(lower, ".stp") {
		return "", nil, workerError(ErrorInvalidRequest, operation, "filename must end in .step or .stp", ErrInvalidRequest)
	}
	return filepath.Base(filename), step, nil
}

func (worker *Worker) command(operation Operation, payload []byte) Command {
	script := worker.scriptPath
	if operation == OperationTessellate {
		script = worker.meshScriptPath
	}
	if relative, err := filepath.Rel(worker.cadQueryRoot, script); err == nil &&
		relative != ".." && !strings.HasPrefix(relative, ".."+string(filepath.Separator)) {
		script = relative
	}
	if worker.pythonExecutable != "" {
		return Command{
			Dir:            worker.cadQueryRoot,
			Name:           worker.pythonExecutable,
			Args:           []string{script, "--stdin"},
			Stdin:          payload,
			MaxOutputBytes: worker.maxOutputBytes,
		}
	}
	return Command{
		Dir:            worker.cadQueryRoot,
		Name:           worker.pixiExecutable,
		Args:           []string{"run", "python", script, "--stdin"},
		Stdin:          payload,
		MaxOutputBytes: worker.maxOutputBytes,
	}
}

// operationTimeout keeps the sampled-topology ceiling untouched while giving
// OCCT surface meshing the headroom it needs on large assemblies.
func (worker *Worker) operationTimeout(operation Operation) time.Duration {
	if operation == OperationTessellate {
		return worker.meshTimeout
	}
	return worker.timeout
}

func (worker *Worker) classifyRunError(operation Operation, ctx context.Context, runErr error, stderr []byte) error {
	switch {
	case errors.Is(ctx.Err(), context.DeadlineExceeded):
		return worker.contextError(operation, context.DeadlineExceeded)
	case errors.Is(ctx.Err(), context.Canceled):
		return worker.contextError(operation, context.Canceled)
	case errors.Is(runErr, errOutputLimit):
		return workerError(
			ErrorPayloadTooLarge,
			operation,
			fmt.Sprintf("response exceeds the %d byte limit", worker.maxOutputBytes),
			ErrPayloadTooLarge,
		)
	case errors.Is(runErr, exec.ErrNotFound):
		return workerError(ErrorUnavailable, operation, "configured Python launcher was not found", ErrUnavailable)
	default:
		detail := strings.TrimSpace(string(stderr))
		if detail == "" {
			detail = runErr.Error()
		}
		if len(detail) > 4096 {
			detail = detail[len(detail)-4096:]
		}
		return workerError(ErrorWorkerFailed, operation, detail, ErrWorkerFailed)
	}
}

func (worker *Worker) contextError(operation Operation, err error) error {
	if errors.Is(err, context.DeadlineExceeded) {
		return workerError(
			ErrorTimeout,
			operation,
			fmt.Sprintf("request deadline exceeded (worker timeout limit %s)", worker.operationTimeout(operation)),
			ErrTimeout,
		)
	}
	return workerError(ErrorCanceled, operation, "request canceled", ErrCanceled)
}

type responseEnvelope struct {
	Fingerprint  string          `json:"fingerprint"`
	Unit         string          `json:"unit"`
	Edges        json.RawMessage `json:"edges"`
	Seam         json.RawMessage `json:"seam"`
	WeldSequence json.RawMessage `json:"weld_sequence"`
	Mesh         json.RawMessage `json:"mesh"`
}

func validateResponse(operation Operation, fingerprint string, raw []byte) error {
	if len(raw) == 0 || !json.Valid(raw) {
		return workerError(ErrorInvalidResponse, operation, "worker returned invalid JSON", ErrInvalidResponse)
	}
	var envelope responseEnvelope
	if err := json.Unmarshal(raw, &envelope); err != nil {
		return workerError(ErrorInvalidResponse, operation, fmt.Sprintf("decode response: %v", err), ErrInvalidResponse)
	}
	if !strings.EqualFold(envelope.Fingerprint, fingerprint) {
		return workerError(ErrorInvalidResponse, operation, "response STEP fingerprint does not match request", ErrInvalidResponse)
	}
	if envelope.Unit != "mm" {
		return workerError(
			ErrorInvalidResponse,
			operation,
			fmt.Sprintf("unsupported worker length unit %q", envelope.Unit),
			ErrInvalidResponse,
		)
	}
	var required json.RawMessage
	switch operation {
	case OperationExtractTopology:
		required = envelope.Edges
	case OperationBuildSeam:
		required = envelope.Seam
	case OperationBuildSequence:
		required = envelope.WeldSequence
	case OperationTessellate:
		required = envelope.Mesh
	}
	if len(required) == 0 || bytes.Equal(required, []byte("null")) {
		return workerError(ErrorInvalidResponse, operation, "response is missing operation result", ErrInvalidResponse)
	}
	return nil
}

func validateWindow(window SeamWindow) error {
	if math.IsNaN(window.Start) || math.IsNaN(window.End) ||
		math.IsInf(window.Start, 0) || math.IsInf(window.End, 0) {
		return errors.New("seam window values must be finite")
	}
	if window.Start < 0 || window.Start > 1 || window.End < 0 || window.End > 1 {
		return errors.New("seam window must be within [0, 1]")
	}
	if window.Start == window.End {
		return errors.New("seam window must have non-zero length")
	}
	return nil
}

func payloadWithoutSTEP(request pythonRequest) []byte {
	request.STEPBase64 = ""
	raw, _ := json.Marshal(request)
	return raw
}

func readBounded(reader io.Reader, limit int64) ([]byte, error) {
	raw, err := io.ReadAll(io.LimitReader(reader, limit+1))
	if err != nil {
		return nil, err
	}
	if int64(len(raw)) > limit {
		return nil, ErrPayloadTooLarge
	}
	return raw, nil
}

func workerError(code ErrorCode, operation Operation, message string, cause error) *WorkerError {
	return &WorkerError{Code: code, Operation: operation, Message: message, Cause: cause}
}

func floatPointer(value float64) *float64 {
	return &value
}

// ExecRunner is the production subprocess runner.
type ExecRunner struct{}

var errOutputLimit = errors.New("CAD worker output limit exceeded")

func (ExecRunner) Run(ctx context.Context, command Command) ([]byte, []byte, error) {
	process := exec.CommandContext(ctx, command.Name, command.Args...)
	process.Dir = command.Dir
	process.Stdin = bytes.NewReader(command.Stdin)
	stdout := &limitedBuffer{limit: command.MaxOutputBytes}
	stderrLimit := int64(64 << 10)
	stderr := &limitedBuffer{limit: stderrLimit}
	process.Stdout = stdout
	process.Stderr = stderr
	err := process.Run()
	if stdout.exceeded {
		return stdout.Bytes(), stderr.Bytes(), errOutputLimit
	}
	return stdout.Bytes(), stderr.Bytes(), err
}

type limitedBuffer struct {
	buffer   bytes.Buffer
	limit    int64
	exceeded bool
}

func (buffer *limitedBuffer) Write(value []byte) (int, error) {
	originalLength := len(value)
	remaining := buffer.limit - int64(buffer.buffer.Len())
	if remaining <= 0 {
		buffer.exceeded = true
		return originalLength, nil
	}
	if int64(len(value)) > remaining {
		buffer.exceeded = true
		value = value[:remaining]
	}
	_, _ = buffer.buffer.Write(value)
	return originalLength, nil
}

func (buffer *limitedBuffer) Bytes() []byte {
	return append([]byte(nil), buffer.buffer.Bytes()...)
}

type transportRequest struct {
	Operation            Operation          `json:"operation"`
	Filename             string             `json:"filename"`
	STEPBase64           string             `json:"step_base64"`
	STEPBase64Camel      string             `json:"stepBase64"`
	FixtureID            string             `json:"fixture_id"`
	SampleCount          *int               `json:"sample_count"`
	SampleCountCamel     *int               `json:"sampleCount"`
	EdgeID               string             `json:"edge_id"`
	EdgeIDCamel          string             `json:"edgeId"`
	Start                *float64           `json:"start_s"`
	StartCamel           *float64           `json:"startS"`
	End                  *float64           `json:"end_s"`
	EndCamel             *float64           `json:"endS"`
	SeamSampleCount      *int               `json:"seam_sample_count"`
	SeamSampleCountCamel *int               `json:"seamSampleCount"`
	Reversed             *bool              `json:"reversed"`
	Reverse              *bool              `json:"reverse"`
	GeometryMethod       string             `json:"geometry_method"`
	GeometryMethodCamel  string             `json:"geometryMethod"`
	PathMethod           string             `json:"path_method"`
	PathMethodCamel      string             `json:"pathMethod"`
	Segments             []transportSegment `json:"segments"`

	LinearDeflectionMM        *float64 `json:"linear_deflection_mm"`
	LinearDeflectionMMCamel   *float64 `json:"linearDeflectionMm"`
	AngularDeflectionRad      *float64 `json:"angular_deflection_rad"`
	AngularDeflectionRadCamel *float64 `json:"angularDeflectionRad"`
	MaxTriangles              *int     `json:"max_triangles"`
	MaxTrianglesCamel         *int     `json:"maxTriangles"`
}

type transportSegment struct {
	ID          string   `json:"id"`
	EdgeID      string   `json:"edge_id"`
	EdgeIDCamel string   `json:"edgeId"`
	Start       *float64 `json:"start_s"`
	StartCamel  *float64 `json:"startS"`
	End         *float64 `json:"end_s"`
	EndCamel    *float64 `json:"endS"`
	Reversed    *bool    `json:"reversed"`
	Reverse     *bool    `json:"reverse"`
}

func (request transportRequest) stepInput() (STEPInput, error) {
	fixtureID := strings.TrimSpace(request.FixtureID)
	encoded := strings.TrimSpace(firstString(request.STEPBase64, request.STEPBase64Camel))
	if fixtureID != "" && encoded != "" {
		return STEPInput{}, errors.New("provide fixture_id or step_base64, not both")
	}
	if fixtureID != "" {
		return STEPInput{Filename: request.Filename, FixtureID: fixtureID}, nil
	}
	if comma := strings.IndexByte(encoded, ','); comma >= 0 {
		encoded = encoded[comma+1:]
	}
	step, err := base64.StdEncoding.DecodeString(encoded)
	if err != nil {
		return STEPInput{}, errors.New("step_base64 is not valid base64")
	}
	return STEPInput{Filename: request.Filename, Bytes: step}, nil
}

func (request transportRequest) normalizedSegments() ([]SequenceSegment, error) {
	segments := make([]SequenceSegment, len(request.Segments))
	for index, segment := range request.Segments {
		start := firstFloat(segment.Start, segment.StartCamel, 0.12)
		end := firstFloat(segment.End, segment.EndCamel, 0.88)
		edgeID := firstString(segment.EdgeID, segment.EdgeIDCamel)
		if edgeID == "" {
			return nil, fmt.Errorf("segment %d requires edge_id", index)
		}
		segments[index] = SequenceSegment{
			ID:       segment.ID,
			EdgeID:   edgeID,
			Start:    start,
			End:      end,
			Reversed: firstBool(segment.Reversed, segment.Reverse),
		}
	}
	return segments, nil
}

func requireJSONEOF(decoder *json.Decoder) error {
	var extra any
	if err := decoder.Decode(&extra); errors.Is(err, io.EOF) {
		return nil
	} else if err != nil {
		return fmt.Errorf("invalid trailing JSON: %v", err)
	}
	return errors.New("request must contain exactly one JSON object")
}

func firstInt(values ...*int) int {
	for _, value := range values {
		if value != nil {
			return *value
		}
	}
	return 0
}

func firstFloat(primary, alternate *float64, fallback float64) float64 {
	if primary != nil {
		return *primary
	}
	if alternate != nil {
		return *alternate
	}
	return fallback
}

func firstBool(values ...*bool) bool {
	for _, value := range values {
		if value != nil {
			return *value
		}
	}
	return false
}

func firstString(values ...string) string {
	for _, value := range values {
		if trimmed := strings.TrimSpace(value); trimmed != "" {
			return trimmed
		}
	}
	return ""
}
