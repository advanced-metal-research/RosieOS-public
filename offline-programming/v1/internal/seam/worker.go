package seam

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/base64"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	// The bounds `internal/cad` already enforces on the classic worker. Same
	// shape, and the STEP ceiling is the same file either way.
	DefaultTimeout      = 90 * time.Second
	DefaultMaxSTEPBytes = int64(8 << 20)
	// A build_plan_request carries the STEP (8 MB ceiling, 10.7 MB as base64)
	// and the planning context beside it -- the description's registered
	// files, which measure 2.23 MB for rosie_1400_v3 and 0.91 MB for
	// rosie_1420_v1, 3 MB as base64 -- so the 12 MB the other operations
	// live under is not enough for this one. 32 MB leaves room for a model
	// with heavier meshes without lifting the STEP ceiling.
	DefaultMaxRequestBytes = int64(32 << 20)
	DefaultMaxOutputBytes  = int64(64 << 20)

	// Two, not one, because the caller asks two questions at once: every
	// angle search runs the seam forward AND reverse and keeps the better
	// coverage, and at one slot those two serialize. Measured on
	// `dog_box.step`: one direction 1.36 s, both together 2.70 s at a
	// ceiling of 1 and 1.50 s at 2 -- the second search costs 0.14 s rather
	// than 1.35, because the machine has cores idle while a worker casts
	// rays.
	//
	// Two rather than more because a worker peaks at 486 MB resident
	// (measured, `plan_torch_path` on the same part) and the ceiling is
	// about what a wrong answer costs: several tabs each fanning out would
	// swap, and swapping is slower than serializing.
	DefaultMaxConcurrent = 2

	// Workers started ahead of their request (spares.go). One per slot: a
	// click takes both at once for the two search directions, and their
	// replacements take 0.42 s to start -- ready about when those searches
	// finish (0.39 s each) for the clearance check that follows. Measured on
	// bracket_b_2x through the live API, a whole click (both searches, the
	// check, the narrowed re-check) went 3.40 s -> 1.54 s on seam_0010 and
	// 3.76 s -> 2.04 s on seam_0014. The cost is memory held while idle: 293 MB
	// resident per waiting python, plus 46 MB for its `pixi run` launcher when
	// SEAM_WORKER_PYTHON is unset.
	DefaultSpares = DefaultMaxConcurrent

	// Joint detection walks every face pair and then fits a rail per weld
	// line, so it gets the ceiling tessellation gets rather than the sampled
	// one. Measured on `bracket_b.step`: 0.7 s for 6 joints and 24 seams,
	// which is nowhere near either, but the parts that matter are assemblies.
	DefaultDetectTimeout = 240 * time.Second
)

var errOutputLimit = errors.New("seam worker output limit exceeded")

// Config configures a Worker. The zero value takes every default above.
type Config struct {
	// WeldPlannerRoot is `weld_planner/v1`, the directory the worker runs in
	// so it resolves the same pixi environment its imports expect.
	WeldPlannerRoot string
	// PythonExecutable runs the worker directly. When empty, PixiExecutable
	// runs it through the project environment, which is what a checkout that
	// has not been provisioned by hand needs.
	PythonExecutable string
	PixiExecutable   string
	PixiEnvironment  string

	Timeout         time.Duration
	DetectTimeout   time.Duration
	MaxSTEPBytes    int64
	MaxRequestBytes int64
	MaxOutputBytes  int64
	MaxConcurrent   int
	// Spares is how many workers wait started ahead of their request once
	// Warm is called. Zero takes SEAM_WORKER_SPARES, then DefaultSpares;
	// negative (or SEAM_WORKER_SPARES=0) starts every request cold.
	Spares int
	Runner CommandRunner
}

// Worker invokes `seam_worker.workers` as one bounded process per request.
type Worker struct {
	root             string
	pythonExecutable string
	pixiExecutable   string
	pixiEnvironment  string
	timeout          time.Duration
	detectTimeout    time.Duration
	maxSTEPBytes     int64
	maxRequestBytes  int64
	maxOutputBytes   int64
	runner           CommandRunner
	slots            chan struct{}

	// Started processes waiting for their one request; see spares.go.
	spareCount int
	spares     chan Spare
	spareMu    sync.Mutex
	warmed     bool
	closed     bool
}

// New builds a Worker. It does not touch the filesystem or start anything:
// spares start only when Warm is called, and each still serves one request.
func New(config Config) (*Worker, error) {
	if strings.TrimSpace(config.WeldPlannerRoot) == "" {
		return nil, errors.New("seam: WeldPlannerRoot is required")
	}
	root, err := filepath.Abs(config.WeldPlannerRoot)
	if err != nil {
		return nil, fmt.Errorf("seam: WeldPlannerRoot: %w", err)
	}
	for name, value := range map[string]int64{
		"MaxSTEPBytes":    config.MaxSTEPBytes,
		"MaxRequestBytes": config.MaxRequestBytes,
		"MaxOutputBytes":  config.MaxOutputBytes,
	} {
		if value < 0 {
			return nil, fmt.Errorf("seam: %s cannot be negative", name)
		}
	}
	if config.MaxConcurrent < 0 {
		return nil, errors.New("seam: MaxConcurrent cannot be negative")
	}

	defaulted := func(value, fallback int64) int64 {
		if value == 0 {
			return fallback
		}
		return value
	}
	if config.Timeout == 0 {
		config.Timeout = DefaultTimeout
	}
	if config.DetectTimeout == 0 {
		config.DetectTimeout = DefaultDetectTimeout
	}
	if config.MaxConcurrent == 0 {
		config.MaxConcurrent = DefaultMaxConcurrent
	}
	if config.PythonExecutable == "" {
		config.PythonExecutable = strings.TrimSpace(os.Getenv("SEAM_WORKER_PYTHON"))
	}
	if config.PixiExecutable == "" {
		config.PixiExecutable = "pixi"
	}
	if config.PixiEnvironment == "" {
		config.PixiEnvironment = "default"
	}
	if config.Runner == nil {
		config.Runner = ExecRunner{}
	}
	if config.Spares == 0 {
		if raw := strings.TrimSpace(os.Getenv("SEAM_WORKER_SPARES")); raw != "" {
			value, err := strconv.Atoi(raw)
			if err != nil {
				return nil, fmt.Errorf("seam: SEAM_WORKER_SPARES: %w", err)
			}
			config.Spares = value
			if value == 0 {
				config.Spares = -1
			}
		}
	}
	spareCount := config.Spares
	if spareCount == 0 {
		spareCount = DefaultSpares
	}
	if spareCount < 0 {
		spareCount = 0
	}

	return &Worker{
		root:             root,
		pythonExecutable: config.PythonExecutable,
		pixiExecutable:   config.PixiExecutable,
		pixiEnvironment:  config.PixiEnvironment,
		timeout:          config.Timeout,
		detectTimeout:    config.DetectTimeout,
		maxSTEPBytes:     defaulted(config.MaxSTEPBytes, DefaultMaxSTEPBytes),
		maxRequestBytes:  defaulted(config.MaxRequestBytes, DefaultMaxRequestBytes),
		maxOutputBytes:   defaulted(config.MaxOutputBytes, DefaultMaxOutputBytes),
		runner:           config.Runner,
		slots:            make(chan struct{}, config.MaxConcurrent),
		spareCount:       spareCount,
		spares:           make(chan Spare, spareCount),
	}, nil
}

// DetectJoints returns where the parts meet and the seams each joint defaults
// to, in one document -- a joints answer with no seams is a list of places a
// weld could go and nothing an operator can act on.
func (worker *Worker) DetectJoints(ctx context.Context, request JointsRequest) (Result, error) {
	// Sent only when asked for. A Go zero value is indistinguishable from
	// "unset" once it is on the wire, and zero here is the one value that
	// breaks the rail -- see JointsRequest.RailOffsetM.
	body := map[string]any{}
	if request.RailOffsetM > 0 {
		body["rail_offset_m"] = request.RailOffsetM
	}
	if err := putRaw(body, "topology", request.Topology, OperationDetectJoints); err != nil {
		return Result{}, err
	}
	options := map[string]any{}
	if request.MinLengthM > 0 {
		options["min_length_m"] = request.MinLengthM
	}
	if request.MaxCornerAngleDeg > 0 {
		options["max_corner_angle_deg"] = request.MaxCornerAngleDeg
	}
	if len(options) > 0 {
		body["seam_options"] = options
	}
	return worker.run(ctx, OperationDetectJoints, request.STEP, body)
}

// PlanTorchPath measures how much arc is clear at every angle the search may
// use, then picks the profile that welds the most of the seam.
func (worker *Worker) PlanTorchPath(ctx context.Context, request TorchPathRequest) (Result, error) {
	body := map[string]any{}
	for key, raw := range map[string][]byte{
		"joint": request.Joint, "seam": request.Seam,
		"torch": request.Torch, "intersector": request.Intersector,
	} {
		if err := putRaw(body, key, raw, OperationPlanTorchPath); err != nil {
			return Result{}, err
		}
	}
	for _, key := range []string{"joint", "seam", "torch"} {
		if _, present := body[key]; !present {
			return Result{}, workerError(ErrorInvalidRequest, OperationPlanTorchPath,
				key+" is required", ErrInvalidRequest)
		}
	}
	if request.DefaultStandoffMM <= 0 {
		return Result{}, workerError(ErrorInvalidRequest, OperationPlanTorchPath,
			"default_standoff_mm is required and must be positive", ErrInvalidRequest)
	}
	if request.Search.WorkDeviationDeg < 0 || request.Search.TravelDeviationDeg < 0 {
		return Result{}, workerError(ErrorInvalidRequest, OperationPlanTorchPath,
			"a deviation cannot be negative", ErrInvalidRequest)
	}
	direction := request.Direction
	if direction == "" {
		direction = "forward"
	}
	if direction != "forward" && direction != "reverse" {
		return Result{}, workerError(ErrorInvalidRequest, OperationPlanTorchPath,
			fmt.Sprintf("direction must be forward or reverse, got %q", direction), ErrInvalidRequest)
	}
	body["direction"] = direction
	body["default_standoff_mm"] = request.DefaultStandoffMM
	body["search"] = request.Search
	return worker.run(ctx, OperationPlanTorchPath, request.STEP, body)
}

// BuildPlanRequest packs one `.weldplan`. Determinism is the contract -- same
// inputs, same bytes, same digests -- which is why it is packed in one place
// and not wherever a caller happens to be: a second implementation would be a
// second answer to "did the request change?", the first question anyone asks
// when a plan comes back different.
func (worker *Worker) BuildPlanRequest(ctx context.Context, request PlanRequestBuild) (Result, error) {
	body := map[string]any{"workpiece_absent": request.WorkpieceAbsent}
	if err := putRaw(body, "program", request.Program, OperationBuildPlanRequest); err != nil {
		return Result{}, err
	}
	if err := putRaw(body, "tooling", request.Tooling, OperationBuildPlanRequest); err != nil {
		return Result{}, err
	}
	if _, present := body["program"]; !present {
		return Result{}, workerError(ErrorInvalidRequest, OperationBuildPlanRequest,
			"program is required", ErrInvalidRequest)
	}
	if len(request.PlanningContext) == 0 {
		return Result{}, workerError(ErrorInvalidRequest, OperationBuildPlanRequest,
			"planning_context is required: a robot named rather than carried resolves differently "+
				"on another machine, and the placement is measured from its work frame",
			ErrInvalidRequest)
	}
	if err := putRaw(body, "planning_context", request.PlanningContext, OperationBuildPlanRequest); err != nil {
		return Result{}, err
	}
	if err := putRaw(body, "start_state", request.StartState, OperationBuildPlanRequest); err != nil {
		return Result{}, err
	}
	if trimmed := strings.TrimSpace(string(request.IOOutputs)); trimmed != "" && trimmed != "null" {
		if err := putRaw(body, "io_outputs", request.IOOutputs, OperationBuildPlanRequest); err != nil {
			return Result{}, err
		}
	}
	if request.STEPFilename != "" {
		body["step_filename"] = request.STEPFilename
	}
	if request.RequestID != "" {
		body["request_id"] = request.RequestID
	}
	if request.Label != "" {
		body["label"] = request.Label
	}
	if trimmed := strings.TrimSpace(string(request.Fixtures)); trimmed != "" && trimmed != "null" {
		body["fixtures"] = request.Fixtures
	}
	return worker.run(ctx, OperationBuildPlanRequest, request.STEP, body)
}

// SampleSeamFrames returns the frame at each sampled point along a seam.
func (worker *Worker) SampleSeamFrames(ctx context.Context, request SeamFramesRequest) (Result, error) {
	body := map[string]any{}
	for key, raw := range map[string][]byte{"curve": request.Curve, "reference": request.Reference} {
		if err := putRaw(body, key, raw, OperationSampleSeamFrames); err != nil {
			return Result{}, err
		}
		if _, present := body[key]; !present {
			return Result{}, workerError(ErrorInvalidRequest, OperationSampleSeamFrames,
				key+" is required", ErrInvalidRequest)
		}
	}
	if request.Samples > 0 {
		body["samples"] = request.Samples
	}
	if request.MaxChordM > 0 {
		body["max_chord_m"] = request.MaxChordM
	}
	if request.MaxAngleRad > 0 {
		body["max_angle_rad"] = request.MaxAngleRad
	}
	return worker.run(ctx, OperationSampleSeamFrames, nil, body)
}

// CheckTorchFits answers whether a torch of this shape reaches along this seam,
// and where it does not.
func (worker *Worker) CheckTorchFits(ctx context.Context, request TorchFitRequest) (Result, error) {
	body := map[string]any{}
	for key, raw := range map[string][]byte{
		"joint": request.Joint, "seam": request.Seam,
		"torch": request.Torch, "options": request.Options,
	} {
		if err := putRaw(body, key, raw, OperationCheckTorchFits); err != nil {
			return Result{}, err
		}
	}
	for _, key := range []string{"joint", "seam", "torch"} {
		if _, present := body[key]; !present {
			return Result{}, workerError(ErrorInvalidRequest, OperationCheckTorchFits,
				key+" is required", ErrInvalidRequest)
		}
	}
	return worker.run(ctx, OperationCheckTorchFits, request.STEP, body)
}

// FingerprintSTEP names the part an answer is about, so a caller holding
// several can tell them apart without re-reading the STEP.
func FingerprintSTEP(step []byte) string {
	sum := sha256.Sum256(step)
	return hex.EncodeToString(sum[:])
}

func putRaw(body map[string]any, key string, raw []byte, operation Operation) error {
	if len(raw) == 0 {
		return nil
	}
	var decoded any
	if err := json.Unmarshal(raw, &decoded); err != nil {
		return workerError(ErrorInvalidRequest, operation,
			fmt.Sprintf("%s is not valid JSON: %v", key, err), ErrInvalidRequest)
	}
	body[key] = decoded
	return nil
}

func (worker *Worker) operationTimeout(operation Operation) time.Duration {
	if operation == OperationDetectJoints || operation == OperationPlanTorchPath {
		return worker.detectTimeout
	}
	return worker.timeout
}

func (worker *Worker) run(
	ctx context.Context, operation Operation, step []byte, body map[string]any,
) (Result, error) {
	// Every operation but one measures the part, and none of them can look a
	// hash up in a workspace to find it. `sample_seam_frames` is the exception
	// on purpose: a curve and a rail are the whole input and no ray is cast.
	if len(step) == 0 && operation != OperationSampleSeamFrames && !(operation == OperationBuildPlanRequest && body["workpiece_absent"] == true) {
		return Result{}, workerError(ErrorInvalidRequest, operation,
			"a STEP is required; this worker resolves nothing from a workspace", ErrInvalidRequest)
	}
	if int64(len(step)) > worker.maxSTEPBytes {
		return Result{}, workerError(ErrorPayloadTooLarge, operation,
			fmt.Sprintf("STEP is %d bytes, over the %d byte limit", len(step), worker.maxSTEPBytes),
			ErrPayloadTooLarge)
	}

	body["operation"] = string(operation)
	if len(step) > 0 {
		body["step_base64"] = base64.StdEncoding.EncodeToString(step)
	}
	payload, err := json.Marshal(body)
	if err != nil {
		return Result{}, workerError(ErrorInvalidRequest, operation, err.Error(), ErrInvalidRequest)
	}
	if int64(len(payload)) > worker.maxRequestBytes {
		return Result{}, workerError(ErrorPayloadTooLarge, operation,
			fmt.Sprintf("request is %d bytes, over the %d byte limit", len(payload), worker.maxRequestBytes),
			ErrPayloadTooLarge)
	}

	// Nothing is cached here. A response cache would have to be keyed by the
	// whole request to be correct -- two joint detections of one part at
	// different rail offsets are different answers about the same bytes -- and
	// keyed that way it only ever hits on a request repeated verbatim, which
	// authoring does not do: the next click is the next seam. What it cannot
	// touch is the cost that actually dominates. Measured on a 52 KB part,
	// 0.34 s of the 0.67 s is `import OCP`, paid per process, and the STEP
	// parse it would save is 50 ms over a local pipe. The caller holds the
	// derived documents instead and hands them back -- `topology` on a joints
	// or mesh request is exactly that.
	fingerprint := ""
	if len(step) > 0 {
		fingerprint = FingerprintSTEP(step)
	}

	select {
	case worker.slots <- struct{}{}:
		defer func() { <-worker.slots }()
	case <-ctx.Done():
		return Result{}, worker.contextError(operation, ctx.Err())
	}

	runCtx, cancel := context.WithTimeout(ctx, worker.operationTimeout(operation))
	defer cancel()

	stdout, stderr, runErr := worker.execute(runCtx, payload)
	if runErr != nil && len(stdout) == 0 {
		return Result{}, worker.classifyRunError(operation, runCtx, runErr, stderr)
	}

	// The worker answers with a code rather than a traceback, including when
	// it exits non-zero, so a body is read before the exit status is trusted.
	var envelope struct {
		Error *struct {
			Code   string `json:"code"`
			Detail string `json:"detail"`
		} `json:"error"`
	}
	if err := json.Unmarshal(stdout, &envelope); err != nil {
		if runErr != nil {
			return Result{}, worker.classifyRunError(operation, runCtx, runErr, stderr)
		}
		return Result{}, workerError(ErrorInvalidResponse, operation,
			fmt.Sprintf("response is not JSON: %v", err), ErrInvalidResponse)
	}
	if envelope.Error != nil {
		code, cause := classifyWorkerCode(envelope.Error.Code)
		return Result{}, workerError(code, operation, envelope.Error.Detail, cause)
	}
	if runErr != nil {
		return Result{}, worker.classifyRunError(operation, runCtx, runErr, stderr)
	}

	return Result{Operation: operation, Fingerprint: fingerprint, JSON: stdout}, nil
}

func classifyWorkerCode(code string) (ErrorCode, error) {
	switch ErrorCode(code) {
	case ErrorInvalidRequest:
		return ErrorInvalidRequest, ErrInvalidRequest
	default:
		return ErrorWorkerFailed, ErrWorkerFailed
	}
}

func (worker *Worker) command(payload []byte) Command {
	if worker.pythonExecutable != "" {
		return Command{
			Dir:            worker.root,
			Name:           worker.pythonExecutable,
			Args:           []string{"-m", "seam_worker.workers", "--stdin"},
			Stdin:          payload,
			MaxOutputBytes: worker.maxOutputBytes,
		}
	}
	return Command{
		Dir:  worker.root,
		Name: worker.pixiExecutable,
		Args: []string{
			"run", "-e", worker.pixiEnvironment,
			"python", "-m", "seam_worker.workers", "--stdin",
		},
		Stdin:          payload,
		MaxOutputBytes: worker.maxOutputBytes,
	}
}

func (worker *Worker) classifyRunError(
	operation Operation, ctx context.Context, runErr error, stderr []byte,
) error {
	switch {
	case errors.Is(ctx.Err(), context.DeadlineExceeded):
		return worker.contextError(operation, context.DeadlineExceeded)
	case errors.Is(ctx.Err(), context.Canceled):
		return worker.contextError(operation, context.Canceled)
	case errors.Is(runErr, errOutputLimit):
		return workerError(ErrorPayloadTooLarge, operation,
			fmt.Sprintf("response exceeds the %d byte limit", worker.maxOutputBytes), ErrPayloadTooLarge)
	case errors.Is(runErr, exec.ErrNotFound):
		return workerError(ErrorUnavailable, operation,
			"configured Python launcher was not found", ErrUnavailable)
	default:
		detail := strings.TrimSpace(string(stderr))
		if detail == "" && runErr != nil {
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
		return workerError(ErrorTimeout, operation,
			fmt.Sprintf("request deadline exceeded (worker timeout limit %s)",
				worker.operationTimeout(operation)), ErrTimeout)
	}
	return workerError(ErrorCanceled, operation, "request canceled", ErrCanceled)
}

// ExecRunner is the production subprocess runner.
type ExecRunner struct{}

func (ExecRunner) Run(ctx context.Context, command Command) ([]byte, []byte, error) {
	process := exec.CommandContext(ctx, command.Name, command.Args...)
	containProcessTree(process)
	process.Cancel = func() error { return killProcessTree(process) }
	process.Dir = command.Dir
	process.Stdin = bytes.NewReader(command.Stdin)
	// The package lives under `python/`, and a worker invoked directly rather
	// than through pixi has no manifest to say so.
	process.Env = append(os.Environ(), "PYTHONPATH="+filepath.Join(command.Dir, "python"))
	stdout := &limitedBuffer{limit: command.MaxOutputBytes}
	stderr := &limitedBuffer{limit: 64 << 10}
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
	remaining := buffer.limit - int64(buffer.buffer.Len())
	if remaining <= 0 {
		buffer.exceeded = true
		return len(value), nil
	}
	if int64(len(value)) > remaining {
		buffer.buffer.Write(value[:remaining])
		buffer.exceeded = true
		return len(value), nil
	}
	return buffer.buffer.Write(value)
}

func (buffer *limitedBuffer) Bytes() []byte { return buffer.buffer.Bytes() }

// ---- the transport surface ---------------------------------------------------

// transportRequest is what a browser posts. It is the worker's own request
// shape rather than a translation of it, so the HTTP layer forwards a body
// instead of restating an API -- which is what `internal/cad` does, and why
// neither adapter has product wiring in it.
type transportRequest struct {
	WorkpieceAbsent bool            `json:"workpiece_absent"`
	Operation       Operation       `json:"operation"`
	STEPBase64      string          `json:"step_base64"`
	Topology        json.RawMessage `json:"topology"`
	RailOffsetM     float64         `json:"rail_offset_m"`
	SeamOptions     struct {
		MinLengthM        float64 `json:"min_length_m"`
		MaxCornerAngleDeg float64 `json:"max_corner_angle_deg"`
	} `json:"seam_options"`
	Joint             json.RawMessage `json:"joint"`
	Seam              json.RawMessage `json:"seam"`
	Torch             json.RawMessage `json:"torch"`
	Options           json.RawMessage `json:"options"`
	Intersector       json.RawMessage `json:"intersector"`
	Program           json.RawMessage `json:"program"`
	Tooling           json.RawMessage `json:"tooling"`
	PlanningContext   json.RawMessage `json:"planning_context"`
	StartState        json.RawMessage `json:"start_state"`
	IOOutputs         json.RawMessage `json:"io_outputs"`
	Fixtures          json.RawMessage `json:"fixtures"`
	STEPFilename      string          `json:"step_filename"`
	RequestID         string          `json:"request_id"`
	Label             string          `json:"label"`
	Curve             json.RawMessage `json:"curve"`
	Reference         json.RawMessage `json:"reference"`
	Samples           int             `json:"samples"`
	MaxChordM         float64         `json:"max_chord_m"`
	MaxAngleRad       float64         `json:"max_angle_rad"`
	Direction         string          `json:"direction"`
	Search            SearchKnobs     `json:"search"`
	DefaultStandoffMM float64         `json:"default_standoff_mm"`
}

// ExecuteJSON runs one request given as the bytes a caller posted.
func (worker *Worker) ExecuteJSON(ctx context.Context, raw []byte) (Result, error) {
	if len(raw) == 0 {
		return Result{}, workerError(ErrorInvalidRequest, "", "request body is empty", ErrInvalidRequest)
	}
	if int64(len(raw)) > worker.maxRequestBytes {
		return Result{}, workerError(ErrorPayloadTooLarge, "",
			fmt.Sprintf("request exceeds the %d byte limit", worker.maxRequestBytes), ErrPayloadTooLarge)
	}
	var request transportRequest
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&request); err != nil {
		return Result{}, workerError(ErrorInvalidRequest, "",
			fmt.Sprintf("invalid JSON: %v", err), ErrInvalidRequest)
	}
	step, err := base64.StdEncoding.DecodeString(request.STEPBase64)
	if err != nil {
		return Result{}, workerError(ErrorInvalidRequest, request.Operation,
			fmt.Sprintf("step_base64 is not base64: %v", err), ErrInvalidRequest)
	}

	switch request.Operation {
	case OperationDetectJoints:
		return worker.DetectJoints(ctx, JointsRequest{
			STEP:              step,
			Topology:          request.Topology,
			RailOffsetM:       request.RailOffsetM,
			MinLengthM:        request.SeamOptions.MinLengthM,
			MaxCornerAngleDeg: request.SeamOptions.MaxCornerAngleDeg,
		})
	case OperationPlanTorchPath:
		return worker.PlanTorchPath(ctx, TorchPathRequest{
			STEP: step, Joint: request.Joint, Seam: request.Seam, Torch: request.Torch,
			Direction: request.Direction, Search: request.Search,
			DefaultStandoffMM: request.DefaultStandoffMM, Intersector: request.Intersector,
		})
	case OperationBuildPlanRequest:
		return worker.BuildPlanRequest(ctx, PlanRequestBuild{
			STEP: step, STEPFilename: request.STEPFilename, Program: request.Program, WorkpieceAbsent: request.WorkpieceAbsent,
			PlanningContext: request.PlanningContext, StartState: request.StartState, IOOutputs: request.IOOutputs,
			Tooling: request.Tooling, Fixtures: request.Fixtures, RequestID: request.RequestID, Label: request.Label,
		})
	case OperationSampleSeamFrames:
		return worker.SampleSeamFrames(ctx, SeamFramesRequest{
			Curve: request.Curve, Reference: request.Reference,
			Samples: request.Samples, MaxChordM: request.MaxChordM,
			MaxAngleRad: request.MaxAngleRad,
		})
	case OperationCheckTorchFits:
		return worker.CheckTorchFits(ctx, TorchFitRequest{
			STEP:    step,
			Joint:   request.Joint,
			Seam:    request.Seam,
			Torch:   request.Torch,
			Options: request.Options,
		})
	default:
		return Result{}, workerError(ErrorInvalidRequest, request.Operation,
			fmt.Sprintf("unknown operation %q", request.Operation), ErrInvalidRequest)
	}
}
