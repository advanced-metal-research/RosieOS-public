package simulator

import (
	"bufio"
	"bytes"
	"context"
	"crypto/rand"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"sort"
	"strings"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

const (
	defaultCanonicalBinary = "motion-server/v1/bin/robot-v4-cartesiand"
	defaultCollisionPython = "tesseract/v1/.pixi/envs/default/bin/python3"
	defaultCollisionScript = "tesseract/v1/python/persistent_collision_check.py"
	defaultProbeInput      = "motion-server/v1/testdata/spreadsheet-plan-clear-input.json"
	defaultPlannerURDF     = "robot_description/robots/rosie_1400_v3/robot.urdf"
	canonicalBinaryName    = "robot-v4-cartesiand"
	maxPlannerStderrBytes  = 64 * 1024
)

type PCBackendConfig struct {
	Enabled   bool
	RepoRoot  string
	ProofTTL  time.Duration
	Clock     func() time.Time
	RunBinary string

	runner plannerProcessRunner
}

type plannerProcessRunner interface {
	Run(context.Context, []byte, ResourceLimits) ([]byte, error)
}

type persistentPlannerProcessRunner interface {
	plannerProcessRunner
	Start(context.Context, ResourceLimits) error
	Stop(context.Context) error
	Persistent() bool
}

// PCNativeBackend owns an isolated, process-group-bounded Motion Server planner
// and Tesseract collision worker. They stay warm for the local simulator
// session but have no command, realtime, hardware, broker, or output transport.
type PCNativeBackend struct {
	config PCBackendConfig
	runner plannerProcessRunner

	mu             sync.Mutex
	active         bool
	bootID         string
	proofSequence  uint64
	topology       Topology
	limits         ResourceLimits
	availability   BackendAvailability
	acceptedAssets planning.AcceptedPlanAssets
	assetError     error
}

func NewPCNativeBackend(config PCBackendConfig) *PCNativeBackend {
	config.RepoRoot = filepath.Clean(strings.TrimSpace(config.RepoRoot))
	if config.ProofTTL <= 0 || config.ProofTTL > 30*time.Second {
		config.ProofTTL = 5 * time.Second
	}
	if config.Clock == nil {
		config.Clock = time.Now
	}
	binary := strings.TrimSpace(config.RunBinary)
	if binary == "" && config.RepoRoot != "." && config.RepoRoot != "" {
		relative := defaultCanonicalBinary
		if runtime.GOOS == "windows" {
			relative += ".exe"
		}
		binary = filepath.Join(config.RepoRoot, filepath.FromSlash(relative))
	}
	if config.runner == nil {
		config.runner = &canonicalProcessRunner{
			repoRoot:        config.RepoRoot,
			binary:          binary,
			collisionPython: collisionPythonPath(config.RepoRoot),
			collisionScript: filepath.Join(
				config.RepoRoot,
				filepath.FromSlash(defaultCollisionScript),
			),
			plannerURDF: filepath.Join(
				config.RepoRoot,
				filepath.FromSlash(defaultPlannerURDF),
			),
		}
	}
	backend := &PCNativeBackend{
		config: config,
		runner: config.runner,
	}
	backend.availability = backend.inspectAvailability(binary)
	if backend.availability.Available {
		backend.availability.PersistentProcess = backend.runnerIsPersistent()
	}
	if backend.availability.Available {
		backend.acceptedAssets, backend.assetError = localAcceptedPlanAssets(
			config.RepoRoot,
			binary,
		)
		if backend.assetError != nil {
			backend.availability = unavailable(
				AvailabilityCanonicalAssets,
				"canonical Robot V4 planner asset digests are unavailable",
			)
		}
	}
	return backend
}

func (backend *PCNativeBackend) Availability() BackendAvailability {
	if backend == nil {
		return unavailable(AvailabilityCanonicalBinary, "canonical local planner backend is nil")
	}
	backend.mu.Lock()
	defer backend.mu.Unlock()
	return backend.availability
}

func (backend *PCNativeBackend) AcceptedPlanAssets() (
	planning.AcceptedPlanAssets,
	error,
) {
	if backend == nil {
		return planning.AcceptedPlanAssets{}, errors.New(
			"canonical Robot V4 planner backend is nil",
		)
	}
	backend.mu.Lock()
	defer backend.mu.Unlock()
	if backend.assetError != nil {
		return planning.AcceptedPlanAssets{}, backend.assetError
	}
	if !backend.availability.Available {
		return planning.AcceptedPlanAssets{}, errors.New(
			"canonical Robot V4 planner assets are unavailable",
		)
	}
	return backend.acceptedAssets, nil
}

func (backend *PCNativeBackend) Start(
	ctx context.Context,
	request StartRequest,
) (RuntimeSnapshot, error) {
	if backend == nil {
		return RuntimeSnapshot{}, errors.New("canonical local planner backend is nil")
	}
	if err := validateStartRequest(request); err != nil {
		return RuntimeSnapshot{}, err
	}
	backend.mu.Lock()
	availability := backend.availability
	backend.mu.Unlock()
	if !availability.Available {
		return RuntimeSnapshot{}, errors.New(availability.Reason)
	}

	probe, err := os.ReadFile(filepath.Join(
		backend.config.RepoRoot,
		filepath.FromSlash(defaultProbeInput),
	))
	if err != nil {
		backend.markUnavailable(AvailabilityCanonicalAssets, "canonical planner probe input is unavailable")
		return RuntimeSnapshot{}, errors.New("canonical planner probe input is unavailable")
	}
	probeContext, cancel := context.WithTimeout(ctx, request.Limits.PlanTimeout)
	defer cancel()
	if lifecycle, ok := backend.runner.(persistentPlannerProcessRunner); ok {
		if err := lifecycle.Start(probeContext, request.Limits); err != nil {
			return RuntimeSnapshot{}, fmt.Errorf(
				"start persistent canonical planner: %w", err)
		}
	}
	raw, err := backend.runner.Run(probeContext, probe, request.Limits)
	if err != nil {
		backend.stopRunner(context.WithoutCancel(ctx))
		return RuntimeSnapshot{}, fmt.Errorf("canonical planner readiness probe failed: %w", err)
	}
	if err := validateCanonicalReport(raw, true); err != nil {
		backend.stopRunner(context.WithoutCancel(ctx))
		return RuntimeSnapshot{}, fmt.Errorf(
			"canonical planner readiness proof rejected: %w (%s)",
			err,
			summarizeCanonicalReport(raw),
		)
	}

	bootID, err := randomBootID()
	if err != nil {
		return RuntimeSnapshot{}, fmt.Errorf("create local runtime identity: %w", err)
	}
	backend.mu.Lock()
	backend.active = true
	backend.bootID = bootID
	backend.proofSequence = 0
	backend.topology = request.Topology
	backend.limits = request.Limits
	backend.availability = backend.readyAvailability()
	snapshot := backend.snapshotLocked(request.Topology)
	backend.mu.Unlock()
	return snapshot, nil
}

func (backend *PCNativeBackend) Probe(context.Context) (RuntimeSnapshot, error) {
	if backend == nil {
		return RuntimeSnapshot{}, errors.New("canonical local planner backend is nil")
	}
	backend.mu.Lock()
	defer backend.mu.Unlock()
	if !backend.active {
		return RuntimeSnapshot{}, errors.New("canonical local planner session is not active")
	}
	return backend.snapshotLocked(backend.topology), nil
}

func (backend *PCNativeBackend) SubmitPlan(
	ctx context.Context,
	submission planning.MotionServerSubmission,
) ([]byte, error) {
	if backend == nil {
		return nil, errors.New("canonical local planner backend is nil")
	}
	backend.mu.Lock()
	active := backend.active
	limits := backend.limits
	backend.mu.Unlock()
	if !active {
		return nil, errors.New("canonical local planner session is not active")
	}
	raw, err := backend.runner.Run(ctx, submission.Payload, limits)
	if err != nil {
		return nil, err
	}
	if err := validateCanonicalReport(raw, false); err != nil {
		return nil, err
	}
	return planning.TagMotionServerReportRevision(raw, submission.PlanRevision)
}

func (backend *PCNativeBackend) Stop(ctx context.Context) error {
	if backend == nil {
		return nil
	}
	backend.mu.Lock()
	backend.active = false
	backend.bootID = ""
	backend.proofSequence = 0
	backend.topology = Topology{}
	backend.limits = ResourceLimits{}
	backend.mu.Unlock()
	if lifecycle, ok := backend.runner.(persistentPlannerProcessRunner); ok {
		return lifecycle.Stop(ctx)
	}
	return nil
}

func (backend *PCNativeBackend) runnerIsPersistent() bool {
	if backend == nil {
		return false
	}
	lifecycle, ok := backend.runner.(persistentPlannerProcessRunner)
	return ok && lifecycle.Persistent()
}

func (backend *PCNativeBackend) readyAvailability() BackendAvailability {
	availability := available()
	availability.PersistentProcess = backend.runnerIsPersistent()
	return availability
}

func (backend *PCNativeBackend) stopRunner(ctx context.Context) {
	if lifecycle, ok := backend.runner.(persistentPlannerProcessRunner); ok {
		stopContext, cancel := context.WithTimeout(ctx, 5*time.Second)
		defer cancel()
		_ = lifecycle.Stop(stopContext)
	}
}

func (backend *PCNativeBackend) inspectAvailability(binary string) BackendAvailability {
	if !backend.config.Enabled {
		return unavailable(
			AvailabilityExplicitOptInRequired,
			"local simulator requires serve --enable-local-simulator",
		)
	}
	if !boundedProcessSupported() {
		return unavailable(
			AvailabilityProcessContainment,
			"canonical planner process-group containment is unavailable on this platform",
		)
	}
	if backend.config.RepoRoot == "" || backend.config.RepoRoot == "." {
		return unavailable(AvailabilityCanonicalAssets, "repository root is unavailable")
	}
	binaryName := strings.TrimSuffix(
		strings.ToLower(filepath.Base(binary)),
		strings.ToLower(filepath.Ext(binary)),
	)
	if binaryName != canonicalBinaryName {
		return unavailable(
			AvailabilityCanonicalBinary,
			"Robot V4 Motion Server planner binary is required",
		)
	}
	if !regularExecutable(binary) {
		reason := "Robot V4 Motion Server planner binary is unavailable"
		if runtime.GOOS == "windows" {
			linuxBinary := strings.TrimSuffix(binary, ".exe")
			if isELFExecutable(linuxBinary) {
				reason = "Robot V4 Motion Server planner binary is a Linux ELF; " +
					"serve Offline Programming under WSL " +
					"(offline-programming/v1/serve-wsl.sh), not native Windows go run"
			}
		}
		return unavailable(AvailabilityCanonicalBinary, reason)
	}
	if !regularExecutable(collisionPythonPath(backend.config.RepoRoot)) {
		return unavailable(
			AvailabilityCollisionRuntime,
			"canonical Tesseract collision Python environment is unavailable",
		)
	}
	for _, relative := range []string{
		defaultCollisionScript,
		defaultProbeInput,
		defaultPlannerURDF,
	} {
		if !regularFile(filepath.Join(backend.config.RepoRoot, filepath.FromSlash(relative))) {
			return unavailable(AvailabilityCanonicalAssets, "canonical planner assets are unavailable")
		}
	}
	return available()
}

func (backend *PCNativeBackend) snapshotLocked(topology Topology) RuntimeSnapshot {
	backend.proofSequence++
	now := backend.config.Clock().UTC()
	return RuntimeSnapshot{
		ProfileID:      topology.ProfileID,
		RobotModel:     topology.RobotModel,
		PlannerReady:   true,
		SimulatorReady: true,
		PlanTransport:  PlanTransportInProcess,
		Safety: SafetyProof{
			Schema:                   SafetyProofSchema,
			ProfileID:                topology.ProfileID,
			RuntimeBootID:            backend.bootID,
			StatusSequence:           backend.proofSequence,
			ObservedAt:               now,
			FreshUntil:               now.Add(backend.config.ProofTTL),
			Simulation:               true,
			NoRealMovement:           true,
			LoopbackOnly:             true,
			EtherCATOutputEnabled:    false,
			MotionOutputEnabled:      false,
			HardwareAccess:           false,
			PhysicalServicesStarted:  false,
			CommandSubmissionEnabled: false,
		},
	}
}

func (backend *PCNativeBackend) markUnavailable(code AvailabilityCode, reason string) {
	backend.mu.Lock()
	backend.active = false
	backend.availability = unavailable(code, reason)
	backend.mu.Unlock()
}

type canonicalProcessRunner struct {
	repoRoot        string
	binary          string
	collisionPython string
	collisionScript string
	plannerURDF     string

	mu            sync.Mutex
	temp          string
	planner       *managedPlannerProcess
	collision     *managedPlannerProcess
	plannerInput  io.WriteCloser
	plannerOutput *bufio.Reader
	plannerStderr *boundedBuffer
	nextRequestID uint64
}

type managedPlannerProcess struct {
	command     *exec.Cmd
	containment *commandContainment
	done        chan struct{}

	mu      sync.Mutex
	waitErr error
}

func (process *managedPlannerProcess) setWaitError(err error) {
	process.mu.Lock()
	process.waitErr = err
	process.mu.Unlock()
}

func (process *managedPlannerProcess) waitError() error {
	if process == nil {
		return errors.New("planner process is not running")
	}
	process.mu.Lock()
	defer process.mu.Unlock()
	return process.waitErr
}

func startManagedPlannerProcess(command *exec.Cmd) (*managedPlannerProcess, error) {
	containment, err := prepareBoundedCommand(command)
	if err != nil {
		return nil, err
	}
	if err := command.Start(); err != nil {
		containment.Close()
		return nil, err
	}
	if err := containment.AfterStart(); err != nil {
		_ = command.Cancel()
		_ = command.Wait()
		containment.Close()
		return nil, err
	}
	process := &managedPlannerProcess{
		command:     command,
		containment: containment,
		done:        make(chan struct{}),
	}
	go func() {
		process.setWaitError(command.Wait())
		close(process.done)
	}()
	return process, nil
}

func stopManagedPlannerProcess(
	ctx context.Context,
	process *managedPlannerProcess,
) error {
	if process == nil {
		return nil
	}
	select {
	case <-process.done:
	default:
		if process.command != nil && process.command.Cancel != nil {
			_ = process.command.Cancel()
		}
	}
	select {
	case <-process.done:
		process.containment.Close()
		return nil
	case <-ctx.Done():
		process.containment.Close()
		return ctx.Err()
	}
}

func (runner *canonicalProcessRunner) Persistent() bool {
	return runner != nil
}

func (runner *canonicalProcessRunner) Start(
	ctx context.Context,
	limits ResourceLimits,
) error {
	if runner == nil {
		return errors.New("canonical planner runner is nil")
	}
	runner.mu.Lock()
	defer runner.mu.Unlock()
	if runner.planner != nil && runner.collision != nil {
		return nil
	}
	if err := limits.Validate(); err != nil {
		return err
	}
	if !regularExecutable(runner.binary) {
		return errors.New("canonical motion-server planner binary is unavailable")
	}
	if !regularExecutable(runner.collisionPython) ||
		!regularFile(runner.collisionScript) {
		return errors.New("canonical Tesseract collision runtime is unavailable")
	}
	temp, err := os.MkdirTemp("", "offline-programming-v1-planner-session-")
	if err != nil {
		return err
	}
	runner.temp = temp

	readyFile := filepath.Join(temp, "collision-server-ready.json")
	collisionStderr := &boundedBuffer{maximum: maxPlannerStderrBytes}
	collisionCommand := exec.CommandContext(
		context.Background(),
		runner.collisionPython,
		runner.collisionScript,
		"--serve-loopback",
		"--ready-file", readyFile,
	)
	collisionCommand.Dir = runner.repoRoot
	collisionCommand.Env = isolatedPlannerEnvironment(
		temp, "", "", runner.plannerURDF)
	collisionCommand.Stdout = io.Discard
	collisionCommand.Stderr = collisionStderr
	runner.collision, err = startManagedPlannerProcess(collisionCommand)
	if err != nil {
		runner.stopLocked(context.Background())
		return fmt.Errorf("start persistent Tesseract collision worker: %w", err)
	}
	host, port, err := waitForCollisionServer(
		ctx, readyFile, runner.collision, collisionStderr)
	if err != nil {
		runner.stopLocked(context.Background())
		return err
	}

	collisionServer := fmt.Sprintf("%s:%d", host, port)
	plannerCommand := exec.CommandContext(
		context.Background(),
		runner.binary,
		"--self-test", "spreadsheet-tesseract-plan-server",
	)
	plannerCommand.Dir = runner.repoRoot
	plannerCommand.Env = isolatedPlannerEnvironment(
		temp, "", collisionServer, runner.plannerURDF)
	runner.plannerStderr = &boundedBuffer{maximum: maxPlannerStderrBytes}
	plannerCommand.Stderr = runner.plannerStderr
	runner.plannerInput, err = plannerCommand.StdinPipe()
	if err != nil {
		runner.stopLocked(context.Background())
		return fmt.Errorf("open persistent planner input: %w", err)
	}
	plannerOutput, err := plannerCommand.StdoutPipe()
	if err != nil {
		runner.stopLocked(context.Background())
		return fmt.Errorf("open persistent planner output: %w", err)
	}
	runner.plannerOutput = bufio.NewReaderSize(
		plannerOutput, limits.MaxPlanReportBytes+4096)
	runner.planner, err = startManagedPlannerProcess(plannerCommand)
	if err != nil {
		runner.stopLocked(context.Background())
		return fmt.Errorf("start persistent canonical planner: %w", err)
	}
	return nil
}

func (runner *canonicalProcessRunner) Run(
	ctx context.Context,
	payload []byte,
	limits ResourceLimits,
) ([]byte, error) {
	if runner == nil {
		return nil, errors.New("canonical planner runner is nil")
	}
	runner.mu.Lock()
	defer runner.mu.Unlock()
	if runner.planner == nil || runner.collision == nil || runner.temp == "" {
		return nil, errors.New("persistent canonical planner is not started")
	}
	if len(payload) == 0 || len(payload) > limits.MaxPlanRequestBytes {
		return nil, errors.New("canonical planner input is empty or exceeds the request bound")
	}
	requestDir, err := os.MkdirTemp(runner.temp, "request-")
	if err != nil {
		return nil, err
	}
	defer os.RemoveAll(requestDir)
	input := filepath.Join(requestDir, "request.json")
	output := filepath.Join(requestDir, "report.json")
	if err := os.WriteFile(input, payload, 0o600); err != nil {
		return nil, err
	}
	runner.nextRequestID++
	requestID := fmt.Sprintf("plan-%d", runner.nextRequestID)
	envelope, err := json.Marshal(map[string]string{
		"request_id": requestID,
		"input":      input,
		"output":     output,
	})
	if err != nil {
		return nil, err
	}
	envelope = append(envelope, '\n')
	if _, err := runner.plannerInput.Write(envelope); err != nil {
		return nil, fmt.Errorf("write persistent planner request: %w", err)
	}

	type responseLine struct {
		line []byte
		err  error
	}
	response := make(chan responseLine, 1)
	go func() {
		line, readErr := runner.plannerOutput.ReadBytes('\n')
		response <- responseLine{line: line, err: readErr}
	}()
	var acknowledgement responseLine
	select {
	case <-ctx.Done():
		runner.stopLocked(context.Background())
		return nil, ctx.Err()
	case <-runner.planner.done:
		detail := strings.TrimSpace(runner.plannerStderr.String())
		if detail == "" && runner.planner.waitError() != nil {
			detail = runner.planner.waitError().Error()
		}
		return nil, fmt.Errorf("persistent canonical planner exited: %s", detail)
	case acknowledgement = <-response:
	}
	if acknowledgement.err != nil {
		return nil, fmt.Errorf(
			"read persistent planner response: %w", acknowledgement.err)
	}
	var ack struct {
		OK        bool   `json:"ok"`
		RequestID string `json:"request_id"`
		ExitCode  int    `json:"exit_code"`
		Error     string `json:"error"`
	}
	if err := json.Unmarshal(acknowledgement.line, &ack); err != nil {
		return nil, fmt.Errorf("decode persistent planner response: %w", err)
	}
	if !ack.OK || ack.RequestID != requestID || ack.ExitCode != 0 {
		detail := strings.TrimSpace(ack.Error)
		if detail == "" {
			detail = strings.TrimSpace(runner.plannerStderr.String())
		}
		return nil, fmt.Errorf("persistent canonical planner failed: %s", detail)
	}
	file, err := os.Open(output)
	if err != nil {
		return nil, errors.New("canonical planner did not produce a report")
	}
	defer file.Close()
	raw, err := io.ReadAll(io.LimitReader(
		file, int64(limits.MaxPlanReportBytes)+1))
	if err != nil {
		return nil, err
	}
	if len(raw) == 0 || len(raw) > limits.MaxPlanReportBytes {
		return nil, errors.New(
			"canonical planner report is empty or exceeds the response bound")
	}
	return raw, nil
}

func (runner *canonicalProcessRunner) Stop(ctx context.Context) error {
	if runner == nil {
		return nil
	}
	runner.mu.Lock()
	defer runner.mu.Unlock()
	return runner.stopLocked(ctx)
}

func (runner *canonicalProcessRunner) stopLocked(ctx context.Context) error {
	if runner.plannerInput != nil {
		_ = runner.plannerInput.Close()
	}
	plannerErr := stopManagedPlannerProcess(ctx, runner.planner)
	collisionErr := stopManagedPlannerProcess(ctx, runner.collision)
	if runner.temp != "" {
		_ = os.RemoveAll(runner.temp)
	}
	runner.temp = ""
	runner.planner = nil
	runner.collision = nil
	runner.plannerInput = nil
	runner.plannerOutput = nil
	runner.plannerStderr = nil
	if plannerErr != nil {
		return plannerErr
	}
	return collisionErr
}

func waitForCollisionServer(
	ctx context.Context,
	readyFile string,
	process *managedPlannerProcess,
	stderr *boundedBuffer,
) (string, int, error) {
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for {
		raw, err := os.ReadFile(readyFile)
		if err == nil {
			var ready struct {
				Schema string `json:"schema"`
				Host   string `json:"host"`
				Port   int    `json:"port"`
			}
			if json.Unmarshal(raw, &ready) == nil &&
				ready.Schema == "robot-v4.tesseract-collision-server-ready.v1" &&
				ready.Host == "127.0.0.1" && ready.Port > 0 && ready.Port <= 65535 {
				return ready.Host, ready.Port, nil
			}
		}
		select {
		case <-ctx.Done():
			return "", 0, ctx.Err()
		case <-process.done:
			detail := strings.TrimSpace(stderr.String())
			if detail == "" && process.waitError() != nil {
				detail = process.waitError().Error()
			}
			return "", 0, fmt.Errorf(
				"persistent Tesseract collision worker exited: %s", detail)
		case <-ticker.C:
		}
	}
}

type boundedBuffer struct {
	mu      sync.Mutex
	buffer  bytes.Buffer
	maximum int
}

func (writer *boundedBuffer) Write(raw []byte) (int, error) {
	writer.mu.Lock()
	defer writer.mu.Unlock()
	original := len(raw)
	if writer.maximum > writer.buffer.Len() {
		remaining := writer.maximum - writer.buffer.Len()
		if remaining < len(raw) {
			raw = raw[:remaining]
		}
		_, _ = writer.buffer.Write(raw)
	}
	return original, nil
}

func (writer *boundedBuffer) String() string {
	writer.mu.Lock()
	defer writer.mu.Unlock()
	return writer.buffer.String()
}

func isolatedPlannerEnvironment(
	temp, collisionWorker, collisionServer, plannerURDF string,
) []string {
	environment := []string{
		"HOME=" + temp,
		"LANG=C",
		"LC_ALL=C",
		"PATH=/usr/local/bin:/usr/bin:/bin",
		"ROBOT_V4_SIMULATION_MODE=1",
		"TMPDIR=" + temp,
		"TZ=UTC",
	}
	if collisionWorker != "" {
		environment = append(
			environment,
			"ROBOT_V4_TESSERACT_COLLISION_WORKER="+collisionWorker,
		)
	}
	if collisionServer != "" {
		environment = append(
			environment,
			"ROBOT_V4_TESSERACT_COLLISION_SERVER="+collisionServer,
		)
	}
	if plannerURDF != "" {
		environment = append(
			environment,
			"ROBOT_V4_TESSERACT_URDF_PATH="+plannerURDF,
		)
	}
	for _, item := range os.Environ() {
		name, _, found := strings.Cut(item, "=")
		if !found {
			continue
		}
		switch name {
		case "LD_LIBRARY_PATH", "PYTHONHOME", "PYTHONPATH":
			environment = append(environment, item)
		}
	}
	return environment
}

func validateStartRequest(request StartRequest) error {
	if err := request.Topology.Validate(); err != nil {
		return err
	}
	if request.Topology.ProfileID != "local-simulator" ||
		request.Topology.RobotModel != planning.RobotModelRosie1400V3 {
		return errors.New("PC-native canonical planner supports only local-simulator/rosie_1400_v3")
	}
	if err := request.Limits.Validate(); err != nil {
		return err
	}
	if request.Policy != requiredSafetyPolicy() {
		return errors.New("local simulator start policy does not require every no-output invariant")
	}
	return nil
}

func summarizeCanonicalReport(raw []byte) string {
	var report map[string]json.RawMessage
	if err := json.Unmarshal(raw, &report); err != nil {
		return "report=<undecodable>"
	}
	keys := []string{
		"accepted",
		"collision_gate_clear",
		"collision_check_state",
		"collision_error",
		"collision_worker_error",
		"worker_error",
		"planner_status",
		"error",
		"rejection_reason",
		"message",
		"detail",
		"details",
		"planned_sample_count",
		"name",
		"self_test",
	}
	parts := make([]string, 0, len(keys)+8)
	for _, key := range keys {
		if value, ok := report[key]; ok {
			parts = append(parts, fmt.Sprintf("%s=%s", key, string(value)))
		}
	}
	for key, value := range report {
		lower := strings.ToLower(key)
		if strings.Contains(lower, "collision") ||
			strings.Contains(lower, "error") ||
			strings.Contains(lower, "worker") {
			already := false
			prefix := key + "="
			for _, part := range parts {
				if strings.HasPrefix(part, prefix) {
					already = true
					break
				}
			}
			if !already {
				text := string(value)
				if len(text) > 240 {
					text = text[:240] + "…"
				}
				parts = append(parts, fmt.Sprintf("%s=%s", key, text))
			}
		}
	}
	if len(parts) == 0 {
		return "report=<no summary fields>"
	}
	return strings.Join(parts, " ")
}

func validateCanonicalReport(raw []byte, requireReady bool) error {
	var report struct {
		Name                          string             `json:"name"`
		SelfTest                      string             `json:"self_test"`
		PlannerOwner                  string             `json:"planner_owner"`
		PlannerBackend                string             `json:"planner_backend"`
		PlannerStatus                 string             `json:"planner_status"`
		RobotModel                    string             `json:"robot_model"`
		HardwareAccess                *bool              `json:"hardware_access"`
		CollisionGateClear            *bool              `json:"collision_gate_clear"`
		CollisionState                string             `json:"collision_check_state"`
		CollisionCheckedWaypointCount *int               `json:"collision_checked_waypoint_count"`
		Accepted                      *bool              `json:"accepted"`
		PlannedSamples                int                `json:"planned_sample_count"`
		Trajectory                    *[]json.RawMessage `json:"trajectory"`
	}
	decoder := json.NewDecoder(bytes.NewReader(raw))
	if err := decoder.Decode(&report); err != nil {
		return fmt.Errorf("decode canonical planner report: %w", err)
	}
	var trailing json.RawMessage
	if err := decoder.Decode(&trailing); !errors.Is(err, io.EOF) {
		return errors.New("canonical planner report contains multiple JSON values")
	}
	if report.Name != canonicalBinaryName ||
		report.SelfTest != "spreadsheet-tesseract-plan" ||
		report.PlannerOwner != planning.MotionServerPlannerOwner ||
		report.PlannerBackend != planning.MotionServerPlannerBackend ||
		report.PlannerStatus != "planned" ||
		report.RobotModel != string(planning.RobotModelRosie1400V3) {
		return errors.New("canonical planner identity or planned state is invalid")
	}
	if report.HardwareAccess == nil || *report.HardwareAccess {
		return errors.New("canonical planner did not prove hardware_access=false")
	}
	if report.Trajectory == nil || report.PlannedSamples < 1 ||
		len(*report.Trajectory) != report.PlannedSamples {
		return errors.New("canonical planner trajectory evidence is incomplete")
	}
	if requireReady &&
		(report.Accepted == nil || !*report.Accepted ||
			report.CollisionGateClear == nil || !*report.CollisionGateClear ||
			report.CollisionState != "tesseract_discrete_contact_checked" ||
			report.CollisionCheckedWaypointCount == nil ||
			*report.CollisionCheckedWaypointCount != report.PlannedSamples) {
		return errors.New("canonical planner collision/acceptance readiness proof is unavailable")
	}
	return nil
}

func regularFile(path string) bool {
	info, err := os.Stat(path)
	return err == nil && info.Mode().IsRegular()
}

func regularExecutable(path string) bool {
	info, err := os.Stat(path)
	if runtime.GOOS == "windows" &&
		!strings.EqualFold(filepath.Ext(path), ".exe") {
		return false
	}
	return err == nil &&
		info.Mode().IsRegular() &&
		(runtime.GOOS == "windows" || info.Mode().Perm()&0o111 != 0)
}

func isELFExecutable(path string) bool {
	file, err := os.Open(path)
	if err != nil {
		return false
	}
	defer file.Close()
	header := make([]byte, 4)
	if _, err := io.ReadFull(file, header); err != nil {
		return false
	}
	return header[0] == 0x7f &&
		header[1] == 'E' &&
		header[2] == 'L' &&
		header[3] == 'F'
}

func collisionPythonPath(repoRoot string) string {
	relative := defaultCollisionPython
	if runtime.GOOS == "windows" {
		relative = "tesseract/v1/.pixi/envs/default/python.exe"
	}
	return filepath.Join(repoRoot, filepath.FromSlash(relative))
}

func collisionWorkerCommand(repoRoot string) string {
	python := collisionPythonPath(repoRoot)
	script := filepath.Join(repoRoot, filepath.FromSlash(defaultCollisionScript))
	if !regularExecutable(python) || !regularFile(script) {
		return ""
	}
	return shellQuote(python) + " " + shellQuote(script)
}

func shellQuote(value string) string {
	if runtime.GOOS == "windows" {
		return `"` + strings.ReplaceAll(value, `"`, `""`) + `"`
	}
	return "'" + strings.ReplaceAll(value, "'", "'\\''") + "'"
}

func localAcceptedPlanAssets(
	repoRoot string,
	binary string,
) (planning.AcceptedPlanAssets, error) {
	plannerDigest, err := digestFile(binary)
	if err != nil {
		return planning.AcceptedPlanAssets{}, err
	}
	urdfPath := filepath.Join(repoRoot, filepath.FromSlash(defaultPlannerURDF))
	urdfDigest, err := digestFile(urdfPath)
	if err != nil {
		return planning.AcceptedPlanAssets{}, err
	}
	meshRoot := filepath.Join(
		repoRoot,
		filepath.FromSlash("robot_description/robots/rosie_1400_v3/meshes"),
	)
	meshDigest, err := digestDirectory(meshRoot)
	if err != nil {
		return planning.AcceptedPlanAssets{}, err
	}
	collisionDigest := digestValues("robot-v4-collision-world-v1", urdfDigest, meshDigest)
	calibrationDigest := digestValues(
		"robot-v4-local-simulator-no-physical-calibration-v1",
	)
	return planning.AcceptedPlanAssets{
		PlannerBuildDigest:   plannerDigest,
		TrajectoryMethod:     "linear-tcp-dense-csp",
		ManifestRevision:     "local-simulator:" + strings.TrimPrefix(plannerDigest, "sha256:")[:16],
		CellConfigRevision:   "local-simulator-cell-v1",
		CalibrationDigest:    calibrationDigest,
		URDFDigest:           urdfDigest,
		MeshDigest:           meshDigest,
		CollisionWorldDigest: collisionDigest,
		SettingsRevision:     1,
	}, nil
}

func digestFile(path string) (string, error) {
	file, err := os.Open(path)
	if err != nil {
		return "", err
	}
	defer file.Close()
	hash := sha256.New()
	if _, err := io.Copy(hash, file); err != nil {
		return "", err
	}
	return "sha256:" + hex.EncodeToString(hash.Sum(nil)), nil
}

func digestDirectory(root string) (string, error) {
	paths := []string{}
	err := filepath.WalkDir(root, func(path string, entry os.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		if entry.Type().IsRegular() {
			paths = append(paths, path)
		}
		return nil
	})
	if err != nil {
		return "", err
	}
	if len(paths) == 0 {
		return "", errors.New("canonical Robot V4 mesh directory is empty")
	}
	sort.Strings(paths)
	hash := sha256.New()
	for _, path := range paths {
		relative, err := filepath.Rel(root, path)
		if err != nil {
			return "", err
		}
		fileDigest, err := digestFile(path)
		if err != nil {
			return "", err
		}
		_, _ = io.WriteString(hash, filepath.ToSlash(relative))
		_, _ = io.WriteString(hash, "\x00")
		_, _ = io.WriteString(hash, fileDigest)
		_, _ = io.WriteString(hash, "\n")
	}
	return "sha256:" + hex.EncodeToString(hash.Sum(nil)), nil
}

func digestValues(values ...string) string {
	hash := sha256.New()
	for _, value := range values {
		_, _ = io.WriteString(hash, value)
		_, _ = io.WriteString(hash, "\n")
	}
	return "sha256:" + hex.EncodeToString(hash.Sum(nil))
}

func randomBootID() (string, error) {
	raw := make([]byte, 16)
	if _, err := rand.Read(raw); err != nil {
		return "", err
	}
	return "pc-" + hex.EncodeToString(raw), nil
}

func available() BackendAvailability {
	return BackendAvailability{
		Schema:            AvailabilitySchema,
		Available:         true,
		Code:              AvailabilityReady,
		Reason:            "",
		CanonicalPlanner:  true,
		ProcessBounded:    true,
		PersistentProcess: false,
		ExplicitOptIn:     true,
	}
}

func unavailable(code AvailabilityCode, reason string) BackendAvailability {
	return BackendAvailability{
		Schema:            AvailabilitySchema,
		Available:         false,
		Code:              code,
		Reason:            boundedText(reason, 240),
		CanonicalPlanner:  true,
		ProcessBounded:    boundedProcessSupported(),
		PersistentProcess: false,
		ExplicitOptIn:     true,
	}
}

func boundedReason(err error) string {
	if err == nil {
		return "canonical planner unavailable"
	}
	return boundedText(err.Error(), 240)
}

func boundedText(value string, maximum int) string {
	value = strings.TrimSpace(value)
	if len(value) <= maximum {
		return value
	}
	return value[:maximum]
}
