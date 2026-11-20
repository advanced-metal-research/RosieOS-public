package simulator

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

type retryProbeRunner struct {
	calls int
	raw   []byte
}

type persistentProbeRunner struct {
	starts int
	stops  int
	runs   int
	raw    []byte
}

func (runner *persistentProbeRunner) Persistent() bool { return true }

func (runner *persistentProbeRunner) Start(
	context.Context,
	ResourceLimits,
) error {
	runner.starts++
	return nil
}

func (runner *persistentProbeRunner) Stop(context.Context) error {
	runner.stops++
	return nil
}

func (runner *persistentProbeRunner) Run(
	context.Context,
	[]byte,
	ResourceLimits,
) ([]byte, error) {
	runner.runs++
	return runner.raw, nil
}

func (runner *retryProbeRunner) Run(
	context.Context,
	[]byte,
	ResourceLimits,
) ([]byte, error) {
	runner.calls++
	if runner.calls == 1 {
		return nil, errors.New("transient planner interruption")
	}
	return runner.raw, nil
}

func TestPCNativeBackendRequiresExplicitOptIn(t *testing.T) {
	backend := NewPCNativeBackend(PCBackendConfig{
		Enabled:  false,
		RepoRoot: t.TempDir(),
	})
	availability := backend.Availability()
	if availability.Available ||
		availability.Code != AvailabilityExplicitOptInRequired ||
		!availability.ExplicitOptIn ||
		availability.PersistentProcess {
		t.Fatalf("unexpected disabled availability: %+v", availability)
	}
}

func TestPCNativeBackendReportsMissingCanonicalDependency(t *testing.T) {
	root := t.TempDir()
	backend := NewPCNativeBackend(PCBackendConfig{
		Enabled:  true,
		RepoRoot: root,
	})
	availability := backend.Availability()
	if availability.Available {
		t.Fatalf("empty root reported available: %+v", availability)
	}
	if runtime.GOOS == "linux" || runtime.GOOS == "windows" {
		if availability.Code != AvailabilityCanonicalBinary {
			t.Fatalf("availability code=%q want %q", availability.Code, AvailabilityCanonicalBinary)
		}
	} else if availability.Code != AvailabilityProcessContainment {
		t.Fatalf("unsupported platform availability=%+v", availability)
	}
	if strings.Contains(availability.Reason, filepath.Clean(root)) {
		t.Fatalf("public availability leaked an absolute path: %+v", availability)
	}
}

func TestCanonicalProbeInterruptionRemainsRetryable(t *testing.T) {
	root := t.TempDir()
	probePath := filepath.Join(root, filepath.FromSlash(defaultProbeInput))
	if err := os.MkdirAll(filepath.Dir(probePath), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(probePath, []byte(`{"probe":true}`), 0o600); err != nil {
		t.Fatal(err)
	}
	readyReport := map[string]any{
		"name":                  "robot-v4-cartesiand",
		"self_test":             "spreadsheet-tesseract-plan",
		"planner_owner":         "motion-server",
		"planner_backend":       "motion_server_tesseract_trajectory_v1",
		"planner_status":        "planned",
		"robot_model":           "rosie_1400_v3",
		"hardware_access":       false,
		"collision_gate_clear":  true,
		"collision_check_state": "tesseract_discrete_contact_checked",
		"accepted":              true,
		"planned_sample_count":  1,
		"trajectory":            []map[string]any{{"waypoint_index": 0}},
	}
	readyReport["collision_checked_waypoint_count"] = 1
	report, err := json.Marshal(readyReport)
	if err != nil {
		t.Fatal(err)
	}
	now := time.Date(2026, 7, 31, 12, 0, 0, 0, time.UTC)
	runner := &retryProbeRunner{raw: report}
	backend := &PCNativeBackend{
		config: PCBackendConfig{
			Enabled:  true,
			RepoRoot: root,
			ProofTTL: 5 * time.Second,
			Clock:    func() time.Time { return now },
		},
		runner:       runner,
		availability: available(),
	}
	request := StartRequest{
		Topology: NewPCTopology(
			"local-simulator",
			planning.RobotModelRosie1400V3,
			"127.0.0.1",
		),
		Limits: DefaultResourceLimits(),
		Policy: requiredSafetyPolicy(),
	}
	if _, err := backend.Start(context.Background(), request); err == nil {
		t.Fatal("transient probe interruption was unexpectedly accepted")
	}
	if availability := backend.Availability(); !availability.Available {
		t.Fatalf("transient probe permanently poisoned availability: %+v", availability)
	}
	status, err := backend.Start(context.Background(), request)
	if err != nil || !status.PlannerReady || !status.SimulatorReady {
		t.Fatalf("retry did not recover the canonical planner: status=%+v err=%v", status, err)
	}
}

func TestPersistentPlannerStartsOnceAndServesRepeatedPlans(t *testing.T) {
	root := t.TempDir()
	probePath := filepath.Join(root, filepath.FromSlash(defaultProbeInput))
	if err := os.MkdirAll(filepath.Dir(probePath), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(probePath, []byte(`{"probe":true}`), 0o600); err != nil {
		t.Fatal(err)
	}
	readyReport := map[string]any{
		"name":                  "robot-v4-cartesiand",
		"self_test":             "spreadsheet-tesseract-plan",
		"planner_owner":         "motion-server",
		"planner_backend":       "motion_server_tesseract_trajectory_v1",
		"planner_status":        "planned",
		"robot_model":           "rosie_1400_v3",
		"hardware_access":       false,
		"collision_gate_clear":  true,
		"collision_check_state": "tesseract_discrete_contact_checked",
		"accepted":              true,
		"planned_sample_count":  1,
		"trajectory":            []map[string]any{{"waypoint_index": 0}},
	}
	readyReport["collision_checked_waypoint_count"] = 1
	report, err := json.Marshal(readyReport)
	if err != nil {
		t.Fatal(err)
	}
	runner := &persistentProbeRunner{raw: report}
	backend := &PCNativeBackend{
		config: PCBackendConfig{
			Enabled:  true,
			RepoRoot: root,
			ProofTTL: 5 * time.Second,
			Clock:    time.Now,
		},
		runner:       runner,
		availability: available(),
	}
	backend.availability = backend.readyAvailability()
	if !backend.Availability().PersistentProcess {
		t.Fatal("persistent runner was not reported as persistent")
	}
	request := StartRequest{
		Topology: NewPCTopology(
			"local-simulator",
			planning.RobotModelRosie1400V3,
			"127.0.0.1",
		),
		Limits: DefaultResourceLimits(),
		Policy: requiredSafetyPolicy(),
	}
	if _, err := backend.Start(context.Background(), request); err != nil {
		t.Fatal(err)
	}
	for revision := uint64(1); revision <= 2; revision++ {
		if _, err := backend.SubmitPlan(
			context.Background(),
			planning.MotionServerSubmission{
				PlanRevision: revision,
				Payload:      json.RawMessage(`{"program":true}`),
			},
		); err != nil {
			t.Fatalf("plan %d failed: %v", revision, err)
		}
	}
	if runner.starts != 1 || runner.runs != 3 {
		t.Fatalf(
			"persistent lifecycle starts=%d runs=%d, want starts=1 runs=3",
			runner.starts, runner.runs,
		)
	}
	if err := backend.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	if runner.stops != 1 {
		t.Fatalf("persistent lifecycle stops=%d want 1", runner.stops)
	}
}

func TestCanonicalReadinessReportRequiresHardwareAndCollisionProof(t *testing.T) {
	report := map[string]any{
		"name":                  "robot-v4-cartesiand",
		"self_test":             "spreadsheet-tesseract-plan",
		"planner_owner":         "motion-server",
		"planner_backend":       "motion_server_tesseract_trajectory_v1",
		"planner_status":        "planned",
		"robot_model":           "rosie_1400_v3",
		"hardware_access":       false,
		"collision_gate_clear":  true,
		"collision_check_state": "tesseract_discrete_contact_checked",
		"accepted":              true,
		"planned_sample_count":  1,
		"trajectory":            []map[string]any{{"waypoint_index": 0}},
	}
	report["collision_checked_waypoint_count"] = 1
	raw, err := json.Marshal(report)
	if err != nil {
		t.Fatal(err)
	}
	if err := validateCanonicalReport(raw, true); err != nil {
		t.Fatalf("safe canonical report rejected: %v", err)
	}
	delete(report, "collision_checked_waypoint_count")
	missingCoverage, _ := json.Marshal(report)
	if err := validateCanonicalReport(missingCoverage, true); err == nil {
		t.Fatal("canonical report without collision coverage count accepted")
	}
	report["collision_checked_waypoint_count"] = 2
	mismatchedCoverage, _ := json.Marshal(report)
	if err := validateCanonicalReport(mismatchedCoverage, true); err == nil {
		t.Fatal("canonical report with mismatched collision coverage accepted")
	}
	report["collision_checked_waypoint_count"] = 1
	report["hardware_access"] = true
	unsafe, _ := json.Marshal(report)
	if err := validateCanonicalReport(unsafe, true); err == nil {
		t.Fatal("hardware-enabled canonical report accepted")
	}
	report["hardware_access"] = false
	report["collision_check_state"] = "unavailable_worker_not_provisioned"
	unavailableReport, _ := json.Marshal(report)
	if err := validateCanonicalReport(unavailableReport, true); err == nil {
		t.Fatal("canonical report without collision readiness accepted")
	}
}

func TestPlannerEnvironmentStripsPhysicalServiceAndBrokerConfiguration(t *testing.T) {
	t.Setenv("MOTION_SERVER_V1_MANIFEST", "physical.json")
	t.Setenv("ROBOT_V4_MANIFEST", "robot-v4-physical.json")
	t.Setenv("ROBOT_V4_RTCORE_STATUS_SUBJECT", "physical.status")
	t.Setenv("NATS_URL", "nats://physical.example:4222")
	t.Setenv("RTCORE_TARGET", "physical.example:46000")
	root := t.TempDir()
	urdf := filepath.Join(root, "robot.urdf")
	environment := isolatedPlannerEnvironment(
		root,
		"python collision_check.py",
		"127.0.0.1:43117",
		urdf,
	)
	joined := strings.Join(environment, "\n")
	for _, forbidden := range []string{
		"MOTION_SERVER_V1_MANIFEST",
		"ROBOT_V4_MANIFEST",
		"ROBOT_V4_RTCORE_STATUS_SUBJECT",
		"NATS_URL",
		"RTCORE_TARGET",
	} {
		if strings.Contains(joined, forbidden) {
			t.Fatalf("isolated planner environment retained %s: %s", forbidden, joined)
		}
	}
	if !strings.Contains(joined, "ROBOT_V4_SIMULATION_MODE=1") {
		t.Fatalf("isolated planner environment lacks explicit simulation mode: %s", joined)
	}
	if !strings.Contains(joined, "ROBOT_V4_TESSERACT_COLLISION_WORKER=python collision_check.py") {
		t.Fatalf("isolated planner environment lacks Robot V4 collision worker: %s", joined)
	}
	if !strings.Contains(joined, "ROBOT_V4_TESSERACT_COLLISION_SERVER=127.0.0.1:43117") {
		t.Fatalf("isolated planner environment lacks persistent collision server: %s", joined)
	}
	if !strings.Contains(joined, "ROBOT_V4_TESSERACT_URDF_PATH="+urdf) {
		t.Fatalf("isolated planner environment lacks Robot V4 URDF binding: %s", joined)
	}
}

func TestCanonicalPlannerDefaultsToMotionServerV1(t *testing.T) {
	if defaultCanonicalBinary !=
		"motion-server/v1/bin/robot-v4-cartesiand" {
		t.Fatalf("unexpected canonical planner binary %q", defaultCanonicalBinary)
	}
	if defaultProbeInput !=
		"motion-server/v1/testdata/spreadsheet-plan-clear-input.json" {
		t.Fatalf("unexpected canonical planner probe %q", defaultProbeInput)
	}
}

func TestIsELFExecutableRecognizesMagic(t *testing.T) {
	dir := t.TempDir()
	elfPath := filepath.Join(dir, "fake-elf")
	if err := os.WriteFile(elfPath, []byte{0x7f, 'E', 'L', 'F', 0x02}, 0o644); err != nil {
		t.Fatal(err)
	}
	if !isELFExecutable(elfPath) {
		t.Fatal("expected ELF magic to be recognized")
	}
	other := filepath.Join(dir, "not-elf")
	if err := os.WriteFile(other, []byte("MZ"), 0o644); err != nil {
		t.Fatal(err)
	}
	if isELFExecutable(other) {
		t.Fatal("non-ELF file was incorrectly treated as ELF")
	}
}

func TestSummarizeCanonicalReportIncludesCollisionFields(t *testing.T) {
	summary := summarizeCanonicalReport([]byte(`{
		"accepted": true,
		"collision_gate_clear": false,
		"collision_check_state": "failed_worker_error",
		"collision_check_error": "ValueError: unsupported robot_model: ''"
	}`))
	if !strings.Contains(summary, `collision_check_error="ValueError: unsupported robot_model: ''"`) {
		t.Fatalf("summary missing collision error: %s", summary)
	}
}
