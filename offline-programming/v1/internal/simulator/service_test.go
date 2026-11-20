package simulator

import (
	"context"
	"encoding/json"
	"errors"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

type fakeBackend struct {
	mu          sync.Mutex
	snapshot    RuntimeSnapshot
	startErr    error
	probeErr    error
	planErr     error
	stopErr     error
	startCalls  int
	probeCalls  int
	planCalls   int
	stopCalls   int
	startInput  StartRequest
	submission  planning.MotionServerSubmission
	planResult  func(planning.MotionServerSubmission) []byte
	planEntered chan struct{}
	planRelease chan struct{}
}

func (backend *fakeBackend) AcceptedPlanAssets() (planning.AcceptedPlanAssets, error) {
	return planning.AcceptedPlanAssets{
		PlannerBuildDigest:   "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
		TrajectoryMethod:     "linear-tcp-dense-csp",
		ManifestRevision:     "local-simulator-test-manifest",
		CellConfigRevision:   "local-simulator-test-cell",
		CalibrationDigest:    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
		URDFDigest:           "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
		MeshDigest:           "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
		CollisionWorldDigest: "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
		SettingsRevision:     1,
	}, nil
}

func (backend *fakeBackend) Start(_ context.Context, request StartRequest) (RuntimeSnapshot, error) {
	backend.mu.Lock()
	defer backend.mu.Unlock()
	backend.startCalls++
	backend.startInput = request
	return backend.snapshot, backend.startErr
}

func (backend *fakeBackend) Probe(context.Context) (RuntimeSnapshot, error) {
	backend.mu.Lock()
	defer backend.mu.Unlock()
	backend.probeCalls++
	return backend.snapshot, backend.probeErr
}

func (backend *fakeBackend) SubmitPlan(
	_ context.Context,
	submission planning.MotionServerSubmission,
) ([]byte, error) {
	backend.mu.Lock()
	backend.planCalls++
	backend.submission = submission
	entered := backend.planEntered
	release := backend.planRelease
	result := backend.planResult
	err := backend.planErr
	backend.mu.Unlock()
	if entered != nil {
		close(entered)
	}
	if release != nil {
		<-release
	}
	if err != nil {
		return nil, err
	}
	return result(submission), nil
}

func (backend *fakeBackend) Stop(context.Context) error {
	backend.mu.Lock()
	defer backend.mu.Unlock()
	backend.stopCalls++
	return backend.stopErr
}

func newTestService(
	t *testing.T,
	now *time.Time,
	backend *fakeBackend,
) *Service {
	t.Helper()
	topology := NewPCTopology(
		"local-simulator",
		planning.RobotModelRosie1400V3,
		"127.0.0.1",
	)
	service, err := NewService(Options{
		ServiceID:             "offline-programming-local",
		Topology:              topology,
		Limits:                DefaultResourceLimits(),
		ProofTTL:              time.Second,
		PlannerRequestSubject: "robot.v4.local-simulator.plan",
		Clock:                 func() time.Time { return *now },
	}, backend)
	if err != nil {
		t.Fatal(err)
	}
	return service
}

func runtimeSnapshot(now time.Time) RuntimeSnapshot {
	return RuntimeSnapshot{
		ProfileID:      "local-simulator",
		RobotModel:     planning.RobotModelRosie1400V3,
		PlannerReady:   true,
		SimulatorReady: true,
		PlanTransport:  PlanTransportInProcess,
		Safety:         safeProof(now),
	}
}

func localPlanRequest(now time.Time) planning.NormalizedPlanRequest {
	rows := []planning.PlanRow{
		{
			RowID: "row-1", Revision: 3, Enabled: true, RowType: "motion", MoveType: "line_tcp",
			MotionPhase: "travel", ToolFrame: "torch_tcp", WorkFrame: "world",
			TCPVelocityMS: 0.05, JointVelocityRadS: 0.25,
		},
		{
			RowID: "row-2", Revision: 3, Enabled: true, RowType: "motion", MoveType: "line_tcp",
			MotionPhase: "weld", ToolFrame: "torch_tcp", WorkFrame: "world",
			TCPVelocityMS: 0.05, JointVelocityRadS: 0.25, WeldEnable: true,
		},
	}
	rows[1].TargetTCPM[0] = 0.1
	rows[1].TargetJointsRad[0] = 0.1
	request := planning.NormalizedPlanRequest{
		Schema:              planning.NormalizedPlanRequestSchema,
		RequestID:           "request-local-1",
		ProjectID:           "project-1",
		ProjectRevision:     3,
		ProgramID:           "program-1",
		ProgramRevision:     3,
		BaseProgramRevision: 2,
		RobotModel:          planning.RobotModelRosie1400V3,
		Target:              planning.TargetIdentity{ProfileID: "local-simulator"},
		SubmittedAt:         now.Add(-time.Millisecond),
		Rows:                rows,
	}
	programRows := make([]map[string]any, len(rows))
	for index, row := range rows {
		programRows[index] = map[string]any{
			"row_id":               row.RowID,
			"revision":             row.Revision,
			"row_hash":             "sha256:test-row",
			"enabled":              row.Enabled,
			"row_type":             row.RowType,
			"move_type":            row.MoveType,
			"motion_phase":         row.MotionPhase,
			"tool_frame":           row.ToolFrame,
			"work_frame":           row.WorkFrame,
			"target_tcp_m":         row.TargetTCPM,
			"target_joints_rad":    row.TargetJointsRad,
			"tcp_velocity_m_s":     row.TCPVelocityMS,
			"joint_velocity_rad_s": row.JointVelocityRadS,
			"weld_enable":          row.WeldEnable,
		}
	}
	request.Program, _ = json.Marshal(map[string]any{
		"schema_version":        1,
		"program_id":            request.ProgramID,
		"program_name":          "Simulator test",
		"program_revision":      request.ProgramRevision,
		"base_program_revision": request.BaseProgramRevision,
		"robot_model":           request.RobotModel,
		"robotic-cell-type":     request.RobotModel,
		"execution_mode":        "simulation",
		"rows":                  programRows,
	})
	return request
}

func successfulPlanReport(
	t *testing.T,
	submission planning.MotionServerSubmission,
) []byte {
	t.Helper()
	var request struct {
		ProgramID       string              `json:"program_id"`
		ProgramRevision uint64              `json:"program_revision"`
		RobotModel      planning.RobotModel `json:"robot_model"`
	}
	if err := json.Unmarshal(submission.Payload, &request); err != nil {
		t.Fatal(err)
	}
	trajectory := []map[string]any{
		{
			"row_index": 0, "waypoint_index": 0, "time_s": 0.0,
			"move_type": "line_tcp", "motion_phase": "travel", "weld_on": false,
			"tcp_pose":               []float64{0, 0, 0, 0, 0, 0},
			"positions_rad":          []float64{0, 0, 0, 0, 0, 0, 0, 0, 0},
			"applied_velocity_scale": 1.0,
		},
		{
			"row_index": 1, "waypoint_index": 1, "time_s": 1.0,
			"move_type": "line_tcp", "motion_phase": "weld", "weld_on": true,
			"tcp_pose":               []float64{0.1, 0, 0, 0, 0, 0},
			"positions_rad":          []float64{0.1, 0, 0, 0, 0, 0, 0, 0, 0},
			"applied_velocity_scale": 1.0,
		},
	}
	report := map[string]any{
		"plan_request_revision":          submission.PlanRevision,
		"planner_owner":                  planning.MotionServerPlannerOwner,
		"planner_backend":                planning.MotionServerPlannerBackend,
		"planner_status":                 "planned",
		"program_id":                     request.ProgramID,
		"program_revision":               request.ProgramRevision,
		"accepted_program_revision":      request.ProgramRevision,
		"program_hash":                   "fnv1a64:1234",
		"robot_model":                    request.RobotModel,
		"validation_state":               "valid",
		"accepted":                       true,
		"play_enabled":                   true,
		"runtime_preload_enabled":        true,
		"collision_gate_clear":           true,
		"collision_check_state":          "tesseract_discrete_contact_checked",
		"collision_points":               []any{},
		"singularity_analysis_available": true,
		"singularity_gate_clear":         true,
		"singularity_check_state":        "jacobian_svd_checked",
		"singularity_points":             []any{},
		"limit_check_state":              "clear",
		"unreachable_points":             []any{},
		"planned_sample_count":           2,
		"trajectory":                     trajectory,
		"hardware_access":                false,
	}
	report["collision_checked_waypoint_count"] = len(trajectory)
	raw, err := json.Marshal(report)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}

func TestServiceLifecycleRequiresRuntimeSafeProof(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	backend := &fakeBackend{snapshot: runtimeSnapshot(now)}
	service := newTestService(t, &now, backend)
	if status := service.Status(); status.Phase != PhaseStopped || status.Readiness.Ready {
		t.Fatalf("initial status = %+v", status)
	}
	status, err := service.Start(context.Background())
	if err != nil || status.Phase != PhaseReady || !status.Readiness.Ready {
		t.Fatalf("start status=%+v err=%v", status, err)
	}
	wire, err := json.Marshal(StatusEnvelope{Status: status})
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(wire), `"readiness":{"ready":true,"reasons":[]}`) {
		t.Fatalf("ready status did not preserve the JSON array contract: %s", wire)
	}
	if backend.startInput.Topology.PhysicalServiceStart ||
		backend.startInput.Topology.CommandTransport ||
		backend.startInput.Policy.EtherCATOutputEnabled ||
		backend.startInput.Policy.MotionOutputEnabled ||
		backend.startInput.Policy.HardwareAccess ||
		backend.startInput.Policy.PhysicalServicesStarted ||
		backend.startInput.Policy.CommandSubmissionEnabled ||
		!backend.startInput.Policy.Simulation ||
		!backend.startInput.Policy.NoRealMovement ||
		!backend.startInput.Policy.LoopbackOnly {
		t.Fatalf("unsafe start policy = %+v", backend.startInput)
	}
	status, err = service.Stop(context.Background())
	if err != nil || status.Phase != PhaseStopped || status.Readiness.Ready || backend.stopCalls != 1 {
		t.Fatalf("stop status=%+v err=%v calls=%d", status, err, backend.stopCalls)
	}
}

func TestServiceUnsafeStartStopsBackendAndFailsClosed(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	snapshot := runtimeSnapshot(now)
	snapshot.Safety.MotionOutputEnabled = true
	backend := &fakeBackend{snapshot: snapshot}
	service := newTestService(t, &now, backend)
	status, err := service.Start(context.Background())
	if !errors.Is(err, ErrUnsafeRuntime) ||
		status.Phase != PhaseFailed ||
		status.Readiness.Ready ||
		backend.stopCalls != 1 {
		t.Fatalf("unsafe start status=%+v err=%v stopCalls=%d", status, err, backend.stopCalls)
	}
}

func TestServiceRejectsUnboundedSafetyProofFreshness(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	snapshot := runtimeSnapshot(now)
	snapshot.Safety.FreshUntil = now.Add(2 * time.Second)
	backend := &fakeBackend{snapshot: snapshot}
	service := newTestService(t, &now, backend)
	status, err := service.Start(context.Background())
	if !errors.Is(err, ErrUnsafeRuntime) ||
		status.Phase != PhaseFailed ||
		backend.stopCalls != 1 {
		t.Fatalf("unbounded proof status=%+v err=%v stopCalls=%d", status, err, backend.stopCalls)
	}
}

func TestServiceSubmitsAndCorrelatesCanonicalPlan(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	backend := &fakeBackend{snapshot: runtimeSnapshot(now)}
	backend.planResult = func(submission planning.MotionServerSubmission) []byte {
		return successfulPlanReport(t, submission)
	}
	service := newTestService(t, &now, backend)
	if _, err := service.Start(context.Background()); err != nil {
		t.Fatal(err)
	}
	result, err := service.SubmitPlan(context.Background(), localPlanRequest(now))
	if err != nil {
		t.Fatalf("submit plan: %v result=%+v", err, result)
	}
	if !result.RuntimePlayable() ||
		result.Evidence.Identity.Target.ProfileID != "local-simulator" ||
		service.Status().LastPlan.PlanRevision == 0 {
		t.Fatalf("playable result/status missing: %+v %+v", result, service.Status())
	}
	if result.AcceptedPlan == nil ||
		result.AcceptedPlan.Schema != planning.RobotV4AcceptedPlanSchema ||
		result.AcceptedPlan.AuthorityClassification != planning.AcceptedPlanSimulationOnly ||
		result.AcceptedPlan.Evidence.RuntimePlayable ||
		len(result.AcceptedPlan.Trajectory) != len(result.Trajectory) ||
		result.AcceptedPlan.Trajectory[0].RowID != "row-1" {
		t.Fatalf("immutable simulation accepted plan missing or unsafe: %+v", result.AcceptedPlan)
	}
	if backend.submission.Transport != planning.MotionServerPlanTransport {
		t.Fatalf("submission = %+v", backend.submission)
	}
	var payload map[string]any
	if err := json.Unmarshal(backend.submission.Payload, &payload); err != nil {
		t.Fatal(err)
	}
	if _, exists := payload["planned_path"]; exists ||
		payload["execution_mode"] != "simulation" {
		t.Fatalf("unsafe planner payload = %#v", payload)
	}
}

func TestServiceRejectsUncorrelatedAndBlockedReports(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	tests := []struct {
		name string
		edit func(map[string]any)
		want error
	}{
		{
			name: "uncorrelated revision",
			edit: func(report map[string]any) {
				report["plan_request_revision"] = float64(999)
			},
			want: ErrUncorrelatedReport,
		},
		{
			name: "collision blocked",
			edit: func(report map[string]any) {
				report["accepted"] = false
				report["validation_state"] = "invalid"
				report["play_enabled"] = false
				report["runtime_preload_enabled"] = false
				report["collision_gate_clear"] = false
				report["collision_points"] = []map[string]any{{
					"row_index": 1, "waypoint_index": 1,
					"link_a": "link_1", "link_b": "fixture", "depth_m": 0.01,
					"affected_joints": []string{"J1"},
				}}
			},
			want: ErrPlanRejected,
		},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			backend := &fakeBackend{snapshot: runtimeSnapshot(now)}
			backend.planResult = func(submission planning.MotionServerSubmission) []byte {
				raw := successfulPlanReport(t, submission)
				var report map[string]any
				if err := json.Unmarshal(raw, &report); err != nil {
					t.Fatal(err)
				}
				test.edit(report)
				edited, _ := json.Marshal(report)
				return edited
			}
			service := newTestService(t, &now, backend)
			if _, err := service.Start(context.Background()); err != nil {
				t.Fatal(err)
			}
			result, err := service.SubmitPlan(context.Background(), localPlanRequest(now))
			if !errors.Is(err, test.want) || result.RuntimePlayable() {
				t.Fatalf("result=%+v err=%v want=%v", result, err, test.want)
			}
			if strings.TrimSpace(err.Error()) == "" {
				t.Fatal("rejection error did not retain a fail-closed reason")
			}
		})
	}
}

func TestServiceRechecksSafetyAfterPlanning(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	backend := &fakeBackend{snapshot: runtimeSnapshot(now)}
	backend.planResult = func(submission planning.MotionServerSubmission) []byte {
		backend.mu.Lock()
		backend.snapshot.Safety.EtherCATOutputEnabled = true
		backend.mu.Unlock()
		return successfulPlanReport(t, submission)
	}
	service := newTestService(t, &now, backend)
	if _, err := service.Start(context.Background()); err != nil {
		t.Fatal(err)
	}
	result, err := service.SubmitPlan(context.Background(), localPlanRequest(now))
	if !errors.Is(err, ErrUnsafeRuntime) ||
		result.RuntimePlayable() ||
		!containsReason(result.ReportReasons, "post_plan_runtime_proof_invalid") ||
		service.Status().Phase != PhaseFailed {
		t.Fatalf("unsafe postflight result=%+v err=%v status=%+v", result, err, service.Status())
	}
}

func TestServiceIsSingleFlightAndDoesNotQueuePlans(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	backend := &fakeBackend{
		snapshot:    runtimeSnapshot(now),
		planEntered: make(chan struct{}),
		planRelease: make(chan struct{}),
	}
	backend.planResult = func(submission planning.MotionServerSubmission) []byte {
		return successfulPlanReport(t, submission)
	}
	service := newTestService(t, &now, backend)
	if _, err := service.Start(context.Background()); err != nil {
		t.Fatal(err)
	}
	firstDone := make(chan error, 1)
	go func() {
		_, err := service.SubmitPlan(context.Background(), localPlanRequest(now))
		firstDone <- err
	}()
	<-backend.planEntered
	if _, err := service.SubmitPlan(context.Background(), localPlanRequest(now)); !errors.Is(err, ErrBusy) {
		t.Fatalf("second plan error = %v, want busy", err)
	}
	close(backend.planRelease)
	if err := <-firstDone; err != nil {
		t.Fatalf("first plan failed: %v", err)
	}
	if backend.planCalls != 1 {
		t.Fatalf("backend plan calls = %d, want 1", backend.planCalls)
	}
}

func TestServiceRejectsWrongTargetBeforeBackend(t *testing.T) {
	now := time.Date(2026, 7, 30, 14, 30, 0, 0, time.UTC)
	backend := &fakeBackend{snapshot: runtimeSnapshot(now)}
	service := newTestService(t, &now, backend)
	if _, err := service.Start(context.Background()); err != nil {
		t.Fatal(err)
	}
	request := localPlanRequest(now)
	request.Target.ProfileID = "bench-relay"
	if _, err := service.SubmitPlan(context.Background(), request); !errors.Is(err, ErrInvalidPlan) {
		t.Fatalf("wrong target error = %v", err)
	}
	if backend.planCalls != 0 {
		t.Fatalf("wrong-target request reached backend %d times", backend.planCalls)
	}
}
