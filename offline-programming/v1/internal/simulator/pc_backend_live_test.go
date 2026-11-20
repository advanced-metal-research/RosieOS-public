package simulator

import (
	"bytes"
	"context"
	"encoding/json"
	"flag"
	"os"
	"path/filepath"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
)

var liveCanonicalPlanner = flag.Bool(
	"offline-live-planner",
	false,
	"run the real Robot V4 Motion Server/Tesseract local-planner acceptance test",
)

// TestPCNativeBackendLiveCanonicalPlanner is intentionally outside the fast
// default suite. It invokes the production binary and genuine Tesseract
// collision worker twice: once for readiness and once for a collision-clear
// shared Robot V4 program. Run it through:
//
//	./rosie.sh offline-programming v1 test --filter live
func TestPCNativeBackendLiveCanonicalPlanner(t *testing.T) {
	if !*liveCanonicalPlanner {
		t.Skip("run with -offline-live-planner")
	}
	repoRoot, err := filepath.Abs(filepath.Join("..", "..", "..", ".."))
	if err != nil {
		t.Fatal(err)
	}
	fixturePath := filepath.Join(
		repoRoot,
		"robot",
		"v4",
		"motion-server",
		"testdata",
		"spreadsheet-plan-clear-input.json",
	)
	program := liveSharedProgram(t, fixturePath)
	now := time.Now().UTC()
	request, err := planning.NewNormalizedPlanRequestFromRobotV4Program(
		planning.NormalizedPlanRequestSchema,
		"live-canonical-planner-acceptance",
		"live-canonical-planner-project",
		1,
		planning.TargetIdentity{ProfileID: "local-simulator"},
		now,
		program,
	)
	if err != nil {
		t.Fatal(err)
	}

	backend := NewPCNativeBackend(PCBackendConfig{
		Enabled:  true,
		RepoRoot: repoRoot,
		ProofTTL: 5 * time.Second,
	})
	if availability := backend.Availability(); !availability.Available {
		t.Fatalf("real canonical backend unavailable: %+v", availability)
	} else if !availability.PersistentProcess {
		t.Fatalf("real canonical backend is not persistent: %+v", availability)
	}
	service, err := NewService(Options{
		ServiceID: "offline-programming-live-acceptance",
		Topology: NewPCTopology(
			"local-simulator",
			planning.RobotModelRosie1400V3,
			"127.0.0.1",
		),
		Limits:   DefaultResourceLimits(),
		ProofTTL: 5 * time.Second,
	}, backend)
	if err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 9*time.Minute)
	defer cancel()
	t.Cleanup(func() {
		stopContext, stopCancel := context.WithTimeout(
			context.Background(),
			30*time.Second,
		)
		defer stopCancel()
		_, _ = service.Stop(stopContext)
	})

	status, err := service.Start(ctx)
	if err != nil {
		t.Fatalf("real canonical readiness probe failed: %v", err)
	}
	if !status.Readiness.Ready ||
		!status.Safety.Simulation ||
		!status.Safety.NoRealMovement ||
		!status.Safety.LoopbackOnly ||
		status.Safety.EtherCATOutputEnabled ||
		status.Safety.MotionOutputEnabled ||
		status.Safety.HardwareAccess ||
		status.Safety.PhysicalServicesStarted ||
		status.Safety.CommandSubmissionEnabled {
		t.Fatalf("real canonical readiness proof is unsafe: %+v", status)
	}

	result, err := service.SubmitPlan(ctx, request)
	if err != nil {
		t.Fatalf("real collision-clear canonical plan failed: %v; result=%+v", err, result)
	}
	if !result.RuntimePlayable() ||
		result.Evidence.PlannerOwner != planning.MotionServerPlannerOwner ||
		result.Evidence.PlannerBackend != planning.MotionServerPlannerBackend ||
		result.Evidence.Collision.State != planning.CheckClear ||
		result.Evidence.Collision.CheckState != "tesseract_discrete_contact_checked" ||
		result.Evidence.Singularity.State != planning.CheckClear ||
		result.Evidence.Singularity.CheckState != "jacobian_svd_checked" ||
		result.Evidence.Limits.State != planning.CheckClear ||
		result.Evidence.Runtime.State != planning.CheckClear ||
		!result.Evidence.Runtime.Playable {
		t.Fatalf("real canonical evidence did not validate: %+v", result.Evidence)
	}
	if result.AcceptedPlan == nil ||
		result.AcceptedPlan.AuthorityClassification != planning.AcceptedPlanSimulationOnly ||
		result.AcceptedPlan.Planner.Owner != planning.MotionServerPlannerOwner ||
		result.AcceptedPlan.Planner.Backend != planning.MotionServerPlannerBackend ||
		len(result.AcceptedPlan.Trajectory) != result.Evidence.SampleCount {
		t.Fatalf("real simulation accepted plan is incomplete: %+v", result.AcceptedPlan)
	}
	if err := result.AcceptedPlan.Validate(); err != nil {
		t.Fatalf("real simulation accepted plan failed its production contract: %v", err)
	}
	repeated, err := service.SubmitPlan(ctx, request)
	if err != nil {
		t.Fatalf("repeated warm canonical plan failed: %v", err)
	}
	if repeated.Evidence.Timing.CollisionCacheHitWaypointCount !=
		repeated.Evidence.SampleCount {
		t.Fatalf(
			"warm plan reused %d/%d exact collision samples",
			repeated.Evidence.Timing.CollisionCacheHitWaypointCount,
			repeated.Evidence.SampleCount,
		)
	}
	if repeated.Evidence.Timing.CollisionCheckTimeS > 0.1 {
		t.Fatalf(
			"warm cached collision check took %.6fs, want <= 0.1s",
			repeated.Evidence.Timing.CollisionCheckTimeS,
		)
	}
	wire, err := json.Marshal(result)
	if err != nil {
		t.Fatal(err)
	}
	for _, forbidden := range [][]byte{
		[]byte(`"reasons":null`),
		[]byte(`"affected_joints":null`),
		[]byte(`"collision_points":null`),
		[]byte(`"singularity_points":null`),
	} {
		if bytes.Contains(wire, forbidden) {
			t.Fatalf("real browser wire response contains a null array: %s", forbidden)
		}
	}
}

func liveSharedProgram(t *testing.T, path string) json.RawMessage {
	t.Helper()
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var program map[string]any
	if err := json.Unmarshal(raw, &program); err != nil {
		t.Fatal(err)
	}
	program["base_program_revision"] = float64(0)
	revision, ok := program["program_revision"].(float64)
	if !ok || revision < 1 {
		t.Fatalf("live fixture has invalid program_revision: %#v", program["program_revision"])
	}
	rows, ok := program["rows"].([]any)
	if !ok || len(rows) < 2 {
		t.Fatal("live fixture must contain at least two rows")
	}
	for index, entry := range rows {
		row, ok := entry.(map[string]any)
		if !ok {
			t.Fatalf("live fixture row %d is invalid", index)
		}
		row["revision"] = revision
		row["command"] = "MOVE"
		row["motion_phase"] = "travel"
		row["tool_frame"] = "end_effector"
		row["work_frame"] = "world"
		row["joint_velocity_rad_s"] = 0.25
		row["weld_enable"] = false
		row["arc_state"] = "off"
	}
	encoded, err := json.Marshal(program)
	if err != nil {
		t.Fatal(err)
	}
	return encoded
}
