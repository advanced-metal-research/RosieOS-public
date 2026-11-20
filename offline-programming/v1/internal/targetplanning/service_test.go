package targetplanning

import (
	"context"
	"encoding/json"
	"errors"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/planning"
	"rosieos/offline-programming/v1/internal/targets"
)

type testTargets struct {
	target targets.ConnectedManifestTarget
}

func (source testTargets) PlanningTarget(
	profileID string,
) (targets.ConnectedManifestTarget, bool) {
	return source.target, profileID == source.target.Discovery.ProfileID
}

type testAdapter struct {
	now   time.Time
	calls int
}

type fixedProgramAuthority struct {
	program json.RawMessage
}

func (authority fixedProgramAuthority) SavedProgram(
	_ context.Context,
	_ string,
	_ uint64,
) (json.RawMessage, error) {
	return append(json.RawMessage(nil), authority.program...), nil
}

func (adapter *testAdapter) Available(targets.ConnectedManifestTarget) bool {
	return true
}

func (adapter *testAdapter) Plan(
	_ context.Context,
	target targets.ConnectedManifestTarget,
	request planning.NormalizedPlanRequest,
	_ MutationIdentity,
) (planning.RobotV4AcceptedPlan, error) {
	adapter.calls++
	result := planning.NormalizedPlanResult{
		Schema: planning.NormalizedPlanResultSchema,
		Evidence: planning.NormalizedPlanEvidence{
			Schema:          planning.NormalizedPlanEvidenceSchema,
			RequestID:       request.RequestID,
			GeneratedAt:     adapter.now,
			PlannerOwner:    planning.MotionServerPlannerOwner,
			PlannerBackend:  planning.MotionServerPlannerBackend,
			State:           planning.PlanServerValidated,
			Accepted:        true,
			ValidationState: "valid",
			SampleCount:     2,
			Identity: planning.PlanIdentity{
				ProjectID:       request.ProjectID,
				ProjectRevision: request.ProjectRevision,
				ProgramID:       request.ProgramID,
				ProgramRevision: request.ProgramRevision,
				PlanID:          "target-plan-1",
				PlanRevision:    9,
				ProgramHash:     "target-report-hash",
				RobotModel:      request.RobotModel,
				Target:          request.Target,
			},
			Collision:   planning.CollisionEvidence{State: planning.CheckClear},
			Singularity: planning.SingularityEvidence{State: planning.CheckClear},
			Limits:      planning.LimitEvidence{State: planning.CheckClear},
			Runtime: planning.RuntimeEvidence{
				State:    planning.CheckClear,
				Playable: true,
			},
		},
		Trajectory: []planning.TrajectorySample{
			{
				RowIndex:      0,
				WaypointIndex: 0,
				TimeS:         0,
				MoveType:      "joint_ptp",
				MotionPhase:   "start",
			},
			{
				RowIndex:      1,
				WaypointIndex: 1,
				TimeS:         1,
				MoveType:      "joint_ptp",
				MotionPhase:   "finish",
			},
		},
	}
	return planning.MintRobotV4AcceptedPlan(
		request,
		result,
		planning.AcceptedPlanAssets{
			PlannerBuildDigest:   digest("1"),
			TrajectoryMethod:     "tesseract_dense_trajectory",
			ManifestRevision:     target.Binding.ManifestRevision,
			CellConfigRevision:   "cell-config-1",
			CalibrationDigest:    digest("2"),
			URDFDigest:           digest("3"),
			MeshDigest:           digest("4"),
			CollisionWorldDigest: digest("5"),
			SettingsRevision:     1,
		},
		planning.AcceptedPlanPhysicalTarget,
		adapter.now,
	)
}

type fixedClock struct{ now time.Time }

func (clock fixedClock) Now() time.Time { return clock.now }

func TestServiceStoresOnlyExactTargetIssuedPhysicalPlan(t *testing.T) {
	now := time.Date(2026, 7, 30, 12, 0, 0, 0, time.UTC)
	target := targetFixture()
	adapter := &testAdapter{now: now}
	request := requestFixture(t, target)
	service := NewService(Options{
		Targets:  testTargets{target: target},
		Programs: fixedProgramAuthority{program: request.Program},
		Adapter:  adapter,
		Clock:    fixedClock{now: now},
	})
	mutation := MutationIdentity{
		CorrelationID:  "correlation-0001",
		IdempotencyKey: "target-plan-idempotency-0001",
	}

	plan, err := service.Submit(
		context.Background(),
		target.Discovery.ProfileID,
		request,
		mutation,
	)
	if err != nil {
		t.Fatalf("submit target plan: %v", err)
	}
	if plan.AuthorityClassification != planning.AcceptedPlanPhysicalTarget ||
		!plan.Evidence.RuntimePlayable ||
		adapter.calls != 1 {
		t.Fatalf("unexpected accepted plan: %#v", plan)
	}
	replay, err := service.Submit(
		context.Background(),
		target.Discovery.ProfileID,
		request,
		mutation,
	)
	if err != nil || replay.TrajectoryDigest != plan.TrajectoryDigest ||
		adapter.calls != 1 {
		t.Fatalf("idempotent replay failed: plan=%#v err=%v calls=%d", replay, err, adapter.calls)
	}
	stored, ok := service.Get(plan.PlanID)
	if !ok || stored.TrajectoryDigest != plan.TrajectoryDigest {
		t.Fatalf("stored accepted plan unavailable: %#v", stored)
	}
	storedWithRequest, ok := service.GetStored(plan.PlanID)
	if !ok ||
		storedWithRequest.Request.RequestID != request.RequestID ||
		storedWithRequest.Plan.TrajectoryDigest != plan.TrajectoryDigest {
		t.Fatalf("stored plan/request pair unavailable: %#v", storedWithRequest)
	}
	storedWithRequest.Request.Program[0] = 'x'
	storedWithRequest.Request.Rows[0].RowID = "mutated"
	storedWithRequest.Plan.Trajectory[0].TimeS = 99
	again, ok := service.GetStored(plan.PlanID)
	if !ok ||
		again.Request.Program[0] == 'x' ||
		again.Request.Rows[0].RowID == "mutated" ||
		again.Plan.Trajectory[0].TimeS == 99 {
		t.Fatalf("GetStored leaked mutable authority state: %#v", again)
	}
}

func TestServiceRejectsChangedInputAndTargetIdentity(t *testing.T) {
	now := time.Date(2026, 7, 30, 12, 0, 0, 0, time.UTC)
	target := targetFixture()
	adapter := &testAdapter{now: now}
	request := requestFixture(t, target)
	service := NewService(Options{
		Targets:  testTargets{target: target},
		Programs: fixedProgramAuthority{program: request.Program},
		Adapter:  adapter,
		Clock:    fixedClock{now: now},
	})
	mutation := MutationIdentity{
		CorrelationID:  "correlation-0002",
		IdempotencyKey: "target-plan-idempotency-0002",
	}
	if _, err := service.Submit(
		context.Background(),
		target.Discovery.ProfileID,
		request,
		mutation,
	); err != nil {
		t.Fatalf("first submit: %v", err)
	}
	request.RequestID = "request-changed"
	if _, err := service.Submit(
		context.Background(),
		target.Discovery.ProfileID,
		request,
		mutation,
	); !errors.Is(err, ErrIdempotencyReplay) {
		t.Fatalf("expected idempotency rejection, got %v", err)
	}

	request = requestFixture(t, target)
	request.Target.ManifestRef = "rt-core/config/cells/other.json"
	if _, err := service.Submit(
		context.Background(),
		target.Discovery.ProfileID,
		request,
		MutationIdentity{
			CorrelationID:  "correlation-0003",
			IdempotencyKey: "target-plan-idempotency-0003",
		},
	); !errors.Is(err, ErrIdentityMismatch) {
		t.Fatalf("expected target mismatch, got %v", err)
	}
}

func TestServiceRejectsProgramThatDiffersFromSavedRevision(t *testing.T) {
	now := time.Date(2026, 7, 30, 12, 0, 0, 0, time.UTC)
	target := targetFixture()
	request := requestFixture(t, target)
	saved := append(json.RawMessage(nil), request.Program...)
	var changed map[string]any
	if err := json.Unmarshal(saved, &changed); err != nil {
		t.Fatal(err)
	}
	changed["cell_config_revision"] = "different-cell-config"
	saved, err := json.Marshal(changed)
	if err != nil {
		t.Fatal(err)
	}
	service := NewService(Options{
		Targets:  testTargets{target: target},
		Programs: fixedProgramAuthority{program: saved},
		Adapter:  &testAdapter{now: now},
		Clock:    fixedClock{now: now},
	})
	if _, err := service.Submit(
		context.Background(),
		target.Discovery.ProfileID,
		request,
		MutationIdentity{
			CorrelationID:  "correlation-mismatch-0001",
			IdempotencyKey: "idempotency-mismatch-0001",
		},
	); !errors.Is(err, ErrProgramMismatch) {
		t.Fatalf("expected saved program mismatch, got %v", err)
	}
}

func requestFixture(
	t *testing.T,
	target targets.ConnectedManifestTarget,
) planning.NormalizedPlanRequest {
	t.Helper()
	raw := json.RawMessage(`{
	  "schema_version":1,
	  "program_id":"program-physical-1",
	  "program_revision":2,
	  "base_program_revision":1,
	  "program_digest":"` + digest("a") + `",
	  "robot_model":"rosie_1400_v3",
	  "robotic-cell-type":"rosie_1400_v3",
	  "cell_config_revision":"cell-config-1",
	  "execution_mode":"production",
	  "settings":{"revision":1},
	  "defaults":{"default_tool_frame":"tool0","default_work_frame":"world"},
	  "rows":[
	    {"row_id":"row-1","revision":1,"enabled":true,"row_type":"motion","move_type":"joint_ptp","motion_phase":"start","tool_frame":"tool0","work_frame":"world","target_tcp_m":[0,0,0,0,0,0],"target_joints_rad":[0,0,0,0,0,0,0,0,0],"tcp_velocity_m_s":0.1,"joint_velocity_rad_s":0.2},
	    {"row_id":"row-2","revision":1,"enabled":true,"row_type":"motion","move_type":"joint_ptp","motion_phase":"finish","tool_frame":"tool0","work_frame":"world","target_tcp_m":[0,0,0,0,0,0],"target_joints_rad":[0.1,0,0,0,0,0,0,0,0],"tcp_velocity_m_s":0.1,"joint_velocity_rad_s":0.2}
	  ]
	}`)
	request, err := planning.NewNormalizedPlanRequestFromRobotV4Program(
		planning.NormalizedPlanRequestSchema,
		"request-target-0001",
		"project-target-1",
		7,
		planning.TargetIdentity{
			ProfileID:   target.Discovery.ProfileID,
			ManifestRef: target.Discovery.ManifestRef,
		},
		time.Date(2026, 7, 30, 11, 59, 0, 0, time.UTC),
		raw,
	)
	if err != nil {
		t.Fatalf("request fixture: %v", err)
	}
	return request
}

func targetFixture() targets.ConnectedManifestTarget {
	axes := make([]targets.AxisDescriptor, planning.JointCount)
	for index := range axes {
		axes[index] = targets.AxisDescriptor{
			Index:                 index,
			Name:                  "J" + string(rune('1'+index)),
			URDFJoint:             "J" + string(rune('1'+index)),
			Kind:                  "arm",
			GearRatio:             1,
			Sign:                  1,
			CartesianParticipates: index < 6,
		}
	}
	return targets.ConnectedManifestTarget{
		Discovery: targets.ConnectedTargetDiscoveryDescriptor{
			Schema:      targets.ConnectedTargetDiscoverySchema,
			ProfileID:   "bench-relay",
			ManifestRef: "rt-core/config/cells/bench-relay.json",
			CellID:      "bench-relay",
			Target: targets.TargetDescriptor{
				Schema:       targets.TargetDescriptorSchema,
				ProfileID:    "bench-relay",
				ManifestRef:  "rt-core/config/cells/bench-relay.json",
				CellID:       "bench-relay",
				Kind:         targets.TargetRealCell,
				Axes:         axes,
				Capabilities: targets.TargetCapabilities{Planning: true},
			},
		},
		Binding: targets.ConnectedTargetServerBinding{
			ManifestRevision: digest("b"),
		},
	}
}

func digest(char string) string {
	value := ""
	for len(value) < 64 {
		value += char
	}
	return "sha256:" + value[:64]
}
