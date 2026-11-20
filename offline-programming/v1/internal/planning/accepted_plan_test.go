package planning

import (
	"encoding/json"
	"os"
	"strings"
	"testing"
	"time"
)

func TestMintAcceptedPlanBindsDigestsRowsAndSimulationAuthority(t *testing.T) {
	now := time.Date(2026, 7, 30, 12, 0, 0, 0, time.UTC)
	context := strictContext(now)
	for index := range context.Request.Rows {
		context.Request.Rows[index].MoveType = "line_tcp"
		context.Request.Rows[index].MotionPhase = "travel"
		context.Request.Rows[index].ToolFrame = "torch_tcp"
		context.Request.Rows[index].WorkFrame = "world"
		context.Request.Rows[index].TCPVelocityMS = 0.05
		context.Request.Rows[index].JointVelocityRadS = 0.25
	}
	context.Request = withRobotV4Program(t, context.Request)
	result, err := NormalizeMotionServerPlanResult(
		context,
		strictPlanReport(t, context.Request, context.PlanRevision),
		100,
	)
	if err != nil {
		t.Fatal(err)
	}
	accepted, err := MintRobotV4AcceptedPlan(
		context.Request,
		result,
		AcceptedPlanAssets{
			PlannerBuildDigest:   "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
			TrajectoryMethod:     "linear-tcp-dense-csp",
			ManifestRevision:     "local-manifest-1",
			CellConfigRevision:   "local-cell-1",
			CalibrationDigest:    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
			URDFDigest:           "sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc",
			MeshDigest:           "sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd",
			CollisionWorldDigest: "sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee",
			SettingsRevision:     1,
		},
		AcceptedPlanSimulationOnly,
		now,
	)
	if err != nil {
		t.Fatal(err)
	}
	if accepted.Schema != RobotV4AcceptedPlanSchema ||
		accepted.AuthorityClassification != AcceptedPlanSimulationOnly ||
		accepted.Evidence.RuntimePlayable ||
		accepted.Trajectory[0].RowID != context.Request.Rows[0].RowID ||
		!sha256DigestPattern.MatchString(accepted.ProgramDigest) ||
		!sha256DigestPattern.MatchString(accepted.TrajectoryDigest) {
		t.Fatalf("accepted plan is not fully identity-bound: %+v", accepted)
	}
	wire, err := json.Marshal(accepted)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(wire), `"collision_points":[]`) ||
		!strings.Contains(string(wire), `"singularity_points":[]`) {
		t.Fatalf("accepted plan did not preserve empty evidence arrays: %s", wire)
	}
}

func TestMintAcceptedPlanRejectsUnvalidatedOrInventedAssets(t *testing.T) {
	now := time.Now().UTC()
	request := validPlanRequest(now)
	for index := range request.Rows {
		request.Rows[index].MoveType = "line_tcp"
		request.Rows[index].MotionPhase = "travel"
		request.Rows[index].ToolFrame = "torch_tcp"
		request.Rows[index].WorkFrame = "world"
	}
	request = withRobotV4Program(t, request)
	if _, err := MintRobotV4AcceptedPlan(
		request,
		NormalizedPlanResult{},
		AcceptedPlanAssets{},
		AcceptedPlanSimulationOnly,
		now,
	); err == nil {
		t.Fatal("unvalidated result became an accepted plan")
	}
}

func TestTargetIssuedAcceptedPlanArtifactWhenProvided(t *testing.T) {
	path := os.Getenv("ROBOT_V4_ACCEPTED_PLAN_CONTRACT_FIXTURE")
	if path == "" {
		t.Skip("external Robot V4 accepted-plan artifact not provided")
	}
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var plan RobotV4AcceptedPlan
	if err := json.Unmarshal(raw, &plan); err != nil {
		t.Fatal(err)
	}
	if err := plan.Validate(); err != nil {
		t.Fatalf("target-issued accepted plan failed the Go contract: %v", err)
	}
}
