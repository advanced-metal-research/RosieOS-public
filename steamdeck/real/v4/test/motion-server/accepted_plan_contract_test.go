package motion_server

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

func TestRobotV4MotionServerAcceptedPlanContractDryRun(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	inputPath := filepath.Join(
		repoRoot,
		"motion-server", "v1", "testdata",
		"spreadsheet-plan-program-ref-rosie-1420-input.json",
	)
	manifestPath := filepath.Join(
		repoRoot,
		"rt-core", "config", "cells", "bench-relay.json",
	)
	outputPath := filepath.Join(t.TempDir(), "accepted-plan.json")

	cmd := exec.Command(
		binary,
		"--self-test", "accepted-plan-contract",
		"--manifest", manifestPath,
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server accepted-plan self-test failed: %v\n%s", err, output)
	}

	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read accepted plan: %v", err)
	}
	var plan struct {
		Schema                  string `json:"schema"`
		AuthorityClassification string `json:"authority_classification"`
		PlanID                  string `json:"plan_id"`
		PlanRevision            uint64 `json:"plan_revision"`
		TrajectoryDigest        string `json:"trajectory_digest"`
		ProgramID               string `json:"program_id"`
		ProgramRevision         uint64 `json:"program_revision"`
		ProgramDigest           string `json:"program_digest"`
		RequestID               string `json:"request_id"`
		Planner                 struct {
			Owner            string `json:"owner"`
			Backend          string `json:"backend"`
			TrajectoryMethod string `json:"trajectory_method"`
			BuildDigest      string `json:"build_digest"`
			RequestSubject   string `json:"request_subject"`
		} `json:"planner"`
		Target struct {
			TargetID             string   `json:"target_id"`
			RobotModel           string   `json:"robot_model"`
			JointOrder           []string `json:"joint_order"`
			ManifestRevision     string   `json:"manifest_revision"`
			CellConfigRevision   string   `json:"cell_config_revision"`
			CalibrationDigest    string   `json:"calibration_digest"`
			URDFDigest           string   `json:"urdf_digest"`
			CollisionWorldDigest string   `json:"collision_world_digest"`
			SettingsRevision     uint64   `json:"settings_revision"`
		} `json:"target"`
		Evidence struct {
			PlanningContractAccepted bool   `json:"planning_contract_accepted"`
			Collision                string `json:"collision"`
			Limits                   string `json:"limits"`
			RuntimePlayable          bool   `json:"runtime_playable"`
		} `json:"evidence"`
		Trajectory []struct {
			RowID             string    `json:"row_id"`
			RowIndex          int       `json:"row_index"`
			TimeS             float64   `json:"time_s"`
			TCPPoseM          []float64 `json:"tcp_pose_m"`
			JointPositionsRad []float64 `json:"joint_positions_rad"`
		} `json:"trajectory"`
	}
	if err := json.Unmarshal(raw, &plan); err != nil {
		t.Fatalf("decode accepted plan: %v\n%s", err, raw)
	}
	for name, digest := range map[string]string{
		"trajectory":      plan.TrajectoryDigest,
		"program":         plan.ProgramDigest,
		"planner build":   plan.Planner.BuildDigest,
		"manifest":        plan.Target.ManifestRevision,
		"calibration":     plan.Target.CalibrationDigest,
		"URDF":            plan.Target.URDFDigest,
		"collision world": plan.Target.CollisionWorldDigest,
	} {
		if len(digest) != 71 || !strings.HasPrefix(digest, "sha256:") {
			t.Fatalf("%s digest is invalid: %q", name, digest)
		}
	}
	if plan.Schema != "robot.v4.accepted-plan.v1" ||
		plan.AuthorityClassification != "physical_target" ||
		!strings.HasPrefix(plan.PlanID, "rv4-bench-relay-") ||
		plan.PlanRevision != 1 ||
		plan.ProgramID != "motion-server-spreadsheet-clear" ||
		plan.ProgramRevision != 1 ||
		plan.RequestID != "accepted-plan-contract-1" ||
		plan.Planner.Owner != "motion-server" ||
		plan.Planner.Backend != "motion_server_tesseract_trajectory_v1" ||
		plan.Planner.TrajectoryMethod != "linear-tcp-dense-csp" ||
		plan.Planner.RequestSubject != "robot/v4/motion-server.bench-relay.plan" ||
		plan.Target.TargetID != "bench-relay" ||
		plan.Target.RobotModel != "rosie_1400_v3" ||
		strings.Join(plan.Target.JointOrder, ",") != "J1,J2,J3,J4,J5,J6,J7,J8,J9" ||
		plan.Target.CellConfigRevision != "bench-relay-cell-config-v1" ||
		plan.Target.SettingsRevision != 1 ||
		!plan.Evidence.PlanningContractAccepted ||
		plan.Evidence.Collision != "clear" ||
		plan.Evidence.Limits != "clear" ||
		!plan.Evidence.RuntimePlayable ||
		len(plan.Trajectory) < 2 {
		t.Fatalf("accepted plan identity or evidence mismatch: %s", raw)
	}
	priorTime := -1.0
	for index, sample := range plan.Trajectory {
		if sample.RowID == "" || sample.RowIndex < 0 ||
			sample.TimeS < priorTime ||
			len(sample.TCPPoseM) != 6 ||
			len(sample.JointPositionsRad) != 9 {
			t.Fatalf("accepted plan trajectory sample %d is invalid: %+v", index, sample)
		}
		priorTime = sample.TimeS
	}
}

func TestRobotV4MotionServerAcceptedPlanBindsRosie1420PairModel(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	inputPath := filepath.Join(
		repoRoot,
		"motion-server", "v1", "testdata",
		"spreadsheet-plan-program-ref-rosie-1420-input.json",
	)
	manifestPath := filepath.Join(
		repoRoot,
		"rt-core", "config", "cells", "bench-relay.json",
	)
	urdfPath := filepath.Join(
		repoRoot, "robot_description", "robots", "rosie_1420_v1", "robot.urdf")
	collisionPython := filepath.Join(
		os.Getenv("HOME"), ".rosie", "robot-v4-tesseract-v1", "bin", "python3")
	if _, err := os.Stat(collisionPython); err != nil {
		t.Skip("Rosie 1420 accepted-plan test requires the provisioned Tesseract Python worker")
	}
	collisionWorker := collisionPython + " " + filepath.Join(
		repoRoot, "tesseract", "v1", "python", "collision_check.py")
	outputPath := filepath.Join(t.TempDir(), "accepted-plan-rosie-1420.json")

	cmd := exec.Command(
		binary,
		"--self-test", "accepted-plan-contract",
		"--manifest", manifestPath,
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	cmd.Env = append(
		os.Environ(),
		"ROBOT_V4_TESSERACT_URDF_PATH="+urdfPath,
		"ROBOT_V4_TESSERACT_COLLISION_WORKER="+collisionWorker,
	)
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server Rosie 1420 accepted-plan self-test failed: %v\n%s",
			err, output)
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatal(err)
	}
	var plan struct {
		Schema string `json:"schema"`
		Target struct {
			RobotModel string `json:"robot_model"`
			URDFDigest string `json:"urdf_digest"`
		} `json:"target"`
	}
	if err := json.Unmarshal(raw, &plan); err != nil {
		t.Fatalf("decode Rosie 1420 accepted plan: %v\n%s", err, raw)
	}
	urdfRaw, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatal(err)
	}
	digest := sha256.Sum256(urdfRaw)
	wantDigest := "sha256:" + hex.EncodeToString(digest[:])
	if plan.Schema != "robot.v4.accepted-plan.v1" ||
		plan.Target.RobotModel != "rosie_1420_v1" ||
		plan.Target.URDFDigest != wantDigest {
		t.Fatalf("Rosie 1420 accepted plan is not pair-bound: %s", raw)
	}
}

func TestRobotV4MotionServerAcceptedPlanDerivesMissingProgramDigest(t *testing.T) {
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	inputPath := filepath.Join(
		repoRoot,
		"motion-server", "v1", "testdata",
		"spreadsheet-plan-program-ref-rosie-1420-input.json",
	)
	raw, err := os.ReadFile(inputPath)
	if err != nil {
		t.Fatal(err)
	}
	var program map[string]any
	if err := json.Unmarshal(raw, &program); err != nil {
		t.Fatal(err)
	}
	delete(program, "program_digest")
	tampered, err := json.Marshal(program)
	if err != nil {
		t.Fatal(err)
	}
	tamperedPath := filepath.Join(t.TempDir(), "missing-digest.json")
	if err := os.WriteFile(tamperedPath, tampered, 0o600); err != nil {
		t.Fatal(err)
	}
	outputPath := filepath.Join(t.TempDir(), "accepted-plan.json")
	cmd := exec.Command(
		binary,
		"--self-test", "accepted-plan-contract",
		"--manifest", filepath.Join(
			repoRoot, "rt-core", "config", "cells", "bench-relay.json",
		),
		"--input", tamperedPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	collisionPython := filepath.Join(
		os.Getenv("HOME"), ".rosie", "robot-v4-tesseract-v1", "bin", "python3")
	if _, err := os.Stat(collisionPython); err != nil {
		t.Skip("accepted-plan rejection test requires the provisioned Tesseract Python worker")
	}
	cmd.Env = append(
		os.Environ(),
		"ROBOT_V4_TESSERACT_URDF_PATH="+filepath.Join(
			repoRoot, "robot_description", "robots", "rosie_1420_v1", "robot.urdf"),
		"ROBOT_V4_TESSERACT_COLLISION_WORKER="+collisionPython+" "+
			filepath.Join(
				repoRoot, "tesseract", "v1", "python", "collision_check.py"),
	)
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("missing digest was not derived: err=%v output=%s", err, output)
	}
	acceptedRaw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatal(err)
	}
	var accepted struct {
		ProgramDigest string `json:"program_digest"`
	}
	if err := json.Unmarshal(acceptedRaw, &accepted); err != nil {
		t.Fatal(err)
	}
	if len(accepted.ProgramDigest) != 71 ||
		!strings.HasPrefix(accepted.ProgramDigest, "sha256:") {
		t.Fatalf("server-derived program digest is invalid: %s", acceptedRaw)
	}
}

func TestRobotV4MotionServerAcceptedPlanReportsExactRejectionPredicate(
	t *testing.T,
) {
	repoRoot := findRepoRoot(t)
	raw, err := os.ReadFile(filepath.Join(
		repoRoot,
		"motion-server", "v1", "src",
		"robot_v4_cartesian_accepted_plan.hpp",
	))
	if err != nil {
		t.Fatal(err)
	}
	source := string(raw)
	for _, required := range []string{
		`accepted.rejection_reason = "program_json_missing"`,
		`program_digest = robot_v4_sha256(program_json)`,
		`return reject("program_digest_invalid")`,
		`return reject("manifest_digest_invalid")`,
		`return reject("planner_pair_identity_invalid")`,
		`return reject("planner_robot_model_mismatch")`,
		`accepted_plan_rejection_reason`,
	} {
		if !strings.Contains(source, required) {
			t.Fatalf("accepted-plan rejection diagnostics missing %q", required)
		}
	}
}
