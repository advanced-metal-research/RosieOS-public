package motion_server

import (
	"bufio"

	"encoding/json"
	"fmt"
	"io"
	"math"
	"net"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"testing"
	"time"
)

type tableCalibrationFitReport struct {
	Ok                bool      `json:"ok"`
	Error             string    `json:"error"`
	TranslationM      []float64 `json:"translation_m"`
	RPYRad            []float64 `json:"rpy_rad"`
	RotationAngleDeg  float64   `json:"rotation_angle_deg"`
	MaxPointShiftM    float64   `json:"max_point_shift_m"`
	ResidualsM        []float64 `json:"residuals_m"`
	MaxResidualM      float64   `json:"max_residual_m"`
	RMSResidualM      float64   `json:"rms_residual_m"`
	ResidualWarning   bool      `json:"residual_warning"`
	ResidualWarnM     float64   `json:"residual_warn_threshold_m"`
	ExceedsSafeLimits bool      `json:"exceeds_safety_limits"`
}

type tableCalibrationReport struct {
	Name              string                    `json:"name"`
	SelfTest          string                    `json:"self_test"`
	Ok                bool                      `json:"ok"`
	State             string                    `json:"state"`
	Error             string                    `json:"error"`
	CalibrationKind   string                    `json:"calibration_kind"`
	Algorithm         string                    `json:"algorithm"`
	CalibratedJoint   string                    `json:"calibrated_joint"`
	PlannerFrame      string                    `json:"planner_frame"`
	WorldFrame        string                    `json:"world_frame"`
	NoDoubleApply     bool                      `json:"planner_dh_kinematics_untouched_no_double_apply"`
	PosturePreserving bool                      `json:"posture_preserving_no_motion_no_arming"`
	URDFPath          string                    `json:"urdf_path"`
	RecordPath        string                    `json:"record_path"`
	NominalOriginXYZ  string                    `json:"nominal_origin_xyz"`
	NominalOriginRPY  string                    `json:"nominal_origin_rpy"`
	AppliedOriginXYZ  string                    `json:"applied_origin_xyz"`
	AppliedOriginRPY  string                    `json:"applied_origin_rpy"`
	URDFWritten       bool                      `json:"urdf_written"`
	RecordWritten     bool                      `json:"record_written"`
	Reverted          bool                      `json:"reverted"`
	PlannerFit        tableCalibrationFitReport `json:"planner_fit"`
	WorldFit          tableCalibrationFitReport `json:"world_fit"`
	NominalWorldM     []float64                 `json:"nominal_world_m"`
	MeasuredWorldM    []float64                 `json:"measured_world_m"`
}

type tableCalibrationRequestPoint struct {
	NominalTCPM       []float64 `json:"nominal_tcp_m"`
	MeasuredTCPM      []float64 `json:"measured_tcp_m"`
	NominalWorldM     []float64 `json:"nominal_world_m,omitempty"`
	MeasuredWorldM    []float64 `json:"measured_world_m,omitempty"`
	MeasuredJointsRad []float64 `json:"measured_joints_rad,omitempty"`
}

type tableCalibrationRequest struct {
	Source             string                         `json:"source"`
	NominalWorldSource string                         `json:"nominal_world_source,omitempty"`
	Apply              *bool                          `json:"apply,omitempty"`
	Revert             bool                           `json:"revert,omitempty"`
	Points             []tableCalibrationRequestPoint `json:"points"`
}

func tableCalibrationNominalWorldCorners() [4][3]float64 {
	return [4][3]float64{
		{1.813, 0.0638, 0.7509},
		{2.013, 0.0638, 0.7509},
		{2.013, 1.3078, 0.7509},
		{1.813, 1.3078, 0.7509},
	}
}

const tableCalibrationTableCenterX = 1.913
const tableCalibrationTableCenterY = 0.6858

func tableCalibrationNominalPlannerCorners() [4][3]float64 {
	return [4][3]float64{
		{0.9, -0.3, 0.2},
		{0.9, 0.3, 0.2},
		{0.7, 0.3, 0.2},
		{0.7, -0.3, 0.2},
	}
}

func tableCalibrationSyntheticCorrection(point [3]float64, centerX, centerY float64) [3]float64 {
	const yawRad = 2.0 * math.Pi / 180.0
	sin, cos := math.Sin(yawRad), math.Cos(yawRad)
	return [3]float64{
		cos*(point[0]-centerX) - sin*(point[1]-centerY) + centerX + 0.005,
		sin*(point[0]-centerX) + cos*(point[1]-centerY) + centerY,
		point[2],
	}
}

func tableCalibrationSyntheticRequest() tableCalibrationRequest {
	world := tableCalibrationNominalWorldCorners()
	planner := tableCalibrationNominalPlannerCorners()
	request := tableCalibrationRequest{Source: "go_test_synthetic"}
	for i := 0; i < 4; i++ {
		measuredWorld := tableCalibrationSyntheticCorrection(world[i], tableCalibrationTableCenterX, tableCalibrationTableCenterY)
		measuredPlanner := tableCalibrationSyntheticCorrection(planner[i], 0.8, 0.0)
		request.Points = append(request.Points, tableCalibrationRequestPoint{
			NominalTCPM:    planner[i][:],
			MeasuredTCPM:   measuredPlanner[:],
			NominalWorldM:  world[i][:],
			MeasuredWorldM: measuredWorld[:],
		})
	}
	return request
}

func runTableCalibrationSelfTest(t *testing.T, selfTest string, request any, env []string) tableCalibrationReport {
	t.Helper()
	repoRoot := findRepoRoot(t)
	binary := ensureMotionServerCartesianTestBinary(t, repoRoot)
	inputPath := filepath.Join(t.TempDir(), "calibration-input.json")
	outputPath := filepath.Join(t.TempDir(), "calibration-report.json")
	raw, err := json.Marshal(request)
	if err != nil {
		t.Fatalf("encode calibration request: %v", err)
	}
	if err := os.WriteFile(inputPath, raw, 0o644); err != nil {
		t.Fatalf("write calibration request: %v", err)
	}
	cmd := exec.Command(binary,
		"--self-test", selfTest,
		"--input", inputPath,
		"--output", outputPath,
	)
	cmd.Dir = filepath.Join(repoRoot, "motion-server", "v1")
	cmd.Env = append(os.Environ(), env...)
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("motion-server %s self-test failed: %v\n%s", selfTest, err, string(output))
	}
	reportRaw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read %s report: %v", selfTest, err)
	}
	var report tableCalibrationReport
	if err := json.Unmarshal(reportRaw, &report); err != nil {
		t.Fatalf("decode %s report: %v\n%s", selfTest, err, string(reportRaw))
	}
	if report.Name != "robot-v4-cartesiand" || report.SelfTest != selfTest {
		t.Fatalf("%s report identity mismatch: %+v", selfTest, report)
	}
	return report
}

func stageTableCalibrationWorkspace(t *testing.T) (urdfPath, recordPath string, env []string) {
	t.Helper()
	repoRoot := findRepoRoot(t)
	original, err := os.ReadFile(filepath.Join(repoRoot, "robot_description", "robots", "rosie_1400_v3", "robot.urdf"))
	if err != nil {
		t.Fatalf("read source-of-truth URDF: %v", err)
	}
	workDir := t.TempDir()
	urdfPath = filepath.Join(workDir, "robot.urdf")
	recordPath = filepath.Join(workDir, "calibration.json")
	viewerPath := filepath.Join(workDir, "viewer-robot.urdf")
	if err := os.WriteFile(urdfPath, original, 0o644); err != nil {
		t.Fatalf("stage URDF copy: %v", err)
	}
	if err := os.WriteFile(viewerPath, original, 0o644); err != nil {
		t.Fatalf("stage viewer URDF copy: %v", err)
	}
	env = []string{
		"ROBOT_V4_TESSERACT_URDF_PATH=" + urdfPath,
		"ROBOT_V4_TABLE_CALIBRATION_RECORD_PATH=" + recordPath,
		"ROBOT_V4_TABLE_CALIBRATION_VIEWER_URDF_PATH=" + viewerPath,
	}
	return urdfPath, recordPath, env
}

func tableCalibrationViewerPath(urdfPath string) string {
	return filepath.Join(filepath.Dir(urdfPath), "viewer-robot.urdf")
}

func TestRobotV4MotionServerTableCalibrationFitRecoversSyntheticTransform(t *testing.T) {
	report := runTableCalibrationSelfTest(t, "table-calibration-fit", tableCalibrationSyntheticRequest(), nil)
	if !report.Ok || report.State != "fit_only" {
		t.Fatalf("synthetic fit did not pass: ok=%t state=%q error=%q", report.Ok, report.State, report.Error)
	}
	if report.Algorithm != "kabsch_horn_quaternion_least_squares_rigid_no_scale" {
		t.Fatalf("fit algorithm = %q, want the least-squares rigid (no scale) contract", report.Algorithm)
	}
	if report.CalibrationKind != "rigid_work_frame_correction_not_dh_joint_calibration" {
		t.Fatalf("calibration kind = %q, want the honest rigid work-frame statement", report.CalibrationKind)
	}
	fit := report.PlannerFit
	if math.Abs(fit.RotationAngleDeg-2.0) > 1e-9 {
		t.Fatalf("recovered rotation = %.12f deg, want exactly the synthetic 2 deg", fit.RotationAngleDeg)
	}
	if len(fit.RPYRad) != 3 || math.Abs(fit.RPYRad[0]) > 1e-12 || math.Abs(fit.RPYRad[1]) > 1e-12 ||
		math.Abs(fit.RPYRad[2]-2.0*math.Pi/180.0) > 1e-12 {
		t.Fatalf("recovered rpy = %v, want pure +2 deg yaw", fit.RPYRad)
	}
	if fit.MaxResidualM > 1e-12 || fit.ResidualWarning {
		t.Fatalf("synthetic (noise-free) fit has residuals: max=%.3e warning=%t", fit.MaxResidualM, fit.ResidualWarning)
	}
	if len(fit.ResidualsM) != 4 {
		t.Fatalf("per-point residuals = %v, want one per corner", fit.ResidualsM)
	}
	if fit.ExceedsSafeLimits {
		t.Fatalf("small synthetic correction flagged unsafe: %+v", fit)
	}
}

func TestRobotV4MotionServerTableCalibrationFitRejectsDegeneratePointSets(t *testing.T) {
	base := tableCalibrationNominalPlannerCorners()
	cases := []struct {
		name      string
		points    []tableCalibrationRequestPoint
		wantState string
	}{
		{
			name: "too_few_points",
			points: []tableCalibrationRequestPoint{
				{NominalTCPM: base[0][:], MeasuredTCPM: base[0][:]},
				{NominalTCPM: base[1][:], MeasuredTCPM: base[1][:]},
			},
			wantState: "rejected_degenerate_too_few_points",
		},
		{
			name: "duplicate_points",
			points: []tableCalibrationRequestPoint{
				{NominalTCPM: base[0][:], MeasuredTCPM: base[0][:]},
				{NominalTCPM: base[0][:], MeasuredTCPM: base[0][:]},
				{NominalTCPM: base[2][:], MeasuredTCPM: base[2][:]},
				{NominalTCPM: base[3][:], MeasuredTCPM: base[3][:]},
			},
			wantState: "rejected_degenerate_duplicate_points",
		},
		{
			name: "colinear_points",
			points: []tableCalibrationRequestPoint{
				{NominalTCPM: []float64{0.7, 0.0, 0.2}, MeasuredTCPM: []float64{0.7, 0.0, 0.2}},
				{NominalTCPM: []float64{0.8, 0.0, 0.2}, MeasuredTCPM: []float64{0.8, 0.0, 0.2}},
				{NominalTCPM: []float64{0.9, 0.0, 0.2}, MeasuredTCPM: []float64{0.9, 0.0, 0.2}},
				{NominalTCPM: []float64{1.0, 0.0, 0.2}, MeasuredTCPM: []float64{1.0, 0.0, 0.2}},
			},
			wantState: "rejected_degenerate_colinear_points",
		},
	}
	for _, testCase := range cases {
		t.Run(testCase.name, func(t *testing.T) {
			report := runTableCalibrationSelfTest(t, "table-calibration-fit",
				tableCalibrationRequest{Source: "go_test_degenerate", Points: testCase.points}, nil)
			if report.Ok {
				t.Fatalf("degenerate point set accepted: %+v", report)
			}
			if report.State != testCase.wantState {
				t.Fatalf("state = %q, want %q (error=%q)", report.State, testCase.wantState, report.Error)
			}
		})
	}
}

func TestRobotV4MotionServerTableCalibrationFitReportsResidualWarning(t *testing.T) {
	request := tableCalibrationSyntheticRequest()
	// Bend one measured corner 12 mm out of the rigid solution. The
	// least-squares fit distributes a single-point error across all four
	// residuals (roughly a quarter each), so 12 mm leaves ~3 mm everywhere —
	// solidly over the 2 mm warning threshold while the fit still solves.
	request.Points[2].MeasuredTCPM = append([]float64(nil), request.Points[2].MeasuredTCPM...)
	request.Points[2].MeasuredTCPM[2] += 0.012
	report := runTableCalibrationSelfTest(t, "table-calibration-fit", request, nil)
	if !report.Ok {
		t.Fatalf("noisy fit rejected outright: %+v", report)
	}
	if !report.PlannerFit.ResidualWarning {
		t.Fatalf("12 mm off-plane noise did not trip the residual warning: %+v", report.PlannerFit)
	}
	if report.PlannerFit.ResidualWarnM != 0.002 {
		t.Fatalf("residual warn threshold = %v, want 2 mm", report.PlannerFit.ResidualWarnM)
	}
	if report.PlannerFit.MaxResidualM <= 0.002 || report.PlannerFit.MaxResidualM >= 0.012 {
		t.Fatalf("max residual = %.6f m, want between the threshold and the injected noise", report.PlannerFit.MaxResidualM)
	}
}

func TestRobotV4MotionServerTableCalibrationFitRefusesWildCorrections(t *testing.T) {
	planner := tableCalibrationNominalPlannerCorners()
	request := tableCalibrationRequest{Source: "go_test_wild"}
	for i := 0; i < 4; i++ {
		measured := [3]float64{planner[i][0] + 0.100, planner[i][1], planner[i][2]}
		request.Points = append(request.Points, tableCalibrationRequestPoint{
			NominalTCPM:  planner[i][:],
			MeasuredTCPM: measured[:],
		})
	}
	report := runTableCalibrationSelfTest(t, "table-calibration-fit", request, nil)
	if report.State != "refused_exceeds_safety_limits" {
		t.Fatalf("100 mm correction state = %q, want refused_exceeds_safety_limits", report.State)
	}
	if !report.PlannerFit.ExceedsSafeLimits {
		t.Fatalf("100 mm correction not flagged over the 50 mm limit: %+v", report.PlannerFit)
	}
}

func TestRobotV4MotionServerTableCalibrationApplySyntheticEndToEnd(t *testing.T) {
	urdfPath, recordPath, env := stageTableCalibrationWorkspace(t)
	originalURDF, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read staged URDF: %v", err)
	}
	report := runTableCalibrationSelfTest(t, "table-calibration-apply", tableCalibrationSyntheticRequest(), env)
	if !report.Ok || report.State != "applied" || !report.URDFWritten || !report.RecordWritten {
		t.Fatalf("synthetic apply did not land: %+v", report)
	}
	if !report.NoDoubleApply || !report.PosturePreserving {
		t.Fatalf("apply report must state the no-double-apply and posture-preserving contracts: %+v", report)
	}
	if report.CalibratedJoint != "J9" {
		t.Fatalf("calibrated joint = %q, want the J9 world->work-frame mount", report.CalibratedJoint)
	}
	if math.Abs(report.WorldFit.RotationAngleDeg-2.0) > 1e-9 || report.WorldFit.MaxResidualM > 1e-12 {
		t.Fatalf("world fit did not recover the synthetic +2 deg yaw exactly: %+v", report.WorldFit)
	}

	patchedURDF, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read patched URDF: %v", err)
	}
	if string(patchedURDF) == string(originalURDF) {
		t.Fatalf("apply reported urdf_written but the URDF is unchanged")
	}
	// The UI twin loads the staged viewer copy, so the overlay must mirror
	// the calibrated origin there too.
	viewerURDF, err := os.ReadFile(tableCalibrationViewerPath(urdfPath))
	if err != nil {
		t.Fatalf("read viewer URDF: %v", err)
	}
	if string(viewerURDF) != string(patchedURDF) {
		t.Fatalf("viewer URDF was not mirrored with the calibrated origin")
	}
	newXYZ := parseTableCalibrationTriplet(t, report.AppliedOriginXYZ)
	newRPY := parseTableCalibrationTriplet(t, report.AppliedOriginRPY)
	oldXYZ := parseTableCalibrationTriplet(t, report.NominalOriginXYZ)
	oldRPY := parseTableCalibrationTriplet(t, report.NominalOriginRPY)
	if oldRPY[0] != 0 || oldRPY[1] != 0 || oldRPY[2] != 0 {
		t.Fatalf("factory J9 rpy = %v, want zeros (update the test math if the URDF changes)", oldRPY)
	}
	// The correction is an overlay on the J9 origin: pushing every nominal
	// world corner through (new origin) * (old origin)^-1 must land exactly
	// on the operator-measured corners — the replanned table-box program's
	// corners land on the edited points.
	sin, cos := math.Sin(newRPY[2]), math.Cos(newRPY[2])
	world := tableCalibrationNominalWorldCorners()
	for i, corner := range world {
		local := [3]float64{corner[0] - oldXYZ[0], corner[1] - oldXYZ[1], corner[2] - oldXYZ[2]}
		mapped := [3]float64{
			cos*local[0] - sin*local[1] + newXYZ[0],
			sin*local[0] + cos*local[1] + newXYZ[1],
			local[2] + newXYZ[2],
		}
		want := tableCalibrationSyntheticCorrection(corner, tableCalibrationTableCenterX, tableCalibrationTableCenterY)
		for axis := 0; axis < 3; axis++ {
			if math.Abs(mapped[axis]-want[axis]) > 1e-9 {
				t.Fatalf("patched-model corner %d axis %d = %.12f, want %.12f (patched J9 does not absorb the synthetic offset)",
					i, axis, mapped[axis], want[axis])
			}
		}
	}

	recordRaw, err := os.ReadFile(recordPath)
	if err != nil {
		t.Fatalf("read calibration record: %v", err)
	}
	var record struct {
		Schema          string `json:"schema"`
		CalibratedJoint string `json:"calibrated_joint"`
		Records         []struct {
			Role             string `json:"role"`
			Source           string `json:"source"`
			AppliedOriginXYZ string `json:"applied_origin_xyz"`
		} `json:"records"`
	}
	if err := json.Unmarshal(recordRaw, &record); err != nil {
		t.Fatalf("decode calibration record: %v\n%s", err, string(recordRaw))
	}
	if record.Schema != "rosie.table-calibration.v1" || record.CalibratedJoint != "J9" {
		t.Fatalf("calibration record header mismatch: %+v", record)
	}
	if len(record.Records) != 2 || record.Records[0].Role != "active" || record.Records[1].Role != "previous" {
		t.Fatalf("calibration record must retain active + one-step-revert previous: %+v", record.Records)
	}
	if record.Records[1].Source != "identity_uncalibrated" {
		t.Fatalf("first calibration's previous record = %q, want the identity (uncalibrated) baseline", record.Records[1].Source)
	}
}

func TestRobotV4MotionServerTableCalibrationIdentityApplyIsByteStable(t *testing.T) {
	urdfPath, _, env := stageTableCalibrationWorkspace(t)
	originalURDF, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read staged URDF: %v", err)
	}
	world := tableCalibrationNominalWorldCorners()
	planner := tableCalibrationNominalPlannerCorners()
	request := tableCalibrationRequest{Source: "go_test_identity"}
	for i := 0; i < 4; i++ {
		request.Points = append(request.Points, tableCalibrationRequestPoint{
			NominalTCPM:    planner[i][:],
			MeasuredTCPM:   planner[i][:],
			NominalWorldM:  world[i][:],
			MeasuredWorldM: world[i][:],
		})
	}
	report := runTableCalibrationSelfTest(t, "table-calibration-apply", request, env)
	if !report.Ok || report.State != "applied" {
		t.Fatalf("identity apply did not land: %+v", report)
	}
	patched, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read URDF after identity apply: %v", err)
	}
	if string(patched) != string(originalURDF) {
		t.Fatalf("identity calibration must leave the URDF byte-identical")
	}
}

func TestRobotV4MotionServerTableCalibrationRevertRestoresFactoryURDF(t *testing.T) {
	urdfPath, recordPath, env := stageTableCalibrationWorkspace(t)
	originalURDF, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read staged URDF: %v", err)
	}
	applyReport := runTableCalibrationSelfTest(t, "table-calibration-apply", tableCalibrationSyntheticRequest(), env)
	if !applyReport.Ok || applyReport.State != "applied" {
		t.Fatalf("staging apply did not land: %+v", applyReport)
	}
	revertReport := runTableCalibrationSelfTest(t, "table-calibration-apply",
		tableCalibrationRequest{Source: "go_test_revert", Revert: true}, env)
	if !revertReport.Ok || !revertReport.Reverted || revertReport.State != "reverted" {
		t.Fatalf("revert did not land: %+v", revertReport)
	}
	reverted, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read URDF after revert: %v", err)
	}
	if string(reverted) != string(originalURDF) {
		t.Fatalf("one-step revert must restore the factory URDF byte-for-byte")
	}
	viewerReverted, err := os.ReadFile(tableCalibrationViewerPath(urdfPath))
	if err != nil {
		t.Fatalf("read viewer URDF after revert: %v", err)
	}
	if string(viewerReverted) != string(originalURDF) {
		t.Fatalf("one-step revert must restore the viewer URDF too")
	}
	recordRaw, err := os.ReadFile(recordPath)
	if err != nil {
		t.Fatalf("read calibration record after revert: %v", err)
	}
	var record struct {
		Records []struct {
			Role   string `json:"role"`
			Source string `json:"source"`
		} `json:"records"`
	}
	if err := json.Unmarshal(recordRaw, &record); err != nil {
		t.Fatalf("decode calibration record after revert: %v", err)
	}
	if len(record.Records) != 2 {
		t.Fatalf("revert must keep both records for auditability: %+v", record.Records)
	}
	if record.Records[0].Source != "identity_uncalibrated" || record.Records[1].Source != "go_test_synthetic" {
		t.Fatalf("revert must promote the identity baseline and retain the undone calibration: %+v", record.Records)
	}
}

func TestRobotV4MotionServerTableCalibrationRefusesWithoutWorldEvidence(t *testing.T) {
	urdfPath, _, env := stageTableCalibrationWorkspace(t)
	originalURDF, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read staged URDF: %v", err)
	}
	// Planner-frame pairs only: without world corners or measured joints the
	// server must refuse to guess a URDF patch.
	request := tableCalibrationSyntheticRequest()
	for i := range request.Points {
		request.Points[i].NominalWorldM = nil
		request.Points[i].MeasuredWorldM = nil
	}
	report := runTableCalibrationSelfTest(t, "table-calibration-apply", request, env)
	if report.Ok || report.State != "refused_insufficient_world_evidence" {
		t.Fatalf("apply without world evidence = ok=%t state=%q, want a refusal", report.Ok, report.State)
	}
	if !report.PlannerFit.Ok {
		t.Fatalf("planner-frame fit should still be reported for the operator: %+v", report.PlannerFit)
	}
	unchanged, err := os.ReadFile(urdfPath)
	if err != nil {
		t.Fatalf("read URDF after refusal: %v", err)
	}
	if string(unchanged) != string(originalURDF) {
		t.Fatalf("refused calibration must not touch the URDF")
	}
}

func TestRobotV4MotionServerTableCalibrationMeasuredJointsBridgeUsesURDFFK(t *testing.T) {
	_, _, env := stageTableCalibrationWorkspace(t)
	// Send measured joints of all zeros for one corner: the world bridge must
	// produce the URDF chain's zero-pose torch TCP (a fixed, independently
	// verified constant), proving measured corners come from URDF FK of the
	// joints the arm actually had rather than any planner-frame guess.
	request := tableCalibrationSyntheticRequest()
	request.Points[0].MeasuredWorldM = nil
	request.Points[0].MeasuredJointsRad = []float64{0, 0, 0, 0, 0, 0, 0, 0, 0}
	falseValue := false
	request.Apply = &falseValue
	report := runTableCalibrationSelfTest(t, "table-calibration-apply", request, env)
	if len(report.MeasuredWorldM) < 3 {
		t.Fatalf("apply report missing world pairs: %+v", report)
	}
	wantZeroPoseTCP := [3]float64{1.598782, 0.685800, 0.964043}
	for axis := 0; axis < 3; axis++ {
		if math.Abs(report.MeasuredWorldM[axis]-wantZeroPoseTCP[axis]) > 1e-5 {
			t.Fatalf("URDF FK bridge zero-pose TCP axis %d = %.6f, want %.6f",
				axis, report.MeasuredWorldM[axis], wantZeroPoseTCP[axis])
		}
	}
}

func TestRobotV4MotionServerTableCalibrationDerivesTableBoxNominalCorners(t *testing.T) {
	urdfPath, _, env := stageTableCalibrationWorkspace(t)
	request := tableCalibrationSyntheticRequest()
	request.Source = "steamdeck_chord"
	request.NominalWorldSource = "table_box_corners"
	world := tableCalibrationNominalWorldCorners()
	for i := range request.Points {
		request.Points[i].NominalWorldM = nil // force the server-side derivation
		measured := tableCalibrationSyntheticCorrection(world[i], tableCalibrationTableCenterX, tableCalibrationTableCenterY)
		request.Points[i].MeasuredWorldM = measured[:]
	}
	report := runTableCalibrationSelfTest(t, "table-calibration-apply", request, env)
	if !report.Ok || report.State != "applied" {
		t.Fatalf("table_box_corners derivation did not apply: %+v", report)
	}
	if math.Abs(report.WorldFit.RotationAngleDeg-2.0) > 1e-9 || report.WorldFit.MaxResidualM > 1e-9 {
		t.Fatalf("derived-corner world fit inexact: %+v", report.WorldFit)
	}
	for i := 0; i < 4; i++ {
		for axis := 0; axis < 3; axis++ {
			if math.Abs(report.NominalWorldM[3*i+axis]-world[i][axis]) > 1e-9 {
				t.Fatalf("derived nominal corner %d axis %d = %.9f, want the table-box constant %.9f",
					i, axis, report.NominalWorldM[3*i+axis], world[i][axis])
			}
		}
	}
	if _, err := os.Stat(urdfPath); err != nil {
		t.Fatalf("stat patched URDF: %v", err)
	}
}

func TestRobotV4MotionServerTableCalibrationOperatorFlowAbsorbsSyntheticOffset(t *testing.T) {
	urdfPath, _, env := stageTableCalibrationWorkspace(t)
	// URDF-chain IK solutions (damped least squares on the rosie_1400_v3
	// J1..J6 + torch chain, position error < 1e-9 m) that put the torch tip
	// exactly on the four true (offset) corners; the branch seed is the
	// table-box program's corner-1 seed from steamdeck/real/v4/ui/server/tableBoxProgram.ts.
	measuredJoints := [4][9]float64{
		{-0.732690493, 1.575079953, -0.713040493, -0.808893661, 2.322286956, -0.469329375, 0, 0, 0},
		{-0.664329688, 1.549033279, -0.725692391, -0.715552840, 2.675902373, -0.471952395, 0, 0, 0},
		{0.221114034, 1.519685949, -0.739526841, -0.919769028, 2.665293209, -0.469684337, 0, 0, 0},
		{0.301167907, 1.528178598, -0.712357106, -1.075052706, 2.320511021, -0.466603295, 0, 0, 0},
	}
	request := tableCalibrationSyntheticRequest()
	request.Source = "steamdeck_chord"
	request.NominalWorldSource = "table_box_corners"
	for i := range request.Points {
		request.Points[i].NominalWorldM = nil  // daemon derives from its J9 model
		request.Points[i].MeasuredWorldM = nil // daemon bridges via URDF FK of the joints
		request.Points[i].MeasuredJointsRad = measuredJoints[i][:]
	}
	report := runTableCalibrationSelfTest(t, "table-calibration-apply", request, env)
	if !report.Ok || report.State != "applied" {
		t.Fatalf("operator-flow apply did not land: %+v", report)
	}
	// The IK solutions carry ~nm noise, so the fit is exact to well under a
	// micrometer: the synthetic +2 deg yaw / +5 mm X must be recovered and
	// flagged clean (residuals far below the 2 mm warning).
	if math.Abs(report.WorldFit.RotationAngleDeg-2.0) > 1e-5 {
		t.Fatalf("world fit angle = %.9f deg, want the synthetic 2 deg", report.WorldFit.RotationAngleDeg)
	}
	if report.WorldFit.MaxResidualM > 1e-6 || report.WorldFit.ResidualWarning {
		t.Fatalf("operator-flow fit residuals too large: %+v", report.WorldFit)
	}
	world := tableCalibrationNominalWorldCorners()
	for i, corner := range world {
		want := tableCalibrationSyntheticCorrection(corner, tableCalibrationTableCenterX, tableCalibrationTableCenterY)
		for axis := 0; axis < 3; axis++ {
			if math.Abs(report.MeasuredWorldM[3*i+axis]-want[axis]) > 1e-6 {
				t.Fatalf("URDF-FK bridged corner %d axis %d = %.9f, want the true corner %.9f",
					i, axis, report.MeasuredWorldM[3*i+axis], want[axis])
			}
		}
	}
	if _, err := os.Stat(urdfPath); err != nil {
		t.Fatalf("stat patched URDF: %v", err)
	}

	// Second pass against the PATCHED model: the regenerated table-box
	// corners (derived from the patched J9) and the same measured joints must
	// now agree — the correction was absorbed, so the fit reads identity.
	falseValue := false
	request.Apply = &falseValue
	second := runTableCalibrationSelfTest(t, "table-calibration-apply", request, env)
	if !second.Ok || second.State != "fit_only" {
		t.Fatalf("post-patch verification pass failed: %+v", second)
	}
	if second.WorldFit.RotationAngleDeg > 1e-5 || second.WorldFit.MaxPointShiftM > 1e-6 {
		t.Fatalf("patched model did not absorb the offset: second-pass fit angle=%.9f deg shift=%.9f m",
			second.WorldFit.RotationAngleDeg, second.WorldFit.MaxPointShiftM)
	}
}

type tableCalibrationFakeNATS struct {
	address            string
	commandSubject     string
	calibrationSubject string
	commandPayload     string
	calibrationReports chan string
	mu                 sync.Mutex
	lastReport         string
}

func (server *tableCalibrationFakeNATS) lastCalibrationReport() string {
	server.mu.Lock()
	defer server.mu.Unlock()
	return server.lastReport
}

func startTableCalibrationFakeNATS(t *testing.T, commandSubject, calibrationSubject, commandPayload string) *tableCalibrationFakeNATS {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("start fake NATS listener: %v", err)
	}
	server := &tableCalibrationFakeNATS{
		address:            listener.Addr().String(),
		commandSubject:     commandSubject,
		calibrationSubject: calibrationSubject,
		commandPayload:     commandPayload,
		calibrationReports: make(chan string, 8),
	}
	t.Cleanup(func() { _ = listener.Close() })
	go func() {
		for {
			conn, err := listener.Accept()
			if err != nil {
				return
			}
			go server.handleConnection(conn)
		}
	}()
	return server
}

func (server *tableCalibrationFakeNATS) handleConnection(conn net.Conn) {
	defer conn.Close()
	_ = conn.SetDeadline(time.Now().Add(15 * time.Second))
	_, _ = io.WriteString(conn, "INFO {\"server_id\":\"fake-table-calibration-nats\"}\r\n")
	reader := bufio.NewReader(conn)
	for {
		line, err := reader.ReadString('\n')
		if err != nil {
			return
		}
		line = strings.TrimRight(line, "\r\n")
		fields := strings.Fields(line)
		if len(fields) == 0 {
			continue
		}
		switch fields[0] {
		case "PING":
			_, _ = io.WriteString(conn, "PONG\r\n")
		case "SUB":
			if len(fields) >= 3 && fields[1] == server.commandSubject {
				message := fmt.Sprintf("MSG %s %s %d\r\n%s\r\n",
					server.commandSubject, fields[2], len(server.commandPayload), server.commandPayload)
				_, _ = io.WriteString(conn, message)
			}
		case "PUB":
			if len(fields) < 3 {
				return
			}
			size, err := strconv.Atoi(fields[len(fields)-1])
			if err != nil || size < 0 {
				return
			}
			payload := make([]byte, size+2)
			if _, err := io.ReadFull(reader, payload); err != nil {
				return
			}
			if fields[1] == server.calibrationSubject {
				report := string(payload[:size])
				server.mu.Lock()
				server.lastReport = report
				server.mu.Unlock()
				select {
				case server.calibrationReports <- report:
				default:
				}
			}
		}
	}
}

func parseTableCalibrationTriplet(t *testing.T, text string) [3]float64 {
	t.Helper()
	var out [3]float64
	if _, err := fmt.Sscanf(text, "%f %f %f", &out[0], &out[1], &out[2]); err != nil {
		t.Fatalf("parse origin triplet %q: %v", text, err)
	}
	return out
}
