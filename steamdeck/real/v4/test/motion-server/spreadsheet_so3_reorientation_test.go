package motion_server

import (
	"encoding/json"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

// SO(3) interpolation regression coverage for `line_tcp` segments that actually
// reorient the torch.
//
// Every other spreadsheet testdata program holds orientation constant across
// its `line_tcp` segments, so they all take the exact-equality fast path of
// so3_interpolate_rpy and can never exercise the slerp branch end-to-end. The
// fixture this test drives (testdata/spreadsheet-plan-reorient-input.json)
// reorients 96.7 deg on one segment and 112.9 deg on the next, ending on the
// measured Dog Box pair-24 inter-seam frame whose authored yaw is unwrapped
// past 2*pi exactly as the offline-programming UI emits it.
//
// Every bound below is red against the componentwise Euler lerp this branch
// replaced; the executed base-vs-branch differential is recorded in
// motion-server/v1/tests/SO3_INTERP_REPORT.md (section "C2").
const (
	// Adjacent-sample joint step. Branch 0.373682; Euler lerp 1.590825.
	so3ReorientMaxJointStepRad = 0.60
	// Worst per-sample IK tracking residual in orientation, matrix-based and
	// therefore representation-invariant. Branch 0.000315232; Euler lerp
	// 0.950255106 (the IK cannot track the off-geodesic targets at all).
	so3ReorientMaxFKOrientationRad = 0.01
	// Interior samples must lie ON the SO(3) geodesic of their segment:
	// geo(a,s) + geo(s,b) - geo(a,b) <= tol. Branch 2.1e-08 (the acos
	// conditioning floor of the metric); Euler lerp 3.268357633.
	so3ReorientGeodesicToleranceRad = 1.0e-5
	// Guard against the fixture silently losing its reorientation, which is
	// how the pre-existing testdata programs came to cover nothing here.
	so3ReorientMinSegmentRotationRad = 1.5
	// Uniform geodesic advance: 1.971033 rad over 8 samples = 0.246379 rad.
	// The Euler lerp bunches the same rotation into a 0.954289 rad step.
	so3ReorientMaxAdjacentRotationRad = 0.30
)

type so3ReorientSample struct {
	rowIndex int
	rpy      [3]float64
}

func so3ReorientationPlanReport(t *testing.T, repoRoot string) map[string]any {
	t.Helper()
	motionServerDirectory := filepath.Join(repoRoot, "motion-server", "v1")
	outputPath := filepath.Join(t.TempDir(), "reorient-plan.json")
	command := exec.Command(
		ensureMotionServerCartesianTestBinary(t, repoRoot),
		"--self-test", "spreadsheet-tesseract-plan",
		"--input", filepath.Join("testdata", "spreadsheet-plan-reorient-input.json"),
		"--output", outputPath,
	)
	command.Dir = motionServerDirectory
	if output, err := command.CombinedOutput(); err != nil {
		t.Fatalf("plan reorienting spreadsheet program: %v\n%s", err, output)
	}
	raw, err := os.ReadFile(outputPath)
	if err != nil {
		t.Fatalf("read reorienting plan report: %v", err)
	}
	var report map[string]any
	if err := json.Unmarshal(raw, &report); err != nil {
		t.Fatalf("decode reorienting plan report: %v", err)
	}
	return report
}

func so3ReorientTrajectory(t *testing.T, report map[string]any) []so3ReorientSample {
	t.Helper()
	entries := anySlice(report["trajectory"])
	if len(entries) < 4 {
		t.Fatalf("reorienting plan has %d trajectory samples, want at least 4", len(entries))
	}
	samples := make([]so3ReorientSample, 0, len(entries))
	for index, entry := range entries {
		point, ok := entry.(map[string]any)
		if !ok {
			t.Fatalf("trajectory sample %d is not an object", index)
		}
		pose := anySlice(point["tcp_pose"])
		if len(pose) != 6 {
			t.Fatalf("trajectory sample %d has %d tcp_pose components, want 6", index, len(pose))
		}
		sample := so3ReorientSample{rowIndex: int(numericValue(point["row_index"]))}
		for axis := 0; axis < 3; axis++ {
			sample.rpy[axis] = numericValue(pose[3+axis])
		}
		samples = append(samples, sample)
	}
	return samples
}

// Geodesic (angular) distance between two URDF fixed-axis RPY triples. Invariant
// to the RPY spelling of either operand, so whole-turn-unwrapped components and
// principal-range components compare identically. Same formula as
// spreadsheet_pose_orientation_residual_rad in the planner.
func so3ReorientGeodesicRad(a, b [3]float64) float64 {
	left := motionServerRPYMatrixForTest(a)
	right := motionServerRPYMatrixForTest(b)
	var trace float64
	for row := 0; row < 3; row++ {
		for col := 0; col < 3; col++ {
			trace += left[row][col] * right[row][col]
		}
	}
	return math.Acos(math.Max(-1, math.Min(1, (trace-1)/2)))
}

// The planner keeps the source row index while interpolating a segment and
// changes to the destination row on the segment's final sample, so the first
// sample carrying row k is the arrival at row k.
func so3ReorientSegmentBounds(samples []so3ReorientSample) [][2]int {
	arrival := map[int]int{}
	order := []int{}
	for index, sample := range samples {
		if _, seen := arrival[sample.rowIndex]; !seen {
			arrival[sample.rowIndex] = index
			order = append(order, sample.rowIndex)
		}
	}
	bounds := [][2]int{}
	for position := 0; position+1 < len(order); position++ {
		bounds = append(bounds, [2]int{arrival[order[position]], arrival[order[position+1]]})
	}
	return bounds
}

func TestRobotV4MotionServerSO3ReorientationPlan(t *testing.T) {
	repoRoot := findRepoRoot(t)
	report := so3ReorientationPlanReport(t, repoRoot)

	if status := stringValue(report["planner_status"]); status != "planned" {
		t.Fatalf("reorienting program planner_status=%q, want planned", status)
	}
	if unreachable := numericValue(report["unreachable_point_count"]); unreachable != 0 {
		t.Errorf("reorienting plan has %.0f unreachable points, want 0", unreachable)
	}
	if step := numericValue(report["max_joint_step_rad"]); step > so3ReorientMaxJointStepRad {
		t.Errorf("max_joint_step_rad=%.6f, want <= %.6f",
			step, so3ReorientMaxJointStepRad)
	}
	if residual := numericValue(report["max_fk_orientation_residual_rad"]); residual > so3ReorientMaxFKOrientationRad {
		t.Errorf("max_fk_orientation_residual_rad=%.9f, want <= %.6f",
			residual, so3ReorientMaxFKOrientationRad)
	}

	samples := so3ReorientTrajectory(t, report)
	bounds := so3ReorientSegmentBounds(samples)
	if len(bounds) < 2 {
		t.Fatalf("reorienting plan resolved %d segments, want at least 2", len(bounds))
	}

	widestSegmentRotationRad := 0.0
	worstOffGeodesicRad := 0.0
	worstAdjacentRotationRad := 0.0
	worstRPYComponentJumpRad := 0.0
	constantOrientationSegments := 0
	for _, segment := range bounds {
		from := samples[segment[0]].rpy
		to := samples[segment[1]].rpy
		total := so3ReorientGeodesicRad(from, to)
		if total > widestSegmentRotationRad {
			widestSegmentRotationRad = total
		}
		constantOrientation := from == to
		if constantOrientation {
			constantOrientationSegments++
		}
		for index := segment[0] + 1; index < segment[1]; index++ {
			interior := samples[index].rpy
			// A pure-translation segment must keep the authored torch frame
			// BIT-identically: the planner's exact-equality fast path, checked
			// here through the full plan serialization.
			if constantOrientation && interior != from {
				t.Errorf("sample %d of constant-orientation segment %v has orientation %v, want %v",
					index, segment, interior, from)
				continue
			}
			excess := so3ReorientGeodesicRad(from, interior) +
				so3ReorientGeodesicRad(interior, to) - total
			if excess > worstOffGeodesicRad {
				worstOffGeodesicRad = excess
			}
		}
	}
	for index := 1; index < len(samples); index++ {
		advance := so3ReorientGeodesicRad(samples[index-1].rpy, samples[index].rpy)
		if advance > worstAdjacentRotationRad {
			worstAdjacentRotationRad = advance
		}
		for axis := 0; axis < 3; axis++ {
			jump := math.Abs(samples[index].rpy[axis] - samples[index-1].rpy[axis])
			if jump > worstRPYComponentJumpRad {
				worstRPYComponentJumpRad = jump
			}
		}
	}

	if widestSegmentRotationRad < so3ReorientMinSegmentRotationRad {
		t.Errorf("widest segment reorientation %.6f rad, want >= %.6f rad -- the fixture "+
			"no longer exercises the slerp branch",
			widestSegmentRotationRad, so3ReorientMinSegmentRotationRad)
	}
	if constantOrientationSegments == 0 {
		t.Errorf("fixture has no constant-orientation segment, so the exact-equality " +
			"fast path is no longer covered")
	}
	if worstOffGeodesicRad > so3ReorientGeodesicToleranceRad {
		t.Errorf("worst interior sample off-geodesic excess %.9f rad, want <= %.9f rad",
			worstOffGeodesicRad, so3ReorientGeodesicToleranceRad)
	}
	if worstAdjacentRotationRad > so3ReorientMaxAdjacentRotationRad {
		t.Errorf("worst adjacent-sample rotation advance %.6f rad, want <= %.6f rad",
			worstAdjacentRotationRad, so3ReorientMaxAdjacentRotationRad)
	}

	// Recorded, not asserted: the emitted RPY *spelling* is deliberately not
	// continuous across samples. Interior samples are unwrapped toward the
	// segment's start frame while the arrival sample is the operator's authored
	// frame verbatim, so an adjacent pair can differ by a whole turn in a
	// component while the rotation itself advances by a fraction of a radian.
	// Any consumer that differences plan-sample RPY components to recover an
	// angular rate is therefore wrong by construction -- derive it from the
	// rotations, as robot_v4_steamdeck_plan_angular_direction.hpp does.
	t.Logf("reorienting plan: widest segment rotation %.6f rad, worst adjacent rotation "+
		"advance %.6f rad, worst adjacent RPY component jump %.6f rad, worst off-geodesic "+
		"excess %.9f rad",
		widestSegmentRotationRad, worstAdjacentRotationRad,
		worstRPYComponentJumpRad, worstOffGeodesicRad)
}
