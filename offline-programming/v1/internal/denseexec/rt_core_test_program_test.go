//go:build linux

package denseexec

import (
	"context"
	"strings"
	"testing"

	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

// the 1430 cell's completion_tolerance, 0.034 deg, in radians: the start check's
// bound is the cell's own per-axis value, never a number of the test's.
const testProgramTolerance = 0.034 * 3.141592653589793 / 180

// testProgramCell is a fake cell that describes a valid robot and a compiled
// configuration, holds every axis still at start with valid feedback, and has
// the test-program store on. The returned robot block is the cell's identity.
func testProgramCell(t *testing.T, start [codec.AxisCount]float64) (*RTCoreController, *fakeRTCore, map[string]string) {
	t.Helper()
	r, f, _ := rtFake(t)
	describeRobot(t, f, robotDir(t), []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`))
	f.mu.Lock()
	f.description.ConfigurationSHA256 = strings.Repeat("9", 64)
	for i := range f.description.Axes {
		f.description.Axes[i].CompletionTolerance = testProgramTolerance
		f.status.Axes[i].LogicalPosition = start[i]
		f.status.Axes[i].LogicalTarget = start[i]
	}
	robot := map[string]string{
		"model_id":                            f.description.Robot.ModelID,
		"robot_description_sha256":            f.description.Robot.RobotDescriptionSHA256,
		"machine_planning_calibration_sha256": f.description.Robot.MachinePlanningCalibrationSHA256,
		"machine_planning_calibration_source": "target",
		"machine_configuration_sha256":        "sha256:" + f.description.ConfigurationSHA256,
	}
	f.mu.Unlock()
	r.source.cfg.TestPrograms = NewTestPrograms()
	return r, f, robot
}

// upload files the program and points the fake's preparation receipt at it.
func uploadTestProgram(t *testing.T, r *RTCoreController, f *fakeRTCore, blob []byte) *TestProgram {
	t.Helper()
	p, err := r.source.cfg.TestPrograms.Add(blob, "inertia-j4.rdt", nil, "", "tab", testProgramNow)
	if err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.program.Identity = control.Identity{PlanID: p.PlanID, ProgramID: p.ProgramID, ProgramDigest: p.ProgramDigest, TrajectoryDigest: p.TrajectoryDigest,
		ManifestRevision: p.ManifestRevision, PlanRevision: p.PlanRevision, SourceDigest: p.TrajectoryDigest, NormalisedDigest: strings.Repeat("e", 64)}
	f.mu.Unlock()
	return p
}

func acquired(f *fakeRTCore) bool {
	f.mu.Lock()
	defer f.mu.Unlock()
	for _, call := range f.calls {
		if call == "acquire" || call == "prepare_program" {
			return true
		}
	}
	return false
}

// A test program the cell accepts loads through the ordinary Load, is
// labelled as a test program in the status every client polls, and plays
// through the ordinary Play.
func TestRTCoreTestProgramLoadsLabelledAndPlays(t *testing.T) {
	start := [codec.AxisCount]float64{0.1, 0.35, -0.17, 0.2, 0.52, -0.3, 0, 0, 0}
	r, f, robot := testProgramCell(t, start)
	p := uploadTestProgram(t, r, f, testProgramBlob(t, 511, start, robot, nil))
	ctx := context.Background()
	checks, err := r.CheckTestProgram(ctx, p.TrajectoryDigest)
	if err != nil || len(checks) != 4 || !strings.Contains(checks[3], "worst: J1 off by 0.000 deg of 0.034 deg") {
		t.Fatalf("check: %q %v", checks, err)
	}
	if acquired(f) {
		t.Fatal("the upload check acquired authority or prepared bytes")
	}
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: p.Identity, Source: TestProgramSource})
	if err != nil {
		t.Fatalf("load: %v", err)
	}
	session := loaded.Status.Session
	if session == nil || session.Source != TestProgramSource || session.TestProgram == nil || session.TestProgram.FileName != "inertia-j4.rdt" ||
		session.TestProgram.FileSHA256 != p.FileSHA256 || session.TrajectoryDigest != p.TrajectoryDigest || session.State != "loaded" {
		t.Fatalf("session %+v", session)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatalf("play: %v", err)
	}
	f.mu.Lock()
	started := f.started
	f.mu.Unlock()
	if started.TrajectoryDigest != p.TrajectoryDigest {
		t.Fatalf("started %+v", started)
	}
	if status := r.Status(ctx); status.Session.Source != TestProgramSource || status.Session.State != "playing" {
		t.Fatalf("status session %+v", status.Session)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
}

// Every mismatch is refused, by Load and by the upload check alike, before
// any authority is acquired or byte prepared.
func TestRTCoreTestProgramRefusesMismatchBeforeAuthority(t *testing.T) {
	start := [codec.AxisCount]float64{0.1, 0.35, -0.17, 0.2, 0.52, -0.3, 0, 0, 0}
	for _, tc := range []struct {
		name, want string
		setup      func(r *RTCoreController, f *fakeRTCore, robot map[string]string) (uint32, map[string]string)
	}{
		{"robot model", ReasonRobotCellMismatch, func(_ *RTCoreController, _ *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			robot["model_id"] = "cell_b"
			return 511, robot
		}},
		{"calibration", ReasonRobotCellMismatch, func(_ *RTCoreController, _ *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			robot["machine_planning_calibration_sha256"] = "sha256:" + strings.Repeat("3", 64)
			return 511, robot
		}},
		{"machine configuration", "test_program_configuration_mismatch", func(_ *RTCoreController, _ *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			robot["machine_configuration_sha256"] = "sha256:" + strings.Repeat("8", 64)
			return 511, robot
		}},
		{"binding revision", "test_program_binding_mismatch", func(r *RTCoreController, _ *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			r.cfg.Binding.Revision = 8
			return 511, robot
		}},
		{"axes", "test_program_axes_mismatch", func(_ *RTCoreController, _ *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			return 0x3f, robot
		}},
		{"start off by 1.2 x tolerance", "test_program_start_mismatch", func(_ *RTCoreController, f *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			f.status.Axes[3].LogicalPosition += 1.2 * testProgramTolerance
			return 511, robot
		}},
		{"feedback invalid", "test_program_start_unknown", func(_ *RTCoreController, f *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			f.status.Axes[4].LogicalValid = false
			return 511, robot
		}},
		{"no completion tolerance", "test_program_start_unchecked", func(_ *RTCoreController, f *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			f.description.Axes[2].CompletionTolerance = 0
			return 511, robot
		}},
		{"machine binds no robot", ReasonRobotCellUnavailable, func(_ *RTCoreController, f *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			f.description.Robot = nil
			return 511, robot
		}},
		{"home lost", "home_required", func(_ *RTCoreController, f *fakeRTCore, robot map[string]string) (uint32, map[string]string) {
			f.status.Core.HomeValidMask = 511 &^ (1 << 3)
			return 511, robot
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, f, robot := testProgramCell(t, start)
			f.mu.Lock()
			mask, robot := tc.setup(r, f, robot)
			f.mu.Unlock()
			p := uploadTestProgram(t, r, f, testProgramBlob(t, mask, start, robot, nil))
			ctx := context.Background()
			if _, err := r.CheckTestProgram(ctx, p.TrajectoryDigest); errorCode(err) != tc.want {
				t.Fatalf("check: got %s (%v), want %s", errorCode(err), err, tc.want)
			}
			if _, err := r.Load(ctx, LoadRequest{Identity: p.Identity, Source: TestProgramSource}); errorCode(rtFailure(err)) != tc.want {
				t.Fatalf("load: got %s (%v), want %s", errorCode(rtFailure(err)), err, tc.want)
			}
			if acquired(f) {
				t.Fatal("a refused test program acquired authority or prepared bytes")
			}
		})
	}
}

// The start bound is the cell's tolerance, inclusive of the offset native
// settles itself: 0.9 x tolerance off loads, and the refusal names the joint
// and both angles in degrees.
func TestRTCoreTestProgramStartBoundIsTheCellTolerance(t *testing.T) {
	start := [codec.AxisCount]float64{0.1, 0.35, -0.17, 0.2, 0.52, -0.3, 0, 0, 0}
	r, f, robot := testProgramCell(t, start)
	f.mu.Lock()
	f.status.Axes[3].LogicalPosition = start[3] + 0.9*testProgramTolerance
	f.mu.Unlock()
	p := uploadTestProgram(t, r, f, testProgramBlob(t, 511, start, robot, nil))
	checks, err := r.CheckTestProgram(context.Background(), p.TrajectoryDigest)
	if err != nil || !strings.Contains(checks[3], "worst: J4 off by 0.031 deg of 0.034 deg") {
		t.Fatalf("0.9 x tolerance: %q %v", checks, err)
	}
	f.mu.Lock()
	f.status.Axes[3].LogicalPosition = start[3] + 0.5/180*3.141592653589793
	f.mu.Unlock()
	_, err = r.CheckTestProgram(context.Background(), p.TrajectoryDigest)
	if errorCode(err) != "test_program_start_mismatch" || !strings.Contains(err.Error(), "J4 is at 11.959 deg, the program starts at 11.459 deg (tolerance 0.034 deg)") {
		t.Fatalf("0.5 deg off: %v", err)
	}
}

// While any program runs, neither the upload check nor Load replaces it.
func TestRTCoreTestProgramRefusedWhileAProgramRuns(t *testing.T) {
	start := [codec.AxisCount]float64{}
	r, f, robot := testProgramCell(t, start)
	p := uploadTestProgram(t, r, f, testProgramBlob(t, 511, start, robot, nil))
	r.mu.Lock()
	r.session = &SessionInfo{ID: "ds-planner", Kind: SessionKindTrajectory, State: "playing", TrajectoryDigest: "sha256:" + strings.Repeat("4", 64)}
	r.mu.Unlock()
	if _, err := r.CheckTestProgram(context.Background(), p.TrajectoryDigest); errorCode(err) != "program_already_executing" {
		t.Fatalf("check: %v", err)
	}
	if _, err := r.Load(context.Background(), LoadRequest{Identity: p.Identity, Source: TestProgramSource}); errorCode(rtFailure(err)) != "program_already_executing" {
		t.Fatalf("load: %v", err)
	}
	if acquired(f) {
		t.Fatal("a refused test program acquired authority")
	}
}
