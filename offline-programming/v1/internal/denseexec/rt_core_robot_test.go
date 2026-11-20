//go:build linux

package denseexec

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/base64"
	"errors"
	"fmt"
	"rosieos/offline-programming/v1/internal/densejointtraj"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/robotdesc"
	"rosieos/rt-core/sdk/control"
)

// The fake serves resources the way the SDK does: by digest, verified.
func (f *fakeRTCore) Resource(_ context.Context, sha string) (io.ReadCloser, error) {
	f.mu.Lock()
	defer f.mu.Unlock()
	data, ok := f.resources[sha]
	if !ok {
		return nil, &control.Rejected{Reason: "resource_unknown"}
	}
	f.resourceReads++
	return io.NopCloser(bytes.NewReader(data)), nil
}

// describeRobot files a description directory and a machine_planning_calibration.json into the
// fake's description the way rtctl compiles them
// (rt-core/tools/rtctl/compiled_resources.go): the manifest and every
// registered file by description-relative path, machine_planning_calibration.json beside them.
func describeRobot(t *testing.T, f *fakeRTCore, dir string, machinePlanningCalibration []byte) {
	t.Helper()
	model := filepath.Base(dir)
	files, err := robotdesc.DescriptionFiles(dir)
	if err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	f.resources = map[string][]byte{}
	identity, err := robotdesc.RobotDescriptionSHA256(dir)
	if err != nil {
		t.Fatal(err)
	}
	robot := &control.RobotDescription{ModelID: model, RobotDescriptionSHA256: identity}
	add := func(path string, data []byte, media string) {
		sha := fmt.Sprintf("%x", sha256.Sum256(data))
		f.resources[sha] = data
		robot.Resources = append(robot.Resources, control.ResourceInfo{Path: path, SHA256: sha, Bytes: uint64(len(data)), MediaType: media})
	}
	for _, rel := range append([]string{robotdesc.Manifest}, files...) {
		data, err := os.ReadFile(filepath.Join(dir, filepath.FromSlash(rel)))
		if err != nil {
			t.Fatal(err)
		}
		add(rel, data, "application/octet-stream")
	}
	if machinePlanningCalibration != nil {
		add("machine_planning_calibration.json", machinePlanningCalibration, "application/json")
		robot.MachinePlanningCalibrationSHA256 = robotdesc.MachinePlanningCalibrationSHA256(machinePlanningCalibration)
	}
	f.description.Robot = robot
}

// A description directory the test owns, manifest and all.
func robotDir(t *testing.T) string {
	t.Helper()
	dir := filepath.Join(t.TempDir(), "robot_description", "cell_a")
	if err := os.MkdirAll(filepath.Join(dir, "meshes"), 0o755); err != nil {
		t.Fatal(err)
	}
	for name, text := range map[string]string{
		"robot.urdf":      `<robot name="cell_a"><joint name="J1" type="revolute"><limit lower="-3" upper="3" effort="100" velocity="2"/></joint></robot>`,
		"config.json":     `{"schema":"rosie.robot-config.v1","model_id":"cell_a"}`,
		"meshes/base.dae": "<COLLADA/>",
		"notes.txt":       "not registered, not the robot",
		robotdesc.Manifest: `{"schema":"` + robotdesc.ManifestSchema + `","model_id":"cell_a",` +
			`"files":["config.json","meshes/base.dae","robot.urdf"]}`,
	} {
		if err := os.WriteFile(filepath.Join(dir, filepath.FromSlash(name)), []byte(text), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	return dir
}

// The machine's identity under the contract is read off what it describes:
// the robot hash equals the one the workstation computes over the same
// directory, the cell hash is the machine_planning_calibration.json's, and the bytes the planner is
// handed are the machine's own. A second ask costs no read.
func TestRTCoreCellIsReadOffTheDescription(t *testing.T) {
	r, f, _ := rtFake(t)
	dir := robotDir(t)
	machinePlanningCalibration := []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}` + "\n")
	describeRobot(t, f, dir, machinePlanningCalibration)
	identity, err := robotdesc.RobotDescriptionSHA256(dir)
	if err != nil {
		t.Fatal(err)
	}
	ctx := context.Background()
	cell, err := r.Cell(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if !cell.Robot.Configured || !cell.Robot.Valid || cell.Robot.ModelID != "cell_a" || cell.Robot.RobotDescriptionSHA256 != identity {
		t.Fatalf("robot cell %+v; want cell_a at %s", cell.Robot, identity)
	}
	// The machine serves the model's EMPTY document, which is what a machine
	// that pins no calibration of its own serves: valid, and not "present".
	if want := robotdesc.MachinePlanningCalibrationSHA256(machinePlanningCalibration); cell.Robot.MachinePlanningCalibrationSHA256 != want || cell.Robot.MachinePlanningCalibrationPresent {
		t.Fatalf("machine_planning_calibration_sha256 %s present=%v; want %s and not present", cell.Robot.MachinePlanningCalibrationSHA256, cell.Robot.MachinePlanningCalibrationPresent, want)
	}
	if got, _ := base64.StdEncoding.DecodeString(cell.MachinePlanningCalibrationBase64); !bytes.Equal(got, machinePlanningCalibration) {
		t.Fatalf("machine_planning_calibration.json bytes %q; want the machine's", got)
	}
	if _, err := r.Cell(ctx); err != nil {
		t.Fatal(err)
	}
	if f.resourceReads != 1 {
		t.Fatalf("%d resource reads for two asks; a content-addressed read is cached", f.resourceReads)
	}
	if status := r.Status(ctx); status.Robot == nil || status.Robot.RobotDescriptionSHA256 != identity {
		t.Fatalf("status carries %+v; want the robot cell", status.Robot)
	}

	served, err := r.Description(ctx)
	if err != nil {
		t.Fatal(err)
	}
	if served.Robot.RobotDescriptionSHA256 != identity {
		t.Fatalf("description served under %s; want %s", served.Robot.RobotDescriptionSHA256, identity)
	}
	for _, rel := range []string{robotdesc.Manifest, "config.json", "meshes/base.dae", "robot.urdf"} {
		want, _ := os.ReadFile(filepath.Join(dir, filepath.FromSlash(rel)))
		if !bytes.Equal(served.Files[rel], want) {
			t.Fatalf("%s served as %q", rel, served.Files[rel])
		}
	}
	if _, present := served.Files["notes.txt"]; present || len(served.Files) != 4 {
		t.Fatalf("served %d files including unregistered ones: %v", len(served.Files), served.Files)
	}
}

func TestRTCoreCellCarriesFreshNativePlanningLimits(t *testing.T) {
	r, f, _ := rtFake(t)
	describeRobot(t, f, robotDir(t), []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`))
	acceleration := 0.5
	f.mu.Lock()
	f.description.Axes[5].MinPosition = -2.094395102
	f.description.Axes[5].MaxPosition = 2.094395102
	f.description.Axes[5].MaxVelocity = 0.2
	f.description.Axes[5].MaxAcceleration = &acceleration
	f.mu.Unlock()
	cell, err := r.Cell(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	axis := cell.ExecutionLimits["J6"]
	if axis.PositionUnit != "rad" || axis.LowerRad != -2.094395102 || axis.UpperRad != 2.094395102 ||
		axis.VelocityRadS != 0.2 || axis.AccelerationRadS2 == nil || *axis.AccelerationRadS2 != 0.5 {
		t.Fatalf("native J6 limits were discarded: %+v", axis)
	}
	f.mu.Lock()
	f.description.Axes[5].MinPosition = -1.5
	*f.description.Axes[5].MaxAcceleration = 0.25
	f.mu.Unlock()
	latest, err := r.Cell(context.Background())
	if err != nil || latest.ExecutionLimits["J6"].LowerRad != -1.5 || cell.ExecutionLimits["J6"].LowerRad != -2.094395102 {
		t.Fatalf("cell reused a stale native limit snapshot: %+v %v", latest, err)
	}
	if *latest.ExecutionLimits["J6"].AccelerationRadS2 != 0.25 || *cell.ExecutionLimits["J6"].AccelerationRadS2 != 0.5 {
		t.Fatal("a later description mutated the previous native acceleration snapshot")
	}
}

// A machine that binds no robot description says so, in the contract's
// words, and carries the reason in its status; it judges no plan. Such a
// machine still describes its drives: that is what tells it apart from an
// adapter serving no compiled configuration at all (the test below).
func TestRTCoreCellRefusesAMachineThatBindsNoRobot(t *testing.T) {
	r, f, id := rtFake(t)
	f.mu.Lock()
	f.description.Backend = "simulation"
	f.mu.Unlock()
	ctx := context.Background()
	cell, err := r.Cell(ctx)
	if err != nil || cell.Robot.Valid || len(cell.ExecutionLimits) == 0 {
		t.Fatalf("flat simulator must expose native limits without claiming robot calibration: %+v, %v", cell, err)
	}
	f.mu.Lock()
	f.description.Backend = "ethercat"
	f.mu.Unlock()
	_, err = r.Cell(ctx)
	var typed *Error
	if !errors.As(err, &typed) || typed.Code != ReasonRobotCellUnavailable {
		t.Fatalf("want %s, got %v", ReasonRobotCellUnavailable, err)
	}
	if _, err := r.Description(ctx); !errors.As(err, &typed) || typed.Code != ReasonRobotCellUnavailable {
		t.Fatalf("want %s, got %v", ReasonRobotCellUnavailable, err)
	}
	if status := r.Status(ctx); status.Robot == nil || status.Robot.Configured || status.Robot.Error == "" {
		t.Fatalf("status carries %+v; want the reason the machine has no robot", status.Robot)
	}
	if _, err := r.Load(ctx, LoadRequest{Identity: id}); err != nil {
		t.Fatalf("a machine with no robot has nothing to judge a plan by: %v", err)
	}
}

// An adapter that describes neither a robot nor its drives was started
// without its compiled configuration. It looks exactly like a flat machine
// to a reader that only asks "is there a robot block", and a robot-bound
// cell launched that way would play every plan unjudged. It is refused as
// unavailable, before anything is acquired, with the flag named.
func TestRTCoreLoadRefusesAnAdapterWithoutItsCompiledConfiguration(t *testing.T) {
	r, f, id := rtFake(t)
	f.mu.Lock()
	f.description.Drives = nil
	f.mu.Unlock()
	ctx := context.Background()
	_, err := r.Load(ctx, LoadRequest{Identity: id})
	var typed *Error
	if !errors.As(err, &typed) || typed.Code != ReasonRobotCellUnavailable || !strings.Contains(typed.Detail, "--compiled-config") {
		t.Fatalf("want %s naming --compiled-config, got %v", ReasonRobotCellUnavailable, err)
	}
	f.mu.Lock()
	acquired := false
	for _, call := range f.calls {
		acquired = acquired || call == "acquire"
	}
	f.mu.Unlock()
	if acquired {
		t.Fatal("the plan was judged after the machine was acquired")
	}
	if status := r.Status(ctx); status.Robot == nil || !status.Robot.Configured || status.Robot.Valid || !strings.Contains(status.Robot.Error, "--compiled-config") {
		t.Fatalf("status carries %+v; want the missing compiled configuration named", status.Robot)
	}
}

// A machine that binds a robot judges the plan's robot block before
// anything is acquired: a plan that names no robot is refused as missing.
func TestRTCoreLoadJudgesThePlanRobot(t *testing.T) {
	r, f, id := rtFake(t)
	describeRobot(t, f, robotDir(t), []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`))
	_, err := r.Load(context.Background(), LoadRequest{Identity: id})
	var typed *Error
	if !errors.As(err, &typed) || typed.Code != ReasonRobotCellMissing {
		t.Fatalf("want %s, got %v", ReasonRobotCellMissing, err)
	}
	f.mu.Lock()
	acquired := false
	for _, call := range f.calls {
		acquired = acquired || call == "acquire"
	}
	f.mu.Unlock()
	if acquired {
		t.Fatal("the plan was judged after the machine was acquired")
	}
}

// A description served without its manifest, or without a machine_planning_calibration.json, is
// not a robot under the contract.
func TestRTCoreCellRefusesAHalfServedDescription(t *testing.T) {
	r, f, _ := rtFake(t)
	describeRobot(t, f, robotDir(t), nil)
	if _, err := r.Cell(context.Background()); err == nil || !contains(err.Error(), "no machine_planning_calibration.json") {
		t.Fatalf("want the missing machine_planning_calibration.json named, got %v", err)
	}
	describeRobot(t, f, robotDir(t), []byte(`{}`))
	f.mu.Lock()
	kept := f.description.Robot.Resources[:0]
	for _, resource := range f.description.Robot.Resources {
		if filepath.Base(resource.Path) != robotdesc.Manifest {
			kept = append(kept, resource)
		}
	}
	f.description.Robot.Resources = kept
	f.mu.Unlock()
	// The controller keeps the description it first read; a machine that
	// changes underneath it is a redeploy, which its identity pins catch.
	if _, err := r.describe(context.Background()); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Cell(context.Background()); err == nil || !contains(err.Error(), "robot_description_manifest.json") {
		t.Fatalf("want the missing manifest named, got %v", err)
	}
}

func contains(s, sub string) bool { return bytes.Contains([]byte(s), []byte(sub)) }

// The machine's whole configuration is recorded with its identity, and a
// plan that records another one still loads: the difference is a note, not
// a refusal, because that digest also moves for changes no plan depends on.
func TestRTCoreRecordsTheMachineConfigurationAndNeverRefusesOverIt(t *testing.T) {
	r, f, _ := rtFake(t)
	describeRobot(t, f, robotDir(t), []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"cell_a"}`))
	f.mu.Lock()
	f.description.ConfigurationSHA256 = strings.Repeat("9", 64)
	f.mu.Unlock()
	cell, err := r.Cell(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if want := "sha256:" + strings.Repeat("9", 64); cell.Robot.MachineConfigurationSHA256 != want {
		t.Fatalf("machine_configuration_sha256 = %q; want %q", cell.Robot.MachineConfigurationSHA256, want)
	}
	plan := &densejointtraj.Robot{ModelID: cell.Robot.ModelID, RobotDescriptionSHA256: cell.Robot.RobotDescriptionSHA256,
		MachinePlanningCalibrationSHA256: cell.Robot.MachinePlanningCalibrationSHA256, MachinePlanningCalibrationSource: "target",
		MachineConfigurationSHA256: "sha256:" + strings.Repeat("1", 64)}
	if refusal := checkRobot(plan, &cell.Robot); refusal != nil {
		t.Fatalf("a plan was refused over the machine configuration: %v", refusal)
	}
	if note := machineConfigurationNote(plan, &cell.Robot); !strings.Contains(note, "configuration changed since this plan was made") {
		t.Fatalf("note = %q", note)
	}
	plan.MachineConfigurationSHA256 = cell.Robot.MachineConfigurationSHA256
	if note := machineConfigurationNote(plan, &cell.Robot); note != "" {
		t.Fatalf("a matching configuration was noted: %q", note)
	}
	plan.MachineConfigurationSHA256 = ""
	if note := machineConfigurationNote(plan, &cell.Robot); note != "" {
		t.Fatalf("an offline plan was noted: %q", note)
	}
}
