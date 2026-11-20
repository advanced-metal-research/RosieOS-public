//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"math"
	"net/http"
	"strings"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

// testProgramGate is what an uploaded test program must pass on this cell
// before Load acquires authority, on top of everything a planner artifact
// passes (Home, measured-start identity, native preparation's limits). A
// planner plan is made against the cell and its start; a test program was
// made on a Mac from typed-in angles, so the cell checks what the tool could
// only assume:
//
//   - the robot it names is this cell's (model, description, calibration),
//     and a machine that cannot say which robot it is refuses;
//   - the machine configuration it was checked against is this cell's
//     configuration, exactly: its limits, gears and signs were the tool's
//     inputs (a planner plan only gets a note for this);
//   - its manifest_revision is this cell's control binding revision, which
//     native preparation requires (refused here with the reason in words);
//   - its axes are this cell's axes;
//   - every axis's measured position is within that axis's
//     completion_tolerance of the first sample, from valid feedback, so the
//     program starts where the arm is instead of jumping to where it was.
//
// It never moves anything and never widens a bound: native preparation and
// execution still apply every limit and fault after it.
func (r *RTCoreController) testProgramGate(ctx context.Context, plan *densejointtraj.Plan, status control.Status) ([]string, error) {
	d, err := r.describe(ctx)
	if err != nil {
		return nil, err
	}
	h := plan.Header
	cell := robotCellOf(d)
	if refusal := checkRobot(h.Robot, &cell); refusal != nil {
		return nil, refusal
	}
	if h.Robot.MachineConfigurationSHA256 == "" || h.Robot.MachineConfigurationSHA256 != cell.MachineConfigurationSHA256 {
		return nil, httpError(http.StatusConflict, "test_program_configuration_mismatch",
			"the program was checked against machine configuration %s, the cell runs %s; regenerate it against the cell's configuration",
			h.Robot.MachineConfigurationSHA256, cell.MachineConfigurationSHA256)
	}
	if h.ManifestRevision != r.cfg.Binding.Revision {
		return nil, httpError(http.StatusConflict, "test_program_binding_mismatch",
			"the program's manifest_revision is %d, the cell's control binding revision %d; regenerate it with the cell manifest (rt_core.pair_revision)",
			h.ManifestRevision, r.cfg.Binding.Revision)
	}
	if h.Axes.AxisMask != r.cfg.AxisMask {
		return nil, httpError(http.StatusConflict, "test_program_axes_mismatch",
			"the program drives axis mask 0x%x, the cell 0x%x", h.Axes.AxisMask, r.cfg.AxisMask)
	}
	first := plan.Segments[0].Samples[0]
	var worst string
	worstRatio := -1.0
	for i, axis := range d.Axes {
		if i >= densejointtraj.AxisCount || r.cfg.AxisMask&(1<<i) == 0 {
			continue
		}
		name := axis.ID
		if name == "" {
			name = fmt.Sprintf("axis %d", i)
		}
		tolerance := axis.CompletionTolerance
		if !(tolerance > 0) || math.IsInf(tolerance, 0) {
			return nil, httpError(http.StatusConflict, "test_program_start_unchecked", "%s describes no completion_tolerance to check the start against", name)
		}
		if i >= len(status.Axes) || !status.Axes[i].LogicalValid {
			return nil, httpError(http.StatusConflict, "test_program_start_unknown", "%s reports no valid position; the start cannot be checked", name)
		}
		measured, start := status.Axes[i].LogicalPosition, first.QRad[i]
		offset := math.Abs(measured - start)
		if math.IsNaN(offset) || offset > tolerance {
			return nil, httpError(http.StatusConflict, "test_program_start_mismatch",
				"%s is at %s, the program starts at %s (tolerance %s); move the arm to the program's start or regenerate the program from the held pose",
				name, axisValue(axis.PositionUnit, measured), axisValue(axis.PositionUnit, start), axisValue(axis.PositionUnit, tolerance))
		}
		if ratio := offset / tolerance; ratio > worstRatio {
			worstRatio, worst = ratio, fmt.Sprintf("%s off by %s of %s", name, axisValue(axis.PositionUnit, offset), axisValue(axis.PositionUnit, tolerance))
		}
	}
	checks := []string{
		fmt.Sprintf("robot %s, description %s, calibration %s: the cell's", cell.ModelID, shortDigest(cell.RobotDescriptionSHA256), shortDigest(cell.MachinePlanningCalibrationSHA256)),
		fmt.Sprintf("machine configuration %s: the cell's", shortDigest(cell.MachineConfigurationSHA256)),
		fmt.Sprintf("axes 0x%x and binding revision %d: the cell's", r.cfg.AxisMask, r.cfg.Binding.Revision),
		"start within completion_tolerance of the measured pose on every axis (worst: " + worst + ")",
	}
	return checks, nil
}

// CheckTestProgram runs Load's checks for an uploaded program without
// acquiring authority or sending its bytes, so the upload answers with the
// same refusal Load would give. Load runs them again: the arm may move or the
// configuration change between the two.
func (r *RTCoreController) CheckTestProgram(ctx context.Context, digest string) ([]string, error) {
	program, err := r.source.cfg.TestPrograms.get(digest)
	if err != nil {
		return nil, err
	}
	r.mu.Lock()
	playing := r.session != nil && r.session.State == "playing"
	r.mu.Unlock()
	if playing {
		return nil, rtFailure(rtRefused("program_already_executing"))
	}
	status, err := r.c.Status(ctx)
	if err != nil {
		return nil, rtFailure(err)
	}
	if status.Core.HomeValidMask&r.cfg.AxisMask != r.cfg.AxisMask {
		return nil, rtFailure(rtRefused("home_required: establish the current joint position reference in Home before loading the program"))
	}
	d, err := r.describe(ctx)
	if err != nil {
		return nil, rtFailure(err)
	}
	if err := validatePlanningCapture(program.plan, status, d); err != nil {
		return nil, rtFailure(err)
	}
	checks, err := r.testProgramGate(ctx, program.plan, status)
	if err != nil {
		return nil, rtFailure(err)
	}
	return checks, nil
}

func axisValue(unit string, value float64) string {
	if unit == "rad" {
		return fmt.Sprintf("%.3f deg", value*180/math.Pi)
	}
	return fmt.Sprintf("%.6g %s", value, unit)
}

func shortDigest(digest string) string {
	digest = strings.TrimPrefix(digest, "sha256:")
	if len(digest) > 12 {
		return digest[:12]
	}
	return digest
}
