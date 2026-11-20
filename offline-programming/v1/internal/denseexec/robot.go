package denseexec

import (
	"fmt"
	"net/http"
	"strings"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/offline-programming/v1/internal/robotconfig"
)

// The refusals under the robot description contract
// (docs/ROBOT-DESCRIPTION-CONTRACT.md): the words Load's early check and
// the cell route answer in, one vocabulary for the UI.
const (
	ReasonRobotCellMissing     = "robot_cell_missing"
	ReasonRobotCellUnavailable = "robot_cell_unavailable"
	ReasonRobotCellMismatch    = "robot_cell_mismatch"
)

// RobotCell is which robot a machine says it is: the description it was
// compiled with, that description's identity (a sha256 over the files its
// manifest names), and the hash of its machine_planning_calibration.json (rt_core_robot.go).
type RobotCell struct {
	Configured                        bool   `json:"configured"`
	Valid                             bool   `json:"valid"`
	ModelID                           string `json:"model_id"`
	RobotDescriptionSHA256            string `json:"robot_description_sha256"`
	MachinePlanningCalibrationSHA256  string `json:"machine_planning_calibration_sha256"`
	MachinePlanningCalibrationPresent bool   `json:"machine_planning_calibration_present"`
	// MachineConfigurationSHA256 is the machine's whole compiled
	// configuration, "sha256:" + rt-core's configuration_sha256. Recorded in
	// the plans made for this machine; never a reason to refuse one.
	MachineConfigurationSHA256 string `json:"machine_configuration_sha256,omitempty"`
	Error                      string `json:"error"`
}

// CellBody is the HTTP body of GET .../dense-execution/cell: the machine's
// identity and the exact bytes it hashed, which the planner is handed
// verbatim.
type CellBody struct {
	Robot                            RobotCell                   `json:"robot_cell"`
	MachinePlanningCalibrationBase64 string                      `json:"machine_planning_calibration_base64"`
	ExecutionLimits                  robotconfig.ExecutionLimits `json:"execution_limits,omitempty"`
}

// DescriptionBody is a fetched description: the identity the machine
// serves it under and its files -- the manifest and everything it
// registers -- by description-relative path (docs/PLANNING-CONTEXT.md).
// The OLP recomputes the identity over these bytes before it keeps them.
type DescriptionBody struct {
	Robot RobotCell
	Files map[string][]byte
}

// checkRobot is the gate Load asks before anything is acquired, so the
// answer comes back in words. It only runs when the target reports an
// identity at all: a machine that binds no robot has nothing to judge by.
func checkRobot(plan *densejointtraj.Robot, target *RobotCell) *Error {
	if target == nil {
		return nil
	}
	// The target first: a machine that cannot say which robot it is refuses
	// every plan, and telling the operator to re-plan against it would send
	// them round again to the same refusal. Only a machine that can judge is
	// asked whether the plan gave it something to judge.
	if !target.Configured {
		return httpError(http.StatusConflict, ReasonRobotCellUnavailable,
			"the target binds no robot description")
	}
	if !target.Valid {
		return httpError(http.StatusConflict, ReasonRobotCellUnavailable,
			"the target's robot identity is not valid: %s", target.Error)
	}
	if plan == nil {
		return httpError(http.StatusConflict, ReasonRobotCellMissing,
			"the plan names no robot description or cell; it was planned before the cell "+
				"checked, so re-plan it")
	}
	var differs []string
	for _, field := range []struct{ name, plan, cell string }{
		{"model_id", plan.ModelID, target.ModelID},
		{"robot_description_sha256", plan.RobotDescriptionSHA256, target.RobotDescriptionSHA256},
		{"machine_planning_calibration_sha256", plan.MachinePlanningCalibrationSHA256, target.MachinePlanningCalibrationSHA256},
	} {
		if field.plan != field.cell {
			differs = append(differs, fmt.Sprintf("%s: plan %s, cell %s", field.name, field.plan, field.cell))
		}
	}
	if len(differs) == 0 {
		return nil
	}
	return httpError(http.StatusConflict, ReasonRobotCellMismatch, "%s", strings.Join(differs, "; "))
}

// machineConfigurationNote is what Load says when the plan records another
// machine configuration than the machine has now: true, worth knowing, and
// not a refusal -- the digest also moves for changes no plan depends on, so
// the fence stays the description and the planning calibration.
func machineConfigurationNote(plan *densejointtraj.Robot, target *RobotCell) string {
	if plan == nil || target == nil || plan.MachineConfigurationSHA256 == "" || target.MachineConfigurationSHA256 == "" ||
		plan.MachineConfigurationSHA256 == target.MachineConfigurationSHA256 {
		return ""
	}
	return fmt.Sprintf("the machine's configuration changed since this plan was made (plan %s, machine %s); "+
		"the robot description and planning calibration still match, so it loads",
		plan.MachineConfigurationSHA256, target.MachineConfigurationSHA256)
}

// isRobotReason reports whether a reason is one of the contract's, so the
// HTTP code can carry it verbatim instead of a generic "rejected".
func isRobotReason(reason string) bool {
	return strings.HasPrefix(reason, "robot_cell_")
}
