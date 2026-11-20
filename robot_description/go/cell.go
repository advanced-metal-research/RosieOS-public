package robotdesc

// The cell's own settings: read here, checked here, and handed on as numbers.
//
// `machine_planning_calibration.json` says how one cell deviates from the model it mounts -- limits it
// narrows, joint origins a probe jig measured. It is the cell's file, fetched
// from the cell, and this is the only place in the OLP that parses it. What
// leaves here is Settings: per-joint numbers, already bounded. The seam worker
// applies them and never sees the schema, so there is one implementation of
// "is this deviation allowed", not two that can disagree.
//
// The rules are the bounded-deviation rules of
// docs/ROBOT-DESCRIPTION-CONTRACT.md, the same ones the daemon applies on the
// cell before it will play anything: limits only narrow, a hold sits inside
// them, a correction stays under the description's cap, and a key nobody
// defined is an error rather than something quietly ignored.

import (
	"encoding/json"
	"fmt"
	"math"
	"sort"
	"strings"
)

// CellSchema is the only schema machine_planning_calibration.json may carry.
const CellSchema = "rosie.machine-planning-calibration.v1"

// Where a plan's cell settings came from. "empty" is not a failure: it is a
// plan made against the model's empty document, which a cell that has no
// machine_planning_calibration.json of its own hashes to exactly, and which the cell refuses if it has
// one.
const (
	SourceTarget = "target"
	SourceEmpty  = "empty"
)

// Identity is what the trajectory header carries and the daemon compares.
type Identity struct {
	ModelID                          string `json:"model_id"`
	RobotDescriptionSHA256           string `json:"robot_description_sha256"`
	MachinePlanningCalibrationSHA256 string `json:"machine_planning_calibration_sha256"`
	MachinePlanningCalibrationSource string `json:"machine_planning_calibration_source"`
	// MachineConfigurationSHA256 is the rt-core configuration digest of the
	// machine the plan was made for ("sha256:" + the machine's
	// configuration_sha256), when there was one to ask. A RECORD, not part of
	// the fence: it covers the two identities above and also everything about
	// the machine a plan does not depend on -- a CPU number, a bus timeout --
	// so a plan is never refused over it. Absent on a plan made offline.
	MachineConfigurationSHA256 string `json:"machine_configuration_sha256,omitempty"`
}

// JointLimit is one joint's bounds and rate as this cell enforces them, after
// narrowing. Every field is set; the planner needs no defaults of its own.
type JointLimit struct {
	LowerRad    float64  `json:"lower_rad"`
	UpperRad    float64  `json:"upper_rad"`
	VelocityRad *float64 `json:"velocity_rad_s,omitempty"`
}

// Correction is a joint origin's measured offset, composed onto the URDF's.
type Correction struct {
	XYZMetres  [3]float64 `json:"xyz_m"`
	RPYRadians [3]float64 `json:"rpy_rad"`
}

// Settings is what the seam worker is handed: the identity to stamp and the
// numbers to apply. No schema, no document, no bytes.
type Settings struct {
	Identity         Identity              `json:"identity"`
	JointLimits      map[string]JointLimit `json:"joint_limits,omitempty"`
	JointCorrections map[string]Correction `json:"joint_corrections,omitempty"`
	Held             map[string]float64    `json:"held,omitempty"`
	SpeedCeilingRadS float64               `json:"speed_ceiling_rad_s,omitempty"`
}

// EmptyMachinePlanningCalibrationBytes is the document a cell with no machine_planning_calibration.json serves and hashes --
// byte for byte what the daemon synthesises, so a plan made while the target
// was unreachable still matches an unconfigured cell.
func EmptyMachinePlanningCalibrationBytes(modelID string) []byte {
	return []byte(`{"schema":"rosie.machine-planning-calibration.v1","model_id":"` + modelID + `"}` + "\n")
}

type cellDocument struct {
	Schema     string                    `json:"schema"`
	ModelID    string                    `json:"model_id"`
	Limits     *cellLimits               `json:"limits"`
	Kinematics map[string]cellCorrection `json:"kinematics"`
}

type cellLimits struct {
	SpeedCeilingRadS *float64                  `json:"speed_ceiling_rad_s"`
	Joints           map[string]cellJointLimit `json:"joints"`
	Held             map[string]float64        `json:"held"`
}

type cellJointLimit struct {
	LowerRad    *float64 `json:"lower_rad"`
	UpperRad    *float64 `json:"upper_rad"`
	VelocityRad *float64 `json:"velocity_rad_s"`
}

type cellCorrection struct {
	XYZMetres  []float64 `json:"xyz_m"`
	RPYRadians []float64 `json:"rpy_rad"`
}

// Resolve validates `machinePlanningCalibration` against the description and returns the settled
// numbers. Pass nil to plan against the model's empty document.
//
// `source` says where the bytes came from; an empty document may only be
// "empty", because claiming a cell was read when it was not is exactly the
// mistake the header's machine_planning_calibration_source exists to make visible.
func Resolve(description Description, machinePlanningCalibration []byte, source string) (Settings, error) {
	if source == "" {
		if len(machinePlanningCalibration) == 0 {
			source = SourceEmpty
		} else {
			source = SourceTarget
		}
	}
	if source != SourceTarget && source != SourceEmpty {
		return Settings{}, fmt.Errorf("machine_planning_calibration_source %q is neither %q nor %q",
			source, SourceTarget, SourceEmpty)
	}
	if len(machinePlanningCalibration) == 0 {
		if source != SourceEmpty {
			return Settings{}, fmt.Errorf("machine_planning_calibration_source %q without a machine_planning_calibration.json", source)
		}
		machinePlanningCalibration = EmptyMachinePlanningCalibrationBytes(description.ModelID)
	}
	settings := Settings{Identity: Identity{
		ModelID:                          description.ModelID,
		RobotDescriptionSHA256:           description.Identity,
		MachinePlanningCalibrationSHA256: MachinePlanningCalibrationSHA256(machinePlanningCalibration),
		MachinePlanningCalibrationSource: source,
	}}

	var document cellDocument
	decoder := json.NewDecoder(strings.NewReader(string(machinePlanningCalibration)))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&document); err != nil {
		return Settings{}, fmt.Errorf("machine_planning_calibration.json: %w", err)
	}
	if document.Schema != CellSchema {
		return Settings{}, fmt.Errorf("machine_planning_calibration.json: schema %q is not %q", document.Schema, CellSchema)
	}
	if document.ModelID != description.ModelID {
		return Settings{}, fmt.Errorf("machine_planning_calibration.json is for model %q, the description is %q",
			document.ModelID, description.ModelID)
	}

	revolute := func(name, where string) (URDFJoint, error) {
		joint, known := description.URDF.Joints[name]
		if !known {
			return URDFJoint{}, fmt.Errorf("machine_planning_calibration.json: %s names joint %s, which the URDF does not have",
				where, name)
		}
		if !joint.Revolute() {
			return URDFJoint{}, fmt.Errorf("machine_planning_calibration.json: %s names joint %s, which is %s, not revolute",
				where, name, joint.Type)
		}
		return joint, nil
	}

	if limits := document.Limits; limits != nil {
		if limits.SpeedCeilingRadS != nil {
			ceiling := *limits.SpeedCeilingRadS
			fastest, any := description.URDF.FastestRevolute()
			if !finite(ceiling) || !(ceiling > 0) {
				return Settings{}, fmt.Errorf("machine_planning_calibration.json: limits.speed_ceiling_rad_s must be positive")
			}
			if !any || ceiling > fastest {
				return Settings{}, fmt.Errorf(
					"machine_planning_calibration.json: limits.speed_ceiling_rad_s %g is above the URDF's fastest joint", ceiling)
			}
			settings.SpeedCeilingRadS = ceiling
		}
		for _, name := range sortedLimitNames(limits.Joints) {
			spec := limits.Joints[name]
			where := "limits.joints." + name
			joint, err := revolute(name, "limits.joints")
			if err != nil {
				return Settings{}, err
			}
			if !joint.HasLower || !joint.HasUpper {
				return Settings{}, fmt.Errorf(
					"machine_planning_calibration.json: %s: the URDF states no position limit to narrow", where)
			}
			bounds := JointLimit{LowerRad: joint.Lower, UpperRad: joint.Upper}
			if spec.LowerRad != nil {
				if !finite(*spec.LowerRad) {
					return Settings{}, fmt.Errorf("machine_planning_calibration.json: %s.lower_rad is not a number", where)
				}
				if *spec.LowerRad < joint.Lower {
					return Settings{}, fmt.Errorf("machine_planning_calibration.json: %s.lower_rad %g widens the URDF lower limit %g",
						where, *spec.LowerRad, joint.Lower)
				}
				bounds.LowerRad = *spec.LowerRad
			}
			if spec.UpperRad != nil {
				if !finite(*spec.UpperRad) {
					return Settings{}, fmt.Errorf("machine_planning_calibration.json: %s.upper_rad is not a number", where)
				}
				if *spec.UpperRad > joint.Upper {
					return Settings{}, fmt.Errorf("machine_planning_calibration.json: %s.upper_rad %g widens the URDF upper limit %g",
						where, *spec.UpperRad, joint.Upper)
				}
				bounds.UpperRad = *spec.UpperRad
			}
			if !(bounds.LowerRad < bounds.UpperRad) {
				return Settings{}, fmt.Errorf("machine_planning_calibration.json: %s leaves lower %g >= upper %g",
					where, bounds.LowerRad, bounds.UpperRad)
			}
			if spec.VelocityRad != nil {
				rate := *spec.VelocityRad
				if !joint.HasVelocity {
					return Settings{}, fmt.Errorf(
						"machine_planning_calibration.json: %s.velocity_rad_s: the URDF states no velocity to narrow", where)
				}
				if !finite(rate) || !(rate > 0) || rate > joint.Velocity {
					return Settings{}, fmt.Errorf(
						"machine_planning_calibration.json: %s.velocity_rad_s %g is not within (0, %g], the URDF velocity",
						where, rate, joint.Velocity)
				}
				value := rate
				bounds.VelocityRad = &value
			}
			if settings.JointLimits == nil {
				settings.JointLimits = map[string]JointLimit{}
			}
			settings.JointLimits[name] = bounds
		}
		for _, name := range sortedHeldNames(limits.Held) {
			value := limits.Held[name]
			joint, err := revolute(name, "limits.held")
			if err != nil {
				return Settings{}, err
			}
			if !finite(value) {
				return Settings{}, fmt.Errorf("machine_planning_calibration.json: limits.held.%s is not a number", name)
			}
			lower, upper, bounded := joint.Lower, joint.Upper, joint.HasLower && joint.HasUpper
			if narrowed, ok := settings.JointLimits[name]; ok {
				lower, upper, bounded = narrowed.LowerRad, narrowed.UpperRad, true
			}
			if !bounded || value < lower || value > upper {
				return Settings{}, fmt.Errorf(
					"machine_planning_calibration.json: limits.held.%s %g is outside the joint's limits", name, value)
			}
			if settings.Held == nil {
				settings.Held = map[string]float64{}
			}
			settings.Held[name] = value
		}
	}

	for _, name := range sortedCorrectionNames(document.Kinematics) {
		spec := document.Kinematics[name]
		if _, known := description.URDF.Joints[name]; !known {
			return Settings{}, fmt.Errorf(
				"machine_planning_calibration.json: kinematics names joint %s, which the URDF does not have", name)
		}
		correction := Correction{}
		for _, field := range []struct {
			key    string
			values []float64
			cap    float64
			out    *[3]float64
		}{
			{"xyz_m", spec.XYZMetres, description.Config.Caps.XYZMetres, &correction.XYZMetres},
			{"rpy_rad", spec.RPYRadians, description.Config.Caps.RPYRadians, &correction.RPYRadians},
		} {
			if field.values == nil {
				continue
			}
			if len(field.values) != 3 {
				return Settings{}, fmt.Errorf("machine_planning_calibration.json: kinematics.%s.%s must be three numbers",
					name, field.key)
			}
			for i, value := range field.values {
				if !finite(value) {
					return Settings{}, fmt.Errorf("machine_planning_calibration.json: kinematics.%s.%s[%d] is not a number",
						name, field.key, i)
				}
				if math.Abs(value) > field.cap {
					return Settings{}, fmt.Errorf(
						"machine_planning_calibration.json: kinematics.%s.%s[%d] %g exceeds the description's cap %g",
						name, field.key, i, value, field.cap)
				}
				field.out[i] = value
			}
		}
		if settings.JointCorrections == nil {
			settings.JointCorrections = map[string]Correction{}
		}
		settings.JointCorrections[name] = correction
	}
	return settings, nil
}

func finite(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }

// Sorted so a refusal names the same joint every run: a request that differs
// only in map order would otherwise report a different first offender.
func sortedLimitNames(values map[string]cellJointLimit) []string {
	out := make([]string, 0, len(values))
	for name := range values {
		out = append(out, name)
	}
	sort.Strings(out)
	return out
}

func sortedHeldNames(values map[string]float64) []string {
	out := make([]string, 0, len(values))
	for name := range values {
		out = append(out, name)
	}
	sort.Strings(out)
	return out
}

func sortedCorrectionNames(values map[string]cellCorrection) []string {
	out := make([]string, 0, len(values))
	for name := range values {
		out = append(out, name)
	}
	sort.Strings(out)
	return out
}
