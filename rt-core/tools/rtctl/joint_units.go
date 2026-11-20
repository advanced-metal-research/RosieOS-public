package rtctl

import (
	"math"
	"strconv"
)

// Nothing is joint-specific: a bound declared in joint units (radians, rad/s)
// is one value for every axis, converted here to that axis's drive counts by
// its own gearing. A fixed count bound would mean a different joint angle on
// every gear ratio (16 counts is 0.0024 degrees at 18:1 but 0.00044 degrees at
// 100:1), which is the failure these conversions exist to prevent.

// jointCountsPerRad mirrors native counts_per_unit for a rotary axis:
// wire_counts_per_rev * wire_revs_per_axis_rev / (2*pi). For drive-native
// A6 gearing this is 131072 * gear / (2*pi).
func jointCountsPerRad(axis object) float64 {
	if asString(axis.get("type")) != "rotary" {
		reject("axis %s joint-angle bound requires a rotary axis", asString(axis.get("name")))
	}
	counts := finite(axis.get("wire_counts_per_rev"), "wire counts/rev")
	ratio := finite(axis.get("wire_revs_per_axis_rev"), "wire revs/axis rev")
	if counts <= 0 || ratio <= 0 {
		reject("wire scaling must be positive")
	}
	// Measured mathematical constant: radians per revolution, rounded to binary64.
	return counts * ratio / 6.28318530717958647692
}

// jointAngleCounts floors a positive joint-unit bound to whole axis counts, so
// the native count bound never admits more than the declared joint angle or
// rate. A value that floors below one count on some axis is refused rather than
// silently becoming a zero tolerance on that axis.
func jointAngleCounts(axis object, value any, label string, maximum uint64) string {
	v := finite(value, label)
	if v <= 0 {
		reject("%s must be positive", label)
	}
	counts := math.Floor(v * jointCountsPerRad(axis))
	if counts < 1 {
		reject("axis %s %s is below one count", asString(axis.get("name")), label)
	}
	if counts > float64(maximum) {
		reject("axis %s %s exceeds %d counts", asString(axis.get("name")), label, maximum)
	}
	return strconv.FormatUint(uint64(counts), 10)
}

// A shared drive profile serves robots of different gearing (the Rosie 1430 arm and
// 1:1 benches), so a robot's joint-angle values belong to its machine, not to
// the profile. The machine's joint_unit_tolerances block declares each value
// once for every axis; where present it replaces the profile's count field,
// which otherwise remains that bench's declaration.
func machineJointUnit(machine object, key string) any {
	for _, member := range machine {
		if member.key == "joint_unit_tolerances" && member.value == nil {
			reject("machine-level joint_unit_tolerances must be an explicit object")
		}
	}
	block := machine.optional("joint_unit_tolerances", nil)
	if block == nil {
		return nil
	}
	knownFields(asObject(block), "hold_displacement_tolerance_rad", "maximum_feedback_error_rad",
		"maximum_anchor_error_rad", "start_alignment_floor_rad")
	return asObject(block).optional(key, nil)
}

// jointUnitOrCounts converts the machine's joint-unit value for jointKey on
// this axis, or validates the profile's count value when the machine has none.
func jointUnitOrCounts(machine, axis object, jointKey string, counts any, minimum int64, maximum uint64, label string) string {
	if joint := machineJointUnit(machine, jointKey); joint != nil {
		return jointAngleCounts(axis, joint, "joint_unit_tolerances "+jointKey, maximum)
	}
	return integer(counts, minimum, maximum, label)
}

// feedbackErrorCounts is the encoder/PDO agreement bound on this axis.
func feedbackErrorCounts(machine, axis, coordinate object) string {
	return jointUnitOrCounts(machine, axis, "maximum_feedback_error_rad",
		coordinate.get("maximum_feedback_error_counts"), 0, math.MaxUint64, "maximum_feedback_error_counts")
}

// brakeGuardForms are the count fields of each guard and their joint-unit forms.
var brakeGuardForms = map[string]string{
	"max_displacement_counts":   "max_displacement_rad",
	"max_velocity_counts_per_s": "max_velocity_rad_per_s",
}

// axisOrMachineGuard selects one guard source for an axis. A machine-level block
// applies the same joint-unit values to every axis; combining it with a
// per-axis block would reintroduce per-joint numbers, so that is refused.
func axisOrMachineGuard(machine, axis object, key string) (any, bool) {
	for _, member := range machine {
		if member.key == key && member.value == nil {
			reject("machine-level %s must be an explicit object", key)
		}
	}
	shared := machine.optional(key, nil)
	own := axis.optional(key, nil)
	if shared != nil && own != nil {
		reject("axis %s declares %s and the machine declares one for every axis; declare it once", asString(axis.get("name")), key)
	}
	if shared != nil {
		return shared, true
	}
	return own, false
}

// renderBrakeGuardParts returns native counts and milliseconds for one guard.
// A block uses either count fields or joint-unit fields, never a mixture; a
// machine-level block is one joint-unit value set applied to every axis. The
// caller has already refused unknown fields of a per-axis block.
func renderBrakeGuardParts(axis, block object, kind string, keys []string, machineLevel bool) []string {
	if machineLevel {
		for _, m := range block {
			known := false
			for _, key := range keys {
				joint, scaled := brakeGuardForms[key]
				known = known || m.key == joint || !scaled && m.key == key
			}
			if !known {
				reject("unknown machine-level %s field %s", kind, m.key)
			}
		}
	}
	countForm, jointForm := false, false
	for count, joint := range brakeGuardForms {
		countForm = countForm || block.optional(count, nil) != nil
		jointForm = jointForm || block.optional(joint, nil) != nil
	}
	if countForm && jointForm {
		reject("%s mixes count and joint-unit fields", kind)
	}
	if machineLevel && countForm {
		reject("machine-level %s must use joint-unit fields (max_displacement_rad, max_velocity_rad_per_s)", kind)
	}
	parts := make([]string, len(keys))
	for i, key := range keys {
		if joint, ok := brakeGuardForms[key]; ok && jointForm {
			parts[i] = jointAngleCounts(axis, block.get(joint), kind+" "+joint, 0x7fffffff)
			continue
		}
		parts[i] = integer(block.get(key), 1, 0x7fffffff, kind+" "+key)
	}
	return parts
}
