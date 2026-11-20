package rtctl

import (
	"math"
	"strconv"
)

// driveSupervision is the machine-level drive-state supervision block: one
// value per bound for every axis. Timeouts are milliseconds; the motion bound
// is joint units (rad on a rotary axis, m on a linear axis) floored to each
// axis's counts by its own gearing, so no joint carries its own number. Home
// acquisition stationarity is joint_unit_tolerances' held tolerance, not a
// second value here.
type driveSupervision struct {
	enableTimeoutMS, releaseSlackMS string
	disarmMotion                    float64
}

// validateDriveSupervision returns nil when the machine omits the block. The
// native daemon then runs no enable, release or disarm-motion check; nothing
// substitutes an undeclared number, and existing configuration identities are
// unchanged.
func validateDriveSupervision(machine object) *driveSupervision {
	for _, member := range machine {
		if member.key == "drive_supervision" && member.value == nil {
			reject("drive_supervision must be an explicit object")
		}
	}
	value := machine.optional("drive_supervision", nil)
	if value == nil {
		return nil
	}
	block, ok := value.(object)
	if !ok {
		reject("drive_supervision must be an object")
	}
	knownFields(block, "enable_timeout_ms", "release_slack_ms", "disarm_motion_tolerance")
	s := &driveSupervision{
		enableTimeoutMS: integer(block.get("enable_timeout_ms"), 1, math.MaxInt32, "drive_supervision enable_timeout_ms"),
		releaseSlackMS:  integer(block.get("release_slack_ms"), 1, math.MaxInt32, "drive_supervision release_slack_ms"),
		disarmMotion:    finite(block.get("disarm_motion_tolerance"), "drive_supervision disarm_motion_tolerance"),
	}
	if s.disarmMotion <= 0 {
		reject("drive_supervision tolerances must be positive")
	}
	return s
}

// supervisionCountsPerUnit mirrors native counts_per_unit:
// wire_counts_per_rev * wire_revs_per_axis_rev / (2*pi rad, or lead m).
func supervisionCountsPerUnit(axis object) float64 {
	counts := finite(axis.get("wire_counts_per_rev"), "wire counts/rev")
	ratio := finite(axis.get("wire_revs_per_axis_rev"), "wire revs/axis rev")
	if asString(axis.get("type")) == "linear" {
		return counts * ratio / finite(axis.get("lead_m_per_rev"), "lead")
	}
	// Measured mathematical constant: radians per revolution, rounded to binary64.
	return counts * ratio / 6.28318530717958647692
}

// render floors the joint-unit bound to whole counts so the native bound never
// admits more than the declared joint motion. A bound that floors below one
// count on some axis is refused rather than becoming zero on that axis.
func (s *driveSupervision) render(axis object) string {
	perUnit := supervisionCountsPerUnit(axis)
	counts := func(value float64, key string) string {
		c := math.Floor(value * perUnit)
		if c < 1 {
			reject("axis %s drive_supervision %s is below one count", asString(axis.get("name")), key)
		}
		if c > math.MaxInt32 {
			reject("axis %s drive_supervision %s exceeds %d counts", asString(axis.get("name")), key, math.MaxInt32)
		}
		return strconv.FormatUint(uint64(c), 10)
	}
	return s.enableTimeoutMS + "|" + s.releaseSlackMS + "|" + counts(s.disarmMotion, "disarm_motion_tolerance")
}
