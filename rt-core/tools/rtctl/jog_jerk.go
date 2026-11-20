package rtctl

import (
	"math"
	"strconv"
)

// jogJerkMaximum is an unverified admission cap on the jerk value, not a
// measurement; it keeps an absurd entry from silently meaning "unsmoothed".
const jogJerkMaximum = 100000

// jogSmoothingMaximumCycles mirrors JogSmoother::max_window in engine/jog.hpp,
// which sizes the native fixed per-axis history.
const jogSmoothingMaximumCycles = 1024

// validateJogJerk returns the axes.conf jog_jerk value "<jerk>|<window
// cycles>", identical on every axis, or "" when the machine omits the block.
//
// The jog integrator alone is acceleration-limited: every jog start, EndJog,
// expiry and Halt ramp steps the commanded acceleration within one cycle
// (a Rosie 1430 arm, 2026-10-06: 720 deg/s^2 in one 1 ms cycle; J4 rang at ~5.9 Hz). The
// native JogSmoother commands the mean of the integrator's last N positions,
// which bounds jerk by 2*a/(N*cycle) on every axis for any input, reversals
// included. N is sized from the largest acceleration any axis can ramp at
// (jog_acceleration, or max_acceleration when a Halt takes over a trajectory),
// so one joint-unit jerk bounds every axis and all axes share one window: a
// coordinated (Cartesian) jog keeps its direction through starts and stops.
//
// jerk_rad_s3 is a joint-angle value, so every axis must be rotary. Omission
// compiles no line: unsmoothed jog, identities unchanged. The second result is
// the jerk in rad/s^3 (zero when omitted) for Compiled.JogJerk.
func validateJogJerk(machine object, axes []any, profiles []any, cycleNS uint64) (string, float64) {
	for _, member := range machine {
		if member.key == "jog_jerk" && member.value == nil {
			reject("jog_jerk must be an explicit object")
		}
	}
	value := machine.optional("jog_jerk", nil)
	if value == nil {
		return "", 0
	}
	block, ok := value.(object)
	if !ok {
		reject("jog_jerk must be an object")
	}
	knownFields(block, "jerk_rad_s3")
	jerkText := integer(block.get("jerk_rad_s3"), 1, jogJerkMaximum, "jog_jerk jerk_rad_s3")
	jerk, _ := strconv.ParseFloat(jerkText, 64)
	var acceleration float64
	for i, value := range axes {
		axis := asObject(value)
		if kind := axis.optional("type", "rotary"); kind != "rotary" {
			reject("jog_jerk jerk_rad_s3 is a joint-angle value; axis %d is %v", i, kind)
		}
		limits := asObject(axis.get("limits"))
		acceleration = math.Max(acceleration, finite(limits.get("jog_acceleration"), "jog_acceleration"))
		source := asObject(profiles[i])
		for _, entry := range axis {
			if entry.key == "max_acceleration" {
				source = object{entry}
				break
			}
		}
		if bound := source.optional("max_acceleration", nil); bound != nil {
			acceleration = math.Max(acceleration, finite(bound, "max_acceleration"))
		}
	}
	window := math.Ceil(2 * acceleration / jerk * 1e9 / float64(cycleNS))
	if !(window >= 2) {
		reject("jog_jerk %s rad/s^3 does not limit an acceleration of %g rad/s^2 at this cycle; omit it", jerkText, acceleration)
	}
	if window > jogSmoothingMaximumCycles {
		reject("jog_jerk %s rad/s^3 needs a %g-cycle window, above the native %d", jerkText, window, jogSmoothingMaximumCycles)
	}
	return jerkText + "|" + strconv.FormatFloat(window, 'f', 0, 64), jerk
}
