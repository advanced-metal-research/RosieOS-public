package robotconfig

import (
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"math"
	"sort"
)

// ExecutionLimits is a verified connected machine's native limit snapshot,
// keyed by robot joint name. It is separate from calibration: these limits
// constrain a plan without changing the robot description or calibration pin.
type ExecutionLimits map[string]ExecutionAxisLimit

type ExecutionAxisLimit struct {
	PositionUnit      string   `json:"position_unit"`
	LowerRad          float64  `json:"lower_rad"`
	UpperRad          float64  `json:"upper_rad"`
	VelocityRadS      float64  `json:"velocity_rad_s"`
	AccelerationRadS2 *float64 `json:"acceleration_rad_s2,omitempty"`
}

// applyExecutionLimits only narrows the already-composed model/calibration.
// Bounds feed both planner.json and the corrected URDF, so every planning
// backend judges the same machine envelope.
func applyExecutionLimits(planner *PlannerDocument, execution ExecutionLimits) error {
	finite := func(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }
	for _, name := range planner.Driven {
		if _, ok := execution[name]; !ok {
			return fmt.Errorf("native execution limits are missing driven joint %s", name)
		}
	}
	names := make([]string, 0, len(execution))
	for name := range execution {
		names = append(names, name)
	}
	sort.Strings(names)
	for _, name := range names {
		axis := execution[name]
		if axis.PositionUnit != "rad" || !finite(axis.LowerRad) || !finite(axis.UpperRad) ||
			axis.LowerRad >= axis.UpperRad || !finite(axis.VelocityRadS) || axis.VelocityRadS <= 0 ||
			(axis.AccelerationRadS2 != nil && (!finite(*axis.AccelerationRadS2) || *axis.AccelerationRadS2 <= 0)) {
			return fmt.Errorf("native execution limits for %s need radian positions and finite positive rates", name)
		}
		bounds, ok := planner.Bounds[name]
		if !ok {
			return fmt.Errorf("native execution limits name %s, which has no bounded robot joint", name)
		}
		bounds.LowerRad = math.Max(bounds.LowerRad, axis.LowerRad)
		bounds.UpperRad = math.Min(bounds.UpperRad, axis.UpperRad)
		if bounds.LowerRad >= bounds.UpperRad {
			return fmt.Errorf("native execution limits for %s do not overlap the robot's planning bounds", name)
		}
		planner.Bounds[name] = bounds
		if rate, ok := planner.Limits[name]; ok {
			rate.EnforcedVelocityRadS = math.Min(rate.EnforcedVelocityRadS, axis.VelocityRadS)
			rate.VelocityRadS = math.Min(rate.VelocityRadS, rate.EnforcedVelocityRadS)
			if axis.AccelerationRadS2 != nil {
				rate.AccelerationRadS2 = math.Min(rate.AccelerationRadS2, *axis.AccelerationRadS2)
			}
			planner.Limits[name] = rate
		}
	}
	// Virtual held wire columns need no native axis. Actual held/reset joints
	// must remain reachable after all intersections, never silently clamped.
	for _, role := range []struct {
		name   string
		values map[string]float64
	}{{"held", planner.Held}, {"reset", planner.ResetPose}} {
		for name, value := range role.values {
			if bounds, ok := planner.Bounds[name]; ok &&
				(!finite(value) || value < bounds.LowerRad || value > bounds.UpperRad) {
				return fmt.Errorf("%s pose for %s is outside the native planning bounds", role.name, name)
			}
		}
	}
	// Bind the whole snapshot, including limits that happened not to tighten
	// this model, into the packed context so a native change invalidates caches.
	digest := sha256.Sum256(canonicalJSON(execution))
	planner.ExecutionLimitsSHA256 = "sha256:" + hex.EncodeToString(digest[:])
	return nil
}
