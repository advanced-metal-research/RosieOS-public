// Package cartesianpath holds the Cartesian path-hold geometry shared by the
// OLP path hold (internal/denseexec) and the offline acceptance harness
// (tools/cartesianjog-harness), so the harness measures the same sample caps
// and deviation definitions the controller uses. Pure computation, no build
// tag and no controller dependency. The deviation geometry itself lives in
// rt-core/cartesian, shared with tools/tuningreport.
package cartesianpath

import (
	"math"

	"rosieos/rt-core/cartesian"
)

// SampleCaps returns one speed cap per sample in path units/s (+Inf for
// none). It carries OLP's joint velocity reserve onto the path: native
// enforces the configured velocity exactly, so the reserve (the caller
// passes the same reserve its holds and moves use) must reach it as a cap on the
// path speed, from each sample's chord to its predecessor (native takes the
// lower cap of a segment's two knots, so each segment sees both chords). A
// singularity scale below 1 (between the resolver's gates) caps the path at
// that fraction of the operator's 100 % speed. previous is the sample before
// samples[0] (nil for the first chunk).
func SampleCaps(previous *[6]float64, samples [][6]float64, scales []float64, spacing, boxSpeed float64, velocity [6]float64, reserve float64) []float64 {
	caps := make([]float64, len(samples))
	for i := range samples {
		caps[i] = math.Inf(1)
		var before *[6]float64
		if i > 0 {
			before = &samples[i-1]
		} else {
			before = previous
		}
		if before == nil && len(samples) > 1 {
			before = &samples[1]
		}
		if before != nil {
			for j := range velocity {
				slope := math.Abs(samples[i][j]-before[j]) / spacing
				if slope > 0 && velocity[j] > 0 {
					caps[i] = math.Min(caps[i], reserve*velocity[j]/slope)
				}
			}
		}
		if i < len(scales) && scales[i] > 0 && scales[i] < 1 {
			caps[i] = math.Min(caps[i], scales[i]*boxSpeed)
		}
	}
	return caps
}

// LineDeviation is cartesian.LineDeviation (rt-core/cartesian), the one
// definition the hold's monitor, the offline harness and tools/tuningreport
// judge a hold by: linear axes report the TCP distance from the frozen line
// in mm and the orientation drift in degrees, rotational axes the TCP drift in
// mm and the rotation off the commanded axis in degrees; along is the signed
// travel (mm or degrees).
func LineDeviation(origin, actual []float64, frame string, axis, direction int) (offPathMM, orientationDeg, along float64, ok bool) {
	return cartesian.LineDeviation(origin, actual, frame, axis, direction)
}
