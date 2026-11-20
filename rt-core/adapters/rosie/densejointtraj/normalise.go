package densejointtraj

import "math"

// ContinuityEpsRad is the adapter's existing 1e-12 rad comparison in
// adapters/rosie/control/program.go at a4582b9f; hardware significance unverified.
// It is not the legacy format's permission to normalise a stationary seam.
const ContinuityEpsRad = 1e-12

// Stationary uses the format's declared 1e-6 rad/s endpoint tolerance, preventing
// a small position gap from disguising a moving handover.
func Stationary(sample Sample) bool {
	for _, velocity := range sample.QdRadS {
		if math.IsNaN(velocity) || math.IsInf(velocity, 0) || math.Abs(velocity) > BoundaryQdEpsRadS {
			return false
		}
	}
	return true
}

// MovingBoundary identifies the first discontinuous moving seam. Boundary zero
// joins segments zero and one; signed gaps are second minus first in radians.
// It is diagnostic only and never makes an invalid plan executable.
func MovingBoundary(plan *Plan) (int, [AxisCount]float64, bool) {
	for i := 1; i < len(plan.Segments); i++ {
		previous := plan.Segments[i-1].Samples
		a, b := previous[len(previous)-1], plan.Segments[i].Samples[0]
		var gaps [AxisCount]float64
		discontinuous := false
		for axis := range gaps {
			gaps[axis] = b.QRad[axis] - a.QRad[axis]
			discontinuous = discontinuous || math.Abs(gaps[axis]) > ContinuityEpsRad
		}
		if discontinuous && (!Stationary(a) || !Stationary(b)) {
			return i - 1, gaps, true
		}
	}
	return 0, [AxisCount]float64{}, false
}

// NormaliseStationary returns independent segment samples with unchanged local
// clocks and process bits. The legacy 1e-3 rad BoundaryQEpsRad comes from
// motion-server/joint-trajectory/v1/src/dense_joint_trajectory_format.hpp and the Python encoder;
// it permits only stationary seam normalisation, never interpolated bridge motion.
// The second segment owns the shared position. Replacing the preceding constant
// stationary suffix preserves its entire dwell at zero velocity. Work backwards
// so chains of dwell segments share the same endpoint. Validate again afterwards
// because moving the endpoint can invalidate the incoming segment's rate.
func NormaliseStationary(plan *Plan) ([]Segment, error) {
	if err := validateMotion(plan); err != nil {
		return nil, err
	}
	segments := append([]Segment(nil), plan.Segments...)
	for i := range segments {
		segments[i].Samples = append([]Sample(nil), segments[i].Samples...)
	}
	for i := len(segments) - 1; i > 0; i-- {
		if group := plan.Header.Segments[i].ContinuousWeld; group != "" && group == plan.Header.Segments[i-1].ContinuousWeld {
			continue
		}
		previous := segments[i-1].Samples
		last := previous[len(previous)-1]
		next := &segments[i].Samples[0]
		next.QdRadS = [AxisCount]float64{}
		for j := len(previous) - 1; j >= 0; j-- {
			if !Stationary(previous[j]) || !samePosition(previous[j].QRad, last.QRad) {
				break
			}
			previous[j].QRad = next.QRad
			previous[j].QdRadS = [AxisCount]float64{}
		}
	}
	// A chain of individually legal seams must not accumulate an unbounded
	// displacement. Each changed source sample remains within the format limit.
	for i := range segments {
		for j, sample := range segments[i].Samples {
			for axis, q := range sample.QRad {
				if math.Abs(q-plan.Segments[i].Samples[j].QRad[axis]) > BoundaryQEpsRad {
					return nil, reject("boundary_q_discontinuity", i, "normalisation exceeds %v rad at sample %d axis %d", BoundaryQEpsRad, j, axis)
				}
			}
		}
	}
	copyPlan := *plan
	copyPlan.Segments = segments
	if err := validateMotion(&copyPlan); err != nil {
		return nil, err
	}
	return segments, nil
}

func samePosition(a, b [AxisCount]float64) bool {
	for axis := range a {
		if math.Abs(a[axis]-b[axis]) > ContinuityEpsRad {
			return false
		}
	}
	return true
}
