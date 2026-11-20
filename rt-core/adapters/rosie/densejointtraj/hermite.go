package densejointtraj

import (
	"errors"
	"math"
	"sort"
)

// Share the failure value so successful per-segment checks do not allocate it.
var errInvalidHermite = errors.New("invalid Hermite command frame or segment")

// CommandFrame describes native CSP scaling and the current Home offset. The
// offset is subtracted after scaling, preventing a different count-frame curve.
type CommandFrame struct {
	CountsPerUnit float64
	Sign          int
	HomeOffset    int32
}

// HermiteKnot uses an already rounded native nanosecond clock and logical rad/m
// position and velocity. Producers must preserve these clocks when encoding.
type HermiteKnot struct {
	TimeNS             int64
	Position, Velocity float64
}

// HermiteBounds bounds projected CSP position and guarded absolute velocity in
// counts. Compare PeakVelocity to min(source cap, Describe cap)*CountsPerUnit;
// equality passes. Compare position to the signed, Home-shifted limit interval.
type HermiteBounds struct {
	MinPosition, MaxPosition, PeakVelocity float64
}

// BoundHermiteSegment shares native admission arithmetic with offline producers.
// Binary64 operations are explicitly unfused; nonzero peaks round one ULP toward
// positive infinity. Invalid clocks, scaling or arithmetic fail closed, so a
// producer cannot treat an unbounded curve as a prepared or safe segment.
func BoundHermiteSegment(frame CommandFrame, first, second HermiteKnot) (HermiteBounds, error) {
	if !(frame.CountsPerUnit > 0) || math.IsInf(frame.CountsPerUnit, 0) ||
		(frame.Sign != -1 && frame.Sign != 1) || first.TimeNS < 0 || second.TimeNS <= first.TimeNS {
		return HermiteBounds{}, errInvalidHermite
	}
	scale := frame.CountsPerUnit * float64(frame.Sign)
	home := float64(frame.HomeOffset)
	q0, q1 := float64(first.Position*scale)-home, float64(second.Position*scale)-home
	v0, v1 := first.Velocity*scale, second.Velocity*scale
	dt := float64(second.TimeNS-first.TimeNS) / 1e9
	delta := q1 - q0
	minimum, maximum := hermitePositionRange(q0, delta, dt, v0, v1)
	endpoint := q1
	minimum, maximum = math.Min(minimum, endpoint), math.Max(maximum, endpoint)
	peak := hermiteMaxAbsVelocity(delta, dt, v0, v1)
	if peak != 0 {
		peak = math.Nextafter(peak, math.Inf(1))
	}
	if math.IsNaN(minimum) || math.IsInf(minimum, 0) || math.IsNaN(maximum) || math.IsInf(maximum, 0) || math.IsNaN(peak) || math.IsInf(peak, 0) {
		return HermiteBounds{}, errInvalidHermite
	}
	return HermiteBounds{minimum, maximum, peak}, nil
}

// hermitePositionRange checks all derivative roots and projected wrap seams so
// legal knots cannot hide an out-of-limit commanded position between samples.
func hermitePositionRange(first, delta, dt, v0, v1 float64) (float64, float64) {
	c2 := float64(3*delta) - float64(dt*(float64(2*v0)+v1))
	c3 := float64(-2*delta) + float64(dt*(v0+v1))
	times := hermitePositionTimes(dt, v0, c2, c3)
	if math.IsNaN(times[0]) || math.IsNaN(first) || math.IsInf(first, 0) || math.IsNaN(delta) || math.IsInf(delta, 0) {
		return math.NaN(), math.NaN()
	}
	minimum, maximum := math.Min(first, first+delta), math.Max(first, first+delta)
	include := func(u float64) {
		if u > 0 && u < 1 {
			q := first + float64(u*(float64(dt*v0)+float64(u*(c2+float64(u*c3)))))
			minimum, maximum = math.Min(minimum, q), math.Max(maximum, q)
		}
	}
	for _, u := range times {
		include(u)
	}
	if math.IsNaN(minimum) || math.IsNaN(maximum) || math.IsInf(minimum, 0) || math.IsInf(maximum, 0) {
		return math.NaN(), math.NaN()
	}
	return minimum, maximum
}

// A cubic has at most two interior velocity zeros. Sorting them splits recovery
// into monotone pieces, so an outward reversal cannot hide inside its bounds.
func hermitePositionTimes(dt, v0, c2, c3 float64) [4]float64 {
	times := [4]float64{0, 1, 1, 1}
	a, b, c := 3*c3, 2*c2, dt*v0
	discriminant := float64(b*b) - float64(float64(4*a)*c)
	if dt <= 0 || !finite(dt) || !finite(a) || !finite(b) || !finite(c) || !finite(discriminant) {
		return [4]float64{math.NaN(), math.NaN(), math.NaN(), math.NaN()}
	}
	include := func(u float64, slot int) {
		if u > 0 && u < 1 {
			times[slot] = u
		}
	}
	if a == 0 {
		if b != 0 {
			include(-c/b, 1)
		}
	} else if discriminant >= 0 {
		numerator := -.5 * (b + math.Copysign(math.Sqrt(discriminant), b))
		if numerator == 0 {
			include(-b/(2*a), 1)
		} else {
			include(numerator/a, 1)
			include(c/numerator, 2)
		}
	}
	sort.Float64s(times[:])
	return times
}

// hermiteMaxAbsVelocity bounds the derivative over the entire segment, preventing
// admission based solely on knot velocities when the interior overshoots.
func hermiteMaxAbsVelocity(delta, dt, v0, v1 float64) float64 {
	c2 := float64(3*delta) - float64(dt*(float64(2*v0)+v1))
	c3 := float64(-2*delta) + float64(dt*(v0+v1))
	if dt <= 0 || math.IsNaN(dt) || math.IsInf(dt, 0) || math.IsNaN(c2) || math.IsInf(c2, 0) || math.IsNaN(c3) || math.IsInf(c3, 0) {
		return math.NaN()
	}
	peak := math.Max(math.Abs(v0), math.Abs(v1))
	if c3 != 0 {
		u := -c2 / (3 * c3)
		if u > 0 && u < 1 {
			peak = math.Max(peak, math.Abs(v0+float64(float64(u*(float64(2*c2)+float64(float64(u*3)*c3)))/dt)))
		}
	}
	return peak
}
