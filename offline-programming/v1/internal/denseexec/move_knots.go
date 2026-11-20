package denseexec

import (
	"math"
	"time"

	"rosieos/rt-core/sdk/control"
)

// moveKnotPeriod is the spacing of the trajectory knots uploaded for a bounded
// joint or Cartesian step. The native path interpolates each segment as a cubic
// Hermite when both knots supply a velocity (Describe interpolation
// "hermite_position_with_velocity"), so the controller keeps its per-cycle
// profile for the velocity and acceleration gates but uploads only every 20th
// sample with its velocity. Measured 2026-09-16 on the LA bench over the
// Sydney relay (240 ms RTT): a 100 mm step at 5 % sampled at the 1 ms cycle
// was 16,556 points, about 2.5 MB of JSON, and its upload exceeded the
// negotiated request budget (lease + 3 RTT = 3.7 s) often enough that the
// retry met the adapter's busy refusal; at 20 ms knots the same step is 829
// points, about 220 KB, well inside one budget.
const moveKnotPeriod = 20 * time.Millisecond

// moveKnotCornerFraction marks an acceleration corner: a one-cycle change in an
// axis's second difference larger than this fraction of that axis's peak. A
// trapezoid's ramp ends change it by the whole acceleration; the smooth quintic
// in move_knots_test.go changes by under 0.1 percent per cycle. Unverified
// threshold between those two cases, not a measurement.
const moveKnotCornerFraction = 0.05

// moveKnotCornerFloor ignores axes whose peak acceleration is below this many
// rad/s^2, so an axis carrying only IK rounding cannot turn every sample into a
// knot. Unverified: 0.001 rad/s^2 is far below any commanded joint acceleration.
const moveKnotCornerFloor = 0.001

// knotPoints reduces a per-cycle profile to Hermite knots: every knot on the
// period plus the final sample, each carrying the sample's own velocity when the
// profile supplies one and otherwise the central-difference velocity of the
// dense profile, zero at both ends so motion starts and arrives at rest.
// Fewer than three samples, or a profile that is not uniformly spaced from
// time zero, is returned unchanged.
//
// A cubic segment that straddles an acceleration corner cannot follow it: its
// acceleration is linear in time, and matching the corner's endpoint positions
// and velocities drives it up to 4/3 of the planned acceleration when the
// corner lies a third of the way in. A Rosie 1430 arm's one-off move of 2026-10-06 was
// planned at no more than the 720 deg/s^2 joint limit and commanded at up to
// 912 deg/s^2 (J6) at its start and stop. Every sample within one cycle of a corner is therefore a
// knot: one-cycle segments reproduce the dense samples exactly, and every
// longer segment lies inside one constant-acceleration (quadratic) piece,
// which a cubic Hermite with exact end velocities reproduces exactly.
//
// A jerk-limited profile has no acceleration corner for the detector to find,
// but its jerk corners matter the same way, and a central difference is not
// exact on its cubic pieces. Such a planner supplies exact sample velocities
// and names the samples either side of each of its corners; those are knots
// too, so every longer segment lies inside one cubic piece and is reproduced.
func knotPoints(points []control.Point, cycle time.Duration, corners ...int) []control.Point {
	if len(points) < 3 || cycle <= 0 || points[0].Time != 0 || points[1].Time != cycle {
		return points
	}
	stride := int(moveKnotPeriod / cycle)
	if stride < 2 {
		return points
	}
	last := len(points) - 1
	keep := make([]bool, len(points))
	for k := 0; k <= last; k += stride {
		keep[k] = true
	}
	keep[last] = true
	for _, k := range corners {
		keep[min(max(k, 0), last)] = true
	}
	for _, corner := range accelerationCorners(points, cycle) {
		for k := max(corner-1, 0); k <= min(corner+1, last); k++ {
			keep[k] = true
		}
	}
	knots := make([]control.Point, 0, last/stride+2)
	for k, kept := range keep {
		if kept {
			knots = append(knots, knotAt(points, k, cycle))
		}
	}
	return knots
}

// accelerationCorners returns the samples whose second differences a corner
// can mix. A corner between two samples mixes at most two consecutive second
// differences, and the jump that crosses moveKnotCornerFraction of the axis's
// peak can be any of the three around them, so two samples either side of that
// jump are returned.
func accelerationCorners(points []control.Point, cycle time.Duration) []int {
	last := len(points) - 1
	axes := len(points[0].Position)
	second := func(k, axis int) float64 {
		return points[k+1].Position[axis] - 2*points[k].Position[axis] + points[k-1].Position[axis]
	}
	floor := moveKnotCornerFloor * cycle.Seconds() * cycle.Seconds()
	peaks := make([]float64, axes)
	for k := 1; k < last; k++ {
		for axis := range peaks {
			peaks[axis] = math.Max(peaks[axis], math.Abs(second(k, axis)))
		}
	}
	var corners []int
	for k := 2; k < last; k++ {
		for axis, peak := range peaks {
			if peak > floor && math.Abs(second(k, axis)-second(k-1, axis)) > moveKnotCornerFraction*peak {
				corners = append(corners, k-2, k-1, k, k+1)
				break
			}
		}
	}
	return corners
}

func knotAt(points []control.Point, k int, cycle time.Duration) control.Point {
	p := points[k]
	velocity := make([]float64, len(p.Position))
	if k > 0 && k < len(points)-1 && len(p.Velocity) == len(p.Position) {
		copy(velocity, p.Velocity)
	} else if k > 0 && k < len(points)-1 {
		seconds := 2 * cycle.Seconds()
		for i := range velocity {
			velocity[i] = (points[k+1].Position[i] - points[k-1].Position[i]) / seconds
		}
	}
	return control.Point{Time: p.Time, Position: append([]float64(nil), p.Position...), Velocity: velocity}
}
