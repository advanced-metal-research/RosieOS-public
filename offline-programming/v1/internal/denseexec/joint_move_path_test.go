//go:build linux

package denseexec

import (
	"math"
	"testing"
	"time"
)

// A Cartesian step drives every joint by one quintic progress clock along the
// resolver's joint waypoints, so the planned samples lie on the waypoint
// spline. This measures what the native cubic between the uploaded 20 ms knots
// commands: a straight step (two waypoints) stays on its joint segment within
// 1e-9 rad; a curved one (101 waypoints on a quarter circle of 0.3 rad in
// J1/J2, the shape a straight TCP line takes in joint space) stays within
// cartesianKnotDeviation of the planned samples at every cycle.
func TestCartesianMoveProfileStaysOnItsPath(t *testing.T) {
	d := armMoveDescription()
	req := CartesianResolveRequest{Pose: [6]float64{.2}, Lower: [6]float64{-2, -2, -2, -2, -2, -2}, Upper: [6]float64{2, 2, 2, 2, 2, 2}, Fraction: 1}
	curve := [][6]float64{}
	for k := 0; k <= 100; k++ {
		theta := float64(k) / 100 * math.Pi / 2
		curve = append(curve, [6]float64{.2 + .3*math.Sin(theta), .3 * (1 - math.Cos(theta)), .05 * float64(k) / 100})
	}
	for name, waypoints := range map[string][][6]float64{
		"straight": {req.Pose, {.3, -.08, .05}},
		"curved":   curve,
	} {
		result := CartesianResolution{Accepted: true, JointLimitScale: 1, SingularityScale: 1, Waypoints: waypoints}
		points, err := cartesianMoveProfile(d, [6]int{0, 1, 2, 3, 4, 5}, req, result, armMoveJerk)
		if err != nil {
			t.Fatal(err)
		}
		knots := knotPoints(points, time.Millisecond)
		var commanded [6][]float64
		for j := range commanded {
			commanded[j] = hermiteSamples(knots, time.Millisecond, j)
		}
		var worst float64
		first, last := waypoints[0], waypoints[len(waypoints)-1]
		for k := range points {
			for j := 0; j < 6; j++ {
				if name == "straight" {
					// On the segment: the same progress on every moving joint.
					s := (commanded[0][k] - first[0]) / (last[0] - first[0])
					worst = math.Max(worst, math.Abs(commanded[j][k]-first[j]-(last[j]-first[j])*s))
				} else {
					worst = math.Max(worst, math.Abs(commanded[j][k]-points[k].Position[j]))
				}
			}
		}
		bound := 1e-9
		if name == "curved" {
			bound = cartesianKnotDeviation
		}
		t.Logf("%s: %d samples, %d knots, worst deviation %.3g rad", name, len(points), len(knots), worst)
		if worst > bound {
			t.Fatalf("%s: commanded joints leave the planned path by %.3g rad (bound %.3g)", name, worst, bound)
		}
	}
}

// Measured 1.85e-6 rad (about 2 micrometres at 1 m reach) for the curved case
// below on example-relay, 2026-10-06; 2e-6 rad is that rounded up. Unverified as a
// bound for other curves: it grows with waypoint-spline curvature per knot.
const cartesianKnotDeviation = 2e-6

// Owner rule (2026-10-06): a joint Move-to never leaves the straight segment
// in joint space; when one joint is limited, every joint keeps its ratio.
// Every commanded cycle (the native cubic between uploaded knots) must be
// q0 + dq*s for one progress s shared by all joints, within 1e-9 rad of
// floating-point rounding. Native count quantization (at most half a count
// per joint) is applied after this and is not part of the bound.
func TestJointMovesProfileStaysOnTheJointSegment(t *testing.T) {
	uneven := armMoveDescription()
	uneven.Axes[1].MaxVelocity = 0.5 // J2 alone would cruise slower than J1
	low := 2.0
	uneven.Axes[4].MaxAcceleration = &low // J5 alone would ramp longer
	for _, c := range []struct {
		name string
		axes []JointMoveAxis
		jerk float64
		lim  func(axis int) (velocity, acceleration float64)
	}{
		{"2104", move2104, armMoveJerk, nil},
		{"2104_acceleration_only", move2104, 0, nil},
		{"short_and_long", []JointMoveAxis{{Axis: 0, Degrees: 90}, {Axis: 5, Degrees: 2}}, armMoveJerk, nil},
		{"six", []JointMoveAxis{{Axis: 0, Degrees: 90}, {Axis: 1, Degrees: -40}, {Axis: 2, Degrees: 15}, {Axis: 3, Degrees: -5}, {Axis: 4, Degrees: 60}, {Axis: 5, Degrees: -120}}, armMoveJerk, nil},
		{"uneven_limits", []JointMoveAxis{{Axis: 0, Degrees: 40}, {Axis: 1, Degrees: 30}, {Axis: 4, Degrees: -20}}, armMoveJerk,
			func(axis int) (float64, float64) {
				return uneven.Axes[axis].MaxVelocity, math.Min(uneven.Axes[axis].JogAcceleration, *uneven.Axes[axis].MaxAcceleration)
			}},
	} {
		t.Run(c.name, func(t *testing.T) {
			d := armMoveDescription()
			if c.lim != nil {
				d = uneven
			}
			origins := make([]float64, len(c.axes))
			delta := make([]float64, len(c.axes))
			var norm float64
			for i, move := range c.axes {
				origins[i] = 0.1 * float64(i+1)
				delta[i] = move.Degrees * math.Pi / 180
				norm += delta[i] * delta[i]
			}
			points, corners, err := jointMovesProfile(d, c.axes, 0.99, origins, c.jerk)
			if err != nil {
				t.Fatal(err)
			}
			knots := knotPoints(points, time.Millisecond, corners...)
			commanded := make([][]float64, len(c.axes))
			for i, move := range c.axes {
				commanded[i] = hermiteSamples(knots, time.Millisecond, move.Axis)
			}
			var worst, previous float64
			for k := range commanded[0] {
				var s float64
				for i := range c.axes {
					s += (commanded[i][k] - origins[i]) * delta[i]
				}
				s /= norm
				for i := range c.axes {
					worst = math.Max(worst, math.Abs(commanded[i][k]-origins[i]-delta[i]*s))
				}
				if s < previous-1e-12 || s < -1e-12 || s > 1+1e-12 {
					t.Fatalf("cycle %d: progress %g after %g; it must rise from 0 to 1", k, s, previous)
				}
				previous = s
			}
			if worst > 1e-9 {
				t.Fatalf("commanded joint vector leaves the straight joint segment by %.3g rad (%.3g deg)", worst, worst*180/math.Pi)
			}
			if math.Abs(previous-1) > 1e-12 {
				t.Fatalf("progress ends at %.15g", previous)
			}
			// Every joint stays within its own limits at the fraction and the
			// one machine jerk. Allowances: 1e-6 rad/s, 1e-3 rad/s^2, 0.01 rad/s^3.
			for i, move := range c.axes {
				velocityLimit, accelerationLimit := d.Axes[move.Axis].MaxVelocity, math.Min(d.Axes[move.Axis].JogAcceleration, *d.Axes[move.Axis].MaxAcceleration)
				if c.lim != nil {
					velocityLimit, accelerationLimit = c.lim(move.Axis)
				}
				velocity, acceleration, jerk := derivativePeaks(commanded[i])
				if velocity > velocityLimit*0.99*jointMoveVelocityReserve+1e-6 || acceleration > accelerationLimit*0.99+1e-3 ||
					(c.jerk > 0 && jerk > c.jerk+0.01) {
					t.Fatalf("axis %d peaks %g rad/s, %g rad/s^2, %g rad/s^3", move.Axis, velocity, acceleration, jerk)
				}
			}
		})
	}
}
