//go:build linux

package denseexec

import (
	"math"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// armMoveJerk is the the 1430 cell machine's jog_jerk, 150 rad/s^3 (unverified,
// issue #158), which OLP one-off moves share with native jog and Halt ramps.
const armMoveJerk = 150.0

// The the 1430 cell Move-to of 2026-10-06 21:04 at fraction 0.99: J1 and J6 cruising
// at -59.4 deg/s (planned 80 x 0.99 x 0.75), J3 -27, J5 +9.5, J4 +2.9 deg/s,
// 517 ms as an acceleration-limited trapezoid. Its one-cycle acceleration
// steps rang J4 (commanded nearly still) to 1.60 degrees of following error.
var move2104 = []JointMoveAxis{
	{Axis: 0, Degrees: -25.8}, {Axis: 2, Degrees: -11.9}, {Axis: 3, Degrees: 1.26},
	{Axis: 4, Degrees: 4.1}, {Axis: 5, Degrees: -25.7},
}

// commandedMove plans a move from zero and returns, per moving axis, the
// positions the native path commands every cycle from the uploaded knots.
func commandedMove(t *testing.T, axes []JointMoveAxis, jerk float64) ([]control.Point, [][]float64) {
	t.Helper()
	d := armMoveDescription()
	points, corners, err := jointMovesProfile(d, axes, 0.99, make([]float64, len(axes)), jerk)
	if err != nil {
		t.Fatal(err)
	}
	knots := knotPoints(points, time.Millisecond, corners...)
	if len(knots) >= len(points)/3 {
		t.Fatalf("%d knots for %d samples: corner knots must stay local", len(knots), len(points))
	}
	commanded := make([][]float64, len(axes))
	for i, move := range axes {
		commanded[i] = hermiteSamples(knots, time.Millisecond, move.Axis)
		if len(commanded[i]) != len(points) {
			t.Fatalf("axis %d: %d commanded cycles, %d planned", move.Axis, len(commanded[i]), len(points))
		}
		for k, p := range points {
			// Absolute FP allowance: 1e-9 rad.
			if math.Abs(commanded[i][k]-p.Position[move.Axis]) > 1e-9 {
				t.Fatalf("axis %d cycle %d commanded %.12f rad, planned %.12f", move.Axis, k, commanded[i][k], p.Position[move.Axis])
			}
		}
	}
	return points, commanded
}

// derivativePeaks returns the largest |velocity|, |acceleration| and |jerk|
// of per-cycle positions (first, second and third differences at 1 ms).
func derivativePeaks(q []float64) (velocity, acceleration, jerk float64) {
	const dt = 0.001
	for k := 1; k < len(q); k++ {
		velocity = math.Max(velocity, math.Abs(q[k]-q[k-1])/dt)
		if k >= 2 {
			acceleration = math.Max(acceleration, math.Abs(q[k]-2*q[k-1]+q[k-2])/(dt*dt))
		}
		if k >= 3 {
			jerk = math.Max(jerk, math.Abs(q[k]-3*q[k-1]+3*q[k-2]-q[k-3])/(dt*dt*dt))
		}
	}
	return velocity, acceleration, jerk
}

// Under the the 1430 cell limits and jerk every commanded joint stays within the
// planned acceleration (0.99 x 4*pi = 12.44 rad/s^2) and 150 rad/s^3, starts
// and ends at rest with near-zero acceleration, lands exactly on its target
// at the final sample and not before (all joints share one clock).
func TestJointMovesProfileJerkLimited(t *testing.T) {
	planned := 0.99 * 4 * math.Pi
	cruise := 1.3962634015954636 * 0.99 * jointMoveVelocityReserve
	for name, axes := range map[string][]JointMoveAxis{
		"2104":     move2104,
		"short_j6": {{Axis: 5, Degrees: 2}},
		"six":      {{Axis: 0, Degrees: 90}, {Axis: 1, Degrees: -40}, {Axis: 2, Degrees: 15}, {Axis: 3, Degrees: -5}, {Axis: 4, Degrees: 60}, {Axis: 5, Degrees: -120}},
		// Two phases to cruise speed need 9.88 degrees: no cruise, and the
		// peak acceleration is not reached either.
		"near_cruise": {{Axis: 1, Degrees: 9.6}},
	} {
		points, commanded := commandedMove(t, axes, armMoveJerk)
		last := len(points) - 1
		for i, move := range axes {
			q := commanded[i]
			target := move.Degrees * math.Pi / 180
			if q[0] != 0 || q[last] != target {
				t.Fatalf("%s axis %d: ends %g..%g rad, want 0..%g exactly", name, move.Axis, q[0], q[last], target)
			}
			if q[last-1] == target {
				t.Fatalf("%s axis %d reached its target before the shared final sample", name, move.Axis)
			}
			velocity, acceleration, jerk := derivativePeaks(q)
			// Absolute allowances: 1e-6 rad/s, 1e-3 rad/s^2 and 0.01 rad/s^3
			// for floating-point rounding in differences of ~1 rad positions.
			if velocity > cruise+1e-6 || acceleration > planned+1e-3 || jerk > armMoveJerk+0.01 {
				t.Fatalf("%s axis %d peaks %g rad/s, %g rad/s^2 (%g deg/s^2), %g rad/s^3; limits %g, %g, %g",
					name, move.Axis, velocity, acceleration, acceleration*180/math.Pi, jerk, cruise, planned, armMoveJerk)
			}
			// At rest at both ends: zero knot velocity, and the first and last
			// cycles' acceleration under 0.5 rad/s^2 (a trapezoid starts at
			// the full 12.4 rad/s^2).
			if points[0].Velocity[move.Axis] != 0 || points[last].Velocity[move.Axis] != 0 {
				t.Fatalf("%s axis %d end velocities %g, %g", name, move.Axis, points[0].Velocity[move.Axis], points[last].Velocity[move.Axis])
			}
			start, end := (q[2]-2*q[1]+q[0])/1e-6, (q[last]-2*q[last-1]+q[last-2])/1e-6
			if math.Abs(start) > 0.5 || math.Abs(end) > 0.5 {
				t.Fatalf("%s axis %d first/last cycle acceleration %g, %g rad/s^2", name, move.Axis, start, end)
			}
		}
	}
}

// The 21:04 move's shape: the trapezoid it replaces commanded an acceleration
// step (jerk far above 1000 rad/s^3 in one cycle) and lasted 517 ms; the
// jerk-limited move reaches the same 12.44 rad/s^2 within the 150 rad/s^3
// bound and lasts a/j = 83 ms longer (J1 governs: 25.8 degrees at 59.4 deg/s).
func TestJointMovesProfileJerkRegression2104(t *testing.T) {
	trapezoid, before := commandedMove(t, move2104, 0)
	scurve, after := commandedMove(t, move2104, armMoveJerk)
	if got := time.Duration(len(trapezoid)-1) * time.Millisecond; got < 517*time.Millisecond || got > 519*time.Millisecond {
		t.Fatalf("trapezoid lasts %v, want the 517-519 ms of the 21:04 move", got)
	}
	added := time.Duration(len(scurve)-len(trapezoid)) * time.Millisecond
	if added < 82*time.Millisecond || added > 85*time.Millisecond {
		t.Fatalf("jerk limit adds %v, want 82-85 ms (a/j = 12.44/150 s)", added)
	}
	_, accelerationBefore, jerkBefore := derivativePeaks(before[0])
	_, accelerationAfter, jerkAfter := derivativePeaks(after[0])
	if jerkBefore < 1000 {
		t.Fatalf("trapezoid J1 peak jerk %g rad/s^3: the regression expects an acceleration step", jerkBefore)
	}
	// The jerk limit must not cost the move its acceleration: J1 still
	// reaches at least 12.3 rad/s^2 of the planned 12.44.
	if jerkAfter > armMoveJerk+0.01 || accelerationAfter < 12.3 || accelerationAfter > accelerationBefore+1e-3 {
		t.Fatalf("J1 after: %g rad/s^2, %g rad/s^3 (before %g rad/s^2)", accelerationAfter, jerkAfter, accelerationBefore)
	}
}

// Jerk is one finite, non-negative joint-unit value; anything else is refused
// as input rather than silently planning an unbounded profile.
func TestJointMovesProfileRefusesInvalidJerk(t *testing.T) {
	d := armMoveDescription()
	for _, jerk := range []float64{-1, math.NaN(), math.Inf(1)} {
		if _, _, err := jointMovesProfile(d, []JointMoveAxis{{Axis: 0, Degrees: 10}}, 0.5, []float64{0}, jerk); err == nil {
			t.Fatalf("jerk %g admitted", jerk)
		}
	}
}
