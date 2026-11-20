package denseexec

import (
	"math"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// A 16,556-sample step (the bench's 100 mm at 5 %) uploads as 829 knots whose
// times start at zero and increase, whose positions are the dense samples,
// whose end velocities are zero and whose interior velocities match the dense
// slope to within one cycle of acceleration.
func TestKnotPointsReduceDenseProfile(t *testing.T) {
	const cycle = time.Millisecond
	dense := make([]control.Point, 16556)
	for k := range dense {
		u := float64(k) / float64(len(dense)-1)
		s := 10*u*u*u - 15*u*u*u*u + 6*u*u*u*u*u
		dense[k] = control.Point{Time: time.Duration(k) * cycle, Position: []float64{0.1 * s, -0.2 * s, 0, 0, 0, 0}}
	}
	knots := knotPoints(dense, cycle)
	if len(knots) != 829 {
		t.Fatalf("knots=%d want 829", len(knots))
	}
	if knots[0].Time != 0 || knots[len(knots)-1].Time != dense[len(dense)-1].Time {
		t.Fatalf("knot times %v..%v", knots[0].Time, knots[len(knots)-1].Time)
	}
	for i := 1; i < len(knots); i++ {
		if knots[i].Time <= knots[i-1].Time {
			t.Fatalf("knot %d time %v not after %v", i, knots[i].Time, knots[i-1].Time)
		}
	}
	for _, end := range []control.Point{knots[0], knots[len(knots)-1]} {
		for _, v := range end.Velocity {
			if v != 0 {
				t.Fatalf("end velocity %v want 0", end.Velocity)
			}
		}
	}
	if knots[len(knots)-1].Position[0] != 0.1 || knots[len(knots)-1].Position[1] != -0.2 {
		t.Fatalf("final knot %v", knots[len(knots)-1].Position)
	}
	// Peak quintic slope is 1.875 × (0.1 rad / 16.555 s) = 0.01133 rad/s; the
	// mid knot must carry it.
	mid := knots[len(knots)/2]
	if math.Abs(mid.Velocity[0]-0.011325) > 0.0001 || math.Abs(mid.Velocity[1]+0.02265) > 0.0002 {
		t.Fatalf("mid velocity %v", mid.Velocity)
	}
	if len(mid.Velocity) != 6 || len(mid.Position) != 6 {
		t.Fatalf("knot arity %d/%d", len(mid.Position), len(mid.Velocity))
	}
}

// hermiteSamples evaluates uploaded knots at every cycle exactly as the native
// path does (engine/trajectory.hpp: a cubic Hermite per segment when both
// knots carry velocity).
func hermiteSamples(knots []control.Point, cycle time.Duration, axis int) []float64 {
	var out []float64
	for i := 0; i+1 < len(knots); i++ {
		a, b := knots[i], knots[i+1]
		h := (b.Time - a.Time).Seconds()
		for t := a.Time; t < b.Time; t += cycle {
			s := (t - a.Time).Seconds() / h
			h00, h10, h01, h11 := 2*s*s*s-3*s*s+1, s*s*s-2*s*s+s, -2*s*s*s+3*s*s, s*s*s-s*s
			out = append(out, h00*a.Position[axis]+h10*h*a.Velocity[axis]+h01*b.Position[axis]+h11*h*b.Velocity[axis])
		}
	}
	return append(out, knots[len(knots)-1].Position[axis])
}

func peakAcceleration(samples []float64, cycle time.Duration) float64 {
	var peak float64
	dt := cycle.Seconds()
	for k := 1; k+1 < len(samples); k++ {
		peak = math.Max(peak, math.Abs(samples[k+1]-2*samples[k]+samples[k-1])/(dt*dt))
	}
	return peak
}

// The the 1430 cell one-off move of 2026-10-06: J1/J5/J6 at fraction 0.99 under the
// description's 4*pi rad/s^2 (720 deg/s^2) and 80 deg/s limits, cruising at
// 59.4 deg/s. Knots on the 20 ms grid alone straddled the trapezoid's corners
// and the native cubic segments commanded up to 4/3 of the planned
// acceleration (908-912 deg/s^2 measured). With corner knots every commanded
// cycle equals the planned sample and acceleration stays at the limit.
func TestKnotPointsKeepPlannedAcceleration(t *testing.T) {
	d := armMoveDescription()
	planned := 0.99 * 4 * math.Pi
	// Every ramp lasts v/a = 1.396 x 0.75 / (4*pi) = 83.33 ms at any fraction.
	// A single-axis cruise of 3.33 ms (+20k) starts the deceleration 6.67 ms
	// into a 20 ms segment, where a cubic through the corner peaks at 4/3 of
	// the plan; the single-axis moves below pin that worst case.
	cruise := 0.99 * 1.3962634015954636 * jointMoveVelocityReserve
	var gridPeak float64
	for _, axes := range [][]JointMoveAxis{
		{{Axis: 0, Degrees: 42.3}, {Axis: 4, Degrees: -39.1}, {Axis: 5, Degrees: 42.7}},
		{{Axis: 5, Degrees: cruise * (0.08333333 + 0.00333333) * 180 / math.Pi}},
		{{Axis: 5, Degrees: cruise * (0.08333333 + 0.02333333) * 180 / math.Pi}},
	} {
		gridPeak = math.Max(gridPeak, checkKnotAcceleration(t, d, axes, planned))
	}
	if gridPeak < 1.25*planned {
		t.Fatalf("grid-only knots peak at %g rad/s^2; the regression expects above %g", gridPeak, 1.25*planned)
	}
}

// armMoveDescription is the the 1430 cell description's joint limits on six
// joints: 80 deg/s, 4*pi rad/s^2 (720 deg/s^2), 1 ms cycle.
func armMoveDescription() control.Description {
	accel := 4 * math.Pi
	var d control.Description
	d.CycleNS, d.MaxTrajectoryPoints = uint64(time.Millisecond), 100000
	d.Axes = make([]ipcclient.AxisDescription, 6)
	for i := range d.Axes {
		d.Axes[i] = ipcclient.AxisDescription{PositionUnit: "rad", MinPosition: -3.1, MaxPosition: 3.1, MaxVelocity: 1.3962634015954636,
			JogAcceleration: accel, MaxAcceleration: &accel, CompletionTolerance: 0.0006, CompletionTimeoutNS: 1000000000}
	}
	return d
}

// checkKnotAcceleration asserts the uploaded knots reproduce the planned
// samples and acceleration, and returns the peak the grid-only knots this
// change replaces would have commanded.
func checkKnotAcceleration(t *testing.T, d control.Description, axes []JointMoveAxis, planned float64) float64 {
	t.Helper()
	const cycle = time.Millisecond
	origins := make([]float64, len(axes))
	points, corners, err := jointMovesProfile(d, axes, 0.99, origins, 0)
	if err != nil {
		t.Fatal(err)
	}
	knots := knotPoints(points, cycle, corners...)
	if len(knots) >= len(points)/4 {
		t.Fatalf("%d knots for %d samples: corner knots must stay local", len(knots), len(points))
	}
	var grid []control.Point
	for k := 0; k < len(points); k += int(moveKnotPeriod / cycle) {
		grid = append(grid, knotAt(points, k, cycle))
	}
	grid = append(grid, knotAt(points, len(points)-1, cycle))
	var gridPeak float64
	for _, move := range axes {
		gridPeak = math.Max(gridPeak, peakAcceleration(hermiteSamples(grid, cycle, move.Axis), cycle))
		dense := make([]float64, len(points))
		for k, p := range points {
			dense[k] = p.Position[move.Axis]
		}
		if got := peakAcceleration(dense, cycle); got > planned*(1+1e-6) {
			t.Fatalf("axis %d planned profile peaks at %g rad/s^2, above %g", move.Axis, got, planned)
		}
		commanded := hermiteSamples(knots, cycle, move.Axis)
		if len(commanded) != len(dense) {
			t.Fatalf("axis %d: %d commanded cycles, %d planned", move.Axis, len(commanded), len(dense))
		}
		for k := range dense {
			// Absolute FP allowance: 1e-9 rad.
			if math.Abs(commanded[k]-dense[k]) > 1e-9 {
				t.Fatalf("axis %d cycle %d commanded %.12f rad, planned %.12f", move.Axis, k, commanded[k], dense[k])
			}
		}
		// Absolute allowance: 1e-3 rad/s^2 for rounding in second differences.
		if got := peakAcceleration(commanded, cycle); got > planned+1e-3 {
			t.Fatalf("axis %d commanded %g rad/s^2 (%g deg/s^2) above the planned %g", move.Axis, got, got*180/math.Pi, planned)
		}
	}
	return gridPeak
}

// Short or irregular profiles pass through unchanged so the native path still
// receives a valid two-point trajectory.
func TestKnotPointsPassThroughShortProfiles(t *testing.T) {
	two := []control.Point{{Time: 0, Position: []float64{0}}, {Time: time.Millisecond, Position: []float64{1}}}
	if got := knotPoints(two, time.Millisecond); len(got) != 2 || got[1].Velocity != nil {
		t.Fatalf("short profile changed: %+v", got)
	}
	coarse := []control.Point{{Time: 0, Position: []float64{0}}, {Time: 50 * time.Millisecond, Position: []float64{1}}, {Time: 100 * time.Millisecond, Position: []float64{2}}}
	if got := knotPoints(coarse, time.Millisecond); len(got) != 3 || got[1].Velocity != nil {
		t.Fatalf("coarse profile changed: %+v", got)
	}
}
