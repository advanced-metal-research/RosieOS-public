//go:build linux

package main

import (
	"context"
	"encoding/json"
	"io"
	"math"
	"reflect"
	"testing"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

func TestParseMoveSpecs(t *testing.T) {
	for _, tc := range []struct {
		text string
		want []moveSpec
	}{
		{"0:0.2,8:-0.3", []moveSpec{{0, .2}, {8, -.3}}},
		{" 15 : -3 , 1 : +3 ", []moveSpec{{15, -3}, {1, 3}}},
		{"0:12.549", []moveSpec{{0, 12.549}}},
	} {
		got, err := parseMoveSpecs(tc.text)
		if err != nil || !reflect.DeepEqual(got, tc.want) {
			t.Fatalf("%q: %v %v", tc.text, got, err)
		}
	}
	for _, text := range []string{"", " , ", "0:.1,0:.2", "-1:.1", "16:.1", "a:.1", "0:NaN", "0:Inf", "0:0", "0", "0:.1:.2"} {
		if got, err := parseMoveSpecs(text); err == nil {
			t.Fatalf("admitted %q: %v", text, got)
		}
	}
}

func TestMoveProfileGroupSlowestAxisAndReserve(t *testing.T) {
	d := control.Description{NativeDescription: ipcclient.Description{CycleNS: 1000000, MaxTrajectoryPoints: 32768,
		Axes: []ipcclient.AxisDescription{
			{PositionUnit: "rad", MinPosition: -3, MaxPosition: 3, MaxVelocity: 1, JogAcceleration: 2, CompletionTolerance: .0002},
			{PositionUnit: "rad", MinPosition: -3, MaxPosition: 3, MaxVelocity: .4, JogAcceleration: 1, CompletionTolerance: .0002},
		}}}
	o := options{velocityFraction: .5, accelFraction: .5}
	moves := []moveSpec{{0, .2}, {1, -1.5}}
	origins := map[int]float64{0: .123, 1: .456}
	points, err := moveProfileGroup(d, o, moves, origins)
	simfixture.Must(t, err)
	// The slow axis uses 0.15 rad/s and 0.5 rad/s^2: 300 ms ramps,
	// 9700 ms cruise, 10300 ms total. These are fixed fixture expectations.
	if end := points[len(points)-1]; end.Time != 10300000000 || points[0].Time != 0 ||
		points[0].Position[0] != .123 || points[0].Position[1] != .456 ||
		end.Position[0] != origins[0]+moves[0].delta || end.Position[1] != origins[1]+moves[1].delta {
		t.Fatalf("wrong shared profile geometry: first=%+v last=%+v", points[0], end)
	}
	if moveVelocityReserve != .75 {
		t.Fatalf("velocity reserve=%g, want 0.75", moveVelocityReserve)
	}
	var peak float64
	for i, p := range points {
		if math.Abs((p.Position[0]-.123)/.2-(p.Position[1]-.456)/-1.5) > 1e-12 {
			t.Fatal("axes do not share the normalised curve")
		}
		if i == 0 {
			continue
		}
		if p.Time-points[i-1].Time != 1000000 {
			t.Fatal("sample interval is not 1000000 ns")
		}
		for axis, bound := range []float64{.5, .2} {
			v := math.Abs(p.Position[axis]-points[i-1].Position[axis]) * 1000
			if v > bound {
				t.Fatalf("axis %d velocity %g exceeds %g rad/s", axis, v, bound)
			}
			if axis == 1 {
				peak = math.Max(peak, v)
			}
		}
	}
	if math.Abs(peak-.15) > 1e-12 {
		t.Fatalf("reserved peak=%g, want 0.15 rad/s", peak)
	}
	// Reversing input order must leave the slower second axis in control.
	reordered, err := moveProfileGroup(d, o, []moveSpec{moves[1], moves[0]}, origins)
	simfixture.Must(t, err)
	if !reflect.DeepEqual(points, reordered) {
		t.Fatal("move order changed the profile")
	}
}

func TestStationaryCountTolerance(t *testing.T) {
	r := runner{description: control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{{CompletionTolerance: .001, CountsPerUnit: 10000}}}}}
	origin := control.Status{Core: ipcclient.Status{Axes: []ipcclient.AxisStatus{{PositionCounts: 100}}}}
	for _, tc := range []struct {
		change int32
		motion bool
	}{{3, false}, {-3, false}, {10, false}, {-10, false}, {11, true}, {-11, true}} {
		s := control.Status{Core: ipcclient.Status{Axes: []ipcclient.AxisStatus{{PositionCounts: 100 + tc.change}}}}
		if err := r.stationary(s, origin); (err != nil) != tc.motion {
			t.Fatalf("change=%d counts motion=%t: %v", tc.change, tc.motion, err)
		}
	}
}

func TestSimulationMoveStartsAtHeldTarget(t *testing.T) {
	f := startFixtureMode(t, false, "--sim-feedback-offset-counts", "4")
	f.phase(t, "home")
	records := f.phase(t, "move", "--delta-rad", "0.02", "--accel-fraction", "0.5")
	var held, feedback float64
	seenOffset, prepared, started := false, false, false
	for _, rec := range records {
		if rec.Kind == "status" && !prepared {
			var s control.Status
			simfixture.Must(t, json.Unmarshal(rec.Response, &s))
			if s.Core.Armed == 1 && s.Core.Axes[0].Readiness == "ready" {
				held, feedback = s.Core.Axes[0].LogicalTarget, s.Core.Axes[0].LogicalPosition
				if math.Abs(math.Abs(feedback-held)*131072/(2*math.Pi)-4) < 1e-8 {
					seenOffset = true
				}
			}
		}
		if rec.Operation == "prepare_trajectory" {
			var raw map[string]json.RawMessage
			simfixture.Must(t, json.Unmarshal(rec.Request, &raw))
			var points []struct {
				Position []float64 `json:"position"`
			}
			simfixture.Must(t, json.Unmarshal(raw["points"], &points))
			if !seenOffset || len(points) == 0 || points[0].Position[0] != held || points[0].Position[0] == feedback {
				t.Fatalf("start did not use held target with 4-count feedback offset: %s held=%g feedback=%g", rec.Request, held, feedback)
			}
			prepared = true
		}
		if rec.Operation == "start_trajectory" && rec.Kind == "request" {
			started = rec.Reason == ""
		}
	}
	if !prepared || !started {
		t.Fatal("offset start was not admitted")
	}
}

func TestDirectionalJogLimits(t *testing.T) {
	a := ipcclient.AxisDescription{PositionUnit: "rad", MaxVelocity: .1, JogAcceleration: 1, CountsPerUnit: 20000,
		MaxTargetLead: .02, FollowingError: .04, CompletionTolerance: .0002, MinPosition: -.5, MaxPosition: .5, Sign: 1}
	for _, direction := range []int{-1, 1} {
		r := runner{o: options{axis: 0, direction: direction, fraction: .5, jogNS: 2000000000},
			description: control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{a}}}, expectations: map[string]bool{}}
		for _, position := range []float64{.41 * float64(direction), -.49 * float64(direction)} {
			s := control.Status{Core: ipcclient.Status{Axes: []ipcclient.AxisStatus{{HomeValid: true, LogicalValid: true, LogicalPosition: position, LogicalTarget: position}}}}
			err := r.jogLimits(s)
			if (err != nil) != (position*float64(direction) > 0) {
				t.Fatalf("direction=%d position=%g: %v", direction, position, err)
			}
		}
	}
}

func TestDirectionFlags(t *testing.T) {
	f := fixture{socket: "absent", hash: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa", build: testBuildDir(t)}
	for _, direction := range []string{"-1", "+1", "0"} {
		_, err := parseOptions(append(f.args("jog", "flags", true), "--direction", direction, "--motor-revs-per-axis-rev", "1"), io.Discard)
		simfixture.Must(t, err)
	}
	for _, extra := range [][]string{{"--direction", "2"}, {"--direction", "-2"}, {"--direction", "1"}, {"--motor-revs-per-axis-rev", "NaN"}} {
		if _, err := parseOptions(append(f.args("jog", "flags", true), extra...), io.Discard); err == nil {
			t.Fatalf("accepted flags %v", extra)
		}
	}
	if code := run(context.Background(), f.args("session", "dry-session", false), io.Discard); code != 3 {
		t.Fatalf("unconfirmed session exit=%d", code)
	}
}
