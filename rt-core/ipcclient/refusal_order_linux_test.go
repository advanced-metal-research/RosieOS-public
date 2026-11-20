//go:build linux

package ipcclient_test

import (
	"context"
	"errors"
	"fmt"
	"os"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

// Independent point and curve violations must report the earliest ending sample
// and native axis, preventing adapter/native validation passes from disagreeing.
func TestReferenceRefusalOrderParity(t *testing.T) {
	for _, name := range []string{"initial_position", "position_before_later_position", "velocity_before_later_position", "mapped_position_tie", "mapped_velocity_tie", "curve_before_bad_point", "curve_before_later_recovery"} {
		t.Run(name, func(t *testing.T) {
			c, logPath := hermiteAdmissionCore(t, 1, .1)
			description, err := c.Describe()
			if err != nil {
				t.Fatal(err)
			}
			samples := make([]densejointtraj.Sample, 5)
			for i := range samples {
				samples[i].TimeS = float64(i)
			}
			// Exact stationary endpoints and interior knots: each position bow
			// reaches 0.5025 rad against the declared 0.5 rad position bound.
			positionBow := func(axis, start int) {
				for i := range samples {
					samples[i].QRad[axis] = .465
				}
				samples[start].QRad[axis], samples[start+1].QRad[axis] = .49, .49
				samples[start].QdRadS[axis], samples[start+1].QdRadS[axis] = .05, -.05
			}
			// A zero-qd 0.08 rad step peaks at 0.12 rad/s, above 0.1 rad/s.
			velocityBow := func(axis int) {
				for i := 1; i < len(samples); i++ {
					samples[i].QRad[axis] = .08
				}
			}
			wantSample, wantAxis, wantKind := 2, 2, "position"
			switch name {
			case "initial_position":
				// 2147483648 counts cannot be represented, even as an exterior
				// hold; the later J1 interval also exceeds the velocity bound.
				for i := range samples {
					samples[i].QRad[2] = 2147483648 / description.Axes[2].CountsPerUnit
				}
				velocityBow(0)
				wantSample = 0
			case "position_before_later_position":
				positionBow(2, 1)
				positionBow(0, 2)
			case "velocity_before_later_position":
				velocityBow(2)
				positionBow(0, 2)
				wantSample, wantKind = 1, "velocity"
			case "mapped_position_tie":
				positionBow(0, 1)
				positionBow(2, 1)
				wantAxis = 0
			case "mapped_velocity_tie":
				velocityBow(0)
				velocityBow(2)
				wantSample, wantAxis, wantKind = 1, 0, "velocity"
			case "curve_before_bad_point":
				velocityBow(2)
				for i := range samples {
					samples[i].QRad[0] = .49
				}
				samples[3].QRad[0], samples[4].QRad[0] = .51, .51
				wantSample, wantKind = 1, "velocity"
			case "curve_before_later_recovery":
				velocityBow(2)
				for i := range samples {
					samples[i].QRad[0] = .6
				}
				samples[3].QRad[0], samples[4].QRad[0] = .61, .61
				wantSample, wantKind = 1, "velocity"
			}
			points := make([]ipcclient.Point, len(samples))
			for i, sample := range samples {
				points[i] = ipcclient.Point{Time: time.Duration(i) * time.Second,
					Position: append([]float64(nil), sample.QRad[:]...), Velocity: append([]float64(nil), sample.QdRadS[:]...)}
			}
			ids := []string{"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
			if strings.HasPrefix(name, "mapped_") {
				ids[0], ids[2] = ids[2], ids[0]
				for i := range samples {
					samples[i].QRad[0], samples[i].QRad[2] = samples[i].QRad[2], samples[i].QRad[0]
					samples[i].QdRadS[0], samples[i].QdRadS[2] = samples[i].QdRadS[2], samples[i].QdRadS[0]
				}
			}
			_, adapterErr := control.DecodeProgram(hermiteProgram(t, samples, 1, .1), description, ids)
			axisName := fmt.Sprintf("J%d", wantAxis+1)
			if wantKind == "position" {
				var position *control.SegmentPositionError
				if !errors.As(adapterErr, &position) || position.Segment != 0 || position.Sample != wantSample || position.Axis != axisName {
					t.Fatalf("first position refusal: %+v (%v)", position, adapterErr)
				}
				if wantSample == 0 {
					if position.Position*description.Axes[2].CountsPerUnit != 2147483648 || position.Limit*description.Axes[2].CountsPerUnit != 2147483647 {
						t.Fatalf("CSP refusal must report 2147483648 counts against 2147483647 counts: %+v", position)
					}
				} else if position.Limit != .5 || position.Position < .502499999999 || position.Position > .502500000001 {
					t.Fatalf("position bow must reach 0.5025 rad against 0.5 rad: %+v", position)
				}
			} else {
				var velocity *control.SegmentVelocityError
				if !errors.As(adapterErr, &velocity) || velocity.Segment != 0 || velocity.Sample != wantSample || velocity.Axis != axisName || velocity.Limit != .1 || velocity.Peak < .12 || velocity.Peak > .120000000001 {
					t.Fatalf("first velocity refusal: %+v (%v)", velocity, adapterErr)
				}
			}
			_, nativeErr := c.PrepareTrajectory(context.Background(), 8901, 511, points)
			var native *ipcclient.RejectedError
			if name == "initial_position" || name == "curve_before_bad_point" {
				pointIndex := uint32(0)
				if name == "curve_before_bad_point" {
					pointIndex = 3
				}
				var point *ipcclient.TrajectoryPointError
				if !errors.As(nativeErr, &point) || point.PointIndex != pointIndex {
					t.Fatalf("missing refusing upload point %d: %v", pointIndex, nativeErr)
				}
			}
			if !errors.As(nativeErr, &native) || native.Result.Reason != 6 || native.Result.AxisMask != 1<<wantAxis {
				t.Fatalf("first native refusal: %+v (%v)", native, nativeErr)
			}
			raw, err := os.ReadFile(logPath)
			if err != nil {
				t.Fatal(err)
			}
			diagnostic := "point_limit"
			if wantSample > 0 {
				diagnostic = "hermite_" + wantKind + "_extremum"
			}
			found := false
			for _, line := range strings.Split(string(raw), "\n") {
				if strings.Contains(line, fmt.Sprintf("%s axis=%d (%s)", diagnostic, wantAxis, axisName)) &&
					strings.HasSuffix(line, fmt.Sprintf("sample=%d", wantSample)) {
					found = true
				}
			}
			if !found {
				t.Fatalf("missing first-refusal diagnostic: %s", raw)
			}
		})
	}
}
