//go:build linux

package ipcclient_test

import (
	"context"
	"errors"
	"fmt"
	"math"
	"os"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

// Real Commit and dense Decode see identical counts and nanosecond clocks.
// Absolute fixtures include a reversal inside the exterior endpoint envelope,
// preventing parity from being established solely by shared range arithmetic.
func TestHermiteRecoveryAdmissionParity(t *testing.T) {
	for _, sign := range []int{-1, 1} {
		t.Run(fmt.Sprintf("sign_%d", sign), func(t *testing.T) {
			c, logPath := hermiteAdmissionCore(t, sign, 5)
			description, err := c.Describe()
			if err != nil {
				t.Fatal(err)
			}
			scale := description.Axes[0].CountsPerUnit
			if math.Round(scale*(2*math.Pi)) != 131072 {
				t.Fatal("fixture must have 131072 counts per revolution")
			}
			ids := []string{"J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"}
			for index, test := range []struct {
				name   string
				q, qd  []float64
				reason uint32
				sample int
			}{
				{"outward_bow", []float64{12000, 11999, 11994}, []float64{0, -10, 0}, 8, 1},
				{"exterior_reversal", []float64{12004, 12000, 11998, 11994}, []float64{0, -8, -8, 0}, 8, 2},
				{"monotone", []float64{12000, 10000}, []float64{0, 0}, 0, 0},
				{"stationary", []float64{12000, 12000}, []float64{0, 0}, 0, 0},
			} {
				t.Run(test.name, func(t *testing.T) {
					samples := make([]densejointtraj.Sample, len(test.q))
					points := make([]ipcclient.Point, len(test.q))
					for i := range samples {
						samples[i].TimeS = float64(i)
						samples[i].QRad[0], samples[i].QdRadS[0] = test.q[i]/scale, test.qd[i]/scale
						points[i] = ipcclient.Point{Time: time.Duration(i) * time.Second, Position: make([]float64, 9), Velocity: make([]float64, 9)}
						points[i].Position[0], points[i].Velocity[0] = samples[i].QRad[0], samples[i].QdRadS[0]
					}
					_, adapterErr := control.DecodeProgram(hermiteProgram(t, samples, 1, 5), description, ids)
					handle, nativeErr := c.PrepareTrajectory(context.Background(), uint64(8700+index), 511, points)
					if test.reason == 0 {
						if adapterErr != nil || nativeErr != nil || handle == 0 {
							t.Fatalf("legal recovery: adapter=%v native=%v handle=%d", adapterErr, nativeErr, handle)
						}
						if _, err := c.DiscardTrajectory(context.Background(), handle); err != nil {
							t.Fatal(err)
						}
						return
					}
					var native *ipcclient.RejectedError
					var adapter *control.SegmentRecoveryError
					if !errors.As(nativeErr, &native) || native.Result.Reason != 8 || native.Result.AxisMask != 1 ||
						!errors.As(adapterErr, &adapter) || adapter.Error() != control.ReasonOutsideLimitsOutward || adapter.Axis != "J1" || adapter.Sample != test.sample {
						t.Fatalf("reason/axis/sample parity: native=%+v adapter=%+v (%v)", native, adapter, adapterErr)
					}
					raw, err := os.ReadFile(logPath)
					if err != nil || !strings.Contains(string(raw), fmt.Sprintf("outside_limits_outward: hermite_recovery_extremum axis=0 (J1) segment=%d", test.sample-1)) {
						t.Fatalf("recovery diagnostic: %v %s", err, raw)
					}
				})
			}
		})
	}
}

// Count-frame sign and Home offset must not change recovery
// classification. The same exact fixtures are asserted by the C++ policy test.
func TestHermiteRecoveryCountFrameParity(t *testing.T) {
	for _, fixture := range []struct {
		name                     string
		q0, q1, v0, v1, min, max float64
		recovery                 bool
		refused                  bool
	}{
		{"interior", 49, 49, 5, -5, -50, 50, false, true},
		{"bow", 0, 1, 0, 10, 10, 20, true, true},
		{"reversal", 0, 2, 8, 8, 10, 20, true, true},
		{"monotone", 0, 15, 0, 0, 10, 20, false, false},
		{"exterior", 0, 5, 0, 0, 10, 20, false, false},
		{"hold", 0, 0, 0, 0, 10, 20, false, false},
		{"interior_turn", 0, 15, 40, -5, 10, 20, false, false},
		{"opposite_boundary", 0, 15, 120, 0, 10, 20, true, true},
		{"feedforward", 0, 11, 1, -1, 10, 20, true, true},
	} {
		for _, sign := range []int{-1, 1} {
			for _, home := range []int32{0, 123} {
				t.Run(fmt.Sprintf("%s/sign_%d/home_%d", fixture.name, sign, home), func(t *testing.T) {
					// Exact fixture period is 1000 counts after native integer rounding.
					scale := 1000 / (2 * math.Pi)
					frame := densejointtraj.CommandFrame{CountsPerUnit: scale, Sign: sign, HomeOffset: home}
					_, err := densejointtraj.AdmitHermiteSegment(frame,
						densejointtraj.HermiteKnot{Position: fixture.q0 / scale, Velocity: fixture.v0 / scale},
						densejointtraj.HermiteKnot{TimeNS: 1_000_000_000, Position: fixture.q1 / scale, Velocity: fixture.v1 / scale}, fixture.min/scale, fixture.max/scale)
					if (err != nil) != fixture.refused {
						t.Fatalf("count fixture classification: %v", err)
					}
					if fixture.refused {
						var position *densejointtraj.PositionLimitError
						if !errors.As(err, &position) || position.Recovery != fixture.recovery {
							t.Fatalf("count fixture refusal class: %v", err)
						}
					}
				})
			}
		}
	}
}

// Exterior hold permission cannot admit a position outside the signed CSP wire
// range; native point admission rejects these before the extrema check runs.
func TestHermiteRecoveryCountRange(t *testing.T) {
	for _, q := range []float64{-2147483649, 2147483648} {
		_, err := densejointtraj.AdmitHermiteSegment(densejointtraj.CommandFrame{CountsPerUnit: 1, Sign: 1},
			densejointtraj.HermiteKnot{Position: q}, densejointtraj.HermiteKnot{TimeNS: 1_000_000_000, Position: q}, -1, 1)
		var position *densejointtraj.PositionLimitError
		if !errors.As(err, &position) || position.Recovery || position.Position != q ||
			(position.Limit != -2147483648 && position.Limit != 2147483647) {
			t.Fatalf("unrepresentable CSP position admitted: %+v %v", position, err)
		}
	}
}
