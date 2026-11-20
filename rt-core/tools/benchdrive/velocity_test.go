//go:build linux

package main

import (
	"math"
	"testing"

	"rosieos/rt-core/ipcclient"
)

func TestAxisVelocityConsecutiveSamples(t *testing.T) {
	var estimator axisVelocity
	for i, tc := range []struct {
		timeNS   uint64
		position float64
		velocity float64
		fresh    bool
	}{
		{1000000000, 0, 0, false},
		{1100000000, .002, .02, true},
		{1100000000, .002, 0, false},  // Repeated publication contributes no pair.
		{1200000000, .001, .01, true}, // Use the preceding position, not the origin.
		{1300000000, .001, 0, true},
	} {
		// Unverified synthetic feedforward: 99 rad/s must not replace feedback.
		feedforward := 99.0
		v, fresh, err := estimator.observe(ipcclient.AxisStatus{TimeNS: tc.timeNS,
			LogicalPosition: tc.position, LogicalValid: true, LogicalVelocity: &feedforward})
		if err != nil || fresh != tc.fresh || math.Abs(v-tc.velocity) > 1e-12 {
			t.Fatalf("sample %d: velocity=%g fresh=%v err=%v", i, v, fresh, err)
		}
	}
}

func TestAxisVelocityIgnoresNonConsecutiveSamples(t *testing.T) {
	for _, tc := range []struct {
		name string
		axis ipcclient.AxisStatus
	}{
		{"missing", ipcclient.AxisStatus{}},
		{"zero_time", ipcclient.AxisStatus{LogicalValid: true}},
		{"invalid_position", ipcclient.AxisStatus{TimeNS: 1100000000, LogicalValid: true, LogicalPosition: math.NaN()}},
		{"infinite_position", ipcclient.AxisStatus{TimeNS: 1100000000, LogicalValid: true, LogicalPosition: math.Inf(1)}},
		{"invalid_coordinates", ipcclient.AxisStatus{TimeNS: 1100000000}},
		{"reordered", ipcclient.AxisStatus{TimeNS: 900000000, LogicalValid: true}},
		{"changed_duplicate", ipcclient.AxisStatus{TimeNS: 1000000000, LogicalValid: true, LogicalPosition: .001}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			var estimator axisVelocity
			if _, _, err := estimator.observe(ipcclient.AxisStatus{TimeNS: 1000000000, LogicalValid: true}); err != nil {
				t.Fatal(err)
			}
			if v, fresh, err := estimator.observe(tc.axis); err == nil || fresh || v != 0 {
				t.Fatalf("non-consecutive sample estimated: velocity=%g fresh=%v err=%v", v, fresh, err)
			}
			if v, fresh, err := estimator.observe(ipcclient.AxisStatus{TimeNS: 1200000000, LogicalValid: true, LogicalPosition: .2}); err != nil || fresh || v != 0 {
				t.Fatalf("bridged invalid sample: velocity=%g fresh=%v err=%v", v, fresh, err)
			}
			if v, fresh, err := estimator.observe(ipcclient.AxisStatus{TimeNS: 1300000000, LogicalValid: true, LogicalPosition: .201}); err != nil || !fresh || math.Abs(v-.01) > 1e-12 {
				t.Fatalf("new consecutive pair: velocity=%g fresh=%v err=%v", v, fresh, err)
			}
		})
	}
}

// TestProgramVelocityBenchTrackingCatchup preserves bd-program-v16.jsonl lines
// 140/141 and 152. Preserve the measured catch-up without smoothing away the
// 151-count step; the shared expectation records it against the axis limit.
func TestProgramVelocityBenchTrackingCatchup(t *testing.T) {
	previous := ipcclient.AxisStatus{TimeNS: 130518731192689, PositionCounts: 65437,
		LogicalValid: true, LogicalPosition: .004745753062521202, LogicalTarget: .007765777738671058}
	current := ipcclient.AxisStatus{TimeNS: 130518831783323, PositionCounts: 65286,
		LogicalValid: true, LogicalPosition: .011984224905356571, LogicalTarget: .011265171411035177}
	var estimator axisVelocity
	if _, _, err := estimator.observe(previous); err != nil {
		t.Fatal(err)
	}
	v, fresh, err := estimator.observe(current)
	if err != nil || !fresh || math.Abs(v-.0719596999740092) > 1e-12 || v <= .05 {
		t.Fatalf("bench catch-up hidden: velocity=%g fresh=%v err=%v", v, fresh, err)
	}
	if current.TimeNS-previous.TimeNS != 100590634 || previous.PositionCounts-current.PositionCounts != 151 {
		t.Fatal("bench sample interval or count displacement changed")
	}
	// Measured scaling: 131072 counts/rev at ratio 1/1 on bench-servo.
	countsPerRad := 131072 / (2 * math.Pi)
	targetStep := (current.LogicalTarget - previous.LogicalTarget) * countsPerRad
	previousLag := (previous.LogicalTarget - previous.LogicalPosition) * countsPerRad
	currentLag := (current.LogicalTarget - current.LogicalPosition) * countsPerRad
	if math.Abs(targetStep-73) > 1e-9 || math.Abs(previousLag-63) > 1e-9 || math.Abs(currentLag+15) > 1e-9 {
		t.Fatalf("target=%g counts, tracking lag=%g -> %g counts", targetStep, previousLag, currentLag)
	}
	targetVelocity := (current.LogicalTarget - previous.LogicalTarget) * 1e9 / 100590634
	if math.Abs(targetVelocity-.03478846422584551) > 1e-12 {
		t.Fatalf("target velocity=%g rad/s", targetVelocity)
	}
}
