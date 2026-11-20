//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control/anchors"
)

// A completed move may retire the saved restore pose without invalidating the
// live Home frame. A brake warning never touches Home; the independent
// coordinate policy alone decides whether an observed displacement keeps it.
func TestCompletedMoveBrakeWarningPreservesObservedHome(t *testing.T) {
	for _, sag := range []int{0, 33, 60000} {
		t.Run(strconv.Itoa(sag), func(t *testing.T) {
			t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
			machine, profile := compiledCoordinateInputs(t)
			// RevPi capture: 33 counts across Stop; profile: 16-count tolerance,
			// 100 ms release/hold windows. 60000 counts is an unverified contradiction fixture.
			profile["brake"] = map[string]any{"present": true, "gravity_axis": false,
				"release_delay_ms": 100, "hold_delay_ms": 100, "hold_displacement_tolerance_counts": 16}
			compiled, digest, err := compileCoordinateJSON(t, machine, profile)
			if err != nil {
				t.Fatal(err)
			}
			store := anchors.Store{Directory: filepath.Join(filepath.Dir(compiled.path), "anchors")}
			t.Setenv("ROSIE_RT_ANCHOR_DIR", store.Directory)
			f, _ := coordinateModelDaemon(t, compiled, digest, "--sim-brake-sag-counts", strconv.Itoa(sag))
			coordinateHome(t, f, 1)
			homed := wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.CommissioningPhase == 0 })
			if err := store.Save(coordinateModelRecord(homed)); err != nil {
				t.Fatal(err)
			}
			arm(t, f.client, 1)
			// Unverified fixture displacement: 100 counts in the referenced frame.
			coordinateMove(t, f, 9531, []float64{0}, []float64{100 * 2 * math.Pi / 131072})
			moved := wait(t, f.client, func(s Status) bool {
				return s.Axes[0].CoordinateCounts == 100
			})
			if moved.HomeValidMask != 1 || !moved.Axes[0].LogicalValid || moved.ExecutionFaultReasons != 0 {
				t.Fatalf("completed move compared live Home to obsolete restore pose: %+v", moved)
			}
			sequence, err := f.client.Stop(context.Background())
			mustSubmit(t, sequence, err)
			stopped := wait(t, f.client, func(s Status) bool {
				return s.Armed == 0 && s.AxisEnableMask == 0 && s.Axes[0].BrakeState == brakeApplied &&
					s.Axes[0].PositionCounts == int32(100+sag)
			})
			if sag == 0 {
				if stopped.HomeValidMask != 1 || stopped.ExecutionFaultReasons != 0 {
					t.Fatal(stopped)
				}
				return
			}
			if stopped.ExecutionFaultReasons&faultBrakeHold != 0 {
				t.Fatal("brake warning latched a fault", stopped)
			}
			if sag == 33 {
				if stopped.HomeValidMask != 1 || stopped.Axes[0].CoordinateValid != 1 ||
					stopped.Axes[0].CoordinateCounts != 133 || stopped.ExecutionFaultReasons != 0 || stopped.SafetyFaultMask != 0 {
					t.Fatalf("observed displacement lost Home or latched: %+v", stopped)
				}
			} else if stopped.HomeValidMask != 0 || stopped.ExecutionFaultReasons&64 == 0 {
				t.Fatalf("coordinate contradiction retained Home: %+v", stopped)
			}
			// The simulator sags on the first inhibited cycle, inside the drive's
			// 100 ms energized hold window: recorded from the last enabled pose,
			// never a warning, and the torque-off anchor is the sagged rest.
			type diagnostic struct {
				Severity        string `json:"severity"`
				Sequence        uint64 `json:"warning_sequence"`
				Kind            string `json:"kind"`
				Cause           string `json:"cause"`
				TorqueOffNS     uint64 `json:"torque_off_ns"`
				TorqueOffAnchor int32  `json:"torque_off_anchor_counts"`
				EnergizedPeak   int64  `json:"energized_peak_displacement_counts"`
				Tolerance       uint32 `json:"tolerance_counts"`
				Signal          string `json:"released_signal_semantic"`
				SignalValid     bool   `json:"released_signal_valid"`
			}
			var trip diagnostic
			deadline := time.Now().Add(4 * time.Second)
			for trip.TorqueOffNS == 0 && time.Now().Before(deadline) {
				raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
				if err != nil {
					t.Fatal(err)
				}
				var metrics struct {
					Axes []struct {
						Trip diagnostic `json:"brake_hold_diagnostic"`
					} `json:"axes"`
				}
				if json.Unmarshal(raw, &metrics) == nil && len(metrics.Axes) != 0 {
					trip = metrics.Axes[0].Trip
				}
				time.Sleep(time.Millisecond)
			}
			if trip.Severity != "none" || trip.Sequence != 0 || trip.Kind != "none" || trip.Cause != "none" ||
				trip.TorqueOffNS == 0 || trip.TorqueOffAnchor != int32(100+sag) || trip.EnergizedPeak != int64(sag) ||
				trip.Tolerance != 16 || trip.Signal != "none" || trip.SignalValid {
				t.Fatalf("misleading brake diagnosis: %+v", trip)
			}
		})
	}
}
