//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// TestSimulationMoveMeasurement checks the actual public telemetry path at
// 1 kHz. Diagnostic PDO values are injected fixtures, not a physical drive model.
func TestSimulationMoveMeasurement(t *testing.T) {
	f := startFixtureMeasurement(t, false, true, "--sim-following-error-counts", "789", "--sim-velocity-actual-counts-per-s", "-123456")
	f.phase(t, "home")
	args := []string{"--socket", f.socket, "--pair-id", "bench", "--pair-revision", "7", "--configuration-sha256", f.hash,
		"--evidence", filepath.Join(f.build, "measurement.jsonl"), "--phase", "move", "--deltas-rad", "0:0.02,8:-0.02",
		"--velocity-fraction", "0.5", "--accel-fraction", "0.5", "--measure", "--confirm-motion"}
	var out bytes.Buffer
	if code := run(context.Background(), args, &out); code != 0 {
		t.Fatalf("exit=%d: %s", code, &out)
	}
	records := readRecords(t, filepath.Join(f.build, "measurement.jsonl"))
	counts := map[int]int{}
	previous := map[int]uint64{}
	summaries := 0
	for _, rec := range records {
		if rec.Operation == "move_cycle" {
			var s struct {
				Axis        int    `json:"axis"`
				Seq         uint64 `json:"sequence"`
				Error       int32  `json:"following_error_counts"`
				Velocity    int32  `json:"actual_velocity_counts_per_s"`
				Torque      int16  `json:"torque_raw"`
				TorqueValid bool   `json:"torque_valid"`
			}
			simfixture.Must(t, json.Unmarshal(rec.Response, &s))
			if !s.TorqueValid || s.Torque != 0 {
				t.Fatalf("simulator capture torque missing: %+v", s)
			}
			if (s.Axis != 0 && s.Axis != 8) || s.Error != 789 || s.Velocity != -123456 || (previous[s.Axis] != 0 && s.Seq != previous[s.Axis]+1) {
				t.Fatalf("bad cyclic telemetry: %+v", s)
			}
			previous[s.Axis] = s.Seq
			counts[s.Axis]++
		}
		if rec.Operation == "move_measurement" && rec.Kind == "travel" {
			var s axisMeasurement
			simfixture.Must(t, json.Unmarshal(rec.Response, &s))
			summaries++
			if s.PeakLag < 1 || s.PeakLag > 100 || s.CruiseSamples < 20 || s.CruiseLag < 0 || s.CruiseLag > 100 || s.Overshoot > 16 || s.SettlingNS > 500_000_000 {
				t.Fatalf("absolute tracking bounds: %+v", s)
			}
		}
	}
	if counts[0] < 500 || counts[8] < 500 || summaries != 2 {
		t.Fatalf("cycles=%v summaries=%d", counts, summaries)
	}
}

// TestSimulationHaltStopsWithinBound requires a moving source, the exact Halt
// fence and receipt, bounded count-resolved stopping and enabled held targets.
func TestSimulationHaltStopsWithinBound(t *testing.T) {
	f := startFixtureMeasurement(t, false, true)
	f.phase(t, "home")
	for _, tc := range []struct{ name, delta string }{{"positive", "0.2"}, {"negative", "-0.2"}} {
		t.Run(tc.name, func(t *testing.T) {
			args := []string{"--socket", f.socket, "--pair-id", "bench", "--pair-revision", "7", "--configuration-sha256", f.hash,
				"--evidence", filepath.Join(f.build, tc.name+".jsonl"), "--phase", "halt", "--axis", "0", "--delta-rad", tc.delta,
				"--cruise-velocity-rad-s", "0.03", "--accel-fraction", "0.5", "--confirm-motion"}
			var out bytes.Buffer
			if code := run(context.Background(), args, &out); code != 0 {
				t.Fatalf("exit=%d: %s", code, &out)
			}
			records := readRecords(t, filepath.Join(f.build, tc.name+".jsonl"))
			var started, halted control.Request
			measurements, completions := 0, 0
			for _, rec := range records {
				if rec.Kind == "request" && rec.Operation == "start_trajectory" {
					simfixture.Must(t, json.Unmarshal(rec.Request, &started))
				}
				if rec.Kind == "request" && rec.Operation == "halt" {
					simfixture.Must(t, json.Unmarshal(rec.Request, &halted))
					if rec.Reason != "" {
						t.Fatal(rec.Reason)
					}
				}
				if rec.Operation == "halt_measurement" && rec.Kind == "travel" {
					var s axisMeasurement
					simfixture.Must(t, json.Unmarshal(rec.Response, &s))
					measurements++
					if s.CruiseSamples < 1 || s.RequestToZeroNS < 1 || s.RequestToZeroNS > 100_000_000 || s.ReceiptToStandstillNS > 300_000_000 || s.StoppingDistance < 0 || s.StoppingDistance > 100 || s.PeakDeceleration <= 0 || s.PeakDeceleration > 26_000 || s.HeldTarget != s.LastCommandedTarget {
						t.Fatalf("absolute halt bounds: %+v", s)
					}
				}
				if rec.Operation == "halt" && rec.Kind == "completion" {
					completions++
					var c map[string]any
					simfixture.Must(t, json.Unmarshal(rec.Response, &c))
					if c["armed_idle"] != true || c["held_target_equals_last_commanded_target"] != true {
						t.Fatal(c)
					}
				}
			}
			if started.Fence.Session == "" || started.Fence != halted.Fence || measurements != 1 || completions != 1 {
				t.Fatalf("uncorrelated halt: start=%+v halt=%+v measurements=%d completions=%d", started.Fence, halted.Fence, measurements, completions)
			}
		})
	}
}

func TestMeasurementRejectsGapsAndUnavailableFields(t *testing.T) {
	for _, reason := range []string{"measurement_telemetry_gap", "measurement_drive_fields_unavailable", "measurement_drive_not_enabled"} {
		t.Run(reason, func(t *testing.T) {
			r := &runner{o: options{phase: "move"}, e: &evidence{sink: io.Discard}, description: control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{{}}}}}
			m := &cycleMeasurement{r: r, moves: []moveSpec{{axis: 0, delta: 1}}}
			s := ipcclient.CycleCaptureRecordV2{Seq: 1, Cycle: 1, TNs: 1_000_000, Axes: 1}
			s.Ax[0] = ipcclient.CycleCaptureAxisV2{Statusword: 0x27, FollowingErrorValid: 1, VelocityActualValid: 1, TorqueValid: 1}
			simfixture.Must(t, m.append(s))
			s.Seq = 2
			s.Cycle = 2
			s.TNs = 2_000_000
			switch reason {
			case "measurement_telemetry_gap":
				s.Cycle = 3
			case "measurement_drive_fields_unavailable":
				s.Ax[0].VelocityActualValid = 0
			case "measurement_drive_not_enabled":
				s.Ax[0].Statusword = 0x40
			}
			var refused *control.Rejected
			if err := m.append(s); !errors.As(err, &refused) || refused.Reason != reason {
				t.Fatal(err)
			}
		})
	}
}

func TestMeasurementContinuousFeedback(t *testing.T) {
	r := &runner{description: control.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{{CountsPerUnit: 1}}}}}
	m := cycleMeasurement{r: r}
	for _, tc := range []struct {
		target, raw int32
		want        int64
	}{{-3, 131070, 131070}, {131074, 1, 1}, {-131074, 131071, 131071}} {
		if got := m.feedback(0, ipcclient.CycleCaptureAxisV2{Target: tc.target, Position: tc.raw}); got != tc.want {
			t.Fatalf("target=%d raw=%d feedback=%d want=%d counts", tc.target, tc.raw, got, tc.want)
		}
	}
}

// TestDocumentedMeasurementCommands verifies the actual operator command lines
// against the CLI and unchanged bench acceleration/travel bounds.
func TestDocumentedMeasurementCommands(t *testing.T) {
	raw, err := os.ReadFile("../../docs/BENCH_TESTING.md")
	simfixture.Must(t, err)
	d := control.Description{NativeDescription: ipcclient.Description{CycleNS: 1_000_000, MaxTrajectoryPoints: 250_000}}
	// Unverified bench caps: 25 rad/s, profile 0.5 rad/s^2, jog 50 rad/s^2.
	acceleration := 0.5
	for i := 0; i < 9; i++ {
		d.Axes = append(d.Axes, ipcclient.AxisDescription{Index: uint32(i), PositionUnit: "rad", MinPosition: -3, MaxPosition: 3, MaxVelocity: 25, JogAcceleration: 50, MaxAcceleration: &acceleration, CompletionTolerance: 0.001})
	}
	moving, halting := 0, 0
	for _, line := range strings.Split(string(raw), "\n") {
		if !strings.HasPrefix(line, "build/benchdrive ") || (!strings.Contains(line, "--measure") && !strings.Contains(line, "--phase halt")) {
			continue
		}
		fields := strings.Fields(line)[1:]
		for i := range fields {
			fields[i] = strings.Trim(fields[i], "'\"")
			if fields[i] == "$SHA_A" || fields[i] == "$SHA_B" {
				fields[i] = strings.Repeat("a", 64)
			}
		}
		o, err := parseOptions(fields, io.Discard)
		simfixture.Must(t, err)
		if !o.confirm || !o.measure {
			t.Fatal("documented measurement lacks confirmation/capture")
		}
		if o.phase == "halt" {
			halting++
			o.velocityFraction = o.cruiseVelocity / (25 * moveVelocityReserve)
		} else {
			moving++
		}
		origins := map[int]float64{}
		for _, m := range o.moves {
			origins[m.axis] = 0
		}
		points, err := moveProfileGroup(d, o, o.moves, origins)
		simfixture.Must(t, err)
		if len(points) < 100 || points[len(points)-1].Time > 9_000_000_000 {
			t.Fatal("documented profile lacks bounded samples")
		}
	}
	if moving != 4 || halting != 1 {
		t.Fatalf("documented measurement commands: move=%d halt=%d", moving, halting)
	}
}

func TestSimulationMeasurementRefusals(t *testing.T) {
	f := startFixture(t)
	f.phase(t, "home")
	for _, tc := range []struct {
		name   string
		args   []string
		code   int
		reason string
	}{
		{"unconfirmed", []string{"--phase", "halt", "--axis", "0", "--delta-rad", "0.2", "--cruise-velocity-rad-s", "0.03"}, 3, ""},
		{"unreachable", []string{"--phase", "halt", "--axis", "0", "--delta-rad", "0.001", "--cruise-velocity-rad-s", "0.03", "--confirm-motion"}, 1, "halt_cruise_unreachable_within_travel"},
		{"missing-fields", []string{"--phase", "move", "--axis", "0", "--delta-rad", "0.02", "--measure", "--confirm-motion"}, 1, "measurement_drive_fields_unavailable"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			args := append([]string{"--socket", f.socket, "--pair-id", "bench", "--pair-revision", "7", "--configuration-sha256", f.hash, "--evidence", filepath.Join(f.build, tc.name+".jsonl")}, tc.args...)
			var out bytes.Buffer
			if code := run(context.Background(), args, &out); code != tc.code {
				t.Fatalf("exit=%d: %s", code, &out)
			}
			found := tc.reason == ""
			for _, rec := range readRecords(t, filepath.Join(f.build, tc.name+".jsonl")) {
				if rec.Operation == "start_trajectory" || rec.Operation == "halt" && rec.Kind == "request" {
					t.Fatal("refused measurement started motion")
				}
				found = found || rec.Reason == tc.reason
			}
			if !found {
				t.Fatalf("missing reason %s", tc.reason)
			}
		})
	}
}
