//go:build linux

package oracle

import (
	"context"
	"encoding/json"
	"errors"
	"math"
	"os"
	"path/filepath"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	. "rosieos/rt-core/sdk/control"
)

// The synthetic encoder uses the bench's 100:1 count scale. Keep the existing
// simulation completion tolerance; a deliberately slow fixture acceleration
// makes settling observable and allows an actual lease to expire mid-settle.
func startSettlingService(t *testing.T) *service {
	t.Helper()
	return startServiceMachine(t, func(root, build string) (string, string) {
		raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-program.json"))
		must(t, err)
		var machine map[string]any
		must(t, json.Unmarshal(raw, &machine))
		for _, value := range machine["axes"].([]any) {
			axis := value.(map[string]any)
			axis["wire_counts_per_rev"] = 13107200
			axis["limits"].(map[string]any)["jog_acceleration"] = 0.00001
		}
		raw, err = json.Marshal(machine)
		must(t, err)
		path := filepath.Join(build, "settling-machine.json")
		must(t, os.WriteFile(path, raw, 0600))
		profile, err := os.ReadFile(filepath.Join(root, "config/drives/cia402_sim.json"))
		must(t, err)
		var drive map[string]any
		must(t, json.Unmarshal(profile, &drive))
		delete(drive, "max_acceleration") // Exercise native fallback to the fixture jog cap.
		profile, err = json.Marshal(drive)
		must(t, err)
		profiles := filepath.Join(build, "profiles")
		must(t, os.MkdirAll(profiles, 0700))
		must(t, os.WriteFile(filepath.Join(profiles, "cia402_sim.json"), profile, 0600))
		return path, profiles
	})
}

func settlingProgram(t *testing.T, s *service, counts []float64, initialVelocity float64) (Program, []byte) {
	t.Helper()
	description, err := s.client.Describe(context.Background())
	must(t, err)
	plan, err := densejointtraj.Decode(oracleProgram(t, s.root, false))
	must(t, err)
	samples := make([]densejointtraj.Sample, 101)
	for i := range samples {
		samples[i].TimeS = float64(i) / 100
		for axis, count := range counts {
			samples[i].QRad[axis] = count / description.Axes[axis].CountsPerUnit
		}
	}
	samples[0].QdRadS[0] = initialVelocity
	plan.Header.ProcessMarkers = nil
	plan.Header.Sampling = densejointtraj.Sampling{DtS: .01, TotalSampleCount: len(samples), TotalDurationS: 1}
	plan.Header.Segments = []densejointtraj.SegmentMeta{{Index: 0, Kind: "freespace", SourceID: "settling-regression", SampleCount: len(samples), DurationS: 1}}
	blob, err := densejointtraj.Encode(plan.Header, []densejointtraj.Segment{{Kind: "freespace", SourceID: "settling-regression", Samples: samples}})
	must(t, err)
	prepared, err := s.client.PrepareProgram(context.Background(), blob)
	must(t, err)
	return prepared, blob
}

func TestProgramStartSettlesWithinCompletionTolerance(t *testing.T) {
	s := startSettlingService(t)
	s.acquire(t, true)
	counts := []float64{3, 13, -7}
	prepared, blob := settlingProgram(t, s, counts, 0)
	plan, err := densejointtraj.Decode(blob)
	must(t, err)
	description, err := s.client.Describe(context.Background())
	must(t, err)
	for i, count := range counts {
		if math.Abs(count) <= 1 || math.Abs(count) >= description.Axes[i].CompletionTolerance*description.Axes[i].CountsPerUnit {
			t.Fatal("fixture does not exercise a sub-tolerance multi-count offset")
		}
	}
	s.enable(t)
	_, err = s.client.StartProgram(context.Background(), prepared.Identity)
	must(t, err)
	intermediate := s.waitStatus(t, func(v Status) bool {
		q := v.Axes[1].LogicalTarget * description.Axes[1].CountsPerUnit
		return q > 0 && q < 13
	})
	if intermediate.Execution.State != "executing" || intermediate.PlanCursor.SampleIndex != 0 || intermediate.Core.SafetyFaultMask != 0 {
		t.Fatalf("settling advanced or faulted the plan: %+v", intermediate.Motion)
	}
	arrived := s.waitStatus(t, func(v Status) bool { return v.Axes[1].PositionCounts == 13 })
	if arrived.Execution.State != "executing" {
		t.Fatal("program clock completed before settling reached the first pose")
	}
	completed := s.waitStatus(t, func(v Status) bool { return v.Execution.State == "completed" })
	if completed.Core.SafetyFaultMask != 0 || completed.Execution.Identity != prepared.Identity || prepared.Identity.TrajectoryDigest != plan.Header.TrajectoryDigest {
		t.Fatal("settling changed the immutable program identity or faulted")
	}
	if completed.Motion.TimeNS < arrived.Motion.TimeNS+900_000_000 {
		t.Fatal("settling consumed the original one-second trajectory clock")
	}
	for i, want := range counts {
		if math.Abs(float64(completed.Axes[i].PositionCounts)-want) > 1 {
			t.Fatalf("axis %d ended at %d counts, want %g", i, completed.Axes[i].PositionCounts, want)
		}
	}
}

func TestProgramStartStillRejectsOutsideSettlingEnvelope(t *testing.T) {
	for _, moving := range []bool{false, true} {
		name := "outside_completion_tolerance"
		if moving {
			name = "nonstationary_first_sample"
		}
		t.Run(name, func(t *testing.T) {
			s := startSettlingService(t)
			s.acquire(t, true)
			description, err := s.client.Describe(context.Background())
			must(t, err)
			count, velocity := math.Ceil(description.Axes[0].CompletionTolerance*description.Axes[0].CountsPerUnit)+2, 0.0
			if moving {
				count, velocity = 13, .000001
			}
			prepared, _ := settlingProgram(t, s, []float64{count}, velocity)
			s.enable(t)
			_, err = s.client.StartProgram(context.Background(), prepared.Identity)
			var refusal *Rejected
			// The discontinuity revokes authority in the same cycle as its
			// rejection; either receipt may win the adapter response race.
			if !errors.As(err, &refusal) || (refusal.Reason != "native_rejected" && refusal.Reason != "fence") {
				t.Fatalf("expected a native start refusal, got %v", err)
			}
			state := s.waitStatus(t, func(v Status) bool { return v.Core.Armed == 0 && v.Core.SafetyFaultMask != 0 })
			if state.Core.ExecutionFaultReasons != 1 || state.Core.SafetyFaultMask != 1 || state.Axes[0].PositionCounts != 0 {
				t.Fatalf("inadmissible start lost its exact discontinuity diagnosis or moved: reasons=%d axes=%d position=%d", state.Core.ExecutionFaultReasons, state.Core.SafetyFaultMask, state.Axes[0].PositionCounts)
			}
		})
	}
}

func TestProgramStartSettlingCannotOutliveAuthority(t *testing.T) {
	for _, leaseLoss := range []bool{false, true} {
		name := "stop"
		if leaseLoss {
			name = "lease_expiry"
		}
		t.Run(name, func(t *testing.T) {
			s := startSettlingService(t)
			s.acquire(t, true)
			prepared, _ := settlingProgram(t, s, []float64{13}, 0)
			s.enable(t)
			_, err := s.client.StartProgram(context.Background(), prepared.Identity)
			must(t, err)
			s.waitStatus(t, func(v Status) bool { return v.Axes[0].PositionCounts > 0 && v.Axes[0].PositionCounts < 5 })
			if leaseLoss {
				s.client.StopRenewal()
			} else {
				_, err = s.client.Stop(context.Background())
				must(t, err)
			}
			stopped := s.waitStatus(t, func(v Status) bool {
				return v.Core.Armed == 0 && v.Core.AxisEnableMask == 0 && v.Execution.State != "executing"
			})
			if stopped.Axes[0].PositionCounts >= 13 {
				t.Fatal("authority loss happened after the fixture finished settling")
			}
			// Observe past the nominal settling and program duration; a stale
			// alignment deadline must never revive the original trajectory.
			deadline := time.Now().Add(3 * time.Second)
			for time.Now().Before(deadline) {
				state, err := s.client.Status(context.Background())
				must(t, err)
				if state.Core.Armed != 0 || state.Core.AxisEnableMask != 0 || state.Execution.State == "executing" || state.Execution.State == "completed" || state.Axes[0].PositionCounts != stopped.Axes[0].PositionCounts {
					t.Fatal("settling resumed after authority ended")
				}
				time.Sleep(10 * time.Millisecond)
			}
		})
	}
}
