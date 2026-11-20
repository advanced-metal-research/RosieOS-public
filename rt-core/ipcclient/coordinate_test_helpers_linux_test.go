//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
)

func startCoordinateCore(t *testing.T, axes int, requireHome bool, extra ...string) fixture {
	t.Helper()
	machine, profile := compiledCoordinateInputs(t)
	base := machine["axes"].([]any)[0].(map[string]any)
	configured := make([]any, axes)
	for axis := range configured {
		raw, err := json.Marshal(base)
		if err != nil {
			t.Fatal(err)
		}
		var value map[string]any
		if err := json.Unmarshal(raw, &value); err != nil {
			t.Fatal(err)
		}
		value["name"], value["slave_position"], value["require_home"] =
			fmt.Sprintf("axis_%d", axis), 2*axis, requireHome
		configured[axis] = value
	}
	machine["axes"] = configured
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatalf("compile coordinate fixture: %v: %s", err, digest)
	}
	f, _ := coordinateModelDaemon(t, compiled, digest, extra...)
	return f
}

type coordinateAxisObservation struct {
	AxisStatus
	Epoch       *uint64 `json:"coordinate_epoch"`
	Valid       *uint32 `json:"coordinate_valid"`
	Counts      *int64  `json:"coordinate_counts"`
	Reason      *uint32 `json:"coordinate_reason"`
	SourceValid *uint32 `json:"coordinate_source_valid"`
	HomeState   uint32  `json:"native_home_state"`
}

type coordinateObservation struct {
	Status
	RTCycleCounter uint64                      `json:"rt_cycle_counter"`
	Axes           []coordinateAxisObservation `json:"axes"`
}

func waitCoordinateInjection(t *testing.T, f fixture, axes int, call uint64) {
	t.Helper()
	s := waitCoordinateObservation(t, f, axes, func(s coordinateObservation) bool {
		return s.RTCycleCounter >= call
	})
	t.Logf("coordinate injection reached: call=%d completed_cycles=%d faults=%d", call, s.RTCycleCounter, s.ExecutionFaultReasons)
}

func waitCoordinateObservation(t *testing.T, f fixture, axes int, predicate func(coordinateObservation) bool) coordinateObservation {
	t.Helper()
	deadline := time.Now().Add(8 * time.Second)
	var observed, result coordinateObservation
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
		if err == nil {
			observed = coordinateObservation{}
			if err := json.Unmarshal(raw, &observed); err != nil {
				t.Fatal(err)
			}
			now := monotonicNS()
			if observed.TimeNS == 0 || observed.TimeNS > now || now-observed.TimeNS > uint64(time.Second) ||
				observed.Backend != "simulation" || observed.ConfigurationSHA256 != f.options.ConfigurationSHA256 || len(observed.Axes) != axes {
				t.Fatalf("invalid coordinate observation identity/freshness: %+v", observed)
			}
			for axis, value := range observed.Axes {
				if value.Epoch == nil || value.Valid == nil || value.Counts == nil || value.Reason == nil || value.SourceValid == nil ||
					*value.Epoch == 0 || *value.Valid > 1 || *value.SourceValid > 1 {
					t.Fatalf("axis%d missing coordinate contract: %+v", axis, value)
				}
			}
			if predicate(observed) {
				result = observed
				return true
			}
		} else if !errors.Is(err, os.ErrNotExist) {
			t.Fatal(err)
		}
		return false
	}) {
		t.Fatalf("coordinate observation deadline: %+v", observed)
	}
	return result
}

func coordinateMove(t *testing.T, f fixture, id uint64, from, to []float64) uint64 {
	t.Helper()
	deadline := time.Now().Add(4 * time.Second)
	mask := uint32((1 << len(from)) - 1)
	var ready Status
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		var err error
		ready, err = f.client.Status()
		if err != nil || ready.Armed != 1 || ready.AxisEnableMask != mask || len(ready.Axes) != len(from) {
			return false
		}
		for _, axis := range ready.Axes {
			if axis.Readiness != "ready" || axis.PDOFresh != 1 {
				return false
			}
		}
		return true
	}) {
		t.Fatalf("coordinate plan readiness deadline: %+v", ready)
	}
	ctx, cancel := context.WithDeadline(context.Background(), deadline)
	defer cancel()
	seq, err := f.client.Trajectory(ctx, id, mask, []Point{{Position: from}, {Time: time.Second, Position: to}})
	mustSubmit(t, seq, err)
	var result uint64
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		motion, err := f.client.Motion()
		if err == nil && motion.CommandSequence == seq && motion.TrajectoryID == id && motion.Done {
			if motion.State != 4 {
				t.Fatalf("coordinate plan did not complete: %+v", motion)
			}
			result = seq
			return true
		}
		return false
	}) {
		t.Fatal("coordinate plan completion deadline")
	}
	return result
}

func coordinateHome(t *testing.T, f fixture, mask uint32) {
	t.Helper()
	seq, err := f.client.Home(context.Background(), mask)
	mustSubmit(t, seq, err)
	ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
	defer cancel()
	if result, err := f.client.WaitResult(ctx, seq); err != nil || result.Result != ResultAccepted {
		t.Fatalf("qualified Home failed: %+v %v", result, err)
	}
}
