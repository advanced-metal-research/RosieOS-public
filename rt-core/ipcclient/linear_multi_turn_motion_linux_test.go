//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"
	"time"
)

func runContinuousTurnMove(t *testing.T, f fixture, id uint64, from, to float64) []int32 {
	t.Helper()
	wait(t, f.client, func(s Status) bool {
		return s.Armed == 1 && s.AxisEnableMask == 1 && len(s.Axes) == 1 &&
			s.Axes[0].Readiness == "ready" && s.Axes[0].PDOFresh == 1
	})
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	sequence, err := f.client.Trajectory(ctx, id, 1, []Point{
		{Position: []float64{from}},
		{Time: time.Second, Position: []float64{to}},
	})
	mustSubmit(t, sequence, err)

	var positions []int32
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	for {
		status, statusErr := f.client.Status()
		if statusErr == nil && len(status.Axes) == 1 {
			position := status.Axes[0].PositionCounts
			if len(positions) == 0 || positions[len(positions)-1] != position {
				positions = append(positions, position)
			}
		}
		motion, motionErr := f.client.Motion()
		if motionErr == nil && motion.CommandSequence == sequence && motion.TrajectoryID == id && motion.Done {
			if motion.State != 4 {
				status, _ := f.client.Status()
				axis := status.Axes[0]
				var metrics Metrics
				raw, _ := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
				_ = json.Unmarshal(raw, &metrics)
				var detail struct {
					Axes []struct {
						Sampled struct {
							AgeNS          uint64 `json:"age_ns"`
							Distance       uint64 `json:"coherence_error_counts"`
							Reason         uint32 `json:"reason"`
							LastInvalidity uint32 `json:"last_invalidity_reason"`
						} `json:"sampled_coordinate"`
					} `json:"axes"`
				}
				_ = json.Unmarshal(raw, &detail)
				t.Fatalf("continuous turn move failed: motion=%+v safety=%#x execution=%#x coordinate_reason=%d valid=%d source_valid=%d coordinate=%d source=%d position=%d audit=%s/%s sampled=%+v max_jitter_ns=%d max_work_ns=%d",
					motion, status.SafetyFaultMask, status.ExecutionFaultReasons, axis.CoordinateReason,
					axis.CoordinateValid, axis.CoordinateSourceValid, axis.CoordinateCounts,
					axis.AbsoluteSourceCounts, axis.PositionCounts, axis.AuditStatus, axis.AuditReason,
					detail.Axes[0].Sampled, metrics.MaxJitterNS, metrics.MaxWorkNS)
			}
			return positions
		}
		select {
		case <-ctx.Done():
			var last any = "none"
			if len(positions) != 0 {
				last = positions[len(positions)-1]
			}
			t.Fatalf("continuous turn move timed out: positions=%d last=%v", len(positions), last)
		case <-ticker.C:
		}
	}
}

func assertContinuousDirection(t *testing.T, positions []int32, increasing bool, final int32) {
	t.Helper()
	if len(positions) < 3 {
		t.Fatalf("continuous move has only %d samples", len(positions))
	}
	if positions[len(positions)-1] != final {
		t.Fatalf("continuous move final=%d want=%d", positions[len(positions)-1], final)
	}
	for i := 1; i < len(positions); i++ {
		if (increasing && positions[i] < positions[i-1]) ||
			(!increasing && positions[i] > positions[i-1]) {
			t.Fatalf("continuous move reversed at %d: %d -> %d", i, positions[i-1], positions[i])
		}
	}
}

func TestSimulatorExecutesSinglePlusMinus720MoveAndReversal(t *testing.T) {
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	machine, profile := compiledCoordinateInputs(t)
	machine["drive_speed_limit_motor_rpm"] = 3000
	limits := machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
	limits["min"], limits["max"], limits["max_velocity"] = -13, 13, 30
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err, digest)
	}
	f, _ := coordinateModelDaemon(t, compiled, digest)
	coordinateHome(t, f, 1)
	arm(t, f.client, 1)

	const twoTurns = 4 * math.Pi
	assertContinuousDirection(t, runContinuousTurnMove(t, f, 9801, 0, twoTurns), true, 262144)
	assertContinuousDirection(t, runContinuousTurnMove(t, f, 9802, twoTurns, -twoTurns), false, -262144)
}
