//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strconv"
	"testing"
	"time"
)

const faultCompletionTimeout = uint32(1 << 9)

// This is an execution deadline independent of the following-error guard.
// Ten counts exceed this fixture's completion tolerance (~4 counts), while
// remaining well inside its following-error limit (~834 counts).
const settlingTestTimeout = 100 * time.Millisecond
const settlingTestEndpoint = 0.004

func settlingCore(t *testing.T, offset int32, slavePosition uint16) *Client {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "config", "machines", "simulation", "simulation.json"))
	if err != nil {
		t.Fatal(err)
	}
	var machine map[string]any
	if err := json.Unmarshal(raw, &machine); err != nil {
		t.Fatal(err)
	}
	for _, rawAxis := range machine["axes"].([]any) {
		limits := rawAxis.(map[string]any)["limits"].(map[string]any)
		limits["completion_timeout_ms"] = settlingTestTimeout.Milliseconds()
	}
	raw, err = json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "settling.json")
	if err := os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	// Reuse the original compiler/daemon fixture. Only the explicit machine
	// deadline and the simulator's feedback plant differ from simulation.json.
	machinePath, err := filepath.Rel(filepath.Join(root, "config"), path)
	if err != nil {
		t.Fatal(err)
	}
	c := startCoreWithArgs(t, machinePath,
		"--sim-feedback-offset-counts", strconv.FormatInt(int64(offset), 10),
		"--sim-feedback-offset-position", strconv.FormatUint(uint64(slavePosition), 10)).client
	before, err := c.Status()
	if err != nil {
		t.Fatal(err)
	}
	for _, axis := range before.Axes {
		if axis.PositionCounts != 0 || axis.Statusword&0x6f == 0x27 {
			t.Fatalf("tracking injection moved a disabled drive: %+v", before)
		}
	}
	return c
}

func startSettlingPlan(t *testing.T, c *Client, handle uint64, endpoint time.Duration, offset int32, slavePosition uint16) {
	t.Helper()
	prepared, err := c.PrepareTrajectory(context.Background(), handle, 3, []Point{
		{Position: []float64{0, 0}},
		{Time: endpoint, Position: []float64{settlingTestEndpoint, settlingTestEndpoint}},
	})
	if err != nil {
		t.Fatal(err)
	}
	arm(t, c, 3)
	ready := wait(t, c, allOperationEnabled)
	if ready.ExecutionFaultReasons != 0 || ready.SafetyFaultMask != 0 {
		t.Fatalf("tracking error must remain below the following-error guard: %+v", ready)
	}
	for _, axis := range ready.Axes {
		var expected int32
		if axis.SlavePosition == slavePosition {
			expected = offset
		}
		if axis.PositionCounts != expected {
			t.Fatalf("tracking injection must select one physical slave position: axis=%+v expected=%d", axis, expected)
		}
	}
	if _, err := c.StartTrajectory(context.Background(), prepared); err != nil {
		t.Fatal(err)
	}
}

func requireSettlingFault(t *testing.T, c *Client, handle uint64, endpoint time.Duration, axisIndex int, offset int32) {
	t.Helper()
	deadline := time.Now().Add(endpoint + settlingTestTimeout + time.Second)
	var status Status
	var motion MotionState
	for time.Now().Before(deadline) {
		var err error
		status, err = c.Status()
		if err != nil {
			t.Fatal(err)
		}
		motion, err = c.Motion()
		if err == nil && motion.TrajectoryID == handle {
			if motion.State == 4 && motion.Done {
				t.Fatalf("out-of-tolerance endpoint reported completion: %+v", motion)
			}
			if status.ExecutionFaultReasons&faultCompletionTimeout != 0 && motion.State == 6 && motion.Done {
				break
			}
		}
		time.Sleep(10 * time.Millisecond)
	}
	if status.ExecutionFaultReasons != faultCompletionTimeout || motion.TrajectoryID != handle || motion.State != 6 || !motion.Done {
		t.Fatalf("endpoint must fault with bit 9 after its settling deadline: status=%+v motion=%+v", status, motion)
	}
	if status.SafetyFaultMask != 1<<axisIndex || status.Armed != 0 || status.AxisEnableMask != 0 {
		t.Fatalf("settling fault must identify the unsettled axis and inhibit the group: %+v", status)
	}
	finalCounts := int32(math.Round(settlingTestEndpoint * 131072 / (2 * math.Pi)))
	for i, axis := range status.Axes {
		expected := finalCounts
		if i == 1 {
			expected = -expected
		}
		if i == axisIndex {
			expected += offset
		}
		if axis.PositionCounts != expected {
			t.Fatalf("fault must retain the submitted endpoint's actual feedback: axis=%+v expected=%d", axis, expected)
		}
	}
}

func TestTrajectorySettlingFaultsPersistentTrackingError(t *testing.T) {
	for _, tc := range []struct {
		axis     int
		position uint16
		offset   int32
	}{
		{axis: 0, position: 0, offset: 10},
		{axis: 1, position: 2, offset: -10},
	} {
		t.Run(fmt.Sprintf("slave_%d", tc.position), func(t *testing.T) {
			c := settlingCore(t, tc.offset, tc.position)
			startSettlingPlan(t, c, 9701, 200*time.Millisecond, tc.offset, tc.position)
			requireSettlingFault(t, c, 9701, 200*time.Millisecond, tc.axis, tc.offset)
		})
	}
}

func TestTrajectorySettlingTimeoutStartsAtEndpoint(t *testing.T) {
	c := settlingCore(t, 10, 0)
	const endpoint = 800 * time.Millisecond
	startSettlingPlan(t, c, 9702, endpoint, 10, 0)
	// An execution lasting longer than its settling allowance must not consume
	// that allowance while its target is still following the planned path.
	for deadline := time.Now().Add(300 * time.Millisecond); time.Now().Before(deadline); {
		status, err := c.Status()
		if err != nil {
			t.Fatal(err)
		}
		if status.ExecutionFaultReasons != 0 || status.SafetyFaultMask != 0 || status.Armed != 1 {
			t.Fatalf("settling deadline ran before the planned endpoint: %+v", status)
		}
		motion, err := c.Motion()
		if err == nil && motion.TrajectoryID == 9702 && (motion.State != 3 || motion.Done) {
			t.Fatalf("long plan terminated before its endpoint: %+v", motion)
		}
		time.Sleep(10 * time.Millisecond)
	}
	requireSettlingFault(t, c, 9702, endpoint, 0, 10)
}

func TestTrajectorySettlingIdealFeedbackCompletes(t *testing.T) {
	c := settlingCore(t, 0, 0)
	startSettlingPlan(t, c, 9703, 200*time.Millisecond, 0, 0)
	deadline := time.Now().Add(time.Second)
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		motion, err := c.Motion()
		if err == nil && motion.TrajectoryID == 9703 && motion.State == 4 && motion.Done {
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("fresh in-tolerance feedback did not complete: %+v %v", motion, err)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	// A finished plan must not acquire a late timeout after relinquishing the
	// executor. Observe beyond the configured deadline after completion.
	time.Sleep(2 * settlingTestTimeout)
	status, err := c.Status()
	if err != nil || status.ExecutionFaultReasons != 0 || status.SafetyFaultMask != 0 {
		t.Fatalf("completed plan acquired a settling fault: %+v %v", status, err)
	}
}
