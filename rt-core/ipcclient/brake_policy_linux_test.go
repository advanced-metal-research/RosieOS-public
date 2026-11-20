//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// Unverified wire brake states and fault bit; assertions use absolute counts/ns.
const (
	brakeUnknown     = 0
	brakeReleased    = 1
	brakeApplied     = 2
	brakeWaitRelease = 3
	brakeWaitHold    = 4
	faultBrakeHold   = uint32(1 << 10) // execution_policy.hpp FaultBrakeHold; retained wire bit, never raised by the brake guard
)

type brakeAxisObservation struct {
	Position   int32               `json:"pos_counts"`
	Statusword uint16              `json:"statusword"`
	Brake      *uint32             `json:"brake_state"`
	ObservedNS uint64              `json:"pdo_observed_time_ns"`
	Warning    BrakeHoldDiagnostic `json:"brake_hold_diagnostic"`
}

type brakeObservation struct {
	TimeNS       uint64                 `json:"time_ns"`
	Armed        uint32                 `json:"armed"`
	EnableMask   uint32                 `json:"axis_enable_mask"`
	FaultMask    uint32                 `json:"safety_fault_mask"`
	FaultReasons uint32                 `json:"execution_fault_reasons"`
	Axes         []brakeAxisObservation `json:"axes"`
}

// Compile a machine override through the ordinary compiler and launch the real
// simulated daemon. Axis 1 remains unbraked to expose accidental group defaults.
func startBrakeCore(t *testing.T, present, gravity bool, releaseMS, holdMS int, extra ...string) fixture {
	t.Helper()
	return startBrakeCoreWithSignal(t, present, gravity, releaseMS, holdMS, nil, extra...)
}

func startBrakeCoreWithSignal(t *testing.T, present, gravity bool, releaseMS, holdMS int, signal map[string]any, extra ...string) fixture {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation.json"))
	if err != nil {
		t.Fatal(err)
	}
	var machine map[string]any
	if err := json.Unmarshal(raw, &machine); err != nil {
		t.Fatal(err)
	}
	brake := map[string]any{
		"present": present, "gravity_axis": gravity, "release_delay_ms": releaseMS, "hold_delay_ms": holdMS,
	}
	if signal != nil {
		brake["released_signal"] = signal
	}
	machine["axes"].([]any)[0].(map[string]any)["brake"] = brake
	raw, err = json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "brake-machine.json")
	if err := os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	relative, err := filepath.Rel(filepath.Join(root, "config"), path)
	if err != nil {
		t.Fatal(err)
	}
	return startCoreWithArgs(t, relative, extra...)
}

// Read the existing daemon status file, including per-axis brake publication.
// A ticker only paces polling; a fresh predicate, never a sleep, passes a test.
func waitBrake(t *testing.T, f fixture, predicate func(brakeObservation) bool) brakeObservation {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	defer cancel()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	var state brakeObservation
	for {
		raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
		if err == nil && json.Unmarshal(raw, &state) == nil && len(state.Axes) == 2 {
			for _, axis := range state.Axes {
				if axis.Brake == nil {
					t.Fatal("axis status omitted brake_state")
				}
			}
			if predicate(state) {
				return state
			}
		}
		select {
		case <-ctx.Done():
			t.Fatalf("brake observation timeout: %+v", state)
		case <-ticker.C:
		}
	}
}

func TestBrakeReleaseGatesStartAndPublishesSequence(t *testing.T) {
	// Unverified fixture windows: C05.13=1000 ms and C05.10=800 ms.
	f := startBrakeCore(t, true, true, 1000, 800)
	c := f.client
	initial := waitBrake(t, f, func(s brakeObservation) bool {
		return *s.Axes[0].Brake == brakeUnknown || *s.Axes[0].Brake == brakeApplied
	})
	if *initial.Axes[1].Brake != brakeReleased {
		t.Fatal("unbraked axis must pass through")
	}
	handle, err := c.PrepareTrajectory(context.Background(), 9801, 3, []Point{
		{Time: 0, Position: []float64{0, 0}},
		{Time: 200 * time.Millisecond, Position: []float64{0.003, 0.003}},
	})
	if err != nil {
		t.Fatal(err)
	}
	seq, err := c.Enable(context.Background(), 3)
	mustSubmit(t, seq, err)
	beforeArm := monotonicNS()
	seq, err = c.Arm(context.Background())
	mustSubmit(t, seq, err)
	waiting := waitBrake(t, f, func(s brakeObservation) bool {
		return *s.Axes[0].Brake == brakeWaitRelease && s.Axes[0].Statusword&0x6f == 0x27
	})
	_, err = c.StartTrajectory(context.Background(), handle)
	var rejected *RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonNotReady {
		t.Fatalf("Start inside C05.13 must reject not_ready: %v", err)
	}
	if waiting.Axes[0].Position != 0 || waiting.Axes[1].Position != 0 {
		t.Fatalf("feedback moved before brake release: %+v", waiting)
	}
	samples := 0
	lastNS := waiting.TimeNS
	released := waitBrake(t, f, func(s brakeObservation) bool {
		if s.TimeNS <= lastNS {
			return false
		}
		lastNS = s.TimeNS
		if s.FaultReasons != 0 || s.Axes[0].Position != 0 || s.Axes[1].Position != 0 {
			t.Fatalf("rejected plan moved or faulted during enable hold: %+v", s)
		}
		if *s.Axes[0].Brake == brakeWaitRelease {
			samples++
		}
		return *s.Axes[0].Brake == brakeReleased
	})
	if samples < 2 || released.Axes[0].ObservedNS < beforeArm+1000000000 {
		t.Fatalf("C05.13 barrier not observed: samples=%d before=%d released=%+v", samples, beforeArm, released)
	}
	// A rejected Start retires its handle. Prepare fresh work after the whole
	// group can execute together; rejection cannot silently queue motion.
	handle, err = c.PrepareTrajectory(context.Background(), 9802, 3, []Point{
		{Time: 0, Position: []float64{0, 0}},
		{Time: 200 * time.Millisecond, Position: []float64{0.003, 0.003}},
	})
	if err != nil {
		t.Fatal(err)
	}
	seq, err = c.StartTrajectory(context.Background(), handle)
	mustSubmit(t, seq, err)
	moved := wait(t, c, func(s Status) bool {
		return s.Axes[0].PositionCounts == 63 && s.Axes[1].PositionCounts == -63
	})
	if moved.ExecutionFaultReasons != 0 {
		t.Fatalf("released plan faulted: %+v", moved)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	for {
		motion, err := c.Motion()
		if err == nil && motion.TrajectoryID == handle && motion.State == 4 && motion.Done {
			break
		}
		select {
		case <-ctx.Done():
			t.Fatalf("released plan did not complete: %+v %v", motion, err)
		case <-ticker.C:
		}
	}
	seq, err = c.Stop(context.Background())
	mustSubmit(t, seq, err)
	holding := waitBrake(t, f, func(s brakeObservation) bool { return *s.Axes[0].Brake == brakeWaitHold })
	if holding.Armed != 0 || holding.EnableMask != 0 {
		t.Fatalf("Stop waited for brake: %+v", holding)
	}
	applied := waitBrake(t, f, func(s brakeObservation) bool {
		return s.TimeNS > holding.TimeNS && *s.Axes[0].Brake == brakeApplied
	})
	if applied.FaultReasons != 0 || applied.Axes[0].Position != 63 {
		t.Fatalf("disabled brake hold failed: %+v", applied)
	}
}

// The simulator applies its sag on the first inhibited cycle, inside the
// drive's 800 ms energized hold window. That window is recorded, never judged:
// no warning and no fault, the guard anchors at the torque-off rest, Stop still
// inhibits, and a later Arm needs no reset or restart.
func TestBrakeSagDuringHoldWindowIsRecordedNotWarned(t *testing.T) {
	f := startBrakeCore(t, true, true, 100, 800, "--sim-brake-sag-counts", "2000")
	arm(t, f.client, 3)
	waitBrake(t, f, func(s brakeObservation) bool { return *s.Axes[0].Brake == brakeReleased })
	seq, err := f.client.Stop(context.Background())
	mustSubmit(t, seq, err)
	stopped := waitBrake(t, f, func(s brakeObservation) bool {
		return s.Axes[0].Warning.TorqueOffNS != 0 && s.Axes[0].Position == 2000
	})
	w := stopped.Axes[0].Warning
	if stopped.Armed != 0 || stopped.EnableMask != 0 || stopped.FaultReasons != 0 || stopped.FaultMask != 0 {
		t.Fatalf("hold-window sag must not latch a fault: %+v", stopped)
	}
	if w.WarningSequence != 0 || w.Severity != "none" || w.Kind != "none" || w.Message != "" ||
		w.EnergizedPeakDisplacementCounts != 2000 || w.TorqueOffAnchorCounts != 2000 ||
		stopped.Axes[1].Warning.Severity != "none" {
		t.Fatalf("energized hold-window record: %+v", stopped.Axes)
	}
	arm(t, f.client, 3)
	rearmed := wait(t, f.client, allOperationEnabled)
	if rearmed.ExecutionFaultReasons != 0 || rearmed.SafetyFaultMask != 0 {
		t.Fatalf("Arm after a brake warning faulted: %+v", rearmed)
	}
}

func TestBrakeMappedFeedbackPreservesStatusword(t *testing.T) {
	// Unverified simulator feedback occupies statusword bit 10, outside the
	// DS402 state mask. Real drives must map their actual released-signal PDO.
	f := startBrakeCoreWithSignal(t, true, true, 100, 800, map[string]any{"semantic": "sw", "bit": 10})
	arm(t, f.client, 3)
	released := waitBrake(t, f, func(s brakeObservation) bool { return *s.Axes[0].Brake == brakeReleased })
	if released.FaultReasons != 0 || released.Axes[0].Statusword != 0x627 {
		t.Fatalf("mapped released bit lost ordinary statusword or blocked readiness: %+v", released)
	}
	seq, err := f.client.Stop(context.Background())
	mustSubmit(t, seq, err)
	holding := waitBrake(t, f, func(s brakeObservation) bool { return *s.Axes[0].Brake == brakeWaitHold })
	if holding.Axes[0].Statusword != 0x640 {
		t.Fatalf("hold feedback must remain released: %+v", holding)
	}
	applied := waitBrake(t, f, func(s brakeObservation) bool { return *s.Axes[0].Brake == brakeApplied })
	if applied.FaultReasons != 0 || applied.Axes[0].Statusword != 0x240 {
		t.Fatalf("applied feedback bit was not cleared: %+v", applied)
	}
}

func TestBrakeSagWithoutBrakeRetainsCoordinatePolicy(t *testing.T) {
	f := startBrakeCore(t, false, false, 100, 800, "--sim-brake-sag-counts", "2000")
	arm(t, f.client, 3)
	wait(t, f.client, allOperationEnabled)
	seq, err := f.client.Stop(context.Background())
	mustSubmit(t, seq, err)
	disabled := waitBrake(t, f, func(s brakeObservation) bool {
		return s.Armed == 0 && s.Axes[0].Position == 2000 && s.Axes[1].Position == 2000
	})
	if disabled.FaultReasons != 0 || *disabled.Axes[0].Brake != brakeReleased {
		t.Fatalf("unbraked sag changed policy: %+v", disabled)
	}
	arm(t, f.client, 3)
	rearmed := wait(t, f.client, allOperationEnabled)
	if rearmed.ExecutionFaultReasons != 0 || rearmed.Axes[0].PositionCounts != 2000 || rearmed.Axes[1].PositionCounts != 2000 {
		t.Fatalf("continuous coordinates failed to reconcile unbraked sag: %+v", rearmed)
	}
}

func TestBrakeProfileValidation(t *testing.T) {
	f := compiledForValidation(t, "simulation.json")
	policy := "1|100|100|1|di|5"
	profile := editProfileField(t, f.profile, 0, "brake", &policy)
	lines := strings.Split(profile, "\n")
	for i, line := range lines {
		if strings.HasPrefix(line, "tx_layout=") {
			lines[i] += ";di|24829|0|32"
			break
		}
	}
	profile = strings.Join(lines, "\n")
	if output, err := f.validate(t, profile); err != nil {
		t.Fatalf("mapped brake feedback rejected: %v %s", err, output)
	}
	for _, tolerance := range []string{"0", "1", "16"} {
		measured := policy + "|" + tolerance
		if output, err := f.validate(t, editProfileField(t, profile, 0, "brake", &measured)); err != nil {
			t.Fatalf("explicit brake tolerance rejected: %v %s", err, output)
		}
	}
	for _, invalid := range []string{
		"2|100|100|1|di|5", "1|-1|100|1|di|5", "1|4294967296|100|1|di|5",
		"1|100|100|2|di|5", "0|100|100|1|-|0", "0|100|100|0|di|5",
		"1|100|100|1|di|32", "1|100|100|1|missing|0", "1|100|100|1|-|1",
		"1|100|100|1|di|5|0|0", "1|100|100|1|di|-1",
		"1|100|100|1|di|5|-1", "1|100|100|1|di|5|1.5", "1|100|100|1|di|5|2147483648",
	} {
		t.Run(invalid, func(t *testing.T) {
			output, err := f.validate(t, editProfileField(t, profile, 0, "brake", &invalid))
			if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 {
				t.Fatalf("malformed brake accepted: %v %s", err, output)
			}
		})
	}
	if output, err := f.validate(t, editProfileField(t, f.profile, 0, "brake", nil)); err != nil {
		t.Fatalf("absent brake block must preserve native compatibility: %v %s", err, output)
	}
}

func TestBrakeDefaultsKeepExistingMachinesValid(t *testing.T) {
	// Keep synthetic defaults and the current physical bench override valid.
	// The bench explicitly disables its unqualified brake wiring; its source
	// profile still carries the measured 16-count hold-check descriptor.
	for _, machine := range []string{
		"simulation-home.json", "simulation-linear.json",
		"simulation-program.json", "simulation-scalars.json", "simulation.json",
		"machines/bench/rosie1420-six-motors.json",
	} {
		t.Run(machine, func(t *testing.T) {
			f := compiledForValidation(t, machine)
			want := "brake=0|100|100|0|-|0\n"
			if machine == "machines/bench/rosie1420-six-motors.json" {
				want = "brake=0|100|100|0|-|0|16\n"
			}
			if !strings.Contains(f.profile, want) {
				t.Fatalf("compiler omitted profile brake defaults: %s", f.profile)
			}
			if output, err := f.validate(t, f.profile); err != nil {
				t.Fatalf("unchanged machine rejected: %v %s", err, output)
			}
		})
	}
}
