//go:build linux

package ipcclient

import (
	"bufio"
	"context"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"sync"
	"testing"
	"time"
)

var diagnosticsBuild sync.Once
var diagnosticsBuildError error
var diagnosticsBuildOutput []byte

// Run the native cyclic fixture so publisher starvation and the exact 400-cycle
// burst are deterministic even on a descheduled simulation host.
func diagnosticsScenario(t *testing.T, scenario string) (produced, consumed, dropped uint64) {
	t.Helper()
	diagnosticsBuild.Do(func() {
		cmd := exec.Command("make", "build/cycle-capture-tests")
		cmd.Dir = ".."
		diagnosticsBuildOutput, diagnosticsBuildError = cmd.CombinedOutput()
	})
	if diagnosticsBuildError != nil {
		t.Fatalf("build native diagnostics fixture: %v: %s", diagnosticsBuildError, diagnosticsBuildOutput)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, "../build/cycle-capture-tests", scenario)
	output, err := cmd.CombinedOutput()
	if err != nil {
		t.Fatalf("native diagnostics fixture: %v: %s", err, output)
	}
	var cycles, faults uint64
	if _, err := fmt.Sscanf(strings.TrimSpace(string(output)),
		"diagnostics cycles=%d bus_faults=%d produced=%d consumed=%d dropped=%d",
		&cycles, &faults, &produced, &consumed, &dropped); err != nil {
		t.Fatalf("missing exact burst observation: %v: %s", err, output)
	}
	if cycles != 400 || faults != 400 {
		t.Fatalf("required 400 simulated cycles with a bus fault each cycle: %s", output)
	}
	t.Log(strings.TrimSpace(string(output)))
	return
}

func TestDiagnosticsQueueAbsorbsWorstCaseBurst(t *testing.T) {
	produced, consumed, dropped := diagnosticsScenario(t, "--diagnostics-burst")
	if produced != 10800 || consumed != 10800 || dropped != 0 {
		t.Fatalf("400 cycles * 27 records: produced=%d consumed=%d dropped=%d", produced, consumed, dropped)
	}
}

func TestDiagnosticsDropCounterIsExact(t *testing.T) {
	produced, consumed, dropped := diagnosticsScenario(t, "--diagnostics-overflow")
	if produced != 11000 || consumed != 10800 || dropped != 200 || dropped != produced-consumed {
		t.Fatalf("exact overflow accounting: produced=%d consumed=%d dropped=%d", produced, consumed, dropped)
	}
}

type diagnosticsMetrics struct {
	Cycles       uint64 `json:"rt_cycle_counter"`
	WorkNS       uint64 `json:"rt_max_cycle_work_ns"`
	Written      uint64 `json:"capture_records_written"`
	Dropped      uint64 `json:"capture_dropped"`
	Active       uint64 `json:"capture_active"`
	Failed       uint64 `json:"capture_write_failed"`
	BusOperation uint64 `json:"last_bus_failure_operation"`
}

func awaitDiagnostics(t *testing.T, f fixture, predicate func(diagnosticsMetrics) bool) diagnosticsMetrics {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	var pollResult diagnosticsMetrics
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		var metrics diagnosticsMetrics
		raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
		if err == nil && json.Unmarshal(raw, &metrics) == nil && predicate(metrics) {
			pollResult = metrics
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("diagnostics predicate not reached: %+v: %v", metrics, err)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func TestCycleCaptureProducesBoundedRecords(t *testing.T) {
	f := startCoreWithArgs(t, "simulation.json", "--capture-cycles", "100")
	metrics := awaitDiagnostics(t, f, func(m diagnosticsMetrics) bool { return m.Written == 100 && m.Active == 0 })
	if metrics.Dropped != 0 || metrics.Failed != 0 || metrics.WorkNS == 0 {
		t.Fatalf("capture lost evidence or work metric missing: %+v", metrics)
	}
	file, err := os.Open(filepath.Join(filepath.Dir(f.options.SocketPath), "cycle-capture.jsonl"))
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	scanner := bufio.NewScanner(file)
	count := 0
	var previousCycle, previousTime, previousSequence uint64
	for scanner.Scan() {
		var record struct {
			TimeNS uint64                       `json:"t_ns"`
			Seq    uint64                       `json:"seq"`
			Cycle  uint64                       `json:"cycle"`
			WorkNS uint64                       `json:"work_ns"`
			Axes   []map[string]json.RawMessage `json:"ax"`
		}
		if err := json.Unmarshal(scanner.Bytes(), &record); err != nil {
			t.Fatal(err)
		}
		if record.Cycle <= previousCycle || (count > 0 && record.Cycle != previousCycle+1) ||
			record.TimeNS <= previousTime || record.Seq <= previousSequence || record.Seq%2 != 0 || record.WorkNS == 0 {
			t.Fatalf("incoherent or missing cycle snapshot: %+v", record)
		}
		if len(record.Axes) != 2 {
			t.Fatalf("missing configured axes: %+v", record)
		}
		for _, axis := range record.Axes {
			for _, key := range []string{"i", "p", "tp", "sw", "er", "mfr"} {
				if _, ok := axis[key]; !ok {
					t.Fatalf("missing retained trace field %q: %s", key, scanner.Text())
				}
			}
		}
		previousCycle, previousTime, previousSequence = record.Cycle, record.TimeNS, record.Seq
		count++
	}
	if scanner.Err() != nil || count != 100 {
		t.Fatalf("capture must contain exactly 100 complete records: count=%d err=%v", count, scanner.Err())
	}
	// Observe further actual cycles to prove the exhausted arm does not restart.
	later := awaitDiagnostics(t, f, func(m diagnosticsMetrics) bool { return m.Cycles >= metrics.Cycles+100 })
	if later.Written != 100 || later.Active != 0 || later.Dropped != 0 {
		t.Fatalf("exhausted capture resumed: %+v", later)
	}
	t.Logf("capture records=100; simulation rt_max_cycle_work_ns=%d; subsequent_cycles=%d", later.WorkNS, later.Cycles-metrics.Cycles)
}

func TestCycleCaptureDisabledCreatesNoFile(t *testing.T) {
	f := startCore(t, "simulation.json")
	awaitDiagnostics(t, f, func(m diagnosticsMetrics) bool { return m.Cycles >= 100 })
	_, err := os.Stat(filepath.Join(filepath.Dir(f.options.SocketPath), "cycle-capture.jsonl"))
	if !os.IsNotExist(err) {
		t.Fatalf("unarmed capture created a file: %v", err)
	}
}

func TestRetiredFastTraceOptionsAreRejected(t *testing.T) {
	for _, flag := range []string{"--fast-trace-path", "--fast-trace-hz", "--fast-trace-axis-mask"} {
		output, err := exec.Command(simfixture.Locate("..", "rosie-rt-core-sim"), flag, "1").CombinedOutput()
		if err == nil || !strings.Contains(string(output), "unknown arg: "+flag) {
			t.Fatalf("retired option accepted: %s: %v: %s", flag, err, output)
		}
	}
}
