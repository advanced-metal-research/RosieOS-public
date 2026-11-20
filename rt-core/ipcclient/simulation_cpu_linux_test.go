//go:build linux

package ipcclient

import (
	"bytes"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/internal/simfixture"
)

// TestSimulationCPUFallback exercises the daemon with a compiled, unavailable
// CPU, preventing fixture selection from hiding a broken runtime fallback.
// This is the intentional bypass of simfixture's selecting compiler.
func TestSimulationCPUFallback(t *testing.T) {
	restore, err := simfixture.SuspendSimulationCPUReservation()
	if err != nil {
		t.Fatal(err)
	}
	defer restore()
	cpus := affinityTestCPUs(t)
	configured := 0
	for configured == cpus[0] || configured == cpus[1] {
		configured++
	}
	root, err := filepath.Abs("..")
	simfixture.Must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "cpu-")
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(build) })
	sockets, err := os.MkdirTemp("", "cpu-")
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(sockets) })
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation.json"))
	simfixture.Must(t, err)
	var machine map[string]any
	simfixture.Must(t, json.Unmarshal(raw, &machine))
	machine["runtime"].(map[string]any)["cpu"] = configured
	raw, err = json.Marshal(machine)
	simfixture.Must(t, err)
	machinePath := filepath.Join(build, "machine.json")
	simfixture.Must(t, os.WriteFile(machinePath, raw, 0600))
	// Migration warnings on stderr cannot become part of the stdout path.
	command := exec.Command(filepath.Join(root, "build/rtctl"), "compile", "--config", machinePath, "--out", build)
	var stderr bytes.Buffer
	command.Stderr = &stderr
	raw, err = command.Output()
	if err != nil {
		t.Fatalf("compile: %v: %s%s", err, raw, stderr.Bytes())
	}
	compiled := strings.TrimSpace(string(raw))
	raw, err = os.ReadFile(filepath.Join(compiled, "argv.json"))
	simfixture.Must(t, err)
	var argv []string
	simfixture.Must(t, json.Unmarshal(raw, &argv))
	args := []string{"-c", fmt.Sprintf("%d,%d", cpus[0], cpus[1]), simfixture.Locate(root, "rosie-rt-core-sim")}
	args = append(args, argv...)
	args = append(args, "--socket-path", filepath.Join(sockets, "ipc.sock"))
	process := exec.Command("taskset", args...)
	warning := fmt.Sprintf("WARNING: simulation RT CPU %d is outside the allowed cpuset; using RT CPU %d", configured, cpus[1])
	// Unverified watchdog: 5000 ms; success requires published affinity and kernel masks.
	simfixture.SpawnLogged(t, process, filepath.Join(build, "core.log"), 5*time.Second, 0, simfixture.CheckExit(t), func(raw []byte) {
		if strings.Count(string(raw), warning) != 1 {
			t.Errorf("expected exactly one fallback warning %q: %s", warning, raw)
		}
	})
	// Unverified observation ceiling and cadence: 5000 ms and 5 ms.
	simfixture.PollTest(t, 5*time.Second, 5*time.Millisecond, func() bool {
		raw, err := os.ReadFile(filepath.Join(sockets, "metrics.json"))
		var status Status
		if err != nil || json.Unmarshal(raw, &status) != nil || status.ConfigVerifiedMask != 3 {
			return false
		}
		if status.RTCPU != uint32(cpus[1]) || status.HousekeepingCPUs != strconv.Itoa(cpus[0]) {
			t.Fatalf("fallback published wrong CPUs: rt=%d housekeeping=%s", status.RTCPU, status.HousekeepingCPUs)
		}
		return status.AffinityApplied["rt_cycle"] && status.AffinityApplied["main"]
	})
	tasks, err := filepath.Glob(fmt.Sprintf("/proc/%d/task/*/status", process.Process.Pid))
	simfixture.Must(t, err)
	rtThreads := 0
	for _, path := range tasks {
		raw, err := os.ReadFile(path)
		simfixture.Must(t, err)
		fields := map[string]string{}
		for _, line := range strings.Split(string(raw), "\n") {
			key, value, found := strings.Cut(line, ":")
			if found {
				fields[key] = strings.TrimSpace(value)
			}
		}
		if fields["Name"] == "rt-cycle" {
			rtThreads++
			if fields["Cpus_allowed_list"] != strconv.Itoa(cpus[1]) {
				t.Fatalf("RT thread bound to %s", fields["Cpus_allowed_list"])
			}
		}
	}
	if rtThreads != 1 {
		t.Fatalf("expected one RT thread, got %d", rtThreads)
	}
}
