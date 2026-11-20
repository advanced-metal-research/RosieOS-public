//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strconv"
	"strings"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

// Use the host's allowed set so restricted cpusets never turn this regression
// into an accidental test of an unavailable CPU.
func affinityTestCPUs(t *testing.T) []int {
	t.Helper()
	var allowed unix.CPUSet
	if err := unix.SchedGetaffinity(0, &allowed); err != nil {
		t.Fatal(err)
	}
	cpus := []int{}
	for cpu := 0; cpu < 1024; cpu++ {
		if allowed.IsSet(cpu) {
			cpus = append(cpus, cpu)
		}
	}
	if len(cpus) < 2 {
		t.Skip("housekeeping isolation requires at least two allowed CPUs")
	}
	return cpus
}

func TestHousekeepingThreadsAvoidRtCpu(t *testing.T) {
	restore, err := simfixture.SuspendSimulationCPUReservation()
	if err != nil {
		t.Fatal(err)
	}
	defer restore()
	cpus := affinityTestCPUs(t)
	for _, explicit := range []bool{false, true} {
		t.Run(fmt.Sprintf("explicit=%t", explicit), func(t *testing.T) {
			rt := cpus[0]
			args := []string{"--rt-cpu", strconv.Itoa(rt), "--capture-cycles", "100"}
			if explicit {
				args = append(args, "--housekeeping-cpus", strconv.Itoa(cpus[1]))
			}
			f := startCoreWithArgs(t, "simulation.json", args...)
			rawConn, err := f.client.conn.SyscallConn()
			if err != nil {
				t.Fatal(err)
			}
			var peer *unix.Ucred
			var peerErr error
			if err := rawConn.Control(func(fd uintptr) { peer, peerErr = unix.GetsockoptUcred(int(fd), unix.SOL_SOCKET, unix.SO_PEERCRED) }); err != nil {
				t.Fatal(err)
			}
			if peerErr != nil {
				t.Fatal(peerErr)
			}
			// Unverified observation deadline: fail after 5000 ms without all seven flags.
			deadline := time.Now().Add(5 * time.Second)
			var status Status
			if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
				raw, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json"))
				if err == nil {
					err = json.Unmarshal(raw, &status)
				}
				ready := err == nil
				for _, name := range []string{"main", "rt_cycle", "helper", "commissioning", "bulk", "publisher", "capture_drain"} {
					ready = ready && status.AffinityApplied[name]
				}
				if ready {
					return true
				}
				if time.Now().After(deadline) {
					t.Fatalf("affinity not published: %+v: %v", status, err)
				}
				return false
			}) {
				t.Fatal("observation deadline exceeded")
			}
			if status.RTCPU != uint32(rt) || status.HousekeepingCPUs == "" {
				t.Fatalf("missing CPU metrics: %+v", status)
			}
			if explicit && status.HousekeepingCPUs != strconv.Itoa(cpus[1]) {
				t.Fatalf("explicit mask changed: %s", status.HousekeepingCPUs)
			}
			tasks, err := filepath.Glob(fmt.Sprintf("/proc/%d/task/*/status", peer.Pid))
			if err != nil {
				t.Fatal(err)
			}
			// Measured daemon composition: main, cycle, commissioning, metrics,
			// telemetry dump, helper and bulk each own one thread.
			if len(tasks) != 7 {
				t.Fatalf("expected seven daemon threads including telemetry-dump, got %d", len(tasks))
			}
			rtThreads := 0
			dumpThreads := 0
			for _, path := range tasks {
				raw, err := os.ReadFile(path)
				if err != nil {
					t.Fatal(err)
				}
				fields := map[string]string{}
				for _, line := range strings.Split(string(raw), "\n") {
					key, value, found := strings.Cut(line, ":")
					if found {
						fields[key] = strings.TrimSpace(value)
					}
				}
				mask := fields["Cpus_allowed_list"]
				if fields["Name"] == "telemetry-dump" {
					dumpThreads++
				}
				if fields["Name"] == "rt-cycle" {
					rtThreads++
					if mask != strconv.Itoa(rt) {
						t.Fatalf("RT affinity = %s, expected CPU %d", mask, rt)
					}
					continue
				}
				if mask != status.HousekeepingCPUs {
					t.Fatalf("%s affinity %s differs from published %s", fields["Name"], mask, status.HousekeepingCPUs)
				}
				for _, span := range strings.Split(mask, ",") {
					ends := strings.Split(span, "-")
					low, err := strconv.Atoi(ends[0])
					if err != nil {
						t.Fatal(err)
					}
					high := low
					if len(ends) == 2 {
						high, err = strconv.Atoi(ends[1])
						if err != nil {
							t.Fatal(err)
						}
					}
					if rt >= low && rt <= high {
						t.Fatalf("%s includes RT CPU %d", fields["Name"], rt)
					}
				}
			}
			if rtThreads != 1 {
				t.Fatalf("expected one RT thread, got %d", rtThreads)
			}
			if dumpThreads != 1 {
				t.Fatalf("expected one telemetry dump thread, got %d", dumpThreads)
			}
		})
	}
}

func TestHousekeepingCpusRejectsRtCpu(t *testing.T) {
	cpus := affinityTestCPUs(t)
	f := startCoreWithArgs(t, "simulation.json", "--rt-cpu", strconv.Itoa(cpus[0]))
	dir := filepath.Dir(f.options.SocketPath)
	raw, err := os.ReadFile(filepath.Join(dir, "config", f.options.ConfigurationSHA256, "argv.json"))
	if err != nil {
		t.Fatal(err)
	}
	var args []string
	if err := json.Unmarshal(raw, &args); err != nil {
		t.Fatal(err)
	}
	args = append(args, "--rt-cpu", strconv.Itoa(cpus[0]), "--housekeeping-cpus", strconv.Itoa(cpus[0]), "--socket-path", filepath.Join(dir, "reject.sock"))
	// Unverified startup deadline: terminate after 5000 ms if rejection hangs.
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	binary := simfixture.Locate("..", "rosie-rt-core-sim")
	output, err := exec.CommandContext(ctx, binary, args...).CombinedOutput()
	var exit *exec.ExitError
	if !errors.As(err, &exit) || exit.ExitCode() != 2 {
		t.Fatalf("expected exit 2, got %v: %s", err, output)
	}
	if !strings.Contains(string(output), fmt.Sprintf("CPU %d is the RT CPU", cpus[0])) {
		t.Fatalf("missing offending CPU: %s", output)
	}
	if _, err := os.Stat(filepath.Join(dir, "reject.sock")); !os.IsNotExist(err) {
		t.Fatalf("invalid cpuset created a server: %v", err)
	}
}
