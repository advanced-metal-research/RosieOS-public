//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"
	"time"
)

// R29 observations describe host receipt evidence. They do not claim encoder
// acquisition time, a continuous coordinate anchor or physical Stop delivery.
// Pointers distinguish missing fields from the required initial zero values.
type cycleBusContractAxis struct {
	AxisStatus
	PDOFresh          *uint32 `json:"pdo_fresh"`
	PDOObservedTimeNS *uint64 `json:"pdo_observed_time_ns"`
}

type cycleBusContractObservation struct {
	Status
	CycleCount             uint64                 `json:"rt_cycle_counter"`
	StartupReady           uint32                 `json:"startup_ready"`
	StartupQueueSuppressed uint32                 `json:"startup_skip_domain_queue_active"`
	WKCActual              uint32                 `json:"wkc_actual"`
	WKCExpected            uint32                 `json:"wkc_expected"`
	WCState                uint32                 `json:"domain_wc_state"`
	Axes                   []cycleBusContractAxis `json:"axes"`
}

func waitCycleBusContractObservation(t *testing.T, f fixture, predicate func(cycleBusContractObservation) bool) cycleBusContractObservation {
	t.Helper()
	path := filepath.Join(filepath.Dir(f.options.SocketPath), "metrics.json")
	deadline := time.Now().Add(10 * time.Second)
	var observation cycleBusContractObservation
	var pollResult cycleBusContractObservation
	if !simfixture.PollUntil(deadline, 10*time.Millisecond, func() bool {
		raw, err := os.ReadFile(path)
		if err == nil {
			observation = cycleBusContractObservation{}
			if err := json.Unmarshal(raw, &observation); err != nil {
				t.Fatal(err)
			}
			now := monotonicNS()
			if observation.TimeNS == 0 || observation.TimeNS > now || now-observation.TimeNS > uint64(time.Second) {
				t.Fatalf("stale R29 metrics: %+v", observation)
			}
			if observation.Backend != "simulation" || observation.ConfigurationSHA256 != f.options.ConfigurationSHA256 || len(observation.Axes) != 2 {
				t.Fatalf("R29 fixture identity/geometry mismatch: %+v", observation)
			}
			for axis, values := range observation.Axes {
				if values.PDOFresh == nil || values.PDOObservedTimeNS == nil || *values.PDOFresh > 1 {
					t.Fatalf("axis %d missing valid PDO receipt metadata: %+v", axis, values)
				}
				// metrics.time_ns is sampled before its RT snapshot, so a current
				// PDO observation may be later than that wrapper timestamp. Bound
				// it against the reader's host clock, not that invalid ordering.
				if *values.PDOObservedTimeNS > now || (*values.PDOFresh != 0 &&
					(*values.PDOObservedTimeNS == 0 || now-*values.PDOObservedTimeNS > uint64(time.Second))) {
					t.Fatalf("axis %d has impossible/stale fresh host observation: %+v", axis, values)
				}
			}
			if predicate(observation) {
				pollResult = observation
				return true
			}
		} else if !errors.Is(err, os.ErrNotExist) {
			t.Fatal(err)
		}
		if time.Now().After(deadline) {
			t.Fatalf("R29 observation deadline: %+v", observation)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func cycleBusFreshMask(s cycleBusContractObservation) uint32 {
	var mask uint32
	for axis, values := range s.Axes {
		if *values.PDOFresh != 0 {
			mask |= 1 << axis
		}
	}
	return mask
}

func TestCycleBusRejectsUnqualifiedRoundRobinBeforeStartup(t *testing.T) {
	compiled := compiledForValidation(t, "simulation.json")
	for _, test := range []struct {
		name  string
		extra []string
	}{
		{"without_split", nil},
		{"with_split", []string{"--split-domains-per-axis"}},
	} {
		t.Run(test.name, func(t *testing.T) {
			directory := t.TempDir()
			socket := filepath.Join(directory, "ipc.sock")
			args := append(append([]string(nil), compiled.args...), "--socket-path", socket)
			args = append(args, test.extra...)
			args = append(args, "--queue-split-domains-round-robin")
			ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
			defer cancel()
			output, err := exec.CommandContext(ctx, compiled.binary, args...).CombinedOutput()
			var exit *exec.ExitError
			if !errors.As(err, &exit) || exit.ExitCode() != 2 ||
				!strings.Contains(string(output), "--queue-split-domains-round-robin is unsupported") {
				t.Fatalf("unqualified scheduling was not explicitly rejected: %v: %s", err, output)
			}
			for _, path := range []string{socket, filepath.Join(directory, "metrics.json")} {
				if _, err := os.Stat(path); !errors.Is(err, os.ErrNotExist) {
					t.Fatalf("rejected CLI reached runtime startup (%s): %v", path, err)
				}
			}
		})
	}
}

// The ordinary fixture deliberately waits for verified configuration, which
// would hide the initial no-receipt state. Use its unchanged compiler and
// process conventions but observe metrics before establishing a client.
func startUnreadyCycleBusCore(t *testing.T, extraArgs ...string) fixture {
	t.Helper()
	compiled := compiledForValidation(t, "simulation.json")
	directory, err := os.MkdirTemp("", "r29-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(directory) })

	socket := filepath.Join(directory, "ipc.sock")
	args := append(append([]string(nil), compiled.args...), "--socket-path", socket)
	process := exec.Command(compiled.binary, append(args, extraArgs...)...)
	simfixture.SpawnLogged(t, process, filepath.Join(directory, "core.log"), 3*time.Second, 9000, nil)

	return fixture{options: Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(filepath.Dir(compiled.path)), Backend: "simulation"}}
}

func TestCycleBusStartupQueueSuppressionCannotCreateFreshInput(t *testing.T) {
	f := startUnreadyCycleBusCore(t, "--split-domains-per-axis", "--startup-skip-domain-queue-ms", "1400")
	var firstSuppressed cycleBusContractObservation
	sawTwoSuppressedSnapshots := false
	recovered := waitCycleBusContractObservation(t, f, func(s cycleBusContractObservation) bool {
		if s.StartupQueueSuppressed != 0 && s.CycleCount != 0 {
			if cycleBusFreshMask(s) != 0 || s.StartupReady != 0 || s.ConfigVerifiedMask != 0 ||
				s.ExecutionFaultReasons != 0 || s.SafetyFaultMask != 0 || s.Armed != 0 || s.AxisEnableMask != 0 {
				t.Fatalf("startup clock-only traffic fabricated readiness or lost-health fault: %+v", s)
			}
			for axis, values := range s.Axes {
				if *values.PDOObservedTimeNS != 0 {
					t.Fatalf("axis %d acquired an observation without its first domain exchange: %+v", axis, values)
				}
			}
			if firstSuppressed.TimeNS == 0 {
				firstSuppressed = s
			} else if s.TimeNS > firstSuppressed.TimeNS && s.CycleCount > firstSuppressed.CycleCount {
				sawTwoSuppressedSnapshots = true
			}
		}
		return s.StartupQueueSuppressed == 0 && s.StartupReady == 1 && s.ConfigVerifiedMask == 3 &&
			cycleBusFreshMask(s) == 3 && s.WCState == 2 && s.WKCActual == s.WKCExpected
	})
	if !sawTwoSuppressedSnapshots {
		t.Fatal("test did not observe queue suppression across distinct RT/metrics publications")
	}
	if recovered.ExecutionFaultReasons != 0 || recovered.SafetyFaultMask != 0 ||
		*recovered.Axes[0].PDOObservedTimeNS != *recovered.Axes[1].PDOObservedTimeNS {
		t.Fatalf("initial full-group exchange failed to establish healthy coherent host observations: %+v", recovered)
	}
}

func TestCycleBusReceiptTimesFreezeDuringFiniteInputFailure(t *testing.T) {
	for _, test := range []struct {
		name        string
		fromFlag    string
		countFlag   string
		wantedFault uint32
	}{
		{"receive", "--sim-receive-fail-from-call", "--sim-receive-fail-for-calls", faultBusTransport},
		{"wkc", "--sim-wkc-incomplete-from-call", "--sim-wkc-incomplete-for-calls", 1 << 7},
	} {
		t.Run(test.name, func(t *testing.T) {
			f := startCoreWithArgs(t, "simulation.json", "--split-domains-per-axis",
				test.fromFlag, "2000", test.countFlag, "1200")
			healthy := waitCycleBusContractObservation(t, f, func(s cycleBusContractObservation) bool {
				return s.StartupReady == 1 && s.ConfigVerifiedMask == 3 && cycleBusFreshMask(s) == 3
			})
			if healthy.ExecutionFaultReasons != 0 || healthy.SafetyFaultMask != 0 || healthy.Armed != 0 || healthy.AxisEnableMask != 0 {
				t.Fatalf("finite input failure must follow a healthy disarmed state: %+v", healthy)
			}
			faulted := waitCycleBusContractObservation(t, f, func(s cycleBusContractObservation) bool {
				return cycleBusFreshMask(s) == 0 && s.StartupReady == 0 && s.ExecutionFaultReasons&test.wantedFault != 0
			})
			for axis, values := range faulted.Axes {
				if *values.PDOObservedTimeNS < *healthy.Axes[axis].PDOObservedTimeNS {
					t.Fatalf("axis %d receipt time regressed on input loss", axis)
				}
			}
			var frozen cycleBusContractObservation
			recovered := waitCycleBusContractObservation(t, f, func(s cycleBusContractObservation) bool {
				if cycleBusFreshMask(s) == 0 {
					for axis, values := range s.Axes {
						if *values.PDOObservedTimeNS != *faulted.Axes[axis].PDOObservedTimeNS {
							t.Fatalf("axis %d renewed its receipt time during input failure: first=%d later=%d", axis,
								*faulted.Axes[axis].PDOObservedTimeNS, *values.PDOObservedTimeNS)
						}
					}
					if s.TimeNS > faulted.TimeNS && s.CycleCount > faulted.CycleCount {
						frozen = s
					}
				}
				return cycleBusFreshMask(s) == 3 && s.StartupReady == 1 && s.WCState == 2 &&
					s.WKCActual == s.WKCExpected && s.TimeNS > faulted.TimeNS
			})
			if frozen.TimeNS == 0 {
				t.Fatal("input failure was not observed across distinct publications")
			}
			for axis, values := range recovered.Axes {
				if *values.PDOObservedTimeNS <= *faulted.Axes[axis].PDOObservedTimeNS {
					t.Fatalf("axis %d full-group recovery did not advance its receipt time", axis)
				}
			}
			if *recovered.Axes[0].PDOObservedTimeNS != *recovered.Axes[1].PDOObservedTimeNS ||
				recovered.ExecutionFaultReasons&test.wantedFault == 0 || recovered.Armed != 0 || recovered.AxisEnableMask != 0 {
				t.Fatalf("fresh input recovery must not silently clear the execution fault or enable motion: %+v", recovered)
			}
		})
	}
}
