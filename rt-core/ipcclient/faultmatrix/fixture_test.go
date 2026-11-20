//go:build linux

package faultmatrix

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"syscall"
	"testing"
	"time"

	ipc "rosieos/rt-core/ipcclient"
	"rosieos/rt-core/tools/rtctl"
)

// WSL TestMatrixPublication measured a maximum 200,216,563 ns gap over 6 samples.
// Unverified host publication allowance: two 200,000,000 ns metrics periods
// (last healthy and first fault samples) plus 50,000,000 ns scheduling margin.
// TestMatrixPublication measures the actual gaps without deriving its bound.
const publicationNS uint64 = 450_000_000

// Unverified fixture watchdog; never substitutes for a daemon timestamp bound.
const watchdog = 8 * time.Second

type fixture struct {
	t                                              *testing.T
	root, dir, socketDir, binary, profile, logPath string
	args                                           []string
	compiled                                       rtctl.Compiled
	cycle                                          uint64
	c                                              *ipc.Client
	options                                        ipc.Options
	process                                        *exec.Cmd
	log                                            *os.File
}

// compileFixture keeps compiled artifacts in build and live logs/sockets under
// TMPDIR, avoiding concurrent /mnt/c reads and writes to frozen source fixtures.
func compileFixture(t *testing.T, machine string, edit func(map[string]any)) *fixture {
	t.Helper()
	root, err := filepath.Abs("../..")
	must(t, err)
	raw, err := os.ReadFile(simfixture.MachineConfigPath(root, machine))
	must(t, err)
	var config map[string]any
	must(t, json.Unmarshal(raw, &config))
	if edit != nil {
		edit(config)
	}
	raw, err = json.Marshal(config)
	must(t, err)
	compiled, err := simfixture.CompileRaw(raw, filepath.Join(root, "config/drives"), rtctl.Compile)
	must(t, err)
	must(t, os.MkdirAll(filepath.Join(root, "build"), 0700))
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "faults-")
	must(t, err)
	if os.Getenv("TMPDIR") == "" {
		t.Fatal("TMPDIR is required for socket storage")
	}
	socketDir, err := os.MkdirTemp("", "fm-")
	must(t, err)
	f := &fixture{t: t, root: root, dir: dir, socketDir: socketDir, compiled: compiled, cycle: uint64(config["cycle_ns"].(float64))}
	f.binary = simfixture.Locate(root, "rosie-rt-core-sim")
	if binary := os.Getenv("RTCORE_TEST_BINARY"); binary != "" {
		f.binary = binary
	}
	f.profile = filepath.Join(dir, "axes.conf")
	f.logPath = filepath.Join(socketDir, "daemon.log")
	f.options = ipc.Options{SocketPath: filepath.Join(socketDir, "s"), ConfigurationSHA256: compiled.SHA256, Backend: "simulation", SimulationTest: true}
	f.args = append(append([]string{}, compiled.Argv...), "--axis-profiles", f.profile, "--configuration-hash", compiled.SHA256, "--socket-path", f.options.SocketPath)
	must(t, os.WriteFile(f.profile, []byte(compiled.AxisProfiles), 0600))
	t.Cleanup(func() {
		f.shutdown()
		if t.Failed() {
			raw, _ := os.ReadFile(f.logPath)
			t.Logf("daemon: %s", raw)
		}
		_ = os.RemoveAll(dir)
		_ = os.RemoveAll(socketDir)
	})
	return f
}

func start(t *testing.T, args ...string) *fixture {
	f := compileFixture(t, "simulation.json", nil)
	f.launch(true, args...)
	return f
}

func (f *fixture) launch(oracle bool, args ...string) {
	t := f.t
	t.Helper()
	f.options.SimulationTest = oracle
	var err error
	f.log, err = os.Create(f.logPath)
	must(t, err)
	f.process = exec.Command(f.binary, append(append([]string{}, f.args...), args...)...)
	f.process.Stdout, f.process.Stderr = f.log, f.log
	f.process.Env = append(os.Environ(), "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED=0")
	launched := ipc.HostMonotonicNS()
	must(t, f.process.Start())
	deadline := time.Now().Add(watchdog)
	for {
		raw, _ := os.ReadFile(filepath.Join(f.socketDir, "metrics.json"))
		var s ipc.Status
		if json.Unmarshal(raw, &s) == nil && s.TimeNS >= launched && s.ConfigVerifiedMask != 0 {
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("daemon failed to verify configuration")
		}
		pause()
	}
	ctx, cancel := context.WithTimeout(context.Background(), watchdog)
	defer cancel()
	f.c, err = ipc.Dial(ctx, f.options)
	must(t, err)
}

func (f *fixture) shutdown() {
	if f.c != nil {
		_ = f.c.Close()
		f.c = nil
	}
	if f.process == nil {
		return
	}
	_ = f.process.Process.Signal(syscall.SIGTERM)
	done := make(chan error, 1)
	go func() { done <- f.process.Wait() }()
	select {
	case <-done:
	case <-time.After(watchdog):
		_ = f.process.Process.Kill()
		<-done
		f.t.Error("daemon shutdown watchdog")
	}
	f.process = nil
	_ = f.log.Close()
}

var must = simfixture.Must

func pause()                          { timer := time.NewTimer(time.Millisecond); <-timer.C }
func (f *fixture) status() ipc.Status { f.t.Helper(); s, err := f.c.Status(); must(f.t, err); return s }

// until requires a new daemon observation within an absolute ns deadline;
// wall-clock polling only limits a stalled publisher and can never pass a row.
func (f *fixture) until(after, bound uint64, predicate func(ipc.Status) bool) ipc.Status {
	f.t.Helper()
	deadline := time.Now().Add(watchdog)
	for {
		s := f.status()
		if s.TimeNS >= after {
			if s.TimeNS-after > bound {
				f.t.Fatalf("status exceeded bound: elapsed_ns=%d bound_ns=%d status=%+v", s.TimeNS-after, bound, s)
			}
			if predicate(s) {
				return s
			}
		}
		if time.Now().After(deadline) {
			f.t.Fatal("status publisher watchdog")
		}
		pause()
	}
}
func enabled(s ipc.Status) bool {
	if s.Armed != 1 {
		return false
	}
	return ipc.AllOperationEnabled(s)
}
func (f *fixture) command(sequence uint64, err error) ipc.CommandResult {
	f.t.Helper()
	must(f.t, err)
	ctx, cancel := context.WithTimeout(context.Background(), watchdog)
	defer cancel()
	r, err := f.c.WaitResult(ctx, sequence)
	must(f.t, err)
	return r
}
func (f *fixture) arm() ipc.Status {
	f.t.Helper()
	at := ipc.HostMonotonicNS()
	f.command(f.c.Enable(context.Background(), uint32(1<<f.c.NumAxes())-1))
	f.command(f.c.Arm(context.Background()))
	return f.until(at, 2_000_000_000, enabled)
}
func (f *fixture) prepare(id uint64, end float64, duration time.Duration) uint64 {
	f.t.Helper()
	s := f.status()
	q := make([]float64, len(s.Axes))
	target := make([]float64, len(q))
	for i, a := range s.Axes {
		sign := 1.0
		if i == 1 {
			sign = -1
		}
		q[i] = float64(a.PositionCounts) * 6.283185307179586 / 131072 * sign
		target[i] = q[i] + end
	}
	ctx, cancel := context.WithTimeout(context.Background(), watchdog)
	defer cancel()
	h, err := f.c.PrepareTrajectory(ctx, id, uint32(1<<len(q))-1, []ipc.Point{{Time: 0, Position: q}, {Time: duration, Position: target}})
	must(f.t, err)
	return h
}
func (f *fixture) execute(h uint64) {
	f.t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), watchdog)
	defer cancel()
	_, err := f.c.StartTrajectory(ctx, h)
	must(f.t, err)
}
func reject(t *testing.T, err error, reason uint32) {
	t.Helper()
	var r *ipc.RejectedError
	if !errors.As(err, &r) || r.Result.Reason != reason {
		t.Fatalf("want native reason %d, got %v", reason, err)
	}
}
func inhibited(t *testing.T, s ipc.Status, bit uint32) {
	t.Helper()
	if s.Armed != 0 || s.AxisEnableMask != 0 || s.ExecutionFaultReasons&bit != bit {
		t.Fatalf("expected inhibition and fault %#x: %+v", bit, s)
	}
}
func (f *fixture) hold(s ipc.Status, ns uint64, tolerance int32) {
	f.t.Helper()
	f.until(s.TimeNS, ns+publicationNS, func(next ipc.Status) bool {
		for i, a := range next.Axes {
			d := a.PositionCounts - s.Axes[i].PositionCounts
			if d < -tolerance || d > tolerance {
				f.t.Fatalf("axis %d moved %d counts; allowed %d", i, d, tolerance)
			}
		}
		return next.TimeNS >= s.TimeNS+ns
	})
}
func (f *fixture) event(name string) (uint64, int32) {
	f.t.Helper()
	deadline := time.Now().Add(watchdog)
	for {
		raw, err := os.ReadFile(f.logPath)
		must(f.t, err)
		for _, line := range strings.Split(string(raw), "\n") {
			if strings.HasPrefix(line, name+" time_ns=") {
				var at uint64
				var counts int32
				_, err := fmt.Sscanf(line, name+" time_ns=%d position_counts=%d", &at, &counts)
				if at != 0 {
					return at, counts
				}
				must(f.t, err)
			}
		}
		if time.Now().After(deadline) {
			f.t.Fatalf("missing injection event %s", name)
		}
		pause()
	}
}
