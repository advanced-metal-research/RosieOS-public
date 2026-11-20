//go:build linux

package oracle

import (
	"context"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"testing"
	"time"

	ipc "rosieos/rt-core/ipcclient"
)

var ctx = context.Background()

type fixture struct {
	t                           *testing.T
	c                           *ipc.Client
	process                     *exec.Cmd
	root, dir, compiled, socket string
	options                     ipc.Options
}

var must = simfixture.Must

func submitted(t *testing.T, _ uint64, err error) { t.Helper(); must(t, err) }

var poll = func(t *testing.T, budget time.Duration, predicate func() bool) {
	t.Helper()
	simfixture.PollTest(t, budget, time.Millisecond, predicate)
}

// start compiles the checked-in machine with rtctl and waits for verified input,
// preventing a test from passing against a synthetic client-only configuration.
func start(t *testing.T, machine string) *fixture {
	t.Helper()
	root, err := filepath.Abs("../..")
	must(t, err)
	dir, err := os.MkdirTemp("", "o-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "oracle-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	compiled, args, err := simfixture.Compile(root, filepath.Join(root, "config/machines/simulation", machine), filepath.Join(build, "config"))
	simfixture.Must(t, err)
	socket := filepath.Join(dir, "ipc.sock")
	binary := simfixture.Locate(root, "rosie-rt-core-sim")
	process := exec.Command(binary, append(args, "--socket-path", socket)...)
	simfixture.SpawnLogged(t, process, filepath.Join(build, "core.log"), 3*time.Second, 0, simfixture.CheckExit(t))
	f := &fixture{t: t, process: process, root: root, dir: dir, compiled: compiled, socket: socket}
	t.Cleanup(func() {
		if f.c != nil {
			f.c.Close()
		}
	})
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 5*time.Second, time.Millisecond, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask != 0 })
	f.options = ipc.Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation", SimulationTest: true}
	f.connect()
	f.waitWithin(5*time.Second, func(s ipc.Status) bool {
		return s.LeaseValid != 0 && s.ConfigVerifiedMask == uint32(1<<f.c.NumAxes())-1
	})
	return f
}
func (f *fixture) connect() {
	poll(f.t, time.Second, func() bool { var err error; f.c, err = ipc.Dial(ctx, f.options); return err == nil })
}
func (f *fixture) wait(predicate func(ipc.Status) bool) ipc.Status {
	f.t.Helper()
	return f.waitWithin(3*time.Second, predicate)
}
func (f *fixture) waitWithin(budget time.Duration, predicate func(ipc.Status) bool) ipc.Status {
	f.t.Helper()
	c, cancel := context.WithTimeout(ctx, budget)
	defer cancel()
	s, err := f.c.Wait(c, predicate)
	if err != nil {
		f.t.Fatalf("wait: %v; state=%+v", err, s)
	}
	return s
}
func (f *fixture) status() ipc.Status { s, err := f.c.Status(); must(f.t, err); return s }
func (f *fixture) arm(mask uint32) ipc.Status {
	seq, err := f.c.Enable(ctx, mask)
	submitted(f.t, seq, err)
	seq, err = f.c.Arm(ctx)
	submitted(f.t, seq, err)
	return f.wait(func(s ipc.Status) bool { return s.Armed == 1 })
}
func (f *fixture) stop() {
	seq, err := f.c.Stop(ctx)
	submitted(f.t, seq, err)
	f.wait(func(s ipc.Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
}
func (f *fixture) prepare(id uint64, points []ipc.Point) uint64 {
	h, err := f.c.PrepareTrajectory(ctx, id, 3, points)
	must(f.t, err)
	return h
}
func (f *fixture) trajectory(id uint64, points []ipc.Point) uint64 {
	seq, err := f.c.Trajectory(ctx, id, 3, points)
	must(f.t, err)
	return seq
}
func (f *fixture) motion() ipc.MotionState { m, err := f.c.Motion(); must(f.t, err); return m }
func (f *fixture) rawStatus() map[string]any {
	var s map[string]any
	poll(f.t, time.Second, func() bool {
		raw, err := os.ReadFile(filepath.Join(f.dir, "metrics.json"))
		return err == nil && json.Unmarshal(raw, &s) == nil
	})
	return s
}

// stable checks every observation throughout the original negative assertion
// window, preventing a transient replay from hiding behind one final sample.
func (f *fixture) stable(window time.Duration, predicate func(ipc.Status) bool) {
	f.t.Helper()
	deadline := time.Now().Add(window)
	for {
		s := f.status()
		if !predicate(s) {
			f.t.Fatalf("invariant violated: %+v", s)
		}
		if !time.Now().Before(deadline) {
			return
		}
		time.Sleep(time.Millisecond)
	}
}
func rejected(t *testing.T, err error, reason uint32) {
	t.Helper()
	var r *ipc.RejectedError
	if !errors.As(err, &r) {
		t.Fatalf("expected native rejection: %v", err)
	}
	if reason != 0 && r.Result.Reason != reason {
		t.Fatalf("reason=%d, want %d", r.Result.Reason, reason)
	}
}
func points(seconds, q0, q1 float64, velocity []float64) []ipc.Point {
	return []ipc.Point{{Position: []float64{0, 0}, Velocity: velocity}, {Time: time.Duration(seconds * 1e9), Position: []float64{q0, q1}, Velocity: func() []float64 {
		if velocity != nil {
			return []float64{0, 0}
		}
		return nil
	}()}}
}
