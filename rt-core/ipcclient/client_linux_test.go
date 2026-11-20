//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"
	"time"
)

type fixture struct {
	client  *Client
	options Options
}

func startCore(t *testing.T, machine string) fixture {
	t.Helper()
	return startCoreWithArgs(t, machine)
}

func startCoreWithArgs(t *testing.T, machine string, extraArgs ...string) fixture {
	t.Helper()
	return startCoreWithRole(t, machine, true, extraArgs...)
}

func startCoreWithRole(t *testing.T, machine string, simulationTest bool, extraArgs ...string) fixture {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	binary := simfixture.Locate(root, "rosie-rt-core-sim")
	if _, err := os.Stat(binary); err != nil {
		t.Fatal("run make sim first: ", err)
	}
	// Keep the Unix socket path below sockaddr_un's limit even when TMPDIR
	// lives inside the workspace and the regression has a descriptive name.
	dir, err := os.MkdirTemp("", "ipc-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	compiled, args, err := simfixture.Compile(root, simfixture.MachineConfigPath(root, machine), filepath.Join(dir, "config"))
	simfixture.Must(t, err)
	if len(extraArgs) != 0 {
		args = legacyIdentityArgs(args)
	}
	socket := filepath.Join(dir, "ipc.sock")

	args = append(args, "--socket-path", socket)
	process := exec.Command(binary, append(args, extraArgs...)...)
	simfixture.SpawnLogged(t, process, filepath.Join(dir, "core.log"), 3*time.Second, 9000, nil)

	var client *Client
	t.Cleanup(func() {
		if client != nil {
			_ = client.Close()
		}
	})
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 5*time.Second, 20*time.Millisecond, func(status simfixture.Verified) bool { return status.ConfigVerifiedMask != 0 })
	options := Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation", SimulationTest: simulationTest}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	client, err = Dial(ctx, options)
	if err != nil {
		t.Fatal(err)
	}
	return fixture{client, options}
}

func mustSubmit(t *testing.T, _ uint64, err error) {
	t.Helper()
	if err != nil {
		t.Fatal(err)
	}
}

func wait(t *testing.T, c *Client, predicate func(Status) bool) Status {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	defer cancel()
	state, err := c.Wait(ctx, predicate)
	if err != nil {
		t.Fatalf("wait: %v; state=%+v", err, state)
	}
	return state
}

func arm(t *testing.T, c *Client, mask uint32) {
	t.Helper()
	seq, err := c.Enable(context.Background(), mask)
	mustSubmit(t, seq, err)
	seq, err = c.Arm(context.Background())
	mustSubmit(t, seq, err)
	wait(t, c, func(s Status) bool { return s.Armed == 1 && s.AxisEnableMask == mask })
}

func TestBufferedTrajectoryUsesNativeTimingAndFeedforward(t *testing.T) {
	f := startCore(t, "simulation.json")
	c := f.client
	if c.NumAxes() != 2 || c.Cycle() != time.Millisecond {
		t.Fatal("wrong handshake")
	}
	arm(t, c, 3)
	seq, err := c.Trajectory(context.Background(), 42, 3, []Point{
		{Time: 0, Position: []float64{0, 0}, Velocity: []float64{.02, .02}},
		{Time: time.Second, Position: []float64{.02, .02}, Velocity: []float64{0, 0}},
	})
	mustSubmit(t, seq, err)
	s := wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts >= 410 && s.Axes[1].PositionCounts <= -410 })
	if s.SafetyFaultMask != 0 || s.Axes[0].VendorID == s.Axes[1].VendorID {
		t.Fatal("mixed drive execution failed")
	}
	deadline := time.Now().Add(time.Second)
	if !simfixture.PollUntil(deadline, 20*time.Millisecond, func() bool {
		motion, err := c.Motion()
		if err == nil && motion.TrajectoryID == 42 && motion.CommandSequence == seq && motion.Done {
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("missing native completion: %+v %v", motion, err)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
}

func TestHomeGateAndReconnectNeverResumes(t *testing.T) {
	f := startCore(t, "simulation-home.json")
	c := f.client
	seq, err := c.Enable(context.Background(), 1)
	mustSubmit(t, seq, err)
	seq, err = c.Arm(context.Background())
	mustSubmit(t, seq, err)
	time.Sleep(250 * time.Millisecond)
	s, err := c.Status()
	if err != nil || s.Armed != 0 {
		t.Fatalf("home gate: %+v %v", s, err)
	}
	seq, err = c.Home(context.Background(), 1)
	mustSubmit(t, seq, err)
	wait(t, c, func(s Status) bool { return s.HomeValidMask == 1 })
	arm(t, c, 1)
	seq, err = c.Jog(context.Background(), 1, []float64{.02}, 150*time.Millisecond)
	mustSubmit(t, seq, err)
	wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts > 0 })
	_ = c.Close()
	var replacement *Client
	for deadline := time.Now().Add(time.Second); time.Now().Before(deadline); {
		replacement, err = Dial(context.Background(), f.options)
		if err == nil {
			break
		}
		time.Sleep(20 * time.Millisecond)
	}
	if err != nil {
		t.Fatal(err)
	}
	defer replacement.Close()
	wait(t, replacement, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	position, _ := replacement.Status()
	time.Sleep(250 * time.Millisecond)
	later, _ := replacement.Status()
	if position.Axes[0].PositionCounts != later.Axes[0].PositionCounts {
		t.Fatal("motion resumed after reconnect")
	}
}

func TestCancellationInterruptsBackpressuredWriter(t *testing.T) {
	for _, operation := range []string{"disarm", "stop"} {
		t.Run(operation, func(t *testing.T) {
			f := startCore(t, "simulation.json")
			c := f.client
			arm(t, c, 3)
			// Backpressure is controlled at the ring boundary: this exercises the real
			// writer cancellation path without relying on scheduler timing/huge uploads.
			c.commandMu.Lock()
			generation := c.revocation.Load()
			written, read := c.commands.indices()
			if written != read {
				c.commandMu.Unlock()
				t.Fatal("command ring not drained")
			}
			// Use a separate in-memory ring, leaving the daemon's ring untouched.
			original := c.commands
			c.commands = ring{data: make([]byte, 128+32+128*GRADIENT_RING_MSG_BYTES), offset: 128, capacity: 128}
			atomicWrite := c.commands.counter(12)
			*atomicWrite = 128
			c.commandMu.Unlock()
			finished := make(chan error, 1)
			go func() {
				c.commandMu.Lock()
				defer c.commandMu.Unlock()
				_, err := c.sendLocked(context.Background(), generation, 0x0101, make([]byte, 8))
				finished <- err
			}()
			time.Sleep(10 * time.Millisecond)
			cancel := c.Disarm
			if operation == "stop" {
				cancel = func() error { _, err := c.Stop(context.Background()); return err }
			}
			if err := cancel(); err != nil {
				t.Fatal(err)
			}
			select {
			case err := <-finished:
				if !errors.Is(err, ErrRevoked) {
					t.Fatal(err)
				}
			case <-time.After(time.Second):
				t.Fatal("writer ignored revoke")
			}
			c.commandMu.Lock()
			c.commands = original
			c.commandMu.Unlock()
			wait(t, c, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
		})
	}
}

func TestConfigurationAndBackendMismatchRefused(t *testing.T) {
	f := startCore(t, "simulation.json")
	_ = f.client.Close()
	for _, options := range []Options{
		{SocketPath: f.options.SocketPath, ConfigurationSHA256: strings.Repeat("0", 64), Backend: "simulation"},
		{SocketPath: f.options.SocketPath, ConfigurationSHA256: f.options.ConfigurationSHA256, Backend: "ethercat"},
	} {
		time.Sleep(50 * time.Millisecond)
		c, err := Dial(context.Background(), options)
		if err == nil {
			c.Close()
			t.Fatal("wrong configuration/backend accepted")
		}
		if !strings.Contains(err.Error(), "identity mismatch") {
			t.Fatal(err)
		}
	}
}

func TestInvalidTrajectoryNeverBeginsUpload(t *testing.T) {
	f := startCore(t, "simulation.json")
	c := f.client
	before := c.sequence
	_, err := c.Trajectory(context.Background(), 1, 3, []Point{{Position: []float64{0, 0}}, {Time: time.Second, Position: []float64{0, math.NaN()}}})
	if err == nil || c.sequence != before {
		t.Fatal("invalid sample partially uploaded")
	}
	_, err = c.Jog(context.Background(), 4, []float64{0, 0}, time.Millisecond)
	if err == nil {
		t.Fatal("axis outside mapping accepted")
	}
}

func TestStopCancelsBufferedTrajectoryWithoutReplay(t *testing.T) {
	c := startCore(t, "simulation.json").client
	arm(t, c, 3)
	seq, err := c.Trajectory(context.Background(), 77, 3, []Point{
		{Time: 0, Position: []float64{0, 0}, Velocity: []float64{.02, 0}},
		{Time: 10 * time.Second, Position: []float64{.2, 0}, Velocity: []float64{0, 0}},
	})
	mustSubmit(t, seq, err)
	wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts > 150 })
	// A caller's expired operation context must not suppress urgent Stop.
	ctx, cancel := context.WithCancel(context.Background())
	cancel()
	seq, err = c.Stop(ctx)
	mustSubmit(t, seq, err)
	stopped := wait(t, c, func(s Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	arm(t, c, 3)
	time.Sleep(400 * time.Millisecond)
	after, err := c.Status()
	if err != nil {
		t.Fatal(err)
	}
	if after.Axes[0].PositionCounts != stopped.Axes[0].PositionCounts {
		t.Fatal("old trajectory replayed after Stop")
	}
}

// Model a suspended old convenience call resuming after Stop while another
// caller has reused its numeric handle for a fresh prepared plan.
func TestRevokedTrajectoryContinuationCannotConsumeReusedHandle(t *testing.T) {
	c := startCore(t, "simulation.json").client
	generation := c.revocation.Load()
	if err := c.Disarm(); err != nil {
		t.Fatal(err)
	}
	points := []Point{{Position: []float64{0, 0}}, {Time: time.Second, Position: []float64{0, 0}}}
	handle, err := c.PrepareTrajectory(context.Background(), 91, 3, points)
	if err != nil {
		t.Fatal(err)
	}
	before := c.sequence
	if _, err := c.startTrajectory(context.Background(), generation, handle); !errors.Is(err, ErrRevoked) {
		t.Fatal("old start was not revoked", err)
	}
	if _, err := c.prepareTrajectory(context.Background(), generation, 92, 3, points); !errors.Is(err, ErrRevoked) {
		t.Fatal("old upload was not revoked", err)
	}
	if c.sequence != before {
		t.Fatal("revoked continuation published a command")
	}
	arm(t, c, 3)
	if _, err := c.StartTrajectory(context.Background(), handle); err != nil {
		t.Fatal("fresh prepared handle was consumed", err)
	}
}
