//go:build linux

package ipcclient

import (
	"context"
	"crypto/sha256"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"
	"time"
)

// TestNineMotorDefaultDriveTelemetryValidity runs the current six-axis bench robot
// configuration at 1 kHz; absent DI must never be published as valid zero inputs.
func TestNineMotorDefaultDriveTelemetryValidity(t *testing.T) {
	root, err := filepath.Abs("..")
	simfixture.Must(t, err)
	dir, err := os.MkdirTemp("", "jb04-")
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	compiled, args, err := simfixture.Compile(root, filepath.Join(root, "config", "machines", "bench", "rosie1420-six-motors.json"), filepath.Join(dir, "config"))
	simfixture.Must(t, err)
	socket := filepath.Join(dir, "ipc.sock")
	// Unverified injected diagnostics: signed drive-frame counts/s and counts;
	// observation watchdogs are 5000 ms, shutdown 3000 ms. No motion is requested.
	args = append(args, "--socket-path", socket, "--sim-velocity-actual-counts-per-s", "-123456", "--sim-following-error-counts", "-789")
	process := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), args...)
	simfixture.SpawnLogged(t, process, filepath.Join(dir, "core.log"), 3*time.Second, 9000, nil)
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	var c *Client
	simfixture.PollTest(t, 5*time.Second, time.Millisecond, func() bool {
		c, err = Dial(ctx, Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation", SimulationTest: true})
		return err == nil
	})
	defer c.Close()
	s := wait(t, c, func(s Status) bool {
		if len(s.Axes) != 6 {
			return false
		}
		for _, a := range s.Axes {
			if !a.VelocityActualValid || !a.FollowingErrorValid {
				return false
			}
		}
		return true
	})
	for i, a := range s.Axes {
		if a.DIValid || a.DIBits != 0 || a.VelocityActualCountsPerS != -123456 || a.FollowingErrorCounts != -789 {
			t.Fatalf("J%d B04 validity/signed units: %+v", i+1, a)
		}
	}
	reader, err := c.OpenTelemetry()
	simfixture.Must(t, err)
	defer reader.Close()
	if reader.CyclePeriodNS != 1000000 {
		t.Fatalf("cycle period: %d ns", reader.CyclePeriodNS)
	}
	// Compare the running daemon with this compile's canonical identities.
	machineIdentity, err := os.ReadFile(filepath.Join(compiled, "machine.identity"))
	simfixture.Must(t, err)
	description, err := c.Describe()
	simfixture.Must(t, err)
	if description.ConfigurationSHA256 != filepath.Base(compiled) ||
		description.MachineSHA256 != fmt.Sprintf("%x", sha256.Sum256(machineIdentity)) {
		t.Fatalf("robot identity does not match compiled configuration: %+v", description)
	}
}

// PREOP mismatch must refuse before registration or activation; the daemon
// must never interpret process data through an unconfirmed layout.
func TestDriveTelemetryMappingMismatchBeforeBusReady(t *testing.T) {
	compiled := compiledForValidation(t, "simulation.json")
	dir, err := os.MkdirTemp("", "dmm-")
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	args := append(append([]string(nil), compiled.args...), "--socket-path", filepath.Join(dir, "ipc.sock"),
		"--sim-pdo-mismatch", "--sim-wkc-incomplete-from-call", "1", "--sim-wkc-incomplete-for-calls", "10000")
	// Unverified process watchdog: 5000000000 ns; pass requires refusal/exit.
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	output, err := exec.CommandContext(ctx, compiled.binary, args...).CombinedOutput()
	var exit *exec.ExitError
	if !errors.As(err, &exit) || exit.ExitCode() != 1 || ctx.Err() != nil {
		t.Fatalf("expected bring-up refusal, err=%v context=%v: %s", err, ctx.Err(), output)
	}
	log := string(output)
	if !strings.Contains(log, "phase=PREOP result=pdo_mapping_mismatch") ||
		!strings.Contains(log, "activation=blocked") ||
		strings.Contains(log, "IgH libecrt active") || strings.Contains(log, "phase=domain_reg_") {
		t.Fatalf("mapping mismatch did not refuse before activation: %s", log)
	}
}

// TestDriveTelemetryStatusAndRing injects exact signed drive-frame samples at
// 1 kHz; the legacy preset must report absent signals, never valid zero values.
func TestDriveTelemetryStatusAndRing(t *testing.T) {
	root, err := filepath.Abs("..")
	simfixture.Must(t, err)
	dir, err := os.MkdirTemp("", "dt-")
	simfixture.Must(t, err)
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	profiles := filepath.Join(dir, "config/drives")
	simfixture.Must(t, os.MkdirAll(profiles, 0755))
	for _, name := range []string{"a6ec_bench_1to1", "a6ec_1b02"} {
		raw, err := os.ReadFile(filepath.Join(root, "config/drives", name+".json"))
		simfixture.Must(t, err)
		var p map[string]any
		simfixture.Must(t, json.Unmarshal(raw, &p))
		// Isolate PDO diagnostics from sampled coordinate/Home acquisition in this fixture.
		delete(p, "coordinate_evidence")
		p["brake"].(map[string]any)["present"] = false
		raw, err = json.Marshal(p)
		simfixture.Must(t, err)
		simfixture.Must(t, os.WriteFile(filepath.Join(profiles, name+".json"), raw, 0644))
	}
	raw, err := os.ReadFile(filepath.Join(root, "config", "machines", "simulation", "simulation.json"))
	simfixture.Must(t, err)
	var machine map[string]any
	simfixture.Must(t, json.Unmarshal(raw, &machine))
	axes := machine["axes"].([]any)
	for i, name := range []string{"a6ec_bench_1to1", "a6ec_1b02"} {
		a := axes[i].(map[string]any)
		a["profile"] = name
	}
	raw, err = json.Marshal(machine)
	simfixture.Must(t, err)
	config := filepath.Join(dir, "machine.json")
	simfixture.Must(t, os.WriteFile(config, raw, 0644))
	compiled, args, err := simfixture.CompileProfiles(root, config, profiles, filepath.Join(dir, "config"))
	simfixture.Must(t, err)
	socket := filepath.Join(dir, "ipc.sock")
	args = append(args, "--socket-path", socket, "--sim-velocity-actual-counts-per-s", "-123456", "--sim-following-error-counts", "789")
	// Unverified fixture observation and shutdown budgets in ms; success requires data.
	process := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), args...)
	simfixture.SpawnLogged(t, process, filepath.Join(dir, "core.log"), 3*time.Second, 9000, nil)
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 5*time.Second, time.Millisecond, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 3 })
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	c, err := Dial(ctx, Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation", SimulationTest: true})
	simfixture.Must(t, err)
	defer c.Close()
	before := wait(t, c, func(s Status) bool { return len(s.Axes) == 2 && s.Axes[0].VelocityActualValid })
	if before.Axes[0].MaxAbsVelocityActualCountsPerS != 0 {
		t.Fatal("peak started before Arm")
	}
	arm(t, c, 3)
	s := wait(t, c, func(s Status) bool { return s.Axes[0].MaxAbsVelocityActualCountsPerS == 123456 })
	a, b := s.Axes[0], s.Axes[1]
	if a.VelocityActualCountsPerS != -123456 || a.FollowingErrorCounts != 789 || !a.FollowingErrorValid || a.MaxAbsFollowingErrorCounts != 789 {
		t.Fatalf("signed drive units: %+v", a)
	}
	if b.VelocityActualValid || b.FollowingErrorValid || b.VelocityActualCountsPerS != 0 || b.FollowingErrorCounts != 0 || b.MaxAbsVelocityActualCountsPerS != 0 {
		t.Fatalf("legacy mapping: %+v", b)
	}
	if a.DIValid || a.DIBits != 0 || !b.DIValid {
		t.Fatalf("B04/B02 DI validity: %+v / %+v", a, b)
	}
	// Drive health (docs/decisions/0001): the metrics thread's last second of
	// following error from the ring, and the profile's monitor list (a6ec_1b02
	// names the heatsink temperature; the simulator has no mailbox, so it is
	// listed without a value).
	s = wait(t, c, func(s Status) bool { return s.Axes[0].Load.FollowingErrorSamples > 0 })
	a, b = s.Axes[0], s.Axes[1]
	if a.Load.WindowMS != 1000 || a.Load.FollowingErrorPeakCounts != 789 || a.Load.FollowingErrorSamples > 1000 || len(a.Monitor) != 0 {
		t.Fatalf("axis 0 load and monitor: %+v %+v", a.Load, a.Monitor)
	}
	if b.Load.FollowingErrorSamples != 0 || len(b.Monitor) != 1 || b.Monitor[0].Key != "heatsink_temperature" ||
		b.Monitor[0].Unit != "degC" || b.Monitor[0].Value != nil || b.Monitor[0].AgeMS != nil {
		t.Fatalf("axis 1 load and monitor: %+v %+v", b.Load, b.Monitor)
	}
	reader, err := c.OpenTelemetry()
	simfixture.Must(t, err)
	defer reader.Close()
	if reader.CyclePeriodNS != 1000000 {
		t.Fatalf("cycle period ns: %d", reader.CyclePeriodNS)
	}
	var cursor uint64
	simfixture.PollTest(t, 5*time.Second, time.Millisecond, func() bool {
		head, err := reader.Written()
		if errors.Is(err, ErrTelemetryBusy) {
			return false
		}
		simfixture.Must(t, err)
		cursor = head + 1
		return true
	})
	seen := 0
	simfixture.PollTest(t, 5*time.Second, time.Millisecond, func() bool {
		batch, err := reader.ReadFrom(cursor, 100)
		if errors.Is(err, ErrTelemetryBusy) {
			return false
		}
		simfixture.Must(t, err)
		for _, record := range batch.Records {
			if record.Seq != cursor {
				t.Fatalf("cycle sequence: got %d want %d", record.Seq, cursor)
			}
			a, b := record.Ax[0], record.Ax[1]
			if a.TorqueValid != 1 || b.TorqueValid != 1 || a.PdoFresh != 1 || b.PdoFresh != 1 || a.ModeDisplay != 8 || b.ModeDisplay != 8 {
				t.Fatalf("B04/B02 torque/mode freshness: %+v / %+v", a, b)
			}
			if record.Armed != 1 || record.AxisEnableMask != 3 || record.SubmittedAxisMask != 3 || record.BusSubmissionOperation != 5 || record.BusSubmissionReturnCode < 0 {
				t.Fatalf("capture authority/submission: %+v", record)
			}
			if a.VelocityActualCountsPerS != -123456 || a.FollowingErrorCounts != 789 || a.VelocityActualValid != 1 || a.FollowingErrorValid != 1 {
				t.Fatalf("ring sign/units: %+v", a)
			}
			if b.VelocityActualValid != 0 || b.FollowingErrorValid != 0 || b.VelocityActualCountsPerS != 0 || b.FollowingErrorCounts != 0 {
				t.Fatalf("legacy ring: %+v", b)
			}
			cursor = record.Seq + 1
			seen++
		}
		return seen >= 100
	})
	// Public status must forward the same drive samples and retained peaks.
	simfixture.Must(t, c.Close())
	controlSocket := filepath.Join(dir, "control.sock")
	adapter := simfixture.StartControl(root, socket, controlSocket, filepath.Base(compiled), "drive-telemetry", 1)
	simfixture.SpawnLogged(t, adapter, filepath.Join(dir, "control.log"), 3*time.Second, 9000, nil)
	transport := &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", controlSocket)
	}}
	defer transport.CloseIdleConnections()
	httpClient := &http.Client{Transport: transport, Timeout: time.Second}
	simfixture.PollTest(t, 5*time.Second, time.Millisecond, func() bool {
		response, err := httpClient.Get("http://unix/v1/status")
		if err != nil {
			return false
		}
		defer response.Body.Close()
		var envelope struct {
			Data struct{ Axes []LogicalAxisStatus }
		}
		if json.NewDecoder(response.Body).Decode(&envelope) != nil || response.StatusCode != 200 || len(envelope.Data.Axes) != 2 {
			return false
		}
		a, b := envelope.Data.Axes[0], envelope.Data.Axes[1]
		if !a.VelocityActualValid {
			return false
		}
		if a.VelocityActualCountsPerS != -123456 || a.FollowingErrorCounts != 789 || !a.FollowingErrorValid || a.MaxAbsVelocityActualCountsPerS != 123456 || a.MaxAbsFollowingErrorCounts != 789 || b.VelocityActualValid || b.FollowingErrorValid {
			t.Fatalf("public drive status: %+v", envelope.Data.Axes)
		}
		return true
	})
	t.Logf("1000000 ns cycle; %d records: velocity=-123456 counts/s, following_error=789 counts; legacy flags=false", seen)
}
