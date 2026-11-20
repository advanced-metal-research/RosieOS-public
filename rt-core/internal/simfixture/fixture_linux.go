//go:build linux

// Package simfixture shares process mechanics without importing the native client.
package simfixture

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"syscall"
	"time"
)

// MachineConfigPath resolves a fixture basename in the machine layer. Explicit
// config-relative artifact paths remain explicit, preventing generated fixtures
// from being mistaken for shipped simulation configurations.
func MachineConfigPath(root, name string) string {
	directory := filepath.Join(root, "config")
	if filepath.Base(name) == name {
		directory = filepath.Join(directory, "machines")
		if strings.HasPrefix(name, "simulation") {
			directory = filepath.Join(directory, "simulation")
		}
	}
	return filepath.Join(directory, name)
}

type TB interface {
	Helper()
	Fatal(...any)
	Errorf(string, ...any)
	Logf(string, ...any)
	Failed() bool
	Cleanup(func())
}

// Locate honours instrumented test binaries without changing the default build.
func Locate(root, name string) string {
	key := map[string]string{"rosie-rt-core-sim": "RTCORE_TEST_BINARY", "rt-control": "RTCONTROL_TEST_BINARY"}[name]
	if key != "" {
		if path := os.Getenv(key); path != "" {
			return path
		}
	}
	return filepath.Join(root, "build", name)
}

// Compile invokes the parity-pinned Go compiler; rtctl imports sdk/control, so
// importing it here would cycle through package-internal native and SDK tests.
// Simulation CPU selection precedes compilation so restricted affinity cannot
// leave argv and deployment identity describing different CPU choices.
func Compile(root, machine, output string) (string, []string, error) {
	return CompileProfiles(root, machine, filepath.Join(root, "config/drives"), output)
}

// CompileProfiles retains synthetic profile directories used by native tests.
func CompileProfiles(root, machine, profiles, output string) (string, []string, error) {
	return CompileProfilesContext(context.Background(), root, machine, profiles, output)
}

// CompileProfilesContext preserves cancellation for tool callers while selecting
// before compilation; a cancelled run cannot proceed to daemon startup.
func CompileProfilesContext(ctx context.Context, root, machine, profiles, output string) (string, []string, error) {
	rtctl := filepath.Join(root, "build", "rtctl")
	if _, err := os.Stat(rtctl); err != nil {
		return "", nil, fmt.Errorf("compile: %s is missing (run make control); fixtures compile through the built rtctl, so a stale binary tests stale compiler behaviour", rtctl)
	}
	machine, cleanup, err := SelectMachine(machine, output)
	defer cleanup()
	if err != nil {
		return "", nil, err
	}
	cmd := exec.CommandContext(ctx, rtctl, "compile", "--config", machine, "--profiles", profiles, "--out", output)
	// Compiler migration warnings belong to stderr; treating them as part of
	// the stdout directory would make valid legacy configurations unreadable.
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	raw, err := cmd.Output()
	if err != nil {
		return "", nil, fmt.Errorf("compile: %w: %s%s", err, raw, stderr.Bytes())
	}
	directory := strings.TrimSpace(string(raw))
	raw, err = os.ReadFile(filepath.Join(directory, "argv.json"))
	if err != nil {
		return "", nil, err
	}
	var args []string
	err = json.Unmarshal(raw, &args)
	return directory, args, err
}

// Must skips typed CPU unavailability for test callers and reports every other
// setup failure, preventing an unsuitable host from masquerading as daemon drift.
func Must(t TB, err error) {
	t.Helper()
	if err != nil {
		var requirement *CPURequirementError
		if errors.As(err, &requirement) {
			if skipper, ok := t.(interface{ Skip(...any) }); ok {
				skipper.Skip(requirement)
			}
		}
		t.Fatal(err)
	}
}

// IsolateCommand expands simulator children onto the CPU reserved by TestMain.
// taskset execs the target in place, so process identity and signal handling are
// unchanged. The simulator is matched by its default name or by the
// RTCORE_TEST_BINARY override Locate returned: the sanitize lane's
// rosie-rt-core-asan otherwise inherited only the housekeeping CPU and refused
// to start ("no housekeeping CPU remains").
func IsolateCommand(cmd *exec.Cmd) error {
	simulator := filepath.Base(cmd.Path) == "rosie-rt-core-sim"
	if override := os.Getenv("RTCORE_TEST_BINARY"); override != "" && cmd.Path == override {
		simulator = true
	}
	if cpus := os.Getenv(processCPUsEnv); cpus != "" && simulator {
		taskset, err := exec.LookPath("taskset")
		if err != nil {
			return err
		}
		args := append([]string{taskset, "-c", cpus, cmd.Path}, cmd.Args[1:]...)
		cmd.Path, cmd.Args = taskset, args
	}
	return nil
}

// SpawnLogged registers bounded termination and log diagnostics immediately, so
// a failed readiness assertion cannot leak the child. A nil exited callback
// retains fixtures which intentionally ignore the child's exit status.
func SpawnLogged(t TB, cmd *exec.Cmd, path string, budget time.Duration, tail int, exited func(error), inspect ...func([]byte)) *exec.Cmd {
	t.Helper()
	Must(t, IsolateCommand(cmd))
	log, err := os.Create(path)
	Must(t, err)
	cmd.Stdout, cmd.Stderr = log, log
	if err := cmd.Start(); err != nil {
		log.Close()
		t.Fatal(err)
	}
	t.Cleanup(func() {
		_ = cmd.Process.Signal(syscall.SIGCONT)
		_ = cmd.Process.Signal(syscall.SIGTERM)
		done := make(chan error, 1)
		go func() { done <- cmd.Wait() }()
		select {
		case err := <-done:
			if exited != nil {
				exited(err)
			}
		case <-time.After(budget):
			// Capture the child before termination so a shutdown deadlock is not
			// mistaken for a slow test host. Reading /proc does not extend the
			// grace period or authorize ignoring the failed shutdown assertion.
			tasks, _ := filepath.Glob(fmt.Sprintf("/proc/%d/task/*", cmd.Process.Pid))
			for _, task := range tasks {
				comm, _ := os.ReadFile(filepath.Join(task, "comm"))
				wait, _ := os.ReadFile(filepath.Join(task, "wchan"))
				call, _ := os.ReadFile(filepath.Join(task, "syscall"))
				t.Logf("shutdown thread %s (%s): wait=%s syscall=%s", filepath.Base(task), strings.TrimSpace(string(comm)), strings.TrimSpace(string(wait)), strings.TrimSpace(string(call)))
			}
			_ = cmd.Process.Kill()
			<-done
			t.Errorf("%s shutdown exceeded %d ms", filepath.Base(cmd.Path), budget.Milliseconds())
		}
		_ = log.Close()
		if t.Failed() || len(inspect) != 0 {
			raw, _ := os.ReadFile(path)
			for _, check := range inspect {
				check(raw)
			}
			if t.Failed() {
				if tail > 0 && len(raw) > tail {
					raw = raw[len(raw)-tail:]
				}
				t.Logf("%s: %s", filepath.Base(cmd.Path), raw)
			}
		}
	})
	return cmd
}

// ControlArgs keeps argv construction in one place; extras remain last so
// existing test overrides keep their original precedence.
func ControlArgs(coreSocket, socket, hash, pair string, revision uint64, extra ...string) []string {
	return append([]string{"--core-socket", coreSocket, "--socket", socket,
		"--configuration-sha256", hash, "--backend", "simulation",
		"--pair-id", pair, "--pair-revision", strconv.FormatUint(revision, 10)}, extra...)
}

// StartControl applies binary overrides to the same explicitly supplied argv.
// The dense axis map is the description's; no argument names it.
func StartControl(root, coreSocket, socket, hash, pair string, revision uint64, extra ...string) *exec.Cmd {
	return exec.Command(Locate(root, "rt-control"), ControlArgs(coreSocket, socket, hash, pair, revision, extra...)...)
}

// Poll observes immediately and preserves a caller-selected ticker cadence.
// Only the predicate can establish success; cancellation bounds failed progress.
func Poll(ctx context.Context, cadence time.Duration, predicate func() (bool, error)) error {
	ticker := time.NewTicker(cadence)
	defer ticker.Stop()
	for {
		ready, err := predicate()
		if err != nil {
			return err
		}
		if ready {
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

// PollTest bounds an observation without embedding a test-specific deadline.
func PollTest(t TB, budget, cadence time.Duration, predicate func() bool) {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), budget)
	defer cancel()
	Must(t, Poll(ctx, cadence, func() (bool, error) { return predicate(), nil }))
}

// Verified contains only startup metrics, avoiding a dependency on ipcclient.
type Verified struct {
	TimeNS             uint64 `json:"time_ns"`
	ConfigVerifiedMask uint32 `json:"config_verified_mask"`
}

// WaitVerified retains the caller's mask/time predicate; malformed or absent
// startup metrics remain pending rather than becoming zero-valued success.
func WaitVerified(t TB, path string, budget, cadence time.Duration, predicate func(Verified) bool) {
	t.Helper()
	PollTest(t, budget, cadence, func() bool {
		raw, _ := os.ReadFile(path)
		var v Verified
		return json.Unmarshal(raw, &v) == nil && predicate(v)
	})
}

// Renew joins cancellation before dependent cleanup, preventing renewal after
// native/client teardown. The callback owns each fixture's error policy.
func Renew(ctx context.Context, cadence time.Duration, renew func() bool) func() {
	ctx, cancel := context.WithCancel(ctx)
	done := make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(cadence)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				if !renew() {
					return
				}
			}
		}
	}()
	return func() { cancel(); <-done }
}

// PollUntil preserves sleep-after-observation cadence for legacy loops. The
// absolute deadline is checked after the predicate as in those fixtures.
func PollUntil(deadline time.Time, cadence time.Duration, predicate func() bool) bool {
	for {
		if predicate() {
			return true
		}
		if time.Now().After(deadline) {
			return false
		}
		time.Sleep(cadence)
	}
}

// CheckExit retains fixtures that assert graceful child shutdown.
func CheckExit(t TB) func(error) {
	return func(err error) {
		if err != nil {
			t.Errorf("process shutdown: %v", err)
		}
	}
}

// ErrorText exposes compiler diagnostics to rejection assertions.
func ErrorText(err error) string {
	if err == nil {
		return ""
	}
	return fmt.Sprint(err)
}
