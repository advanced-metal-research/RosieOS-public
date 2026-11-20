//go:build linux

package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/ipcclient"
	"syscall"
	"time"

	control "rosieos/rt-core/sdk/control"
)

// Unverified interaction watchdog: 10 s; success always requires an observation.
const operationBudget = 10 * time.Second

// Unverified bulk receipt watchdog: 60 s; limits a stuck 250,000-point upload.
const uploadBudget = 60 * time.Second

// Unverified observation cadence: 10 ms, below the 100 ms metrics publication.
const observationPeriod = 10 * time.Millisecond

// Unverified sample cadence: the requested 1 s process and native metrics series.
const samplePeriod = time.Second

// Unverified lease renewal cadence: 50 ms, below the native 500 ms grant lease.
const renewalPeriod = 50 * time.Millisecond

// Unverified workload axis mask: J1 for plans and concurrent jog.
const motionMask uint32 = 1

type process struct {
	cmd  *exec.Cmd
	done chan error
	log  *os.File
}

func startProcess(root, logPath, binary string, args ...string) (*process, error) {
	log, err := os.Create(logPath)
	if err != nil {
		return nil, err
	}
	// Honours RTCORE_TEST_BINARY and RTCONTROL_TEST_BINARY like the test fixtures, so a
	// soak can run against an instrumented daemon; unset, the built binaries are used.
	cmd := exec.Command(simfixture.Locate(root, binary), args...)
	cmd.Dir, cmd.Stdout, cmd.Stderr = root, log, log
	cmd.Env = append(os.Environ(), "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED=0")
	if err := cmd.Start(); err != nil {
		log.Close()
		return nil, err
	}
	p := &process{cmd: cmd, done: make(chan error, 1), log: log}
	go func() { p.done <- cmd.Wait() }()
	return p, nil
}

// close joins the actual process; timeout or early death cannot be reported as
// a clean soak shutdown, and every started child is reaped on a failure path.
func (p *process) close() error {
	if p == nil {
		return nil
	}
	defer p.log.Close()
	select {
	case err := <-p.done:
		return fmt.Errorf("%s exited before shutdown: %v", filepath.Base(p.cmd.Path), err)
	default:
	}
	if err := p.cmd.Process.Signal(syscall.SIGTERM); err != nil {
		return err
	}
	select {
	case err := <-p.done:
		return err
	case <-time.After(operationBudget):
		_ = p.cmd.Process.Kill()
		<-p.done
		return fmt.Errorf("%s shutdown watchdog expired", filepath.Base(p.cmd.Path))
	}
}

type service struct {
	client                   *control.Client
	daemon, adapter          *process
	sockets, artifacts, hash string
	renewals                 <-chan control.Renewal
}

func (s *service) close() error {
	if s.client != nil {
		s.client.Close()
	}
	err := errors.Join(s.adapter.close(), s.daemon.close())
	if s.sockets != "" {
		err = errors.Join(err, os.RemoveAll(s.sockets))
	}
	return err
}

// startService compiles an explicitly simulated fixture with the shipped Go CLI
// and starts the shipped binaries. It never acquires a native control connection.
func startService(ctx context.Context, r *record) (s *service, resultErr error) {
	s = &service{}
	defer func() {
		if resultErr != nil {
			resultErr = errors.Join(resultErr, s.close())
		}
	}()
	root := r.Configuration.Root
	for _, name := range []string{"rtctl", "rosie-rt-core-sim", "rt-control"} {
		info, err := os.Stat(filepath.Join(root, "build", name))
		if err != nil {
			return s, err
		}
		if !info.Mode().IsRegular() || info.Mode().Perm()&0111 == 0 {
			return s, fmt.Errorf("build/%s is not executable", name)
		}
	}
	var err error
	s.artifacts, err = os.MkdirTemp(filepath.Join(root, "build"), "soak-")
	if err != nil {
		return s, err
	}
	r.Configuration.ArtifactDirectory = s.artifacts
	s.sockets, err = os.MkdirTemp("", "s-")
	if err != nil {
		return s, err
	}
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-program.json"))
	if err != nil {
		return s, err
	}
	var machine map[string]any
	if err := json.Unmarshal(raw, &machine); err != nil {
		return s, err
	}
	if machine["backend"] != "simulation" {
		return s, errors.New("fixture must be simulation")
	}
	raw, err = json.MarshalIndent(machine, "", "  ")
	if err != nil {
		return s, err
	}
	r.Configuration.Machine = raw
	r.Configuration.CycleLatenessBoundNS = uint64(machine["max_cycle_lateness_ns"].(float64))
	machinePath := filepath.Join(s.artifacts, "machine.json")
	if err := os.WriteFile(machinePath, raw, 0600); err != nil {
		return s, err
	}
	compiled, args, err := simfixture.CompileProfilesContext(ctx, root, machinePath, filepath.Join(root, "config/drives"), filepath.Join(s.artifacts, "config"))
	if err != nil {
		return s, err
	}
	s.hash = filepath.Base(compiled)
	r.Configuration.MachineSHA = s.hash
	coreSocket := filepath.Join(s.sockets, "ipc.sock")
	args = append(args, "--socket-path", coreSocket)
	r.Configuration.DaemonArgs = args
	s.daemon, err = startProcess(root, filepath.Join(s.artifacts, "daemon.log"), "rosie-rt-core-sim", args...)
	if err != nil {
		return s, err
	}
	r.Configuration.DaemonPID = s.daemon.cmd.Process.Pid
	if err := poll(ctx, operationBudget, func() (bool, error) {
		raw, err := os.ReadFile(filepath.Join(s.sockets, "metrics.json"))
		if errors.Is(err, os.ErrNotExist) {
			return false, nil
		}
		if err != nil {
			return false, err
		}
		var state ipcclient.Metrics
		if err := json.Unmarshal(raw, &state); err != nil {
			return false, err
		}
		return state.ConfigVerifiedMask == 511, nil
	}); err != nil {
		return s, fmt.Errorf("native startup: %w", err)
	}
	socket := filepath.Join(s.sockets, "control.sock")
	args = simfixture.ControlArgs(coreSocket, socket, s.hash, "soak", 1)
	r.Configuration.AdapterArgs = args
	s.adapter, err = startProcess(root, filepath.Join(s.artifacts, "adapter.log"), "rt-control", args...)
	if err != nil {
		return s, err
	}
	r.Configuration.AdapterPID = s.adapter.cmd.Process.Pid
	s.client, err = control.Dial(control.UnixPath(socket))
	if err != nil {
		return s, err
	}
	if err := poll(ctx, operationBudget, func() (bool, error) {
		call, cancel := context.WithTimeout(ctx, samplePeriod)
		defer cancel()
		state, err := s.client.Status(call)
		return err == nil && state.Motion.TimeNS != 0, nil
	}); err != nil {
		return s, fmt.Errorf("adapter startup: %w", err)
	}
	description, err := s.client.Describe(ctx)
	if err != nil {
		return s, err
	}
	r.Configuration.JogAgeBoundNS = description.MaxJogInputAgeNS
	r.Configuration.RTTID, err = cycleTID(r.Configuration.DaemonPID)
	return s, err
}

func (s *service) acquire(ctx context.Context) error {
	s.client.StopRenewal()
	s.client.SetFence(control.Fence{})
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	_, err := s.client.Acquire(call, "soak", control.Binding{PairID: "soak", Revision: 1, ConfigurationSHA256: s.hash})
	if err != nil {
		return err
	}
	s.renewals, err = s.client.StartRenewal(ctx, renewalPeriod)
	return err
}

// poll has a bounded failure path and requires an observed predicate for success;
// a timer firing can never establish readiness, completion or reclamation.
func poll(parent context.Context, budget time.Duration, predicate func() (bool, error)) error {
	ctx, cancel := context.WithTimeout(parent, budget)
	defer cancel()
	ticker := time.NewTicker(observationPeriod)
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

func (s *service) waitStatus(ctx context.Context, predicate func(control.Status) bool) (state control.Status, err error) {
	err = poll(ctx, operationBudget, func() (bool, error) {
		call, cancel := context.WithTimeout(ctx, samplePeriod)
		defer cancel()
		var err error
		state, err = s.client.Status(call)
		return err == nil && predicate(state), err
	})
	return state, err
}

func (s *service) enable(ctx context.Context, mask uint32) error {
	call, cancel := context.WithTimeout(ctx, operationBudget)
	defer cancel()
	if _, err := s.client.Enable(call, mask); err != nil {
		return err
	}
	if _, err := s.client.Arm(call); err != nil {
		return err
	}
	_, err := s.waitStatus(ctx, func(v control.Status) bool {
		if v.Core.Armed != 1 || v.Core.AxisEnableMask != mask || v.Core.ExecutionFaultReasons != 0 {
			return false
		}
		for i, a := range v.Core.Axes {
			if mask&(1<<i) != 0 && a.Statusword&0x6f != 0x27 {
				return false
			}
		}
		return true
	})
	return err
}
