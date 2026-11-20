//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"rosieos/rt-core/internal/unixhttp"
	"strings"
	"sync"
	"syscall"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

// Unverified test observation budgets, in absolute host time, never computed
// from the implementation's measured response or from its observed cadence.
const (
	restartWait = 5 * time.Second
	restartPoll = 5 * time.Millisecond
)

type restartProcess struct {
	cmd  *exec.Cmd
	done chan struct{}
	err  error
	log  *os.File
}

func restartEventually(t *testing.T, label string, predicate func() bool) {
	t.Helper()
	timer, tick := time.NewTimer(restartWait), time.NewTicker(restartPoll)
	defer timer.Stop()
	defer tick.Stop()
	for {
		if predicate() {
			return
		}
		select {
		case <-timer.C:
			t.Fatalf("%s: condition absent after %d ms", label, restartWait/time.Millisecond)
		case <-tick.C:
		}
	}
}

func restartSpawn(t *testing.T, dir, binary string, args []string) *restartProcess {
	t.Helper()
	log, err := os.CreateTemp(dir, "process-*.log")
	if err != nil {
		t.Fatal(err)
	}
	p := &restartProcess{cmd: exec.Command(binary, args...), done: make(chan struct{}), log: log}
	p.cmd.Stdout, p.cmd.Stderr = log, log
	if err := simfixture.IsolateCommand(p.cmd); err != nil {
		log.Close()
		t.Fatal(err)
	}
	if err := p.cmd.Start(); err != nil {
		log.Close()
		t.Fatal(err)
	}
	go func() { p.err = p.cmd.Wait(); close(p.done) }()
	t.Cleanup(func() {
		select {
		case <-p.done:
		default:
			p.signal(t, syscall.SIGTERM)
		}
		log.Close()
		raw, _ := os.ReadFile(log.Name())
		if strings.Contains(string(raw), "DATA RACE") {
			t.Error(string(raw))
		}
		if t.Failed() {
			t.Logf("%s: %s", filepath.Base(binary), raw)
		}
	})
	return p
}

func (p *restartProcess) signal(t *testing.T, signal syscall.Signal) {
	t.Helper()
	if err := p.cmd.Process.Signal(signal); err != nil {
		t.Fatal(err)
	}
	select {
	case <-p.done:
	case <-time.After(restartWait):
		_ = p.cmd.Process.Kill()
		<-p.done
		t.Fatal("process did not exit after signal")
	}
	if signal == syscall.SIGTERM && p.err != nil {
		t.Fatalf("clean shutdown: %v", p.err)
	}
}

type restartFixture struct {
	t                                                 *testing.T
	dir, binary, adapterBinary, socket, controlSocket string
	args, adapterArgs                                 []string
	options                                           Options
	core, adapter                                     *restartProcess
	service                                           *jogService
	launchNS                                          uint64
}

func newRestartFixture(t *testing.T) *restartFixture {
	t.Helper()
	compiled := compiledForValidation(t, "simulation.json")
	dir, err := os.MkdirTemp("", "r-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	f := &restartFixture{t: t, dir: dir, binary: compiled.binary, adapterBinary: simfixture.Locate(root, "rt-control"),
		socket: filepath.Join(dir, "ipc.sock"), controlSocket: filepath.Join(dir, "control.sock")}
	if binary := os.Getenv("RTCONTROL_TEST_BINARY"); binary != "" {
		f.adapterBinary = binary
	}
	f.args = append(append([]string(nil), compiled.args...), "--socket-path", f.socket)
	f.options = Options{SocketPath: f.socket, ConfigurationSHA256: filepath.Base(filepath.Dir(compiled.path)), Backend: "simulation"}
	f.adapterArgs = simfixture.ControlArgs(f.socket, f.controlSocket, f.options.ConfigurationSHA256, "restart-test", 1)
	transport := unixhttp.NewClient(f.controlSocket).Transport.(*http.Transport)
	f.service = &jogService{http: &http.Client{Transport: transport, Timeout: time.Second}, t: t}
	t.Cleanup(transport.CloseIdleConnections)
	f.startCore()
	return f
}

func (f *restartFixture) startCore() {
	f.t.Helper()
	f.launchNS = HostMonotonicNS()
	f.core = restartSpawn(f.t, f.dir, f.binary, f.args)
	restartEventually(f.t, "fresh daemon readiness", func() bool {
		s := f.diagnostic()
		return s.TimeNS >= f.launchNS && s.ConfigVerifiedMask == 3 && len(s.Axes) == 2
	})
}

func (f *restartFixture) diagnostic() Status {
	raw, _ := os.ReadFile(filepath.Join(f.dir, "metrics.json"))
	var status Status
	_ = json.Unmarshal(raw, &status)
	return status
}

func (f *restartFixture) startAdapter() {
	f.t.Helper()
	started := HostMonotonicNS()
	f.adapter = restartSpawn(f.t, f.dir, f.adapterBinary, f.adapterArgs)
	restartEventually(f.t, "public adapter readiness", func() bool {
		return f.service.get("/v1/jog/clock", &f.service.clock) == nil
	})
	if elapsed := HostMonotonicNS() - started; elapsed > 5_000_000_000 {
		f.t.Fatalf("adapter bootstrap exceeded 5000 ms: %d ns", elapsed)
	}
}

type restartStatus struct {
	DaemonIncarnation  string            `json:"daemon_incarnation"`
	AdapterIncarnation string            `json:"adapter_incarnation"`
	Core               Status            `json:"core"`
	Motion             MotionState       `json:"motion"`
	Grant              *GrantObservation `json:"grant"`
	Jog                JogObservation    `json:"jog"`
	Execution          struct {
		State, Error string
		Handles      []struct {
			Handle uint64
			State  string
		}
	} `json:"execution"`
}

func (f *restartFixture) status(predicate func(restartStatus) bool) restartStatus {
	f.t.Helper()
	var status restartStatus
	restartEventually(f.t, "public status", func() bool {
		err := f.service.get("/v1/status", &status)
		return err == nil && status.Grant != nil && status.Jog.NowHostNs != 0 && predicate(status)
	})
	return status
}

func (f *restartFixture) acquire() {
	f.t.Helper()
	r := f.service.must("acquire", map[string]any{"session": "", "generation": 0, "controller": "restart-test",
		"binding": map[string]any{"pair_id": "restart-test", "revision": 1, "configuration_sha256": f.options.ConfigurationSHA256}})
	var grant struct {
		Session    string
		Generation uint64
	}
	if err := json.Unmarshal(r.Data, &grant); err != nil {
		f.t.Fatal(err)
	}
	f.service.session, f.service.grant = grant.Session, grant.Generation
}

// Each renewer captures one fence; it cannot accidentally renew a successor.
func (f *restartFixture) renew() func() {
	session, generation := f.service.session, f.service.grant
	joined := simfixture.Renew(context.Background(), 50*time.Millisecond, func() bool {
		_, _ = f.service.request("renew", map[string]any{"session": session, "generation": generation})
		return true
	})
	var once sync.Once
	stop := func() { once.Do(func() { joined() }) }
	f.t.Cleanup(stop)
	return stop
}

func (f *restartFixture) arm() {
	f.t.Helper()
	f.service.must("enable", map[string]any{"axis_mask": 3})
	f.service.must("arm", nil)
	f.status(func(s restartStatus) bool { return s.Core.Armed == 1 && allOperationEnabled(s.Core) })
}

func (f *restartFixture) prepare(duration time.Duration, endpoint float64, id string) uint64 {
	f.t.Helper()
	r := f.service.must("prepare_trajectory", map[string]any{"axis_mask": 3, "request_id": id,
		"points": []Point{{Time: 0, Position: []float64{0, 0}}, {Time: duration, Position: []float64{endpoint, endpoint}}}})
	if r.Handle == 0 {
		f.t.Fatal("zero prepared handle")
	}
	return r.Handle
}

func restartLock(t *testing.T, path string, held bool, pid int) {
	t.Helper()
	data, err := os.ReadFile(path + ".lock")
	if err != nil || !strings.Contains(string(data), fmt.Sprintf("pid=%d start_time_ticks=", pid)) {
		t.Fatalf("owner identity: %s %v", data, err)
	}
	fd, err := unix.Open(path+".lock", unix.O_RDWR|unix.O_CLOEXEC, 0)
	if err != nil {
		t.Fatal(err)
	}
	defer unix.Close(fd)
	err = unix.Flock(fd, unix.LOCK_EX|unix.LOCK_NB)
	if held && !errors.Is(err, unix.EWOULDBLOCK) {
		t.Fatalf("owner lock not held: %v", err)
	}
	if !held && err != nil {
		t.Fatalf("owner lock not released: %v", err)
	}
}

func restartRefused(t *testing.T, p *restartProcess) {
	t.Helper()
	select {
	case <-p.done:
	case <-time.After(restartWait):
		t.Fatal("competing owner did not exit")
	}
	var exited *exec.ExitError
	if !errors.As(p.err, &exited) || exited.ExitCode() != 3 {
		t.Fatalf("competing owner exit: %v", p.err)
	}
	raw, _ := os.ReadFile(p.log.Name())
	if !strings.Contains(string(raw), "socket owned by a live process") {
		t.Fatalf("wrong ownership diagnostic: %s", raw)
	}
}

func TestSecondDaemonRefusesLiveSocket(t *testing.T) {
	f := newRestartFixture(t)
	before, err := os.Lstat(f.socket)
	if err != nil {
		t.Fatal(err)
	}
	description, err := os.ReadFile(filepath.Join(f.dir, "description.json"))
	if err != nil {
		t.Fatal(err)
	}
	for _, attached := range []bool{false, true} {
		if attached {
			f.startAdapter()
			f.acquire()
			defer f.renew()()
		}
		restartRefused(t, restartSpawn(t, f.dir, f.binary, f.args))
		after, err := os.Lstat(f.socket)
		if err != nil || !os.SameFile(before, after) {
			t.Fatal("live daemon socket changed")
		}
		current, err := os.ReadFile(filepath.Join(f.dir, "description.json"))
		if err != nil || string(current) != string(description) {
			t.Fatal("live daemon description changed")
		}
	}
	f.status(func(s restartStatus) bool { return s.Grant.GrantActive == 1 })
	restartLock(t, f.socket, true, f.core.cmd.Process.Pid)
}

func TestDaemonRecoversStaleSocketAfterCrash(t *testing.T) {
	f := newRestartFixture(t)
	ctx, cancel := context.WithTimeout(context.Background(), restartWait)
	defer cancel()
	client, err := Dial(ctx, f.options)
	if err != nil {
		t.Fatal(err)
	}
	clock, err := client.LocalJogClock()
	if err != nil {
		t.Fatal(err)
	}
	f.core.signal(t, syscall.SIGKILL)
	_ = client.Close()
	if _, err := os.Lstat(f.socket); err != nil {
		t.Fatalf("crash did not leave socket: %v", err)
	}
	restartLock(t, f.socket, false, f.core.cmd.Process.Pid)
	f.startCore()
	client, err = Dial(ctx, f.options)
	if err != nil {
		t.Fatal(err)
	}
	defer client.Close()
	next, err := client.LocalJogClock()
	if err != nil || next.Incarnation == clock.Incarnation {
		t.Fatalf("daemon incarnation did not change: %+v %v", next, err)
	}
	_ = client.Close()
	f.core.signal(t, syscall.SIGTERM)
	if _, err := os.Lstat(f.socket); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("clean shutdown left socket: %v", err)
	}
	restartLock(t, f.socket, false, f.core.cmd.Process.Pid)
	f.startCore()
}

func TestAdapterRefusesLiveControlSocket(t *testing.T) {
	f := newRestartFixture(t)
	f.startAdapter()
	f.acquire()
	defer f.renew()()
	before, err := os.Lstat(f.controlSocket)
	if err != nil {
		t.Fatal(err)
	}
	restartRefused(t, restartSpawn(t, f.dir, f.adapterBinary, f.adapterArgs))
	after, err := os.Lstat(f.controlSocket)
	if err != nil || !os.SameFile(before, after) {
		t.Fatal("live control socket changed")
	}
	f.service.must("renew", nil)
	f.status(func(s restartStatus) bool { return s.Grant.GrantActive == 1 })
	restartLock(t, f.controlSocket, true, f.adapter.cmd.Process.Pid)
	restartLock(t, filepath.Join(f.dir, "jog.sock"), true, f.adapter.cmd.Process.Pid)
}
