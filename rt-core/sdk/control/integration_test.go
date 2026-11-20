//go:build linux

package control

import (
	"context"
	"errors"
	"fmt"
	"io"
	"net/http"
	"net/http/httptrace"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"sync"
	"syscall"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

type service struct {
	client                         *Client
	core                           *exec.Cmd
	root, dir, build, socket, hash string
}

var must = simfixture.Must

func process(t *testing.T, dir, name, binary string, args ...string) *exec.Cmd {
	t.Helper()
	return simfixture.SpawnLogged(t, exec.Command(binary, args...), filepath.Join(dir, name+".log"), 5*time.Second, 1800, simfixture.CheckExit(t))
}

func startService(t *testing.T, remoteArgs ...string) *service {
	t.Helper()
	return startServiceWith(t, "config/machines/simulation/simulation-program.json", remoteArgs...)
}

// startServiceWith compiles machine (relative to rt-core) instead of the
// default simulation machine.
func startServiceWith(t *testing.T, machine string, remoteArgs ...string) *service {
	t.Helper()
	root, err := filepath.Abs("../..")
	must(t, err)
	dir, err := os.MkdirTemp("", "s-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "sdk-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	if !filepath.IsAbs(machine) {
		machine = filepath.Join(root, machine)
	}
	compiled, args, err := simfixture.Compile(root, machine, filepath.Join(build, "config"))
	simfixture.Must(t, err)
	coreSocket := filepath.Join(dir, "ipc.sock")
	core := process(t, build, "core", simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", coreSocket)...)
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 3*time.Second, time.Millisecond, func(s simfixture.Verified) bool { return s.ConfigVerifiedMask == 511 })
	socket := filepath.Join(dir, "control.sock")
	args = simfixture.ControlArgs(coreSocket, socket, filepath.Base(compiled), "bench", 7)
	process(t, build, "control", simfixture.Locate(root, "rt-control"), append(args, remoteArgs...)...)
	client, err := Dial(UnixPath(socket))
	must(t, err)
	t.Cleanup(func() { client.Close() })
	poll(t, 3*time.Second, func() bool {
		ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
		defer cancel()
		state, err := client.Status(ctx)
		return err == nil && state.Motion.TimeNS != 0
	})
	return &service{client: client, core: core, root: root, dir: dir, build: build, socket: socket, hash: filepath.Base(compiled)}
}

var poll = func(t *testing.T, budget time.Duration, predicate func() bool) {
	t.Helper()
	simfixture.PollTest(t, budget, time.Millisecond, predicate)
}

func (s *service) acquire(t *testing.T, renewal bool) {
	t.Helper()
	_, err := s.client.Acquire(context.Background(), "motion-server", Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: s.hash})
	must(t, err)
	if renewal {
		_, err = s.client.StartRenewal(context.Background(), 50*time.Millisecond)
		must(t, err)
	}
}

func TestResetReacquireRenewsFreshSession(t *testing.T) {
	s := startService(t)
	s.acquire(t, false)
	old := s.client.Fence()
	_, err := s.client.Command(context.Background(), &Request{Operation: "reset_fault", AxisMask: 511})
	must(t, err)
	if s.client.Fence() != (Fence{}) {
		t.Fatal("Reset retained native-retired session")
	}
	s.acquire(t, false)
	fresh := s.client.Fence()
	if fresh.Session == "" || fresh.Session == old.Session {
		t.Fatal("Acquire did not replace reset session")
	}
	grant, err := s.client.Renew(context.Background())
	must(t, err)
	if grant.Session != fresh.Session || grant.Generation != fresh.Generation {
		t.Fatal("Renew used old session")
	}
	_, err = s.client.Release(context.Background())
	must(t, err)
}

func (s *service) enable(t *testing.T) {
	t.Helper()
	_, err := s.client.Enable(context.Background(), 511)
	must(t, err)
	_, err = s.client.Arm(context.Background())
	must(t, err)
	s.waitStatus(t, func(v Status) bool {
		if v.Core.Armed != 1 || v.Core.AxisEnableMask != 511 {
			return false
		}
		return ipcclient.AllOperationEnabled(v.Core)
	})
}

func (s *service) waitStatus(t *testing.T, predicate func(Status) bool) Status {
	t.Helper()
	var state Status
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Second)
	defer cancel()
	poll(t, 4*time.Second, func() bool { var err error; state, err = s.client.Status(ctx); return err == nil && predicate(state) })
	return state
}

// oracleProgram applies the frozen Python fixture transformation using the
// existing dense codec, preserving independent sample-count and count assertions.
func oracleProgram(t *testing.T, root string, torch bool) []byte {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join(root, "adapters/rosie/densejointtraj/testdata/corpus/dense_joint_trajectory_reference.rdt"))
	must(t, err)
	plan, err := densejointtraj.Decode(raw)
	must(t, err)
	initial := plan.Segments[0].Samples[0].QRad
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			s := &plan.Segments[i].Samples[j]
			for axis := range s.QRad {
				s.QRad[axis] = (s.QRad[axis] - initial[axis]) * .05
				s.QdRadS[axis] *= .05
			}
			if !torch {
				s.TorchOn = false
			}
		}
	}
	result, err := densejointtraj.Encode(plan.Header, plan.Segments)
	must(t, err)
	return result
}

func rejected(t *testing.T, err error, reason string) {
	t.Helper()
	var r *Rejected
	if !errors.As(err, &r) || r.Reason != reason {
		t.Fatalf("want rejection %q, got %v", reason, err)
	}
}

func TestSDKLifecycleMatchesPythonOracleSequence(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	c := s.client
	ctx := context.Background()
	description, err := c.Describe(ctx)
	must(t, err)
	if len(description.Axes) != 9 {
		t.Fatalf("axes: %d", len(description.Axes))
	}
	for i, axis := range description.Axes {
		if axis.ID != fmt.Sprintf("J%d", i+1) {
			t.Fatalf("axis %d: %s", i, axis.ID)
		}
	}
	program, err := c.PrepareProgram(ctx, oracleProgram(t, s.root, false))
	must(t, err)
	if program.Samples != 11 || program.RequiresProcessIO {
		t.Fatalf("program: %+v", program)
	}
	state, err := c.Status(ctx)
	must(t, err)
	if state.Core.Armed != 0 {
		t.Fatal("prepare armed motion")
	}
	wrong := program.Identity
	wrong.PlanRevision++
	_, err = c.StartProgram(ctx, wrong)
	rejected(t, err, "program_identity_mismatch")
	s.enable(t)
	_, err = c.StartProgram(ctx, program.Identity)
	must(t, err)
	state = s.waitStatus(t, func(v Status) bool { return v.Execution.State == "completed" && v.Core.Axes[0].PositionCounts > 15 })
	if state.Core.SafetyFaultMask != 0 || state.Execution.Generation != 1 {
		t.Fatalf("completed: %+v", state)
	}
	old := c.Fence()
	grant, err := c.Stop(ctx)
	must(t, err)
	if grant.Generation != old.Generation+1 || c.Fence().Generation != grant.Generation {
		t.Fatalf("stop fence: %+v", grant)
	}
	_, err = c.Command(ctx, &Request{Operation: "arm", Fence: old})
	rejected(t, err, "control_session_stale")
	s.waitStatus(t, func(v Status) bool { return v.Core.Armed == 0 && v.Core.AxisEnableMask == 0 })
	_, err = c.PrepareProgram(ctx, oracleProgram(t, s.root, true))
	rejected(t, err, "process_io_executor_not_qualified")
	_, err = c.Release(ctx)
	must(t, err)
	if c.Fence().Session != "" {
		t.Fatal("release retained fence")
	}
	_, err = c.Arm(ctx)
	rejected(t, err, "control_session_stale")
}

func TestSDKRenewalReportsStoppingWithoutError(t *testing.T) {
	s := startService(t)
	s.acquire(t, false)
	c := s.client
	must(t, s.core.Process.Signal(syscall.SIGSTOP))
	defer s.core.Process.Signal(syscall.SIGCONT)
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	stopped := make(chan error, 1)
	go func() { _, err := c.Stop(ctx); stopped <- err }()
	updates, err := c.StartRenewal(ctx, time.Millisecond)
	must(t, err)
	select {
	case update := <-updates:
		must(t, update.Err)
		if !update.Grant.Stopping || update.Grant.Generation != 2 {
			t.Fatalf("stopping grant: %+v", update)
		}
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	c.StopRenewal()
	must(t, s.core.Process.Signal(syscall.SIGCONT))
	must(t, <-stopped)
	if c.Grant().Stopping {
		t.Fatal("completed Stop retained stopping flag")
	}
}

func TestSDKRetryWithRequestIDDoesNotDoubleStart(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	s.enable(t)
	prepared, err := s.client.PrepareTrajectory(context.Background(), 511, []Point{{Position: make([]float64, 9)}, {Time: 2 * time.Second, Position: make([]float64, 9)}})
	must(t, err)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	ctx = httptrace.WithClientTrace(ctx, &httptrace.ClientTrace{WroteRequest: func(info httptrace.WroteRequestInfo) {
		if info.Err == nil {
			cancel()
		}
	}})
	request := Request{Operation: "start_trajectory", Handle: prepared.Handle}
	_, err = s.client.Command(ctx, &request)
	if !errors.Is(err, ErrTransport) || !errors.Is(err, context.Canceled) {
		t.Fatalf("lost reply: %v", err)
	}
	if request.RequestID == nil || len(*request.RequestID) != 32 {
		t.Fatal("missing random 16-byte ID")
	}
	s.waitStatus(t, func(v Status) bool { return v.Execution.Generation == 1 })
	reply, err := s.client.Command(context.Background(), &request)
	must(t, err)
	state := s.waitStatus(t, func(v Status) bool { return v.Motion.CommandSequence == reply.Sequence })
	if state.Execution.Generation != 1 || state.Motion.CommandSequence != reply.Sequence {
		t.Fatalf("duplicate start: %+v %+v", reply, state)
	}
	_, err = s.client.StartTrajectory(context.Background(), prepared.Handle)
	rejected(t, err, "handle_started")
}

type roundTripFunc func(*http.Request) (*http.Response, error)

func (f roundTripFunc) RoundTrip(r *http.Request) (*http.Response, error) { return f(r) }

type gatedBody struct {
	io.ReadCloser
	entered, release chan struct{}
	once             sync.Once
	delivered        int
	ctx              context.Context
}

func (b *gatedBody) Read(p []byte) (int, error) {
	// Unverified test barrier: pass 64 KiB through the HTTP copy buffers before
	// holding the remaining body, so Stop overlaps actual wire progress.
	const prefixBytes = 64 << 10
	if b.delivered < prefixBytes {
		if len(p) > prefixBytes-b.delivered {
			p = p[:prefixBytes-b.delivered]
		}
		n, err := b.ReadCloser.Read(p)
		b.delivered += n
		return n, err
	}
	b.once.Do(func() { close(b.entered) })
	select {
	case <-b.release:
		return b.ReadCloser.Read(p)
	case <-b.ctx.Done():
		return 0, b.ctx.Err()
	}
}

func TestSDKBulkDoesNotBlockStop(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	s.enable(t)
	c := s.client
	if c.bulk.Transport == c.control.Transport || c.bulk.Transport == c.urgent.Transport {
		t.Fatal("shared connection pool")
	}
	entered, release := make(chan struct{}), make(chan struct{})
	var once sync.Once
	unblock := func() { once.Do(func() { close(release) }) }
	defer unblock()
	transport := c.bulk.Transport
	c.bulk.Transport = roundTripFunc(func(r *http.Request) (*http.Response, error) {
		r.Body = &gatedBody{ReadCloser: r.Body, entered: entered, release: release, ctx: r.Context()}
		return transport.RoundTrip(r)
	})
	t.Cleanup(func() { transport.(*http.Transport).CloseIdleConnections() })
	points := make([]Point, 250000)
	for i := range points {
		points[i] = Point{Time: time.Duration(i) * time.Millisecond, Position: make([]float64, 9)}
	}
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	uploaded := make(chan error, 1)
	go func() { _, err := c.PrepareTrajectory(ctx, 511, points); uploaded <- err }()
	select {
	case <-entered:
	case <-ctx.Done():
		t.Fatal("upload did not enter")
	}
	// Client write progress does not establish server admission. Stop may reach
	// the other connection first and correctly reject an unread old-generation
	// envelope. Observe the occupied slot so this test exercises an admitted body.
	s.waitStatus(t, func(v Status) bool { return v.Adapter.OpenPreparationSlots == 1 })
	select {
	case err := <-uploaded:
		t.Fatalf("admitted upload completed before Stop: %v", err)
	default:
	}
	old := c.Fence()
	stopCtx, stopCancel := context.WithTimeout(context.Background(), time.Second)
	defer stopCancel()
	_, err := c.Stop(stopCtx)
	must(t, err)
	if c.Fence().Generation != old.Generation+1 {
		t.Fatalf("Stop generation: before=%d after=%d", old.Generation, c.Fence().Generation)
	}
	poll(t, time.Second, func() bool {
		state, err := c.Status(stopCtx)
		return err == nil && state.Core.Armed == 0 && state.Core.AxisEnableMask == 0
	})
	select {
	case err := <-uploaded:
		t.Fatalf("admitted upload completed while body remained gated: %v", err)
	default:
	}
	unblock()
	select {
	case err := <-uploaded:
		rejected(t, err, "control_session_stale")
	case <-ctx.Done():
		t.Fatal("upload did not complete")
	}
}

// A queued bulk envelope may lose to the urgent connection. Force that ordering
// so the expected stale refusal cannot be mistaken for a Stop-path regression.
func TestSDKBulkStopBeforeAdmission(t *testing.T) {
	// Measured fixture: nine axes (mask 511), two preparation slots, one generation per Stop.
	s := startService(t)
	s.acquire(t, true)
	s.enable(t)
	c := s.client
	entered, release := make(chan struct{}), make(chan struct{})
	var once sync.Once
	unblock := func() { once.Do(func() { close(release) }) }
	defer unblock()
	transport := c.bulk.Transport
	c.bulk.Transport = roundTripFunc(func(r *http.Request) (*http.Response, error) {
		close(entered)
		select {
		case <-release:
			return transport.RoundTrip(r)
		case <-r.Context().Done():
			return nil, r.Context().Err()
		}
	})
	t.Cleanup(func() { transport.(*http.Transport).CloseIdleConnections() })
	// Unverified fixture deadline: 5 seconds; channels and observations establish ordering.
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	uploaded := make(chan error, 1)
	go func() {
		_, err := c.PrepareTrajectory(ctx, 511, []Point{{Position: make([]float64, 9)}, {Time: time.Second, Position: make([]float64, 9)}})
		uploaded <- err
	}()
	select {
	case <-entered:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	state, err := c.Status(ctx)
	must(t, err)
	if state.Adapter.OpenPreparationSlots != 2 {
		t.Fatalf("bulk admitted before barrier release: %+v", state.Adapter)
	}
	old := c.Fence()
	grant, err := c.Stop(ctx)
	must(t, err)
	if grant.Generation != old.Generation+1 {
		t.Fatalf("Stop generation: %d -> %d", old.Generation, grant.Generation)
	}
	s.waitStatus(t, func(v Status) bool { return v.Core.Armed == 0 && v.Core.AxisEnableMask == 0 })
	unblock()
	select {
	case err := <-uploaded:
		rejected(t, err, "control_session_stale")
		t.Logf("Stop generation %d -> %d before bulk admission: %v", old.Generation, grant.Generation, err)
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
}
