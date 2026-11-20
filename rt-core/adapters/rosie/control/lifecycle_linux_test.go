//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

type lifecycleFixture struct {
	controller *Controller
	client     *ipcclient.Client
	fence      Fence
}

// Fixture operations are quiescent when replacing this boundary. Renewals run
// independently, so serialize replacement with their authority transaction.
func replaceNativeTestClient(c *Controller, client nativeClient) {
	c.authorityMu.Lock()
	defer c.authorityMu.Unlock()
	c.mu.Lock()
	c.core = client
	c.mu.Unlock()
}

func startLifecycleCore(t *testing.T) lifecycleFixture {
	return startLifecycleCoreConfig(t, "machines/simulation/simulation-program.json")
}

func startLifecycleCoreConfig(t *testing.T, configuration string, extra ...string) lifecycleFixture {
	t.Helper()
	return startLifecycleCoreConfigArmed(t, configuration, true, extra...)
}

func startLifecycleCoreConfigArmed(t *testing.T, configuration string, armed bool, extra ...string) lifecycleFixture {
	t.Helper()
	root, err := filepath.Abs("../../..")
	if err != nil {
		t.Fatal(err)
	}
	binaryPath := simfixture.Locate(root, "rosie-rt-core-sim")
	if _, err := os.Stat(binaryPath); err != nil {
		t.Fatal("run make sim first: ", err)
	}
	dir, err := os.MkdirTemp("", "ctl-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	configPath := simfixture.MachineConfigPath(root, configuration)
	if filepath.IsAbs(configuration) {
		configPath = configuration
	}
	compiled, args, err := simfixture.Compile(root, configPath, filepath.Join(dir, "config"))
	simfixture.Must(t, err)
	socket := filepath.Join(dir, "ipc.sock")

	args = append(args, extra...)
	process := exec.Command(binaryPath, append(args, "--socket-path", socket)...)
	simfixture.SpawnLogged(t, process, filepath.Join(dir, "core.log"), 3*time.Second, 9000, nil)

	var client *ipcclient.Client
	var controller *Controller
	var stopRenew func()
	t.Cleanup(func() {
		if stopRenew != nil {
			stopRenew()
		}
		if controller != nil {
			_ = controller.Close()
		}
		if client != nil {
			_ = client.Close()
		}
	})
	simfixture.WaitVerified(t, filepath.Join(dir, "metrics.json"), 5*time.Second, 20*time.Millisecond, func(state simfixture.Verified) bool { return state.ConfigVerifiedMask == 511 })
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	client, err = ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: filepath.Base(compiled), Backend: "simulation"})
	if err != nil {
		t.Fatal(err)
	}
	binding := Binding{PairID: "lifecycle-test", Revision: 7, ConfigurationSHA256: filepath.Base(compiled)}
	controller, err = New(client, binding)
	if err != nil {
		t.Fatal(err)
	}
	grant, err := controller.Acquire("lifecycle-test", binding)
	if err != nil {
		t.Fatal(err)
	}
	fence := Fence{grant.Session, grant.Generation}
	stopRenew = simfixture.Renew(context.Background(), 100*time.Millisecond, func() bool {
		_, _ = controller.Renew(fence)
		return true
	})
	t.Cleanup(stopRenew)
	if armed {
		armLifecycle(t, controller, client, fence)
	}
	return lifecycleFixture{controller, client, fence}
}

func armLifecycle(t *testing.T, c *Controller, client *ipcclient.Client, fence Fence) {
	t.Helper()
	if _, err := c.Enable(fence, 511); err != nil {
		t.Fatal(err)
	}
	if _, err := c.Arm(fence); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	state, err := client.Wait(ctx, func(s ipcclient.Status) bool {
		if s.Armed != 1 || s.AxisEnableMask != 511 || len(s.Axes) != 9 {
			return false
		}
		return ipcclient.AllOperationEnabled(s)
	})
	if err != nil {
		t.Fatalf("drive readiness: %+v %v", state, err)
	}
}

// The lifecycle suite uses a stationary, valid .rdt program so its assertions
// concern native handle ownership rather than the motion planner's geometry.
func stationaryProgram(t *testing.T, name string) []byte {
	t.Helper()
	const duration = 2.0
	samples := []densejointtraj.Sample{{TimeS: 0}, {TimeS: duration}}
	segments := []densejointtraj.Segment{{Kind: "freespace", SourceID: "test-segment", Samples: samples}}
	limits := make([]float64, 9)
	for i := range limits {
		limits[i] = .1
	}
	header := densejointtraj.Header{
		Schema: densejointtraj.Schema, PlanID: name, ProgramID: "test-program", ProgramDigest: "sha256:" + strings.Repeat("0", 64),
		ManifestRevision: 7, PlanRevision: 1,
		Axes:     densejointtraj.HeaderAxes{AxisCount: 9, AxisMask: 511, PositionUnit: "rad", VelocityUnit: "rad/s"},
		Sampling: densejointtraj.Sampling{DtS: duration, TotalSampleCount: 2, TotalDurationS: duration},
		Limits:   densejointtraj.Limits{MaxAbsQdRadS: limits},
		Segments: []densejointtraj.SegmentMeta{{Index: 0, Kind: "freespace", SourceID: "test-segment", SampleCount: 2, DurationS: duration,
			Block: densejointtraj.BlockMeta{}}},
		SampleEncoding: densejointtraj.EncodingMeta{Encoding: densejointtraj.SampleEncoding, RecordBytes: densejointtraj.RecordBytes},
	}
	blob, err := densejointtraj.Encode(header, segments)
	if err != nil {
		t.Fatal(err)
	}
	return blob
}

func mustPrepareProgram(t *testing.T, f lifecycleFixture, name string) *Program {
	t.Helper()
	program, err := f.controller.PrepareProgram(f.fence, stationaryProgram(t, name))
	if err != nil {
		t.Fatal(err)
	}
	return program
}

func assertNativeUnknown(t *testing.T, client *ipcclient.Client, handle uint64) {
	t.Helper()
	_, err := client.StartTrajectory(context.Background(), handle)
	var rejected *ipcclient.RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonUnknownHandle {
		t.Fatalf("handle %d should be absent: %v", handle, err)
	}
}

func waitActiveProgram(t *testing.T, client *ipcclient.Client, handle, sequence, afterNS uint64) ipcclient.MotionState {
	t.Helper()
	for deadline := time.Now().Add(750 * time.Millisecond); ; {
		motion, err := client.Motion()
		if err == nil && motion.TrajectoryID == handle && motion.CommandSequence == sequence && motion.TimeNS > afterNS {
			if motion.Done {
				t.Fatalf("program unexpectedly terminated: %+v", motion)
			}
			return motion
		}
		if time.Now().After(deadline) {
			t.Fatalf("missing correlated active program observation: %+v %v", motion, err)
		}
		time.Sleep(5 * time.Millisecond)
	}
}

func TestProgramReplacementReclaimsSupersededNativeHandles(t *testing.T) {
	f := startLifecycleCore(t)
	var previous *Program
	for i := 0; i < 9; i++ {
		program := mustPrepareProgram(t, f, fmt.Sprintf("replacement-%d", i))
		if previous != nil {
			assertNativeUnknown(t, f.client, previous.handle)
		}
		previous = program
	}
	if _, err := f.controller.StartProgram(f.fence, previous.Identity); err != nil {
		t.Fatalf("latest replacement did not remain prepared: %v", err)
	}
}

func TestFailedProgramReplacementRetainsPreviousPreparation(t *testing.T) {
	f := startLifecycleCore(t)
	previous := mustPrepareProgram(t, f, "previous")
	if _, err := f.controller.PrepareProgram(f.fence, []byte("invalid .rdt")); err == nil {
		t.Fatal("malformed replacement accepted")
	}
	var extra []uint64
	for i := 0; i < 2; i++ {
		handle, err := f.controller.Prepare(f.fence, 511, []ipcclient.Point{
			{Time: 0, Position: make([]float64, 9)}, {Time: time.Second, Position: make([]float64, 9)},
		})
		if err != nil {
			t.Fatal(err)
		}
		extra = append(extra, handle)
	}
	_, err := f.controller.PrepareProgram(f.fence, stationaryProgram(t, "capacity-failure"))
	var rejected *ipcclient.RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ipcclient.ReasonCapacity {
		t.Fatalf("expected native capacity rejection: %v", err)
	}
	for _, handle := range extra {
		if _, err := f.controller.Discard(f.fence, handle); err != nil {
			t.Fatal(err)
		}
	}
	if _, err := f.controller.StartProgram(f.fence, previous.Identity); err != nil {
		t.Fatalf("rejected replacement destroyed previous preparation: %v", err)
	}
}

// This wrapper executes the real native preparation, then controls only its
// return to the adapter. It exposes the post-commit publication window without
// a daemon test hook, scheduler sleep or replacement of the transport codec.
type committedPreparation struct {
	nativeClient
	committed   chan uint64
	release     chan struct{}
	returnErr   error
	releaseOnce sync.Once
}

func (c *committedPreparation) releaseCommit() { c.releaseOnce.Do(func() { close(c.release) }) }

func (c *committedPreparation) PrepareTrajectory(ctx context.Context, id uint64, mask uint32, points []ipcclient.Point) (uint64, error) {
	handle, err := c.nativeClient.PrepareTrajectory(ctx, id, mask, points)
	if err != nil {
		return handle, err
	}
	if c.committed != nil {
		c.committed <- handle
		<-c.release
	}
	if c.returnErr != nil {
		return 0, c.returnErr
	}
	return handle, nil
}

func waitCommitted(t *testing.T, barrier *committedPreparation) uint64 {
	t.Helper()
	select {
	case handle := <-barrier.committed:
		return handle
	case <-time.After(2 * time.Second):
		t.Fatal("preparation did not reach native commit")
		return 0
	}
}

func waitPreparationError(t *testing.T, result <-chan error) error {
	t.Helper()
	select {
	case err := <-result:
		if err == nil {
			t.Fatal("cancelled preparation unexpectedly published")
		}
		return err
	case <-time.After(3 * time.Second):
		t.Fatal("preparation rollback did not finish")
		return nil
	}
}

func TestLatePreparationAuthorizationFailureDiscardsNewHandle(t *testing.T) {
	f := startLifecycleCore(t)
	previous := mustPrepareProgram(t, f, "previous")
	barrier := &committedPreparation{nativeClient: f.client, committed: make(chan uint64, 1), release: make(chan struct{})}
	t.Cleanup(barrier.releaseCommit)
	replaceNativeTestClient(f.controller, barrier)
	result := make(chan error, 1)
	blob := stationaryProgram(t, "late-revocation")
	go func() { _, err := f.controller.PrepareProgram(f.fence, blob); result <- err }()
	handle := waitCommitted(t, barrier)
	// Invalidate only the application fence, so native Stop cannot accidentally
	// supply the reclamation this transaction itself is required to perform.
	f.controller.mu.Lock()
	f.controller.grant.Generation++
	next := Fence{f.controller.grant.Session, f.controller.grant.Generation}
	f.controller.expires = time.Now().Add(500 * time.Millisecond)
	f.controller.mu.Unlock()
	barrier.releaseCommit()
	if err := waitPreparationError(t, result); err.Error() != "control_session_stale" {
		t.Fatalf("unexpected rollback: %v", err)
	}
	assertNativeUnknown(t, f.client, handle)
	if _, err := f.controller.StartProgram(next, previous.Identity); err != nil {
		t.Fatalf("late rejected replacement destroyed prior program: %v", err)
	}
}

func TestLostPreparationResultCleansKnownAttemptedHandle(t *testing.T) {
	f := startLifecycleCore(t)
	previous := mustPrepareProgram(t, f, "previous")
	lost := errors.New("test lost preparation acknowledgement")
	replaceNativeTestClient(f.controller, &committedPreparation{nativeClient: f.client, returnErr: lost})
	for i := 0; i < 7; i++ {
		if _, err := f.controller.PrepareProgram(f.fence, stationaryProgram(t, "lost-result")); !errors.Is(err, lost) {
			t.Fatalf("replacement %d leaked a committed handle or hid its error: %v", i, err)
		}
		assertNativeUnknown(t, f.client, uint64(i+2))
	}
	if _, err := f.controller.StartProgram(f.fence, previous.Identity); err != nil {
		t.Fatalf("lost replacement result destroyed prior program: %v", err)
	}
}

func TestStopInhibitsWhilePreparedProgramPublicationIsBlocked(t *testing.T) {
	f := startLifecycleCore(t)
	previous := mustPrepareProgram(t, f, "previous")
	barrier := &committedPreparation{nativeClient: f.client, committed: make(chan uint64, 1), release: make(chan struct{})}
	t.Cleanup(barrier.releaseCommit)
	replaceNativeTestClient(f.controller, barrier)
	prepared := make(chan error, 1)
	blob := stationaryProgram(t, "stopped-replacement")
	go func() { _, err := f.controller.PrepareProgram(f.fence, blob); prepared <- err }()
	handle := waitCommitted(t, barrier)
	type stopped struct {
		grant Grant
		err   error
	}
	stopResult := make(chan stopped, 1)
	go func() { grant, err := f.controller.Stop(f.fence, false); stopResult <- stopped{grant, err} }()
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	state, err := f.client.Wait(ctx, func(s ipcclient.Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	if err != nil {
		barrier.releaseCommit()
		t.Fatalf("urgent Stop waited behind publication: %+v %v", state, err)
	}
	if grant, err := f.controller.Renew(f.fence); err != nil || !grant.Stopping {
		t.Errorf("Renew must keep the lease alive while Stop fences publication: %+v %v", grant, err)
	}
	barrier.releaseCommit()
	_ = waitPreparationError(t, prepared)
	select {
	case result := <-stopResult:
		if result.err != nil {
			t.Fatal(result.err)
		}
		armLifecycle(t, f.controller, f.client, Fence{result.grant.Session, result.grant.Generation})
	case <-time.After(3 * time.Second):
		t.Fatal("Stop did not finish its writer fence")
	}
	assertNativeUnknown(t, f.client, handle)
	assertNativeUnknown(t, f.client, previous.handle)
	_, _, execution, err := f.controller.Snapshot()
	if err != nil || execution.State != "cancelled" {
		t.Fatalf("stopped preparation was republished: %+v %v", execution, err)
	}
}

func TestHTTPDiscardUsesNativeResultAndRetiresPreparedProgram(t *testing.T) {
	f := startLifecycleCore(t)
	program := mustPrepareProgram(t, f, "discarded-program")
	handler := NewHandler(f.controller)
	discard := func(handle uint64) (int, Response) {
		raw, _ := json.Marshal(Request{Schema: "rosie.rt-control.request.v1", Operation: "discard_trajectory", Fence: f.fence, Handle: handle})
		recorder := httptest.NewRecorder()
		handler.ServeHTTP(recorder, httptest.NewRequest(http.MethodPost, "/v1/control", bytes.NewReader(raw)))
		var response Response
		if err := json.Unmarshal(recorder.Body.Bytes(), &response); err != nil {
			t.Fatal(err)
		}
		return recorder.Code, response
	}
	for attempt := 0; attempt < 2; attempt++ {
		status, response := discard(program.handle)
		if status != http.StatusOK || response.Error != "" || response.Sequence == 0 {
			t.Fatalf("prepared Discard response: %d %+v", status, response)
		}
	}
	assertNativeUnknown(t, f.client, program.handle)
	if _, err := f.controller.StartProgram(f.fence, program.Identity); err == nil {
		t.Fatal("discarded program remained published")
	}
	active := mustPrepareProgram(t, f, "active-program")
	sequence, err := f.controller.StartProgram(f.fence, active.Identity)
	if err != nil {
		t.Fatal(err)
	}
	before := waitActiveProgram(t, f.client, active.handle, sequence, 0)
	status, response := discard(active.handle)
	if status != http.StatusConflict || response.NativeResult == nil || response.NativeResult.Reason != ipcclient.ReasonModeConflict || response.NativeResult.Handle != active.handle {
		t.Fatalf("active Discard did not expose native rejection: %d %+v", status, response)
	}
	_ = waitActiveProgram(t, f.client, active.handle, sequence, before.TimeNS)
}

func TestFailedNativeConnectionInvalidatesPreparedPublication(t *testing.T) {
	f := startLifecycleCore(t)
	previous := mustPrepareProgram(t, f, "transport-lost")
	if err := f.client.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := f.controller.PrepareProgram(f.fence, stationaryProgram(t, "replacement")); err == nil {
		t.Fatal("replacement succeeded after native connection loss")
	}
	f.controller.mu.Lock()
	program, execution := f.controller.program, f.controller.execution
	f.controller.mu.Unlock()
	if program != nil || execution.State != "faulted" {
		t.Fatalf("lost transport retained prepared publication %d: %+v", previous.handle, execution)
	}
}

type lostDiscardResult struct {
	nativeClient
	err error
}

func (c *lostDiscardResult) DiscardTrajectory(ctx context.Context, handle uint64) (uint64, error) {
	sequence, err := c.nativeClient.DiscardTrajectory(ctx, handle)
	if err != nil {
		return sequence, err
	}
	return sequence, c.err
}

func TestLostDiscardResultInvalidatesMatchingProgramPublication(t *testing.T) {
	f := startLifecycleCore(t)
	program := mustPrepareProgram(t, f, "discard-result-lost")
	lost := errors.New("test lost discard acknowledgement")
	replaceNativeTestClient(f.controller, &lostDiscardResult{nativeClient: f.client, err: lost})
	if _, err := f.controller.Discard(f.fence, program.handle); !errors.Is(err, lost) {
		t.Fatalf("lost Discard result was hidden: %v", err)
	}
	assertNativeUnknown(t, f.client, program.handle)
	if _, err := f.controller.StartProgram(f.fence, program.Identity); err == nil {
		t.Fatal("ambiguous Discard retained prepared publication")
	}
	_, _, execution, err := f.controller.Snapshot()
	if err != nil || execution.State != "faulted" {
		t.Fatalf("ambiguous Discard state: %+v %v", execution, err)
	}
}

func TestHTTPPreparePreservesRollbackFailureAlongsideNativeRejection(t *testing.T) {
	f := startLifecycleCore(t)
	mustPrepareProgram(t, f, "previous")
	for i := 0; i < 2; i++ {
		if _, err := f.controller.Prepare(f.fence, 511, []ipcclient.Point{
			{Time: 0, Position: make([]float64, 9)}, {Time: time.Second, Position: make([]float64, 9)},
		}); err != nil {
			t.Fatal(err)
		}
	}
	lost := errors.New("test lost rollback acknowledgement")
	replaceNativeTestClient(f.controller, &lostDiscardResult{nativeClient: f.client, err: lost})
	request := httptest.NewRequest(http.MethodPost, "/v1/program", bytes.NewReader(stationaryProgram(t, "rejected")))
	request.Header.Set("X-Control-Session", f.fence.Session)
	request.Header.Set("X-Control-Generation", fmt.Sprint(f.fence.Generation))
	recorder := httptest.NewRecorder()
	NewHandler(f.controller).ServeHTTP(recorder, request)
	var response Response
	if err := json.Unmarshal(recorder.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	if recorder.Code != http.StatusConflict || response.NativeResult == nil || response.NativeResult.Reason != ipcclient.ReasonCapacity ||
		!strings.Contains(response.Error, "discard preparation") || !strings.Contains(response.Error, lost.Error()) {
		t.Fatalf("HTTP response hid rollback failure: %d %+v", recorder.Code, response)
	}
}
