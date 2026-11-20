package seam

import (
	"context"
	"errors"
	"os"
	"path/filepath"
	"regexp"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// spareRunner is a CommandRunner that can also start spares, without processes.
type spareRunner struct {
	fakeRunner
	mu      sync.Mutex
	started []*fakeSpare
	exited  bool
	at      time.Time
}

func (runner *spareRunner) Start(Command) (Spare, error) {
	runner.mu.Lock()
	defer runner.mu.Unlock()
	at := runner.at
	if at.IsZero() {
		at = time.Now()
	}
	spare := &fakeSpare{response: runner.response, exited: runner.exited, at: at}
	runner.started = append(runner.started, spare)
	return spare, nil
}

func (runner *spareRunner) starts() int {
	return len(runner.snapshot())
}

// snapshot is the spares started so far, read under the lock refills append
// under.
func (runner *spareRunner) snapshot() []*fakeSpare {
	runner.mu.Lock()
	defer runner.mu.Unlock()
	return append([]*fakeSpare(nil), runner.started...)
}

type fakeSpare struct {
	response  []byte
	exited    bool
	at        time.Time
	served    atomic.Int64
	discarded atomic.Bool
}

func (spare *fakeSpare) Serve(context.Context, []byte) ([]byte, []byte, error) {
	spare.served.Add(1)
	return spare.response, nil, nil
}
func (spare *fakeSpare) Exited() bool         { return spare.exited }
func (spare *fakeSpare) StartedAt() time.Time { return spare.at }
func (spare *fakeSpare) Discard()             { spare.discarded.Store(true) }

func waitFor(t *testing.T, what string, ok func() bool) {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for !ok() {
		if time.Now().After(deadline) {
			t.Fatalf("timed out waiting for %s", what)
		}
		time.Sleep(time.Millisecond)
	}
}

func spareWorker(t *testing.T, runner *spareRunner) *Worker {
	t.Helper()
	root := t.TempDir()
	if err := os.MkdirAll(filepath.Join(root, "python"), 0o755); err != nil {
		t.Fatal(err)
	}
	worker, err := New(Config{WeldPlannerRoot: root, Runner: runner, Spares: 2})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	t.Cleanup(worker.Close)
	return worker
}

func frames(t *testing.T, worker *Worker) {
	t.Helper()
	if _, err := worker.SampleSeamFrames(context.Background(), SeamFramesRequest{
		Curve: []byte(`{}`), Reference: []byte(`{}`),
	}); err != nil {
		t.Fatalf("SampleSeamFrames: %v", err)
	}
}

func TestNoSpareStartsUntilTheWorkerIsWarmed(t *testing.T) {
	// New touches nothing, and a request before Warm runs the way it always did.
	runner := &spareRunner{fakeRunner: fakeRunner{response: []byte(`{"frames":{}}`)}}
	worker := spareWorker(t, runner)
	frames(t, worker)
	if runner.starts() != 0 || runner.calls.Load() != 1 {
		t.Fatalf("want one cold run and no spares, got %d starts and %d runs", runner.starts(), runner.calls.Load())
	}
}

func TestAWarmedRequestIsServedByASpareAndTheSpareIsReplaced(t *testing.T) {
	runner := &spareRunner{fakeRunner: fakeRunner{response: []byte(`{"frames":{}}`)}}
	worker := spareWorker(t, runner)
	worker.Warm()
	waitFor(t, "two spares", func() bool { return len(worker.spares) == 2 })

	frames(t, worker)
	if runner.calls.Load() != 0 {
		t.Fatalf("a spare was waiting and the request still ran cold")
	}
	served := 0
	for _, spare := range runner.snapshot() {
		served += int(spare.served.Load())
	}
	if served != 1 {
		t.Fatalf("want exactly one spare to serve, got %d", served)
	}
	// ONE request per process: the one that answered is never handed back.
	waitFor(t, "a replacement", func() bool { return runner.starts() == 3 && len(worker.spares) == 2 })
}

func TestASpareThatDiedIsDiscardedAndTheRequestRunsCold(t *testing.T) {
	runner := &spareRunner{fakeRunner: fakeRunner{response: []byte(`{"frames":{}}`)}, exited: true}
	worker := spareWorker(t, runner)
	worker.Warm()
	waitFor(t, "two spares", func() bool { return len(worker.spares) == 2 })
	frames(t, worker)
	if runner.calls.Load() != 1 {
		t.Fatalf("want the request to run cold, got %d cold runs", runner.calls.Load())
	}
	for _, spare := range runner.snapshot()[:2] {
		if spare.served.Load() != 0 || !spare.discarded.Load() {
			t.Fatalf("a dead spare was served or kept")
		}
	}
}

func TestASpareOlderThanAnEditToTheWorkerIsRetired(t *testing.T) {
	// A spare imported the code as it was when it started.
	runner := &spareRunner{
		fakeRunner: fakeRunner{response: []byte(`{"frames":{}}`)},
		at:         time.Now().Add(-time.Hour),
	}
	worker := spareWorker(t, runner)
	worker.Warm()
	waitFor(t, "two spares", func() bool { return len(worker.spares) == 2 })
	if err := os.WriteFile(filepath.Join(worker.root, "python", "edited.py"), []byte("x = 1\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	frames(t, worker)
	if runner.calls.Load() != 1 {
		t.Fatalf("a spare older than the edit answered")
	}
}

func TestCloseEndsTheSparesAndStartsNoMore(t *testing.T) {
	runner := &spareRunner{fakeRunner: fakeRunner{response: []byte(`{"frames":{}}`)}}
	worker := spareWorker(t, runner)
	worker.Warm()
	waitFor(t, "two spares", func() bool { return len(worker.spares) == 2 })
	worker.Close()
	for _, spare := range runner.snapshot() {
		if !spare.discarded.Load() {
			t.Fatalf("a waiting spare survived Close")
		}
	}
	worker.Warm()
	frames(t, worker)
	time.Sleep(20 * time.Millisecond)
	if runner.starts() != 2 {
		t.Fatalf("spares started after Close: %d", runner.starts())
	}
}

func TestAnExecSpareAnswersItsRequestOnStdin(t *testing.T) {
	spare, err := ExecRunner{}.Start(Command{Name: "/bin/sh", Args: []string{"-c", "cat"}, MaxOutputBytes: 1 << 20})
	if err != nil {
		t.Skipf("no /bin/sh: %v", err)
	}
	// Bigger than a pipe holds, as a request carrying a STEP is.
	payload := make([]byte, 200<<10)
	for i := range payload {
		payload[i] = byte('a' + i%26)
	}
	stdout, _, err := spare.Serve(context.Background(), payload)
	if err != nil {
		t.Fatalf("Serve: %v", err)
	}
	if string(stdout) != string(payload) {
		t.Fatalf("got %d bytes back, want %d", len(stdout), len(payload))
	}
	if !spare.Exited() {
		t.Fatalf("a served spare is still running")
	}
}

func TestAnExecSpareIsKilledWhenItsRequestIsCanceled(t *testing.T) {
	spare, err := ExecRunner{}.Start(Command{Name: "/bin/sh", Args: []string{"-c", "cat >/dev/null; sleep 30"}, MaxOutputBytes: 1 << 20})
	if err != nil {
		t.Skipf("no /bin/sh: %v", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
	defer cancel()
	started := time.Now()
	_, _, err = spare.Serve(ctx, []byte("x"))
	if !errors.Is(err, context.DeadlineExceeded) {
		t.Fatalf("want the deadline, got %v", err)
	}
	if waited := time.Since(started); waited > 5*time.Second {
		t.Fatalf("Serve waited %s for a killed spare", waited)
	}
}

func TestTheRealWorkerAnswersTheSameFromASpare(t *testing.T) {
	root := weldPlannerRoot(t)
	step, err := os.ReadFile(filepath.Join(root, "data", "fixtures", "bracket_b.step"))
	if err != nil {
		t.Skipf("fixture: %v", err)
	}
	worker, err := New(Config{WeldPlannerRoot: root, Spares: 1})
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	t.Cleanup(worker.Close)
	ctx, cancel := context.WithTimeout(context.Background(), 4*time.Minute)
	defer cancel()

	cold, err := worker.DetectJoints(ctx, JointsRequest{STEP: step})
	if err != nil {
		t.Fatalf("cold DetectJoints: %v", err)
	}
	worker.Warm()
	waitFor(t, "a spare", func() bool { return len(worker.spares) == 1 })
	warm, err := worker.DetectJoints(ctx, JointsRequest{STEP: step})
	if err != nil {
		t.Fatalf("spare DetectJoints: %v", err)
	}
	// Two fresh processes differ in exactly one thing: `source.path`, the temp
	// file each wrote the request's STEP to. Everything else must match.
	scratch := regexp.MustCompile(`/seam-worker-[A-Za-z0-9_]+/`)
	normal := func(raw []byte) string { return scratch.ReplaceAllString(string(raw), "/seam-worker-X/") }
	if normal(warm.JSON) != normal(cold.JSON) {
		t.Fatalf("a spare answered differently from a fresh process (%d vs %d bytes)", len(warm.JSON), len(cold.JSON))
	}
}
