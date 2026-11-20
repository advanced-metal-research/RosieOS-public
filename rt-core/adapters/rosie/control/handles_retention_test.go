//go:build linux

package control

import (
	"context"
	"encoding/json"
	"net/http/httptest"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// retentionCore acknowledges every preparation with the requested handle,
// the native behaviour the adapter's handle numbering relies on.
type retentionCore struct{ statusViewCore }

func (*retentionCore) PrepareTrajectory(_ context.Context, id uint64, _ uint32, _ []ipcclient.Point) (uint64, error) {
	return id, nil
}

func retentionFixture() (*Controller, Fence) {
	view := statusViewFixture(0)
	core := &retentionCore{statusViewCore: *view.core.(*statusViewCore)}
	view.core = core
	view.grant = Grant{Session: "retention", Generation: 1}
	view.expires = time.Now().Add(time.Hour)
	return view, Fence{Session: "retention", Generation: 1}
}

// prepareTerminal prepares one handle through the production path and makes
// it terminal, as a completed loop cycle does.
func prepareTerminal(t *testing.T, c *Controller, f Fence, to HandleState) uint64 {
	t.Helper()
	id, err := c.prepare(f, context.Background(), 1, []ipcclient.Point{{Position: []float64{0}}})
	if err != nil {
		t.Fatal(err)
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	h := c.handles[id]
	if to == HandleConsumed {
		transitionHandle(h, HandleStarted)
	}
	if !transitionHandle(h, to) {
		t.Fatalf("handle %d: %s refused", id, to)
	}
	return id
}

func TestHandleHistoryIsBoundedAndKeepsLiveHandles(t *testing.T) {
	c, f := retentionFixture()
	// Live handles older than every terminal one must survive any pruning.
	prepared, err := c.prepare(f, context.Background(), 1, []ipcclient.Point{{Position: []float64{0}}})
	if err != nil {
		t.Fatal(err)
	}
	started, err := c.prepare(f, context.Background(), 1, []ipcclient.Point{{Position: []float64{0}}})
	if err != nil {
		t.Fatal(err)
	}
	c.mu.Lock()
	transitionHandle(c.handles[started], HandleStarted)
	c.mu.Unlock()

	states := []HandleState{HandleConsumed, HandleDiscarded, HandleSuperseded, HandleRetired}
	const total = 3*retainedTerminalHandles + 7
	var first, last uint64
	for i := 0; i < total; i++ {
		id := prepareTerminal(t, c, f, states[i%len(states)])
		if i == 0 {
			first = id
		}
		last = id
	}
	// The next preparation prunes; the bound holds after every insertion.
	next, err := c.prepare(f, context.Background(), 1, []ipcclient.Point{{Position: []float64{0}}})
	if err != nil {
		t.Fatal(err)
	}

	c.mu.Lock()
	view := c.handleViewLocked()
	c.mu.Unlock()
	if len(view) != retainedTerminalHandles+3 {
		t.Fatalf("history %d records, want %d terminal plus 3 live", len(view), retainedTerminalHandles)
	}
	kept := map[uint64]HandleState{}
	for _, record := range view {
		kept[record.Handle] = record.State
	}
	if kept[prepared] != HandlePrepared || kept[started] != HandleStarted || kept[next] != HandlePrepared {
		t.Fatalf("live handle dropped: prepared=%q started=%q next=%q", kept[prepared], kept[started], kept[next])
	}
	if _, ok := kept[first]; ok {
		t.Fatal("oldest terminal handle retained past the bound")
	}
	// The newest terminal handles are the ones kept, contiguous up to the last.
	for id := last - retainedTerminalHandles + 1; id <= last; id++ {
		if state, ok := kept[id]; !ok || state == HandlePrepared || state == HandleStarted {
			t.Fatalf("recent terminal handle %d: %q, %v", id, state, ok)
		}
	}

	// A forgotten handle can never start again: it answers unknown_handle,
	// which, like the terminal reasons it replaces, means upload anew.
	if _, err := c.startHandle(context.Background(), f, first); err == nil || err.Error() != ReasonUnknownHandle {
		t.Fatalf("start of a forgotten handle: %v", err)
	}
	if _, err := c.startHandle(context.Background(), f, last); err == nil || err.Error() != ReasonHandleSuperseded {
		t.Fatalf("start of a retained terminal handle lost its reason: %v", err)
	}
}

func TestFullStatusStaysWellUnderTheResponseCap(t *testing.T) {
	c, f := retentionFixture()
	for i := 0; i < 4*retainedTerminalHandles; i++ {
		prepareTerminal(t, c, f, HandleConsumed)
	}
	// Pruning runs at each preparation, so the handle that turned terminal
	// after the last one is counted at the next.
	if _, err := c.prepare(f, context.Background(), 1, []ipcclient.Point{{Position: []float64{0}}}); err != nil {
		t.Fatal(err)
	}
	rec := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(rec, httptest.NewRequest("GET", "/v1/status", nil))
	if rec.Code != 200 {
		t.Fatalf("status %d: %s", rec.Code, rec.Body.String())
	}
	var response struct {
		Data ProcessStatus `json:"data"`
	}
	if err := json.Unmarshal(rec.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	if n := len(response.Data.Execution.Handles); n != retainedTerminalHandles+1 {
		t.Fatalf("full status carries %d handles, want %d terminal plus 1 prepared", n, retainedTerminalHandles)
	}
	// The SDK refuses a response above 8 MiB (sdk/control/client.go). The
	// bounded history must leave that cap, and the reset freshness budget it
	// competes with, a wide margin: under 1 MiB here.
	t.Logf("full status: %d bytes, %d handles", rec.Body.Len(), len(response.Data.Execution.Handles))
	if size := rec.Body.Len(); size > 1<<20 {
		t.Fatalf("full status %d bytes with %d retained handles", size, retainedTerminalHandles)
	}
}
