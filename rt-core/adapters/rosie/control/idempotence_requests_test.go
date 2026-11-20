//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"strings"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

type countedHandleNative struct {
	nativeClient
	starts         atomic.Uint64
	accepted       atomic.Uint64
	renews         atomic.Uint64
	startCommitted chan uint64
	startReturn    chan struct{}
}

func (n *countedHandleNative) StartTrajectory(ctx context.Context, handle uint64) (uint64, error) {
	n.starts.Add(1)
	seq, err := n.nativeClient.StartTrajectory(ctx, handle)
	if err == nil {
		n.accepted.Add(1)
		if n.startCommitted != nil {
			n.startCommitted <- seq
			<-n.startReturn
		}
	}
	return seq, err
}

func (n *countedHandleNative) MirrorRenew(ctx context.Context, identity ipcclient.GrantIdentity, deadline uint64) (ipcclient.GrantReceipt, error) {
	n.renews.Add(1)
	return n.nativeClient.MirrorRenew(ctx, identity, deadline)
}

type capturedHandleReply struct {
	http.ResponseWriter
	body bytes.Buffer
}

func (w *capturedHandleReply) Write(body []byte) (int, error) {
	_, _ = w.body.Write(body)
	return w.ResponseWriter.Write(body)
}

func TestLostStartReplyDoesNotSubmitMotionTwice(t *testing.T) {
	f := newHandleFixture(t)
	native := &countedHandleNative{nativeClient: f.client, startCommitted: make(chan uint64, 1), startReturn: make(chan struct{})}
	replaceNativeTestClient(f.controller, native)
	var releaseOnce sync.Once
	release := func() { releaseOnce.Do(func() { close(native.startReturn) }) }
	t.Cleanup(release)
	original := make(chan []byte, 1)
	arrivals := make(chan struct{}, 2)
	id := "lost-start"
	a := serveHandleAPI(t, f.controller, func(next http.Handler) http.Handler {
		return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
			capture := &capturedHandleReply{ResponseWriter: w}
			if r.Header.Get("X-Test-Start") != "" {
				arrivals <- struct{}{}
			}
			next.ServeHTTP(capture, r)
			var response Response
			if json.Unmarshal(capture.body.Bytes(), &response) == nil && response.Operation == "start_trajectory" && r.Header.Get("X-Test-Start") == "original" {
				select {
				case original <- append([]byte(nil), capture.body.Bytes()...):
				default:
				}
			}
		})
	})
	keepHandleGrant(t, a, f.fence)
	enableHandles(t, a, f.fence)
	request := controlRequest(f.fence, "prepare_trajectory")
	request.AxisMask, request.Points = 511, handlePoints(300*time.Millisecond)
	request.Points[1].Position[0] = .002
	handle := a.call(t, request, "").Handle
	before, _, execution := a.snapshot(t)
	if before.Axes[0].PositionCounts != 0 || execution.Generation != 0 {
		t.Fatalf("initial execution: %+v %+v", before, execution)
	}
	request = controlRequest(f.fence, "start_trajectory")
	request.Handle, request.RequestID = handle, &id
	body, err := json.Marshal(request)
	if err != nil {
		t.Fatal(err)
	}
	connection, err := net.Dial("unix", a.socket)
	if err != nil {
		t.Fatal(err)
	}
	// Write the complete request and close without reading its reply. The
	// native return barrier keeps the original pending when its retry arrives.
	_, err = fmt.Fprintf(connection, "POST /v1/control HTTP/1.1\r\nHost: unix\r\nContent-Length: %d\r\nX-Test-Start: original\r\n\r\n%s", len(body), body)
	_ = connection.Close()
	if err != nil {
		t.Fatal(err)
	}
	var sequence uint64
	select {
	case sequence = <-native.startCommitted:
	case <-time.After(3 * time.Second):
		t.Fatal("lost request did not reach native admission")
	}
	<-arrivals
	type retryResult struct {
		response Response
		body     []byte
		err      error
	}
	retried := make(chan retryResult, 1)
	go func() {
		r, _ := http.NewRequest(http.MethodPost, "http://unix/v1/control", bytes.NewReader(body))
		r.Header.Set("X-Test-Start", "retry")
		response, err := a.client.Do(r)
		if err != nil {
			retried <- retryResult{err: err}
			return
		}
		defer response.Body.Close()
		var raw bytes.Buffer
		_, err = raw.ReadFrom(response.Body)
		var value Response
		if err == nil {
			err = json.Unmarshal(raw.Bytes(), &value)
		}
		retried <- retryResult{value, raw.Bytes(), err}
	}()
	select {
	case <-arrivals:
	case <-time.After(3 * time.Second):
		t.Fatal("retry never reached handler")
	}
	release()
	var retry retryResult
	select {
	case retry = <-retried:
	case <-time.After(3 * time.Second):
		t.Fatal("retry did not join original outcome")
	}
	var saved []byte
	select {
	case saved = <-original:
	case <-time.After(3 * time.Second):
		t.Fatal("original response missing")
	}
	if retry.err != nil || retry.response.Error != "" || !bytes.Equal(saved, retry.body) || retry.response.Sequence != sequence {
		t.Fatalf("retry=%s original=%s err=%v", retry.body, saved, retry.err)
	}
	last, moved := int32(0), false
	awaitHandleCondition(t, func() bool { _, motion, _ := a.snapshot(t); return motion.CommandSequence == sequence })
	awaitHandleCondition(t, func() bool {
		state, motion, execution := a.snapshot(t)
		position := state.Axes[0].PositionCounts
		if position < last || position > 42 {
			t.Fatalf("feedback replay/overshoot: %d -> %d counts", last, position)
		}
		moved = moved || position > 0
		last = position
		if motion.CommandSequence != sequence || execution.Generation != 1 {
			t.Fatalf("execution advanced more than once: %+v %+v", motion, execution)
		}
		return motion.Done && position == 42
	})
	if !moved || native.starts.Load() != 1 || native.accepted.Load() != 1 {
		t.Fatalf("motion submissions=%d admissions=%d moved=%v", native.starts.Load(), native.accepted.Load(), moved)
	}
	_, secondReplay, err := a.do(context.Background(), request)
	if err != nil || !bytes.Equal(secondReplay, saved) || native.starts.Load() != 1 {
		t.Fatalf("completed retry replayed execution: %s %v", secondReplay, err)
	}
	requireHandle(t, a, handle, HandleConsumed)
}

func TestRequestIdConflictAndEviction(t *testing.T) {
	f := newHandleFixture(t)
	native := &countedHandleNative{nativeClient: f.client}
	replaceNativeTestClient(f.controller, native)
	a := serveHandleAPI(t, f.controller, nil)
	id := "first"
	request := controlRequest(f.fence, "renew")
	request.RequestID = &id
	first, raw, err := a.do(context.Background(), request)
	if err != nil || first.Error != "" {
		t.Fatalf("first: %+v %v", first, err)
	}
	_, replay, err := a.do(context.Background(), request)
	if err != nil || !bytes.Equal(raw, replay) || native.renews.Load() != 1 {
		t.Fatalf("renew was re-executed: %s %v calls=%d", replay, err, native.renews.Load())
	}
	conflict := request
	conflict.AxisMask = 1
	a.call(t, conflict, "request_id_conflict")
	for i := 0; i < 256; i++ {
		id := fmt.Sprintf("evict-%d", i)
		next := request
		next.RequestID = &id
		a.call(t, next, "")
	}
	again, _, err := a.do(context.Background(), request)
	if err != nil || again.Error != "" || native.renews.Load() != 258 {
		t.Fatalf("oldest did not re-execute: %+v %v calls=%d", again, err, native.renews.Load())
	}
	if grantFrom(t, again).DeadlineHostNS <= grantFrom(t, first).DeadlineHostNS {
		t.Fatal("evicted renewal did not advance its absolute deadline")
	}
	f.controller.mu.Lock()
	size := len(f.controller.requests.entries)
	f.controller.mu.Unlock()
	if size != 256 {
		t.Fatalf("table contains %d entries", size)
	}
	for _, id := range []string{"", strings.Repeat("x", 65), "newline\n", "del\x7f", "nonascii-µ"} {
		invalid := request
		invalid.RequestID = &id
		a.call(t, invalid, "invalid_request_id")
	}
	for _, rawID := range []string{"null", "7", "true", "[]", "{}"} {
		body := fmt.Sprintf(`{"schema":"rosie.rt-control.request.v1","operation":"renew","session":%q,"generation":%d,"request_id":%s}`, f.fence.Session, f.fence.Generation, rawID)
		response, err := a.client.Post("http://unix/v1/control", "application/json", strings.NewReader(body))
		if err != nil {
			t.Fatal(err)
		}
		var result Response
		err = json.NewDecoder(response.Body).Decode(&result)
		_ = response.Body.Close()
		if err != nil || result.Error != "invalid_request_id" {
			t.Fatalf("non-string ID: %+v %v", result, err)
		}
	}
	a.call(t, controlRequest(f.fence, "release"), "")
	f.controller.mu.Lock()
	size = len(f.controller.requests.entries)
	f.controller.mu.Unlock()
	if size != 0 {
		t.Fatalf("release retained %d IDs", size)
	}
	acquire := controlRequest(Fence{}, "acquire")
	acquire.Binding, acquire.Controller, acquire.RequestID = f.controller.binding, "acquire-retry", &id
	one, raw, err := a.do(context.Background(), acquire)
	if err != nil || one.Error != "" {
		t.Fatalf("acquire: %+v %v", one, err)
	}
	_, replay, err = a.do(context.Background(), acquire)
	if err != nil || !bytes.Equal(raw, replay) {
		t.Fatalf("lost Acquire reply changed session: %s %s %v", raw, replay, err)
	}
}
