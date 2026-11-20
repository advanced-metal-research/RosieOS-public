//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Accepted marks publish one immutable receipt before the HTTP reply; retries
// must not duplicate the event or disclose the session capability.
func TestTelemetryMarkEventPublishedOnce(t *testing.T) {
	f := startLifecycleCore(t)
	a := serveHandleAPI(t, f.controller, nil)
	before := publicEvents(t, a.client, 0)
	id := "mark-once"
	request := controlRequest(f.fence, OperationMarkTelemetry)
	request.RequestID, request.Label = &id, "scope / Δ \"test\""
	start := ipcclient.HostMonotonicNS()
	accepted := a.call(t, request, "")
	end := ipcclient.HostMonotonicNS()
	if accepted.Sequence == 0 {
		t.Fatal("accepted mark has no native sequence")
	}
	retry := a.call(t, request, "")
	if retry.Sequence != accepted.Sequence {
		t.Fatalf("retry sequence=%d want=%d", retry.Sequence, accepted.Sequence)
	}
	batch := publicEvents(t, a.client, before.LatestSequence)
	marks := 0
	for _, event := range batch.Events {
		if event.Type != "telemetry_mark" {
			continue
		}
		marks++
		if event.Mark == nil || event.Mark.Label != request.Label || event.Mark.NativeSequence != accepted.Sequence || event.Mark.Generation != f.fence.Generation {
			t.Fatalf("mark receipt does not match acceptance: %+v", event)
		}
		if event.Sequence <= before.LatestSequence || event.TimeNS < start || event.TimeNS > end {
			t.Fatalf("mark cursor/time outside request: %+v bounds=%d..%d", event, start, end)
		}
		if event.DaemonIncarnation != f.controller.daemonIncarnation || event.AdapterIncarnation != f.controller.adapterIncarnation {
			t.Fatalf("mark identity mismatch: %+v", event)
		}
		wire, err := json.Marshal(event)
		if err != nil || bytes.Contains(wire, []byte(f.fence.Session)) || bytes.Contains(wire, []byte(`"session"`)) {
			t.Fatalf("mark disclosed session capability: %s %v", wire, err)
		}
	}
	if marks != 1 {
		t.Fatalf("published marks=%d want=1", marks)
	}
	for _, event := range publicEvents(t, a.client, batch.NextSequence).Events {
		if event.Type == "telemetry_mark" {
			t.Fatalf("mark repeated after resume: %+v", event)
		}
	}
}

// Refused marks never enter event history, including native admission errors
// that carry a submitted sequence but no accepted receipt.
func TestTelemetryMarkRefusalsPublishNoEvent(t *testing.T) {
	f := startLifecycleCore(t)
	a := serveHandleAPI(t, f.controller, nil)
	before := publicEvents(t, a.client, 0)
	for _, label := range []string{"", strings.Repeat("a", 129), strings.Repeat("λ", 65)} {
		request := controlRequest(f.fence, OperationMarkTelemetry)
		request.Label = label
		a.call(t, request, ReasonTelemetryLabelInvalid)
	}
	for _, fence := range []Fence{{Session: "stale", Generation: f.fence.Generation}, {Session: f.fence.Session, Generation: f.fence.Generation + 1}} {
		request := controlRequest(fence, OperationMarkTelemetry)
		request.Label = "refused"
		a.call(t, request, ReasonControlSessionStale)
	}
	for _, event := range publicEvents(t, a.client, before.LatestSequence).Events {
		if event.Type == "telemetry_mark" || event.Mark != nil {
			t.Fatalf("refused mark published: %+v", event)
		}
	}
}

type telemetryMarkNative struct {
	nativeClient
	mark func(context.Context, string) (uint64, error)
}

func (n telemetryMarkNative) MarkTelemetry(ctx context.Context, label string) (uint64, error) {
	return n.mark(ctx, label)
}

// Admission completion retains the authorizing identity across replacement;
// native refusal must not be mistaken for success merely because it has a sequence.
func TestTelemetryMarkEventNativeReceipt(t *testing.T) {
	for _, refused := range []bool{false, true} {
		t.Run(map[bool]string{false: "accepted", true: "refused"}[refused], func(t *testing.T) {
			c := &Controller{
				grant: Grant{Session: "private-session", Generation: 7}, expires: time.Now().Add(time.Minute),
				sessionCtx: context.Background(), daemonIncarnation: "original-daemon", adapterIncarnation: "original-adapter",
			}
			nativeErr := errors.New("native mark refused")
			c.core = telemetryMarkNative{mark: func(context.Context, string) (uint64, error) {
				c.mu.Lock()
				c.daemonIncarnation = "replacement-daemon"
				c.grant.Generation = 8
				c.mu.Unlock()
				if refused {
					return 42, nativeErr
				}
				return 42, nil
			}}
			start := ipcclient.HostMonotonicNS()
			sequence, err := c.MarkTelemetry(Fence{Session: "private-session", Generation: 7}, "camera")
			end := ipcclient.HostMonotonicNS()
			if sequence != 42 || refused && !errors.Is(err, nativeErr) || !refused && err != nil {
				t.Fatalf("native result changed: sequence=%d err=%v", sequence, err)
			}
			batch, _, err := c.events.batch(0)
			if err != nil {
				t.Fatal(err)
			}
			if refused {
				if len(batch.Events) != 0 {
					t.Fatalf("native refusal published %d events", len(batch.Events))
				}
				return
			}
			if len(batch.Events) != 1 {
				t.Fatalf("accepted native mark published %d events want=1", len(batch.Events))
			}
			event := batch.Events[0]
			if event.Sequence != 1 || event.Type != "telemetry_mark" || event.DaemonIncarnation != "original-daemon" || event.AdapterIncarnation != "original-adapter" || event.TimeNS < start || event.TimeNS > end || event.Mark == nil || *event.Mark != (TelemetryMarkEvent{Label: "camera", NativeSequence: 42, Generation: 7}) {
				t.Fatalf("incorrect native mark receipt: %+v", event)
			}
		})
	}
}

// Marks need a current session and a bounded envelope, but observation must
// remain available while a motion operation owns the commissioning lock.
func TestTelemetryHTTPAdmissionAndObservationLane(t *testing.T) {
	f := startLifecycleCore(t)
	capability, ok := Capability(OperationMarkTelemetry)
	if !ok || capability.State != "implemented" {
		t.Fatal("missing mark capability")
	}
	handler := NewHandler(f.controller)
	request := Request{Schema: RequestSchema, Operation: OperationMarkTelemetry, Fence: f.fence, Label: "scope / Δ \"test\""}
	call := func(request Request) *httptest.ResponseRecorder {
		raw, _ := json.Marshal(request)
		response := httptest.NewRecorder()
		handler.ServeHTTP(response, httptest.NewRequest(http.MethodPost, "/v1/control", bytes.NewReader(raw)))
		return response
	}
	stale := request
	stale.Session = ""
	if response := call(stale); response.Code == http.StatusOK {
		t.Fatalf("sessionless mark admitted: %s", response.Body)
	}
	f.controller.operationMu.Lock()
	responses := make(chan *httptest.ResponseRecorder, 1)
	go func() { responses <- call(request) }()
	var response *httptest.ResponseRecorder
	select {
	case response = <-responses:
	case <-time.After(3 * time.Second):
		f.controller.operationMu.Unlock()
		t.Fatal("observation blocked behind operation lock")
	}
	f.controller.operationMu.Unlock()
	var result Response
	if err := json.Unmarshal(response.Body.Bytes(), &result); err != nil || response.Code != http.StatusOK || result.Sequence == 0 {
		t.Fatalf("mark rejected: %s %v", response.Body, err)
	}
	for _, label := range []string{"", strings.Repeat("a", 129), strings.Repeat("λ", 65)} {
		request.Label = label
		response := call(request)
		var result Response
		if err := json.Unmarshal(response.Body.Bytes(), &result); err != nil || response.Code == http.StatusOK || result.Error != ReasonTelemetryLabelInvalid {
			t.Fatalf("invalid label reason: %s (%v)", response.Body, err)
		}
	}
	if _, err := f.controller.MarkTelemetry(f.fence, string([]byte{0xff})); err == nil || err.Error() != ReasonTelemetryLabelInvalid {
		t.Fatalf("invalid UTF-8 admission: %v", err)
	}
}
