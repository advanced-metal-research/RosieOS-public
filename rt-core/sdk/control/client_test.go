//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"sync/atomic"
	"testing"
	"time"
)

func TestSDKTransportRetryRetainsExactMutation(t *testing.T) {
	requests := make(chan []byte, 2)
	var calls atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		body, err := io.ReadAll(r.Body)
		if err != nil {
			t.Error(err)
		}
		requests <- body
		if calls.Add(1) == 1 {
			conn, _, err := w.(http.Hijacker).Hijack()
			if err != nil {
				t.Error(err)
				return
			}
			conn.Close()
			return
		}
		_ = json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: "enable", Sequence: 19})
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	client.SetFence(Fence{Session: "owner", Generation: 7})
	reply, err := client.Enable(context.Background(), 3)
	must(t, err)
	if reply.Sequence != 19 || calls.Load() != 2 {
		t.Fatalf("retry: %+v calls=%d", reply, calls.Load())
	}
	first, second := <-requests, <-requests
	if string(first) != string(second) {
		t.Fatal("retry changed wire request")
	}
	var request Request
	must(t, json.Unmarshal(first, &request))
	if request.RequestID == nil || len(*request.RequestID) != 32 || request.Session != "owner" || request.Generation != 7 {
		t.Fatalf("lost id/fence: %+v", request)
	}
}

func TestSDKRestoreAnchorDecodesRecovery(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		var request Request
		if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
			t.Error(err)
			return
		}
		if r.URL.Path != "/v1/control" || request.Operation != "restore_anchor" || request.AxisMask != 3 || request.Session != "owner" || request.Generation != 7 {
			t.Errorf("restore request: %+v", request)
		}
		_, _ = io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","operation":"restore_anchor","sequence":9007199254740995,"data":{"home_valid_mask":3,"faults":[],"reset_sequence":9007199254740993,"reason":"","outcomes":[{"bit":8,"name":"bus_transport","axis_mask":3,"recovery":"reset_after_condition_clears","outcome":"cleared","rehome_axis_mask":0}]}}`)
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	client.SetFence(Fence{Session: "owner", Generation: 7})
	recovery, err := client.RestoreAnchor(context.Background(), 3)
	must(t, err)
	if recovery.HomeValidMask != 3 || recovery.ResetSequence != 9007199254740993 || recovery.Reason != "" || recovery.Faults == nil || len(recovery.Faults) != 0 || len(recovery.Outcomes) != 1 {
		t.Fatalf("restore recovery: %+v", recovery)
	}
	outcome := recovery.Outcomes[0]
	if outcome.Bit != 8 || outcome.Name != "bus_transport" || outcome.AxisMask != 3 || outcome.Recovery != "reset_after_condition_clears" || outcome.Outcome != "cleared" || outcome.RehomeAxisMask != 0 {
		t.Fatalf("restore outcome: %+v", outcome)
	}
}

func TestSDKRestoreAnchorRejectsSequenceOnlyReply(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","operation":"restore_anchor","sequence":19}`)
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	if _, err := client.RestoreAnchor(context.Background(), 1); !errors.Is(err, ErrProtocol) {
		t.Fatalf("sequence-only restore accepted: %v", err)
	}
}

func TestSDKDescribePreservesTestOnlyCapability(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_, _ = io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","operation":"describe","data":{"capabilities":[{"name":"jog","state":"test_only","transport":"POST /v1/control"}]}}`)
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	description, err := client.Describe(context.Background())
	must(t, err)
	if len(description.Capabilities) != 1 || description.Capabilities[0] != (CapabilityInfo{Name: "jog", State: CapabilityStateTestOnly, Transport: "POST /v1/control"}) {
		t.Fatalf("legacy capability: %+v", description.Capabilities)
	}
}

func TestSDKBusyIsTypedAndNeverRetried(t *testing.T) {
	var calls atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		calls.Add(1)
		w.WriteHeader(http.StatusConflict)
		_, _ = io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","operation":"prepare_trajectory","error":"busy","native_result":{"reason":7}}`)
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	_, err := client.PrepareTrajectory(context.Background(), 1, []Point{{Position: []float64{0}}, {Time: 1_000_000, Position: []float64{0}}})
	rejected(t, err, "busy")
	var rejection *Rejected
	if !errors.As(err, &rejection) || rejection.NativeResult == nil || rejection.NativeResult.Reason != 7 || calls.Load() != 1 {
		t.Fatalf("rejection=%+v calls=%d", rejection, calls.Load())
	}
}

func TestSDKUncachedMutationsNeverRetryLostReplies(t *testing.T) {
	for _, operation := range []string{"release", "prepare_program"} {
		t.Run(operation, func(t *testing.T) {
			var calls atomic.Int32
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				_, _ = io.Copy(io.Discard, r.Body)
				calls.Add(1)
				conn, _, err := w.(http.Hijacker).Hijack()
				if err != nil {
					t.Error(err)
					return
				}
				conn.Close()
			}))
			defer server.Close()
			client := newClient(server.URL, &http.Transport{})
			defer client.Close()
			var err error
			if operation == "release" {
				_, err = client.Release(context.Background())
			} else {
				_, err = client.PrepareProgram(context.Background(), []byte("program"))
			}
			if !errors.Is(err, ErrTransport) || calls.Load() != 1 {
				t.Fatalf("uncertain %s: %v calls=%d", operation, err, calls.Load())
			}
		})
	}
}

func TestSDKLateRenewalCannotRestoreStoppingAfterStop(t *testing.T) {
	entered, release := make(chan struct{}), make(chan struct{}, 1)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		var request Request
		if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
			t.Error(err)
			return
		}
		grant := Grant{Session: "owner", Generation: 2, DeadlineHostNS: 123_000_000}
		if request.Operation == "renew" {
			grant.Stopping = true
			close(entered)
			<-release
		}
		body, err := json.Marshal(grant)
		if err != nil {
			t.Error(err)
			return
		}
		_ = json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: request.Operation, Data: body})
	}))
	defer server.Close()
	defer close(release)
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	client.SetFence(Fence{Session: "owner", Generation: 1})
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	renewed := make(chan error, 1)
	go func() { _, err := client.Renew(ctx); renewed <- err }()
	select {
	case <-entered:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	_, err := client.Stop(ctx)
	release <- struct{}{}
	must(t, err)
	must(t, <-renewed)
	if client.Grant().Stopping || client.Fence().Generation != 2 {
		t.Fatalf("late renewal restored stale state: %+v", client.Grant())
	}
}
