//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"reflect"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

func statusObservationFixture(t *testing.T) Status {
	t.Helper()
	// Every top-level current status field has evidence, including references,
	// fault details, execution identity, I/O, feedback and large integer clocks.
	const raw = `{
		"daemon_incarnation":"daemon-1","adapter_incarnation":"adapter-1",
		"time_ns":9007199254740993,
		"grant":{"NowHostNs":9007199254740995,"HomeEpoch":9007199254740997,"GrantActive":1,"AxisMask":63},
		"jog":{"Open":1,"SourceSequence":9007199254740999},
		"jog_ingress":{"source_sequence":9007199254741001,"reason":7,"now_host_ns":9007199254741003,"refused":3},
		"core":{"time_ns":9007199254741005,"configuration_epoch":9007199254741007,"home_epoch":9007199254741009,
			"configuration_sha256":"configuration","daemon_incarnation":"native-1","backend":"ethercat","home_valid_mask":63,
			"config_verified_mask":63,"control_generation":9007199254741011,"lease_valid":1,"safety_fault_mask":16,"execution_fault_reasons":128,
			"bus":{"readiness":"operational","cycles":9007199254741013},"affinity_applied":{"cyclic":true},
			"recovery":{"home_valid_mask":63,"reset_sequence":9007199254741015,"faults":[{"bit":7,"name":"drive_readiness","axis_mask":16,"recovery":"reset_after_condition_clears","reset_preserves_home":true}]},
			"io":{"time_ns":9007199254741017,"cycle":9007199254741019,"inputs":[{"name":"sensor","value":true,"valid":true}],"outputs":[{"name":"lamp","commanded":true,"readback":true,"valid":true}]},
			"axes":[{"position_counts":1234,"home_valid":true,"home_offset":5678,"coordinate_identity_sha256":"axis-reference"}]},
		"motion":{"mode":2,"state":3,"trajectory_id":9007199254741021,"point_index":42,"queue_depth":12,"done":true,"command_sequence":9007199254741023,"time_ns":9007199254741025},
		"execution":{"generation":9007199254741027,"state":"faulted","identity":{"configuration_sha256":"configuration"},"segment":4,"native_sequence":9007199254741029,"error":"drive_readiness"},
		"axes":[{"time_ns":9007199254741031,"logical_position":1.25,"logical_valid":true,"logical_target":1.5,"logical_velocity":0.5,"readiness":"faulted","position_counts":1234,"home_valid":true,"torque_raw":23,"following_error_counts":45,"following_error_valid":true,"drive_alarm":"Er41.1"}],
		"generations":{"time_ns":9007199254741033,"native":9007199254741035,"configuration_epoch":9007199254741037,"home_epoch":9007199254741039,"execution":9007199254741041,"grant":9007199254741043},
		"plan_cursor":{"time_ns":9007199254741045,"active_handle":9007199254741047,"sample_index":7,"native_clock_ns":9007199254741049},
		"buffer_health":{"time_ns":9007199254741051,"slots_free":3,"staging_in_progress":true,"ring_drops":9007199254741053,"native_slot_occupancy":{"program":2}},
		"adapter":{"heap_inuse":9007199254741055,"sys":9007199254741057,"goroutines":9,"open_preparation_slots":2}
	}`
	var s Status
	if err := json.Unmarshal([]byte(raw), &s); err != nil {
		t.Fatal(err)
	}
	for i := uint64(1); i <= 4000; i++ {
		s.Execution.Handles = append(s.Execution.Handles, HandleRecord{Handle: 9007199254740993 + i, State: "retired", NativeSequence: i, Identity: s.Execution.Identity})
	}
	return s
}

func statusObservationServer(t *testing.T, handler http.HandlerFunc) *Client {
	t.Helper()
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodGet || r.URL.Path != "/v1/status" {
			t.Errorf("observation issued unexpected request: %s %s", r.Method, r.URL.Path)
			w.WriteHeader(http.StatusBadRequest)
			return
		}
		handler(w, r)
	}))
	t.Cleanup(server.Close)
	c := newClient(server.URL, &http.Transport{})
	t.Cleanup(func() { c.Close() })
	return c
}

func TestStatusObservationPreservesCurrentFieldsWithoutHandleHistory(t *testing.T) {
	want := statusObservationFixture(t)
	data, err := json.Marshal(want)
	if err != nil {
		t.Fatal(err)
	}
	c := statusObservationServer(t, func(w http.ResponseWriter, _ *http.Request) {
		json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: data})
	})
	got, err := c.StatusObservation(context.Background())
	if err != nil {
		t.Fatal(err)
	}
	if got.Execution.Handles != nil {
		t.Fatal("observation decoded historical handles")
	}
	got.Execution.Handles = want.Execution.Handles
	if !reflect.DeepEqual(got, want) {
		t.Fatal("observation changed a current Status field")
	}
	// These integers exceed float64's exact range; neither envelope nor
	// projection may round timestamps, epochs, identities or sequence numbers.
	if got.TimeNS != 9007199254740993 || got.Grant.NowHostNs != 9007199254740995 || got.Core.HomeEpoch != 9007199254741009 || got.Motion.CommandSequence != 9007199254741023 || got.Execution.NativeSequence != 9007199254741029 {
		t.Fatal("observation rounded native integer evidence")
	}
	full, err := c.Status(context.Background())
	if err != nil || !reflect.DeepEqual(full, want) || len(full.Execution.Handles) != 4000 {
		t.Fatalf("existing full Status changed: handles=%d err=%v", len(full.Execution.Handles), err)
	}
}

func TestStatusObservationMissingNativeEvidenceStaysMissing(t *testing.T) {
	for _, data := range []string{`{}`, `null`, `{"grant":null,"core":{"recovery":null}}`} {
		t.Run(data, func(t *testing.T) {
			c := statusObservationServer(t, func(w http.ResponseWriter, _ *http.Request) {
				json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: json.RawMessage(data)})
			})
			s, err := c.StatusObservation(context.Background())
			if err != nil || !reflect.DeepEqual(s, Status{}) {
				t.Fatalf("missing native proof was synthesized: %+v %v", s, err)
			}
		})
	}
}

func TestStatusObservationPreservesTypedRefusalData(t *testing.T) {
	const data = `{"reset_sequence":9007199254740993,"reason":"fault_persists","extra":[1,2,3]}`
	c := statusObservationServer(t, func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusConflict)
		io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","error":"fault_persists","native_result":{"Sequence":9007199254740995},"native_jog_result":{"Open":1},"data":`+data+`}`)
	})
	_, err := c.StatusObservation(context.Background())
	var rejected *Rejected
	if !errors.As(err, &rejected) || rejected.Reason != "fault_persists" || string(rejected.ResponseData) != data || rejected.NativeResult == nil || rejected.NativeJogResult == nil || rejected.NativeJogResult.Open != 1 {
		t.Fatalf("typed refusal lost evidence: %#v %v", rejected, err)
	}
}

func TestStatusObservationRejectsInvalidResponses(t *testing.T) {
	for _, tc := range []struct {
		name string
		code int
		body string
	}{
		{"http_error", http.StatusServiceUnavailable, `{"schema":"rosie.rt-control.response.v1","data":{}}`},
		{"http_error_malformed", http.StatusConflict, `{`},
		{"malformed", http.StatusOK, `{`},
		{"schema", http.StatusOK, `{"schema":"other","data":{}}`},
		{"typed_error", http.StatusOK, `{"schema":"rosie.rt-control.response.v1","error":"failed","data":{}}`},
		{"bad_current_data", http.StatusOK, `{"schema":"rosie.rt-control.response.v1","data":{"time_ns":"wrong"}}`},
		{"oversized", http.StatusOK, strings.Repeat(" ", (8<<20)+1)},
	} {
		t.Run(tc.name, func(t *testing.T) {
			c := statusObservationServer(t, func(w http.ResponseWriter, _ *http.Request) {
				w.WriteHeader(tc.code)
				io.WriteString(w, tc.body)
			})
			_, err := c.StatusObservation(context.Background())
			if !errors.Is(err, ErrProtocol) {
				t.Fatalf("invalid response accepted: %v", err)
			}
		})
	}
}

func TestStatusObservationClosedAndAlreadyCanceledSendNothing(t *testing.T) {
	for _, closed := range []bool{false, true} {
		t.Run(map[bool]string{false: "canceled", true: "closed"}[closed], func(t *testing.T) {
			var calls atomic.Int32
			c := statusObservationServer(t, func(w http.ResponseWriter, _ *http.Request) {
				calls.Add(1)
				io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","data":{}}`)
			})
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			want := context.Canceled
			if closed {
				c.Close()
				want = ErrClosed
			} else {
				cancel()
			}
			_, err := c.StatusObservation(ctx)
			if !errors.Is(err, want) || calls.Load() != 0 {
				t.Fatalf("closed/canceled read sent a request: calls=%d err=%v", calls.Load(), err)
			}
		})
	}
}

func TestStatusObservationCancellationDuringBodyReadReleasesLane(t *testing.T) {
	entered := make(chan struct{})
	var calls atomic.Int32
	c := statusObservationServer(t, func(w http.ResponseWriter, r *http.Request) {
		if calls.Add(1) == 1 {
			io.WriteString(w, `{`)
			w.(http.Flusher).Flush()
			close(entered)
			<-r.Context().Done()
			return
		}
		io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","data":{}}`)
	})
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	done := make(chan error, 1)
	go func() { _, err := c.StatusObservation(ctx); done <- err }()
	select {
	case <-entered:
	case <-time.After(2 * time.Second):
		t.Fatal("status read did not enter response body")
	}
	cancel()
	select {
	case err := <-done:
		if !errors.Is(err, context.Canceled) {
			t.Fatalf("canceled body read lost cancellation: %v", err)
		}
	case <-time.After(2 * time.Second):
		t.Fatal("status body read ignored cancellation")
	}
	// Four subsequent calls cycle through every reserved control lane. A
	// canceled response must neither leak its lane nor poison its next read.
	ctx, cancel = context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	for i := 0; i < 4; i++ {
		if _, err := c.StatusObservation(ctx); err != nil {
			t.Fatalf("status reservation was not reusable: %v", err)
		}
	}
}

func TestStatusObservationRequestsControlViewWithUnchangedCurrentFields(t *testing.T) {
	want := statusObservationFixture(t)
	want.Execution.Handles = nil
	c := statusObservationServer(t, func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Query().Get("view") != "control" {
			t.Error("current observation did not request bounded view")
		}
		current := want
		current.Execution.Handles = []HandleRecord{}
		raw, _ := json.Marshal(current)
		json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: raw})
	})
	got, err := c.StatusObservation(context.Background())
	if err != nil || !reflect.DeepEqual(got, want) {
		t.Fatalf("current view changed evidence: %v", err)
	}
}
