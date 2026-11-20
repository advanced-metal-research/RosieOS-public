//go:build linux

package publisher

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"net"
	"reflect"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	"golang.org/x/sys/unix"
	contract "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// sampleObserver supplies independent fresh observations and an explicit source
// outage, preventing diagnostics tests from depending on daemon scheduling.
type sampleObserver struct{ unavailable atomic.Bool }

func (o *sampleObserver) Describe(context.Context) (control.Description, error) {
	var d control.Description
	d.ContractVersion, d.CapabilitiesDigest = contract.ContractVersion, contract.CapabilitiesDigest
	d.ProtocolMajor, d.ProtocolMinor = ipcclient.VersionMajor, ipcclient.VersionMinor
	d.Backend, d.ConfigurationSHA256 = "simulation", "test-identity"
	d.Axes = []ipcclient.AxisDescription{{ID: "J1", PositionUnit: "rad", MinPosition: -1, MaxPosition: 1}}
	return d, nil
}

func (o *sampleObserver) Status(context.Context) (control.Status, error) {
	if o.unavailable.Load() {
		return control.Status{}, errors.New("observer unavailable")
	}
	var ts unix.Timespec
	if err := unix.ClockGettime(unix.CLOCK_MONOTONIC, &ts); err != nil {
		return control.Status{}, err
	}
	s := control.Status{}
	s.Core = ipcclient.Status{TimeNS: uint64(ts.Nano()), Backend: "simulation", ConfigurationSHA256: "test-identity",
		Axes: []ipcclient.AxisStatus{{TimeNS: uint64(ts.Nano()), PDOFresh: 1, PositionCounts: 1234}}}
	return s, nil
}

func TestPermanentBrokerFailuresReturnWithoutReconnect(t *testing.T) {
	for _, tc := range []struct {
		name      string
		configure func(*broker)
		want      error
	}{
		{"authentication", func(b *broker) { b.info = `{"auth_required":true,"jetstream":true}` }, ErrAuthRequired},
		{"TLS", func(b *broker) { b.info = `{"tls_required":true,"jetstream":true}` }, ErrTLSRequired},
		{"no-JetStream", func(b *broker) { b.info = `{}` }, ErrJetStreamUnavailable},
		{"no-responders", func(b *broker) { b.noResponders = true }, ErrJetStreamUnavailable},
		{"storage", func(b *broker) { c := desiredStream(); c.Storage = "memory"; b.config = &c }, ErrStreamConfig},
		{"retention", func(b *broker) { c := desiredStream(); c.Retention = "workqueue"; b.config = &c }, ErrStreamConfig},
		{"update-refused", func(b *broker) { c := desiredStream(); c.MaxMsgs = 1; b.config = &c; b.rejectUpdate = true }, ErrStreamConfig},
	} {
		t.Run(tc.name, func(t *testing.T) {
			b := newBroker(t, tc.configure)
			p, err := New(&sampleObserver{}, Config{URL: b.url(), Host: "other-relay"})
			must(t, err)
			var log bytes.Buffer
			p.log = &log
			ctx, cancel := context.WithTimeout(context.Background(), observation)
			defer cancel()
			if err := p.Run(ctx); !errors.Is(err, tc.want) {
				t.Fatalf("Run: got %v want %v", err, tc.want)
			}
			b.mu.Lock()
			defer b.mu.Unlock()
			if b.connections != 1 || b.publishes != 0 || b.updates != 0 || b.creates != 0 || p.diagnostics().Reconnects != 0 {
				t.Fatalf("permanent refusal retried or mutated stream: %+v", p.diagnostics())
			}
			if strings.Count(log.String(), "transition=refused ") != 1 || !strings.Contains(log.String(), "last_error=") {
				t.Fatalf("missing refusal: %s", log.String())
			}
		})
	}
}

func TestBrokerEnforcesImmutableUpdateFields(t *testing.T) {
	b := newBroker(t)
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	n, err := dialNATS(ctx, b.url())
	must(t, err)
	defer n.close()
	must(t, n.ensureStream())
	want := desiredStream()
	for _, field := range []string{"name", "storage", "retention"} {
		candidate := want
		switch field {
		case "name":
			candidate.Name = "OTHER"
		case "storage":
			candidate.Storage = "memory"
		case "retention":
			candidate.Retention = "workqueue"
		}
		raw, err := json.Marshal(candidate)
		must(t, err)
		response, err := n.api("$JS.API.STREAM.UPDATE.ROBOT_V4_RTCORE", raw)
		must(t, err)
		if response.Error == nil || response.Error.Code != 400 {
			t.Fatalf("fake accepted immutable %s", field)
		}
		assertStream(t, b)
	}
	// Mutable subject/limit/discard changes exercise an accepted real API path.
	want.Subjects, want.MaxMsgs, want.Discard = []string{"archive.>"}, 17, "new"
	raw, err := json.Marshal(want)
	must(t, err)
	response, err := n.api("$JS.API.STREAM.UPDATE.ROBOT_V4_RTCORE", raw)
	must(t, err)
	if response.Error != nil || !reflect.DeepEqual(response.Config, want) {
		t.Fatalf("mutable update refused: %+v", response)
	}
	must(t, n.ensureStream())
	assertStream(t, b)
}

func TestDiagnosticsSuppressRepeatedFailuresAndEscapeRecords(t *testing.T) {
	p, err := New(&sampleObserver{}, Config{URL: "nats://127.0.0.1:4222", Host: "other-relay"})
	must(t, err)
	var log bytes.Buffer
	p.log = &log
	p.setTransport("connecting", nil)
	for i := 0; i < 4; i++ {
		p.setTransport("dial_failed", errors.New("refused\nforged=record"))
		p.drop("pending sample replaced")
	}
	p.setTransport("publishing", nil)
	p.recovered(4)
	for i := 0; i < 3; i++ {
		p.setSource(errors.New("observer unavailable"))
	}
	d := p.diagnostics()
	if d.State != "source_unavailable" || d.LastError != "observer unavailable" || d.Drops != 4 || d.Reconnects != 0 {
		t.Fatalf("diagnostics: %+v", d)
	}
	p.setSource(nil)
	if p.diagnostics().State != "publishing" {
		t.Fatal("source recovery not visible")
	}
	for _, transition := range []string{"connecting", "dial_failed", "dropping", "publishing", "recovered", "source_unavailable", "source_ready"} {
		if strings.Count(log.String(), "transition="+transition+" ") != 1 {
			t.Fatalf("transition %s duplicated/missing: %s", transition, log.String())
		}
	}
	if len(strings.Split(strings.TrimSpace(log.String()), "\n")) != 7 || !strings.Contains(log.String(), `refused\nforged=record`) {
		t.Fatalf("journal framing: %s", log.String())
	}
}

func TestDialRetryLogsOnlyOneRefusalTransition(t *testing.T) {
	ln, err := net.Listen("tcp", "127.0.0.1:0")
	must(t, err)
	url := "nats://" + ln.Addr().String()
	must(t, ln.Close())
	p, err := New(&sampleObserver{}, Config{URL: url, Host: "other-relay"})
	must(t, err)
	var log bytes.Buffer
	p.log = &log
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- p.Run(ctx) }()
	t.Cleanup(func() { cancel(); must(t, <-done) })
	poll(t, func() bool { return p.diagnostics().Reconnects >= 2 })
	p.mu.Lock()
	defer p.mu.Unlock()
	if strings.Count(log.String(), "transition=dial_failed ") != 1 {
		t.Fatalf("per-attempt logging: %s", log.String())
	}
}

func TestSourceOutagePublishesDiagnosticsWithoutStaleStatus(t *testing.T) {
	o := &sampleObserver{}
	b := newBroker(t)
	p := startPublisher(t, o, b)
	poll(t, func() bool { return len(b.latest(subject("other-relay", "telemetry.batch"))) > 0 })
	o.unavailable.Store(true)
	poll(t, func() bool {
		var status map[string]any
		if json.Unmarshal(b.latest(subject("other-relay", "status")), &status) != nil {
			return false
		}
		return status["publisher_state"] == "source_unavailable" && status["last_error"] == "observer unavailable" && status["status"] == nil && statusNumber(status["drops"]) >= 1 && status["reconnects"] == float64(0)
	})
	o.unavailable.Store(false)
	poll(t, func() bool {
		var status map[string]any
		if json.Unmarshal(b.latest(subject("other-relay", "status")), &status) != nil {
			return false
		}
		return status["publisher_state"] == "publishing" && status["status"] != nil && status["last_error"] == "observer unavailable"
	})
	if p.diagnostics().Reconnects != 0 {
		t.Fatal("source outage reconnected the healthy broker")
	}
}

func TestDuplicatePublisherFailsBeforePublishing(t *testing.T) {
	b := newBroker(t)
	o := &sampleObserver{}
	first, err := New(o, Config{URL: b.url(), Host: "other-relay", Cadence: 20 * time.Millisecond})
	must(t, err)
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan error, 1)
	go func() { done <- first.Run(ctx) }()
	t.Cleanup(func() { cancel(); must(t, <-done) })
	poll(t, func() bool { return len(b.latest(subject("other-relay", "telemetry.batch"))) > 0 })
	second, err := New(o, Config{URL: b.url(), Host: "other-relay"})
	must(t, err)
	bounded, stop := context.WithTimeout(ctx, observation)
	defer stop()
	if err := second.Run(bounded); !errors.Is(err, ErrPublisherActive) {
		t.Fatalf("duplicate publisher: %v", err)
	}
	b.mu.Lock()
	secondPublications := b.publications[2]
	b.mu.Unlock()
	if secondPublications != 0 || second.diagnostics().Reconnects != 0 {
		t.Fatal("duplicate published or retried")
	}
	// A different concrete identity can publish despite the first peer's traffic.
	other, err := New(o, Config{URL: b.url(), Host: "bench-relay", Cadence: 20 * time.Millisecond})
	must(t, err)
	otherCtx, otherCancel := context.WithCancel(ctx)
	otherDone := make(chan error, 1)
	go func() { otherDone <- other.Run(otherCtx) }()
	t.Cleanup(func() { otherCancel(); must(t, <-otherDone) })
	poll(t, func() bool { return len(b.latest(subject("bench-relay", "telemetry.batch"))) > 0 })
}

func TestStartupCheckObservesFullConfiguredCadence(t *testing.T) {
	b := newBroker(t)
	ctx, cancel := context.WithTimeout(context.Background(), observation)
	defer cancel()
	n, err := dialNATS(ctx, b.url())
	must(t, err)
	defer n.close()
	start := time.Now()
	must(t, n.checkPublisher(subject("other-relay", "status"), 500*time.Millisecond))
	if elapsed := time.Since(start).Nanoseconds(); elapsed < 500000000 {
		t.Fatalf("startup guard ended after %d ns", elapsed)
	}
}

func TestCadencePayloadDiagnosticsAndBatchReaderFields(t *testing.T) {
	p, err := New(&sampleObserver{}, Config{URL: "nats://127.0.0.1:4222", Host: "other-relay", Cadence: 250 * time.Millisecond})
	must(t, err)
	s, err := p.sample(context.Background(), 7)
	must(t, err)
	health := diagnostics{State: "dropping", LastError: "ack lost", Reconnects: 3, Drops: 5}
	statusRaw, batchRaw, err := payloads("other-relay", s, health, p.config.Cadence)
	must(t, err)
	var status map[string]any
	must(t, json.Unmarshal(statusRaw, &status))
	if status["publisher_state"] != "dropping" || status["last_error"] != "ack lost" || status["reconnects"] != float64(3) || status["drops"] != float64(5) {
		t.Fatalf("status diagnostics: %v", status)
	}
	if status["status"].(map[string]any)["rt_core"] == nil {
		t.Fatal("status lost full public observation")
	}
	// v4 S4/S11 and A1-A3 decode the envelope and record maps. Their evidence
	// projection reads these raw fields without requiring the additive rt_core.
	var batch struct {
		IntervalMS int64 `json:"interval_ms"`
		Count      int
		Records    []map[string]any
		Sequence   uint64
	}
	must(t, json.Unmarshal(batchRaw, &batch))
	if batch.IntervalMS != 250 || batch.Count != 1 || batch.Sequence != 7 || len(batch.Records) != 1 {
		t.Fatalf("batch: %+v", batch)
	}
	record := batch.Records[0]
	if _, exists := record["rt_core"]; exists {
		t.Fatal("batch duplicates rt_core")
	}
	for _, key := range []string{"test_session_id", "test-session-id", "axis_current_ma", "axis_enable_mask", "axis_error_mask", "axis_fault_mask", "axis_pos_counts", "axis_target_counts", "axis_voltage_mv", "event", "hm35_home_position_counts", "sample", "servo_control_enabled", "servos_armed_requested", "time_ms"} {
		if _, exists := record[key]; !exists {
			t.Fatalf("batch reader field missing: %s", key)
		}
	}
	for _, key := range []string{"realtime_core_command_log_complete", "pair_config_drive_ratio_converged"} {
		if value, exists := record[key]; !exists || value != nil {
			t.Fatalf("unknown measurement %s: %v", key, value)
		}
	}
	if record["axis_pos_counts"].([]any)[0] != float64(1234) || record["sample"] != float64(6) {
		t.Fatalf("record observations: %v", record)
	}
	for _, cadence := range []time.Duration{-time.Millisecond, time.Microsecond, 5001 * time.Millisecond} {
		if _, err := New(&sampleObserver{}, Config{URL: p.config.URL, Host: p.config.Host, Cadence: cadence}); err == nil {
			t.Fatalf("invalid cadence accepted: %s", cadence)
		}
	}
	defaultPublisher, err := New(&sampleObserver{}, Config{URL: p.config.URL, Host: p.config.Host})
	must(t, err)
	if defaultPublisher.config.Cadence.Nanoseconds() != 500000000 {
		t.Fatal("default differs from deployed 500 ms")
	}
}

// PR 97 keeps periodic status separate from owner-issued terminal/permit replies.
// Decode the legacy envelope and require unavailable authority to stay unknown.
func TestPositionOwnerMergePreservesStatusCompatibility(t *testing.T) {
	p, err := New(&sampleObserver{}, Config{URL: "nats://127.0.0.1:4222", Host: "bench-servo"})
	must(t, err)
	s, err := p.sample(context.Background(), 1)
	must(t, err)
	raw, _, err := payloads("bench-servo", s, diagnostics{}, p.config.Cadence)
	must(t, err)
	// Matches natsbridge.StatusMessage at main 4978a161; additive diagnostics
	// must not replace the status object with a command receipt.
	var legacy struct {
		Schema, Stream, Subject, Host, Component string
		PublishedAt                              string `json:"published_at"`
		Status                                   map[string]json.RawMessage
	}
	must(t, json.Unmarshal(raw, &legacy))
	if legacy.Schema != "robot.v4.rtcore.status.v1" || legacy.Stream != "ROBOT_V4_RTCORE" ||
		legacy.Subject != "robot/v4/rtcore.bench-servo.status" || legacy.Host != "bench-servo" || legacy.PublishedAt == "" {
		t.Fatalf("legacy status envelope changed: %s %s %s %s", legacy.Schema, legacy.Stream, legacy.Subject, legacy.Host)
	}
	for _, key := range []string{"recent_commands", "runtime_mode_epoch", "pair_config_activation_epoch", "pair_config_home_epoch"} {
		if string(legacy.Status[key]) != "null" {
			t.Fatalf("unobserved legacy authority/terminal field %s: %s", key, legacy.Status[key])
		}
	}
	for _, key := range []string{"command_id", "accepted", "bridge_boot_id", "run_id", "remaining_ms"} {
		if _, exists := legacy.Status[key]; exists {
			t.Fatalf("status synthesized an execution receipt: %s", key)
		}
	}
	if string(legacy.Status["axis_pos_counts"]) != "[1234]" {
		t.Fatalf("raw count observation changed: %s", legacy.Status["axis_pos_counts"])
	}
}

// statusNumber reads a JSON number from a decoded envelope without panicking the
// poll predicate on a missing or non-numeric field; -1 fails any threshold.
func statusNumber(value any) float64 {
	if number, ok := value.(float64); ok {
		return number
	}
	return -1
}
