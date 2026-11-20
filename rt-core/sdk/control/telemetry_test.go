//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"reflect"
	"strings"
	"sync/atomic"
	"testing"
	"time"

	contract "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
)

// Both entry points preserve busy as a typed refusal; consumers can retry the
// same cursor without confusing transient metadata with a protocol violation.
func TestTelemetryCaptureDoesNotBlockAcquisitionPreparation(t *testing.T) {
	release := make(chan struct{})
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/v1/telemetry" {
			w.Header().Set("Content-Type", "application/octet-stream")
			w.(http.Flusher).Flush()
			<-release
			return
		}
		_ = json.NewEncoder(w).Encode(contract.Response{Schema: contract.ResponseSchema, Data: map[string]any{}})
	}))
	defer server.Close()
	defer close(release)
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	response, err := c.telemetryResponse(ctx, "/v1/telemetry", 0)
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	// The capture body stays open throughout preparation. No grant is acquired.
	if err := c.warmConnections(ctx, true); err != nil {
		t.Fatalf("capture blocked acquisition preparation: %v", err)
	}
}

func TestTelemetryBusyRejected(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.RawQuery != "after=17" {
			t.Errorf("cursor changed: %s", r.URL.RawQuery)
		}
		w.WriteHeader(http.StatusConflict)
		_ = json.NewEncoder(w).Encode(contract.Response{Schema: contract.ResponseSchema, Operation: contract.OperationTelemetry, Error: contract.ReasonTelemetryBusy})
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_, pollErr := c.TelemetryBatches(ctx, 17)
	streamErr := c.TelemetryStream(ctx, 17, func(TelemetryBatch) error { t.Error("busy invoked callback"); return nil })
	for _, err := range []error{pollErr, streamErr} {
		var refusal *Rejected
		if !errors.As(err, &refusal) || refusal.Reason != "telemetry_busy" || errors.Is(err, ErrProtocol) {
			t.Fatalf("busy refusal: %v", err)
		}
	}
}

// Polling and streaming preserve the native typed mismatch and refuse before
// callbacks or retries, so V1 cannot masquerade as a transient transport loss.
func TestTelemetryV1LayoutMismatch(t *testing.T) {
	// Measured pre-P8-34 digest and record size, retained for compatibility refusal.
	const oldDigest = "ae42a2ad68df32eff0d7745b7791a3c8b8e2f629991bbdb600b57d303e77b59c"
	h := telemetryFixture().Header
	copy(h.RecordLayoutDigest[:], oldDigest)
	h.RecordBytes = 2088
	raw := ipcclient.EncodeTelemetryBatchHeaderV1(h)
	var requests atomic.Int32
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		requests.Add(1)
		w.Header().Set("Content-Type", "application/octet-stream")
		_, _ = w.Write(raw[:])
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	// Unverified watchdog; completion requires the typed refusal, never a timeout.
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_, pollErr := c.TelemetryBatches(ctx, 0)
	streamErr := c.TelemetryStream(ctx, 0, func(TelemetryBatch) error { t.Error("V1 invoked callback"); return nil })
	for _, err := range []error{pollErr, streamErr} {
		var mismatch *ipcclient.TelemetryLayoutMismatchError
		if !errors.Is(err, ErrProtocol) || errors.Is(err, ErrTransport) || !errors.As(err, &mismatch) || mismatch.StoredDigest != oldDigest ||
			mismatch.ExpectedDigest != ipcclient.CycleCaptureRecordV2LayoutDigest ||
			!strings.Contains(err.Error(), oldDigest) || !strings.Contains(err.Error(), ipcclient.CycleCaptureRecordV2LayoutDigest) {
			t.Fatalf("V1 refusal: %v", err)
		}
	}
	if requests.Load() != 2 {
		t.Fatalf("layout refusal retried: %d requests", requests.Load())
	}
}

func telemetryFixture() TelemetryBatch {
	h := ipcclient.TelemetryBatchHeaderV1{Magic: 0x31425452, Version: 1, HeaderBytes: 312, RecordBytes: 5392, CyclePeriodNs: 1000000, AxisCount: 1, RecordCount: 2, FirstSequence: 9007199254740997, LastSequence: 9007199254740998, Dropped: 3}
	copy(h.AdapterIncarnation[:], strings.Repeat("a", 32))
	copy(h.DaemonIncarnation[:], strings.Repeat("e", 32))
	copy(h.MachineSha256[:], strings.Repeat("b", 64))
	copy(h.DeploymentSha256[:], strings.Repeat("c", 64))
	copy(h.RecordLayoutDigest[:], ipcclient.CycleCaptureRecordV2LayoutDigest)
	records := []ipcclient.CycleCaptureRecordV2{{Seq: h.FirstSequence, Axes: 1, TNs: 123, WorkNs: 456}, {Seq: h.LastSequence, Axes: 1, TNs: 1000123, WorkNs: 789}}
	records[0].Ax[0].Position = -42
	records[1].Ax[0].Target = 19
	return TelemetryBatch{Header: h, Records: records}
}

// Fragmentation, explicit loss and high cursors must survive both transports;
// malformed payloads must fail before a callback sees partial evidence.
func TestTelemetrySDKBinaryFraming(t *testing.T) {
	b := telemetryFixture()
	raw, err := b.Encode()
	must(t, err)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.RawQuery != "after=9007199254740993" {
			t.Errorf("cursor changed: %s", r.URL.RawQuery)
		}
		w.Header().Set("Content-Type", "application/octet-stream")
		for start := 0; start < len(raw); start += 17 {
			_, _ = w.Write(raw[start:min(start+17, len(raw))])
			w.(http.Flusher).Flush()
		}
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	got, err := c.TelemetryBatches(ctx, 9007199254740993)
	must(t, err)
	if !reflect.DeepEqual(got, b) {
		t.Fatal("polling changed telemetry")
	}
	done := errors.New("observed")
	err = c.TelemetryStream(ctx, 9007199254740993, func(got TelemetryBatch) error {
		if !reflect.DeepEqual(got, b) {
			t.Fatal("stream changed telemetry")
		}
		return done
	})
	if !errors.Is(err, done) {
		t.Fatal(err)
	}
	for _, mutate := range []func([]byte) []byte{
		func(p []byte) []byte { return p[:311] }, func(p []byte) []byte { return p[:len(p)-1] },
		func(p []byte) []byte { binary.LittleEndian.PutUint32(p[268:], 4097); return p },
		func(p []byte) []byte { p[168] ^= 1; return p }, func(p []byte) []byte { p[276] = 1; return p },
		func(p []byte) []byte { p[312+8] ^= 1; return p }, func(p []byte) []byte { p[312+36] = 1; return p },
	} {
		if _, err := ipcclient.ReadTelemetryPublication(bytes.NewReader(mutate(append([]byte(nil), raw...)))); err == nil {
			t.Fatal("malformed telemetry accepted")
		}
	}
	b.Header.Dropped = 2
	if b.ValidateAfter(9007199254740993) == nil {
		t.Fatal("unreported loss accepted")
	}
}

// Real streaming cancellation and ring equivalence establish observation without
// authority; an impossible cursor must retain the catalogue refusal reason.
func TestTelemetryLiveStream(t *testing.T) {
	s := startService(t)
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	_, err := s.client.TelemetryBatches(ctx, ^uint64(0))
	var rejected *Rejected
	if !errors.As(err, &rejected) || rejected.Reason != "telemetry_cursor_ahead" {
		t.Fatalf("future cursor: %v", err)
	}
	records := 0
	err = s.client.TelemetryStream(ctx, 0, func(b TelemetryBatch) error {
		records += len(b.Records)
		if records >= 2 {
			cancel()
		}
		return nil
	})
	if !errors.Is(err, context.Canceled) || records < 2 {
		t.Fatalf("stream: %d records, %v", records, err)
	}
}

// Pin the public allocation ceiling in bytes; oversized count fields cannot
// allocate a body, and the largest accepted batch has exactly 4096 records.
func TestTelemetryAbsoluteByteBound(t *testing.T) {
	b := telemetryFixture()
	b.Header.FirstSequence = 1
	b.Header.LastSequence = 4096
	b.Header.Dropped = 0
	b.Header.RecordCount = 4096
	b.Records = make([]ipcclient.CycleCaptureRecordV2, 4096)
	for i := range b.Records {
		b.Records[i] = ipcclient.CycleCaptureRecordV2{Seq: uint64(i + 1), Axes: 1}
	}
	raw, err := b.Encode()
	must(t, err)
	if len(raw) != 22085944 {
		t.Fatalf("maximum batch bytes=%d", len(raw))
	}
	got, err := ipcclient.ReadTelemetryPublication(bytes.NewReader(raw))
	must(t, err)
	if len(got.Records) != 4096 {
		t.Fatal("maximum batch truncated")
	}
	binary.LittleEndian.PutUint32(raw[268:], 4097)
	if _, err = ipcclient.ReadTelemetryPublication(bytes.NewReader(raw[:312])); err == nil {
		t.Fatal("oversized count accepted")
	}
}

// The peek reads the header and the first record of a batch and no more, so a
// consumer can anchor (sequence, host time) at the cost of one record: the
// body the server would go on writing is abandoned, not drained.
func TestTelemetryPeekReadsHeaderAndFirstRecordOnly(t *testing.T) {
	fixture := telemetryFixture()
	encoded, err := fixture.Encode()
	if err != nil {
		t.Fatal(err)
	}
	var served atomic.Int64
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.RawQuery != "after=0" {
			t.Errorf("cursor changed: %s", r.URL.RawQuery)
		}
		w.Header().Set("Content-Type", "application/octet-stream")
		n, _ := w.Write(encoded)
		served.Store(int64(n))
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	h, first, err := c.TelemetryPeek(ctx, 0)
	if err != nil {
		t.Fatal(err)
	}
	if h.FirstSequence != fixture.Header.FirstSequence || h.RecordCount != 2 || h.CyclePeriodNs != 1000000 {
		t.Fatalf("header: %+v", h)
	}
	if first == nil || first.Seq != fixture.Records[0].Seq || first.TNs != 123 || first.Ax[0].Position != -42 {
		t.Fatalf("first record: %+v", first)
	}
	if served.Load() != int64(len(encoded)) {
		t.Fatalf("server wrote %d of %d", served.Load(), len(encoded))
	}

	// An empty batch yields the header alone.
	empty := TelemetryBatch{Header: fixture.Header}
	empty.Header.RecordCount = 0
	empty.Header.FirstSequence = 0
	empty.Header.LastSequence = 40
	encodedEmpty, err := empty.Encode()
	if err != nil {
		t.Fatal(err)
	}
	emptyServer := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.Header().Set("Content-Type", "application/octet-stream")
		_, _ = w.Write(encodedEmpty)
	}))
	defer emptyServer.Close()
	e := newClient(emptyServer.URL, &http.Transport{})
	defer e.Close()
	h, first, err = e.TelemetryPeek(ctx, 40)
	if err != nil || first != nil || h.LastSequence != 40 {
		t.Fatalf("empty peek: %+v %v %v", h, first, err)
	}
}
