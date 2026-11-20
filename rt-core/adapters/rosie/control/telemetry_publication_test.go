//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

type busyTelemetryReader struct{ calls, busyCalls int }

func (r *busyTelemetryReader) ReadFrom(sequence uint64, limit int) (ipcclient.TelemetryBatch, error) {
	r.calls++
	if r.calls <= r.busyCalls {
		return ipcclient.TelemetryBatch{NextSequence: sequence}, ipcclient.ErrTelemetryBusy
	}
	return ipcclient.TelemetryBatch{Written: sequence, NextSequence: sequence + 1, Records: []ipcclient.CycleCaptureRecordV2{{Seq: sequence, Axes: 1}}}, nil
}

// Valid fixed identities keep reader tests focused on contention and cursors.
func telemetryHeaderFixture() ipcclient.TelemetryBatchHeaderV1 {
	h := ipcclient.TelemetryBatchHeaderV1{Magic: 0x31425452, Version: 1, HeaderBytes: 312, RecordBytes: 5392, CyclePeriodNs: 1000000, AxisCount: 1}
	copy(h.AdapterIncarnation[:], strings.Repeat("a", 32))
	copy(h.DaemonIncarnation[:], strings.Repeat("b", 32))
	copy(h.MachineSha256[:], strings.Repeat("c", 64))
	copy(h.DeploymentSha256[:], strings.Repeat("d", 64))
	copy(h.RecordLayoutDigest[:], ipcclient.CycleCaptureRecordV2LayoutDigest)
	return h
}

// A torn header gets exactly one handler retry; permanent contention remains a
// retryable HTTP result, never a zero-record success or false future cursor.
func TestTelemetryBusyRetryBound(t *testing.T) {
	h := telemetryHeaderFixture()
	reader := &busyTelemetryReader{busyCalls: 1}
	batch, err := telemetryRead(reader, h, 0)
	if err != nil || reader.calls != 2 || len(batch.Records) != 1 || batch.Header.LastSequence != 1 {
		t.Fatalf("repaired header: calls=%d batch=%+v err=%v", reader.calls, batch, err)
	}
	reader = &busyTelemetryReader{busyCalls: 3}
	batch, err = telemetryRead(reader, h, 17)
	if !errors.Is(err, ipcclient.ErrTelemetryBusy) || reader.calls != 2 || len(batch.Records) != 0 {
		t.Fatalf("retry bound: calls=%d err=%v", reader.calls, err)
	}
	w := httptest.NewRecorder()
	telemetryFailure(w, err)
	var response Response
	if json.Unmarshal(w.Body.Bytes(), &response) != nil || w.Code != 409 || w.Header().Get("Retry-After") != "1" || response.Error != ReasonTelemetryBusy {
		t.Fatalf("busy HTTP: %d %v %s", w.Code, w.Header(), w.Body.String())
	}
	w = httptest.NewRecorder()
	telemetryFailure(w, errors.New(ReasonTelemetryUnavailable))
	if w.Code != 409 || w.Header().Get("Retry-After") != "" {
		t.Fatal("unavailable ring became busy")
	}
}

// A busy mid-stream sample must emit no records and preserve the cursor, then
// deliver the next coherent record on the same HTTP connection without EOF.
func TestTelemetryStreamBusyContinues(t *testing.T) {
	h := telemetryHeaderFixture()
	initial, err := telemetryRead(&busyTelemetryReader{}, h, 0)
	if err != nil {
		t.Fatal(err)
	}
	reader := &busyTelemetryReader{busyCalls: 2}
	finished := make(chan struct{})
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		defer close(finished)
		w.Header().Set("Content-Type", "application/octet-stream")
		streamTelemetry(w, r, reader, h, initial, func() error { return nil }, nil)
	}))
	defer server.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	request, _ := http.NewRequestWithContext(ctx, http.MethodGet, server.URL, nil)
	response, err := server.Client().Do(request)
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	for i, want := range []struct {
		last  uint64
		count int
	}{{1, 1}, {1, 0}, {2, 1}} {
		batch, err := ipcclient.ReadTelemetryPublication(response.Body)
		if err != nil || batch.Header.LastSequence != want.last || len(batch.Records) != want.count || batch.Header.Dropped != 0 {
			t.Fatalf("batch %d: %+v err=%v", i, batch.Header, err)
		}
		if want.count == 0 && batch.Header.FirstSequence != 0 {
			t.Fatal("busy batch fabricated a first sequence")
		}
	}
	cancel()
	<-finished
}

// HTTP observes through the live descriptor handoff while the motion operation
// lock is held; invalid/future cursors retain explicit catalogue reasons.
func TestTelemetryPublicHTTP(t *testing.T) {
	f := startLifecycleCore(t)
	server := httptest.NewServer(NewHandler(f.controller))
	defer server.Close()
	for _, query := range []string{"after=", "after=-1", "after=1&after=2", "after=18446744073709551616", "after=18446744073709551615"} {
		response, err := server.Client().Get(server.URL + "/v1/telemetry?" + query)
		if err != nil {
			t.Fatal(err)
		}
		var result Response
		err = json.NewDecoder(response.Body).Decode(&result)
		response.Body.Close()
		want := ReasonInvalidTelemetryCursor
		if query == "after=18446744073709551615" {
			want = ReasonTelemetryCursorAhead
		}
		if err != nil || response.StatusCode != 409 || result.Error != want {
			t.Fatalf("%s: %d %+v %v", query, response.StatusCode, result, err)
		}
	}
	f.controller.operationMu.Lock()
	defer f.controller.operationMu.Unlock()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	request, _ := http.NewRequestWithContext(ctx, http.MethodGet, server.URL+"/v1/telemetry?after=0", nil)
	response, err := server.Client().Do(request)
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	if !response.Uncompressed || response.Header.Get("Vary") != "Accept-Encoding" {
		t.Fatal("finite telemetry did not negotiate transparent gzip")
	}
	raw, err := io.ReadAll(io.LimitReader(response.Body, ipcclient.TelemetryBatchMaxBytes+1))
	if err != nil {
		t.Fatal(err)
	}
	if response.StatusCode != 200 || response.Header.Get("Content-Type") != "application/octet-stream" || len(raw) < 312 || len(raw) > 22085944 || (len(raw)-312)%5392 != 0 {
		t.Fatalf("binary response: status=%d bytes=%d", response.StatusCode, len(raw))
	}
	header, err := ipcclient.DecodeTelemetryBatchHeaderV1(raw[:312])
	if err != nil {
		t.Fatal(err)
	}
	if header.RecordCount == 0 || len(raw) != 312+int(header.RecordCount)*5392 || header.FirstSequence != header.Dropped+1 {
		t.Fatalf("header: %+v", header)
	}
	f.controller.mu.Lock()
	native := f.controller.native
	f.controller.mu.Unlock()
	client := native.current()
	clock, err := client.LocalJogClock()
	if err != nil || string(header.DaemonIncarnation[:]) != client.Incarnation() || clock.Incarnation != client.Incarnation() {
		t.Fatalf("publication identity: header=%s welcome=%s clock=%s err=%v", header.DaemonIncarnation, client.Incarnation(), clock.Incarnation, err)
	}
	// Explicitly refusing compression keeps the existing wire contract.
	request, _ = http.NewRequestWithContext(ctx, http.MethodGet, server.URL+"/v1/telemetry?after=0", nil)
	request.Header.Set("Accept-Encoding", "gzip;q=0")
	response, err = server.Client().Do(request)
	if err != nil {
		t.Fatal(err)
	}
	defer response.Body.Close()
	if response.Header.Get("Content-Encoding") != "" || response.ContentLength < 312 {
		t.Fatal("uncompressed telemetry contract changed")
	}
	if _, err := ipcclient.ReadTelemetryPublication(response.Body); err != nil {
		t.Fatal(err)
	}
}
