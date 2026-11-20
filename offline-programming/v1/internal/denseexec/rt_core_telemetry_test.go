package denseexec

import (
	"context"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// A ring of records seq [first, last] at 1 kHz, served the way the adapter
// serves it: a cursor behind the ring returns from the oldest record, a cursor
// past the head is refused, a batch holds at most 4096 records.
type fakeTelemetryClient struct {
	*fakeRTCore
	first, last uint64
	peeks       int
	reads       []uint64
}

const fakeRingStartNS = 1_000_000_000_000

func TestTelemetryWindowCoordinateEvidence(t *testing.T) {
	v := ipcclient.CycleCaptureRecordV2{Axes: 1, ConfigurationEpoch: 4, HomeEpoch: 7, HomeValidMask: 1, ActiveTrajId: 9, SubmittedAxisMask: 1}
	v.Ax[0].CoordinateCounts = 6_553_637
	v.Ax[0].NativeHomePositionOffset = -6_553_600
	v.Ax[0].CoordinateValid, v.Ax[0].CoordinateSourceValid, v.Ax[0].PdoFresh = 1, 1, 1
	got := windowRecord(v, 1)
	if !got.Ax[0].CoordinateOK || got.Ax[0].CoordinateCounts+int64(got.Ax[0].HomeOffset) != 37 || got.TrajectoryID != 9 || got.HomeEpoch != 7 || got.ConfigurationEpoch != 4 || got.SubmittedMask != 1 {
		t.Fatalf("coordinate/command evidence lost: %+v", got)
	}
	v.Ax[0].CoordinateSourceValid = 0
	if windowRecord(v, 1).Ax[0].CoordinateOK {
		t.Fatal("unqualified encoder plotted as valid")
	}
	v.Ax[0].CoordinateSourceValid = 1
	v.HomeValidMask = 0
	if windowRecord(v, 1).Ax[0].CoordinateOK {
		t.Fatal("unhomed coordinate plotted as valid")
	}
}

func (f *fakeTelemetryClient) record(seq uint64) ipcclient.CycleCaptureRecordV2 {
	v := ipcclient.CycleCaptureRecordV2{Seq: seq, TNs: fakeRingStartNS + seq*1_000_000, Axes: 2, Armed: 1}
	v.Ax[0].Position = int32(seq)
	v.Ax[0].VelocityActualCountsPerS = int32(seq % 7)
	v.Ax[0].VelocityActualValid = 1
	v.Ax[0].TorqueRaw = -3
	v.Ax[0].TorqueValid = 1
	v.Ax[1].Position = -int32(seq)
	return v
}

func (f *fakeTelemetryClient) header() ipcclient.TelemetryBatchHeaderV1 {
	var h ipcclient.TelemetryBatchHeaderV1
	h.CyclePeriodNs = 1_000_000
	h.AxisCount = 2
	copy(h.DaemonIncarnation[:], "daemon")
	return h
}

func (f *fakeTelemetryClient) TelemetryPeek(context.Context, uint64) (ipcclient.TelemetryBatchHeaderV1, *ipcclient.CycleCaptureRecordV2, error) {
	f.peeks++
	h := f.header()
	if f.last < f.first {
		return h, nil, nil
	}
	h.FirstSequence, h.LastSequence, h.RecordCount = f.first, f.last, 1
	first := f.record(f.first)
	return h, &first, nil
}

func (f *fakeTelemetryClient) TelemetryBatches(_ context.Context, after uint64) (control.TelemetryBatch, error) {
	f.reads = append(f.reads, after)
	if after > f.last {
		return control.TelemetryBatch{}, &control.Rejected{Reason: "telemetry_cursor_ahead"}
	}
	start := after + 1
	if start < f.first {
		start = f.first
	}
	b := control.TelemetryBatch{Header: f.header()}
	for seq := start; seq <= f.last && len(b.Records) < int(ipcclient.TelemetryBatchMaxRecords); seq++ {
		b.Records = append(b.Records, f.record(seq))
	}
	b.Header.RecordCount = uint32(len(b.Records))
	if len(b.Records) > 0 {
		b.Header.FirstSequence, b.Header.LastSequence = b.Records[0].Seq, b.Records[len(b.Records)-1].Seq
	} else {
		b.Header.LastSequence = after
	}
	return b, nil
}

func telemetryFake(t *testing.T, first, last uint64, statusSeq uint64) (*RTCoreController, *fakeTelemetryClient) {
	t.Helper()
	r, f, _ := rtFake(t)
	f.mu.Lock()
	f.status.TimeNS = fakeRingStartNS + statusSeq*1_000_000
	f.status.DaemonIncarnation = "daemon"
	f.mu.Unlock()
	client := &fakeTelemetryClient{fakeRTCore: f, first: first, last: last}
	r.c = client
	return r, client
}

// The newest second of a ring is read with one anchoring peek and one small
// batch, not by paging from the oldest record; the stride thins the answer.
func TestTelemetryWindowReadsTheNewestSecondsOnly(t *testing.T) {
	r, client := telemetryFake(t, 1000, 60_000, 60_000)
	window, err := r.TelemetryWindow(context.Background(), 1.0, 10)
	if err != nil {
		t.Fatal(err)
	}
	if client.peeks != 1 {
		t.Fatalf("peeks: %d", client.peeks)
	}
	if window.LastSequence != 60_000 || window.FirstSequence != 59_001 {
		t.Fatalf("window %d..%d", window.FirstSequence, window.LastSequence)
	}
	if window.HeadEstimate != 60_000 || window.FetchedRecords > 1_200 {
		t.Fatalf("head %d fetched %d", window.HeadEstimate, window.FetchedRecords)
	}
	if len(window.Records) != 100 || window.Records[1].Seq-window.Records[0].Seq != 10 {
		t.Fatalf("stride: %d records, first gap %d", len(window.Records), window.Records[1].Seq-window.Records[0].Seq)
	}
	if got := window.Records[0].Ax[0]; got.P != int32(window.Records[0].Seq) || !got.VOK || got.TQ != -3 || !got.TQOK {
		t.Fatalf("axis fields: %+v", got)
	}
	if len(window.Records[0].Ax) != 2 || window.AxisCount != 2 || window.CyclePeriodNS != 1_000_000 {
		t.Fatalf("shape: %+v", window)
	}
	for _, after := range client.reads {
		if after < 58_000 {
			t.Fatalf("read from %d: the window must not page from the oldest record", after)
		}
	}
	// The next window reuses the anchor: no second peek.
	if _, err = r.TelemetryWindow(context.Background(), 0.5, 1); err != nil || client.peeks != 1 {
		t.Fatalf("second window: %v peeks %d", err, client.peeks)
	}
}

// A status clock that leads the ring makes the estimate run past the head: the
// refusal steps the cursor back, and a first read that lands short of the
// window is filled from older sequences, so the answer is still the newest
// full second ending at the newest record.
func TestTelemetryWindowBacksOffFromACursorAhead(t *testing.T) {
	r, client := telemetryFake(t, 1000, 20_000, 22_000)
	window, err := r.TelemetryWindow(context.Background(), 1.0, 1)
	if err != nil {
		t.Fatal(err)
	}
	if window.LastSequence != 20_000 || window.FirstSequence != 19_001 || len(window.Records) != 1000 {
		t.Fatalf("window %d..%d, %d records", window.FirstSequence, window.LastSequence, len(window.Records))
	}
	if client.reads[0] <= 20_000 {
		t.Fatalf("the first read should have overshot: %v", client.reads)
	}
	// A lead smaller than the margin still yields the full window.
	r2, _ := telemetryFake(t, 1000, 20_000, 20_800)
	window, err = r2.TelemetryWindow(context.Background(), 1.0, 1)
	if err != nil || window.FirstSequence != 19_001 || window.LastSequence != 20_000 || len(window.Records) != 1000 {
		t.Fatalf("short first read not filled: %d..%d %d records %v", window.FirstSequence, window.LastSequence, len(window.Records), err)
	}
	// A ring shorter than the window gives what it has, from its oldest record.
	r3, _ := telemetryFake(t, 19_800, 20_000, 20_000)
	window, err = r3.TelemetryWindow(context.Background(), 1.0, 1)
	if err != nil || window.FirstSequence != 19_800 || window.LastSequence != 20_000 || len(window.Records) != 201 {
		t.Fatalf("short ring: %d..%d %d records %v", window.FirstSequence, window.LastSequence, len(window.Records), err)
	}
}

// Bounds are the operator's: a window longer than the ring's useful span or a
// stride past the maximum is refused before any read; an empty ring is an
// empty answer, not an error.
func TestTelemetryWindowBoundsAndEmptyRing(t *testing.T) {
	r, client := telemetryFake(t, 1000, 999, 5000)
	for _, bad := range []struct {
		seconds float64
		stride  int
	}{{0, 1}, {101, 1}, {1, 0}, {1, 101}} {
		if _, err := r.TelemetryWindow(context.Background(), bad.seconds, bad.stride); err == nil || !strings.Contains(err.Error(), "telemetry_window_invalid") {
			t.Fatalf("%v accepted: %v", bad, err)
		}
	}
	if client.peeks != 0 || len(client.reads) != 0 {
		t.Fatal("refused parameters must not touch the ring")
	}
	window, err := r.TelemetryWindow(context.Background(), 1, 1)
	if err != nil || len(window.Records) != 0 || window.FetchedRecords != 0 {
		t.Fatalf("empty ring: %+v %v", window, err)
	}
}

// A client without the telemetry surface is a typed refusal, not a panic.
func TestTelemetryWindowNeedsATelemetryReader(t *testing.T) {
	r, _, _ := rtFake(t)
	_, err := r.TelemetryWindow(context.Background(), 1, 1)
	if err == nil || !strings.Contains(err.Error(), "telemetry_unavailable") {
		t.Fatalf("plain client: %v", err)
	}
}
