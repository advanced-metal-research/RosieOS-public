//go:build linux

package control

import (
	"compress/gzip"
	"errors"
	"net/http"
	"runtime"
	"strconv"
	"strings"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Unverified off-cycle batching budget: 20 ms amortizes HTTP writes at 1 kHz.
const telemetryChunkPeriod = 20 * time.Millisecond

// Unverified slow-consumer budget: 5 seconds, matching event stream writes.
const telemetryWriteTimeout = 5 * time.Second

// telemetrySource snapshots identity under the adapter mutex and performs the
// descriptor handoff after releasing it; telemetry never owns motion locks.
func (c *Controller) telemetrySource() (*ipcclient.TelemetryReader, *ipcclient.Client, ipcclient.TelemetryBatchHeaderV1, error) {
	c.mu.Lock()
	n := c.native
	d := c.description
	incarnation := c.adapterIncarnation
	c.mu.Unlock()
	var h ipcclient.TelemetryBatchHeaderV1
	if n == nil {
		return nil, nil, h, errors.New(ReasonTelemetryUnavailable)
	}
	client := n.current()
	reader, err := client.OpenTelemetry()
	if err != nil {
		return nil, nil, h, errors.New(ReasonTelemetryUnavailable)
	}
	h.Magic = ipcclient.TelemetryBatchMagic
	h.Version = 1
	h.HeaderBytes = ipcclient.TelemetryBatchHeaderV1Bytes
	h.RecordBytes = ipcclient.CycleCaptureRecordV2Bytes
	h.CyclePeriodNs = reader.CyclePeriodNS
	h.AxisCount = uint32(reader.AxisCount)
	copy(h.AdapterIncarnation[:], incarnation)
	copy(h.DaemonIncarnation[:], client.Incarnation())
	copy(h.MachineSha256[:], d.MachineSHA256)
	copy(h.DeploymentSha256[:], d.DeploymentSHA256)
	copy(h.RecordLayoutDigest[:], ipcclient.CycleCaptureRecordV2LayoutDigest)
	if err = ipcclient.ValidateTelemetryHeader(h); err != nil || reader.CyclePeriodNS != d.CycleNS || int(reader.AxisCount) != len(d.Axes) {
		reader.Close()
		return nil, nil, h, errors.New(ReasonTelemetryUnavailable)
	}
	return reader, client, h, nil
}

type telemetryBatchReader interface {
	ReadFrom(uint64, int) (ipcclient.TelemetryBatch, error)
}

// telemetryRead retries a transient header once, then reports explicit busy;
// no empty batch or future-cursor refusal can be inferred from a torn header.
func telemetryRead(reader telemetryBatchReader, h ipcclient.TelemetryBatchHeaderV1, after uint64) (ipcclient.TelemetryPublication, error) {
	var out ipcclient.TelemetryPublication
	if after == ^uint64(0) {
		return out, errors.New(ReasonTelemetryCursorAhead)
	}
	var b ipcclient.TelemetryBatch
	var err error
	// Unverified handler bound: two ReadFrom calls, each at most eight header
	// samples (16 pairs total), with one scheduler yield between calls.
	for attempt := 0; attempt < 2; attempt++ {
		b, err = reader.ReadFrom(after+1, ipcclient.TelemetryBatchMaxRecords)
		if !errors.Is(err, ipcclient.ErrTelemetryBusy) {
			break
		}
		if attempt == 0 {
			runtime.Gosched()
		}
	}
	if errors.Is(err, ipcclient.ErrTelemetryBusy) {
		return out, err
	}
	if err != nil {
		return out, errors.New(ReasonTelemetryUnavailable)
	}
	if after > b.Written {
		return out, errors.New(ReasonTelemetryCursorAhead)
	}
	h.Dropped = b.Drops
	h.LastSequence = after + b.Drops
	h.RecordCount = uint32(len(b.Records))
	if len(b.Records) > 0 {
		h.FirstSequence = b.Records[0].Seq
		h.LastSequence = b.Records[len(b.Records)-1].Seq
	}
	out = ipcclient.TelemetryPublication{Header: h, Records: b.Records}
	return out, out.ValidateAfter(after)
}

// telemetryFailure distinguishes retryable busy metadata from an unavailable
// ring, preserving typed HTTP 409 refusals and making retries explicit.
func telemetryFailure(w http.ResponseWriter, err error) {
	busy := errors.Is(err, ipcclient.ErrTelemetryBusy)
	if busy {
		err = errors.New(ReasonTelemetryBusy)
	}
	o := encodeResponse(Response{Schema: ResponseSchema, Operation: OperationTelemetry}, err)
	if busy {
		// Unverified HTTP retry hint: one second; no server-side sleep.
		w.Header().Set("Retry-After", "1")
	}
	w.WriteHeader(o.status)
	_, _ = w.Write(o.body)
}

// serveTelemetry publishes bounded private copies without session authority;
// stalled writes and native replacement close the stream without hiding loss.
func (h *Handler) serveTelemetry(w http.ResponseWriter, r *http.Request) {
	if !h.checkEventPrincipal(w, r) {
		return
	}
	fail := func(err error) { telemetryFailure(w, err) }
	after, err := eventCursor(r)
	if err != nil {
		fail(errors.New(ReasonInvalidTelemetryCursor))
		return
	}
	reader, client, header, err := h.Controller.telemetrySource()
	if err != nil {
		fail(err)
		return
	}
	defer reader.Close()
	batch, err := telemetryRead(reader, header, after)
	if err != nil {
		fail(err)
		return
	}
	raw, err := batch.Encode()
	if err != nil {
		fail(errors.New(ReasonTelemetryUnavailable))
		return
	}
	rc := http.NewResponseController(w)
	if err = rc.SetWriteDeadline(time.Now().Add(telemetryWriteTimeout)); err != nil {
		return
	}
	defer rc.SetWriteDeadline(time.Time{})
	w.Header().Set("Content-Type", "application/octet-stream")
	w.Header().Set("Cache-Control", "no-store")
	if r.URL.Path == "/v1/telemetry" {
		// Compress only finite observations, off-cycle. Go's HTTP client decodes
		// this transparently; the binary layout and its validation stay intact.
		w.Header().Add("Vary", "Accept-Encoding")
		for _, encoding := range strings.Split(r.Header.Get("Accept-Encoding"), ",") {
			if strings.TrimSpace(encoding) == "gzip" {
				w.Header().Set("Content-Encoding", "gzip")
				compressed, _ := gzip.NewWriterLevel(w, gzip.BestSpeed)
				_, _ = compressed.Write(raw)
				_ = compressed.Close()
				return
			}
		}
		w.Header().Set("Content-Length", strconv.Itoa(len(raw)))
		_, _ = w.Write(raw)
		return
	}
	streamTelemetry(w, r, reader, header, batch, client.Err, h.Controller.done)
}

// streamTelemetry keeps transient header contention on the same connection;
// empty batches preserve the cursor so later reads still report actual loss.
func streamTelemetry(w http.ResponseWriter, r *http.Request, reader telemetryBatchReader, header ipcclient.TelemetryBatchHeaderV1, batch ipcclient.TelemetryPublication, nativeError func() error, done <-chan struct{}) {
	rc := http.NewResponseController(w)
	ticker := time.NewTicker(telemetryChunkPeriod)
	defer ticker.Stop()
	for {
		raw, err := batch.Encode()
		if err != nil {
			return
		}
		if err = rc.SetWriteDeadline(time.Now().Add(telemetryWriteTimeout)); err != nil {
			return
		}
		if _, err = w.Write(raw); err != nil {
			return
		}
		if err = rc.Flush(); err != nil {
			return
		}
		after := batch.Header.LastSequence
		select {
		case <-r.Context().Done():
			return
		case <-done:
			return
		case <-ticker.C:
		}
		if nativeError() != nil {
			return
		}
		batch, err = telemetryRead(reader, header, after)
		if errors.Is(err, ipcclient.ErrTelemetryBusy) {
			empty := header
			empty.FirstSequence, empty.LastSequence = 0, after
			empty.RecordCount, empty.Dropped = 0, 0
			batch = ipcclient.TelemetryPublication{Header: empty}
			continue
		}
		if err != nil {
			return
		}
	}
}
