//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"strconv"

	contract "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
)

type TelemetryBatch = ipcclient.TelemetryPublication

func (c *Client) telemetryResponse(ctx context.Context, path string, after uint64) (*http.Response, error) {
	c.mu.Lock()
	closed := c.closed
	c.mu.Unlock()
	if closed {
		return nil, ErrClosed
	}
	r, err := http.NewRequestWithContext(ctx, http.MethodGet, c.base+path+"?after="+strconv.FormatUint(after, 10), nil)
	if err != nil {
		return nil, err
	}
	r.Header.Set("Accept", "application/octet-stream")
	response, err := c.reservedObservationHTTP(r, c.telemetrySlots, true)
	if err != nil {
		return nil, err
	}
	if response.StatusCode != http.StatusOK {
		defer response.Body.Close()
		var refusal contract.Response
		if response.StatusCode == http.StatusConflict && json.NewDecoder(io.LimitReader(response.Body, 65536)).Decode(&refusal) == nil && refusal.Schema == contract.ResponseSchema && refusal.Error != "" {
			return nil, &Rejected{Reason: refusal.Error}
		}
		return nil, ErrProtocol
	}
	if response.Header.Get("Content-Type") != "application/octet-stream" {
		response.Body.Close()
		return nil, ErrProtocol
	}
	return response, nil
}

// TelemetryBatches reads one bounded binary batch without authority. The caller
// resumes at Header.LastSequence and must account for Header.Dropped explicitly.
func (c *Client) TelemetryBatches(ctx context.Context, after uint64) (TelemetryBatch, error) {
	response, err := c.telemetryResponse(ctx, "/v1/telemetry", after)
	if err != nil {
		return TelemetryBatch{}, err
	}
	defer response.Body.Close()
	b, err := ipcclient.ReadTelemetryPublication(response.Body)
	if err != nil {
		return TelemetryBatch{}, errors.Join(ErrProtocol, err)
	}
	if err = b.ValidateAfter(after); err != nil {
		return TelemetryBatch{}, errors.Join(ErrProtocol, err)
	}
	var tail [1]byte
	if n, e := io.ReadFull(response.Body, tail[:]); n != 0 || e != io.EOF {
		return TelemetryBatch{}, ErrProtocol
	}
	return b, nil
}

// TelemetryStream decodes self-framed batches across arbitrary HTTP chunks until
// cancellation, callback failure or disconnect; no reconnect silently resets loss.
// Mid-stream busy samples are empty batches with the last handled cursor.
func (c *Client) TelemetryStream(ctx context.Context, after uint64, callback func(TelemetryBatch) error) error {
	if callback == nil {
		return errors.New("nil telemetry callback")
	}
	response, err := c.telemetryResponse(ctx, "/v1/telemetry/stream", after)
	if err != nil {
		return err
	}
	defer response.Body.Close()
	var incarnation [32]byte
	var daemon [32]byte
	first := true
	for {
		b, err := ipcclient.ReadTelemetryPublication(response.Body)
		if ctx.Err() != nil {
			return ctx.Err()
		}
		if err != nil {
			var mismatch *ipcclient.TelemetryLayoutMismatchError
			if errors.As(err, &mismatch) {
				return errors.Join(ErrProtocol, err)
			}
			return errors.Join(ErrTransport, err)
		}
		if err = b.ValidateAfter(after); err != nil {
			return errors.Join(ErrProtocol, err)
		}
		if !first && (b.Header.AdapterIncarnation != incarnation || b.Header.DaemonIncarnation != daemon) {
			return ErrProtocol
		}
		first = false
		incarnation = b.Header.AdapterIncarnation
		daemon = b.Header.DaemonIncarnation
		if err = callback(b); err != nil {
			return err
		}
		after = b.Header.LastSequence
	}
}

// TelemetryPeek reads one batch's header and its first record, then abandons
// the rest of the body. A consumer that needs a (sequence, host time) anchor
// for the ring, to estimate the newest sequence from the daemon's status time
// without paging through the ring, pays 312 + 5392 bytes instead of a full
// batch. The record is nil when the batch is empty. Cursor semantics are
// TelemetryBatches'.
func (c *Client) TelemetryPeek(ctx context.Context, after uint64) (ipcclient.TelemetryBatchHeaderV1, *ipcclient.CycleCaptureRecordV2, error) {
	response, err := c.telemetryResponse(ctx, "/v1/telemetry", after)
	if err != nil {
		return ipcclient.TelemetryBatchHeaderV1{}, nil, err
	}
	defer response.Body.Close()
	var raw [ipcclient.TelemetryBatchHeaderV1Bytes]byte
	if _, err = io.ReadFull(response.Body, raw[:]); err != nil {
		return ipcclient.TelemetryBatchHeaderV1{}, nil, errors.Join(ErrProtocol, err)
	}
	h, err := ipcclient.DecodeTelemetryBatchHeaderV1(raw[:])
	if err != nil {
		return ipcclient.TelemetryBatchHeaderV1{}, nil, errors.Join(ErrProtocol, err)
	}
	if err = ipcclient.ValidateTelemetryHeader(h); err != nil {
		return ipcclient.TelemetryBatchHeaderV1{}, nil, errors.Join(ErrProtocol, err)
	}
	if h.RecordCount == 0 {
		return h, nil, nil
	}
	var record [ipcclient.CycleCaptureRecordV2Bytes]byte
	if _, err = io.ReadFull(response.Body, record[:]); err != nil {
		return ipcclient.TelemetryBatchHeaderV1{}, nil, errors.Join(ErrProtocol, err)
	}
	v, err := ipcclient.DecodeCycleCaptureRecordV2(record[:])
	if err != nil {
		return ipcclient.TelemetryBatchHeaderV1{}, nil, errors.Join(ErrProtocol, err)
	}
	if v.Seq != h.FirstSequence || v.Axes != h.AxisCount {
		return ipcclient.TelemetryBatchHeaderV1{}, nil, errors.Join(ErrProtocol, errors.New("invalid telemetry record identity"))
	}
	return h, &v, nil
}
