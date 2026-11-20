//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"sync/atomic"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

// Unverified envelope policy: at most 16,384 bytes of metadata before points.
const controlEnvelopeBytes int64 = 16 << 10

// Measured by TestAdmissionWireMeasurements: a compact point with int64 time
// and two 16-axis vectors of the longest finite JSON float occupies 890 bytes,
// including its array separator. Whitespace also consumes this wire allowance.
const jsonPointWireBytes int64 = 890

// Unverified dense metadata policy: at most 1,048,576 bytes plus its u32 length.
const programEnvelopeBytes int64 = (1 << 20) + 4

// These geometry-derived caps use the generated native point ceiling; the dense
// record measurement is 153 bytes in TestAdmissionWireMeasurements.
const jsonPreparationBytes = controlEnvelopeBytes + ipcclient.MaxTrajectoryPoints*jsonPointWireBytes
const programPreparationBytes = programEnvelopeBytes + ipcclient.MaxTrajectoryPoints*densejointtraj.RecordBytes

var errBodyTooLarge = errors.New(ReasonBodyTooLarge)

type preparationSlots struct {
	json    atomic.Bool
	program atomic.Bool
}

// admitPreparation reserves a slot before any body allocation. Ownership is
// shared by all HTTP handlers for this controller, preventing listener bypass.
func (c *Controller) admitPreparation(f Fence, program bool) (func(), error) {
	c.mu.Lock()
	err := c.authorize(f)
	c.mu.Unlock()
	if err != nil {
		return nil, err
	}
	slot := &c.preparations.json
	if program {
		slot = &c.preparations.program
	}
	if !slot.CompareAndSwap(false, true) {
		return nil, errors.New("busy")
	}
	return func() { slot.Store(false) }, nil
}

func knownOperation(operation string) bool {
	if operation == "mark_telemetry" {
		return true
	}
	switch operation {
	case OperationIoArm, OperationIoDisarm, OperationIoTest, OperationRestoreReferenceVerifiedDelta:
		return true
	case "acquire", "renew", "release", "stop", "halt", "enable", "arm", "home", "restore_anchor", "reset_fault", "recovery_status", "jog", "begin_jog", "begin_path_jog", "end_jog", "prepare_trajectory", "start_trajectory", "discard_trajectory", "start_program":
		return true
	}
	return false
}

// readEnvelope stops before points once the fence is available. Small unordered
// requests may retain raw points within the bounded prefix, without decoding any
// Point or vector. Large preparations require metadata before points. The replay
// prefix includes decoder read-ahead, preventing lost bytes.
func readEnvelope(body io.Reader) (Request, []byte, error) {
	var prefix bytes.Buffer
	limit := &io.LimitedReader{R: body, N: controlEnvelopeBytes}
	decoder := json.NewDecoder(io.TeeReader(limit, &prefix))
	fields := make(map[string]json.RawMessage)
	var request Request
	token, err := decoder.Token()
	if err == nil && token != json.Delim('{') {
		err = errors.New("invalid_request_envelope")
	}
	for err == nil && decoder.More() {
		var keyToken json.Token
		keyToken, err = decoder.Token()
		if err != nil {
			break
		}
		key, ok := keyToken.(string)
		if !ok {
			err = errors.New("invalid_request_envelope")
			break
		}
		if _, duplicate := fields[key]; duplicate {
			err = errors.New("duplicate_request_field")
			break
		}
		if key == "points" {
			if request.Operation == "prepare_trajectory" && request.Schema != "" && request.Session != "" && request.Generation != 0 {
				break
			}
			var value json.RawMessage
			err = decoder.Decode(&value)
			fields[key] = value
			continue
		}
		var value json.RawMessage
		err = decoder.Decode(&value)
		if err != nil {
			break
		}
		fields[key] = value
		switch key {
		case "schema":
			err = json.Unmarshal(value, &request.Schema)
		case "operation":
			err = json.Unmarshal(value, &request.Operation)
		case "session":
			err = json.Unmarshal(value, &request.Session)
		case "generation":
			err = json.Unmarshal(value, &request.Generation)
		}
	}
	if limit.N == 0 && err != nil {
		err = errBodyTooLarge
	}
	if err == nil && request.Schema != "rosie.rt-control.request.v1" {
		err = errors.New("schema_mismatch")
	}
	if err == nil && !knownOperation(request.Operation) {
		err = errors.New("unknown_operation")
	}
	return request, prefix.Bytes(), err
}

// readAdmittedBody enforces the observed size, including chunked bodies and a
// dishonest Content-Length. No payload decoder runs until this check succeeds.
func readAdmittedBody(w http.ResponseWriter, r *http.Request, prefix []byte, cap int64) ([]byte, error) {
	if r.ContentLength > cap {
		return nil, errBodyTooLarge
	}
	reader := io.MultiReader(bytes.NewReader(prefix), r.Body)
	body, err := io.ReadAll(http.MaxBytesReader(w, io.NopCloser(reader), cap))
	var tooLarge *http.MaxBytesError
	if errors.As(err, &tooLarge) {
		err = errBodyTooLarge
	}
	return body, err
}

// rejectUnreadBody closes rejected uploads and drains only already available
// bytes, with a 16,384-byte hard limit and an immediate read deadline. A slow
// sender cannot delay the reply or net/http's implicit drain on handler return.
func rejectUnreadBody(w http.ResponseWriter, r *http.Request) {
	r.Close = true
	w.Header().Set("Connection", "close")
	if http.NewResponseController(w).SetReadDeadline(time.Now()) == nil {
		_, _ = io.CopyN(io.Discard, r.Body, controlEnvelopeBytes)
	}
}

// preparationContext joins HTTP cancellation to session revocation. Cleanup
// still uses the independent discard context, so disconnect cannot strand native
// staging or resurrect a handle whose reply raced cancellation.
func preparationContext(session, request context.Context) (context.Context, func()) {
	ctx, cancel := context.WithCancel(session)
	stop := context.AfterFunc(request, cancel)
	if request.Err() != nil {
		cancel()
	}
	return ctx, func() { stop(); cancel() }
}

func (c *Controller) prepareRequest(ctx context.Context, request Request) (uint64, error) {
	return c.preparationOperation(ctx, request.Fence, func(joined context.Context) (uint64, error) {
		handle, err := c.prepare(request.Fence, joined, request.AxisMask, request.Points, request.Identity)
		if err == nil && joined.Err() != nil {
			err = c.cleanupPreparation(handle, joined.Err())
		}
		return handle, err
	})
}

// preparationOperation never queues decoded plans behind another native writer.
// This also prevents a disconnected upload from retaining its slot while waiting
// for unrelated Home/Start work to release the lifecycle lock. Recheck the fence
// before a capacity refusal so Stop during body admission cannot return busy.
func (c *Controller) preparationOperation(request context.Context, f Fence, fn func(context.Context) (uint64, error)) (uint64, error) {
	if !c.operationMu.TryLock() {
		c.mu.Lock()
		err := c.authorize(f)
		c.mu.Unlock()
		if err != nil {
			return 0, err
		}
		return 0, errors.New("busy")
	}
	defer c.operationMu.Unlock()
	c.mu.Lock()
	err := c.authorize(f)
	session := c.sessionCtx
	c.mu.Unlock()
	if err != nil {
		return 0, err
	}
	ctx, cancel := preparationContext(session, request)
	defer cancel()
	if err := ctx.Err(); err != nil {
		return 0, err
	}
	return fn(ctx)
}
