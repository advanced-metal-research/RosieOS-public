//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"fmt"
	"sync/atomic"
	"testing"
	"time"
)

type receiptOnWaitContext struct {
	context.Context
	onWait func()
}

func (c receiptOnWaitContext) Done() <-chan struct{} {
	c.onWait()
	return c.Context.Done()
}

func TestCellIOPreparePointRefusals(t *testing.T) {
	for _, test := range []struct {
		name   string
		mask   uint32
		values uint32
		reason uint32
	}{
		{"torch_off", 2, 0, ReasonIoTorchUnqualified},
		{"torch_on", 2, 2, ReasonIoTorchUnqualified},
		{"unconfigured", 4, 4, ReasonIoNotConfigured},
	} {
		t.Run(test.name, func(t *testing.T) {
			c := startCore(t, "cell-io/simulation.json").client
			points := []Point{
				{IOMask: test.mask, IOValues: test.values, Position: make([]float64, 9)},
				{Time: time.Second, Position: make([]float64, 9)},
			}
			handle, err := c.PrepareTrajectory(context.Background(), 42, 511, points)
			var point *TrajectoryPointError
			var rejected *RejectedError
			if handle != 0 || !errors.As(err, &point) || point.PointIndex != 0 || !errors.As(err, &rejected) ||
				rejected.Result.Operation != MSG_CMD_TRAJECTORY_POINT || rejected.Result.Reason != test.reason || rejected.Result.Handle != 42 {
				t.Fatalf("point refusal lost: handle=%d err=%v receipt=%+v", handle, err, rejected)
			}
		})
	}
}

// Real status-ring receipts must stop both publication and COMMIT waiting, even
// after bounded history evicts the original receipt or cancellation also arrives.
func TestTrajectoryPointReceiptStopsUpload(t *testing.T) {
	for _, reason := range []uint32{ReasonInvalidCommand, ReasonNotReady, ReasonInvalidTrajectory,
		ReasonOutsideLimitsOutward, ReasonIoNotConfigured, ReasonIoTorchUnqualified, 99} {
		t.Run(fmt.Sprint(reason), func(t *testing.T) {
			upload := &trajectoryUpload{firstSequence: 11, pointCount: 256}
			c := &Client{upload: upload, results: make(map[uint64]CommandResult), done: make(chan struct{}),
				commands: ring{data: make([]byte, 32+GRADIENT_RING_MSG_BYTES), capacity: 1},
				statuses: ring{data: make([]byte, 32+GRADIENT_RING_MSG_BYTES), capacity: 1}}
			enqueue := func(result CommandResult) {
				t.Helper()
				written, _ := c.statuses.indices()
				header := EncodeMsgHeader(MsgHeader{Type: MsgCommandResult, Bytes: MsgHeaderBytes + CommandResultBytes})
				payload := EncodeCommandResult(result)
				copy(c.statuses.data[32:], header[:])
				copy(c.statuses.data[32+MsgHeaderBytes:], payload[:])
				atomic.StoreUint32(c.statuses.counter(RingHeaderV1WriteIdxOffset), written+1)
			}
			publish := func(result CommandResult) {
				enqueue(result)
				if err := c.drain(); err != nil {
					t.Fatal(err)
				}
			}
			// A prior upload cannot contaminate this one's diagnosis.
			publish(CommandResult{Sequence: 10, Operation: MSG_CMD_TRAJECTORY_POINT, Result: ResultRejected, Reason: ReasonIoTorchUnqualified})
			if err := c.uploadError(upload); err != nil {
				t.Fatal("captured unrelated receipt", err)
			}
			atomic.StoreUint32(c.commands.counter(RingHeaderV1WriteIdxOffset), 1)
			// Unverified wait deadline; success requires the refusing receipt.
			ctx, cancel := context.WithTimeout(context.Background(), time.Second)
			defer cancel()
			finished := make(chan error, 1)
			go func() {
				_, err := c.sendUploadLocked(ctx, 0, MSG_CMD_TRAJECTORY_POINT, nil, upload)
				finished <- err
			}()
			want := CommandResult{Sequence: 12, Generation: 7, Operation: MSG_CMD_TRAJECTORY_POINT, Result: ResultRejected, Reason: reason, AxisMask: 1}
			publish(want)
			for sequence := uint64(13); sequence <= 267; sequence++ {
				operation := uint32(MSG_CMD_TRAJECTORY_POINT)
				if sequence == 267 {
					operation = MSG_CMD_TRAJECTORY_COMMIT
				}
				publish(CommandResult{Sequence: sequence, Operation: operation, Result: ResultRejected, Reason: ReasonNotReady})
			}
			if _, retained := c.results[12]; retained {
				t.Fatal("test did not evict original receipt")
			}
			check := func(err error) {
				t.Helper()
				var point *TrajectoryPointError
				var rejected *RejectedError
				if !errors.As(err, &point) || point.PointIndex != 1 || !errors.As(err, &rejected) || rejected.Result != want {
					t.Fatalf("original point refusal lost: %v", err)
				}
			}
			check(<-finished)
			// Cancellation must not hide a refusal that has already been observed.
			cancel()
			c.revocation.Store(1)
			for _, operation := range []uint16{MSG_CMD_TRAJECTORY_POINT, MSG_CMD_TRAJECTORY_COMMIT} {
				_, err := c.sendUploadLocked(ctx, 0, operation, nil, upload)
				check(err)
			}
			written, _ := c.commands.indices()
			if written != 1 {
				t.Fatalf("command count changed after refusal: %d", written)
			}
			_, err := c.waitUploadResult(ctx, 267, 0, upload)
			check(err)

			// Also deliver the refusal and cancellation after the wait has checked
			// its receipt cache, rather than only testing an already cached refusal.
			c.stateMu.Lock()
			upload.refusal = nil
			delete(c.results, 267)
			c.stateMu.Unlock()
			c.revocation.Store(0)
			waiting, stopWaiting := context.WithTimeout(context.Background(), time.Second)
			defer stopWaiting()
			_, err = c.waitUploadResult(receiptOnWaitContext{waiting, func() {
				publish(want)
				stopWaiting()
			}}, 267, 0, upload)
			check(err)

			// Revocation can be visible in shared status before the service has
			// drained the already-published point receipt into its local cache.
			c.stateMu.Lock()
			upload.refusal = nil
			c.stateMu.Unlock()
			waiting, stopWaiting = context.WithTimeout(context.Background(), time.Second)
			defer stopWaiting()
			_, err = c.waitUploadResult(receiptOnWaitContext{waiting, func() {
				enqueue(want)
				stopWaiting()
			}}, 267, 0, upload)
			check(err)
			// Closed/unmapped status storage cannot be read, but a captured
			// refusal still survives teardown.
			close(c.done)
			c.statuses.data = nil
			check(c.uploadTerminalError(upload, context.Canceled))
			if err := c.drain(); !errors.Is(err, ErrClosed) {
				t.Fatalf("closed status ring was accessed: %v", err)
			}
		})
	}
}
