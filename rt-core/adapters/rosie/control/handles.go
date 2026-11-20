//go:build linux

package control

// Public handle transitions (all others are refused):
//
// From       To          Evidence
// absent     prepared    Native preparation acknowledged
// prepared   prepared    Start refused before admission; native storage retained/restored
// prepared   started     Native Start accepted
// started    consumed    Correlated native completion or a later accepted execution
// prepared   discarded   Native Discard acknowledged
// prepared   superseded  Replacement committed and old native storage released
// prepared   retired     Stop, grant loss, connection loss or ambiguous cleanup
// started    retired     Stop, grant loss, connection loss or ambiguous cleanup
//
// Terminal states never regain permission. Discard of an already discarded
// handle remains an idempotent native no-op. Retired is used whenever admission
// or reclamation is uncertain, preventing replay of possibly accepted motion.
import (
	"context"
	"errors"
	"sort"

	"rosieos/rt-core/ipcclient"
)

type preparation struct {
	HandleRecord
	owner  *Controller
	mask   uint32
	points []ipcclient.Point
}

// cloneTrajectoryPoints preserves every point field and owns the sample slices,
// preventing caller mutation or Start restoration from changing marker intent.
func cloneTrajectoryPoints(points []ipcclient.Point) []ipcclient.Point {
	owned := append([]ipcclient.Point(nil), points...)
	for i := range owned {
		owned[i].Position = append(points[i].Position[:0:0], points[i].Position...)
		owned[i].Velocity = append(points[i].Velocity[:0:0], points[i].Velocity...)
	}
	return owned
}

// transitionHandle is the sole state writer under c.mu; illegal transitions
// return false so a late native reply cannot revive a retired preparation.
func transitionHandle(h *preparation, to HandleState) bool {
	allowed := (h.State == "" && to == HandlePrepared) ||
		(h.State == HandlePrepared && (to == HandlePrepared || to == HandleStarted || to == HandleDiscarded || to == HandleSuperseded || to == HandleRetired)) ||
		(h.State == HandleStarted && (to == HandleConsumed || to == HandleRetired))
	if !allowed {
		return false
	}
	from := h.State
	h.State = to
	if to != HandlePrepared {
		h.points = nil
	}
	if h.owner != nil && from != to {
		h.owner.handleEventLocked(h, from)
	}
	return true
}

func (c *Controller) retireHandlesLocked() {
	for _, h := range c.handles {
		transitionHandle(h, HandleRetired)
	}
}

func (c *Controller) retireNativeLossLocked(err error) {
	var rejected *ipcclient.RejectedError
	if c.core.Err() != nil || errors.Is(err, ipcclient.ErrRevoked) ||
		(errors.As(err, &rejected) && (rejected.Result.Reason == ipcclient.ReasonInvalidCommand || rejected.Result.Reason == ipcclient.ReasonInvalidTrajectory)) {
		c.retireHandlesLocked()
	}
}

func (c *Controller) observeHandlesLocked(m ipcclient.MotionState) {
	for _, h := range c.handles {
		if h.State == HandleStarted {
			if m.CommandSequence == h.NativeSequence && m.TrajectoryID == h.Handle && m.Done {
				if m.State == 4 {
					transitionHandle(h, HandleConsumed)
				} else {
					transitionHandle(h, HandleRetired)
				}
			} else if m.CommandSequence > h.NativeSequence && m.Mode == 2 {
				if m.TrajectoryID == 0 {
					transitionHandle(h, HandleRetired)
				} else {
					transitionHandle(h, HandleConsumed)
				}
			}
		}
	}
}

// retainedTerminalHandles bounds the terminal handle records (consumed,
// discarded, superseded, retired) the adapter keeps for Status. Every loop
// cycle prepares a handle, so an unbounded history grows each full status
// read until it passes the SDK's 8 MiB response cap (sdk/control/client.go)
// and the read fails. A Rosie 1430 cell carried 3,927 handles, about 1.1 MB per read, on
// 2026-11-01; the 2026-11-02 fixture measured 1,129,453 bytes for 4,000
// handles (about 282 bytes each), so 1,024 records are about 290 KB. Clients
// correlate the receipt of a handle they just prepared or started, never one
// 1,024 handles old. This is a generic memory bound, not a machine setting.
const retainedTerminalHandles = 1024

// pruneTerminalHandlesLocked forgets the oldest terminal handles beyond
// retainedTerminalHandles. It runs at each preparation, the only place the
// map grows; a handle that turns terminal later is counted at the next one.
// Prepared and started handles are never dropped:
// one still carries Start permission, the other an outcome to observe. A
// forgotten handle answers unknown_handle, which like every terminal reason
// means upload anew. Handles are adapter-assigned and increase. Caller holds
// c.mu.
func (c *Controller) pruneTerminalHandlesLocked() {
	terminal := make([]uint64, 0, len(c.handles))
	for id, h := range c.handles {
		if h.State != HandlePrepared && h.State != HandleStarted {
			terminal = append(terminal, id)
		}
	}
	if len(terminal) <= retainedTerminalHandles {
		return
	}
	sort.Slice(terminal, func(i, j int) bool { return terminal[i] < terminal[j] })
	for _, id := range terminal[:len(terminal)-retainedTerminalHandles] {
		delete(c.handles, id)
	}
}

func (c *Controller) handleViewLocked() []HandleRecord {
	view := make([]HandleRecord, 0, len(c.handles))
	for _, h := range c.handles {
		view = append(view, h.HandleRecord)
	}
	sort.Slice(view, func(i, j int) bool { return view[i].Handle < view[j].Handle })
	return view
}

// publicRejection adds a stable reason while preserving the native receipt for
// existing clients that inspect native_result or errors.As.
type publicRejection struct {
	reason string
	cause  error
}

func (e *publicRejection) Error() string { return e.reason }
func (e *publicRejection) Unwrap() error { return e.cause }

// startHandle runs under operationMu. Native currently retires a cyclically
// refused Start, so definite pre-admission rejection restores its immutable
// preparation before reporting failure. Uncertain outcomes never permit retry.
func (c *Controller) startHandle(ctx context.Context, f Fence, id uint64, identity ...Identity) (uint64, error) {
	motion, _ := c.core.Motion()
	c.mu.Lock()
	c.observeHandlesLocked(motion)
	h := c.handles[id]
	if h == nil {
		c.mu.Unlock()
		return 0, errors.New(ReasonUnknownHandle)
	}
	var supplied Identity
	if len(identity) != 0 {
		supplied = identity[0]
	}
	if h.Identity != supplied {
		c.mu.Unlock()
		return 0, errors.New(ReasonTrajectoryIdentityMismatch)
	}
	if h.State != HandlePrepared {
		reason := handleStateReason(h.State)
		c.mu.Unlock()
		return 0, errors.New(reason)
	}
	if c.execution.State == "executing" {
		c.mu.Unlock()
		return 0, errors.New(ReasonModeConflict)
	}
	mask, points := h.mask, h.points
	c.mu.Unlock()
	seq, err := c.core.StartTrajectory(ctx, id)
	if err != nil {
		var rejected *ipcclient.RejectedError
		if errors.As(err, &rejected) && (rejected.Result.Reason == ipcclient.ReasonNotReady || rejected.Result.Reason == ipcclient.ReasonModeConflict) {
			reason := ReasonNotReady
			if rejected.Result.Reason == ipcclient.ReasonModeConflict {
				reason = ReasonModeConflict
			}
			_, restoreErr := c.core.PrepareTrajectory(ctx, id, mask, cloneTrajectoryPoints(points))
			c.mu.Lock()
			if restoreErr != nil || c.authorize(f) != nil {
				transitionHandle(h, HandleRetired)
			}
			c.retireNativeLossLocked(restoreErr)
			c.mu.Unlock()
			if restoreErr != nil {
				return seq, errors.Join(&publicRejection{reason, err}, restoreErr)
			}
			return seq, &publicRejection{reason, err}
		}
		c.mu.Lock()
		transitionHandle(h, HandleRetired)
		c.retireNativeLossLocked(err)
		c.mu.Unlock()
		if errors.Is(err, ipcclient.ErrRevoked) || errors.Is(err, context.Canceled) {
			return seq, &publicRejection{ReasonFence, err}
		}
		return seq, err
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	c.executionGeneration++
	for _, previous := range c.handles {
		if previous.State == HandleStarted {
			transitionHandle(previous, HandleConsumed)
		}
	}
	h.NativeSequence, h.ExecutionGeneration = seq, c.executionGeneration
	transitionHandle(h, HandleStarted)
	return seq, nil
}

// handleStateReason returns a closed refusal label; invalid internal states fail
// closed as retired rather than inventing a new public reason or allowing Start.
func handleStateReason(state HandleState) string {
	switch state {
	case HandleStarted:
		return ReasonHandleStarted
	case HandleConsumed:
		return ReasonHandleConsumed
	case HandleDiscarded:
		return ReasonHandleDiscarded
	case HandleSuperseded:
		return ReasonHandleSuperseded
	default:
		return ReasonHandleRetired
	}
}

// handleReceipt copies the immutable preparation identity and observed execution
// generation under the state lock; later transitions cannot rewrite a receipt.
func (c *Controller) handleReceipt(id uint64) HandleRecord {
	c.mu.Lock()
	defer c.mu.Unlock()
	if h := c.handles[id]; h != nil {
		return h.HandleRecord
	}
	return HandleRecord{}
}
