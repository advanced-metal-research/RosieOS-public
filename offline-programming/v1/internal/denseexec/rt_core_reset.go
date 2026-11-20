//go:build linux

package denseexec

import (
	"context"
	"errors"
	"time"

	"rosieos/rt-core/sdk/control"
)

// ResetFault is the explicit, inhibited Clear fault operation. A native proof
// that a saved reference independently recovered permits clearing its remaining
// latch without Home. Arm's automatic reset uses the narrower path below: a
// held L2 must not clear a reference fault and resume motion.
func (r *RTCoreController) ResetFault(ctx context.Context) (*ArmResult, error) {
	return r.resetFault(ctx, true)
}

func (r *RTCoreController) resetFault(ctx context.Context, explicit bool) (*ArmResult, error) {
	ctx, cancel := context.WithTimeout(ctx, 8*time.Second)
	defer cancel()
	err := r.operationKind(ctx, "reset", func(ctx context.Context) error {
		before, err := r.resetRead(ctx, nil)
		if err != nil {
			return err
		}
		if !latchedFault(before) && !driveFaulted(before) && before.Core.LeaseValid == 0 && before.Grant.GrantActive == 0 {
			return nil
		}
		r.mu.Lock()
		held := r.held
		r.mu.Unlock()
		if !held && (before.Core.LeaseValid != 0 || before.Grant.GrantActive != 0) {
			return rtRefused("control_busy: another controller holds authority")
		}
		var reference *homeReferenceRecovery
		if explicit {
			reference = recoveredReferenceReset(before, r.cfg.AxisMask)
		}
		if !resetAdmits(before, reference) {
			return rtRefused("reset_fault_unavailable: native recovery requires Home or restart, or no resettable fault is present")
		}
		r.mu.Lock()
		r.armResetting = true
		r.resetReference = reference
		r.mu.Unlock()
		defer func() {
			r.mu.Lock()
			r.armResetting = false
			r.resetReference = nil
			r.homeResetPending = false
			r.mu.Unlock()
		}()
		if err = r.acquire(ctx); err != nil {
			return err
		}
		// Recheck after authority acquisition, including a new native publication.
		admitted, err := r.resetRead(ctx, &before)
		if err != nil {
			return err
		}
		if !resetAdmits(admitted, reference) {
			return rtRefused("reset_fault_unavailable: recovery policy changed")
		}
		r.mu.Lock()
		r.homeResetPending = true
		r.mu.Unlock()
		if err = r.resetCommand(ctx, r.cfg.AxisMask); err != nil {
			return err
		}
		r.mu.Lock()
		r.homeResetPending = false
		r.mu.Unlock()
		// Native ACK precedes the next status publication. Wait for clearance
		// and authority retirement together; never resend an acknowledged reset.
		after, err := r.resetAfter(ctx, admitted)
		if err != nil {
			return err
		}
		r.mu.Lock()
		r.setObservationLocked(&after)
		r.reason = ""
		r.nativeSeen = time.Now()
		r.program = nil
		if r.session != nil {
			r.session.State = "stopped"
			r.session.StopReason = "reset_fault"
			r.session.LastError = ""
		}
		r.mu.Unlock()
		return nil
	})
	result := &ArmResult{OK: true, Accepted: err == nil, Status: r.Status(context.Background())}
	if err != nil {
		result.Error = err.Error()
	}
	return result, err
}

// resetAdmits admits a native reset attempt: every latched fault's recovery
// is a reset, or nothing is latched and a drive reports Fault. Native resets
// such a drive only when the drive profile classes its alarm resettable
// (rt-core 99fd3f23) and otherwise refuses fault_persists naming the alarm;
// a disarmed Rosie 1430 arm showed J2 Er41.1 with both masks 0 (2026-10-11), which
// this reset never reached.
func resetAdmits(s control.Status, reference *homeReferenceRecovery) bool {
	if latchedFault(s) {
		if resettableFaults(s) {
			return true
		}
		current := recoveredReferenceReset(s, ^uint32(0))
		return reference != nil && current != nil && current.adapter == reference.adapter &&
			current.daemon == reference.daemon && current.timeNS >= reference.timeNS && current.affected == reference.affected
	}
	return driveFaulted(s)
}

// Native owns this proof: an older server, missing proof, unresolved reference,
// new affected joint or any other non-resettable fault refuses. The additional
// mask checks bind the proof to this observation; they do not reconstruct the
// native encoder/anchor recovery decision in OLP.
func recoveredReferenceReset(s control.Status, requested uint32) *homeReferenceRecovery {
	if s.Core.Recovery == nil || requested == 0 {
		return nil
	}
	var affected, classified uint32
	for _, fault := range s.Core.Recovery.Faults {
		if fault.Bit >= 32 {
			return nil
		}
		classified |= 1 << fault.Bit
		if fault.Name == "coordinate_reference" {
			if fault.Bit != 6 || fault.Recovery != "rehome_required" || !fault.ResetPreservesHome ||
				fault.AxisMask == 0 || fault.AxisMask & ^requested != 0 || fault.RehomeAxisMask != 0 ||
				s.Core.HomeValidMask&fault.AxisMask != fault.AxisMask {
				return nil
			}
			affected |= fault.AxisMask
		} else if fault.Recovery != "reset_clears" && fault.Recovery != "reset_after_condition_clears" || fault.RehomeAxisMask != 0 {
			return nil
		}
	}
	if affected == 0 || classified != s.Core.ExecutionFaultReasons || s.Core.SafetyFaultMask&affected != affected {
		return nil
	}
	return &homeReferenceRecovery{requested: requested, affected: affected, adapter: s.AdapterIncarnation, daemon: s.DaemonIncarnation, timeNS: s.TimeNS}
}

func latchedFault(s control.Status) bool {
	return s.Core.SafetyFaultMask != 0 || s.Core.ExecutionFaultReasons != 0 || s.Core.Recovery != nil && len(s.Core.Recovery.Faults) > 0
}

// driveFaulted: some axis's drive is in Fault (native readiness "faulted").
func driveFaulted(s control.Status) bool {
	for _, axis := range s.Axes {
		if axis.Readiness == "faulted" {
			return true
		}
	}
	return false
}

// Keep the existing observation deadline and freshness ceiling. On a
// Rosie 1430 arm (2026-11-01), one full status read carried 3,927 historical handles: native
// publication age plus transfer straddled 150ms although later reads were fresh.
// An old publication is a reason to await a newer one, not to widen admission.
const resetObservationBudget = 500 * time.Millisecond
const resetObservationPoll = 25 * time.Millisecond

var errResetOldPublication = rtRefused("rt_core_status_stale: reset old publication")

// Recovery checks current execution state, never historical handle records.
// The SDK's one-pass observation decoder avoids materializing that history.
// Keep Status as the fallback for alternate clients that do not implement it.
type resetStatusObserver interface {
	StatusObservation(context.Context) (control.Status, error)
}

// resetRead uses bounded advancing publications and the adapter's same-host
// FastObservation age plus HTTP time, never cross-host clock equality. Identity,
// inhibited-state and invalid-clock refusals are terminal even on an old sample.
func (r *RTCoreController) resetRead(parent context.Context, before *control.Status) (control.Status, error) {
	ctx, cancel := context.WithTimeout(parent, resetObservationBudget)
	defer cancel()
	baseline := before
	var pending error
	read := r.c.Status
	if observer, ok := r.c.(resetStatusObserver); ok {
		read = observer.StatusObservation
	}
	for {
		started := time.Now()
		s, err := read(ctx)
		elapsed := time.Since(started)
		if ctx.Err() != nil {
			if parent.Err() != nil {
				return s, parent.Err()
			}
			if pending != nil {
				return s, pending
			}
			return s, ctx.Err()
		}
		if err != nil {
			return s, err
		}
		if baseline != nil {
			if err = resetIdentityUnchanged(*baseline, s); err != nil {
				return s, err
			}
		}
		err = resetStatusAdmission(s, r.cfg.Binding.ConfigurationSHA256, elapsed)
		if err != nil && !errors.Is(err, errResetOldPublication) {
			return s, err
		}
		if err == nil {
			if before == nil || s.TimeNS > before.TimeNS && s.Motion.TimeNS > before.Motion.TimeNS {
				return s, nil
			}
			err = rtRefused("rt_core_status_stale: reset requires advancing status")
		}
		pending = err
		if baseline == nil {
			first := s
			baseline = &first
		}
		if waitErr := waitResetObservation(ctx); waitErr != nil {
			if parent.Err() != nil {
				return s, parent.Err()
			}
			return s, err
		}
	}
}

// resetAfter accepts only a fresh post-command publication with both faults
// cleared and native authority retired. An advancing publication may still
// predate the reset receipt; allow it to catch up within one bounded window.
func (r *RTCoreController) resetAfter(parent context.Context, before control.Status) (control.Status, error) {
	ctx, cancel := context.WithTimeout(parent, resetObservationBudget)
	defer cancel()
	var pending error
	for {
		after, err := r.resetRead(ctx, &before)
		if err != nil {
			if pending != nil && parent.Err() == nil && errors.Is(err, context.DeadlineExceeded) && ctx.Err() == context.DeadlineExceeded {
				return after, pending
			}
			return after, err
		}
		if after.Core.SafetyFaultMask != 0 || after.Core.ExecutionFaultReasons != 0 {
			err = rtRefused("fault_persists: native fault remains after reset")
		} else if after.Core.LeaseValid != 0 || after.Grant == nil || after.Grant.GrantActive != 0 {
			err = rtRefused("reset_fault_cleanup_unverified: authority remains after reset")
		} else {
			return after, nil
		}
		if waitErr := waitResetObservation(ctx); waitErr != nil {
			if parent.Err() != nil {
				return after, parent.Err()
			}
			return after, err
		}
		pending = err
		before = after
	}
}

func waitResetObservation(ctx context.Context) error {
	timer := time.NewTimer(resetObservationPoll)
	defer timer.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-timer.C:
		return nil
	}
}

func resetStatusAdmission(s control.Status, pin string, elapsed time.Duration) error {
	if s.TimeNS == 0 || s.Motion.TimeNS == 0 || s.AdapterIncarnation == "" || s.DaemonIncarnation == "" || pin == "" || s.Core.ConfigurationSHA256 != pin || s.Core.ConfigurationEpoch == 0 {
		return rtRefused("rt_core_identity_mismatch: reset observation")
	}
	if s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 || s.Core.CommissioningPhase != 0 || s.Core.NativeHomeActiveMask != 0 || s.Execution.State == "executing" || (s.Motion.Mode == 2 || s.Motion.Mode == 3) && !s.Motion.Done || s.Jog != nil && s.Jog.Open != 0 {
		return rtRefused("reset_requires_inhibited: Stop and disarm before clearing faults")
	}
	// FastObservation validates this clock against the native host at read time.
	if s.Grant == nil || s.Grant.NowHostNs == 0 || elapsed > 150*time.Millisecond {
		return rtRefused("rt_core_status_stale: reset clock evidence unavailable")
	}
	old := false
	for _, stamp := range []uint64{s.TimeNS, s.Motion.TimeNS} {
		now := s.Grant.NowHostNs
		if stamp > now {
			if stamp-now > 100000000 {
				return rtRefused("rt_core_status_stale: reset future publication")
			}
		} else if now-stamp > uint64(150*time.Millisecond-elapsed) {
			old = true
		}
	}
	if old {
		return errResetOldPublication
	}
	return nil
}
func resetIdentityUnchanged(a, b control.Status) error {
	if a.AdapterIncarnation != b.AdapterIncarnation || a.DaemonIncarnation != b.DaemonIncarnation || a.Core.ConfigurationSHA256 != b.Core.ConfigurationSHA256 || a.Core.ConfigurationEpoch != b.Core.ConfigurationEpoch || a.Core.HomeEpoch != b.Core.HomeEpoch || a.Core.HomeValidMask != b.Core.HomeValidMask || len(a.Core.Axes) != len(b.Core.Axes) {
		return rtRefused("rt_core_identity_mismatch: reset changed configuration or Home")
	}
	for i, x := range a.Core.Axes {
		y := b.Core.Axes[i]
		if x.CoordinateIdentitySHA256 != y.CoordinateIdentitySHA256 || x.HomeOffset != y.HomeOffset || x.HomeValid != y.HomeValid {
			return rtRefused("rt_core_identity_mismatch: reset changed axis %d reference", i)
		}
	}
	if b.TimeNS < a.TimeNS || b.Motion.TimeNS < a.Motion.TimeNS {
		return rtRefused("rt_core_status_stale: reset publication moved backwards")
	}
	return nil
}

// A reset never preserves owned authority on a slow/unsent request. Conversely,
// a preflight or typed acquisition refusal must not Stop another controller.
func (r *RTCoreController) resetFailure(cause error) error {
	r.mu.Lock()
	owned := r.held || r.acquiring || r.fence.Session != ""
	r.mu.Unlock()
	owned = owned || r.c.Grant().Session != ""
	if !owned {
		return rtFailure(cause)
	}
	_, cleanup := r.stopObserved(cause.Error(), nil)
	ctx, cancel := context.WithTimeout(context.Background(), 500*time.Millisecond)
	defer cancel()
	s, observed := r.resetRead(ctx, nil)
	if observed == nil && (s.Core.LeaseValid != 0 || s.Grant.GrantActive != 0) {
		observed = rtRefused("reset_fault_cleanup_unverified: authority remains")
	}
	if cleanup != nil || observed != nil {
		return rtFailure(errors.Join(cause, rtRefused("reset_fault_cleanup_unverified"), cleanup, observed))
	}
	return rtFailure(cause)
}
