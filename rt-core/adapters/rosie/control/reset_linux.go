//go:build linux

package control

import (
	"context"
	"errors"
	"fmt"
	"time"

	"rosieos/rt-core/ipcclient"
)

// FaultReset admits only authorized, inhibited, idle recovery. A submitted reset
// retires authority even on rejection, preventing accidental motion on recovery.
func (c *Controller) FaultReset(f Fence, mask uint32) (uint64, *ipcclient.CommandResult, error) {
	sequence, result, _, err := c.FaultResetRecovery(f, mask)
	return sequence, result, err
}

// FaultResetRecovery returns native per-bit outcomes only after their sequence
// matches the receipt, preventing cached status from claiming a fault cleared.
func (c *Controller) FaultResetRecovery(f Fence, mask uint32) (uint64, *ipcclient.CommandResult, *ipcclient.RecoveryStatus, error) {
	var result *ipcclient.CommandResult
	var recovery *ipcclient.RecoveryStatus
	sequence, err := c.operation(f, func(ctx context.Context) (uint64, error) {
		c.mu.Lock()
		playing := c.execution.State == "executing"
		c.mu.Unlock()
		if playing {
			return 0, errors.New(ReasonProgramAlreadyExecuting)
		}
		if err := c.waitResetInhibited(ctx); err != nil {
			c.mu.Lock()
			stale := c.authorize(f)
			c.mu.Unlock()
			if stale != nil {
				return 0, stale
			}
			return 0, err
		}
		// Exclude renewal across native revocation and local session retirement.
		// Urgent Stop still cancels ctx and submits without either operation lock.
		c.authorityMu.Lock()
		defer c.authorityMu.Unlock()
		c.mu.Lock()
		err := c.authorize(f)
		c.mu.Unlock()
		if err != nil {
			return 0, err
		}
		sequence, err := c.core.FaultReset(ctx, mask)
		if err != nil {
			return sequence, err
		}
		native, err := c.core.WaitResult(ctx, sequence)
		if native.Sequence == sequence {
			result = &native
		}
		// Require the native extension; an absent correlated publication cannot
		// support a claim that the submitted reset changed any fault.
		if result != nil {
			var observationErr error
			recovery, observationErr = c.waitRecovery(ctx, sequence)
			if observationErr != nil {
				err = observationErr
			} else if recovery.Reason == ReasonFaultPersists {
				err = errors.New(ReasonFaultPersists)
				// Name the joint and drive alarm that stayed faulted; the
				// code stays the refusal's prefix for callers that parse it.
				if recovery.Message != "" {
					err = fmt.Errorf("%s: %s", ReasonFaultPersists, recovery.Message)
				}
			}
		}
		c.mu.Lock()
		defer c.mu.Unlock()
		if stale := c.authorize(f); stale != nil {
			return sequence, stale
		}
		c.cancel()
		c.grant.Session = ""
		c.program = nil
		c.execution = Execution{State: "cancelled", NativeSequence: sequence}
		if err != nil {
			c.execution.State = "faulted"
			c.execution.Error = err.Error()
		}
		return sequence, err
	})
	return sequence, result, recovery, err
}

// RecoveryStatus is an authenticated, inhibited, idle observation of the native
// matrix. It never resets faults, revokes a grant or changes Home evidence.
func (c *Controller) RecoveryStatus(f Fence) (*ipcclient.RecoveryStatus, error) {
	var recovery *ipcclient.RecoveryStatus
	_, err := c.operation(f, func(ctx context.Context) (uint64, error) {
		if err := c.waitResetInhibited(ctx); err != nil {
			return 0, err
		}
		var err error
		recovery, err = c.waitRecovery(ctx, 0)
		return 0, err
	})
	return recovery, err
}

// Require a matching native observation; elapsed time alone proves no recovery.
func (c *Controller) waitRecovery(parent context.Context, sequence uint64) (*ipcclient.RecoveryStatus, error) {
	// Unverified observation budget: 1 second; failure reports no recovery claim.
	ctx, cancel := context.WithTimeout(parent, time.Second)
	defer cancel()
	// Unverified polling interval: 5 ms; success requires a native observation.
	ticker := time.NewTicker(5 * time.Millisecond)
	defer ticker.Stop()
	for {
		status, err := c.core.Status()
		if err == nil && status.Recovery != nil && (sequence == 0 || status.Recovery.ResetSequence == sequence) {
			return status.Recovery, nil
		}
		select {
		case <-ctx.Done():
			return nil, errors.New(ReasonRecoveryObservationUnavailable)
		case <-ticker.C:
		}
	}
}

// Observe after taking operationMu so a preceding accepted Arm/Jog/Start cannot
// be hidden by cached disarmed metrics. Missing observations fail closed.
func (c *Controller) waitResetInhibited(ctx context.Context) error {
	after := ipcclient.HostMonotonicNS()
	// Unverified admission observation budget: 1 second; never authorizes motion.
	ctx, cancel := context.WithTimeout(ctx, time.Second)
	defer cancel()
	// Unverified polling interval: 5 ms; admission requires timestamped evidence.
	ticker := time.NewTicker(5 * time.Millisecond)
	defer ticker.Stop()
	for {
		state, stateErr := c.core.Status()
		motion, motionErr := c.core.Motion()
		if stateErr == nil && motionErr == nil && state.TimeNS >= after && motion.TimeNS >= after {
			// Native mode 2 is trajectory, mode 3 is jog (include/ipc_v1.hpp).
			if motion.Mode == 2 && !motion.Done {
				return errors.New(ReasonProgramAlreadyExecuting)
			}
			if state.Armed != 0 || (motion.Mode == 3 && !motion.Done) ||
				state.CommissioningPhase != 0 || state.NativeHomeActiveMask != 0 {
				return errors.New(ReasonResetRequiresInhibited)
			}
			return nil
		}
		select {
		case <-ctx.Done():
			return errors.New(ReasonResetRequiresInhibited)
		case <-ticker.C:
		}
	}
}
