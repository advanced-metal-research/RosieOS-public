//go:build linux

package control

import (
	"context"
	"errors"
	"time"

	"rosieos/rt-core/ipcclient"
)

type haltNative interface {
	Halt(context.Context, uint64) (uint64, error)
}

func (n *restartNative) Halt(ctx context.Context, generation uint64) (uint64, error) {
	n.mu.RLock()
	defer n.mu.RUnlock()
	return n.client.Halt(ctx, generation)
}

// Halt retires the active source after native admission while retaining the
// grant. Stop revokes independently and can never wait for the ramp to finish.
func (c *Controller) Halt(f Fence) (uint64, error) {
	c.operationMu.Lock()
	defer c.operationMu.Unlock()
	c.mu.Lock()
	if err := c.restartFenceLocked(f); err != nil {
		c.mu.Unlock()
		return 0, err
	}
	var refusal string
	switch {
	case c.grant.Session == "" || f.Session != c.grant.Session || !time.Now().Before(c.expires):
		refusal = ReasonNoGrant
	case f.Generation != c.grant.Generation:
		refusal = ReasonWrongGeneration
	case c.stopping:
		refusal = ReasonInhibited
	}
	ctx := c.sessionCtx
	c.mu.Unlock()
	if refusal != "" {
		return 0, errors.New(refusal)
	}
	native, ok := c.core.(haltNative)
	if !ok {
		return 0, errors.New(ReasonCapabilityUnimplemented)
	}
	sequence, err := native.Halt(ctx, f.Generation)
	if err != nil {
		var rejected *ipcclient.RejectedError
		if errors.As(err, &rejected) {
			reasons := map[uint32]string{ipcclient.ReasonNoGrant: ReasonNoGrant,
				ipcclient.ReasonWrongGeneration: ReasonWrongGeneration, ipcclient.ReasonInhibited: ReasonInhibited}
			if reason := reasons[rejected.Result.Reason]; reason != "" {
				return sequence, &publicRejection{reason, err}
			}
		}
		return sequence, err
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if err := c.authorize(f); err != nil {
		return sequence, err
	}
	c.localJog = nil
	for _, handle := range c.handles {
		if handle.State == HandleStarted {
			transitionHandle(handle, HandleRetired)
		}
	}
	if c.execution.State == "executing" {
		c.execution.State = "halted"
		c.program = nil
	}
	return sequence, nil
}
