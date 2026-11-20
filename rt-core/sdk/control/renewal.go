//go:build linux

package control

import (
	"context"
	"errors"
	"time"

	"rosieos/rt-core/ipcclient"
)

type Renewal struct {
	Grant Grant
	Err   error
	// StartedHostMonotonicNS precedes Renew, so observers cannot mistake
	// channel delivery time for the start of the request.
	StartedHostMonotonicNS uint64
}

// StartRenewal starts one explicit lease keeper. Results use a latest-value
// channel: an unread observer cannot block renewal. stopping=true is a successful
// grant and remains visible in Grant as well as this stream. Any error terminates
// renewal, preventing an expired grant from being silently reacquired.
func (c *Client) StartRenewal(ctx context.Context, interval time.Duration) (<-chan Renewal, error) {
	if interval < 0 {
		return nil, errors.New("renewal interval must be positive")
	}
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed {
		return nil, ErrClosed
	}
	if c.renewCancel != nil {
		return nil, errors.New("renewal already started; call StopRenewal first")
	}
	if c.grant.Session == "" {
		return nil, errors.New("acquire or set a fence before renewal")
	}
	lease := time.Duration(c.grant.LeaseMS) * time.Millisecond
	// Unverified compatibility lease for SetFence: existing LAN policy.
	if lease == 0 {
		lease = 500 * time.Millisecond
	}
	if interval == 0 {
		interval = lease / 3
	}
	if interval > lease/3 {
		return nil, errors.New("renewal interval must not exceed one third of lease")
	}
	if lease < 3*c.roundTrip+interval {
		return nil, &Rejected{Reason: "control_session_stale"}
	}
	budget := lease - interval
	ctx, cancel := context.WithCancel(ctx)
	c.renewCancel, c.renewDone = cancel, make(chan struct{})
	done := c.renewDone
	updates := make(chan Renewal, 1)
	go func() {
		defer close(done)
		defer close(updates)
		ticker := time.NewTicker(interval)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
			}
			// Bound each receipt by the effective lease minus the next renewal cadence.
			callCtx, stop := context.WithTimeout(ctx, budget)
			started := ipcclient.HostMonotonicNS()
			grant, err := c.Renew(callCtx)
			if ctx.Err() != nil {
				stop()
				return
			}
			select {
			case <-updates:
			default:
			}
			updates <- Renewal{Grant: grant, Err: err, StartedHostMonotonicNS: started}
			if err != nil {
				stop()
				return
			}
			stop()
		}
	}()
	return updates, nil
}

// StopRenewal cancels and joins the producer. It does not relinquish authority,
// so Release can stop the keeper before sending its terminal fence operation.
func (c *Client) StopRenewal() {
	c.mu.Lock()
	cancel, done := c.renewCancel, c.renewDone
	c.mu.Unlock()
	if cancel == nil {
		return
	}
	cancel()
	<-done
	c.mu.Lock()
	if c.renewDone == done {
		c.renewCancel, c.renewDone = nil, nil
	}
	c.mu.Unlock()
}
