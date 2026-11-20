//go:build linux

package control

import (
	"context"
	"time"
)

// Unverified legacy adapter idle bound: 30000 ms; maintenance uses one third.
const legacyControlIdleTimeout = 30 * time.Second

func (c *Client) keepAliveIntervalLocked() time.Duration {
	idle := c.idleTimeout
	if idle == 0 {
		idle = legacyControlIdleTimeout
	}
	return idle / 3
}

// startKeepAlive binds reservations to successful discovery until Close,
// preventing an idle selected cell from depending on grant renewal for repair.
func (c *Client) startKeepAlive() {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.closed || c.keepCancel != nil {
		return
	}
	ctx, cancel := context.WithCancel(context.Background())
	c.keepCancel, c.keepDone = cancel, make(chan struct{})
	go func() {
		defer close(c.keepDone)
		c.keepAlive(ctx)
	}()
}

// keepAlive consumes cadence or repair notifications independently of authority.
// Busy lanes remain with their operation; maintenance never renews a grant.
func (c *Client) keepAlive(ctx context.Context) {
	for {
		c.mu.Lock()
		interval := c.keepAliveIntervalLocked()
		c.mu.Unlock()
		timer := time.NewTimer(interval)
		select {
		case <-ctx.Done():
			timer.Stop()
			return
		case <-timer.C:
		case <-c.keepWake:
		}
		timer.Stop()
		maintenance, cancel := context.WithTimeout(ctx, interval)
		_ = c.warmConnections(maintenance, false)
		cancel()
	}
}

func (c *Client) wakeKeepAlive() {
	select {
	case c.keepWake <- struct{}{}:
	default:
	}
}

func (c *Client) stopKeepAlive() {
	c.mu.Lock()
	cancel, done := c.keepCancel, c.keepDone
	c.mu.Unlock()
	if cancel != nil {
		cancel()
		<-done
	}
}
