//go:build linux

package control

import (
	"context"
	"fmt"
	adapter "rosieos/rt-core/adapters/rosie/control"
	"time"
)

// LeaseTiming keeps renewal and observation budgets tied to measured transport
// and the effective grant; it cannot increase the cell's authority ceiling.
type LeaseTiming struct {
	RoundTrip, Lease, RenewInterval, CallBudget, StatusAge time.Duration
}

// LeaseTimingRejected preserves the typed session refusal and reports the
// measured warm RTT and cell ceiling, preventing opaque timing refusals.
type LeaseTimingRejected struct {
	*Rejected
	RoundTrip, Ceiling time.Duration
}

func (e *LeaseTimingRejected) Error() string {
	return fmt.Sprintf("%s: measured_warm_rtt_ns=%d lease_ceiling_ns=%d", e.Reason, e.RoundTrip, e.Ceiling)
}

func (e *LeaseTimingRejected) Unwrap() error { return e.Rejected }

// TimingForLease preserves the effective lease and derives a cadence and receipt
// budget. Callers must reject leases below three round trips plus that cadence.
func TimingForLease(lease, roundTrip time.Duration) LeaseTiming {
	return LeaseTiming{RoundTrip: roundTrip, Lease: lease, RenewInterval: lease / 3,
		CallBudget: lease + 3*roundTrip, StatusAge: lease + 2*roundTrip}
}

// Unverified LAN RTT bound: 20 ms; slower links reserve the cell's jitter budget.
const lanRoundTripBound = 20 * time.Millisecond

// RequestedLease requests the ceiling on remote links so connection jitter does
// not consume a minimally sized lease. LAN retains its capped 500 ms floor.
func RequestedLease(roundTrip, ceiling time.Duration) time.Duration {
	if roundTrip > lanRoundTripBound && ceiling > 0 {
		return ceiling
	}
	lease := (9*roundTrip + 1) / 2
	if lease < 500*time.Millisecond {
		lease = 500 * time.Millisecond
	}
	lease = ((lease + time.Millisecond - 1) / time.Millisecond) * time.Millisecond
	if ceiling > 0 && lease > ceiling {
		lease = ceiling
	}
	return lease
}

// Timing returns the most recent warm Describe measurement and effective receipt,
// preventing consumers from substituting the advertised ceiling for their lease.
func (c *Client) Timing() LeaseTiming {
	c.mu.Lock()
	defer c.mu.Unlock()
	return TimingForLease(time.Duration(c.grant.LeaseMS)*time.Millisecond, c.roundTrip)
}

// AcquireLease measures Describe when needed and requests a bounded lease.
// A positive requested value may increase the measured requirement, never lower
// it. An inadequate effective receipt is refused typed before motion is usable.
func (c *Client) AcquireLease(ctx context.Context, controller string, binding Binding, requested time.Duration) (Grant, error) {
	c.mu.Lock()
	measured := c.roundTrip
	c.mu.Unlock()
	if measured == 0 {
		if _, err := c.Describe(ctx); err != nil {
			return Grant{}, err
		}
	}
	c.mu.Lock()
	measured, ceiling := c.roundTrip, c.leaseCeiling
	c.mu.Unlock()
	// Unverified compatibility ceiling for older descriptions: LAN's 500 ms.
	if ceiling == 0 {
		ceiling = 500 * time.Millisecond
	}
	if ceiling < 3*measured+ceiling/3 {
		return Grant{}, &LeaseTimingRejected{&Rejected{Reason: adapter.ReasonControlSessionStale}, measured, ceiling}
	}
	want := RequestedLease(measured, ceiling)
	if requested > want {
		want = requested
	}
	if want > ceiling {
		want = ceiling
	}
	c.mu.Lock()
	prepared := c.acquisitionPrepared
	c.acquisitionPrepared = false
	c.mu.Unlock()
	if measured > lanRoundTripBound && !prepared {
		if err := c.warmConnections(ctx, true); err != nil {
			return Grant{}, err
		}
	}
	r, err := c.Command(ctx, &Request{Operation: "acquire", Controller: controller, Binding: binding, RequestedLeaseMS: int(want / time.Millisecond)})
	g, err := data[Grant](r, err)
	if err != nil {
		return g, err
	}
	timing := TimingForLease(time.Duration(g.LeaseMS)*time.Millisecond, measured)
	if g.LeaseMS <= 0 || timing.Lease > want || timing.Lease < 3*measured+timing.RenewInterval {
		// Release uses the issued fence; failure still leaves native expiry active.
		_, _ = c.Release(ctx)
		c.mu.Lock()
		if c.grant.Session == g.Session && c.grant.Generation == g.Generation {
			c.grant = Grant{}
		}
		c.mu.Unlock()
		return Grant{}, &Rejected{Reason: adapter.ReasonControlSessionStale}
	}
	return g, nil
}
