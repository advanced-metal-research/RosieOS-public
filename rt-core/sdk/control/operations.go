//go:build linux

package control

import (
	"context"
	"errors"
	"net/http"
	"time"
)

// Describe measures a warm second read on first discovery. Later identity reads
// retain that measurement, so queued UI polls cannot inflate lease qualification.
func (c *Client) Describe(ctx context.Context) (description Description, resultErr error) {
	ctx, cancel := context.WithTimeout(ctx, 2*c.requestBudget())
	defer cancel()
	// Keep both calibration reads on one reserved socket. LAN observations use
	// the other slots too, so a slow Home cannot serialize status behind it.
	slots := c.warmLanes[c.control]
	var pool *http.Client
	select {
	case pool = <-slots:
	case <-ctx.Done():
		return Description{}, ctx.Err()
	}
	defer func() {
		c.mu.Lock()
		broken := c.warmReady && (errors.Is(resultErr, context.DeadlineExceeded) || errors.Is(resultErr, ErrTransport))
		if broken {
			c.warmBroken[pool] = slots
		}
		c.mu.Unlock()
		if !broken {
			slots <- pool
		}
		if resultErr == nil {
			c.startKeepAlive()
		}
	}()
	// This HTTP client view already owns its slot; call must not borrow it twice.
	reserved := &http.Client{Transport: pool.Transport, CheckRedirect: pool.CheckRedirect}
	read := func() (Description, error) {
		response, err := c.call(ctx, reserved, "GET", "/v1/describe", nil, nil, true)
		return data[Description](response, err)
	}
	c.mu.Lock()
	measured := c.roundTrip != 0
	c.mu.Unlock()
	if !measured {
		if _, err := read(); err != nil {
			return Description{}, err
		}
	}
	started := time.Now()
	d, err := read()
	if err == nil {
		// Unverified absolute protocol ceilings: 10 s lease, 2 s input age.
		if d.MaxGrantLeaseNS > 10000000000 || d.MaxGrantLeaseNS%1000000 != 0 || d.MaxJogInputAgeNS > 2000000000 {
			return Description{}, ErrProtocol
		}
		// Unverified idle contract bounds: 3 ms through 90000 ms; reject overflow
		// and unusable intervals before starting any reservation maintenance.
		if d.ControlIdleTimeoutNS != 0 && (d.ControlIdleTimeoutNS < 3000000 || d.ControlIdleTimeoutNS > 90000000000) {
			return Description{}, ErrProtocol
		}
		c.mu.Lock()
		if !measured {
			c.roundTrip = time.Since(started)
		}
		c.leaseCeiling = time.Duration(d.MaxGrantLeaseNS)
		c.maxJogInputAge = d.MaxJogInputAgeNS
		if d.ControlIdleTimeoutNS != 0 {
			if c.idleTimeout != 0 && c.idleTimeout != time.Duration(d.ControlIdleTimeoutNS) {
				c.wakeKeepAlive()
			}
			c.idleTimeout = time.Duration(d.ControlIdleTimeoutNS)
		}
		c.mu.Unlock()
	}
	return d, err
}
func (c *Client) Status(ctx context.Context) (Status, error) {
	return get[Status](c, ctx, "/v1/status")
}
func (c *Client) JogClock(ctx context.Context) (JogClock, error) {
	return get[JogClock](c, ctx, "/v1/jog/clock")
}
func (c *Client) JogStatus(ctx context.Context) (JogObservation, error) {
	return get[JogObservation](c, ctx, "/v1/jog")
}
func (c *Client) JogIngress(ctx context.Context) (JogIngressObservation, error) {
	return get[JogIngressObservation](c, ctx, "/v1/jog/ingress")
}

// Maintenance reads the servo host's maintenance notice as rt-control enforces
// it (acquire and motion starts refused busy while active) and the current
// grant's holder by name and principal. It needs no session and never renews.
func (c *Client) Maintenance(ctx context.Context) (MaintenanceObservation, error) {
	return get[MaintenanceObservation](c, ctx, "/v1/maintenance")
}
func (c *Client) Acquire(ctx context.Context, controller string, binding Binding) (Grant, error) {
	return c.AcquireLease(ctx, controller, binding, 0)
}
func (c *Client) Renew(ctx context.Context) (Grant, error) {
	r, e := c.Command(ctx, &Request{Operation: "renew"})
	return data[Grant](r, e)
}
func (c *Client) Release(ctx context.Context) (Grant, error) {
	c.StopRenewal()
	r, e := c.Command(ctx, &Request{Operation: "release"})
	return data[Grant](r, e)
}
func (c *Client) Stop(ctx context.Context) (Grant, error) {
	r, e := c.Command(ctx, &Request{Operation: "stop"})
	return data[Grant](r, e)
}
func (c *Client) Enable(ctx context.Context, mask uint32) (Response, error) {
	return c.Command(ctx, &Request{Operation: "enable", AxisMask: mask})
}
func (c *Client) Arm(ctx context.Context) (Response, error) {
	return c.Command(ctx, &Request{Operation: "arm"})
}

// IOArm grants fenced output permission; its receipt does not establish physical feedback.
func (c *Client) IOArm(ctx context.Context) (Response, error) {
	return c.Command(ctx, &Request{Operation: "io_arm"})
}

// IODisarm clears output permission and intent in the native cycle.
func (c *Client) IODisarm(ctx context.Context) (Response, error) {
	return c.Command(ctx, &Request{Operation: "io_disarm"})
}
func (c *Client) Home(ctx context.Context, mask uint32) (Response, error) {
	return c.Command(ctx, &Request{Operation: "home", AxisMask: mask})
}

// RestoreAnchor decodes observed recovery so a sequence-only reply cannot
// establish restored Home evidence.
func (c *Client) RestoreAnchor(ctx context.Context, mask uint32) (RecoveryStatus, error) {
	r, e := c.Command(ctx, &Request{Operation: "restore_anchor", AxisMask: mask})
	return data[RecoveryStatus](r, e)
}
func (c *Client) ResetFault(ctx context.Context, mask uint32) (Response, error) {
	return c.Command(ctx, &Request{Operation: "reset_fault", AxisMask: mask})
}
func (c *Client) Jog(ctx context.Context, mask uint32, velocity []float64, timeoutMS uint32) (Response, error) {
	return c.Command(ctx, &Request{Operation: "jog", AxisMask: mask, Velocity: velocity, TimeoutMS: timeoutMS})
}
func (c *Client) BeginJog(ctx context.Context, mask uint32, clock string, originNS, deadlineNS uint64) (Response, error) {
	return c.Command(ctx, &Request{Operation: "begin_jog", AxisMask: mask, ClockIncarnation: clock, SourceOriginHostNS: originNS, DeadlineHostNS: deadlineNS})
}
func (c *Client) EndJog(ctx context.Context, generation uint64) (Response, error) {
	return c.Command(ctx, &Request{Operation: "end_jog", JogGeneration: generation})
}
func (c *Client) PrepareTrajectory(ctx context.Context, mask uint32, points []Point) (Response, error) {
	return c.Command(ctx, &Request{Operation: "prepare_trajectory", AxisMask: mask, Points: points})
}
func (c *Client) StartTrajectory(ctx context.Context, handle uint64) (Response, error) {
	return c.Command(ctx, &Request{Operation: "start_trajectory", Handle: handle})
}
func (c *Client) DiscardTrajectory(ctx context.Context, handle uint64) (Response, error) {
	return c.Command(ctx, &Request{Operation: "discard_trajectory", Handle: handle})
}
func (c *Client) StartProgram(ctx context.Context, identity Identity) (Response, error) {
	return c.Command(ctx, &Request{Operation: "start_program", Identity: identity})
}

// RestoreReferenceVerifiedDelta re-establishes a saved HM35 (v3) anchor under
// the explicitly compiled absolute-delta policy; it never defines a new zero.
func (c *Client) RestoreReferenceVerifiedDelta(ctx context.Context, mask uint32) (VerifiedDeltaRestoreReceipt, error) {
	r, e := c.Command(ctx, &Request{Operation: "restore_reference_verified_delta", AxisMask: mask})
	receipt, err := data[VerifiedDeltaRestoreReceipt](r, e)
	if err != nil {
		return receipt, err
	}
	v := receipt.AnchorDeltaRestore
	if r.Sequence == 0 || v == nil || !v.Valid || v.Method != "independent_absolute_delta_v1" || v.AnchorVersion != 3 || v.BoundRestoreCommandSequence != r.Sequence {
		return receipt, ErrProtocol
	}
	return receipt, nil
}
