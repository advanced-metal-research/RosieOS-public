//go:build linux

package control

import (
	"context"
	"errors"
	"sync"
	"time"
)

// ErrRequestTimeout identifies an SDK-expired attempt whose socket was closed.
// Caller cancellation and transport header timeouts do not authorize a replay.
var ErrRequestTimeout = errors.New("rt_core_request_timeout")

func deadlineExpired(ctx context.Context) bool {
	deadline, ok := ctx.Deadline()
	return ok && !time.Now().Before(deadline)
}

// Unverified discovery budget: 5000 ms before a warm RTT has been measured.
const discoveryCallBudget = 5 * time.Second

// Unverified synchronous Home allowance: the adapter's 60000 ms commissioning
// watchdog plus one negotiated call budget for its terminal receipt.
const homeCommissioningBudget = 60 * time.Second

// Unverified upload throughput floor: 256 KiB/s. Measured 2026-09-16 on the LA
// bench over the Sydney relay (240 ms RTT): a 2.5 MB prepare_trajectory body
// completed in about 2 s (about 1.2 MB/s); the floor keeps a 4x margin so a
// slow link lengthens the budget instead of expiring a request the adapter is
// still decoding.
const uploadThroughputFloor = 256 << 10

// UploadAllowance is the extra request time a body of the given size earns at
// the throughput floor, added to the negotiated call budget for bulk uploads.
func UploadAllowance(bodyBytes int) time.Duration {
	if bodyBytes <= 0 {
		return 0
	}
	return time.Duration(float64(bodyBytes) / uploadThroughputFloor * float64(time.Second))
}

func (c *Client) commandBudget(operation string, bodyBytes int) time.Duration {
	budget := c.requestBudget()
	switch operation {
	case "home":
		return homeCommissioningBudget + budget
	case "prepare_trajectory":
		return budget + UploadAllowance(bodyBytes)
	}
	return budget
}

// requestBudget bounds connection establishment, headers and body by the
// negotiated lease plus three warm round trips, preventing half-open reads.
func (c *Client) requestBudget() time.Duration {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.roundTrip == 0 {
		return discoveryCallBudget
	}
	lease := time.Duration(c.grant.LeaseMS) * time.Millisecond
	if lease == 0 {
		lease = c.leaseCeiling
	}
	// Unverified legacy lease fallback: 500 ms, matching acquisition policy.
	if lease == 0 {
		lease = 500 * time.Millisecond
	}
	return TimingForLease(lease, c.roundTrip).CallBudget
}

// contextMutex allows a cancelled reservation to leave maintenance immediately;
// a plain Mutex would retain an Arm even after all its request deadlines expired.
type contextMutex struct {
	once sync.Once
	gate chan struct{}
}

func (m *contextMutex) LockContext(ctx context.Context) error {
	m.once.Do(func() { m.gate = make(chan struct{}, 1) })
	if err := ctx.Err(); err != nil {
		return err
	}
	select {
	case m.gate <- struct{}{}:
		return nil
	case <-ctx.Done():
		return ctx.Err()
	}
}

func (m *contextMutex) Lock()   { _ = m.LockContext(context.Background()) }
func (m *contextMutex) Unlock() { <-m.gate }
