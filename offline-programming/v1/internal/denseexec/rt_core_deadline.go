//go:build linux

package denseexec

import (
	"context"
	"sync"
	"time"
)

// Unverified Arm bound: eight call budgets, capped by the existing 30000 ms
// watchdog. The final eighth is reserved for Stop/Release cleanup.
const rtArmCallBudgets = 8

func (r *RTCoreController) armBudget() time.Duration {
	return min(rtOperationBudget, rtArmCallBudgets*r.callBudget())
}

// operationMutex bounds queued work by its caller's context, preventing expired
// Arm or discovery requests from retaining busy behind another operation.
type operationMutex struct {
	once sync.Once
	gate chan struct{}
}

func (m *operationMutex) init() { m.once.Do(func() { m.gate = make(chan struct{}, 1) }) }
func (m *operationMutex) LockContext(ctx context.Context) error {
	m.init()
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
func (m *operationMutex) Lock()   { _ = m.LockContext(context.Background()) }
func (m *operationMutex) Unlock() { <-m.gate }
func (m *operationMutex) TryLock() bool {
	m.init()
	select {
	case m.gate <- struct{}{}:
		return true
	default:
		return false
	}
}
