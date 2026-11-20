//go:build linux

package denseexec

import (
	"context"
	"errors"
	"rosieos/rt-core/sdk/control"
)

// rtAuthorityFailure identifies a keeper failure, so a renewal deadline still
// retires authority while an unrelated request deadline preserves the grant.
type rtAuthorityFailure struct{ error }

func (e *rtAuthorityFailure) Unwrap() error { return e.error }

// slowRequest separates request-channel uncertainty from the keeper's proof of
// authority loss. Refused native fences and protocol faults remain failures.
func slowRequest(err error) bool {
	var authority *rtAuthorityFailure
	return !errors.As(err, &authority) && (errors.Is(err, context.DeadlineExceeded) || errors.Is(err, control.ErrWarmConnection) || errors.Is(err, control.ErrTransport))
}

// rtRequest gives a deadline one warm retry under the caller's cancellation.
// Mutations retain the same request ID and fence in the supplied closure; a
// slow read cannot revoke a grant that the independent keeper still renews.
func rtRequest[T any](r *RTCoreController, ctx context.Context, note string, call func(context.Context) (T, error)) (T, error) {
	for attempt := 0; ; attempt++ {
		request, cancel := context.WithTimeout(ctx, r.callBudget())
		value, err := call(request)
		cancel()
		if errors.Is(err, control.ErrConnectionUnsent) {
			r.mu.Lock()
			r.statusNote = "rt_core_connection_reopened"
			r.mu.Unlock()
			return value, err
		}
		if !errors.Is(err, context.DeadlineExceeded) {
			return value, err
		}
		r.mu.Lock()
		r.statusNote = note
		r.mu.Unlock()
		if attempt == 1 || ctx.Err() != nil {
			return value, err
		}
	}
}
