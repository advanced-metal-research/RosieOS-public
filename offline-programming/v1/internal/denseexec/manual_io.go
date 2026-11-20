//go:build linux

package denseexec

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"rosieos/rt-core/sdk/control"
	"strings"
	"time"
)

func (r *RTCoreController) BeginManualIO(ctx context.Context) (token string, err error) {
	err = r.operationKind(ctx, "io_test", func(ctx context.Context) error {
		state, e := rtRequest(r, ctx, "rt_core_status_slow", r.c.Status)
		if e != nil {
			return e
		}
		r.mu.Lock()
		playing := r.session != nil && r.session.State == "playing"
		r.mu.Unlock()
		if playing || state.Core.Armed != 0 || state.Core.AxisEnableMask != 0 {
			return httpError(409, "io_test_disarm_required", "Release L2 and stop the program before testing outputs")
		}
		if e = r.acquire(ctx); e != nil {
			return e
		}
		r.mu.Lock()
		fence := r.fence
		r.mu.Unlock()
		if _, e = r.c.Command(ctx, &control.Request{Operation: "io_disarm", Fence: fence}); e != nil {
			return e
		}
		var random [16]byte
		if _, e = rand.Read(random[:]); e != nil {
			return e
		}
		token = hex.EncodeToString(random[:])
		r.mu.Lock()
		r.manualToken = token
		r.manualFence = fence
		r.manualDeadline = time.Now().Add(ManualIOSetupLifetime)
		r.manualSequence = 0
		r.mu.Unlock()
		return nil
	})
	return
}

func (r *RTCoreController) ManualIO(ctx context.Context, in ManualIORequest) error {
	if len(in.Samples) > 16 || in.Session == "" || in.Sequence == 0 {
		return httpError(400, "io_test_invalid", "A test session, sequence, and at most 16 channels are required")
	}
	seen := map[string]bool{}
	for _, s := range in.Samples {
		if strings.TrimSpace(s.Channel) == "" || len(s.Channel) > 31 || seen[s.Channel] || (s.Begin && !s.Value) {
			return httpError(400, "io_test_invalid", "Each channel needs one valid explicit sample")
		}
		seen[s.Channel] = true
	}
	received := time.Now()
	return r.operationKind(ctx, "io_test", func(ctx context.Context) error {
		r.mu.Lock()
		valid := r.held && in.Session == r.manualToken && r.fence == r.manualFence && in.Sequence > r.manualSequence
		timely := received.Before(r.manualDeadline) && time.Now().Before(r.manualDeadline)
		fence := r.fence
		playing := r.session != nil && r.session.State == "playing"
		if valid {
			r.manualSequence = in.Sequence
			r.manualNativeSequence++
		}
		if valid && (!timely || in.End || playing) {
			r.manualToken = ""
		}
		nativeSequence := r.manualNativeSequence
		r.mu.Unlock()
		if !valid {
			return httpError(409, "io_test_expired", "Test session ended; enable output testing again")
		}
		if !timely || in.End || playing {
			// Remove I/O permission only; never arm/disarm motion from this path.
			if !playing {
				_, _ = r.c.Command(ctx, &control.Request{Operation: "io_disarm", Fence: fence})
			}
			if in.End {
				return nil
			}
			return httpError(409, "io_test_expired", "Test session expired; OFF requested. Enable testing again")
		}
		for _, s := range in.Samples {
			response, e := r.c.Command(ctx, &control.Request{Operation: "io_test", Fence: fence, Label: s.Channel, Value: s.Value, SourceSequence: nativeSequence, Begin: s.Begin})
			if e != nil || response.Operation != "io_test" || response.Sequence == 0 {
				r.mu.Lock()
				r.manualToken = ""
				r.mu.Unlock()
				_, _ = r.c.Command(ctx, &control.Request{Operation: "io_disarm", Fence: fence})
				if e != nil {
					return e
				}
				return control.ErrProtocol
			}
		}
		r.mu.Lock()
		r.manualDeadline = received.Add(ManualIOSetupLifetime)
		r.mu.Unlock()
		return nil
	})
}
