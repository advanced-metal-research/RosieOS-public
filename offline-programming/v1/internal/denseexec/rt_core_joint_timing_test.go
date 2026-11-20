//go:build linux

package denseexec

import (
	"context"
	"math"
	"testing"
	"time"
)

type reviewEndDeadline struct {
	t     *testing.T
	calls int
}

func (s *reviewEndDeadline) End(ctx context.Context) error {
	s.calls++
	deadline, ok := ctx.Deadline()
	remaining := time.Until(deadline)
	if !ok || remaining < 3500000000 || remaining > 3750000000 {
		s.t.Fatalf("negotiated End remaining_ns=%d deadline_present=%v", remaining, ok)
	}
	return nil
}

// TestReviewJointNegotiatedTiming admits an observed remote publication within
// its negotiated age and gives cleanup its lease/RTT budget, preventing a LAN
// constant from rejecting a healthy remote joint controller after integration.
func TestReviewJointNegotiatedTiming(t *testing.T) {
	t.Run("jog", func(t *testing.T) {
		r, _, _, in := jointFake(t)
		r.cancel()
		<-r.done
		r.mu.Lock()
		r.effectiveLease, r.roundTrip = 3000000000, 250000000
		r.nativeSeen = time.Now().Add(-time.Second)
		r.mu.Unlock()
		if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
			t.Fatal(err)
		}
		if _, err := r.JointJog(context.Background(), "end", in); err != nil {
			t.Fatal(err)
		}
		lane := &reviewEndDeadline{t: t}
		if err := r.endJointJog(lane); err != nil || lane.calls != 1 {
			t.Fatalf("End calls=%d err=%v", lane.calls, err)
		}
	})
	t.Run("move", func(t *testing.T) {
		r, _, in := moveFake(t)
		r.cancel()
		<-r.done
		r.mu.Lock()
		r.effectiveLease, r.roundTrip = 3000000000, 250000000
		r.nativeSeen = time.Now().Add(-time.Second)
		r.mu.Unlock()
		if _, err := r.JointMove(context.Background(), in); err != nil {
			t.Fatal(err)
		}
	})
	t.Run("completion_overflow", func(t *testing.T) {
		r, f, in := moveFake(t)
		r.cancel()
		<-r.done
		r.mu.Lock()
		r.effectiveLease, r.roundTrip = 3000000000, 250000000
		r.description.Axes[0].CompletionTimeoutNS = math.MaxInt64
		r.mu.Unlock()
		_, err := r.JointMove(context.Background(), in)
		jointCode(t, err, "joint_move_too_many_points")
		if len(f.points) != 0 {
			t.Fatal("overflowing completion budget reached Prepare")
		}
	})
}
