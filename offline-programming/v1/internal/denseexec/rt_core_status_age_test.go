//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"
)

// TestReviewPickerStatusFreshness requires the 3500 ms remote freshness bound
// and prevents identical native publications from refreshing jog readiness.
func TestReviewPickerStatusFreshness(t *testing.T) {
	for _, ageMS := range []int{3000, 4000} {
		r, f, _, in := jointFake(t)
		r.cancel()
		<-r.done
		r.mu.Lock()
		r.effectiveLease, r.roundTrip = 3000000000, 250000000
		if r.timingLocked().StatusAge != 3500000000 {
			t.Fatal("status age is not 3500 ms")
		}
		r.nativeSeen = time.Now().Add(-time.Duration(ageMS) * time.Millisecond)
		r.mu.Unlock()
		status := r.Status(context.Background())
		if status.RoundTripNS != 250000000 || status.LinkLeaseNS != 3000000000 {
			t.Fatalf("SDK timing not exposed: %+v", status)
		}
		_, err := r.JointJog(context.Background(), "begin", in)
		if ageMS == 3000 {
			if err != nil {
				t.Fatal(err)
			}
			if _, err := r.JointJog(context.Background(), "end", in); err != nil {
				t.Fatal(err)
			}
		} else {
			jointCode(t, err, "jog_not_ready")
			f.mu.Lock()
			f.status.TimeNS++
			f.status.Motion.TimeNS++
			f.mu.Unlock()
			_ = r.Status(context.Background())
			if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
				t.Fatalf("fresh picker evidence: %v", err)
			}
		}
	}
}
