//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"
)

// Published input age controls native frames and local retirement together;
// the LAN fallback cannot expire an internet hold. (Cartesian path holds:
// TestCartesianPathHoldUsesNegotiatedLifetime750.)
func TestJogPublishedInputLifetime750(t *testing.T) {
	r, _, lane, joint := jointFake(t)
	r.cancel()
	<-r.done
	r.mu.Lock()
	r.description.MaxJogInputAgeNS = 750000000
	r.mu.Unlock()
	if _, err := r.JointJog(context.Background(), "begin", joint); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	sample := lane.samples[0]
	lane.mu.Unlock()
	if sample.deadline-sample.origin != 750000000 {
		t.Fatalf("input age_ns=%d", sample.deadline-sample.origin)
	}
	r.mu.Lock()
	h := r.jog
	last := h.last
	h.timer.Stop()
	r.mu.Unlock()
	r.expireJointJog(h, last.Add(749999999))
	if !r.JointJogState().Active {
		t.Fatal("hold expired before 750000000 ns")
	}
	r.expireJointJog(h, last.Add(750000000))
	if s := r.JointJogState(); s.Active || s.Revision != 1 || s.Reason != "jog_input_deadline_expired" {
		t.Fatal(s)
	}
}

func TestCartesianPathHoldUsesNegotiatedLifetime750(t *testing.T) {
	r, _, lane, _, in := pathFake(t)
	r.mu.Lock()
	r.description.MaxJogInputAgeNS = 750000000
	r.mu.Unlock()
	var lifetime time.Duration
	r.pathOpen = func(_ context.Context, _ uint32, _, _ float64, l time.Duration) (pathJogLane, error) {
		lifetime = l
		return lane, nil
	}
	if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
		t.Fatal(err)
	}
	// Every speed deadline is the cell's published input age after the intent.
	if lifetime != 750*time.Millisecond {
		t.Fatalf("lifetime %v", lifetime)
	}
	if _, err := r.CartesianJog(context.Background(), "halt", in); err != nil {
		t.Fatal(err)
	}
}
