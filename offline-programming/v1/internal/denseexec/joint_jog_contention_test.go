//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

func TestJointJogContendedSampleIsDroppedWithoutRenewal(t *testing.T) {
	r, _, lane, in := jointFake(t)
	ctx := context.Background()
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	h := r.jog
	last, sequence := h.last, h.sequence
	h.timer.Stop()
	r.mu.Unlock()
	lane.mu.Lock()
	lane.err = control.ErrJogWouldBlock
	lane.mu.Unlock()
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatalf("transient contention retired a valid hold: %v", err)
	}
	r.mu.Lock()
	unchanged := r.jog == h && h.last.Equal(last) && h.sequence == sequence
	r.mu.Unlock()
	if !unchanged {
		t.Fatal("unsent sample renewed input or changed generation")
	}
	// Contention must not extend the existing 100 ms browser deadman.
	r.expireJointJog(h, last.Add(100*time.Millisecond))
	if state := r.JointJogState(); state.Active || state.Reason != "jog_input_deadline_expired" {
		t.Fatalf("unsent input kept hold alive: %+v", state)
	}
}
