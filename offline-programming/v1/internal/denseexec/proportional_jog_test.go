//go:build linux

package denseexec

import (
	"context"
	"math"
	"testing"
	"time"
)

// Port the one-lane contract from e58f391cb's JSLOP X experiment to
// today's joint/path lanes, including reordered and queued requests.
func TestProportionalJointKeepsLaneAndDropsOldSpeed(t *testing.T) {
	r, f, lane, in := jointFake(t)
	in.MaxFraction, in.InputSequence = .6, 1
	ctx := context.Background()
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	generation := r.JointJogState().Generation
	old := in
	for i, fraction := range []float64{.25, .6, .005} {
		in.Fraction, in.InputSequence = fraction, uint64(i+2)
		if _, err := r.JointJog(ctx, "update", in); err != nil {
			t.Fatal(err)
		}
		if r.JointJogState().Generation != generation {
			t.Fatal("speed restarted lane")
		}
		lane.mu.Lock()
		got := lane.samples[len(lane.samples)-1].velocities
		lane.mu.Unlock()
		if math.Abs(got[0]-fraction*1.5) > 1e-12 {
			t.Fatalf("velocity %v at fraction %g", got, fraction)
		}
		for _, v := range got[1:] {
			if v != 0 {
				t.Fatal("another joint moved", got)
			}
		}
		if i == 1 {
			old = in
		}
	}
	r.mu.Lock()
	last, sequence := r.jog.last, r.jog.sequence
	r.mu.Unlock()
	if _, err := r.JointJog(ctx, "update", old); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	if r.jog.last != last || r.jog.sequence != sequence {
		t.Fatal("old speed renewed input")
	}
	r.mu.Unlock()
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
	_, err := r.JointJog(ctx, "update", in)
	jointCode(t, err, "jog_session_stale")
	begins, ends := 0, 0
	for _, call := range jointCalls(f) {
		if call == "begin_jog" {
			begins++
		}
		if call == "end_jog" {
			ends++
		}
	}
	if begins != 1 || ends != 1 {
		t.Fatalf("begins/ends %d/%d", begins, ends)
	}
}

func TestProportionalJointPinsModeAndCeiling(t *testing.T) {
	for _, change := range []string{"ceiling", "direction", "axis", "legacy"} {
		t.Run(change, func(t *testing.T) {
			r, _, _, in := jointFake(t)
			in.MaxFraction, in.InputSequence = .5, 1
			if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
				t.Fatal(err)
			}
			in.InputSequence++
			switch change {
			case "ceiling":
				in.MaxFraction = .6
			case "direction":
				in.Direction = -1
			case "axis":
				in.Axis = 1
			case "legacy":
				in.MaxFraction, in.InputSequence = 0, 0
			}
			_, err := r.JointJog(context.Background(), "update", in)
			jointCode(t, err, "jog_mode_conflict")
		})
	}
	for _, fraction := range []float64{0, -.1, .6, math.NaN(), math.Inf(1)} {
		r, _, _, in := jointFake(t)
		in.MaxFraction, in.InputSequence, in.Fraction = .5, 1, fraction
		_, err := r.JointJog(context.Background(), "begin", in)
		jointCode(t, err, "jog_invalid_vector")
	}
}

func TestProportionalPathDropsReorderedSpeed(t *testing.T) {
	r, _, lane, _, in := pathFake(t)
	in.MaxFraction, in.InputSequence = .8, 1
	ctx := context.Background()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	old := in
	in.Fraction, in.InputSequence = .01, 3
	if _, err := r.CartesianJog(ctx, "intent", in); err != nil {
		t.Fatal(err)
	}
	_, speeds, _ := lane.snapshot()
	if math.Abs(speeds[len(speeds)-1].speed-.004) > 1e-12 {
		t.Fatal(speeds)
	}
	old.InputSequence = 2
	if _, err := r.CartesianJog(ctx, "intent", old); err != nil {
		t.Fatal(err)
	}
	_, after, _ := lane.snapshot()
	if len(after) != len(speeds) {
		t.Fatal("old path speed replayed")
	}
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
}

func TestProportionalQueuedJointStartsAtLatestSpeed(t *testing.T) {
	q := queueJointFake(t)
	q.in.MaxFraction, q.in.InputSequence = .8, 1
	first := q.in
	result := make(chan error, 1)
	go func() { _, err := q.r.JointJog(context.Background(), "begin", first); result <- err }()
	waitFor(t, "queued proportional press", func() bool { return q.r.JointJogState().Queued })
	in := first
	in.Fraction, in.InputSequence = .02, 3
	if _, err := q.r.JointJog(context.Background(), "update", in); err != nil {
		t.Fatal(err)
	}
	old := first
	old.InputSequence = 2
	if _, err := q.r.JointJog(context.Background(), "update", old); err != nil {
		t.Fatal(err)
	}
	q.setRamping(false)
	if err := <-result; err != nil {
		t.Fatal(err)
	}
	q.lane.mu.Lock()
	got := q.lane.samples[len(q.lane.samples)-1].velocities[0]
	q.lane.mu.Unlock()
	if math.Abs(got-.03) > 1e-12 {
		t.Fatalf("queued speed = %g, want .03 rad/s", got)
	}
	q.r.mu.Lock()
	if q.r.jog.inputSequence != 3 {
		t.Fatal("queued sequence lost")
	}
	// Expiry still applies without any fresh input.
	h := q.r.jog
	last := h.last
	q.r.mu.Unlock()
	q.r.expireJointJog(h, last.Add(time.Second))
	if q.r.JointJogState().Active {
		t.Fatal("proportional hold outlived deadman")
	}
}

func TestProportionalStopBoundUsesCeiling(t *testing.T) {
	r, _, _, in := jointFake(t)
	armJogLimits(r)
	r.mu.Lock()
	defer r.mu.Unlock()
	in.Fraction = .5
	full, _ := r.jogStopBoundLocked(&jointJogHold{intent: in})
	in.MaxFraction, in.Fraction = .5, .005
	slow, _ := r.jogStopBoundLocked(&jointJogHold{intent: in})
	if slow != full {
		t.Fatalf("decelerating hold shortened stop bound %v -> %v", full, slow)
	}
}
