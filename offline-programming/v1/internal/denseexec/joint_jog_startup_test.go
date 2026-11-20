//go:build linux

package denseexec

import (
	"context"
	"math"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type delayedJointPrimeClient struct {
	*fakeRTCore
	entered, release               chan struct{}
	once                           sync.Once
	reads, fullReads, currentReads atomic.Int32
}

func (c *delayedJointPrimeClient) read(ctx context.Context, current bool) (control.Status, error) {
	if current {
		c.currentReads.Add(1)
	} else {
		c.fullReads.Add(1)
	}
	if c.reads.Add(1) == 1 {
		close(c.entered)
		select {
		case <-c.release:
		case <-ctx.Done():
			return control.Status{}, ctx.Err()
		}
	}
	s, err := c.fakeRTCore.Status(ctx)
	if current {
		s.Execution.Handles = nil
	}
	return s, err
}

func (c *delayedJointPrimeClient) Status(ctx context.Context) (control.Status, error) {
	return c.read(ctx, false)
}

func (c *delayedJointPrimeClient) StatusObservation(ctx context.Context) (control.Status, error) {
	return c.read(ctx, true)
}

func (c *delayedJointPrimeClient) unblock() { c.once.Do(func() { close(c.release) }) }

func jointStartupFixture(t *testing.T) (*RTCoreController, *fakeRTCore, *fakeJointLane, *delayedJointPrimeClient, JointJogIntent) {
	t.Helper()
	r, f, lane, in := jointFake(t)
	// Make the startup Status read deterministic without a background observer
	// consuming its gate. The armed/current fixture remains valid for 500 ms.
	r.cancel()
	<-r.done
	r.cfg.CartesianModel = "rosie_1430"
	r.mu.Lock()
	r.description.MaxJogInputAgeNS = uint64(250 * time.Millisecond)
	r.nativeSeen = time.Now()
	r.mu.Unlock()
	f.mu.Lock()
	f.description.MaxJogInputAgeNS = uint64(250 * time.Millisecond)
	for i := uint64(1); i <= 4000; i++ {
		f.status.Execution.Handles = append(f.status.Execution.Handles, control.HandleRecord{Handle: i, State: "retired"})
	}
	f.mu.Unlock()
	c := &delayedJointPrimeClient{fakeRTCore: f, entered: make(chan struct{}), release: make(chan struct{})}
	r.c = c
	t.Cleanup(c.unblock)
	in.MaxFraction, in.InputSequence = 0.25, 1
	return r, f, lane, c, in
}

func beginBlockedJoint(t *testing.T, r *RTCoreController, c *delayedJointPrimeClient, ctx context.Context, in JointJogIntent) <-chan error {
	t.Helper()
	done := make(chan error, 1)
	go func() { _, err := r.JointJog(ctx, "begin", in); done <- err }()
	select {
	case <-c.entered:
	case <-time.After(2 * time.Second):
		t.Fatal("Begin did not reach stationary prime")
	}
	return done
}

func assertNoJointMotion(t *testing.T, lane *fakeJointLane) {
	t.Helper()
	lane.mu.Lock()
	defer lane.mu.Unlock()
	for _, sample := range lane.samples {
		for _, v := range sample.velocities {
			if v != 0 {
				t.Fatalf("preparation/release sent motion: %+v", sample)
			}
		}
	}
}

func TestJointJogPreparingAcceptsFreshPipelinedIntents(t *testing.T) {
	r, _, lane, c, in := jointStartupFixture(t)
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	done := beginBlockedJoint(t, r, c, ctx, in)
	// Observed the 1430 cell setup took 354–401 ms, beyond its 250 ms input lifetime.
	// The Deck continues supplying input every 20 ms during that preparation.
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	for i := 0; i < 18; i++ {
		<-ticker.C
		in.InputSequence++
		in.Fraction = 0.11 + float64(i)*0.005
		if _, err := r.JointJog(ctx, "update", in); err != nil {
			t.Fatalf("fresh preparation intent %d refused: %v", in.InputSequence, err)
		}
	}
	assertNoJointMotion(t, lane)
	r.mu.Lock()
	lastCaptured := r.jog.last
	r.mu.Unlock()
	lateStamp, lateOrigin := time.Now(), uint64(0)
	r.mu.Lock()
	lateOrigin = r.jog.preparing.origin + 1
	r.mu.Unlock()
	c.unblock()
	if err := <-done; err != nil {
		t.Fatalf("fresh preparation failed: %v", err)
	}
	assertNoJointMotion(t, lane)
	r.mu.Lock()
	if !r.jog.last.Equal(lastCaptured) {
		r.mu.Unlock()
		t.Fatal("readiness renewed the last input timestamp")
	}
	r.mu.Unlock()
	// A request already captured before ready may arrive behind the final
	// preparation lock. It must be dropped rather than sent with an old clock.
	late := in
	late.InputSequence++
	if err := r.jointJog(ctx, "update", late, lateStamp, lateOrigin); err != nil {
		t.Fatal(err)
	}
	assertNoJointMotion(t, lane)
	if c.currentReads.Load() != 1 || c.fullReads.Load() != 0 {
		t.Fatalf("prime decoded historical handles: current=%d full=%d", c.currentReads.Load(), c.fullReads.Load())
	}
	// Prepared joint lanes still require a capture after readiness; this fresh
	// input retains its capture timestamp and exactly the published lifetime.
	in.InputSequence++
	in.Fraction = 0.2
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	samples := append([]jointSample(nil), lane.samples...)
	lane.mu.Unlock()
	if len(samples) != 1 || math.Abs(samples[0].velocities[0]-0.3) > 1e-12 || samples[0].deadline-samples[0].origin != uint64(250*time.Millisecond) {
		t.Fatalf("fresh post-ready input lost its speed or original deadline: %+v", samples)
	}
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
}

func TestJointJogPreparingExpiredGapIsNotRenewed(t *testing.T) {
	r, _, lane, c, in := jointStartupFixture(t)
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	done := beginBlockedJoint(t, r, c, ctx, in)
	time.Sleep(275 * time.Millisecond)
	in.InputSequence++
	_, err := r.JointJog(ctx, "update", in)
	jointCode(t, err, "jog_input_deadline_expired")
	c.unblock()
	if err := <-done; err == nil {
		t.Fatal("new input revived an expired preparing generation")
	}
	assertNoJointMotion(t, lane)
	if r.JointJogState().Active {
		t.Fatal("expired gap left a pending hold")
	}
}

func TestJointJogPreparingDropsOvertakenAndConflictingIntents(t *testing.T) {
	r, _, lane, c, in := jointStartupFixture(t)
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()
	done := beginBlockedJoint(t, r, c, ctx, in)
	in.InputSequence = 2
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	last, origin := r.jog.last, r.jog.preparing.origin
	r.mu.Unlock()
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	newerSequence := in
	newerSequence.InputSequence = 3
	if err := r.jointJog(ctx, "update", newerSequence, last.Add(time.Millisecond), origin-1); err != nil {
		t.Fatal(err)
	}
	conflict := newerSequence
	conflict.Axis = 1
	_, err := r.JointJog(ctx, "update", conflict)
	jointCode(t, err, "jog_mode_conflict")
	r.mu.Lock()
	unchanged := r.jog != nil && r.jog.last.Equal(last) && r.jog.preparing.origin == origin && r.jog.inputSequence == 2 && r.jog.intent.Axis == 0
	r.mu.Unlock()
	if !unchanged {
		t.Fatal("overtaken/conflicting preparation input changed the hold or deadline")
	}
	if _, err := r.JointJog(context.Background(), "end", in); err != nil {
		t.Fatal(err)
	}
	if err := <-done; err == nil {
		t.Fatal("ended preparation succeeded")
	}
	assertNoJointMotion(t, lane)
}

func TestJointJogPreparationDoesNotRenewExpiredBegin(t *testing.T) {
	r, _, lane, c, in := jointStartupFixture(t)
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	done := beginBlockedJoint(t, r, c, ctx, in)
	// No fresh operator input arrives. Readiness cannot manufacture another
	// 250 ms of authorization from an expired press.
	time.Sleep(300 * time.Millisecond)
	c.unblock()
	jointCode(t, <-done, "jog_input_deadline_expired")
	assertNoJointMotion(t, lane)
	if r.JointJogState().Active {
		t.Fatal("expired preparation left an active hold")
	}
}

type acknowledgingJointPrimeLane struct{ *fakeJointLane }

func (s *acknowledgingJointPrimeLane) UpdateAt(v []float64, origin, deadline uint64) error {
	if err := s.fakeJointLane.UpdateAt(v, origin, deadline); err != nil {
		return err
	}
	s.f.mu.Lock()
	s.f.status.Jog.HasInput = 1
	s.f.mu.Unlock()
	return nil
}

func TestJointJogStationaryPrimeRetainsNativeAcknowledgement(t *testing.T) {
	r, f, lane, c, in := jointStartupFixture(t)
	c.unblock()
	f.mu.Lock()
	f.status.Jog = &control.JogObservation{JogGeneration: 17, AppGrantGeneration: r.fence.Generation, Open: 1}
	f.status.Grant = &ipcclient.GrantObservation{GrantActive: 1, AppGrantGeneration: r.fence.Generation}
	f.mu.Unlock()
	open := r.jogOpen
	r.jogOpen = func(ctx context.Context, mask uint32, origin uint64) (jointJogLane, error) {
		_, err := open(ctx, mask, origin)
		return &acknowledgingJointPrimeLane{lane}, err
	}
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	assertNoJointMotion(t, lane)
	if c.currentReads.Load() != 2 || c.fullReads.Load() != 0 {
		t.Fatalf("prime did not observe native acknowledgement through current status: current=%d full=%d", c.currentReads.Load(), c.fullReads.Load())
	}
	lane.mu.Lock()
	count := len(lane.samples)
	lane.mu.Unlock()
	if count != 1 || r.JointJogState().Sequence != 1 {
		t.Fatal("native stationary prime was skipped or repeated")
	}
	if _, err := r.JointJog(context.Background(), "end", in); err != nil {
		t.Fatal(err)
	}
}

func TestJointJogPreparingReleaseCancelsWithoutLateMotion(t *testing.T) {
	for _, action := range []string{"end", "stop", "disarm", "cancel"} {
		t.Run(action, func(t *testing.T) {
			r, f, lane, c, in := jointStartupFixture(t)
			ctx, cancel := context.WithCancel(context.Background())
			defer cancel()
			done := beginBlockedJoint(t, r, c, ctx, in)
			in.InputSequence++
			if _, err := r.JointJog(ctx, "update", in); err != nil {
				t.Fatal(err)
			}
			switch action {
			case "end":
				_, _ = r.JointJog(context.Background(), "end", in)
			case "stop":
				_, _ = r.Stop(context.Background())
			case "disarm":
				_, _ = r.Arm(context.Background(), false)
			case "cancel":
				cancel()
			}
			select {
			case err := <-done:
				if err == nil {
					t.Fatal("released preparation succeeded")
				}
			case <-time.After(2 * time.Second):
				t.Fatal("release could not cancel preparation")
			}
			c.unblock()
			assertNoJointMotion(t, lane)
			if r.JointJogState().Active {
				t.Fatal("release retained pending jog")
			}
			if _, err := r.JointJog(context.Background(), "update", in); err == nil {
				t.Fatal("late input revived released preparation")
			}
			f.mu.Lock()
			defer f.mu.Unlock()
			arms := 0
			for _, call := range f.calls {
				if call == "arm" {
					arms++
				}
			}
			if arms != 1 {
				t.Fatalf("preparation changed arm ownership: %v", f.calls)
			}
		})
	}
}
