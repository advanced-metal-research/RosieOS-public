//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"rosieos/rt-core/internal/simfixture"
	"runtime"
	"sync/atomic"
	"testing"
	"time"
)

func TestJogObservationWriterCollision(t *testing.T) {
	previous := runtime.GOMAXPROCS(1)
	defer runtime.GOMAXPROCS(previous)
	view, storage := sharedTestStorage(t, JogObservationBytes/8)
	c := &Client{done: make(chan struct{}), fast: &applicationGrantRegion{
		header: FastHeader{ConnectionId: 7, ConfiguredMask: 1}, jogObservation: view,
	}}
	observation := JogObservation{ConnectionId: 7, NativeGeneration: 1,
		ConfigurationEpoch: 1, HomeEpoch: 1, ClockMappingGeneration: 1, NowHostNs: HostMonotonicNS()}
	encoded := EncodeJogObservation(observation)
	words := make([]uint64, JogObservationBytes/8)
	sharedWordsFromLE(words, encoded[:])
	atomic.StoreUint64(&storage[0], 1)
	done := make(chan struct{})
	go func() {
		for i, word := range words {
			atomic.StoreUint64(&storage[i+1], word)
		}
		atomic.StoreUint64(&storage[0], 2)
		close(done)
	}()
	got, err := c.JogObservation()
	<-done
	if err != nil || got != observation {
		t.Fatalf("writer collision: %+v, %v", got, err)
	}
	atomic.StoreUint64(&storage[0], 3) // A stalled writer cannot return the prior snapshot.
	if _, err := c.JogObservation(); !errors.Is(err, ErrJogUnavailable) {
		t.Fatalf("stalled writer: %v", err)
	}
	observation.NowHostNs = 1
	encoded = EncodeJogObservation(observation)
	sharedWordsFromLE(words, encoded[:])
	atomic.StoreUint64(&storage[0], 4)
	view.publish(words)
	if _, err := c.JogObservation(); !errors.Is(err, ErrJogUnavailable) {
		t.Fatalf("stale observation: %v", err)
	}
}

func waitJog(t *testing.T, c *Client, predicate func(JogObservation) bool) JogObservation {
	t.Helper()
	deadline := time.Now().Add(time.Second)
	var pollResult JogObservation
	if !simfixture.PollUntil(deadline, time.Millisecond, func() bool {
		observation, err := c.JogObservation()
		if err == nil && predicate(observation) {
			pollResult = observation
			return true
		}
		if time.Now().After(deadline) {
			t.Fatalf("jog observation: %+v, %v", observation, err)
		}
		return false
	}) {
		t.Fatal("observation deadline exceeded")
	}
	return pollResult
}

func readyLocalJog(t *testing.T) (*Client, GrantIdentity) {
	t.Helper()
	c := applicationGrantNativeClient(t)
	identity := GrantIdentity{CapabilityID: [16]byte{0x6a, 0x6f, 0x67}, Generation: 1}
	if _, err := c.MirrorAcquire(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
		t.Fatal(err)
	}
	arm(t, c, 3)
	readyAfter := HostMonotonicNS()
	wait(t, c, func(s Status) bool { return s.TimeNS >= readyAfter && allOperationEnabled(s) })
	if _, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
		t.Fatal(err)
	}
	return c, identity
}

func TestLocalJogMovesWhileCommandWriterLocked(t *testing.T) {
	c, identity := readyLocalJog(t)
	// This is the exact mutex held across bulk point publication. Leave it
	// locked until Begin, submitted motion and End have all completed.
	c.commandMu.Lock()
	defer c.commandMu.Unlock()
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	origin := HostMonotonicNS()
	session, err := c.BeginLocalJog(ctx, identity, 3, origin, origin+uint64(200*time.Millisecond))
	if err != nil {
		t.Fatal(err)
	}
	origin = HostMonotonicNS()
	if err := c.UpdateLocalJog(ctx, session, 1, origin, origin+uint64(200*time.Millisecond), 3, []float64{.05, .05}); err != nil {
		t.Fatal(err)
	}
	admitted := waitJog(t, c, func(o JogObservation) bool { return o.Open == 1 && o.HasInput == 1 && o.SourceSequence == 1 })
	if admitted.InputDeadlineHostNs > origin+uint64(200*time.Millisecond) {
		t.Fatal("publication extended the producer deadline")
	}
	moving := wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts >= 10 && s.Axes[1].PositionCounts <= -10 })
	if moving.Armed != 1 || moving.ExecutionFaultReasons != 0 {
		t.Fatalf("jog did not drive the healthy simulated plant: %+v", moving)
	}
	if err := c.EndLocalJog(ctx, session); err != nil {
		t.Fatal(err)
	}
	waitJog(t, c, func(o JogObservation) bool { return o.Open == 0 && o.ControlAccepted == 1 })
	closed := waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonClosed })
	wait(t, c, func(s Status) bool { return s.TimeNS >= closed.NowHostNs && s.Armed == 1 })
}

func TestLocalJogExpiryCannotRestartFromLateUpdate(t *testing.T) {
	c, identity := readyLocalJog(t)
	ctx := context.Background()
	origin := HostMonotonicNS()
	session, err := c.BeginLocalJog(ctx, identity, 3, origin, origin+uint64(200*time.Millisecond))
	if err != nil {
		t.Fatal(err)
	}
	origin = HostMonotonicNS()
	if err := c.UpdateLocalJog(ctx, session, 1, origin, origin+uint64(70*time.Millisecond), 3, []float64{.02, .02}); err != nil {
		t.Fatal(err)
	}
	waitJog(t, c, func(o JogObservation) bool { return o.Open == 1 && o.SourceSequence == 1 })
	expired := waitJog(t, c, func(o JogObservation) bool { return o.Open == 0 && o.NowHostNs >= origin+uint64(70*time.Millisecond) })
	later := HostMonotonicNS()
	if err := c.UpdateLocalJog(ctx, session, 2, later, later+uint64(100*time.Millisecond), 3, []float64{.02, .02}); err != nil {
		t.Fatal(err)
	}
	rejected := waitJog(t, c, func(o JogObservation) bool { return o.UpdateReason == JogReasonClosed })
	if rejected.Open != 0 || rejected.HasInput != 0 || rejected.SourceSequence != expired.SourceSequence {
		t.Fatalf("late frame reopened expired jog: %+v", rejected)
	}
	waitJog(t, c, func(o JogObservation) bool { return o.StateReason == JogReasonExpired })
	if _, err := c.MirrorRenew(ctx, identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
		t.Fatal(err)
	}
	origin = HostMonotonicNS()
	fresh, err := c.BeginLocalJog(ctx, identity, 3, origin, origin+uint64(100*time.Millisecond))
	if err != nil || fresh.Identity.JogGeneration <= session.Identity.JogGeneration {
		t.Fatalf("fresh Begin after qualified hold: %+v %v", fresh, err)
	}
}

func TestLocalJogReservesModeAgainstPlanStart(t *testing.T) {
	c, identity := readyLocalJog(t)
	ctx := context.Background()
	handle, err := c.PrepareTrajectory(ctx, 9971, 3, []Point{{Position: []float64{0, 0}}, {Time: time.Second, Position: []float64{.02, .02}}})
	if err != nil {
		t.Fatal(err)
	}
	origin := HostMonotonicNS()
	session, err := c.BeginLocalJog(ctx, identity, 3, origin, origin+uint64(200*time.Millisecond))
	if err != nil {
		t.Fatal(err)
	}
	_, err = c.StartTrajectory(ctx, handle)
	var rejected *RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonModeConflict {
		t.Fatalf("Start ignored reserved jog: %v", err)
	}
	if err := c.EndLocalJog(ctx, session); err != nil {
		t.Fatal(err)
	}
}

// Publish a generated native frame without client admission to prove the RT
// owner independently rejects a stale or expired adapter publication.
func publishNativeJogTest(t *testing.T, c *Client, session LocalJogSession, sequence, origin, deadline uint64) {
	t.Helper()
	id := session.Identity
	frame := JogUpdate{ConnId: id.ConnectionId, NativeGeneration: id.NativeGeneration, CapabilityId: id.CapabilityId,
		AppGrantGeneration: id.AppGrantGeneration, ConfigEpoch: id.ConfigurationEpoch, HomeEpoch: id.HomeEpoch,
		ClockMappingGeneration: id.ClockMappingGeneration, JogGeneration: id.JogGeneration, SourceSequence: sequence,
		EarliestOriginHostNs: origin, ImmutableDeadlineHostNs: deadline, AxisMask: 3, Velocity: [16]float64{.05, .05}}
	encoded := EncodeJogUpdate(frame)
	var words [JogUpdateBytes / 8]uint64
	if sharedWordsFromLE(words[:], encoded[:]) != sharedWordsOK {
		t.Fatal("encode native jog")
	}
	c.jogPublishMu.Lock()
	defer c.jogPublishMu.Unlock()
	c.fastMu.RLock()
	defer c.fastMu.RUnlock()
	if _, status := c.fast.jogUpdate.publish(words[:]); status != sharedWordsOK {
		t.Fatal(status)
	}
}

func TestNativeExpiredJogUpdateNeverApplies(t *testing.T) {
	c, identity := readyLocalJog(t)
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(context.Background(), identity, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	before := wait(t, c, allOperationEnabled)
	now = HostMonotonicNS()
	publishNativeJogTest(t, c, session, 1, now-10_000_000, now-1_000_000)
	o := waitJog(t, c, func(o JogObservation) bool { return o.ObservedSourceSequence == 1 })
	if o.UpdateReason != JogReasonExpired || o.HasInput != 0 || o.SourceSequence != 0 {
		t.Fatalf("expired frame: %+v", o)
	}
	until := HostMonotonicNS() + 250_000_000
	for {
		state := wait(t, c, func(Status) bool { return true })
		if state.Axes[0].PositionCounts != before.Axes[0].PositionCounts || state.Axes[1].PositionCounts != before.Axes[1].PositionCounts {
			t.Fatal("expired native frame moved feedback")
		}
		if state.TimeNS >= until {
			break
		}
		time.Sleep(time.Millisecond)
	}
}

func TestNativeStopClosesJogAndFencesLateUpdates(t *testing.T) {
	c, identity := readyLocalJog(t)
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(context.Background(), identity, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	now = HostMonotonicNS()
	if err := c.UpdateLocalJog(context.Background(), session, 1, now, now+200_000_000, 3, []float64{.05, .05}); err != nil {
		t.Fatal(err)
	}
	wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts >= 10 })
	if _, err := c.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	waitJog(t, c, func(o JogObservation) bool { return o.Open == 0 })
	now = HostMonotonicNS()
	publishNativeJogTest(t, c, session, 2, now, now+100_000_000)
	o := waitJog(t, c, func(o JogObservation) bool { return o.ObservedSourceSequence == 2 })
	if o.Open != 0 || o.HasInput != 0 || (o.UpdateReason != JogReasonClosed && o.UpdateReason != JogReasonInvalidGeneration) {
		t.Fatalf("late frame: %+v", o)
	}
	wait(t, c, func(s Status) bool { return s.TimeNS >= now && s.Armed == 0 && s.AxisEnableMask == 0 })
}

func TestLocalJogAndLegacyJogExcludeEachOther(t *testing.T) {
	c, identity := readyLocalJog(t)
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(context.Background(), identity, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	sequence, err := c.Jog(context.Background(), 3, []float64{.01, .01}, 200*time.Millisecond)
	if err != nil {
		t.Fatal(err)
	}
	result, err := c.WaitResult(context.Background(), sequence)
	if err == nil || result.Reason != ReasonModeConflict {
		t.Fatalf("legacy bypassed reservation: %+v %v", result, err)
	}
	if err := c.EndLocalJog(context.Background(), session); err != nil {
		t.Fatal(err)
	}
	arm(t, c, 3)
	readyAfter := HostMonotonicNS()
	wait(t, c, func(s Status) bool { return s.TimeNS >= readyAfter && allOperationEnabled(s) })
	if _, err := c.MirrorRenew(context.Background(), identity, HostMonotonicNS()+applicationGrantMaxLease); err != nil {
		t.Fatal(err)
	}
	sequence, err = c.Jog(context.Background(), 3, []float64{.01, .01}, 200*time.Millisecond)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := c.WaitResult(context.Background(), sequence); err != nil {
		t.Fatal(err)
	}
	// Acquire/Renew already returned the native fence used by Begin. Hold the
	// separately published grant observation mid-write: it must not mask the
	// native legacy-mode verdict with a shared-slot transport refusal.
	busyGrant, busyStorage := sharedTestStorage(t, GrantObservationBytes/8)
	atomic.StoreUint64(&busyStorage[0], 1)
	c.fastMu.Lock()
	grantObservation := c.fast.observation
	c.fast.observation = busyGrant
	c.fastMu.Unlock()
	now = HostMonotonicNS()
	_, err = c.BeginLocalJog(context.Background(), identity, 3, now, now+100_000_000)
	c.fastMu.Lock()
	c.fast.observation = grantObservation
	c.fastMu.Unlock()
	var rejected *JogRejectedError
	if !errors.As(err, &rejected) || rejected.Reason != JogReasonModeConflict {
		t.Fatalf("Begin bypassed legacy mode: %v", err)
	}
}

func TestNativeHomeClosesOpenJog(t *testing.T) {
	c, identity := readyLocalJog(t)
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(context.Background(), identity, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	sequence, err := c.Home(context.Background(), 3)
	if err != nil {
		t.Fatal(err)
	}
	// This fixture has no Home descriptor. The refused procedure must still
	// close jog ownership and fence input before reporting its native failure.
	result, err := c.WaitResult(context.Background(), sequence)
	if err == nil || result.Result != ResultRejected || result.Reason != ReasonNotReady {
		t.Fatalf("unconfigured Home: %+v %v", result, err)
	}
	o := waitJog(t, c, func(o JogObservation) bool { return o.Open == 0 })
	if o.JogGeneration != session.Identity.JogGeneration || o.HasInput != 0 || o.HomeEpoch <= session.Identity.HomeEpoch {
		t.Fatalf("Home left input: %+v", o)
	}
}

func TestNativeGrantExpiryClosesJog(t *testing.T) {
	c, identity := readyLocalJog(t)
	now := HostMonotonicNS()
	receipt, err := c.MirrorRenew(context.Background(), identity, now+150_000_000)
	if err != nil {
		t.Fatal(err)
	}
	session, err := c.BeginLocalJog(context.Background(), identity, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	now = HostMonotonicNS()
	if err := c.UpdateLocalJog(context.Background(), session, 1, now, now+250_000_000, 3, []float64{.05, .05}); err != nil {
		t.Fatal(err)
	}
	wait(t, c, func(s Status) bool { return s.Axes[0].PositionCounts >= 10 })
	o := waitJog(t, c, func(o JogObservation) bool {
		grant, err := c.FastObservation()
		return err == nil && grant.GrantActive == 0 && o.Open == 0 && o.NowHostNs >= receipt.DeadlineNS
	})
	if o.HasInput != 0 {
		t.Fatalf("grant expiry retained input: %+v", o)
	}
	wait(t, c, func(s Status) bool { return s.TimeNS >= o.NowHostNs && s.Armed == 0 && s.AxisEnableMask == 0 })
	now = HostMonotonicNS()
	publishNativeJogTest(t, c, session, 2, now, now+100_000_000)
	o = waitJog(t, c, func(o JogObservation) bool { return o.ObservedSourceSequence == 2 })
	if o.UpdateReason != JogReasonClosed || o.Open != 0 {
		t.Fatalf("grant-expired input: %+v", o)
	}
}
