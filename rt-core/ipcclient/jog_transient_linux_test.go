//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// the 1430 cell 2026-10-10 22:01:13: one path speed frame met another writer of the
// fast-control region and came back as an untyped error, which rt-control
// reported as not_ready and the SDK treated as the end of the hold. A
// collision must be typed transient, must publish nothing, and the same
// frame must publish once the other writer is done.
func TestLocalJogPublisherCollisionIsTransient(t *testing.T) {
	c, identity := readyLocalJog(t)
	ctx := context.Background()
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(ctx, identity, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	origin := HostMonotonicNS()
	deadline := origin + 200_000_000
	for _, collision := range []struct {
		name string
		lock *sync.Mutex
		want error
	}{
		// Begin/End, path chunks and grant requests publish under fastPublishMu.
		{"fast region writer", &c.fastPublishMu, ErrFastPublisherBusy},
		{"second jog producer", &c.jogPublishMu, ErrJogPublisherBusy},
	} {
		collision.lock.Lock()
		err := c.UpdateLocalJog(ctx, session, 1, origin, deadline, 3, []float64{.02, .02})
		collision.lock.Unlock()
		if !errors.Is(err, collision.want) || !TransientJogRefusal(err) {
			t.Fatalf("%s: %v, want transient %v", collision.name, err, collision.want)
		}
	}
	if o, err := c.JogObservation(); err != nil || o.ObservedSourceSequence != 0 {
		t.Fatalf("a refused collision reached native: %+v %v", o, err)
	}
	if err := c.UpdateLocalJog(ctx, session, 1, origin, deadline, 3, []float64{.02, .02}); err != nil {
		t.Fatalf("the same frame after the collision: %v", err)
	}
	o := waitJog(t, c, func(o JogObservation) bool { return o.ObservedSourceSequence == 1 })
	if o.UpdateReason != JogReasonNone || o.HasInput != 1 {
		t.Fatalf("native did not admit the frame after the collision: %+v", o)
	}
	// Input that is invalid or has expired, and lost authority, are verdicts and never transient.
	if err := c.UpdateLocalJog(ctx, session, 2, origin, origin, 3, []float64{.02, .02}); err == nil || TransientJogRefusal(err) {
		t.Fatalf("invalid timing classified transient: %v", err)
	}
	stale := session
	stale.Identity.AppGrantGeneration++
	later := HostMonotonicNS()
	if err := c.UpdateLocalJog(ctx, stale, 2, later, later+100_000_000, 3, []float64{.02, .02}); !errors.Is(err, ErrRevoked) || TransientJogRefusal(err) {
		t.Fatalf("lost authority: %v", err)
	}
	if err := c.EndLocalJog(ctx, session); err != nil {
		t.Fatal(err)
	}
}

// Only a writer caught mid-publication on every bounded attempt is transient;
// a stale or invalid observation stays a plain unavailability.
func TestObservationBusyIsTransientStaleIsNot(t *testing.T) {
	jogView, jogStorage := sharedTestStorage(t, JogObservationBytes/8)
	grantView, grantStorage := sharedTestStorage(t, GrantObservationBytes/8)
	c := &Client{done: make(chan struct{}), fast: &applicationGrantRegion{
		header: FastHeader{ConnectionId: 7, ConfiguredMask: 1}, jogObservation: jogView, observation: grantView,
	}}
	jog := JogObservation{ConnectionId: 7, NativeGeneration: 1, ConfigurationEpoch: 1, HomeEpoch: 1,
		ClockMappingGeneration: 1, NowHostNs: HostMonotonicNS()}
	grant := GrantObservation{ConnectionId: 7, NativeGeneration: 1, ConfigurationEpoch: 1, HomeEpoch: 1,
		NowHostNs: HostMonotonicNS()}
	publish := func(view sharedWords, encoded []byte) {
		words := make([]uint64, len(encoded)/8)
		sharedWordsFromLE(words, encoded)
		view.publish(words)
	}
	encodedJog := EncodeJogObservation(jog)
	encodedGrant := EncodeGrantObservation(grant)
	publish(jogView, encodedJog[:])
	publish(grantView, encodedGrant[:])
	if _, err := c.JogObservation(); err != nil {
		t.Fatal(err)
	}
	if _, err := c.FastObservation(); err != nil {
		t.Fatal(err)
	}
	// A writer stalled mid-publication (odd sequence) on every attempt.
	jogSequence, grantSequence := atomic.LoadUint64(&jogStorage[0]), atomic.LoadUint64(&grantStorage[0])
	atomic.StoreUint64(&jogStorage[0], jogSequence+1)
	atomic.StoreUint64(&grantStorage[0], grantSequence+1)
	if _, err := c.JogObservation(); !errors.Is(err, ErrJogUnavailable) || !TransientJogRefusal(err) {
		t.Fatalf("busy jog observation: %v", err)
	}
	if _, err := c.FastObservation(); !errors.Is(err, ErrFastObservationUnavailable) || !TransientJogRefusal(err) {
		t.Fatalf("busy grant observation: %v", err)
	}
	atomic.StoreUint64(&jogStorage[0], jogSequence+2)
	atomic.StoreUint64(&grantStorage[0], grantSequence+2)
	// A published but stale observation (native not publishing) is not transient.
	jog.NowHostNs, grant.NowHostNs = HostMonotonicNS()-uint64(time.Second), HostMonotonicNS()-uint64(time.Second)
	encodedJog, encodedGrant = EncodeJogObservation(jog), EncodeGrantObservation(grant)
	publish(jogView, encodedJog[:])
	publish(grantView, encodedGrant[:])
	if _, err := c.JogObservation(); !errors.Is(err, ErrJogUnavailable) || TransientJogRefusal(err) {
		t.Fatalf("stale jog observation: %v", err)
	}
	if _, err := c.FastObservation(); !errors.Is(err, ErrFastObservationUnavailable) || TransientJogRefusal(err) {
		t.Fatalf("stale grant observation: %v", err)
	}
	for _, err := range []error{ErrJogUnavailable, ErrFastObservationUnavailable, ErrRevoked, ErrClosed,
		&JogRejectedError{Reason: JogReasonNotReady}, context.DeadlineExceeded, nil} {
		if TransientJogRefusal(err) {
			t.Fatalf("%v classified transient", err)
		}
	}
}

func TestLocalJogUpdateUsesAdmittedGrantReceipt(t *testing.T) {
	c, identity := readyLocalJog(t)
	now := HostMonotonicNS()
	session, err := c.BeginLocalJog(context.Background(), identity, 3, now, now+200_000_000)
	if err != nil {
		t.Fatal(err)
	}
	busy, storage := sharedTestStorage(t, GrantObservationBytes/8)
	atomic.StoreUint64(&storage[0], 1)
	c.fastMu.Lock()
	observation := c.fast.observation
	c.fast.observation = busy
	c.fastMu.Unlock()
	origin := HostMonotonicNS()
	err = c.UpdateLocalJog(context.Background(), session, 1, origin, origin+100_000_000, 3, []float64{.01, .01})
	c.fastMu.Lock()
	c.fast.observation = observation
	c.fastMu.Unlock()
	if err != nil {
		t.Fatalf("admitted grant was replaced by a contended snapshot: %v", err)
	}
	o := waitJog(t, c, func(o JogObservation) bool { return o.ObservedSourceSequence == 1 })
	if o.UpdateReason != JogReasonNone {
		t.Fatalf("native rejected admitted update: %+v", o)
	}
}
