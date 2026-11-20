//go:build linux

package control

import (
	"context"
	"errors"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Capture-aware producers preserve input age across Begin and Update, so a
// delayed transport cannot manufacture a new input lifetime.
func TestSDKLocalJogPreservesCaptureTime(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	s.enable(t)
	origin := ipcclient.HostMonotonicNS() - 30_000_000
	deadline := origin + 150_000_000
	ctx := context.Background()
	jog, err := s.client.NewJogSessionAt(ctx, 1, origin, deadline)
	must(t, err)
	defer jog.End(ctx)
	must(t, jog.UpdateAt([]float64{.001}, origin, deadline))
	var observed JogObservation
	poll(t, time.Second, func() bool {
		observed, err = s.client.JogStatus(ctx)
		return err == nil && observed.SourceSequence == 1
	})
	if observed.ObservedOriginHostNs != origin || observed.InputDeadlineHostNs != deadline {
		t.Fatalf("capture was restamped: %+v, want %d/%d", observed, origin, deadline)
	}
}

// Capture-aware samples must reject equal or regressing origins before send,
// preventing an increasing sequence from disguising older producer input.
func TestSDKCaptureMonotonicity(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	s.enable(t)
	ctx := context.Background()
	origin := ipcclient.HostMonotonicNS()
	jog, err := s.client.NewJogSessionAt(ctx, 511, origin, origin+150_000_000)
	must(t, err)
	defer jog.End(ctx)
	must(t, jog.UpdateAt(make([]float64, 9), origin, origin+150_000_000))
	for _, capture := range []uint64{origin - 10_000_000, origin} {
		err = jog.UpdateAt(make([]float64, 9), capture, capture+150_000_000)
		var rejected *Rejected
		if !errors.As(err, &rejected) || rejected.Reason != "jog_invalid_timing" {
			t.Fatalf("capture=%d after %d: want typed jog_invalid_timing, got %v", capture, origin, err)
		}
	}
	next := ipcclient.HostMonotonicNS()
	must(t, jog.UpdateAt(make([]float64, 9), next, next+150_000_000))
	poll(t, time.Second, func() bool {
		o, err := s.client.JogStatus(ctx)
		return err == nil && o.ObservedOriginHostNs == next
	})
	o, err := s.client.JogStatus(ctx)
	must(t, err)
	if o.SourceSequence != 2 || o.ObservedSourceSequence != 2 {
		t.Fatalf("rejected capture advanced the sequence: %+v", o)
	}
	ingress, err := s.client.JogIngress(ctx)
	must(t, err)
	if ingress.Refused != 0 {
		t.Fatalf("SDK sent invalid capture to native ingress: %+v", ingress)
	}
}
