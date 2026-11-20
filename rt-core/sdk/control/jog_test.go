//go:build linux

package control

import (
	"context"
	"errors"
	"math"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

func TestSDKLocalJogUsesGeneratedFramesAndEnds(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	s.enable(t)
	session, err := s.client.NewJogSession(context.Background(), 511, ipcclient.HostMonotonicNS()+200_000_000)
	must(t, err)
	velocity := make([]float64, 9)
	velocity[0] = .001
	must(t, session.Update(velocity, ipcclient.HostMonotonicNS()+200_000_000))
	poll(t, 100*time.Millisecond, func() bool {
		observation, err := s.client.JogIngress(context.Background())
		return err == nil && observation.SourceSequence == 1 && observation.Reason == 0
	})
	_, err = s.client.JogStatus(context.Background())
	must(t, err)
	must(t, session.End(context.Background()))
	if !errors.Is(session.Update(velocity, ipcclient.HostMonotonicNS()+200_000_000), ErrClosed) {
		t.Fatal("closed session accepted input")
	}
}

func TestSDKJogFrameRejectsInvalidInput(t *testing.T) {
	fence := Fence{Session: strings.Repeat("01", 32), Generation: 4}
	frame, err := BuildJogFrame(fence, strings.Repeat("02", 16), 5, 6, 1, []float64{.1}, 1_000_000, 2_000_000)
	must(t, err)
	decoded, ok := ipcclient.DecodeLocalJogUpdate(frame[:])
	if !ok || decoded.GrantGeneration != 4 || decoded.JogGeneration != 5 || decoded.SourceSequence != 6 || decoded.DeadlineHostNs != 2_000_000 || decoded.OriginHostNs != 1_000_000 || decoded.Velocity[0] != .1 {
		t.Fatalf("frame: %+v", decoded)
	}
	for _, velocities := range [][]float64{{math.NaN()}, {math.Inf(1)}, {.1, .2}, {}} {
		if _, err := BuildJogFrame(fence, strings.Repeat("02", 16), 5, 6, 1, velocities, 1_000_000, 2_000_000); err == nil {
			t.Fatalf("accepted %v", velocities)
		}
	}
}

// The SDK must decode identical refusal history from both generated public
// surfaces, including after acceptance; absent counter fields cannot pass as zero.
func TestSDKJogIngressRefusalStatusParity(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	s.enable(t)
	ctx := context.Background()
	session, err := s.client.NewJogSession(ctx, 511, ipcclient.HostMonotonicNS()+200_000_000)
	must(t, err)
	defer session.End(ctx)
	_, err = session.conn.Write([]byte{1})
	must(t, err)
	var refused JogIngressObservation
	// Unverified observation deadline: 2000 ms; require exactly one refusal.
	poll(t, 2*time.Second, func() bool {
		refused, err = s.client.JogIngress(ctx)
		return err == nil && refused.Refused == 1
	})
	if refused.RefusedByReason[7] != 1 || refused.LatestRefusal.Reason != 7 || refused.LatestRefusal.NowHostNS == 0 {
		t.Fatalf("SDK lost typed refusal: %+v", refused)
	}
	must(t, session.Update(make([]float64, 9), ipcclient.HostMonotonicNS()+200_000_000))
	poll(t, 2*time.Second, func() bool {
		o, err := s.client.JogIngress(ctx)
		return err == nil && o.SourceSequence == 1 && o.Reason == 0
	})
	status, err := s.client.Status(ctx)
	must(t, err)
	diagnostic, err := s.client.JogIngress(ctx)
	must(t, err)
	if status.JogIngress != diagnostic || diagnostic.Refused != 1 || diagnostic.RefusedByReason != refused.RefusedByReason || diagnostic.LatestRefusal != refused.LatestRefusal {
		t.Fatalf("SDK status/diagnostics lost refusal history: %+v / %+v", status.JogIngress, diagnostic)
	}
}
