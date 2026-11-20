//go:build linux

package control

import (
	"context"
	"crypto/tls"
	"errors"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// A held producer must survive multiple clock-calibration lifetimes without
// widening freshness. Once input stops, calibration must not keep motion alive.
func TestRemoteJogRefreshDuringHeldInput(t *testing.T) {
	p := newPKI(t)
	addr, flags := p.remoteArgs(t)
	// Fixture-only network allowance (100 ms), leaving the calibration expiry
	// at one second. The test isolates refresh/lifetime behavior from host
	// scheduler jitter; the tighter conformance tests remain unchanged.
	flags = append(flags, "--remote-jog-max-uncertainty-ns", "100000000",
		"--remote-jog-drift-ppb", "2000000", "--remote-jog-calibration-max-age-ns", "1000000000")
	s := startService(t, flags...)
	c, err := DialTLS(addr, &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}})
	must(t, err)
	defer c.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	remoteJogReady(t, s, c, ctx)
	source := ipcclient.HostMonotonicNS
	j, err := c.NewRemoteJogSession(ctx, 511, 100_000_000, source)
	must(t, err)
	defer j.End(context.Background())
	refusal := make(chan error, 1)
	go func() { refusal <- j.Rejection(ctx) }()
	velocity := make([]float64, 9)
	velocity[0] = 0.005
	// Simulation policy expires calibration after one second. Two seconds of
	// physical-state observations must remain admitted in the same generation.
	start := time.Now()
	var latest JogObservation
	for time.Since(start) < 2*time.Second {
		select {
		case e := <-refusal:
			t.Fatalf("held input refused after %s: %v", time.Since(start), e)
		default:
		}
		if err := j.Update(velocity, source()); err != nil && !errors.Is(err, ErrJogWouldBlock) {
			t.Fatal(err)
		}
		latest, err = j.Observe(ctx)
		must(t, err)
		if time.Since(start) > 200*time.Millisecond && (latest.Open != 1 || latest.HasInput != 1) {
			t.Fatalf("held input lost after %s: %+v", time.Since(start), latest)
		}
		time.Sleep(5 * time.Millisecond)
	}
	if latest.JogGeneration != j.generation {
		t.Fatalf("refresh replaced the live jog generation: %+v", latest)
	}
	// Stop producing while the same authority stays alive. Deadline expiry
	// must close the generation; a late update may not revive it.
	poll(t, time.Second, func() bool {
		state, e := c.Status(ctx)
		return e == nil && state.Jog.Open == 0 && state.Jog.HasInput == 0
	})
	_ = j.Update(velocity, source())
	time.Sleep(30 * time.Millisecond)
	state, err := c.Status(ctx)
	must(t, err)
	if state.Jog.Open != 0 || state.Jog.HasInput != 0 || state.Jog.JogGeneration != j.generation {
		t.Fatalf("late input resumed an expired generation: %+v", state.Jog)
	}
}

func TestRemoteJogRefreshRejectsUnmatchedAcknowledgement(t *testing.T) {
	for _, test := range []struct {
		name   string
		change func(*remoteJogMessage)
	}{
		{"wrong_sequence", func(m *remoteJogMessage) { m.Seq++ }},
		{"wrong_source", func(m *remoteJogMessage) { m.SourceNS++ }},
		{"zero_host_time", func(m *remoteJogMessage) { m.HostTXNS = 0 }},
		{"backward_host_time", func(m *remoteJogMessage) { m.HostRXNS = m.HostTXNS + 1 }},
	} {
		t.Run(test.name, func(t *testing.T) {
			s := &RemoteJogSession{calibrationPending: 2, calibrationSequence: 4,
				calibrationSource: 100, calibratedSource: 50}
			reply := remoteJogMessage{Type: "calibrated", Seq: 4, SourceNS: 100, HostRXNS: 200, HostTXNS: 201}
			test.change(&reply)
			if err := s.acceptCalibration(reply); !errors.Is(err, ErrProtocol) {
				t.Fatalf("unmatched clock acknowledgement accepted: %v", err)
			}
			if s.calibratedSource != 50 {
				t.Fatal("invalid acknowledgement renewed calibration")
			}
		})
	}
}
