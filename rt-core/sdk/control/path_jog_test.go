//go:build linux

package control

import (
	"bytes"
	"context"
	"crypto/tls"
	"errors"
	"os"
	"path/filepath"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// pathJogMachine is the default simulation machine plus the machine-level
// jog_jerk the path generator needs (150 rad/s^3, the 1430 cell's value; on the
// simulation's 0.5 rad/s^2 jog acceleration rtctl sizes a 7-cycle window).
func pathJogMachine(t *testing.T) string {
	t.Helper()
	root, err := filepath.Abs("../..")
	must(t, err)
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-program.json"))
	must(t, err)
	at := bytes.IndexByte(raw, '{')
	if at < 0 {
		t.Fatal("simulation machine is not a JSON object")
	}
	machine := append(append(append([]byte{}, raw[:at+1]...), []byte(`"jog_jerk": {"jerk_rad_s3": 150},`)...), raw[at+1:]...)
	path := filepath.Join(t.TempDir(), "simulation-path-jog.json")
	must(t, os.WriteFile(path, machine, 0o644))
	return path
}

// pathSamples is a straight joint-space segment from the held targets: every
// axis moves 0.1 rad per metre of path, 1e-4 rad per 1 mm sample.
func pathSamples(held []float64, first, count int, direction float64) [][]float64 {
	samples := make([][]float64, count)
	for i := range samples {
		row := make([]float64, len(held))
		for axis, q := range held {
			row[axis] = q + direction*0.1*float64(first+i)*0.001
		}
		samples[i] = row
	}
	return samples
}

// heldTargets reads the held command targets once the status file shows
// them at rest: two reads 150 ms apart (more than one 100 ms status period)
// must agree, the same post-rest rule a path producer needs, because a pose
// published during the previous motion is not the held command.
func heldTargets(t *testing.T, ctx context.Context, c *Client) []float64 {
	t.Helper()
	read := func() []float64 {
		state, err := c.Status(ctx)
		must(t, err)
		held := make([]float64, len(state.Axes))
		for i, axis := range state.Axes {
			held[i] = axis.LogicalTarget
		}
		return held
	}
	previous := read()
	for attempt := 0; attempt < 20; attempt++ {
		time.Sleep(150 * time.Millisecond)
		current := read()
		same := true
		for i := range current {
			same = same && current[i] == previous[i]
		}
		if same {
			return current
		}
		previous = current
	}
	t.Fatal("held targets never came to rest")
	return nil
}

func direction(held []float64) float64 {
	if held[0] > 0 {
		return -1
	}
	return 1
}

func TestSDKPathJogUnavailableWithoutJogJerk(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	description, err := s.client.Describe(context.Background())
	must(t, err)
	if description.PathJog != nil {
		t.Fatalf("path_jog advertised without jog_jerk: %+v", description.PathJog)
	}
	_, err = s.client.NewPathJogSession(context.Background(), 511, 0.001, 0.05, 200_000_000, nil)
	var rejection *PathJogRejection
	if !errors.As(err, &rejection) || rejection.Reason != "path_jog_unavailable" {
		t.Fatalf("path session opened without the capability: %v", err)
	}
}

// Local path jog end to end through rt-control and the simulated core:
// chunked append acknowledged by native, speed, s advancing, release stopping
// on the path, expiry, and a start mismatch refused with its native reason.
func TestSDKLocalPathJog(t *testing.T) {
	s := startServiceWith(t, pathJogMachine(t))
	s.acquire(t, true)
	s.enable(t)
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	description, err := s.client.Describe(ctx)
	must(t, err)
	if description.PathJog == nil || description.PathJog.MaxSamples != 1024 || description.PathJog.MaxChunk != 16 ||
		description.PathJog.MaxSampleStepS != 0.02 {
		t.Fatalf("path_jog capability: %+v", description.PathJog)
	}
	held := heldTargets(t, ctx, s.client)
	dir := direction(held)
	j, err := s.client.NewPathJogSession(ctx, 511, 0.001, 0.05, 200_000_000, nil)
	must(t, err)
	// 300 samples in 19 chunks of at most 16, each acknowledged by native.
	must(t, j.Append(ctx, 0, pathSamples(held, 0, 300, dir), nil, false, 0))
	status, err := get[PathJogStatus](s.client, ctx, "/v1/jog/path")
	must(t, err)
	if status.Accepted != 19 || status.Observation.Received != 300 || status.Observation.ChunkResult != 0 {
		t.Fatalf("handshake: %+v", status)
	}
	started := time.Now()
	for time.Since(started) < 400*time.Millisecond {
		must(t, j.SetSpeed(0.05, ipcclient.HostMonotonicNS()))
		time.Sleep(50 * time.Millisecond)
	}
	moving, err := j.Observe(ctx)
	must(t, err)
	if !moving.Open || !moving.HasSpeed || moving.PathS <= 0.005 || moving.PathSpeed <= 0 {
		t.Fatalf("path did not advance: %+v", moving)
	}
	must(t, j.End(ctx))
	stopped, err := j.Observe(ctx)
	must(t, err)
	if stopped.Open || stopped.PathSpeed != 0 || stopped.StopReason != 1 || stopped.PathS < moving.PathS {
		t.Fatalf("release did not stop on the path: %+v", stopped)
	}
	jog, err := s.client.JogStatus(ctx)
	must(t, err)
	if jog.Open != 0 {
		t.Fatalf("jog session still open after End: %+v", jog)
	}
	state, err := s.client.Status(ctx)
	must(t, err)
	if state.Core.Armed != 1 {
		t.Fatal("a completed path stop disarmed the cell")
	}

	// Expiry: one speed command, then silence; native stops along the path.
	held = heldTargets(t, ctx, s.client)
	expiring, err := s.client.NewPathJogSession(ctx, 511, 0.001, 0.05, 100_000_000, nil)
	must(t, err)
	must(t, expiring.Append(ctx, 0, pathSamples(held, 0, 200, dir), nil, false, 0))
	must(t, expiring.SetSpeed(0.05, ipcclient.HostMonotonicNS()))
	closed := expiring.Rejection(ctx)
	var rejection *PathJogRejection
	if !errors.As(closed, &rejection) || rejection.Reason != "path_closed" {
		t.Fatalf("expiry did not close the session: %v", closed)
	}
	expired, err := expiring.Observe(ctx)
	must(t, err)
	if expired.StopReason != 2 || expired.PathSpeed != 0 {
		t.Fatalf("expiry stop: %+v", expired)
	}
	must(t, expiring.End(ctx))

	// Sample 0 off the held command by 0.01 rad: refused, session closed.
	held = heldTargets(t, ctx, s.client)
	mismatch, err := s.client.NewPathJogSession(ctx, 511, 0.001, 0.05, 100_000_000, nil)
	must(t, err)
	wrong := pathSamples(held, 0, 16, dir)
	for _, row := range wrong {
		row[0] += 0.01
	}
	err = mismatch.Append(ctx, 0, wrong, nil, false, 0)
	if !errors.As(err, &rejection) || rejection.Reason != "path_start_mismatch" || rejection.NativeReason != ipcclient.JogReasonPathStartMismatch {
		t.Fatalf("start mismatch: %v", err)
	}
	_ = mismatch.End(ctx)
}

// The press that opens a hold is captured before the session exists (OLP
// resolves the first look-ahead, then opens the lane). the 1430 cell 2026-10-07: a
// 17 ms warm resolver put that capture before the WSS Begin sample and every
// remote hold ended jog_clock_moved_backwards; locally native refused it as
// InvalidTiming before begin_origin. 30 ms stands for the resolver
// (unverified stimulus). The press alone must start the path, its native
// deadline must stay capture + lifetime, and an earlier capture that would
// extend it is not sent.
func pathPressBeforeOpen(t *testing.T, ctx context.Context, c *Client, source func() uint64, lifetime uint64, remote bool) {
	t.Helper()
	held := heldTargets(t, ctx, c)
	press := source()
	time.Sleep(30 * time.Millisecond)
	j, err := c.NewPathJogSession(ctx, 511, 0.001, 0.05, lifetime, source)
	must(t, err)
	if j.opened <= press {
		t.Fatalf("session opened at %d, not after the press at %d", j.opened, press)
	}
	must(t, j.Append(ctx, 0, pathSamples(held, 0, 300, direction(held)), nil, false, 0))
	must(t, j.SetSpeed(0.05, press))
	if j.lastOrigin != j.opened || j.lastDeadline != press+lifetime {
		t.Fatalf("press sent origin %d deadline %d, want %d and %d", j.lastOrigin, j.lastDeadline, j.opened, press+lifetime)
	}
	if err := j.SetSpeed(0.05, press+1); !errors.Is(err, ErrPathSpeedPredatesSession) || !errors.Is(err, ErrJogWouldBlock) {
		t.Fatalf("a second pre-opening capture extending the deadline: %v", err)
	}
	var moving PathJogObservation
	for deadline := time.Now().Add(time.Duration(lifetime)); ; {
		moving, err = j.Observe(ctx)
		must(t, err)
		if moving.HasSpeed && moving.PathSpeed > 0 || time.Now().After(deadline) {
			break
		}
		time.Sleep(2 * time.Millisecond)
	}
	if !moving.Open || !moving.HasSpeed || moving.PathSpeed <= 0 {
		t.Fatalf("the press did not start the path: %+v", moving)
	}
	if !remote && moving.SpeedDeadlineNS != press+lifetime {
		t.Fatalf("native deadline %d, want the press's %d", moving.SpeedDeadlineNS, press+lifetime)
	}
	if remote {
		select {
		case err := <-j.terminal:
			t.Fatalf("stream refused the press: %v", err)
		default:
		}
	}
	// Only operator input renews the deadman: the press alone expires.
	closed := j.Rejection(ctx)
	var rejection *PathJogRejection
	if !remote && (!errors.As(closed, &rejection) || rejection.Reason != "path_closed") {
		t.Fatalf("press-only session did not expire: %v", closed)
	}
	if remote {
		for {
			o, err := j.Observe(ctx)
			must(t, err)
			if !o.Open {
				if o.StopReason != 2 {
					t.Fatalf("press-only session closed with %d, not expiry: %+v", o.StopReason, o)
				}
				break
			}
			time.Sleep(5 * time.Millisecond)
		}
	}
	_ = j.End(ctx)
	if remote {
		return
	}
	// A capture already expired when the session opened is never sent.
	stale := source() - lifetime
	held = heldTargets(t, ctx, c)
	k, err := c.NewPathJogSession(ctx, 511, 0.001, 0.05, lifetime, source)
	must(t, err)
	must(t, k.Append(ctx, 0, pathSamples(held, 0, 50, direction(held)), nil, false, 0))
	if err := k.SetSpeed(0.05, stale); !errors.Is(err, ErrPathSpeedPredatesSession) {
		t.Fatalf("expired pre-opening capture: %v", err)
	}
	_ = k.End(ctx)
}

func TestSDKLocalPathJogPressBeforeOpen(t *testing.T) {
	s := startServiceWith(t, pathJogMachine(t))
	s.acquire(t, true)
	s.enable(t)
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	pathPressBeforeOpen(t, ctx, s.client, ipcclient.HostMonotonicNS, 200_000_000, false)
	state, err := s.client.Status(ctx)
	must(t, err)
	if state.Core.Armed != 1 {
		t.Fatal("a press-only path hold disarmed the cell")
	}
}

func TestSDKRemotePathJogPressBeforeOpen(t *testing.T) {
	p := newPKI(t)
	addr, args := p.remoteArgs(t)
	s := startServiceWith(t, pathJogMachine(t), append(args, remoteJogTestFlags...)...)
	c, err := DialTLS(addr, &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}})
	must(t, err)
	t.Cleanup(func() { c.Close() })
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	remoteJogReady(t, s, c, ctx)
	source := func() uint64 { return ipcclient.HostMonotonicNS() - 1_000_000_000 }
	pathPressBeforeOpen(t, ctx, c, source, 100_000_000, true)
	state, err := c.Status(ctx)
	must(t, err)
	if state.Core.Armed != 1 {
		t.Fatal("a press-only remote path hold disarmed the cell")
	}
}

// Remote path jog over the WSS jog stream: begin with path, chunks with
// path_ack replies, clock-mapped speed frames, Observe and End.
func TestSDKRemotePathJog(t *testing.T) {
	p := newPKI(t)
	addr, args := p.remoteArgs(t)
	s := startServiceWith(t, pathJogMachine(t), append(args, remoteJogTestFlags...)...)
	c, err := DialTLS(addr, &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}})
	must(t, err)
	t.Cleanup(func() { c.Close() })
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	remoteJogReady(t, s, c, ctx)
	held := heldTargets(t, ctx, c)
	dir := direction(held)
	// Unverified source offset stimulus, as in the remote jog tests.
	source := func() uint64 { return ipcclient.HostMonotonicNS() - 1_000_000_000 }
	j, err := c.NewPathJogSession(ctx, 511, 0.001, 0.05, 100_000_000, source)
	must(t, err)
	must(t, j.Append(ctx, 0, pathSamples(held, 0, 300, dir), nil, false, 0))
	started := time.Now()
	for time.Since(started) < 400*time.Millisecond {
		must(t, j.SetSpeed(0.05, source()))
		time.Sleep(20 * time.Millisecond)
	}
	moving, err := j.Observe(ctx)
	must(t, err)
	if !moving.Open || moving.PathS <= 0.005 {
		t.Fatalf("remote path did not advance: %+v", moving)
	}
	must(t, j.End(ctx))
	stopped, err := j.Observe(ctx)
	must(t, err)
	if stopped.Open || stopped.StopReason != 1 || stopped.PathSpeed != 0 {
		t.Fatalf("remote release did not stop on the path: %+v", stopped)
	}
}
