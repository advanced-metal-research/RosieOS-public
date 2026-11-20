//go:build linux

package denseexec

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/rand/v2"
	"os"
	"path/filepath"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Re-press during the previous stop, composed: the native simulator with
// jog_jerk 150 rad/s^3 (the 1430 cell's 168-cycle Halt-ramp window), rt-control and
// the public SDK, locally and over the WSS jog stream. The client behaves as
// the Steam Deck does: release is fired without waiting for its reply, the
// next press follows after a delay, and the 50 ms intent cadence skips while
// the press is in flight. Native evidence (5 ms status samples) must show the
// new hold's generation only after the released one closed, the robot armed
// throughout, and every intent after the press accepted.

var repressDelays = []time.Duration{0, 100 * time.Millisecond, 200 * time.Millisecond, 300 * time.Millisecond}

// A pipelining client's intents are absorbed only by a press that followed a
// release within its stop bound; an ordinary Begin still refuses input while
// it sets up, so its presses stay inside the bound.
func pressDelays(pipelined bool) []time.Duration {
	if pipelined {
		return repressDelays[:3]
	}
	return repressDelays
}

func TestJointJogRepressComposed(t *testing.T)       { runJointRepressComposed(t, false, false) }
func TestJointJogRepressComposedRemote(t *testing.T) { runJointRepressComposed(t, true, false) }

func TestCartesianRepressComposed100(t *testing.T) { runCartesianRepressComposed(t, false, .1, false) }
func TestCartesianRepressComposed400(t *testing.T) { runCartesianRepressComposed(t, false, .4, false) }
func TestCartesianRepressComposedRemote100(t *testing.T) {
	runCartesianRepressComposed(t, true, .1, false)
}
func TestCartesianRepressComposedRemote400(t *testing.T) {
	runCartesianRepressComposed(t, true, .4, false)
}

// A client that keeps its cadence while the press is in flight: intents
// during the wait and the setup after it refresh the press.
func TestJointJogRepressComposedPipelined(t *testing.T) { runJointRepressComposed(t, true, true) }
func TestCartesianRepressComposedPipelined400(t *testing.T) {
	runCartesianRepressComposed(t, true, .4, true)
}

// Joint holds use the simulator's own limits without jog_jerk: 0.5 rad/s^2
// jog acceleration, J1 at 0.04 of 3.14 rad/s under the 0.75 reserve, 0.094
// rad/s, whose Halt ramp lasts 188 ms as the 1430 cell's 180-196 ms ramps do. With
// jog_jerk the simulator refuses a joint Begin NotReady (reason 3) for 0-300
// ms after a smoothed ramp closes, on unmodified OLP code too (6 of 8 probe
// rounds, example-relay 2026-10-07; without jog_jerk 8 of 8 accepted 16-69 ms
// after closure). the 1430 cell's refused re-presses were all reason 1 (Closed).
// With the 1430 cell's 720 deg/s^2 on the simulated drives the cell disarmed ~30 ms
// after a joint ramp closed, no fault bit.
const repressJointFraction = .04

// the 1430 cell's jog limits on the simulator: 720 deg/s^2 jog acceleration on every
// axis and jog_jerk 150 rad/s^3 (a 168-cycle window), so stops last as long
// as on the cell (joint ramps ~180-200 ms, path stops ~100-170 ms). The
// simulator's own 0.5 rad/s^2 makes every stop outlast the 250 ms lifetime.
func armJogFixture(config map[string]any) {
	config["jog_jerk"] = map[string]any{"jerk_rad_s3": 150}
	for _, axis := range config["axes"].([]any) {
		axis.(map[string]any)["limits"].(map[string]any)["jog_acceleration"] = 4 * math.Pi
	}
}

// the 1430 cell 2026-10-07 15:33: the operator's intents stopped mid-hold at ~400
// mm/s, native expired the input and stopped along the path, OLP's End met
// native's Closed refusal and its Stop fence disarmed the cell. The expiry
// stop must leave the cell armed with no Stop, locally and over WSS.
func TestCartesianDeadmanStopStaysArmedComposed(t *testing.T) { runDeadmanStopComposed(t, false) }
func TestCartesianDeadmanStopStaysArmedComposedRemote(t *testing.T) {
	runDeadmanStopComposed(t, true)
}

func runDeadmanStopComposed(t *testing.T, remote bool) {
	c, d, root, _, cfg := rtComposedTransportFixture(t, remote, armJogFixture)
	if d.PathJog == nil {
		t.Fatal("simulator with jog_jerk does not advertise path_jog")
	}
	binary := os.Getenv("OLP_CARTESIAN_RESOLVER")
	if binary == "" {
		binary = filepath.Join(root, "build/cartmachine/bin/robot-v4-cartesiand")
	}
	cfg.CartesianModel, cfg.MoveJerk = "rosie_1420_v1", 150
	var injected RTCoreClient = c
	if remote {
		injected = nil
	}
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:cartesian-deadman"}, injected)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	resolver := NewCartesianProcess(binary, filepath.Dir(root), "rosie_1420_v1")
	t.Cleanup(resolver.Close)
	r.SetCartesianResolver(resolver.Resolve)
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	in := CartesianJogIntent{TargetID: "bench", SessionID: r.Status(ctx).Session.ID, Revision: r.JointJogState().Revision, Frame: "base", Twist: [6]float64{.4}, Fraction: 1}
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	for start := time.Now(); r.JointJogState().PathSpeed < 380 && time.Since(start) < time.Second; time.Sleep(25 * time.Millisecond) {
		if _, err := r.CartesianJog(ctx, "intent", in); err != nil {
			t.Fatal(err)
		}
	}
	speed := r.JointJogState().PathSpeed
	// The operator's intents stop: the native deadman stops along the path.
	stalled := time.Now()
	rtAwait(t, func() bool { return !r.JointJogState().Active })
	time.Sleep(500 * time.Millisecond)
	s, err := c.Status(ctx)
	if err != nil || s.Core.Armed != 1 || s.Core.AxisEnableMask == 0 {
		t.Fatalf("expiry stop at %.0f mm/s disarmed the cell: armed=%d enable=%#x err=%v", speed, s.Core.Armed, s.Core.AxisEnableMask, err)
	}
	r.mu.Lock()
	held := r.held
	r.mu.Unlock()
	if state := r.JointJogState(); state.Reason != "jog_input_deadline_expired" || !held {
		t.Fatalf("expiry retirement: %+v held=%v", state, held)
	}
	t.Logf("intents stopped at %.0f mm/s: retired after %v, armed, grant held", speed, time.Since(stalled).Round(time.Millisecond))
}

type nativeSample struct {
	at     time.Time
	gen    uint64
	open   uint32
	reason uint32
	done   bool
	armed  uint32
	ready  string
	faults uint32
	enable uint32
	// jogNow: native time of the jog observation; published: the oldest of
	// the six arm axes' (10 Hz) publications.
	jogNow, published uint64
	targets           [6]float64
}

// sampleNative records native jog/motion state every 5 ms until stopped.
func sampleNative(ctx context.Context, c *control.Client) func() []nativeSample {
	var mu sync.Mutex
	var samples []nativeSample
	done := make(chan struct{})
	finished := make(chan struct{})
	go func() {
		defer close(finished)
		for {
			select {
			case <-done:
				return
			case <-time.After(5 * time.Millisecond):
			}
			call, cancel := context.WithTimeout(ctx, 100*time.Millisecond)
			s, err := c.Status(call)
			cancel()
			if err != nil || s.Jog == nil {
				continue
			}
			n := nativeSample{at: time.Now(), gen: s.Jog.JogGeneration, open: s.Jog.Open, reason: s.Jog.StateReason, done: s.Motion.Done, armed: s.Core.Armed, ready: s.Axes[0].Readiness, faults: s.Core.ExecutionFaultReasons | s.Core.SafetyFaultMask, enable: s.Core.AxisEnableMask}
			n.jogNow, n.published = s.Jog.NowHostNs, s.Axes[0].TimeNS
			for j := range n.targets {
				n.targets[j] = s.Axes[j].LogicalTarget
				n.published = min(n.published, s.Axes[j].TimeNS)
			}
			mu.Lock()
			samples = append(samples, n)
			mu.Unlock()
		}
	}()
	return func() []nativeSample {
		close(done)
		<-finished
		mu.Lock()
		defer mu.Unlock()
		return samples
	}
}

// transitions lists native jog/motion state changes relative to the press.
func transitions(samples []nativeSample, pressed time.Time) string {
	out := ""
	var last *nativeSample
	for i := range samples {
		s := &samples[i]
		if last == nil || s.gen != last.gen || s.open != last.open || s.reason != last.reason || s.done != last.done || s.ready != last.ready || s.faults != last.faults || s.armed != last.armed || s.enable != last.enable {
			out += fmt.Sprintf("  %+5dms gen=%d open=%d reason=%d done=%v armed=%d enable=%#x J1=%s faults=%#x\n",
				s.at.Sub(pressed).Milliseconds(), s.gen, s.open, s.reason, s.done, s.armed, s.enable, s.ready, s.faults)
		}
		last = s
	}
	return out
}

// deadmanDrop accepts the one refusal a client that sends nothing while its
// press is in flight may meet: the press's own lifetime ran out before the
// stop finished (or, for a path hold, before the 10 Hz rest publication), so
// it was dropped and nothing opened or moved. A pipelining client never may.
func deadmanDrop(t *testing.T, err error, pipelined bool, evidence []nativeSample, released uint64, pressed time.Time, delay time.Duration) bool {
	t.Helper()
	var e *Error
	if pipelined || !errors.As(err, &e) || e.Code != "jog_input_deadline_expired" {
		return false
	}
	for _, s := range evidence {
		if s.gen > released || s.armed != 1 {
			return false
		}
	}
	closed, _ := stopThenOpen(evidence, released)
	at := "after the samples"
	if closed >= 0 {
		at = evidence[closed].at.Sub(pressed).Round(time.Millisecond).String()
	}
	t.Logf("press %v after release: dropped by its own deadline (no intent while in flight); the stop closed %s after the press; nothing opened", delay, at)
	return true
}

// simulatorNotReady accepts a joint Begin refused NotReady (reason 3) by the
// simulator just after the released ramp closed: an ordinary press meets it
// too on unmodified OLP code (probe: up to ~15 ms after closure without
// jog_jerk, 0-300 ms with it); the 1430 cell's refusals were reason 1. Nothing may
// have opened and the cell must stay armed. It is counted, not hidden.
func simulatorNotReady(t *testing.T, err error, evidence []nativeSample, released uint64, delay time.Duration) bool {
	t.Helper()
	var e *Error
	if !errors.As(err, &e) || e.Code != "jog_not_ready" {
		return false
	}
	closed, _ := stopThenOpen(evidence, released)
	for _, s := range evidence {
		if s.gen > released || s.armed != 1 {
			return false
		}
	}
	if closed < 0 {
		return false
	}
	t.Logf("press %v after release: SIMULATOR NotReady after the ramp closed; nothing opened, armed", delay)
	return true
}

// stopThenOpen finds the first sample with the released generation closed
// (live jog observation) and the first with a newer generation.
//
// Native admits a Begin only once the previous session closed (a ramping
// lane refuses it Closed), so a newer generation after a stopping sample is
// itself ordering evidence when the closure lasted under one sample.
func stopThenOpen(samples []nativeSample, released uint64) (closed, opened int) {
	closed, opened = -1, -1
	stopping := -1
	for i, s := range samples {
		if s.gen == released && s.open == 0 && s.reason == ipcclient.JogReasonRamping {
			stopping = i
		}
		if closed < 0 && s.gen == released && s.open == 0 && s.reason != ipcclient.JogReasonRamping {
			closed = i
		}
		if opened < 0 && s.gen > released {
			opened = i
		}
	}
	if closed < 0 && opened > 0 && stopping == opened-1 {
		closed = stopping
	}
	return closed, opened
}

// repressClient is the Deck's hold loop: a 50 ms cadence that skips while a
// request is in flight; every intent must be accepted.
type repressClient struct {
	send   func(action string) error
	mu     sync.Mutex
	busy   bool
	failed error
	stop   chan struct{}
	done   chan struct{}
}

func startIntents(send func(string) error) *repressClient {
	c := &repressClient{send: send, stop: make(chan struct{}), done: make(chan struct{})}
	go func() {
		defer close(c.done)
		ticker := time.NewTicker(50 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-c.stop:
				return
			case <-ticker.C:
			}
			c.mu.Lock()
			skip := c.busy || c.failed != nil
			c.mu.Unlock()
			if skip {
				continue
			}
			if err := send("update"); err != nil {
				c.mu.Lock()
				c.failed = err
				c.mu.Unlock()
			}
		}
	}()
	return c
}

func (c *repressClient) inFlight(on bool) { c.mu.Lock(); c.busy = on; c.mu.Unlock() }
func (c *repressClient) end() error {
	close(c.stop)
	<-c.done
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.failed
}

func runJointRepressComposed(t *testing.T, remote, pipelined bool) {
	c, _, _, _, cfg := rtComposedTransportFixture(t, remote)
	// the 1430 cell runs with a Cartesian model: prepared joint lanes, as there.
	cfg.CartesianModel = "rosie_1420_v1"
	var injected RTCoreClient = c
	if remote {
		injected = nil
	}
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:joint-repress"}, injected)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 120*time.Second)
	defer cancel()
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	session := r.Status(ctx).Session.ID
	direction := 1
	for _, delay := range pressDelays(pipelined) {
		in := JointJogIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Axis: 0, Direction: direction, Fraction: repressJointFraction}
		send := func(action string) error { _, err := r.JointJog(ctx, action, in); return err }
		client := startIntents(send)
		client.inFlight(true)
		// The setup hold is not under test: it retries the simulator's
		// post-stop NotReady for up to 1 s.
		for started := time.Now(); ; time.Sleep(50 * time.Millisecond) {
			in.Revision = r.JointJogState().Revision
			err := send("begin")
			if err == nil {
				break
			}
			if time.Since(started) > time.Second {
				t.Fatalf("first hold: %v", err)
			}
		}
		client.inFlight(false)
		// Hold 400 ms: J1 at speed, so its release ramps for ~v/a + 168 ms.
		time.Sleep(400 * time.Millisecond)
		released := r.JointJogState().Generation
		samples := sampleNative(ctx, c)
		if err := client.end(); err != nil {
			t.Fatalf("intent during the first hold: %v", err)
		}
		// The release is fired, not awaited (the Deck's endHold).
		release := in
		go func() { _, _ = r.JointJog(ctx, "end", release) }()
		time.Sleep(delay)
		rtAwait(t, func() bool { return r.JointJogState().Revision > release.Revision })
		direction = -direction
		in = JointJogIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Axis: 0, Direction: direction, Fraction: repressJointFraction}
		pressed := time.Now()
		client = startIntents(send)
		client.inFlight(!pipelined)
		if err := send("begin"); err != nil {
			_ = client.end()
			if evidence := samples(); !deadmanDrop(t, err, pipelined, evidence, released, pressed, delay) && !simulatorNotReady(t, err, evidence, released, delay) {
				t.Fatalf("press %v after release (reversed): %v\n%s", delay, err, transitions(evidence, pressed))
			}
			rtAwait(t, func() bool {
				s, err := c.Status(ctx)
				return err == nil && s.Jog != nil && s.Jog.Open == 0 && s.Motion.Done
			})
			continue
		}
		client.inFlight(false)
		began := time.Since(pressed)
		time.Sleep(300 * time.Millisecond)
		if err := client.end(); err != nil {
			t.Fatalf("intent after a queued press: %v", err)
		}
		evidence := samples()
		closed, opened := stopThenOpen(evidence, released)
		if closed < 0 || opened < 0 || opened <= closed {
			t.Fatalf("press %v: released gen %d closed at sample %d, next opened at %d (of %d)", delay, released, closed, opened, len(evidence))
		}
		for _, s := range evidence {
			if s.armed != 1 {
				t.Fatalf("press %v: the cell disarmed", delay)
			}
		}
		t.Logf("press %v after release: begin returned in %v; stop closed %v after the first sample, next generation opened %v after that",
			delay, began.Round(time.Millisecond), evidence[closed].at.Sub(evidence[0].at).Round(time.Millisecond), evidence[opened].at.Sub(evidence[closed].at).Round(time.Millisecond))
		if _, err := r.JointJog(ctx, "end", in); err != nil {
			t.Fatal(err)
		}
		rtAwait(t, func() bool {
			s, err := c.Status(ctx)
			return err == nil && s.Jog != nil && s.Jog.Open == 0 && s.Motion.Done
		})
	}
	if s, err := c.Status(ctx); err != nil || s.Core.Armed != 1 {
		t.Fatalf("not armed after the re-presses: %v", err)
	}
}

func runCartesianRepressComposed(t *testing.T, remote bool, box float64, pipelined bool) {
	c, d, root, _, cfg := rtComposedTransportFixture(t, remote, armJogFixture)
	cfg.MoveJerk = 150
	if d.PathJog == nil {
		t.Fatal("simulator with jog_jerk does not advertise path_jog")
	}
	binary := os.Getenv("OLP_CARTESIAN_RESOLVER")
	if binary == "" {
		binary = filepath.Join(root, "build/cartmachine/bin/robot-v4-cartesiand")
	}
	cfg.CartesianModel = "rosie_1420_v1"
	var injected RTCoreClient = c
	if remote {
		injected = nil
	}
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:cartesian-repress"}, injected)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 120*time.Second)
	defer cancel()
	resolver := NewCartesianProcess(binary, filepath.Dir(root), "rosie_1420_v1")
	t.Cleanup(resolver.Close)
	r.SetCartesianResolver(func(ctx context.Context, in CartesianResolveRequest) (CartesianResolution, error) {
		select {
		case <-ctx.Done():
			return CartesianResolution{}, ctx.Err()
		case <-time.After(jitteredResolverLatency()):
		}
		return resolver.Resolve(ctx, in)
	})
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	session := r.Status(ctx).Session.ID
	direction := 1.0
	twist := func() [6]float64 { return [6]float64{direction * box} }
	in := CartesianJogIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Frame: "base", Twist: twist(), Fraction: 1}
	send := func(action string) error { _, err := r.CartesianJog(ctx, action, in); return err }
	delays := pressDelays(pipelined)
	client := startIntents(func(string) error { return send("intent") })
	client.inFlight(true)
	if err := send("start"); err != nil {
		t.Fatalf("first hold: %v", err)
	}
	client.inFlight(false)
	for _, delay := range delays {
		// Hold until the path runs at the commanded speed (or 600 ms), so the
		// release is a full-speed stop.
		for start := time.Now(); r.JointJogState().PathSpeed < .95*box*1000 && time.Since(start) < 600*time.Millisecond; {
			time.Sleep(5 * time.Millisecond)
		}
		speed := r.JointJogState().PathSpeed
		released := r.JointJogState().Generation
		samples := sampleNative(ctx, c)
		if err := client.end(); err != nil {
			t.Fatalf("intent during the hold: %v", err)
		}
		halted := in
		go func() { _, _ = r.CartesianJog(ctx, "halt", halted) }()
		time.Sleep(delay)
		rtAwait(t, func() bool { return r.JointJogState().Revision > halted.Revision })
		// A press after release, reversed: the next hold's line.
		direction = -direction
		in = CartesianJogIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Frame: "base", Twist: twist(), Fraction: 1}
		pressed := time.Now()
		client = startIntents(func(string) error { return send("intent") })
		client.inFlight(!pipelined)
		if err := send("start"); err != nil {
			_ = client.end()
			if evidence := samples(); !deadmanDrop(t, err, pipelined, evidence, released, pressed, delay) {
				t.Fatalf("press %v after a %.0f mm/s release: %v\n%s", delay, speed, err, transitions(evidence, pressed))
			}
			// The next block starts from a fresh hold.
			in.Revision = r.JointJogState().Revision
			client = startIntents(func(string) error { return send("intent") })
			client.inFlight(true)
			if err := send("start"); err != nil {
				t.Fatalf("hold after a dropped press: %v", err)
			}
			client.inFlight(false)
			continue
		}
		client.inFlight(false)
		began := time.Since(pressed)
		r.mu.Lock()
		var origin [6]float64
		if r.jog != nil && r.jog.path != nil {
			origin = r.jog.path.line.Origin
		}
		r.mu.Unlock()
		// Let the new hold run before judging its start.
		time.Sleep(150 * time.Millisecond)
		evidence := samples()
		closed, opened := stopThenOpen(evidence, released)
		if closed < 0 || opened < 0 || opened <= closed {
			t.Fatalf("press %v: released gen %d closed at sample %d, next opened at %d (of %d)", delay, released, closed, opened, len(evidence))
		}
		// The new line starts at the held command at rest: the first per-axis
		// publication stamped after the native closure (the 10 Hz targets lag
		// it), unchanged until the new hold moves.
		rest := -1
		for i := closed; i < len(evidence); i++ {
			if evidence[i].published >= evidence[closed].jogNow {
				rest = i
				break
			}
		}
		if rest < 0 || origin != evidence[rest].targets {
			t.Fatalf("press %v: line starts at %v; rest publication %d of %d: %+v", delay, origin, rest, len(evidence), evidence[max(rest, 0)])
		}
		for _, s := range evidence {
			if s.armed != 1 {
				t.Fatalf("press %v: the cell disarmed", delay)
			}
		}
		if s := r.JointJogState(); !s.Active {
			t.Fatalf("press %v: hold not active after start: %+v", delay, s)
		}
		t.Logf("%.0f mm/s release, press %v later: begin returned in %v; stop closed %v after the first sample, next generation %v after that",
			speed, delay, began.Round(time.Millisecond), evidence[closed].at.Sub(evidence[0].at).Round(time.Millisecond), evidence[opened].at.Sub(evidence[closed].at).Round(time.Millisecond))
	}
	if err := client.end(); err != nil {
		t.Fatalf("intent after the last press: %v", err)
	}
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	s, err := c.Status(ctx)
	if err != nil || s.Core.Armed != 1 {
		t.Fatalf("not armed after the re-presses: %v", err)
	}
	if state := r.JointJogState(); state.Reason != "" {
		t.Fatalf("hold ended with %s", state.Reason)
	}
}

// the 1430 cell 2026-10-07 15:33: the browser kept one request in flight and skipped
// ticks, so slow round trips left 135/221/>=250 ms between intents at OLP and
// the 250 ms deadman stopped a 400 mm/s hold. The clients now keep the 50 ms
// cadence with up to 5 requests in flight (ui jointJog.ts, pendant
// hold_pacer.hpp). With 150-300 ms round trips (each split at random between
// the request and reply legs) a 400 mm/s path hold must run until release, and
// stop on release with the cell armed. The old pacing on the same link is
// logged for contrast.
func TestCartesianSlowLinkHoldComposed(t *testing.T)       { runSlowLinkHoldComposed(t, false) }
func TestCartesianSlowLinkHoldComposedRemote(t *testing.T) { runSlowLinkHoldComposed(t, true) }

type slowLinkResult struct {
	maxGap  time.Duration
	retired string
	sent    int
	speed   float64
}

func runSlowLinkHoldComposed(t *testing.T, remote bool) {
	c, d, root, _, cfg := rtComposedTransportFixture(t, remote, armJogFixture)
	if d.PathJog == nil {
		t.Fatal("simulator with jog_jerk does not advertise path_jog")
	}
	binary := os.Getenv("OLP_CARTESIAN_RESOLVER")
	if binary == "" {
		binary = filepath.Join(root, "build/cartmachine/bin/robot-v4-cartesiand")
	}
	cfg.CartesianModel, cfg.MoveJerk = "rosie_1420_v1", 150
	var injected RTCoreClient = c
	if remote {
		injected = nil
	}
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:slow-link"}, injected)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 90*time.Second)
	defer cancel()
	resolver := NewCartesianProcess(binary, filepath.Dir(root), "rosie_1420_v1")
	t.Cleanup(resolver.Close)
	r.SetCartesianResolver(resolver.Resolve)
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	session := r.Status(ctx).Session.ID
	direction := 1.0
	hold := func(maxInFlight int) slowLinkResult {
		in := CartesianJogIntent{TargetID: "bench", SessionID: session, Revision: r.JointJogState().Revision, Frame: "base", Twist: [6]float64{.4 * direction}, Fraction: 1}
		direction = -direction
		var mu sync.Mutex
		var arrivals []time.Time
		var res slowLinkResult
		inFlight, released, starting := 0, false, true
		var failed error
		var wg sync.WaitGroup
		// One request over the link: request leg, OLP, reply leg.
		request := func(action string, early bool) {
			defer wg.Done()
			rtt := 150*time.Millisecond + rand.N(150*time.Millisecond)
			up := rand.N(rtt)
			time.Sleep(up)
			mu.Lock()
			skip := released && action == "intent"
			if !skip {
				arrivals = append(arrivals, time.Now())
			}
			mu.Unlock()
			var err error
			if !skip {
				_, err = r.CartesianJog(ctx, action, in)
			}
			time.Sleep(rtt - up)
			mu.Lock()
			defer mu.Unlock()
			inFlight--
			if action == "start" {
				starting = false
			}
			var e *Error
			stale := errors.As(err, &e) && e.Code == "jog_session_stale"
			if err != nil && !released && failed == nil && !(early && stale) {
				failed = fmt.Errorf("%s: %w", action, err)
			}
		}
		mu.Lock()
		inFlight++
		mu.Unlock()
		wg.Add(1)
		go request("start", false)
		ticker := time.NewTicker(50 * time.Millisecond)
		// 650 ms: well inside the fixture's line (its speed caps fall near the
		// jacobian gate ~300-400 mm out), so the release is a full-speed stop.
		holdFor := time.After(650 * time.Millisecond)
	cadence:
		for {
			select {
			case <-holdFor:
				break cadence
			case <-ticker.C:
				mu.Lock()
				send := inFlight < maxInFlight && failed == nil
				early := starting
				if send {
					inFlight++
					res.sent++
				}
				mu.Unlock()
				if send {
					wg.Add(1)
					go request("intent", early)
				}
				if s := r.JointJogState(); !s.Active && !early {
					res.retired = s.Reason
				}
			}
		}
		ticker.Stop()
		res.speed = r.JointJogState().PathSpeed
		mu.Lock()
		released = true
		err := failed
		mu.Unlock()
		if _, herr := r.CartesianJog(ctx, "halt", in); herr != nil && err == nil && res.retired == "" {
			err = herr
		}
		wg.Wait()
		mu.Lock()
		for i := 1; i < len(arrivals); i++ {
			res.maxGap = max(res.maxGap, arrivals[i].Sub(arrivals[i-1]))
		}
		mu.Unlock()
		if err != nil && res.retired == "" {
			res.retired = err.Error()
		}
		t.Logf("in flight <= %d: %d intents, largest gap between arrivals at OLP %v, path speed at release %.0f mm/s, retired early: %q",
			maxInFlight, res.sent, res.maxGap.Round(time.Millisecond), res.speed, res.retired)
		rtAwait(t, func() bool {
			s, err := c.Status(ctx)
			return err == nil && s.Jog != nil && s.Jog.Open == 0 && s.Motion.Done
		})
		return res
	}
	pipelined := hold(5)
	if pipelined.retired != "" {
		t.Fatalf("the hold ended before release on a slow link: %s", pipelined.retired)
	}
	s, err := c.Status(ctx)
	if err != nil || s.Core.Armed != 1 {
		t.Fatalf("not armed after the slow-link hold: %v", err)
	}
	if pipelined.maxGap >= 250*time.Millisecond || pipelined.speed < 380 {
		t.Fatalf("pipelined: %v gap between arrivals, %.0f mm/s at release", pipelined.maxGap, pipelined.speed)
	}
	// The old pacing, for the record: one request in flight, ticks skipped.
	_ = hold(1)
}
