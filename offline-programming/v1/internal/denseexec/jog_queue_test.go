//go:build linux

package denseexec

import (
	"context"
	"math"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// the 1430 cell's published input lifetime (Describe max_jog_input_age_ns).
const queueTestLifetime = 250 * time.Millisecond

// the 1430 cell's jog limits in absolute units: 720 deg/s^2 jog acceleration on every
// axis, jog_jerk 150 rad/s^3, 1 ms cycle (168-cycle smoothing window).
func armJogLimits(r *RTCoreController) {
	r.mu.Lock()
	defer r.mu.Unlock()
	r.description.CycleNS = 1000000
	r.description.MaxJogInputAgeNS = uint64(queueTestLifetime)
	for i := range r.description.Axes {
		r.description.Axes[i].JogAcceleration = 4 * math.Pi
	}
	r.cfg.MoveJerk = 150
}

// The joint stop bound is native's HaltRamp bound from configured limits:
// ceil(|v|/a) + window + one cycle, plus End delivery and observation.
func TestJogStopBoundFromConfiguredLimits(t *testing.T) {
	r, _, _, in := jointFake(t)
	armJogLimits(r)
	r.mu.Lock()
	defer r.mu.Unlock()
	r.roundTrip = 5 * time.Millisecond
	in.Fraction = .5
	// v = 0.5 x 2 rad/s x 0.75 reserve = 0.75 rad/s; v/a = 0.75/(4 pi) s =
	// 59.683104 ms (ceil to ns); window ceil(2 x 12.566/150 s / 1 ms) = 168 ms;
	// + 1 ms cycle. Margin: 2 x 5 ms RTT + 10 ms poll + 1 ms cycle.
	bound, source := r.jogStopBoundLocked(&jointJogHold{intent: in})
	if want := 59683104*time.Nanosecond + 168*time.Millisecond + time.Millisecond + 21*time.Millisecond; bound != want || source != "halt_ramp" {
		t.Fatalf("joint bound %v %s, want %v halt_ramp", bound, source, want)
	}
	// A path stop's bound is not published: the input lifetime.
	path := in
	path.cartesian = &CartesianJogIntent{Frame: "base", Twist: [6]float64{.4}}
	if bound, source := r.jogStopBoundLocked(&jointJogHold{intent: path, path: &cartesianPathHold{}}); bound != queueTestLifetime+21*time.Millisecond || source != "input_lifetime" {
		t.Fatalf("path bound %v %s", bound, source)
	}
	// A cell with path_jog configures jog_jerk: without OLP's compiled jerk
	// the window is unknown, so the lifetime stands in.
	r.cfg.MoveJerk = 0
	r.description.PathJog = &ipcclient.PathJogCapability{MaxSampleStepS: .02}
	if _, source := r.jogStopBoundLocked(&jointJogHold{intent: in}); source != "input_lifetime" {
		t.Fatalf("unknown window used %s", source)
	}
}

type queueJointHarness struct {
	r     *RTCoreController
	f     *fakeRTCore
	lane  *fakeJointLane
	in    JointJogIntent
	mu    sync.Mutex
	opens []time.Time
}

// queueJointFake holds J1, releases it and leaves native ramping (the Halt
// ramp of the released generation is running).
func queueJointFake(t *testing.T) *queueJointHarness {
	t.Helper()
	r, f, lane, in := jointFake(t)
	armJogLimits(r)
	q := &queueJointHarness{r: r, f: f, lane: lane, in: in}
	open := r.jogOpen
	r.jogOpen = func(ctx context.Context, mask uint32, origin uint64) (jointJogLane, error) {
		q.mu.Lock()
		q.opens = append(q.opens, time.Now())
		q.mu.Unlock()
		return open(ctx, mask, origin)
	}
	ctx := context.Background()
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
	q.setRamping(true)
	q.in.Revision = r.JointJogState().Revision
	return q
}

func (q *queueJointHarness) setRamping(ramping bool) {
	q.f.mu.Lock()
	defer q.f.mu.Unlock()
	q.f.status.TimeNS++
	q.f.status.Motion.TimeNS++
	if ramping {
		q.f.status.Jog = &ipcclient.JogObservation{JogGeneration: 17, StateReason: ipcclient.JogReasonRamping}
	} else {
		q.f.status.Jog = nil
	}
}

func (q *queueJointHarness) openTimes() []time.Time {
	q.mu.Lock()
	defer q.mu.Unlock()
	return append([]time.Time(nil), q.opens...)
}

func noStop(t *testing.T, f *fakeRTCore) {
	t.Helper()
	for _, call := range jointCalls(f) {
		if call == "stop" || call == "release" {
			t.Fatalf("a queued press took the Stop fence: %v", jointCalls(f))
		}
	}
	f.mu.Lock()
	defer f.mu.Unlock()
	if f.status.Core.Armed != 1 {
		t.Fatal("a queued press disarmed the cell")
	}
}

// A press 0-150 ms after release waits for the ramp, then opens: the open is
// never before native closed the released generation, and the next update
// is accepted (no jog_session_stale).
func TestJointJogPressDuringHaltRampStartsAfterIt(t *testing.T) {
	for _, delay := range []time.Duration{0, 50 * time.Millisecond, 100 * time.Millisecond, 150 * time.Millisecond} {
		t.Run(delay.String(), func(t *testing.T) {
			q := queueJointFake(t)
			// Released at 0; the ramp completes 170 ms later (~the 1430 cell's 180 ms).
			ramp := 170 * time.Millisecond
			closed := make(chan time.Time, 1)
			released := time.Now()
			go func() {
				time.Sleep(ramp)
				q.setRamping(false)
				closed <- time.Now()
			}()
			time.Sleep(delay)
			if _, err := q.r.JointJog(context.Background(), "begin", q.in); err != nil {
				t.Fatalf("press %v after release: %v", delay, err)
			}
			at := <-closed
			opens := q.openTimes()
			if len(opens) != 2 || opens[1].Before(at) {
				t.Fatalf("opened %v after release, ramp closed at %v", opens[len(opens)-1].Sub(released), at.Sub(released))
			}
			if _, err := q.r.JointJog(context.Background(), "update", q.in); err != nil {
				t.Fatalf("first update after a queued begin: %v", err)
			}
			if s := q.r.JointJogState(); !s.Active || s.Queued {
				t.Fatalf("state %+v", s)
			}
			noStop(t, q.f)
		})
	}
}

// A release while the press waits cancels it: nothing opens.
func TestJointJogQueuedPressReleasedNeverStarts(t *testing.T) {
	q := queueJointFake(t)
	result := make(chan error, 1)
	go func() { _, err := q.r.JointJog(context.Background(), "begin", q.in); result <- err }()
	waitFor(t, "queue", func() bool { return q.r.JointJogState().Queued })
	if _, err := q.r.JointJog(context.Background(), "end", q.in); err != nil {
		t.Fatal(err)
	}
	jointCode(t, <-result, "jog_session_stale")
	q.setRamping(false)
	time.Sleep(50 * time.Millisecond)
	if n := len(q.openTimes()); n != 1 {
		t.Fatalf("a released press opened a lane (%d opens)", n)
	}
	if s := q.r.JointJogState(); s.Active || s.Queued {
		t.Fatalf("state %+v", s)
	}
	noStop(t, q.f)
}

// A press with no refresh whose input expires before the stop finishes is
// dropped at its own deadline, not started late.
func TestJointJogQueuedPressExpiresUnrefreshed(t *testing.T) {
	q := queueJointFake(t)
	q.r.mu.Lock()
	q.r.description.MaxJogInputAgeNS = uint64(100 * time.Millisecond)
	q.r.mu.Unlock()
	pressed := time.Now()
	_, err := q.r.JointJog(context.Background(), "begin", q.in)
	jointCode(t, err, "jog_input_deadline_expired")
	if waited := time.Since(pressed); waited < 100*time.Millisecond || waited > 150*time.Millisecond {
		t.Fatalf("dropped after %v, want at the 100 ms deadline", waited)
	}
	q.setRamping(false)
	if n := len(q.openTimes()); n != 1 {
		t.Fatalf("an expired press opened a lane")
	}
	noStop(t, q.f)
}

// Intents during the wait refresh the press (latest wins) instead of hitting
// jog_session_stale; the hold begins with the latest one.
func TestJointJogQueuedPressRefreshedByIntents(t *testing.T) {
	q := queueJointFake(t)
	q.r.mu.Lock()
	q.r.description.MaxJogInputAgeNS = uint64(100 * time.Millisecond)
	q.r.mu.Unlock()
	result := make(chan error, 1)
	go func() { _, err := q.r.JointJog(context.Background(), "begin", q.in); result <- err }()
	waitFor(t, "queue", func() bool { return q.r.JointJogState().Queued })
	var last uint64
	// 150 ms of 25 ms intents: past the press's own 100 ms lifetime, inside
	// the ~192 ms stop bound.
	for start := time.Now(); time.Since(start) < 150*time.Millisecond; time.Sleep(25 * time.Millisecond) {
		last = ipcclient.HostMonotonicNS()
		if _, err := q.r.JointJog(context.Background(), "update", q.in); err != nil {
			t.Fatalf("intent during the wait: %v", err)
		}
	}
	q.setRamping(false)
	if err := <-result; err != nil {
		t.Fatal(err)
	}
	q.lane.mu.Lock()
	first := q.lane.samples[len(q.lane.samples)-1]
	q.lane.mu.Unlock()
	// The first input of the new hold is the latest intent, with its own
	// deadline: receipt + lifetime.
	if first.origin < last || first.deadline-first.origin != uint64(100*time.Millisecond) {
		t.Fatalf("first input origin %d (latest intent %d) deadline +%d", first.origin, last, first.deadline-first.origin)
	}
	noStop(t, q.f)
}

// A ramp still running past its bound is refused as before, at the bound.
func TestJointJogQueuedPressRefusedPastStopBound(t *testing.T) {
	q := queueJointFake(t)
	q.r.mu.Lock()
	stop := q.r.jogStop
	q.r.mu.Unlock()
	if stop == nil || stop.source != "halt_ramp" {
		t.Fatalf("stop record %+v", stop)
	}
	// Keep the press alive past the bound: refresh it every 25 ms.
	done := make(chan struct{})
	defer close(done)
	go func() {
		for {
			select {
			case <-done:
				return
			case <-time.After(25 * time.Millisecond):
				_, _ = q.r.JointJog(context.Background(), "update", q.in)
			}
		}
	}()
	_, err := q.r.JointJog(context.Background(), "begin", q.in)
	jointCode(t, err, "jog_mode_conflict")
	if late := time.Since(stop.at); late < stop.bound || late > stop.bound+50*time.Millisecond {
		t.Fatalf("refused %v after release, bound %v", late, stop.bound)
	}
	if n := len(q.openTimes()); n != 1 {
		t.Fatal("a press opened while native was still ramping")
	}
	noStop(t, q.f)
}

// A different axis while queued is a new hold: the press is cancelled.
func TestJointJogQueuedPressConflictingIntentCancels(t *testing.T) {
	q := queueJointFake(t)
	result := make(chan error, 1)
	go func() { _, err := q.r.JointJog(context.Background(), "begin", q.in); result <- err }()
	waitFor(t, "queue", func() bool { return q.r.JointJogState().Queued })
	other := q.in
	other.Axis = 1
	_, err := q.r.JointJog(context.Background(), "update", other)
	jointCode(t, err, "jog_mode_conflict")
	jointCode(t, <-result, "jog_session_stale")
	if n := len(q.openTimes()); n != 1 {
		t.Fatal("a cancelled press opened a lane")
	}
	noStop(t, q.f)
}

// A Cartesian press during the previous path stop starts its line at the
// held command at rest after that stop, never at a mid-stop target.
func TestCartesianPressDuringPathStopStartsFromRest(t *testing.T) {
	r, f, _, _, in := pathFake(t)
	r.mu.Lock()
	r.description.MaxJogInputAgeNS = uint64(queueTestLifetime)
	r.mu.Unlock()
	var lanes []*fakePathLane
	var lanesMu sync.Mutex
	r.pathOpen = func(context.Context, uint32, float64, float64, time.Duration) (pathJogLane, error) {
		lanesMu.Lock()
		defer lanesMu.Unlock()
		lane := &fakePathLane{obs: pathObservation{Open: true}}
		lanes = append(lanes, lane)
		return lane, nil
	}
	ctx := context.Background()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	setStatus := func(target float64, done bool, ramping bool, at uint64) {
		f.mu.Lock()
		defer f.mu.Unlock()
		f.status.TimeNS, f.status.Motion.TimeNS, f.status.Core.TimeNS = at, at, at
		f.status.Motion.Done = done
		f.status.Jog = &ipcclient.JogObservation{JogGeneration: 7, StateReason: ipcclient.JogReasonClosed}
		if ramping {
			f.status.Jog.StateReason = ipcclient.JogReasonRamping
		}
		// A new slice: returned statuses share the old one with readers.
		axes := append([]ipcclient.LogicalAxisStatus(nil), f.status.Axes...)
		for i := range axes {
			axes[i].TimeNS = at
			axes[i].LogicalTarget = .1*float64(i) + target
		}
		f.status.Axes = axes
	}
	// Mid-stop: the held command is still moving.
	setStatus(.05, false, true, 5)
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	in.Revision = r.JointJogState().Revision
	rested := make(chan time.Time, 1)
	go func() {
		time.Sleep(40 * time.Millisecond)
		setStatus(.06, false, true, 8)
		// Native closed the session; the 10 Hz motion publication still
		// reports motion with a mid-stop target.
		time.Sleep(40 * time.Millisecond)
		setStatus(.065, false, false, 9)
		time.Sleep(40 * time.Millisecond)
		setStatus(.07, true, false, 10)
		rested <- time.Now()
	}()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatalf("press during the path stop: %v", err)
	}
	began := time.Now()
	if at := <-rested; began.Before(at) {
		t.Fatalf("began %v before rest", at.Sub(began))
	}
	lanesMu.Lock()
	second := lanes[len(lanes)-1]
	n := len(lanes)
	lanesMu.Unlock()
	appends, speeds, _ := second.snapshot()
	if n != 2 || len(appends) == 0 {
		t.Fatalf("lanes %d appends %d", n, len(appends))
	}
	for j := 0; j < 6; j++ {
		if got, want := appends[0].samples[0][j], .1*float64(j)+.07; got != want {
			t.Fatalf("line starts at %v on axis %d, want the rest target %v", got, j, want)
		}
	}
	// ~120 ms of the 250 ms press lifetime used: the press is the first speed.
	if len(speeds) != 1 || math.Abs(speeds[0].speed-.2) > 1e-12 {
		t.Fatalf("speeds %+v", speeds)
	}
	if _, err := r.CartesianJog(ctx, "intent", in); err != nil {
		t.Fatalf("intent after a queued start: %v", err)
	}
	if !r.JointJogState().Active {
		t.Fatal("hold not active")
	}
	noStop(t, f)
}

// Intents sent while the released begin sets up (resolver, lane open) refresh
// the press instead of meeting a hold without input permission; the first
// speed command is the latest of them.
func TestCartesianQueuedPressIntentsDuringSetup(t *testing.T) {
	r, f, _, resolver, in := pathFake(t)
	r.mu.Lock()
	r.description.MaxJogInputAgeNS = uint64(queueTestLifetime)
	r.mu.Unlock()
	var lane *fakePathLane
	var laneMu sync.Mutex
	r.pathOpen = func(context.Context, uint32, float64, float64, time.Duration) (pathJogLane, error) {
		laneMu.Lock()
		defer laneMu.Unlock()
		lane = &fakePathLane{obs: pathObservation{Open: true}}
		return lane, nil
	}
	ctx := context.Background()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.status.Motion.Done = false
	f.status.Jog = &ipcclient.JogObservation{JogGeneration: 7, StateReason: ipcclient.JogReasonRamping}
	f.status.TimeNS, f.status.Motion.TimeNS = 5, 5
	f.mu.Unlock()
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	in.Revision = r.JointJogState().Revision
	// The next begin's resolver takes 60 ms: intents arrive during setup.
	resolver.mu.Lock()
	resolver.delay = 60 * time.Millisecond
	resolver.mu.Unlock()
	go func() {
		time.Sleep(30 * time.Millisecond)
		f.mu.Lock()
		defer f.mu.Unlock()
		f.status.Motion.Done = true
		f.status.Jog = nil
		f.status.TimeNS, f.status.Motion.TimeNS, f.status.Core.TimeNS = 10, 10, 10
		axes := append([]ipcclient.LogicalAxisStatus(nil), f.status.Axes...)
		for i := range axes {
			axes[i].TimeNS = 10
		}
		f.status.Axes = axes
	}()
	result := make(chan error, 1)
	go func() { _, err := r.CartesianJog(ctx, "start", in); result <- err }()
	intent := in
	intent.Fraction = .25
	sent := 0
	for done := false; !done; {
		select {
		case err := <-result:
			if err != nil {
				t.Fatal(err)
			}
			done = true
		case <-time.After(15 * time.Millisecond):
			if _, err := r.CartesianJog(ctx, "intent", intent); err != nil {
				t.Fatalf("intent %d while the press waited or set up: %v", sent, err)
			}
			sent++
		}
	}
	laneMu.Lock()
	l := lane
	laneMu.Unlock()
	_, speeds, _ := l.snapshot()
	// The first speed is the latest intent's (0.25 x 0.4 m/s), not the
	// press's 0.5; an intent that raced the hold's readiness adds a second.
	if sent < 4 || len(speeds) == 0 || math.Abs(speeds[0].speed-.1) > 1e-12 {
		t.Fatalf("sent %d intents; speeds %+v", sent, speeds)
	}
	noStop(t, f)
}

// Releasing a hold while its producer extends the line must not cancel that
// resolver exchange: a cancelled exchange kills the resolver process, which
// refuses every request for its 1 s restart, and so the next press.
func TestCartesianReleaseLeavesResolverExchangeRunning(t *testing.T) {
	r, _, lane, resolver, in := pathFake(t)
	entered, release := make(chan struct{}), make(chan struct{})
	cancelled := make(chan error, 1)
	var once sync.Once
	r.SetCartesianResolver(func(ctx context.Context, req CartesianResolveRequest) (CartesianResolution, error) {
		if req.Operation == "path_extend" && *req.Begin > 0 {
			blocked := false
			once.Do(func() { blocked = true })
			if blocked {
				close(entered)
				<-release
				cancelled <- ctx.Err()
			}
		}
		return resolver.resolve(ctx, req)
	})
	ctx := context.Background()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	lane.obs.Consumed = 60
	lane.mu.Unlock()
	<-entered
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	close(release)
	if err := <-cancelled; err != nil {
		t.Fatalf("release cancelled the resolver exchange: %v", err)
	}
	// Its result is dropped: nothing is appended to the ended lane.
	time.Sleep(30 * time.Millisecond)
	if appends, _, _ := lane.snapshot(); len(appends) != 1 {
		t.Fatalf("a retired hold appended geometry: %d appends", len(appends))
	}
}

// A press with less than one cadence of lifetime left is not sent: the next
// intent starts the hold, so the deadman cannot stop it right after it starts.
func TestCartesianQueuedPressNearExpiryWaitsForNextIntent(t *testing.T) {
	r, f, _, _, in := pathFake(t)
	var lane *fakePathLane
	r.pathOpen = func(context.Context, uint32, float64, float64, time.Duration) (pathJogLane, error) {
		lane = &fakePathLane{obs: pathObservation{Open: true}}
		return lane, nil
	}
	ctx := context.Background()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	f.mu.Lock()
	f.status.Motion.Done = false
	f.status.Jog = &ipcclient.JogObservation{JogGeneration: 7, StateReason: ipcclient.JogReasonRamping}
	f.status.TimeNS, f.status.Motion.TimeNS = 5, 5
	f.mu.Unlock()
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	in.Revision = r.JointJogState().Revision
	go func() {
		// 70 ms of the 100 ms lifetime: 30 ms left, under one 50 ms cadence.
		time.Sleep(70 * time.Millisecond)
		f.mu.Lock()
		defer f.mu.Unlock()
		f.status.Motion.Done = true
		f.status.Jog = nil
		f.status.TimeNS, f.status.Motion.TimeNS, f.status.Core.TimeNS = 10, 10, 10
		axes := append([]ipcclient.LogicalAxisStatus(nil), f.status.Axes...)
		for i := range axes {
			axes[i].TimeNS = 10
		}
		f.status.Axes = axes
	}()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	if _, speeds, _ := lane.snapshot(); len(speeds) != 0 {
		t.Fatalf("a press with 30 ms left was sent: %+v", speeds)
	}
	if _, err := r.CartesianJog(ctx, "intent", in); err != nil {
		t.Fatal(err)
	}
	if _, speeds, _ := lane.snapshot(); len(speeds) != 1 {
		t.Fatalf("the next intent did not start the hold: %+v", speeds)
	}
	noStop(t, f)
}

// Clients keep several intents in flight; two that reach OLP together can
// take the lock out of order. An intent overtaken by a newer one is dropped,
// so origins reach native in order (an origin going back was refused
// jog_invalid_timing and ended slow-link composed holds, 2026-10-07).
func TestOvertakenIntentIsDropped(t *testing.T) {
	t.Run("joint", func(t *testing.T) {
		r, _, lane, in := jointFake(t)
		ctx := context.Background()
		if _, err := r.JointJog(ctx, "begin", in); err != nil {
			t.Fatal(err)
		}
		older := ipcclient.HostMonotonicNS()
		newer := older + uint64(time.Millisecond)
		if err := r.jointJog(ctx, "update", in, time.Now(), newer); err != nil {
			t.Fatal(err)
		}
		if err := r.jointJog(ctx, "update", in, time.Now(), older); err != nil {
			t.Fatalf("an overtaken intent ended the hold: %v", err)
		}
		lane.mu.Lock()
		defer lane.mu.Unlock()
		if n := len(lane.samples); n != 2 || lane.samples[1].origin != newer {
			t.Fatalf("samples %+v", lane.samples)
		}
		if s := r.JointJogState(); !s.Active || !s.PipelinedIntents {
			t.Fatalf("state %+v", s)
		}
	})
	t.Run("path", func(t *testing.T) {
		r, _, lane, _, in := pathFake(t)
		ctx := context.Background()
		if _, err := r.CartesianJog(ctx, "start", in); err != nil {
			t.Fatal(err)
		}
		joint := JointJogIntent{TargetID: in.TargetID, SessionID: in.SessionID, Revision: in.Revision, Axis: 0, Direction: 1, Fraction: .25, cartesian: &in}
		older := ipcclient.HostMonotonicNS()
		newer := older + uint64(time.Millisecond)
		if err := r.jointJog(ctx, "update", joint, time.Now(), newer); err != nil {
			t.Fatal(err)
		}
		if err := r.jointJog(ctx, "update", joint, time.Now(), older); err != nil {
			t.Fatalf("an overtaken intent ended the hold: %v", err)
		}
		_, speeds, _ := lane.snapshot()
		if len(speeds) != 2 || speeds[1].source != newer {
			t.Fatalf("speeds %+v", speeds)
		}
		if !r.JointJogState().Active {
			t.Fatal("hold ended")
		}
	})
}
