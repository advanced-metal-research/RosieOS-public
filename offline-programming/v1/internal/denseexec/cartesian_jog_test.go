//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"math"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// fakePathLane records what the hold streams and serves a scripted native
// observation. Consumption follows the speed command so the producer's
// look-ahead can be checked.
type fakePathLane struct {
	mu       sync.Mutex
	appends  []pathAppend
	speeds   []pathSpeed
	ends     int
	obs      pathObservation
	received uint64
}

type pathAppend struct {
	first   uint64
	samples [][]float64
	caps    []float64
	end     bool
	reason  uint32
}

type pathSpeed struct {
	speed  float64
	source uint64
}

func (l *fakePathLane) Generation() uint64 { return 7 }
func (l *fakePathLane) Append(_ context.Context, first uint64, samples [][]float64, caps []float64, end bool, reason uint32) error {
	l.mu.Lock()
	defer l.mu.Unlock()
	if first != l.received {
		return fmt.Errorf("append at %d, have %d", first, l.received)
	}
	l.appends = append(l.appends, pathAppend{first, samples, caps, end, reason})
	l.received += uint64(len(samples))
	l.obs.Received = l.received
	if end {
		l.obs.Ended, l.obs.EndReason = true, reason
		l.obs.PlayableEnd = l.received - 1
	} else if l.received >= 3 {
		l.obs.PlayableEnd = l.received - 3
	}
	return nil
}
func (l *fakePathLane) SetSpeed(speed float64, source uint64) error {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.speeds = append(l.speeds, pathSpeed{speed, source})
	return nil
}
func (l *fakePathLane) Observe(context.Context) (pathObservation, error) {
	l.mu.Lock()
	defer l.mu.Unlock()
	return l.obs, nil
}
func (l *fakePathLane) End(context.Context) error {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.ends++
	l.obs.Open = false
	return nil
}

func (l *fakePathLane) snapshot() (appends []pathAppend, speeds []pathSpeed, ends int) {
	l.mu.Lock()
	defer l.mu.Unlock()
	return append([]pathAppend(nil), l.appends...), append([]pathSpeed(nil), l.speeds...), l.ends
}

// pathDelta is the fake resolver's joint step per sample: a straight joint line.
var pathDelta = [6]float64{.001, -.0005, .0002, 0, 0, 0}

type pathResolverFake struct {
	mu       sync.Mutex
	requests []CartesianResolveRequest
	endAt    uint64 // first failing sample, 0 = never
	endWhy   string
	// scaleAt, when set, is the resolver's singularity scale at a sample.
	scaleAt func(index uint64) float64
	delay   time.Duration
}

func (p *pathResolverFake) resolve(ctx context.Context, in CartesianResolveRequest) (CartesianResolution, error) {
	p.mu.Lock()
	p.requests = append(p.requests, in)
	endAt, endWhy, delay, scaleAt := p.endAt, p.endWhy, p.delay, p.scaleAt
	p.mu.Unlock()
	if delay > 0 {
		select {
		case <-ctx.Done():
			return CartesianResolution{}, ctx.Err()
		case <-time.After(delay):
		}
	}
	switch in.Operation {
	case "fk":
		// Fake kinematics: TCP position = the first three joints, no rotation.
		return CartesianResolution{Accepted: true, Transform: []float64{1, 0, 0, in.Pose[0], 0, 1, 0, in.Pose[1], 0, 0, 1, in.Pose[2]}}, nil
	case "path_extend":
		out := CartesianResolution{Accepted: true, Spacing: .001}
		for i := uint64(0); i < uint64(in.Count); i++ {
			index := *in.Begin + i
			if endAt != 0 && index >= endAt {
				out.EndReason = endWhy
				break
			}
			var q [6]float64
			for j := range q {
				q[j] = in.Pose[j] + float64(index)*pathDelta[j]
			}
			out.Waypoints = append(out.Waypoints, q)
			scale := 1.0
			if scaleAt != nil {
				scale = scaleAt(index)
			}
			out.SampleScales = append(out.SampleScales, scale)
		}
		return out, nil
	}
	return CartesianResolution{Reason: "cartesian_input_invalid"}, nil
}

func (p *pathResolverFake) snapshot() []CartesianResolveRequest {
	p.mu.Lock()
	defer p.mu.Unlock()
	return append([]CartesianResolveRequest(nil), p.requests...)
}

func pathFake(t *testing.T) (*RTCoreController, *fakeRTCore, *fakePathLane, *pathResolverFake, CartesianJogIntent) {
	t.Helper()
	r, f, _ := rtFake(t)
	f.mu.Lock()
	for i := range f.description.Axes {
		a := &f.description.Axes[i]
		a.ID, a.Index, a.PositionUnit = fmt.Sprintf("J%d", i+1), uint32(i), "rad"
		a.MinPosition, a.MaxPosition, a.MaxVelocity = -3, 3, 1.396
		f.status.Axes[i].TimeNS = 1
		// The held command differs from the measured pose: the line must start
		// at the held command (R4/R6).
		f.status.Axes[i].LogicalTarget = .1 * float64(i)
		f.status.Axes[i].LogicalPosition = .1*float64(i) + .003
	}
	f.description.PathJog = &ipcclient.PathJogCapability{MaxSamples: 1024, MaxChunk: 16, MaxSampleStepS: .02}
	f.status.Core.TimeNS = 1
	f.status.Motion.Done = true
	f.mu.Unlock()
	r.cfg.CartesianModel = "rosie_1420_v1"
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	lane := &fakePathLane{obs: pathObservation{Open: true}}
	r.pathOpen = func(_ context.Context, mask uint32, spacing, maxSpeed float64, lifetime time.Duration) (pathJogLane, error) {
		if mask != 63 || spacing != .001 || maxSpeed != .4 || lifetime != 100*time.Millisecond {
			t.Errorf("path open mask=%d spacing=%v max=%v lifetime=%v", mask, spacing, maxSpeed, lifetime)
		}
		return lane, nil
	}
	resolver := &pathResolverFake{}
	r.SetCartesianResolver(resolver.resolve)
	in := CartesianJogIntent{TargetID: "bench", SessionID: r.Status(context.Background()).Session.ID, Frame: "base", Twist: [6]float64{.4}, Fraction: .5}
	return r, f, lane, resolver, in
}

func waitFor(t *testing.T, what string, done func() bool) {
	t.Helper()
	deadline := time.Now().Add(2 * time.Second)
	for !done() {
		if time.Now().After(deadline) {
			t.Fatalf("timed out waiting for %s", what)
		}
		time.Sleep(2 * time.Millisecond)
	}
}

func TestCartesianPathHoldStreamsFrozenLineAndSpeeds(t *testing.T) {
	r, _, lane, resolver, in := pathFake(t)
	ctx := context.Background()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	appends, speeds, _ := lane.snapshot()
	if len(appends) != 1 || appends[0].first != 0 || len(appends[0].samples) != pathAheadSamples || appends[0].end {
		t.Fatalf("first append %+v", appends)
	}
	// Sample 0 is the held command exactly, laid out on the Describe axes.
	for j := 0; j < 6; j++ {
		if got := appends[0].samples[0][j]; got != .1*float64(j) {
			t.Fatalf("sample 0 axis %d = %v, want held command %v", j, got, .1*float64(j))
		}
	}
	if len(appends[0].samples[0]) != 9 {
		t.Fatalf("sample width %d", len(appends[0].samples[0]))
	}
	// The press is the first speed command: fraction x box speed (0.5 x 0.4 m/s).
	if len(speeds) != 1 || math.Abs(speeds[0].speed-.2) > 1e-12 {
		t.Fatalf("speeds %+v", speeds)
	}
	// Joystick: a fraction change within the hold is a new speed command.
	in.Fraction = .25
	if _, err := r.CartesianJog(ctx, "intent", in); err != nil {
		t.Fatal(err)
	}
	if _, speeds, _ = lane.snapshot(); len(speeds) != 2 || math.Abs(speeds[1].speed-.1) > 1e-12 || speeds[1].source <= speeds[0].source {
		t.Fatalf("speeds %+v", speeds)
	}
	// A new axis, frame or direction is a new hold, never a mid-hold change.
	for _, change := range []func(*CartesianJogIntent){
		func(c *CartesianJogIntent) { c.Twist = [6]float64{0, .4} },
		func(c *CartesianJogIntent) { c.Twist = [6]float64{-.4} },
		func(c *CartesianJogIntent) { c.Frame = "tool" },
	} {
		other := in
		change(&other)
		_, err := r.CartesianJog(ctx, "intent", other)
		jointCode(t, err, "jog_mode_conflict")
	}
	s := r.JointJogState()
	if !s.Active || !s.FractionUpdates {
		t.Fatalf("state %+v", s)
	}
	// Every resolver request names the frozen line at the held command.
	for _, req := range resolver.snapshot() {
		held := [6]float64{}
		for j := range held {
			held[j] = .1 * float64(j)
		}
		if req.Operation == "path_extend" && (req.Pose != held || *req.Axis != 0 || *req.Direction != 1 || req.Frame != "base") {
			t.Fatalf("request %+v", req)
		}
	}
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	if _, _, ends := lane.snapshot(); ends != 1 {
		t.Fatalf("ends=%d", ends)
	}
	if s := r.JointJogState(); s.Active || s.Revision != 1 {
		t.Fatalf("state %+v", s)
	}
}

func TestCartesianPathHoldMeasuredPoseNeverSteers(t *testing.T) {
	r, f, lane, resolver, in := pathFake(t)
	ctx := context.Background()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	// The measured pose wanders; consumption advances so the producer extends.
	f.mu.Lock()
	for i := range f.status.Axes {
		f.status.Axes[i].LogicalPosition += .05
	}
	f.mu.Unlock()
	lane.mu.Lock()
	lane.obs.Consumed, lane.obs.PathS = 60, .06
	lane.mu.Unlock()
	waitFor(t, "extension", func() bool {
		lane.mu.Lock()
		defer lane.mu.Unlock()
		return lane.received >= 60+pathAheadSamples
	})
	appends, speeds, _ := lane.snapshot()
	// Contiguous samples on the same straight joint line, seeded from the
	// previous sample, regardless of the measured pose.
	next := uint64(0)
	for _, a := range appends {
		if a.first != next {
			t.Fatalf("append at %d want %d", a.first, next)
		}
		for i, row := range a.samples {
			index := float64(a.first) + float64(i)
			for j := 0; j < 6; j++ {
				if want := .1*float64(j) + index*pathDelta[j]; math.Abs(row[j]-want) > 1e-12 {
					t.Fatalf("sample %v axis %d = %v want %v", index, j, row[j], want)
				}
			}
		}
		next += uint64(len(a.samples))
	}
	if next < 60+pathAheadSamples {
		t.Fatalf("look-ahead %d samples beyond consumption 60", next-60)
	}
	for _, req := range resolver.snapshot() {
		if req.Operation != "path_extend" || *req.Begin == 0 {
			continue
		}
		var want [6]float64
		for j := range want {
			want[j] = .1*float64(j) + float64(*req.Begin-1)*pathDelta[j]
		}
		if req.Seed == nil || math.Abs(req.Seed[0]-want[0]) > 1e-12 || req.Pose[0] != 0 {
			t.Fatalf("seed %+v pose %v at begin %d", req.Seed, req.Pose, *req.Begin)
		}
	}
	// Geometry never carries or renews a deadline: still one speed command.
	if len(speeds) != 1 {
		t.Fatalf("speeds %+v", speeds)
	}
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
}

func TestCartesianPathHoldNoTimerRenewsTheDeadman(t *testing.T) {
	r, _, lane, _, in := pathFake(t)
	if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
		t.Fatal(err)
	}
	// No operator input for longer than the lifetime: OLP sends no speed
	// command of its own and retires the hold (native stops on the path).
	waitFor(t, "expiry and native End receipt", func() bool {
		// Retirement is published before the asynchronous lane End completes.
		// Observe both receipts rather than racing that deliberately ordered handoff.
		_, _, ends := lane.snapshot()
		return !r.JointJogState().Active && ends == 1
	})
	_, speeds, ends := lane.snapshot()
	if len(speeds) != 1 || ends != 1 {
		t.Fatalf("speeds=%d ends=%d", len(speeds), ends)
	}
	if s := r.JointJogState(); s.Reason != "jog_input_deadline_expired" {
		t.Fatalf("state %+v", s)
	}
}

func TestCartesianHoldRefusedWithoutPathJog(t *testing.T) {
	r, f, lane, _, in := pathFake(t)
	f.mu.Lock()
	f.description.PathJog = nil
	f.mu.Unlock()
	r.mu.Lock()
	r.description.PathJog = nil
	r.mu.Unlock()
	_, err := r.CartesianJog(context.Background(), "start", in)
	jointCode(t, err, "cartesian_hold_unavailable")
	if a, _, _ := lane.snapshot(); len(a) != 0 {
		t.Fatalf("appends %+v", a)
	}
	if s := r.JointJogState(); s.Active || s.FractionUpdates {
		t.Fatalf("state %+v", s)
	}
}

func TestCartesianPathHoldRefusesDiagonals(t *testing.T) {
	r, _, _, _, in := pathFake(t)
	in.Twist = [6]float64{.2, .2}
	_, err := r.CartesianJog(context.Background(), "start", in)
	jointCode(t, err, "cartesian_input_invalid")
}

func TestCartesianPathHoldEndsWithTheResolverReason(t *testing.T) {
	r, _, lane, resolver, in := pathFake(t)
	resolver.endAt, resolver.endWhy = 150, "joint_limit"
	if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	lane.obs.Consumed = 80
	lane.mu.Unlock()
	waitFor(t, "path end", func() bool {
		a, _, _ := lane.snapshot()
		return len(a) > 0 && a[len(a)-1].end
	})
	appends, _, _ := lane.snapshot()
	last := appends[len(appends)-1]
	if last.reason != pathEndCodes["joint_limit"] || last.first+uint64(len(last.samples)) != 150 {
		t.Fatalf("last append %+v", last)
	}
	// Native stopped at the end of the line: the hold retires with that reason.
	lane.mu.Lock()
	lane.obs.PathSpeed, lane.obs.PathS = 0, float64(lane.obs.PlayableEnd)*.001
	lane.mu.Unlock()
	waitFor(t, "retirement", func() bool { return !r.JointJogState().Active })
	if s := r.JointJogState(); s.Reason != "joint_limit" {
		t.Fatalf("state %+v", s)
	}
}

func TestCartesianPathHoldNativeExpiryRetires(t *testing.T) {
	r, _, lane, _, in := pathFake(t)
	if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	lane.obs.Open, lane.obs.StopReason = false, 2
	lane.mu.Unlock()
	waitFor(t, "retirement", func() bool { return !r.JointJogState().Active })
	if s := r.JointJogState(); s.Reason != "jog_input_deadline_expired" {
		t.Fatalf("state %+v", s)
	}
}

func TestCartesianPathHoldSlowResolverIsRefused(t *testing.T) {
	r, _, lane, resolver, in := pathFake(t)
	// 100 samples in 300 ms is 333 samples/s; 400 mm/s needs 800 samples/s.
	resolver.delay = 300 * time.Millisecond
	_, err := r.CartesianJog(context.Background(), "start", in)
	jointCode(t, err, "cartesian_resolver_too_slow")
	if a, _, _ := lane.snapshot(); len(a) != 0 {
		t.Fatalf("appends %+v", a)
	}
}

func TestCartesianPathHoldStartsFromRestOnly(t *testing.T) {
	r, f, _, _, in := pathFake(t)
	f.mu.Lock()
	f.status.Motion.Done = false
	f.mu.Unlock()
	waitFor(t, "observation", func() bool {
		r.mu.Lock()
		defer r.mu.Unlock()
		return r.observation != nil && !r.observation.Motion.Done
	})
	_, err := r.CartesianJog(context.Background(), "start", in)
	jointCode(t, err, "jog_mode_conflict")
}
