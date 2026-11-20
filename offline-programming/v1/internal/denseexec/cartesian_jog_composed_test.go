//go:build linux

package denseexec

import (
	"context"
	"math/rand/v2"
	"os"
	"path/filepath"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/cartesianpath"
	"rosieos/rt-core/sdk/control"
)

// Count quantisation of the simulation fixture's status target: 131072 counts
// per revolution with no gear (simulation-program.json) is 48 urad per count,
// 0.0028 deg at the wrist and ~0.05 mm at 1 m reach. Two counts bound the
// status-sampled pose; the path itself is bounded to 1e-8 rad by
// rt-core/tests/path_jog_tests.cpp and to 0.02 mm by the the 1430 cell harness.
const (
	composedOffLineMM = 0.1
	composedOrientDeg = 0.006
)

// The real resolver (path_extend), the public SDK path session, rt-control and
// the native simulator with the path lane: a held base +X line moves the arm,
// release stops it ON the line (status-sampled commanded pose within the
// fixture's count quantisation of the line through the start), STOP retires a
// hold, and a cell without jog_jerk (no path_jog capability) refuses holds.
func TestCartesianJogComposed(t *testing.T) { runCartesianPathComposed(t, false, nil) }

// The remote variant runs the same hold over the real WSS jog stream with
// production PKI: the transport the 1430 cell uses (OLP on example-relay, adapter on the
// RevPi). It dials its own TLS client: until 2026-10-07 it was handed the
// local observer, so its path lane was the Unix socket and WSS went untested.
func TestCartesianJogComposedRemote(t *testing.T) { runCartesianPathComposed(t, true, nil) }

// the 1430 cell 2026-10-07: with the warm resolver (17 ms) the press reached the
// adapter stamped before the WSS Begin sample; every remote hold ended
// jog_clock_moved_backwards within 61 ms with zero travel and the cell
// disarmed. Locally native refused the same press as InvalidTiming. 20-50 ms
// of jittered resolver latency (unverified stimulus around the measured
// 17 ms) keeps setup inside the 250 ms input lifetime, so the press is sent
// and must start the path by itself.
func TestCartesianJogComposedSlowResolver(t *testing.T) {
	runCartesianPathComposed(t, false, jitteredResolverLatency)
}

func TestCartesianJogComposedRemoteSlowResolver(t *testing.T) {
	runCartesianPathComposed(t, true, jitteredResolverLatency)
}

func jitteredResolverLatency() time.Duration {
	return 20*time.Millisecond + rand.N(30*time.Millisecond)
}

func runCartesianPathComposed(t *testing.T, remote bool, latency func() time.Duration) {
	withJerk := func(config map[string]any) { config["jog_jerk"] = map[string]any{"jerk_rad_s3": 150} }
	c, d, root, _, cfg := rtComposedTransportFixture(t, remote, withJerk)
	if d.PathJog == nil {
		t.Fatal("simulator with jog_jerk does not advertise path_jog")
	}
	repo := filepath.Dir(root)
	binary := os.Getenv("OLP_CARTESIAN_RESOLVER")
	if binary == "" {
		binary = filepath.Join(root, "build/cartmachine/bin/robot-v4-cartesiand")
	}
	cfg.CartesianModel = "rosie_1420_v1"
	var injected RTCoreClient = c
	if remote {
		injected = nil
	}
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:cartesian-test"}, injected)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	resolver := NewCartesianProcess(binary, repo, "rosie_1420_v1")
	t.Cleanup(resolver.Close)
	resolve := resolver.Resolve
	if latency != nil {
		resolve = func(ctx context.Context, in CartesianResolveRequest) (CartesianResolution, error) {
			select {
			case <-ctx.Done():
				return CartesianResolution{}, ctx.Err()
			case <-time.After(latency()):
			}
			return resolver.Resolve(ctx, in)
		}
	}
	r.SetCartesianResolver(resolve)
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	before, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	var start [6]float64
	for j := range start {
		start[j] = before.Axes[j].LogicalTarget
	}
	origin, err := resolver.Resolve(ctx, FKRequest("rosie_1420_v1", start))
	if err != nil || len(origin.Transform) != 12 {
		t.Fatalf("fk %v %+v", err, origin)
	}
	in := CartesianJogIntent{TargetID: "bench", SessionID: r.Status(ctx).Session.ID, Revision: r.JointJogState().Revision, Frame: "base", Twist: [6]float64{.05}, Fraction: .5}
	pressed := time.Now()
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	if latency != nil {
		// The press is the first speed command: with no further input the
		// path must start inside the press's own 250 ms lifetime, and the
		// hold must not end on a clock refusal.
		for s := r.JointJogState(); s.PathSpeed <= 0; s = r.JointJogState() {
			if !s.Active || time.Since(pressed) > 200*time.Millisecond {
				t.Fatalf("the press did not start the path in %v: %+v", time.Since(pressed), s)
			}
			time.Sleep(2 * time.Millisecond)
		}
	}
	ticker := time.NewTicker(50 * time.Millisecond)
	defer ticker.Stop()
	deadline := time.After(3 * time.Second)
	// Unverified fixture bound: hold until the commanded TCP has moved 10 mm.
	// The first intent follows Start at once, as the browser's cadence does
	// (its timer runs while Start is in flight).
	moved := 0.0
	for first := true; moved < 10; first = false {
		if !first {
			select {
			case <-deadline:
				t.Fatalf("path hold moved %.3f mm in 3 s", moved)
			case <-ticker.C:
			}
		}
		if _, err := r.CartesianJog(ctx, "intent", in); err != nil {
			t.Fatal(err)
		}
		s, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		var q [6]float64
		for j := range q {
			q[j] = s.Axes[j].LogicalTarget
		}
		fk, err := resolver.Resolve(ctx, FKRequest("rosie_1420_v1", q))
		if err != nil {
			t.Fatal(err)
		}
		off, ori, along, ok := cartesianpath.LineDeviation(origin.Transform, fk.Transform, "base", 0, 1)
		if !ok || off > composedOffLineMM || ori > composedOrientDeg {
			t.Fatalf("commanded pose off the line: %.4f mm %.5f deg", off, ori)
		}
		moved = along
	}
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	rtAwait(t, func() bool {
		s, err := c.Status(ctx)
		return err == nil && s.Jog != nil && s.Jog.Open == 0 && s.Motion.Done
	})
	after, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	var q [6]float64
	for j := range q {
		q[j] = after.Axes[j].LogicalTarget
	}
	fk, err := resolver.Resolve(ctx, FKRequest("rosie_1420_v1", q))
	if err != nil {
		t.Fatal(err)
	}
	off, ori, along, _ := cartesianpath.LineDeviation(origin.Transform, fk.Transform, "base", 0, 1)
	t.Logf("released after %.2f mm: stopped at %.3f mm, off-line %.6f mm, orientation %.6f deg", moved, along, off, ori)
	if off > composedOffLineMM || ori > composedOrientDeg || along < moved {
		t.Fatalf("stop left the line: along %.3f off %.4f mm ori %.5f deg", along, off, ori)
	}
	if after.Core.Armed != 1 {
		t.Fatal("a released hold disarmed the cell")
	}
	for i := 6; i < len(before.Axes); i++ {
		if after.Axes[i].PositionCounts != before.Axes[i].PositionCounts {
			t.Fatalf("unselected axis %d moved", i)
		}
	}
	if s := r.JointJogState(); s.Active || !s.FractionUpdates || s.Reason != "" {
		t.Fatalf("state %+v", s)
	}

	// STOP retires a live hold through the controller's Stop fence. A new
	// hold starts from rest as OLP observes it (its 50 ms status poll).
	rtAwait(t, func() bool {
		r.mu.Lock()
		defer r.mu.Unlock()
		return r.observation != nil && r.observation.Motion.Done
	})
	in.Revision = r.JointJogState().Revision
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	if r.JointJogState().Active {
		t.Fatal("STOP did not retire the hold")
	}
	rtAwait(t, func() bool { s, err := c.Status(ctx); return err == nil && s.Core.Armed == 0 && s.Motion.Done })
}

// A cell whose machine omits jog_jerk does not advertise path_jog: holds are
// refused (no velocity-hold fallback), steps are unaffected.
func TestCartesianHoldRefusedWithoutJogJerkComposed(t *testing.T) {
	c, d, _, socket := rtComposedFixture(t)
	if d.PathJog != nil {
		t.Skip("simulation fixture configures jog_jerk")
	}
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511, CartesianModel: "rosie_1420_v1"}, Config{ControllerID: "offline-programming:cartesian-refused"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	r.SetCartesianResolver(func(context.Context, CartesianResolveRequest) (CartesianResolution, error) {
		return CartesianResolution{}, nil
	})
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	in := CartesianJogIntent{TargetID: "bench", SessionID: r.Status(ctx).Session.ID, Revision: r.JointJogState().Revision, Frame: "base", Twist: [6]float64{.05}, Fraction: .5}
	_, err = r.CartesianJog(ctx, "start", in)
	jointCode(t, err, "cartesian_hold_unavailable")
	s, err := c.Status(ctx)
	if err != nil || s.Core.Armed != 1 {
		t.Fatalf("refusal changed arm state: %+v %v", s.Core, err)
	}
}
