//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"math"
	"rosieos/rt-core/ipcclient"
	"testing"
	"time"

	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"rosieos/offline-programming/v1/internal/densejointtraj"
)

// Actual SDK, adapter, path lane and native simulated servo. No robot connection.
func TestTraceComposed(t *testing.T)       { runTraceComposed(t, false) }
func TestTraceComposedRemote(t *testing.T) { runTraceComposed(t, true) }

func runTraceComposed(t *testing.T, remote bool) {
	c, d, _, _, cfg := rtComposedTransportFixture(t, remote, func(m map[string]any) {
		m["axes"] = m["axes"].([]any)[:6]
		// Fine synthetic encoder resolution lets this test resolve curve
		// error below 1e-7 rad; it is not a real machine configuration.
		for _, v := range m["axes"].([]any) {
			a := v.(map[string]any)
			a["wire_revs_per_axis_rev"] = 1000
			a["limits"].(map[string]any)["max_velocity"] = .1
		}
		m["jog_jerk"] = map[string]any{"jerk_rad_s3": 150}
	})
	cfg.CartesianModel = "rosie_1420_v1"
	var injected RTCoreClient = c
	if remote {
		injected = nil
	}
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:trace-test"}, injected)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	r.SetCartesianResolver(func(context.Context, CartesianResolveRequest) (CartesianResolution, error) {
		return CartesianResolution{}, nil
	})
	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()
	if _, err = r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	before, err := c.Status(ctx)
	if err != nil {
		t.Fatal(err)
	}
	// Use the same validated dense container as unit tests, with actual native
	// coordinate identity and a smooth, small six-axis fixture path.
	fake, _, _, intent := traceFixture(t)
	raw, err := fake.source.cfg.FetchBlob(ctx, intent.Digest)
	if err != nil {
		t.Fatal(err)
	}
	plan, err := densejointtraj.Decode(raw)
	if err != nil {
		t.Fatal(err)
	}
	start := plan.Header.StartState
	start.CoordinateIdentity = fmt.Sprintf("%s:%d:%d", before.Core.ConfigurationSHA256, before.Core.ConfigurationEpoch, before.Core.HomeEpoch)
	start.ObservationIdentity = before.AdapterIncarnation + ":" + before.DaemonIncarnation
	for j := range d.Axes {
		start.JointNames[j] = d.Axes[j].ID
		start.JointValues[j] = before.Axes[j].LogicalTarget
	}
	plan.Segments[0].Samples = make([]densejointtraj.Sample, 201)
	for i := range plan.Segments[0].Samples {
		s := &plan.Segments[0].Samples[i]
		s.TimeS = float64(i) * .01
		for j := 0; j < 6; j++ {
			s.QRad[j] = start.JointValues[j]
		}
		s.QRad[0] += .02 * float64(i) / 200
		s.QRad[1] += .005 * math.Pow(float64(i)/200, 2)
	}
	raw, err = encodeTraceFixture(plan)
	if err != nil {
		t.Fatal(err)
	}
	decoded, err := densejointtraj.Decode(raw)
	if err != nil {
		t.Fatal(err)
	}
	r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return raw, nil }
	intent = TraceIntent{TargetID: "bench", SessionID: r.Status(ctx).Session.ID, Revision: r.JointJogState().Revision, Fraction: .5, InputSequence: 1, Digest: decoded.Header.TrajectoryDigest, Plan: &LoadRequest{Identity: fixtureIdentity(decoded), DryRun: true}}
	if _, err := r.Trace(ctx, "prepare", intent); err != nil {
		t.Fatal(err)
	}
	intent.Plan = nil // every press exercises the prepared server copy
	checkPath := func() {
		t.Helper()
		s, err := c.Status(ctx)
		if err != nil {
			t.Fatal(err)
		}
		x := s.Axes[0].LogicalTarget - start.JointValues[0]
		y := s.Axes[1].LogicalTarget - start.JointValues[1]
		if math.Abs(y-.005*math.Pow(x/.02, 2)) > 1e-7 {
			t.Fatalf("left original curve: x=%g y=%g", x, y)
		}
	}
	begin := func(reverse bool) {
		intent.Revision = r.JointJogState().Revision
		intent.InputSequence = 1
		intent.Reverse = reverse
		started := time.Now()
		if _, err := r.Trace(ctx, "begin", intent); err != nil {
			t.Fatalf("begin reverse=%v: %v", reverse, err)
		}
		if elapsed := time.Since(started); elapsed > 500*time.Millisecond {
			t.Fatalf("prepared begin took %s", elapsed)
		}
	}
	tick := func() {
		t.Helper()
		intent.InputSequence++
		update := intent
		update.Plan = nil
		if _, err := r.Trace(ctx, "update", update); err != nil {
			t.Fatal(err)
		}
		checkPath()
		time.Sleep(20 * time.Millisecond)
	}
	begin(false)
	moved := time.Now()
	for {
		tick()
		s, _ := c.Status(ctx)
		if s.Axes[0].LogicalTarget > start.JointValues[0]+.002 {
			break
		}
		if time.Since(moved) > 3*time.Second {
			t.Fatal("no trace motion")
		}
	}
	if _, err = r.Trace(ctx, "end", intent); err != nil {
		t.Fatal(err)
	}
	closedAt := ipcclient.HostMonotonicNS()
	rtAwait(t, func() bool {
		s, e := c.Status(ctx)
		return e == nil && s.Motion.Done && s.Jog != nil && s.Jog.Open == 0 && s.Axes[0].TimeNS > closedAt
	})
	stopped, _ := c.Status(ctx)
	time.Sleep(100 * time.Millisecond)
	still, _ := c.Status(ctx)
	if stopped.Axes[0].LogicalTarget != still.Axes[0].LogicalTarget {
		t.Fatalf("release did not settle: before=%g after=%g times=%d/%d", stopped.Axes[0].LogicalTarget, still.Axes[0].LogicalTarget, stopped.Axes[0].TimeNS, still.Axes[0].TimeNS)
	}
	if still.Core.Armed != 1 {
		t.Fatal("D-pad release disarmed the grant")
	}
	// Reverse while between points, then replay the original forward path.
	for _, reverse := range []bool{true, false} {
		begin(reverse)
		deadline := time.Now().Add(8 * time.Second)
		for r.JointJogState().Active {
			tick()
			if time.Now().After(deadline) {
				r.mu.Lock()
				p := r.jog.trace
				r.mu.Unlock()
				o, e := p.lane.Observe(ctx)
				s, _ := c.Status(ctx)
				t.Fatalf("trace never reached endpoint: obs=%+v err=%v rows=%d spacing=%g q=%+v tolerances=%+v", o, e, len(p.rows), p.spacing, s.Axes, d.Axes)
			}
		}
		state := r.JointJogState()
		if state.Reason != "trace_complete" {
			t.Fatalf("endpoint: %+v", state)
		}
		after, _ := c.Status(ctx)
		want := start.JointValues[0]
		if !reverse {
			want += .02
		}
		if math.Abs(after.Axes[0].LogicalTarget-want) > d.Axes[0].CompletionTolerance {
			t.Fatalf("endpoint %g want %g", after.Axes[0].LogicalTarget, want)
		}
		for i := 0; i < 8; i++ {
			tick()
		}
		if r.JointJogState().Active {
			t.Fatal("held completion restarted")
		}
		// Repeating the just-completed endpoint must not open a microscopic
		// native path or fault the grant. This matches the captured double
		// reverse request on the second 1430 cell and runs against the real simulated core.
		begin(reverse)
		if state := r.JointJogState(); state.Active || state.Reason != "trace_complete" {
			t.Fatalf("repeat endpoint: %+v", state)
		}
		afterRepeat, _ := c.Status(ctx)
		if afterRepeat.Core.Armed != 1 {
			t.Fatal("endpoint acknowledgement disarmed")
		}
	}
}

func encodeTraceFixture(plan *densejointtraj.Plan) ([]byte, error) {
	h := plan.Header
	h.Segments = nil
	h.Sampling.TotalSampleCount = 0
	h.Sampling.TotalDurationS = 0
	var body []byte
	for i, seg := range plan.Segments {
		samples := seg.Samples
		block := densejointtraj.EncodeBlock(samples)
		sum := sha256.Sum256(block)
		duration := samples[len(samples)-1].TimeS
		h.Segments = append(h.Segments, densejointtraj.SegmentMeta{Index: i, Kind: seg.Kind, SourceID: seg.SourceID,
			SampleCount: len(samples), DurationS: duration, Block: densejointtraj.BlockMeta{ByteOffset: len(body), ByteLength: len(block), SHA256: "sha256:" + hex.EncodeToString(sum[:])}})
		h.Sampling.TotalSampleCount += len(samples)
		h.Sampling.TotalDurationS += duration
		body = append(body, block...)
	}
	h.TrajectoryDigest = densejointtraj.TrajectoryDigest(h.Axes.AxisMask, h.Sampling.DtS, h.Limits.MaxAbsQdRadS, plan.Segments)
	header, err := json.Marshal(h)
	if err != nil {
		return nil, err
	}
	return append(append(binary.BigEndian.AppendUint32(nil, uint32(len(header))), header...), body...), nil
}
