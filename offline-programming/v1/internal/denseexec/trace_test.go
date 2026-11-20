//go:build linux

package denseexec

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"strings"
	"testing"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

func TestTraceRemainder(t *testing.T) {
	raw := [][]float64{{0, 0}, {.001, .002}, {.002, .004}, {.003, .006}}
	for _, reverse := range []bool{false, true} {
		out, err := traceRemainder(raw, []float64{.0015, .003}, []float64{1e-6, 1e-6}, reverse)
		if err != nil {
			t.Fatal(err)
		}
		want := raw[3]
		if reverse {
			want = raw[0]
		}
		for j := range want {
			if math.Abs(out[len(out)-1][j]-want[j]) > 1e-12 {
				t.Fatalf("wrong endpoint %v", out)
			}
		}
		if out[0][0] != .0015 || out[0][1] != .003 {
			t.Fatal("start changed")
		}
	}
	if _, err := traceRemainder(raw, []float64{.0015, .004}, []float64{1e-6, 1e-6}, false); err == nil {
		t.Fatal("off-path origin admitted")
	}
}

func traceFixture(t *testing.T) (*RTCoreController, *fakeRTCore, *fakePathLane, TraceIntent) {
	r, f, _ := rtFake(t)
	f.description.Axes = f.description.Axes[:6]
	f.status.Axes = f.status.Axes[:6]
	r.cfg.AxisMask = 63
	r.cfg.CartesianModel = "rosie_1420_v1"
	r.cartesianResolve = func(context.Context, CartesianResolveRequest) (CartesianResolution, error) {
		return CartesianResolution{}, nil
	}
	f.description.PathJog = &ipcclient.PathJogCapability{MaxSamples: 1024, MaxChunk: 16, MaxSampleStepS: .02}
	f.status.Core.HomeValidMask = 63
	f.status.Core.ConfigurationSHA256 = strings.Repeat("d", 64)
	f.status.Core.ConfigurationEpoch = 2
	f.status.Core.HomeEpoch = 3
	f.status.Core.TimeNS = 1
	f.status.Motion.Done = true
	for i := range f.description.Axes {
		a := &f.description.Axes[i]
		a.Index = uint32(i)
		a.MinPosition = -3
		a.MaxPosition = 3
		a.MaxVelocity = 2
		a.CountsPerUnit = 100000
		a.CompletionTolerance = .0001
		f.status.Axes[i].TimeNS = 1
	}
	_, reference := fixtureBlob(t)
	h := reference.Header
	samples := make([]densejointtraj.Sample, 21)
	for i := range samples {
		samples[i].TimeS = float64(i) * .01
		samples[i].QRad[0] = .001 * float64(i)
	}
	segment := densejointtraj.Segment{Kind: "freespace", SourceID: "p2", Samples: samples}
	block := densejointtraj.EncodeBlock(samples)
	sum := sha256.Sum256(block)
	h.Axes.AxisMask = 63
	h.Sampling = densejointtraj.Sampling{DtS: .01, TotalSampleCount: len(samples), TotalDurationS: .2}
	h.Segments = []densejointtraj.SegmentMeta{{Index: 0, Kind: segment.Kind, SourceID: segment.SourceID, SampleCount: len(samples), DurationS: .2, Block: densejointtraj.BlockMeta{ByteLength: len(block), SHA256: "sha256:" + hex.EncodeToString(sum[:])}}}
	h.StartState = &densejointtraj.MeasuredStart{Schema: "rosie.measured-planning-start.v1", CoordinateIdentity: fmt.Sprintf("%s:2:3", f.status.Core.ConfigurationSHA256), ObservationIdentity: "adapter:daemon", JointValues: make([]float64, 6)}
	for _, a := range f.description.Axes {
		h.StartState.JointNames = append(h.StartState.JointNames, a.ID)
	}
	h.ProcessMarkers = nil
	h.ProcessMarkersSHA256 = ""
	h.TrajectoryDigest = densejointtraj.TrajectoryDigest(63, .01, h.Limits.MaxAbsQdRadS, []densejointtraj.Segment{segment})
	header, _ := json.Marshal(h)
	blob := binary.BigEndian.AppendUint32(nil, uint32(len(header)))
	blob = append(append(blob, header...), block...)
	plan, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return blob, nil }
	if _, err = r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	lane := &fakePathLane{obs: pathObservation{Open: true}}
	r.pathOpen = func(_ context.Context, mask uint32, spacing, speed float64, lifetime time.Duration) (pathJogLane, error) {
		if mask != 63 || spacing != .01 || speed != 1 || lifetime != 100*time.Millisecond {
			t.Fatalf("open %d %g %g %s", mask, spacing, speed, lifetime)
		}
		return lane, nil
	}
	request := &LoadRequest{Identity: fixtureIdentity(plan), DryRun: true}
	return r, f, lane, TraceIntent{TargetID: "bench", SessionID: r.Status(context.Background()).Session.ID, Revision: r.JointJogState().Revision, Fraction: .1, InputSequence: 1, Digest: h.TrajectoryDigest, Plan: request}
}

func TestTraceFiniteHoldReleaseAndReplay(t *testing.T) {
	r, _, lane, in := traceFixture(t)
	ctx := context.Background()
	if _, err := r.Trace(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	app, speeds, _ := lane.snapshot()
	if len(app) != 1 || !app[0].end || len(speeds) != 0 {
		t.Fatalf("setup moved or did not mark endpoint: %v %v", app, speeds)
	}
	in.Plan = nil
	in.InputSequence++
	if _, err := r.Trace(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	_, speeds, _ = lane.snapshot()
	if len(speeds) != 1 || speeds[0].speed != .1 {
		t.Fatal(speeds)
	}
	in.Fraction = .5
	in.InputSequence++
	if _, err := r.Trace(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Trace(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
	_, _, ends := lane.snapshot()
	if ends != 1 || r.JointJogState().Active {
		t.Fatal("release did not close")
	}
	if _, err := r.Trace(ctx, "update", in); err == nil {
		t.Fatal("late update accepted")
	}
	if _, err := r.Trace(ctx, "begin", in); err == nil {
		t.Fatal("late begin accepted")
	}
}

func TestTracePrepareBusyIsRetryableWithoutMotion(t *testing.T) {
	r, _, lane, in := traceFixture(t)
	ctx := context.Background()
	r.mu.Lock()
	r.opCancel = func() {}
	r.mu.Unlock()
	_, err := r.Trace(ctx, "prepare", in)
	var failure *Error
	if !errors.As(err, &failure) || failure.Status != 409 || failure.Code != "jog_mode_conflict" {
		t.Fatalf("busy preparation lost its public refusal: %v", err)
	}
	r.mu.Lock()
	r.opCancel = nil
	r.mu.Unlock()
	if _, err = r.Trace(ctx, "prepare", in); err != nil {
		t.Fatalf("same plan did not prepare once idle: %v", err)
	}
	appends, speeds, _ := lane.snapshot()
	if len(appends) != 0 || len(speeds) != 0 {
		t.Fatalf("preparation issued motion: %v %v", appends, speeds)
	}
	in.TargetID = "wrong-cell"
	_, err = r.Trace(ctx, "prepare", in)
	if !errors.As(err, &failure) || failure.Code != "dense_target_mismatch" {
		t.Fatalf("identity refusal was lost: %v", err)
	}
}

func TestTraceDeadmanAndCompletion(t *testing.T) {
	for _, complete := range []bool{false, true} {
		t.Run(fmt.Sprint(complete), func(t *testing.T) {
			r, f, lane, in := traceFixture(t)
			ctx := context.Background()
			if _, err := r.Trace(ctx, "begin", in); err != nil {
				t.Fatal(err)
			}
			if complete {
				f.mu.Lock()
				f.status.Axes[0].LogicalPosition = .02
				f.status.Axes[0].LogicalTarget = .02
				f.mu.Unlock()
				_ = r.observe(ctx)
				lane.mu.Lock()
				lane.obs.PathS = .2
				lane.obs.PathSpeed = 0
				lane.obs.StopReason = 5
				lane.mu.Unlock()
			}
			waitFor(t, "trace stops and ends its lane", func() bool {
				_, _, ends := lane.snapshot()
				return !r.JointJogState().Active && ends == 1
			})
			state := r.JointJogState()
			want := "jog_input_deadline_expired"
			if complete {
				want = "trace_complete"
			}
			if state.Reason != want {
				t.Fatalf("%s want %s", state.Reason, want)
			}
			if complete && (state.LastEnd == nil || state.LastEnd.Popup) {
				t.Fatal("completion should not pop up")
			}
			if complete {
				in.Plan = nil
				in.InputSequence++
				if _, err := r.Trace(ctx, "update", in); err != nil {
					t.Fatal("completion receipt", err)
				}
			}
		})
	}
}

func TestTraceCancellationDuringPreparation(t *testing.T) {
	r, _, lane, in := traceFixture(t)
	entered, release := make(chan struct{}), make(chan struct{})
	original := r.source.cfg.FetchBlob
	r.source.cfg.FetchBlob = func(ctx context.Context, d string) ([]byte, error) {
		close(entered)
		<-release
		return original(ctx, d)
	}
	done := make(chan error, 1)
	go func() { _, err := r.Trace(context.Background(), "begin", in); done <- err }()
	<-entered
	if _, err := r.Trace(context.Background(), "end", in); err != nil {
		t.Fatal(err)
	}
	close(release)
	if err := <-done; err == nil {
		t.Fatal("released setup accepted")
	}
	app, speed, _ := lane.snapshot()
	if len(app) != 0 || len(speed) != 0 {
		t.Fatal("released setup reached native")
	}
}

func TestTraceCurvedResume(t *testing.T) {
	raw := make([][]float64, 100)
	for i := range raw {
		x := float64(i) * .01
		raw[i] = []float64{.1 * x, .07 * math.Sin(x)}
	}
	start := make([]float64, 2)
	for j := range start {
		start[j], _ = traceCurve(raw, 48, j, .371)
	}
	for _, reverse := range []bool{false, true} {
		rows, err := traceRemainder(raw, start, []float64{1e-8, 1e-8}, reverse)
		if err != nil {
			t.Fatal(err)
		}
		for j := range start {
			if rows[0][j] != start[j] {
				t.Fatal("resume changed held target")
			}
		}
	}
}

// The production program is validated once without taking a motion lane.
// Measured encoder variation inside the existing completion envelope must not
// trigger the old 1e-5-radian planning-capture check on every press.
func TestTracePreparedPathAndFeedbackVariation(t *testing.T) {
	r, f, lane, in := traceFixture(t)
	if _, err := r.Trace(context.Background(), "prepare", in); err != nil {
		t.Fatal(err)
	}
	app, speed, _ := lane.snapshot()
	if len(app) != 0 || len(speed) != 0 {
		t.Fatal("prepare moved")
	}
	r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) {
		t.Fatal("ready press fetched/replanned")
		return nil, nil
	}
	f.mu.Lock()
	f.status.Axes[3].LogicalPosition += .000038
	f.mu.Unlock()
	if err := r.observe(context.Background()); err != nil {
		t.Fatal(err)
	}
	in.Plan = nil
	if _, err := r.Trace(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Trace(context.Background(), "end", in); err != nil {
		t.Fatal(err)
	}
}
func TestTracePreparedIdentityAndRange(t *testing.T) {
	for _, mode := range []string{"range", "home", "off_path", "settling"} {
		t.Run(mode, func(t *testing.T) {
			r, f, _, in := traceFixture(t)
			if _, err := r.Trace(context.Background(), "prepare", in); err != nil {
				t.Fatal(err)
			}
			in.Plan = nil
			f.mu.Lock()
			switch mode {
			case "range":
				in.LastSegment = 10
			case "home":
				f.status.Core.HomeEpoch++
			case "off_path":
				f.status.Axes[3].LogicalTarget = .01
				f.status.Axes[3].LogicalPosition = .01
			case "settling":
				f.status.Axes[3].LogicalPosition = .01
			}
			f.mu.Unlock()
			if err := r.observe(context.Background()); err != nil {
				t.Fatal(err)
			}
			if _, err := r.Trace(context.Background(), "begin", in); err == nil {
				t.Fatal("invalid start admitted")
			}
		})
	}
}

func TestTraceWholeProgramUsesOnlySelectedWeldGeometry(t *testing.T) {
	r, f, lane, in := traceFixture(t)
	ctx := context.Background()
	blob, _ := r.source.cfg.FetchBlob(ctx, in.Digest)
	p, err := densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	template := p.Segments[0]
	p.Segments = nil
	for k := 0; k < 3; k++ {
		seg := densejointtraj.Segment{Kind: "weld", SourceID: fmt.Sprintf("p%d", k+1), Samples: make([]densejointtraj.Sample, len(template.Samples))}
		for i := range seg.Samples {
			x := float64(i) / float64(len(seg.Samples)-1)
			seg.Samples[i].TimeS = template.Samples[i].TimeS
			seg.Samples[i].QRad[0] = .02 * (float64(k) + x)
			seg.Samples[i].QRad[1] = .002 * math.Sin(math.Pi*x)
			seg.Samples[i].TorchOn = true
		}
		p.Segments = append(p.Segments, seg)
	}
	blob, err = encodeTraceFixture(p)
	if err != nil {
		t.Fatal(err)
	}
	p, err = densejointtraj.Decode(blob)
	if err != nil {
		t.Fatal(err)
	}
	in.Digest = p.Header.TrajectoryDigest
	in.Plan = &LoadRequest{Identity: fixtureIdentity(p), DryRun: true}
	in.FirstSegment = 1
	in.LastSegment = 1
	r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { return blob, nil }
	if _, err = r.Trace(ctx, "prepare", in); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	for _, seg := range r.tracePrepared.plan.Segments {
		for _, sample := range seg.Samples {
			if sample.TorchOn {
				t.Fatal("process output retained")
			}
		}
	}
	r.mu.Unlock()
	r.source.cfg.FetchBlob = func(context.Context, string) ([]byte, error) { t.Fatal("refetched on press"); return nil, nil }
	in.Plan = nil
	for _, reverse := range []bool{false, true} {
		lane.mu.Lock()
		lane.received = 0
		lane.obs = pathObservation{Open: true}
		lane.mu.Unlock()
		in.Reverse = reverse
		in.Revision = r.JointJogState().Revision
		in.InputSequence = 1
		f.mu.Lock()
		f.status.Axes[0].LogicalPosition = .02
		f.status.Axes[0].LogicalTarget = .02
		if reverse {
			f.status.Axes[0].LogicalPosition = .04
			f.status.Axes[0].LogicalTarget = .04
		}
		f.mu.Unlock()
		_ = r.observe(ctx)
		if _, err = r.Trace(ctx, "begin", in); err != nil {
			t.Fatal(err)
		}
		r.mu.Lock()
		rows := r.jog.trace.rows
		r.mu.Unlock()
		for i, row := range rows {
			want := i
			if reverse {
				want = len(rows) - 1 - i
			}
			for j := 0; j < 6; j++ {
				if math.Abs(row[j]-p.Segments[1].Samples[want].QRad[j]) > 1e-10 {
					t.Fatalf("changed weld geometry reverse=%v row=%d axis=%d", reverse, i, j)
				}
			}
		}
		if _, err = r.Trace(ctx, "end", in); err != nil {
			t.Fatal(err)
		}
	}
	_, speeds, _ := lane.snapshot()
	if len(speeds) != 0 {
		t.Fatal("preparation/begin moved without fresh held intent")
	}
}

func TestTraceRetimingAndPinnedGeometry(t *testing.T) {
	for _, rate := range []float64{.5, 4} {
		t.Run(fmt.Sprint(rate), func(t *testing.T) {
			r, _, lane, in := traceFixture(t)
			ctx := context.Background()
			in.TimeScale = rate
			r.pathOpen = func(_ context.Context, mask uint32, spacing, speed float64, lifetime time.Duration) (pathJogLane, error) {
				if mask != 63 || spacing != .01/max(1, rate) || speed != 1 || lifetime != 100*time.Millisecond {
					t.Fatalf("retimed open %d %g %g %s", mask, spacing, speed, lifetime)
				}
				return lane, nil
			}
			if _, err := r.Trace(ctx, "prepare", in); err != nil {
				t.Fatal(err)
			}
			in.Plan = nil
			if _, err := r.Trace(ctx, "begin", in); err != nil {
				t.Fatal(err)
			}
			r.mu.Lock()
			spacing := r.jog.trace.spacing
			scale := r.jog.trace.speedScale
			r.mu.Unlock()
			if spacing != .01/max(1, rate) || scale != min(1, rate) {
				t.Fatalf("retiming %g %g", spacing, scale)
			}
			for _, fraction := range []float64{.05, 1} {
				in.InputSequence++
				in.Fraction = fraction
				if _, err := r.Trace(ctx, "update", in); err != nil {
					t.Fatal(err)
				}
			}
			_, speeds, _ := lane.snapshot()
			if len(speeds) != 2 || speeds[0].speed != .05*scale || speeds[1].speed != scale {
				t.Fatalf("speeds %+v", speeds)
			}
			in.InputSequence++
			in.LastSegment++
			if _, err := r.Trace(ctx, "update", in); err == nil {
				t.Fatal("changed geometry accepted mid-hold")
			}
		})
	}
}

// Entry is finite held motion to a validated program endpoint. Its setup
// never starts the lane, and release/identity changes cannot publish late IK.
func TestTraceEntryFromOffPath(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		t.Run(fmt.Sprint(reverse), func(t *testing.T) {
			r, f, lane, in := traceFixture(t)
			in.AllowEntry = true
			in.LinearSpeed = .2
			in.AngularSpeed = .5
			in.Reverse = reverse
			f.mu.Lock()
			f.status.Axes[3].LogicalPosition = .01
			f.status.Axes[3].LogicalTarget = .01
			f.mu.Unlock()
			if err := r.observe(context.Background()); err != nil {
				t.Fatal(err)
			}
			var request CartesianResolveRequest
			r.cartesianResolve = func(_ context.Context, req CartesianResolveRequest) (CartesianResolution, error) {
				request = req
				return traceEntryResult(req), nil
			}
			if _, err := r.Trace(context.Background(), "begin", in); err != nil {
				t.Fatal(err)
			}
			want := .02
			if reverse {
				want = 0
			}
			if request.Operation != "trace_entry" || request.Target == nil || request.Target[0] != want || request.Pose[3] != .01 || request.LinearSpeed != .2 {
				t.Fatalf("wrong entry %+v", request)
			}
			app, speeds, _ := lane.snapshot()
			if len(app) != 1 || !app[0].end || len(speeds) != 0 {
				t.Fatalf("entry setup moved: %v %v", app, speeds)
			}
			if app[0].samples[0][3] != .01 || app[0].samples[len(app[0].samples)-1][0] != want {
				t.Fatal("entry endpoints changed")
			}
			in.Plan = nil
			in.InputSequence++
			if _, err := r.Trace(context.Background(), "update", in); err != nil {
				t.Fatal(err)
			}
			_, speeds, _ = lane.snapshot()
			if len(speeds) != 1 || speeds[0].speed != in.Fraction {
				t.Fatal(speeds)
			}
			if _, err := r.Trace(context.Background(), "end", in); err != nil {
				t.Fatal(err)
			}
			_, _, ends := lane.snapshot()
			if ends != 1 {
				t.Fatal("entry did not stop on release")
			}
		})
	}
}
func traceEntryResult(req CartesianResolveRequest) CartesianResolution {
	result := CartesianResolution{Accepted: true, Spacing: .01, SingularityScale: 1, JointLimitScale: 1}
	for n := 0; n <= 20; n++ {
		var q [6]float64
		for i := range q {
			q[i] = req.Pose[i] + (req.Target[i]-req.Pose[i])*float64(n)/20
		}
		result.Waypoints = append(result.Waypoints, q)
	}
	result.Waypoints[0] = req.Pose
	result.Waypoints[20] = *req.Target
	return result
}
func TestTraceEntrySetupFences(t *testing.T) {
	for _, mode := range []string{"release", "coordinate_change", "moved", "wrong_endpoint", "bad_speed"} {
		t.Run(mode, func(t *testing.T) {
			r, f, lane, in := traceFixture(t)
			ctx := context.Background()
			if _, err := r.Trace(ctx, "prepare", in); err != nil {
				t.Fatal(err)
			}
			in.Plan = nil
			in.AllowEntry = true
			in.EndpointOnly = true
			in.LinearSpeed = .2
			in.AngularSpeed = .5
			entered, finish := make(chan struct{}), make(chan struct{})
			r.cartesianResolve = func(_ context.Context, req CartesianResolveRequest) (CartesianResolution, error) {
				close(entered)
				<-finish
				result := traceEntryResult(req)
				if mode == "wrong_endpoint" {
					result.Waypoints[20][0] += .001
				}
				return result, nil
			}
			if mode == "bad_speed" {
				in.LinearSpeed = math.NaN()
			}
			done := make(chan error, 1)
			go func() { _, err := r.Trace(ctx, "begin", in); done <- err }()
			if mode != "bad_speed" {
				<-entered
				if mode == "release" {
					if _, err := r.Trace(ctx, "end", in); err != nil {
						t.Fatal(err)
					}
				}
				if mode == "coordinate_change" || mode == "moved" {
					f.mu.Lock()
					if mode == "coordinate_change" {
						f.status.Core.HomeEpoch++
					} else {
						f.status.Axes[0].LogicalTarget = .005
					}
					f.mu.Unlock()
					_ = r.observe(ctx)
				}
				close(finish)
			}
			if err := <-done; err == nil {
				t.Fatal("invalid entry accepted")
			}
			app, speeds, _ := lane.snapshot()
			if len(app) != 0 || len(speeds) != 0 {
				t.Fatal("invalid entry reached native")
			}
		})
	}
}
func TestTraceOnPathDoesNotUseEntry(t *testing.T) {
	r, _, _, in := traceFixture(t)
	in.AllowEntry = true
	in.LinearSpeed = .2
	in.AngularSpeed = .5
	r.cartesianResolve = func(context.Context, CartesianResolveRequest) (CartesianResolution, error) {
		t.Fatal("on-path Trace called entry resolver")
		return CartesianResolution{}, nil
	}
	if _, err := r.Trace(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	_, _ = r.Trace(context.Background(), "end", in)
}

// A reverse press already at the start of its leg completes without creating
// a degenerate native path, and still consumes exactly one revision.
func TestTraceAlreadyAtEndpointDoesNotOpenLane(t *testing.T) {
	for _, reverse := range []bool{false, true} {
		r, f, lane, in := traceFixture(t)
		in.Reverse = reverse
		endpoint := .02
		if reverse {
			endpoint = 0
		}
		f.mu.Lock()
		f.status.Axes[0].LogicalTarget = endpoint + 1e-7
		f.status.Axes[0].LogicalPosition = endpoint
		f.mu.Unlock()
		if err := r.observe(context.Background()); err != nil {
			t.Fatal(err)
		}
		state, err := r.Trace(context.Background(), "begin", in)
		if err != nil || state.Active || state.LastEnd == nil || state.LastEnd.Cause != "trace_complete" {
			t.Fatalf("%+v %v", state, err)
		}
		app, speeds, ends := lane.snapshot()
		if len(app) != 0 || len(speeds) != 0 || ends != 0 {
			t.Fatal("no-op reached native lane")
		}
		if _, err = r.Trace(context.Background(), "begin", in); err == nil {
			t.Fatal("same press replayed")
		}
		in.InputSequence++
		if _, err = r.Trace(context.Background(), "update", in); err != nil {
			t.Fatal("completion receipt lost", err)
		}
		in.SessionID = "wrong"
		if _, err = r.Trace(context.Background(), "update", in); err == nil {
			t.Fatal("foreign session accepted")
		}
	}
}

func TestTraceExcludesOnlyWeldBoundaryDelays(t *testing.T) {
	for _, s := range []densejointtraj.Segment{{Kind: "dwell", SourceID: "p1:weld-start-delay"}, {Kind: "dwell", SourceID: "p2:weld-stop-delay"}} {
		if !traceWeldDelay(s) {
			t.Fatal("production timing retained in trace", s.SourceID)
		}
	}
	for _, s := range []densejointtraj.Segment{{Kind: "weld", SourceID: "p1:weld-start-delay"}, {Kind: "dwell", SourceID: "ordinary-pause"}, {Kind: "freespace", SourceID: "p1"}} {
		if traceWeldDelay(s) {
			t.Fatal("ordinary path omitted", s.SourceID)
		}
	}
}
