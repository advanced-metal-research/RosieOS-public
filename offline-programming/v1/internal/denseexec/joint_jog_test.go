//go:build linux

package denseexec

import (
	"context"
	"errors"
	"io"
	"math"
	"reflect"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type jointSample struct {
	origin, deadline uint64
	velocities       []float64
}
type fakeJointLane struct {
	mu      sync.Mutex
	f       *fakeRTCore
	samples []jointSample
	err     error
}

type fakeJointStream struct {
	*fakeJointLane
	eof chan struct{}
}

func (s *fakeJointStream) Rejection(ctx context.Context) error {
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-s.eof:
		return io.EOF
	}
}

// TestJointJogStreamEOFEndsOnce isolates stream loss from the browser deadman;
// EOF must retire the generation and send one End before later input is refused.
func TestJointJogStreamEOFEndsOnce(t *testing.T) {
	r, f, lane, in := jointFake(t)
	stream := &fakeJointStream{fakeJointLane: lane, eof: make(chan struct{})}
	var closeOnce sync.Once
	closeStream := func() { closeOnce.Do(func() { close(stream.eof) }) }
	t.Cleanup(closeStream)
	open := r.jogOpen
	r.jogOpen = func(ctx context.Context, mask uint32, origin uint64) (jointJogLane, error) {
		_, err := open(ctx, mask, origin)
		return stream, err
	}
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	h := r.jog
	h.timer.Stop()
	r.mu.Unlock()
	closeStream()
	rtAwait(t, func() bool { return len(jointCalls(f)) == 3 })
	if state := r.JointJogState(); state.Active || state.Revision != 1 || state.Reason != "rt_core_failure" {
		t.Fatalf("EOF left input live: %+v", state)
	}
	for _, action := range []string{"update", "begin", "end"} {
		_, err := r.JointJog(context.Background(), action, in)
		jointCode(t, err, "jog_session_stale")
	}
	// A later deadman callback cannot duplicate stream-loss cleanup.
	r.expireJointJog(h, h.last.Add(100*time.Millisecond))
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "update_jog", "end_jog"}) {
		t.Fatal(got)
	}
}

func (s *fakeJointLane) Generation() uint64 { return 17 }
func (s *fakeJointLane) UpdateAt(v []float64, origin, deadline uint64) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.f.mu.Lock()
	defer s.f.mu.Unlock()
	s.f.record("update_jog")
	s.samples = append(s.samples, jointSample{origin, deadline, append([]float64(nil), v...)})
	return s.err
}
func (s *fakeJointLane) End(context.Context) error {
	s.f.mu.Lock()
	defer s.f.mu.Unlock()
	s.f.record("end_jog")
	return nil
}
func jointFake(t *testing.T) (*RTCoreController, *fakeRTCore, *fakeJointLane, JointJogIntent) {
	t.Helper()
	r, f, _ := rtFake(t)
	f.mu.Lock()
	for i := range f.description.Axes {
		f.description.Axes[i].MaxVelocity = 2
	}
	f.mu.Unlock()
	if _, err := r.Arm(context.Background(), true); err != nil {
		t.Fatal(err)
	}
	lane := &fakeJointLane{f: f}
	r.jogOpen = func(_ context.Context, mask uint32, _ uint64) (jointJogLane, error) {
		if mask != 1 {
			t.Errorf("mask = %d; want 1", mask)
		}
		f.mu.Lock()
		defer f.mu.Unlock()
		f.record("begin_jog")
		return lane, nil
	}
	return r, f, lane, JointJogIntent{TargetID: "bench", SessionID: r.Status(context.Background()).Session.ID, Axis: 0, Direction: 1, Fraction: 0.1}
}
func jointCode(t *testing.T, err error, want string) {
	t.Helper()
	var e *Error
	if !errors.As(err, &e) || e.Code != want {
		t.Fatalf("error = %v; want %s", err, want)
	}
}
func jointCalls(f *fakeRTCore) []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	var calls []string
	for _, c := range f.calls {
		if c == "begin_jog" || c == "update_jog" || c == "end_jog" || c == "stop" || c == "release" {
			calls = append(calls, c)
		}
	}
	return calls
}

// TestJointJogCartesianModelWaitsForFreshCapture prevents a prepared SDK lane
// from receiving motion captured before its Begin established the input clock.
func TestJointJogCartesianModelWaitsForFreshCapture(t *testing.T) {
	r, f, lane, in := jointFake(t)
	r.cfg.CartesianModel = "rosie_1420_v1"
	ctx := context.Background()
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog"}) {
		t.Fatalf("setup replayed pre-Begin input: %v", got)
	}
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	samples := append([]jointSample(nil), lane.samples...)
	lane.mu.Unlock()
	// 0.1 of the described 2 rad/s under the measured 0.75 reserve: 0.15 rad/s.
	if len(samples) != 1 || math.Abs(samples[0].velocities[0]-0.15) > 1e-12 || samples[0].deadline-samples[0].origin != 100000000 {
		t.Fatalf("fresh input was not sent once within its original lifetime: %+v", samples)
	}
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
}

func TestJointJogHoldReleaseSequence(t *testing.T) {
	r, f, lane, in := jointFake(t)
	ctx := context.Background()
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	state := r.JointJogState()
	if !state.Active || state.Generation != 17 || state.Sequence != 2 {
		t.Fatalf("state = %+v", state)
	}
	lane.mu.Lock()
	for _, sample := range lane.samples {
		if sample.origin == 0 || sample.deadline-sample.origin != 100000000 || math.Abs(sample.velocities[0]-0.15) > 1e-12 || len(sample.velocities) != 9 {
			t.Fatalf("sample = %+v", sample)
		}
		for _, v := range sample.velocities[1:] {
			if v != 0 {
				t.Fatal("other axis moved")
			}
		}
	}
	if lane.samples[1].origin <= lane.samples[0].origin {
		t.Fatal("capture did not advance")
	}
	lane.mu.Unlock()
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
	if got := r.JointJogState(); got.Active || got.Revision != 1 {
		t.Fatalf("end = %+v", got)
	}
	_, err := r.JointJog(ctx, "update", in)
	jointCode(t, err, "jog_session_stale")
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "update_jog", "update_jog", "end_jog"}) {
		t.Fatal(got)
	}
}
func TestJointJogDeadmanAndDelayedBegin(t *testing.T) {
	r, f, _, in := jointFake(t)
	ctx := context.Background()
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
	_, err := r.JointJog(ctx, "begin", in)
	jointCode(t, err, "jog_session_stale")
	in.Revision = 1
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	h := r.jog
	last := h.last
	h.timer.Stop()
	r.mu.Unlock()
	r.expireJointJog(h, last.Add(99*time.Millisecond))
	if !r.JointJogState().Active {
		t.Fatal("expired before 100 ms")
	}
	r.expireJointJog(h, last.Add(100*time.Millisecond))
	if got := r.JointJogState(); got.Active || got.Revision != 2 || got.Reason != "jog_input_deadline_expired" {
		t.Fatal(got)
	}
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "update_jog", "end_jog"}) {
		t.Fatal(got)
	}
}
func TestJointJogDeadmanTimer(t *testing.T) {
	r, f, _, in := jointFake(t)
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	rtAwait(t, func() bool { return len(jointCalls(f)) == 3 })
	if got := r.JointJogState(); got.Active || got.Reason != "jog_input_deadline_expired" {
		t.Fatal(got)
	}
}
func TestJointJogValidationAndSecondAxis(t *testing.T) {
	for _, tc := range []struct {
		name   string
		change func(*JointJogIntent)
		code   string
	}{
		{"none", func(i *JointJogIntent) { i.TargetID = "" }, "dense_target_unavailable"},
		{"wrong_target", func(i *JointJogIntent) { i.TargetID = "other" }, "dense_target_unavailable"},
		{"axis", func(i *JointJogIntent) { i.Axis = 9 }, "invalid_axis_mask"},
		{"fraction", func(i *JointJogIntent) { i.Fraction = 1.01 }, "jog_invalid_vector"},
		{"zero", func(i *JointJogIntent) { i.Fraction = 0 }, "jog_invalid_vector"},
		{"direction", func(i *JointJogIntent) { i.Direction = 0 }, "jog_invalid_vector"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, f, _, in := jointFake(t)
			tc.change(&in)
			_, err := r.JointJog(context.Background(), "begin", in)
			jointCode(t, err, tc.code)
			if len(jointCalls(f)) != 0 {
				t.Fatal("invalid intent reached SDK")
			}
		})
	}
	r, _, _, in := jointFake(t)
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	in.Axis = 1
	_, err := r.JointJog(context.Background(), "begin", in)
	jointCode(t, err, "jog_mode_conflict")
	_, err = r.JointJog(context.Background(), "update", in)
	jointCode(t, err, "jog_mode_conflict")
	if r.JointJogState().Sequence != 1 {
		t.Fatal("second axis produced input")
	}
}
func TestJointJogRefusalPropagation(t *testing.T) {
	for _, code := range []string{"jog_not_ready", "jog_mode_conflict", "jog_stale_sequence", "jog_input_too_old", "outside_limits_outward"} {
		t.Run(code, func(t *testing.T) {
			r, f, lane, in := jointFake(t)
			if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
				t.Fatal(err)
			}
			lane.mu.Lock()
			lane.err = &control.JogRejection{Reason: code}
			lane.mu.Unlock()
			_, err := r.JointJog(context.Background(), "update", in)
			jointCode(t, err, code)
			if got := r.JointJogState(); got.Active || got.Reason != code {
				t.Fatal(got)
			}
			if got := jointCalls(f); got[len(got)-1] != "end_jog" {
				t.Fatal(got)
			}
		})
	}
}
func TestJointJogNativeRefusal(t *testing.T) {
	r, _, _, in := jointFake(t)
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	r.mu.Lock()
	status := *r.observation
	status.Jog = &control.JogObservation{JogGeneration: 17, UpdateReason: ipcclient.JogReasonStaleSequence}
	r.observation = &status
	r.mu.Unlock()
	r.observeJointJog()
	if got := r.JointJogState(); got.Active || got.Reason != "jog_stale_sequence" {
		t.Fatal(got)
	}
}

func TestJointJogNativeLimitKeepsSession(t *testing.T) {
	r, f, _, in := jointFake(t)
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	for _, reason := range []uint32{ipcclient.JogReasonLimited, 0} {
		r.mu.Lock()
		status := *r.observation
		status.Jog = &control.JogObservation{JogGeneration: 17, StateReason: reason}
		r.observation = &status
		r.mu.Unlock()
		r.observeJointJog()
		got := r.JointJogState()
		if !got.Active || (reason != 0 && got.Reason != "jog_limited") || (reason == 0 && got.Reason != "") {
			t.Fatal(got)
		}
	}
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "update_jog"}) {
		t.Fatal(got)
	}
	if _, err := r.JointJog(context.Background(), "end", in); err != nil {
		t.Fatal(err)
	}
	if r.JointJogState().Active {
		t.Fatal("release must still end a limited jog")
	}
}
func TestJointJogStopDisarmAndGrantLoss(t *testing.T) {
	for _, action := range []string{"stop", "disarm", "grant_loss"} {
		t.Run(action, func(t *testing.T) {
			r, f, _, in := jointFake(t)
			if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
				t.Fatal(err)
			}
			switch action {
			case "stop":
				_, _ = r.Stop(context.Background())
			case "disarm":
				_, _ = r.Arm(context.Background(), false)
			case "grant_loss":
				f.updates <- control.Renewal{Err: &control.Rejected{Reason: "no_grant"}}
			}
			rtAwait(t, func() bool { return len(jointCalls(f)) == 5 })
			_, err := r.JointJog(context.Background(), "update", in)
			jointCode(t, err, "jog_session_stale")
			if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "update_jog", "stop", "end_jog", "release"}) {
				t.Fatal(got)
			}
		})
	}
}
func TestJointJogStopWinsPendingBegin(t *testing.T) {
	r, f, lane, in := jointFake(t)
	entered, release := make(chan struct{}), make(chan struct{})
	r.jogOpen = func(context.Context, uint32, uint64) (jointJogLane, error) {
		f.mu.Lock()
		f.record("begin_jog")
		f.mu.Unlock()
		close(entered)
		<-release
		return lane, nil
	}
	result := make(chan error, 1)
	go func() { _, err := r.JointJog(context.Background(), "begin", in); result <- err }()
	<-entered
	if _, err := r.Stop(context.Background()); err != nil {
		t.Fatal(err)
	}
	close(release)
	jointCode(t, <-result, "jog_session_stale")
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "stop", "release", "end_jog"}) {
		t.Fatal(got)
	}
}

func TestJointJogAuthorityReadinessAndOperationExclusion(t *testing.T) {
	for _, condition := range []string{"no_grant", "disarmed", "undescribed", "stale", "operation", "loop"} {
		t.Run(condition, func(t *testing.T) {
			r, f, _, in := jointFake(t)
			r.mu.Lock()
			want := "jog_not_ready"
			switch condition {
			case "no_grant":
				r.held = false
				want = "no_grant"
			case "disarmed":
				s := *r.observation
				s.Core.Armed = 0
				r.observation = &s
			case "undescribed":
				r.description.Axes = nil
				want = "invalid_axis_mask"
			case "stale":
				r.nativeSeen = time.Now().Add(-501 * time.Millisecond)
			case "operation":
				r.opCancel = func() {}
				want = "jog_mode_conflict"
			case "loop":
				// Between cycles: the last one is done, the reload not yet started.
				r.loop = &loopRun{mode: "loop"}
				r.session.State = "done"
				want = "jog_mode_conflict"
			}
			r.mu.Unlock()
			_, err := r.JointJog(context.Background(), "begin", in)
			jointCode(t, err, want)
			if got := jointCalls(f); len(got) != 0 {
				t.Fatal(got)
			}
		})
	}
	r, f, _, in := jointFake(t)
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatal(err)
	}
	jointCode(t, r.Home(context.Background()), "jog_mode_conflict")
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "update_jog"}) {
		t.Fatal(got)
	}
}

// Teaching on the Deck's Program page (owner, 2026-10-12): jog, record, jog,
// record with a planned program loaded but not playing. The loaded plan never
// refuses a jog; a playing program or a running loop does.
func TestJointJogWithAPlanLoadedButNotPlaying(t *testing.T) {
	r, f, _, in := jointFake(t)
	r.mu.Lock()
	r.session.State = "loaded"
	r.mu.Unlock()
	if _, err := r.JointJog(context.Background(), "begin", in); err != nil {
		t.Fatalf("a loaded, not playing plan refused the jog: %v", err)
	}
	if got := jointCalls(f); !reflect.DeepEqual(got, []string{"begin_jog", "update_jog"}) {
		t.Fatal(got)
	}
	for _, state := range []string{"playing", "loop"} {
		r, f, _, in := jointFake(t)
		r.mu.Lock()
		if state == "loop" {
			r.loop = &loopRun{mode: "count", count: 3}
			r.session.State = "loaded" // the next cycle reloaded, Play not yet sent
		} else {
			r.session.State = state
		}
		r.mu.Unlock()
		_, err := r.JointJog(context.Background(), "begin", in)
		jointCode(t, err, "jog_mode_conflict")
		if got := jointCalls(f); len(got) != 0 {
			t.Fatal(state, got)
		}
	}
}

// The hold's fraction runs to 100 % of the described maximum (owner decision
// 2026-09-21): under the 0.75 reserve a 2 rad/s axis is asked for 1.5 rad/s,
// and a fraction above 1 is refused before any lane is opened.
func TestJointJogFractionRunsToOne(t *testing.T) {
	r, _, lane, in := jointFake(t)
	ctx := context.Background()
	in.Fraction = 1
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.JointJog(ctx, "update", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	samples := append([]jointSample(nil), lane.samples...)
	lane.mu.Unlock()
	if len(samples) == 0 {
		t.Fatal("no sample reached the lane")
	}
	for _, sample := range samples {
		if math.Abs(sample.velocities[0]-1.5) > 1e-12 {
			t.Fatalf("100 %% of 2 rad/s under the reserve should be 1.5 rad/s: %+v", samples)
		}
	}
	if _, err := r.JointJog(ctx, "end", in); err != nil {
		t.Fatal(err)
	}
	r, _, _, in = jointFake(t)
	in.Fraction = 1.01
	_, err := r.JointJog(ctx, "begin", in)
	jointCode(t, err, "jog_invalid_vector")
}
