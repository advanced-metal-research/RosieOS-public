//go:build linux

package denseexec

import (
	"context"
	"math"
	"strings"
	"testing"
)

// The End's end_cause: a real release stays released; what the operator did
// not choose pops up in its own words; what they chose does not; a client
// reporting a refusal is classified by that refusal; an older client (no
// cause) is a release as before.
func TestJogEndClientCauses(t *testing.T) {
	cases := []struct {
		cause, code string
		popup       bool
		repull      string
		text        string
	}{
		{"", JogEndReleased, false, RepullYes, "Jog released."},
		{"trigger_released", JogEndReleased, false, RepullYes, "Jog released."},
		{"pointer_up", JogEndReleased, false, RepullYes, "Jog released."},
		{"stick_neutral", JogEndReleased, false, RepullYes, "Jog released."},
		{"stick_direction_change", JogEndStickDirectionChange, true, RepullAfterNeutral, "Release RT and centre the stick to jog a new direction."},
		{"pointer_leave", JogEndPointerLeft, true, RepullYes, "the pointer left the jog button"},
		{"focus_lost", JogEndFocusLost, true, RepullYes, "this window lost focus"},
		{"visibility_hidden", JogEndFocusLost, true, RepullYes, "this window lost focus"},
		{"controller_disconnected", JogEndControllerDisconnected, true, RepullYes, "the game controller disconnected"},
		{"mode_switch", JogEndModeSwitch, false, RepullYes, "Jog ended: the jog mode changed."},
		{"ui_navigation", JogEndUINavigation, false, RepullYes, "Jog ended: the page changed."},
		{"operator_stop", JogEndHalted, false, RepullAfterRearm, "Stop was pressed"},
		{"operator_disarm", JogEndHalted, false, RepullAfterRearm, "the robot was disarmed"},
		{"selection_changed", JogEndHalted, true, RepullAfterRearm, "the selected cell changed"},
		{"session_changed", JogEndSessionStale, true, RepullYes, "replaced or went stale"},
		{"state_poll_failed", JogEndTransportLost, true, RepullYes, "lost its link to the OLP server"},
		{"jog_disabled", JogEndJogDisabled, true, RepullUnknown, "jogging became unavailable"},
		{"intent_refused:jog_input_deadline_expired", JogEndInputDeadlineExpired, true, RepullYes, "arrived late"},
		{"server_reason:joint_limit", JogEndJointLimit, true, RepullNo, "reached its"},
		{"something_new", JogEndClientEnded, true, RepullUnknown, "by this client (something_new)"},
	}
	for _, tc := range cases {
		c := classifyJogEnd(jogEndFacts{clientCause: tc.cause, jogAxis: -1, lower: []float64{-1}, upper: []float64{1}, q: []float64{.99}})
		if c.code != tc.code || c.popup != tc.popup || c.repull != tc.repull || !strings.Contains(c.text, tc.text) {
			t.Errorf("%q: %+v", tc.cause, c)
		}
		if tc.cause != "" && !strings.Contains(c.detail, "client_cause="+tc.cause) {
			t.Errorf("%q: detail %q lacks the client cause", tc.cause, c.detail)
		}
	}
	// A server-side cause wins: the client's End came after native ended it.
	if c := classifyJogEnd(jogEndFacts{cause: "path_buffer_underrun", clientCause: "pointer_up", path: true}); c.code != JogEndPathUnderrun {
		t.Fatalf("server cause %+v", c)
	}
	for in, want := range map[string]string{"pointer_up": "pointer_up", "  focus_lost ": "focus_lost", "Bad Cause": "invalid_end_cause",
		strings.Repeat("a", 81): "invalid_end_cause", "intent_refused:jog_session_stale": "intent_refused:jog_session_stale"} {
		if got := sanitizeEndCause(in); got != want {
			t.Errorf("sanitize %q = %q", in, got)
		}
	}
}

// The End body's end_cause reaches the classification, the status and the
// journal line; a stick wobble that ended the hold says what to do.
func TestJogEndClientCauseThroughHalt(t *testing.T) {
	r, _, _, _, in := pathFake(t)
	sink := captureLog(t)
	ctx := WithClient(context.Background(), Client{ID: "deck-00112233aabbccdd", Label: "rosie-pendant-v5"})
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	in.EndCause = "stick_direction_change"
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	end := r.Status(context.Background()).LastJogEnd
	if end == nil || end.Code != JogEndStickDirectionChange || !end.Popup || end.Repull != RepullAfterNeutral || end.ClientCause != "stick_direction_change" || end.Cause != "release" {
		t.Fatalf("end %+v", end)
	}
	lines := sink.lines("jog hold: end ")
	if len(lines) != 1 || !strings.Contains(lines[0], "end_code=stick_direction_change") || !strings.Contains(lines[0], "client_cause=stick_direction_change") {
		t.Fatalf("lines %q", lines)
	}
}

// The live readout near a singularity: native's path speed under a "cap"
// that is the resolver's singularity scale, J5 against the URDF's wrist
// angle, the line end ahead, the way out; kept after the hold ends.
func TestPathLiveNearSingularity(t *testing.T) {
	r, _, lane, resolver, in := pathFake(t)
	// The resolver's scale falls from 1 at sample 100 to 0 at 200 and the
	// line ends at its gate at 180.
	resolver.scaleAt = func(i uint64) float64 { return math.Max(0, math.Min(1, 1-(float64(i)-100)/100)) }
	resolver.endAt, resolver.endWhy = 180, "jacobian_gate"
	r.SetRobotGeometrySource(func(model string) (RobotGeometry, error) {
		if model != "rosie_1420_v1" {
			t.Errorf("geometry model %q", model)
		}
		return RobotGeometry{WristSingularRad: []float64{math.Pi / 2, -math.Pi / 2}}, nil
	})
	in.Twist = [6]float64{-.4}
	if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
		t.Fatal(err)
	}
	if s := r.Status(context.Background()).JogLive; s == nil || !s.Active || s.Axis != "-X" || s.Escape != "+X" || s.Near {
		t.Fatalf("at the start %+v", s)
	}
	lane.mu.Lock()
	lane.obs.Consumed, lane.obs.PathS, lane.obs.PathSpeed, lane.obs.Governing = 150, .150, .021, 5
	lane.mu.Unlock()
	waitFor(t, "line end streamed", func() bool {
		a, _, _ := lane.snapshot()
		return len(a) > 0 && a[len(a)-1].end
	})
	want := "limited by: singularity (J5 22.9°, 67.1° from the wrist-singular 90.0°) — slowing, stops in ~29 mm; back out with +X, or joint-jog -J5"
	// The line end's playable index reaches the readout with the next native
	// observation after the end chunk (179: 29 mm beyond 150).
	waitFor(t, "observation", func() bool {
		s := r.Status(context.Background()).JogLive
		return s != nil && s.Text == want
	})
	s := r.Status(context.Background()).JogLive
	if s.LimitKind != "singularity" || !s.Near || s.GateScale == nil || math.Abs(*s.GateScale-.5) > 1e-9 || s.MinGateScaleAhead == nil || math.Abs(*s.MinGateScaleAhead-.21) > 1e-9 {
		t.Fatalf("live %+v", s)
	}
	if math.Abs(s.Speed-21) > 1e-9 || math.Abs(s.RequestedSpeed-200) > 1e-9 || s.SpeedUnit != "mm/s" || s.LineEnd != "jacobian_gate" ||
		math.Abs(*s.DistanceToEnd-29) > 1e-9 || s.JointHint != "-J5" || math.Abs(*s.WristSingularDeg-90) > 1e-9 {
		t.Fatalf("live %+v", s)
	}
	if j := r.JointJogState().PathLive; j == nil || j.Text != want {
		t.Fatalf("jog state %+v", j)
	}
	if _, err := r.CartesianJog(context.Background(), "halt", in); err != nil {
		t.Fatal(err)
	}
	kept := r.Status(context.Background()).JogLive
	if kept == nil || kept.Active || !kept.Near || kept.DistanceToEnd != nil ||
		kept.Text != "stopped near a singularity (J5 22.9°, 67.1° from the wrist-singular 90.0°); back out with +X, or joint-jog -J5" {
		t.Fatalf("kept %+v", kept)
	}
}

// A "cap" that is the joints' velocity reserve, not the singularity scale,
// names the joint; without the robot's geometry J5 is named alone.
func TestPathLiveJointReserveCap(t *testing.T) {
	r, _, lane, _, in := pathFake(t)
	if _, err := r.CartesianJog(context.Background(), "start", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	lane.obs.Consumed, lane.obs.PathS, lane.obs.PathSpeed, lane.obs.Governing = 10, .010, .2, 5
	lane.mu.Unlock()
	waitFor(t, "observation", func() bool {
		s := r.Status(context.Background()).JogLive
		return s != nil && s.Governing == "cap"
	})
	s := r.Status(context.Background()).JogLive
	if s.LimitKind != "joint_reserve" || s.Limit != "joint speed reserve (J1)" || s.Near || s.WristSingularDeg != nil || s.Text != "limited by: joint speed reserve (J1)" {
		t.Fatalf("live %+v", s)
	}
	lane.mu.Lock()
	lane.obs.Governing = 1
	lane.mu.Unlock()
	waitFor(t, "command", func() bool { return r.Status(context.Background()).JogLive.Text == "limited by: your speed" })
}
