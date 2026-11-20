//go:build linux

package denseexec

import (
	"bytes"
	"context"
	"log"
	"math"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Every cause a hold can retire with reaches an operator code with words:
// the native jog reasons (as OLP names them), the native path stop names,
// the resolver's path-end reasons and OLP's own retirement codes. An unknown
// cause is shown with its raw code, never hidden.
func TestJogEndEveryCauseMaps(t *testing.T) {
	var causes []string
	for code := uint32(1); code <= ipcclient.JogReasonPathInvalidConfig; code++ {
		causes = append(causes, nativeJointJogReason(code))
	}
	causes = append(causes, pathStopNames[1:]...)
	for name := range pathEndCodes {
		causes = append(causes, name)
	}
	causes = append(causes, "jog_input_deadline_expired", "path_buffer_underrun", "cartesian_resolver_changed",
		"cartesian_resolver_unavailable", "cartesian_resolver_invalid", "cartesian_resolver_too_slow", "cartesian_pose_stale",
		"operator_stop", "disarm", "selection_changed", "server_shutdown", "no_grant", "control_session_stale",
		"rt_core_transport_lost", "rt_core_request_slow", "jog_transport_congested", "path_closed", "ui_heartbeat_lost",
		"jog_clock_calibration_expired", "rt_core_readiness_lost", "control_renewal_lost", "rt_core_failure",
		"jog_native_rejected_3", "jog_publisher_busy")
	for _, cause := range causes {
		c := classifyJogEnd(jogEndFacts{cause: cause, lifetime: 250 * time.Millisecond, jogAxis: -1})
		if c.code == JogEndUnknown {
			t.Errorf("cause %q has no operator code", cause)
		}
		if c.text == "" || c.detail == "" || c.repull == "" {
			t.Errorf("cause %q: %+v", cause, c)
		}
		if strings.Contains(c.text, `"`) || strings.Contains(c.detail, `"`) && !strings.Contains(c.detail, `resolver_end="`) {
			t.Errorf("cause %q: a double quote breaks key=\"value\" journal parsing: %+v", cause, c)
		}
	}
	c := classifyJogEnd(jogEndFacts{cause: "unknown_jog_reason_99", jogAxis: -1})
	if c.code != JogEndUnknown || !strings.Contains(c.text, "unknown_jog_reason_99") || !c.popup || c.repull != RepullUnknown {
		t.Fatalf("unknown cause %+v", c)
	}
}

func TestJogEndOperatorWords(t *testing.T) {
	deg := math.Pi / 180
	lower, upper := []float64{-3, -2, -2.1, -3, -2, -3}, []float64{3, 2, 2.1, 3, 2, 3}
	cases := []struct {
		name   string
		facts  jogEndFacts
		code   string
		text   []string
		popup  bool
		repull string
	}{
		{"release", jogEndFacts{cause: ""}, JogEndReleased, []string{"Jog released."}, false, RepullYes},
		{"OLP deadman", jogEndFacts{cause: "jog_input_deadline_expired", intentAge: 263 * time.Millisecond, lifetime: 250 * time.Millisecond},
			JogEndInputDeadlineExpired, []string{"last one 263 ms ago, limit 250 ms", "Pull the trigger again"}, true, RepullYes},
		{"native expiry on a path", jogEndFacts{cause: "jog_input_deadline_expired", path: true, stopReason: 2, governing: 8, ahead: 87},
			JogEndInputDeadlineExpired, []string{"arrived late"}, true, RepullYes},
		{"underrun", jogEndFacts{cause: "path_buffer_underrun", path: true, stopReason: 4, governing: 6, ahead: 2, underruns: 1},
			JogEndPathUnderrun, []string{"2 samples ahead", "lower the speed"}, true, RepullYes},
		// the 1430 cell 2026-10-10 22:25: the −X line ended at the wrist singularity,
		// J5 −88.3° (the 1430 cell's is at ±90°, so the text names no singular angle).
		{"singularity", jogEndFacts{cause: "jacobian_gate", path: true, q: []float64{0, .3, .7, 0, -88.3 * deg, 0}, lower: lower, upper: upper},
			JogEndSingularityGate, []string{"reached a singularity (J5 = -88.3°, J3 = 40.1°)", "jog back the way you came"}, true, RepullNo},
		{"step limit", jogEndFacts{cause: "ik_discontinuity", path: true, q: []float64{0, .3, .7, 0, -88.3 * deg, 0}},
			JogEndIKDiscontinuity, []string{"needs a joint to turn faster than it can", "J5 = -88.3°"}, true, RepullNo},
		// the 1430 cell 2026-10-10 22:01:13: the adapter refused one speed frame.
		{"adapter refusal", jogEndFacts{cause: "jog_native_rejected_3", path: true, governing: 1, ahead: 100},
			JogEndCommandRefused, []string{"refused one jog command; the robot stayed armed", "Pull the trigger again"}, true, RepullYes},
		{"joint limit names the joint", jogEndFacts{cause: "joint_limit", path: true, q: []float64{0, .3, -2.09, 0, 1, 0}, lower: lower, upper: upper, jogAxis: -1},
			JogEndJointLimit, []string{"J3 reached its lower limit (-120.3°)", "jog the other way"}, true, RepullNo},
		{"joint jog limit is the jogged axis", jogEndFacts{cause: "outside_limits_outward", q: []float64{0, 1.99, 0}, lower: lower, upper: upper, jogAxis: 1},
			JogEndJointLimit, []string{"J2 reached its upper limit (114.6°)"}, true, RepullNo},
		{"reach", jogEndFacts{cause: "cartesian_reach", path: true}, JogEndWorkspaceLimit, []string{"leaves the robot's reach"}, true, RepullNo},
		{"stall at the cap", jogEndFacts{cause: "", path: true, stall: 900 * time.Millisecond, stallBy: "cap", governing: 5},
			JogEndSpeedGovernedToZero, []string{"held at zero by the per-sample speed cap near a singularity"}, true, RepullNo},
		{"short zero spell is a release", jogEndFacts{cause: "", path: true, stall: 100 * time.Millisecond, stallBy: "cap"},
			JogEndReleased, []string{"Jog released."}, false, RepullYes},
		{"own Stop", jogEndFacts{cause: "operator_stop"}, JogEndHalted, []string{"Stop was pressed"}, false, RepullAfterRearm},
		{"another client's Stop", jogEndFacts{cause: "operator_stop", endedByOther: "Rosie pendant steamdeck-pendant"},
			JogEndHalted, []string{"Rosie pendant steamdeck-pendant pressed Stop", "Arm before jogging again"}, true, RepullAfterRearm},
		{"native Halt", jogEndFacts{cause: "jog_closed", path: true, stopReason: 3}, JogEndHalted, []string{"halted the robot"}, true, RepullYes},
		{"native inhibit", jogEndFacts{cause: "jog_not_ready", path: true, stopReason: 9}, JogEndInhibited, []string{"robot disarmed", "Arm again"}, true, RepullAfterRearm},
		{"native fault", jogEndFacts{cause: "path_fault", path: true, stopReason: 8}, JogEndGeneratorFault, []string{"path generator faulted"}, true, RepullAfterRearm},
		{"start mismatch", jogEndFacts{cause: "path_start_mismatch", atStart: true}, JogEndStartMismatch, []string{"still settling"}, true, RepullYes},
		{"refused at start", jogEndFacts{cause: "cartesian_resolver_too_slow", atStart: true}, JogEndRefusedAtStart, []string{"could not start (cartesian_resolver_too_slow)"}, true, RepullUnknown},
		{"stale at start", jogEndFacts{cause: "jog_session_stale", atStart: true}, JogEndRefusedAtStart, []string{"could not start"}, true, RepullUnknown},
		{"transport", jogEndFacts{cause: "rt_core_transport_lost"}, JogEndTransportLost, []string{"link to the robot controller dropped"}, true, RepullYes},
		{"session stale", jogEndFacts{cause: "jog_session_stale"}, JogEndSessionStale, []string{"replaced or went stale"}, true, RepullYes},
	}
	for _, tc := range cases {
		c := classifyJogEnd(tc.facts)
		if c.code != tc.code || c.popup != tc.popup || c.repull != tc.repull {
			t.Errorf("%s: got code=%s popup=%v repull=%s (%s), want %s %v %s", tc.name, c.code, c.popup, c.repull, c.text, tc.code, tc.popup, tc.repull)
		}
		for _, want := range tc.text {
			if !strings.Contains(c.text, want) {
				t.Errorf("%s: text %q lacks %q", tc.name, c.text, want)
			}
		}
	}
	if c := classifyJogEnd(jogEndFacts{cause: "operator_stop"}); !c.disarmed {
		t.Fatalf("Stop disarms: %+v", c)
	}
}

// syncBuffer is a log sink the controller's goroutines may write to.
type syncBuffer struct {
	mu  sync.Mutex
	buf bytes.Buffer
}

func (b *syncBuffer) Write(p []byte) (int, error) {
	b.mu.Lock()
	defer b.mu.Unlock()
	return b.buf.Write(p)
}

func (b *syncBuffer) lines(prefix string) []string {
	b.mu.Lock()
	defer b.mu.Unlock()
	var out []string
	for _, line := range strings.Split(b.buf.String(), "\n") {
		if strings.Contains(line, prefix) {
			out = append(out, line)
		}
	}
	return out
}

func captureLog(t *testing.T) *syncBuffer {
	t.Helper()
	sink, previous := &syncBuffer{}, log.Writer()
	log.SetOutput(sink)
	t.Cleanup(func() { log.SetOutput(previous) })
	return sink
}

// A path hold that native stops at the end of its samples (an underrun)
// journals one end line with the evidence and publishes the end, attributed
// to the client whose press began it, on the status body and jog state.
func TestJogEndJournalLineAndStatus(t *testing.T) {
	r, _, lane, _, in := pathFake(t)
	sink := captureLog(t)
	owner := WithClient(context.Background(), Client{ID: "tab-0123456789abcdef", Label: "Browser (Chrome on macOS) on http://127.0.0.1:5189"})
	if _, err := r.CartesianJog(owner, "start", in); err != nil {
		t.Fatal(err)
	}
	// The producer tops the look-ahead up to 125 samples first.
	waitFor(t, "look-ahead", func() bool {
		lane.mu.Lock()
		defer lane.mu.Unlock()
		return lane.received == pathAheadSamples+pathChunkSamples
	})
	// Native consumed to within 2 samples of what was received and stopped
	// there with speed asked for: the underrun the producer retires.
	lane.mu.Lock()
	lane.obs.Consumed, lane.obs.PathS, lane.obs.Underruns, lane.obs.Governing = lane.received-2, float64(lane.received-2)*.001, 1, 6
	lane.mu.Unlock()
	waitFor(t, "retirement", func() bool { return !r.JointJogState().Active })
	lines := sink.lines("jog hold: end ")
	if len(lines) != 1 {
		t.Fatalf("end lines %q", lines)
	}
	line := lines[0]
	for _, want := range []string{"kind=cartesian reason=path_buffer_underrun", "end_code=path_underrun", "seq=1",
		"client_id=tab-0123456789abcdef", `client="Browser (Chrome on macOS) on http://127.0.0.1:5189"`, "generation=7",
		"axis=+X frame=base", "requested=200.0", "speed_unit=mm/s", "distance=123.00 distance_unit=mm", "governing=buffer_end",
		"samples_ahead=2", "last_intent_age_ms=", "lifetime_ms=100", "repull=yes popup=true disarmed=false",
		`text="Jog stopped: the path was not computed fast enough to keep up (2 samples ahead at the end).`,
		"detail=\"cause=path_buffer_underrun: native stopped at the end of the samples received native_stop="} {
		if !strings.Contains(line, want) {
			t.Errorf("end line lacks %q:\n%s", want, line)
		}
	}
	end := r.Status(context.Background()).LastJogEnd
	if end == nil || end.Code != JogEndPathUnderrun || end.ClientID != "tab-0123456789abcdef" || !end.Popup || end.SamplesAhead == nil || *end.SamplesAhead != 2 || end.AgeMS < 0 {
		t.Fatalf("status last_jog_end %+v", end)
	}
	if s := r.JointJogState(); s.LastEnd == nil || s.LastEnd.Seq != 1 || s.Reason != "path_buffer_underrun" {
		t.Fatalf("jog state %+v", s)
	}
}

// A release is journalled and published without a pop-up; another client's
// Stop names that client and pops up for the owner; the owner's own Stop
// does not.
func TestJogEndAttributesStop(t *testing.T) {
	owner := WithClient(context.Background(), Client{ID: "deck-00112233aabbccdd", Label: "rosie-pendant-v5 steamdeck-pendant"})
	other := WithClient(context.Background(), Client{ID: "tab-ffeeddccbbaa9988", Label: "Browser (Firefox on Linux)"})

	r, _, _, _, in := pathFake(t)
	if _, err := r.CartesianJog(owner, "start", in); err != nil {
		t.Fatal(err)
	}
	if _, err := r.CartesianJog(owner, "halt", in); err != nil {
		t.Fatal(err)
	}
	if end := r.Status(context.Background()).LastJogEnd; end == nil || end.Code != JogEndReleased || end.Popup || end.EndedBy != "" || end.Seq != 1 {
		t.Fatalf("release %+v", end)
	}

	r1, _, _, _, in1 := pathFake(t)
	if _, err := r1.CartesianJog(owner, "start", in1); err != nil {
		t.Fatal(err)
	}
	if _, err := r1.Stop(other); err != nil {
		t.Fatal(err)
	}
	end := r1.Status(context.Background()).LastJogEnd
	if end == nil || end.Code != JogEndHalted || !end.Popup || end.EndedBy != "Browser (Firefox on Linux)" || !end.Disarmed ||
		end.ClientID != "deck-00112233aabbccdd" || !strings.Contains(end.Text, "Browser (Firefox on Linux) pressed Stop") {
		t.Fatalf("other client's Stop %+v", end)
	}

	r2, _, _, _, in2 := pathFake(t)
	if _, err := r2.CartesianJog(owner, "start", in2); err != nil {
		t.Fatal(err)
	}
	if _, err := r2.Stop(owner); err != nil {
		t.Fatal(err)
	}
	if end := r2.Status(context.Background()).LastJogEnd; end == nil || end.Code != JogEndHalted || end.Popup || end.EndedBy != "" {
		t.Fatalf("own Stop %+v", end)
	}
}

// A joint jog whose intents stop arriving: OLP's own deadman retires it, and
// the end line says so with the intent age against the lifetime.
func TestJogEndJointDeadman(t *testing.T) {
	r, _, _, in := jointFake(t)
	sink := captureLog(t)
	ctx := WithClient(context.Background(), Client{ID: "deck-00112233aabbccdd", Label: "rosie-pendant-v5"})
	if _, err := r.JointJog(ctx, "begin", in); err != nil {
		t.Fatal(err)
	}
	rtAwait(t, func() bool { return !r.JointJogState().Active })
	end := r.Status(context.Background()).LastJogEnd
	if end == nil || end.Code != JogEndInputDeadlineExpired || end.Kind != "joint" || end.Axis != "+J1" || end.ClientID != "deck-00112233aabbccdd" ||
		end.LastIntentAgeMS < end.LifetimeMS || end.SpeedUnit != "deg/s" || math.Abs(end.RequestedSpeed-0.1*2*jointMoveVelocityReserve*180/math.Pi) > 1e-9 {
		t.Fatalf("joint deadman end %+v", end)
	}
	lines := sink.lines("jog hold: end ")
	if len(lines) != 1 || !strings.Contains(lines[0], "kind=joint reason=jog_input_deadline_expired") ||
		!strings.Contains(lines[0], "end_code=input_deadline_expired") || !strings.Contains(lines[0], "samples_ahead=na") {
		t.Fatalf("end lines %q", lines)
	}
}

// Native holding the path at zero under the per-sample cap while the
// operator holds the trigger, then a release: reported as the stall.
func TestJogEndReleaseAfterCapStall(t *testing.T) {
	r, _, lane, _, in := pathFake(t)
	ctx := WithClient(context.Background(), Client{ID: "tab-0123456789abcdef", Label: "Browser"})
	if _, err := r.CartesianJog(ctx, "start", in); err != nil {
		t.Fatal(err)
	}
	lane.mu.Lock()
	lane.obs.PathSpeed, lane.obs.Governing, lane.obs.Consumed = 0, 5, 10
	lane.mu.Unlock()
	// Keep the deadman fed while stalled, as a held trigger does.
	deadline := time.Now().Add(jogStallReportAfter + 150*time.Millisecond)
	for time.Now().Before(deadline) {
		if _, err := r.CartesianJog(ctx, "intent", in); err != nil {
			t.Fatal(err)
		}
		time.Sleep(30 * time.Millisecond)
	}
	if _, err := r.CartesianJog(ctx, "halt", in); err != nil {
		t.Fatal(err)
	}
	end := r.Status(context.Background()).LastJogEnd
	if end == nil || end.Code != JogEndSpeedGovernedToZero || !end.Popup || end.Governing != "cap" || !strings.Contains(end.Text, "speed cap near a singularity") {
		t.Fatalf("stall %+v", end)
	}
}

// the 1430 cell 2026-10-11 12:19:20: J5's drive tripped Er41.1 during a -Ry hold;
// native closed the session and disarmed, and the lane reported
// jog_session_stale. The operator was told the session "was replaced or went
// stale" and to pull the trigger again. A drive fault outranks every cause it
// produces; an operator's release, Stop or a limit keeps its own words.
func TestJogEndDriveFaultOutranksWhatItCaused(t *testing.T) {
	words := "J5 drive alarm Er41.1 (Motor over-temperature due to locked-rotor)"
	for _, cause := range []string{"jog_session_stale", "jog_not_ready", "rt_core_transport_lost", "path_fault", "unknown_jog_reason_99", "control_session_stale"} {
		c := classifyJogEnd(jogEndFacts{cause: cause, path: true, driveFault: words, jogAxis: -1})
		if c.code != JogEndDriveFault || !c.disarmed || !c.popup || c.repull != RepullAfterRearm ||
			!strings.Contains(c.text, "J5 drive alarm Er41.1 (Motor over-temperature due to locked-rotor)") ||
			!strings.Contains(c.text, "Clear faults, then Arm") || strings.Contains(c.text, "stale") ||
			strings.Contains(c.text, `"`) || strings.Contains(c.detail, `"`) {
			t.Errorf("cause %s with a drive fault: %+v", cause, c)
		}
	}
	for cause, code := range map[string]string{"": JogEndReleased, "operator_stop": JogEndHalted, "joint_limit": JogEndJointLimit, "jacobian_gate": JogEndSingularityGate} {
		if c := classifyJogEnd(jogEndFacts{cause: cause, driveFault: words, jogAxis: -1}); c.code != code {
			t.Errorf("cause %q must keep %s, got %s", cause, code, c.code)
		}
	}
}

func driveAlarmStatus() *control.Status {
	s := &control.Status{Axes: make([]ipcclient.LogicalAxisStatus, 6)}
	s.Core.Axes = make([]ipcclient.AxisStatus, 6)
	for i := range s.Axes {
		s.Axes[i].Readiness = "ready"
	}
	s.Axes[4] = ipcclient.LogicalAxisStatus{Readiness: "faulted", DriveAlarm: "Er41.1", DriveAlarmText: "Motor over-temperature due to locked- rotor"}
	s.Core.Axes[4].ErrorCode = 0x7121
	s.Core.SafetyFaultMask = 16
	s.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Bit: 7, Name: "drive_readiness", AxisMask: 16, Recovery: "reset_after_condition_clears"}}}
	return s
}

// The words come from what native published (the 1430 cell's live status at 12:47),
// never from a fault rt-core raised itself or a warning on a ready drive.
func TestDriveFaultWordsFromNativeStatus(t *testing.T) {
	ids := []string{"J1", "J2", "J3", "J4", "J5", "J6"}
	if got := driveFaultWords(driveAlarmStatus(), ids); got != "J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor)" {
		t.Fatalf("live rosie_1420_v1 status: %q", got)
	}
	undecoded := driveAlarmStatus()
	undecoded.Axes[4].DriveAlarm, undecoded.Axes[4].DriveAlarmText = "", ""
	if got := driveFaultWords(undecoded, ids); got != "J5 drive fault 0x7121" {
		t.Fatalf("undecoded alarm: %q", got)
	}
	warning := driveAlarmStatus()
	warning.Axes[4].Readiness, warning.Core.SafetyFaultMask, warning.Core.Recovery = "ready", 0, nil
	if got := driveFaultWords(warning, ids); got != "" {
		t.Fatalf("a drive warning on a ready axis is not a fault: %q", got)
	}
	own := &control.Status{Axes: make([]ipcclient.LogicalAxisStatus, 6)}
	own.Axes[1].Readiness, own.Core.SafetyFaultMask = "faulted", 2
	own.Core.Recovery = &ipcclient.RecoveryStatus{Faults: []ipcclient.FaultRecovery{{Name: "following_error", AxisMask: 2}}}
	if got := driveFaultWords(own, ids); got != "" {
		t.Fatalf("rt-core's own fault is not a drive fault: %q", got)
	}
	if driveFaultWords(nil, ids) != "" {
		t.Fatal("no observation")
	}
}

// The incident order: the lane reports jog_session_stale before the status
// poll shows the drive alarm. The next observation rewords that end in place
// (same seq and time, so clients update the notice); an older end, or one
// the operator ended, keeps its words.
func TestJogEndReattributedToLateDriveFault(t *testing.T) {
	for _, tc := range []struct {
		name string
		code string
		age  time.Duration
		want string
	}{
		{"stale_then_alarm", JogEndSessionStale, 100 * time.Millisecond, JogEndDriveFault},
		{"old_end", JogEndSessionStale, 2 * jogEndDriveFaultWindow, JogEndSessionStale},
		{"operator_release", JogEndReleased, 100 * time.Millisecond, JogEndReleased},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, _, _ := rtFake(t)
			sink := captureLog(t)
			at := time.Now().Add(-tc.age)
			r.mu.Lock()
			r.lastJogEnd = &JogEnd{Seq: 44, Code: tc.code, Text: "Jog stopped: the jog session was replaced or went stale. Pull the trigger again.",
				Repull: RepullYes, Popup: true, At: at.UTC().Format(time.RFC3339Nano), at: at, Kind: "cartesian", Axis: "-Ry"}
			r.setObservationLocked(driveAlarmStatus())
			end := *r.lastJogEnd
			r.mu.Unlock()
			if end.Code != tc.want || end.Seq != 44 || end.At != at.UTC().Format(time.RFC3339Nano) {
				t.Fatalf("end %+v", end)
			}
			if tc.want != JogEndDriveFault {
				return
			}
			if !end.Disarmed || end.Repull != RepullAfterRearm || !strings.Contains(end.Text, "J5 drive alarm Er41.1") || strings.Contains(end.Text, "stale") ||
				!strings.Contains(end.Detail, "reattributed_from=session_stale") {
				t.Fatalf("reworded end %+v", end)
			}
			if lines := sink.lines("jog hold: end seq=44 code=drive_fault (was session_stale"); len(lines) != 1 {
				t.Fatalf("journal %q", sink.lines("jog hold"))
			}
		})
	}
}
