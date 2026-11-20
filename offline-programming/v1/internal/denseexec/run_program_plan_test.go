package denseexec

import (
	"strings"
	"testing"
)

func TestNextRunStep(t *testing.T) {
	resettable := []recoverFault{{Name: "drive_readiness", Axes: []string{"J5"}, Recovery: "reset_after_condition_clears"}}
	cases := []struct {
		name   string
		facts  runFacts
		action string
		code   string
	}{
		{"a program playing refuses", runFacts{recoverFacts: recoverFacts{Busy: "A program"}}, "refuse", "motion_active"},
		{"unconfirmed Stop is confirmed first", runFacts{recoverFacts: recoverFacts{StopLatched: true}}, "confirm_stop", ""},
		{"another controller refuses", runFacts{recoverFacts: recoverFacts{ForeignHolder: true}}, "refuse", "control_held_elsewhere"},
		{"Home invalid refuses, never re-homes", runFacts{recoverFacts: recoverFacts{HomeInvalid: []string{"J2"}}}, "refuse", "home_invalid"},
		{"a rehome fault refuses before the question", runFacts{recoverFacts: recoverFacts{Faults: []recoverFault{{Name: "coordinate_reference", Recovery: "rehome_required"}}}}, "refuse", "fault_not_resettable"},
		{"a resettable fault is reset while disarmed", runFacts{recoverFacts: recoverFacts{Faults: resettable}}, "reset", ""},
		{"disarmed at the start asks", runFacts{AtStart: true}, "confirm", ""},
		{"armed away from the start asks", runFacts{recoverFacts: recoverFacts{Armed: true}}, "confirm", ""},
		{"a move confirmation does not stand for Arm", runFacts{MoveConfirmed: true}, "confirm", ""},
		{"confirmed, disarmed, away: Arm first", runFacts{recoverFacts: recoverFacts{ArmConfirmed: true}, MoveConfirmed: true}, "arm", ""},
		{"armed away, confirmed: move", runFacts{recoverFacts: recoverFacts{Armed: true}, MoveConfirmed: true}, "move", ""},
		{"move tried, still away: refuse", runFacts{recoverFacts: recoverFacts{Armed: true, Tried: map[string]bool{"move": true}}, MoveConfirmed: true}, "refuse", "start_not_reached"},
		{"disarmed at the start, confirmed, not staged: Load before Arm", runFacts{recoverFacts: recoverFacts{ArmConfirmed: true}, AtStart: true}, "load", ""},
		{"staged, disarmed, confirmed: Arm", runFacts{recoverFacts: recoverFacts{ArmConfirmed: true}, AtStart: true, Loaded: true}, "arm", ""},
		{"staged before this press moved: Load again", runFacts{recoverFacts: recoverFacts{Armed: true, Tried: map[string]bool{"move": true}}, AtStart: true, Loaded: true}, "load", ""},
		{"armed, at the start, staged: Play with no question", runFacts{recoverFacts: recoverFacts{Armed: true}, AtStart: true, Loaded: true}, "play", ""},
		{"Play tried and not running: refuse", runFacts{recoverFacts: recoverFacts{Armed: true, Tried: map[string]bool{"play": true}}, AtStart: true, Loaded: true}, "refuse", "play_unconfirmed"},
		{"a loop asks once even armed at the start", runFacts{recoverFacts: recoverFacts{Armed: true}, AtStart: true, Loaded: true, Repeat: true}, "confirm", ""},
		{"a confirmed loop plays", runFacts{recoverFacts: recoverFacts{Armed: true}, AtStart: true, Loaded: true, Repeat: true, LoopConfirmed: true}, "play", ""},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			if c.facts.Tried == nil {
				c.facts.Tried = map[string]bool{}
			}
			d := nextRunStep(c.facts)
			if d.Action != c.action || d.Code != c.code {
				t.Fatalf("got %+v, want %s %s", d, c.action, c.code)
			}
		})
	}
}

func TestRunActionNamesWhatThePressDoes(t *testing.T) {
	if runActionFor(true, false) != "Arm and run" || runActionFor(false, true) != "Move to program start and run" || runActionFor(true, true) != "Arm, move to program start and run" {
		t.Fatal("confirmation names")
	}
}

func TestRunJointsTolerance(t *testing.T) {
	joints := []RecoverJoint{{Axis: 0, ID: "J1", FromDeg: 10}, {Axis: 1, ID: "J2", FromDeg: 0}}
	// 0.001 rad is 0.0573°: J1 0.05° off stays, J2 0.06° off moves.
	placed, why := runJoints(joints, []float64{(10 + 0.05) * 3.141592653589793 / 180, 0.06 * 3.141592653589793 / 180}, []float64{0.001, 0.001})
	if why != "" || placed[0].Moves || !placed[1].Moves {
		t.Fatalf("%+v %q", placed, why)
	}
	if _, why := runJoints(joints, []float64{0, 0}, []float64{0.001, 0}); !strings.Contains(why, "J2") {
		t.Fatalf("a joint without a tolerance was judged: %q", why)
	}
}

func TestRunConfirmationStaleOnANewStart(t *testing.T) {
	confirmed := []RecoverJoint{{Axis: 0, ID: "J1", FromDeg: 10, ToDeg: 20, Moves: true}}
	if stale := runConfirmationStale(confirmed, []RecoverJoint{{Axis: 0, ID: "J1", FromDeg: 10, ToDeg: 20, Moves: true}}); stale != "" {
		t.Fatal(stale)
	}
	if stale := runConfirmationStale(confirmed, []RecoverJoint{{Axis: 0, ID: "J1", FromDeg: 10, ToDeg: 25, Moves: true}}); !strings.Contains(stale, "start changed") {
		t.Fatal(stale)
	}
	if stale := runConfirmationStale(confirmed, []RecoverJoint{{Axis: 0, ID: "J1", FromDeg: 11, ToDeg: 20, Moves: true}}); !strings.Contains(stale, "Run program again") {
		t.Fatal(stale)
	}
}

func TestLoopWordsAndRepeat(t *testing.T) {
	if loopActionFor(false, false, "loop", 0) != "Loop until stopped" || loopActionFor(true, true, "count", 5) != "Arm, move to program start and run 5 times" {
		t.Fatal("loop names")
	}
	for _, c := range []struct {
		in   RunProgramRequest
		mode string
		bad  bool
	}{
		{RunProgramRequest{}, "", false}, {RunProgramRequest{Repeat: "count", Count: 1}, "", false}, {RunProgramRequest{Repeat: "count", Count: 4}, "count", false},
		{RunProgramRequest{Repeat: "count", Count: 0}, "", true}, {RunProgramRequest{Repeat: "loop"}, "loop", false}, {RunProgramRequest{Repeat: "forever"}, "", true},
	} {
		mode, _, why := repeatOf(c.in)
		if mode != c.mode || (why != "") != c.bad {
			t.Fatalf("%+v: %q %q", c.in, mode, why)
		}
	}
}
