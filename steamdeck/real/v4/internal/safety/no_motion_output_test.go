package safety

import "testing"

func TestValidateNoMotionOutputPassesBlockedAndBoundedZeroHold(t *testing.T) {
	evidence := ValidateNoMotionOutput([]MotionOutputObservation{
		{Source: "motion-server", CommandKind: "blocked", MotionOutputBlocked: true},
		{Source: "rtcore", CommandKind: "zero_hold", BoundedZeroHold: true},
	}, NoMotionOutputOptions{RequireStaleInputBlocked: true})

	if !evidence.Passed {
		t.Fatalf("no-motion output failed unexpectedly: %+v", evidence)
	}
	for _, want := range []string{"no_motion_output_start", "no_motion_output_pass"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("events missing %q: %+v", want, evidence.Events)
		}
	}
	if evidence.TelemetryFields["no_motion_output_pass"] != true ||
		evidence.TelemetryFields["accepted_nonzero_command_count"] != 0 ||
		evidence.TelemetryFields["unexpected_nonzero_axes_count"] != 0 ||
		evidence.TelemetryFields["active_motion_command_count"] != 0 ||
		evidence.TelemetryFields["bounded_zero_hold_only"] != true ||
		evidence.TelemetryFields["stale_input_no_new_motion"] != true {
		t.Fatalf("pass telemetry fields missing: %+v", evidence.TelemetryFields)
	}
}

func TestValidateNoMotionOutputFailsActiveMotionAndNonzeroAxes(t *testing.T) {
	evidence := ValidateNoMotionOutput([]MotionOutputObservation{
		{
			Source:                 "motion-server",
			CommandKind:            "joint_jog",
			AcceptedNonzeroCommand: true,
			UnexpectedNonzeroAxes:  2,
			ActiveMotionCommand:    true,
		},
	}, NoMotionOutputOptions{RequireStaleInputBlocked: true})

	if evidence.Passed {
		t.Fatalf("no-motion output passed despite active motion: %+v", evidence)
	}
	for _, want := range []string{"no_motion_output_fail", "motion_output_blocked"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("events missing %q: %+v", want, evidence.Events)
		}
	}
	if evidence.TelemetryFields["no_motion_output_pass"] != false ||
		evidence.TelemetryFields["accepted_nonzero_command_count"] != 1 ||
		evidence.TelemetryFields["unexpected_nonzero_axes_count"] != 2 ||
		evidence.TelemetryFields["active_motion_command_count"] != 1 ||
		evidence.TelemetryFields["bounded_zero_hold_only"] != false {
		t.Fatalf("failure telemetry fields missing: %+v", evidence.TelemetryFields)
	}
}

func TestValidateNoMotionOutputFailsStaleInputGeneratedMotion(t *testing.T) {
	evidence := ValidateNoMotionOutput([]MotionOutputObservation{
		{Source: "deck", CommandKind: "stale_joint_jog", StaleInputGenerated: true},
	}, NoMotionOutputOptions{RequireStaleInputBlocked: true})

	if evidence.Passed {
		t.Fatalf("no-motion output passed despite stale generated motion: %+v", evidence)
	}
	if evidence.TelemetryFields["stale_input_no_new_motion"] != false {
		t.Fatalf("stale-input telemetry missing: %+v", evidence.TelemetryFields)
	}
}
