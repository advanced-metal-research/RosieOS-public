package safety

import "testing"

func TestValidateCommandAuthorityPassesAllowedCommandWithUnlock(t *testing.T) {
	evidence := ValidateCommandAuthority([]CommandObservation{
		{Source: "deck", Command: "joint_jog"},
		{Source: "motion-server", Command: "zero_hold"},
	}, CommandAuthorityOptions{
		AllowedCommands:         []string{"joint_jog", "zero_hold"},
		LiveMotionUnlockPresent: true,
		RequireLiveMotionUnlock: true,
	})

	if !evidence.Passed {
		t.Fatalf("command authority failed unexpectedly: %+v", evidence)
	}
	if evidence.TelemetryFields["command_authority_pass"] != true ||
		evidence.TelemetryFields["forbidden_command_detected"] != false ||
		evidence.TelemetryFields["unexpected_command_count"] != 0 {
		t.Fatalf("command authority pass telemetry missing: %+v", evidence.TelemetryFields)
	}
	for _, want := range []string{"command_authority_start", "command_authority_pass"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("events missing %q: %+v", want, evidence.Events)
		}
	}
}

func TestValidateCommandAuthorityFailsForbiddenCommand(t *testing.T) {
	evidence := ValidateCommandAuthority([]CommandObservation{
		{Source: "rtcore", Command: "hm35"},
	}, CommandAuthorityOptions{
		AllowedCommands:         []string{"joint_jog"},
		LiveMotionUnlockPresent: true,
		RequireLiveMotionUnlock: true,
	})

	if evidence.Passed {
		t.Fatalf("command authority passed despite forbidden command: %+v", evidence)
	}
	if evidence.TelemetryFields["forbidden_command_detected"] != true ||
		evidence.TelemetryFields["forbidden_command_count"] != 1 ||
		evidence.TelemetryFields["unexpected_command_count"] != 0 {
		t.Fatalf("forbidden command telemetry missing: %+v", evidence.TelemetryFields)
	}
	if !containsString(evidence.Events, "forbidden_command_detected") {
		t.Fatalf("forbidden command event missing: %+v", evidence.Events)
	}
}

func TestValidateCommandAuthorityFailsMissingLiveMotionUnlock(t *testing.T) {
	evidence := ValidateCommandAuthority([]CommandObservation{
		{Source: "deck", Command: "joint_jog"},
	}, CommandAuthorityOptions{
		AllowedCommands:         []string{"joint_jog"},
		RequireLiveMotionUnlock: true,
	})

	if evidence.Passed {
		t.Fatalf("command authority passed without live-motion unlock: %+v", evidence)
	}
	if evidence.TelemetryFields["command_authority_pass"] != false {
		t.Fatalf("missing unlock failure telemetry missing: %+v", evidence.TelemetryFields)
	}
}
