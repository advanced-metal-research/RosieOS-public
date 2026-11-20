package safety

import "testing"

func TestValidateNeutralWindowPassesIdleNoDrift(t *testing.T) {
	evidence := ValidateNeutralWindow([]NeutralSample{
		{
			Source:             "rtcore-before",
			JoystickAxes:       []float64{0.0, 0.01},
			AxisTargetCounts:   []int64{100, 200},
			AxisPositionCounts: []int64{101, 199},
		},
		{
			Source:             "rtcore-after",
			JoystickAxes:       []float64{0.0, -0.01},
			AxisTargetCounts:   []int64{100, 200},
			AxisPositionCounts: []int64{102, 198},
		},
	}, NeutralWindowOptions{MaxPositionDeltaCounts: 4})

	if !evidence.Passed {
		t.Fatalf("neutral window evidence failed unexpectedly: %+v", evidence)
	}
	for _, want := range []string{"neutral_window_start", "neutral_window_pass"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("events missing %q: %+v", want, evidence.Events)
		}
	}
	if evidence.TelemetryFields["axis_target_delta_counts"] != int64(0) ||
		evidence.TelemetryFields["axis_position_delta_counts"] != int64(1) ||
		evidence.TelemetryFields["stuck_open_motion_command"] != false {
		t.Fatalf("unexpected telemetry fields: %+v", evidence.TelemetryFields)
	}
	if !containsString(evidence.LogMarkers, "neutral_window_result=pass") ||
		!containsString(evidence.LogMarkers, "stuck-open-check") ||
		!containsString(evidence.LogMarkers, "no-target-walk") {
		t.Fatalf("log markers do not prove neutral pass/no target walk/stuck-open check: %+v", evidence.LogMarkers)
	}
}

func TestValidateNeutralWindowFailsTargetWalkAndActiveJog(t *testing.T) {
	evidence := ValidateNeutralWindow([]NeutralSample{
		{
			Source:             "rtcore-before",
			JoystickAxes:       []float64{0},
			AxisTargetCounts:   []int64{100},
			AxisPositionCounts: []int64{100},
		},
		{
			Source:             "rtcore-after",
			JoystickAxes:       []float64{0.03},
			ActiveJog:          true,
			AxisTargetCounts:   []int64{105},
			AxisPositionCounts: []int64{100},
		},
	}, NeutralWindowOptions{JoystickDeadband: 0.02, MaxTargetDeltaCounts: 0, MaxPositionDeltaCounts: 4})

	if evidence.Passed {
		t.Fatalf("neutral window passed despite active jog and target walk: %+v", evidence)
	}
	for _, want := range []string{"neutral_window_fail", "unexpected_motion", "unexpected_target_walk"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("events missing %q: %+v", want, evidence.Events)
		}
	}
	if evidence.TelemetryFields["joystick_max_abs_axis"] != 0.03 ||
		evidence.TelemetryFields["active_jog"] != true ||
		evidence.TelemetryFields["axis_target_delta_counts"] != int64(5) {
		t.Fatalf("failure telemetry fields did not capture active jog/target walk: %+v", evidence.TelemetryFields)
	}
}

func TestValidateNeutralWindowFailsStuckOpenAndUnauthorizedCommand(t *testing.T) {
	evidence := ValidateNeutralWindow([]NeutralSample{
		{
			Source:                   "motion-server",
			AxisTargetCounts:         []int64{1},
			AxisPositionCounts:       []int64{1},
			UnauthorizedCommandCount: 1,
			StuckOpenMotionCommand:   true,
		},
		{
			Source:             "motion-server",
			AxisTargetCounts:   []int64{1},
			AxisPositionCounts: []int64{1},
		},
	}, NeutralWindowOptions{})

	if evidence.Passed {
		t.Fatalf("neutral window passed despite stuck-open command: %+v", evidence)
	}
	if evidence.TelemetryFields["unauthorized_command_count"] != 1 ||
		evidence.TelemetryFields["stuck_open_motion_command"] != true {
		t.Fatalf("stuck-open telemetry fields missing: %+v", evidence.TelemetryFields)
	}
}

func containsString(items []string, want string) bool {
	for _, item := range items {
		if item == want {
			return true
		}
	}
	return false
}
