package safety

import "testing"

func TestValidatePostReleaseStopPassesZeroHoldNoDrift(t *testing.T) {
	evidence := ValidatePostReleaseStop([]PostReleaseSample{
		{
			Source:           "motion-server",
			Phase:            "active",
			MotionCommandAbs: 0.7,
			ActiveBeforeStop: true,
		},
		{
			Source:             "motion-server",
			Phase:              "post-release",
			MotionCommandAbs:   0,
			AxisTargetCounts:   []int64{100, 200},
			AxisPositionCounts: []int64{101, 199},
			ReleaseCommand:     true,
		},
		{
			Source:             "motion-server",
			Phase:              "post-release",
			MotionCommandAbs:   0,
			AxisTargetCounts:   []int64{100, 200},
			AxisPositionCounts: []int64{102, 198},
		},
	}, PostReleaseOptions{MaxPositionDeltaCounts: 4, RequireActiveBeforeStop: true})

	if !evidence.Passed {
		t.Fatalf("post-release evidence failed unexpectedly: %+v", evidence)
	}
	for _, want := range []string{"post_release_start", "release_command_sent", "release_zero_hold_applied", "post_release_no_drift", "post_release_pass"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("post-release events missing %q: %+v", want, evidence.Events)
		}
	}
	if evidence.TelemetryFields["release_command_seen"] != true ||
		evidence.TelemetryFields["active_before_stop"] != true ||
		evidence.TelemetryFields["post_release_max_motion_command_abs"] != 0.0 ||
		evidence.TelemetryFields["stuck_open_motion_command"] != false {
		t.Fatalf("post-release telemetry fields missing pass evidence: %+v", evidence.TelemetryFields)
	}
	if !containsString(evidence.LogMarkers, "post_release_result=pass") ||
		!containsString(evidence.LogMarkers, "stuck-open-check") {
		t.Fatalf("post-release log markers missing pass/stuck-open proof: %+v", evidence.LogMarkers)
	}
}

func TestValidatePostReleaseStopFailsStuckOpenMotion(t *testing.T) {
	evidence := ValidatePostReleaseStop([]PostReleaseSample{
		{
			Source:           "motion-server",
			Phase:            "active",
			MotionCommandAbs: 0.4,
			ActiveBeforeStop: true,
		},
		{
			Source:           "motion-server",
			Phase:            "post-release",
			MotionCommandAbs: 0.1,
			ReleaseCommand:   true,
		},
	}, PostReleaseOptions{MaxMotionCommandAbs: 0, RequireActiveBeforeStop: true})

	if evidence.Passed {
		t.Fatalf("post-release evidence passed despite residual motion: %+v", evidence)
	}
	if evidence.TelemetryFields["stuck_open_motion_command"] != true ||
		evidence.TelemetryFields["post_release_max_motion_command_abs"] != 0.1 {
		t.Fatalf("stuck-open telemetry fields missing: %+v", evidence.TelemetryFields)
	}
	if !containsString(evidence.Events, "stuck_open_motion_command") ||
		!containsString(evidence.LogMarkers, "post_release_result=fail") {
		t.Fatalf("stuck-open failure evidence missing: events=%+v markers=%+v", evidence.Events, evidence.LogMarkers)
	}
}

func TestValidatePostReleaseStopFailsMissingReleaseOrDrift(t *testing.T) {
	evidence := ValidatePostReleaseStop([]PostReleaseSample{
		{
			Source:             "rtcore",
			Phase:              "post-release",
			MotionCommandAbs:   0,
			AxisTargetCounts:   []int64{10},
			AxisPositionCounts: []int64{10},
		},
		{
			Source:             "rtcore",
			Phase:              "post-release",
			MotionCommandAbs:   0,
			AxisTargetCounts:   []int64{11},
			AxisPositionCounts: []int64{20},
		},
	}, PostReleaseOptions{MaxTargetDeltaCounts: 0, MaxPositionDeltaCounts: 4})

	if evidence.Passed {
		t.Fatalf("post-release evidence passed despite missing release and drift: %+v", evidence)
	}
	if evidence.TelemetryFields["release_command_seen"] != false ||
		evidence.TelemetryFields["post_release_target_delta_counts"] != int64(1) ||
		evidence.TelemetryFields["post_release_position_delta_counts"] != int64(10) {
		t.Fatalf("missing-release/drift telemetry fields missing: %+v", evidence.TelemetryFields)
	}
}
