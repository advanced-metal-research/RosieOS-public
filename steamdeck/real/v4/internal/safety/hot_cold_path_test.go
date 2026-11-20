package safety

import "testing"

func TestValidateHotColdPathPassesFastOutputBeforeColdSnapshot(t *testing.T) {
	evidence := ValidateHotColdPath([]HotColdPathSample{
		{
			CommandID:        "cmd-1",
			CommandSentNS:    1_000_000,
			CommandReceiveNS: 2_000_000,
			HotOutputNS:      7_000_000,
			ColdSnapshotNS:   20_000_000,
		},
		{
			CommandID:        "cmd-2",
			CommandSentNS:    30_000_000,
			CommandReceiveNS: 31_000_000,
			HotOutputNS:      39_000_000,
			ColdSnapshotNS:   90_000_000,
		},
	}, HotColdPathOptions{MaxHotPathLatencyUS: 10_000})

	if !evidence.Passed {
		t.Fatalf("hot/cold evidence failed unexpectedly: %+v", evidence)
	}
	for _, want := range []string{"command_sent", "command_received", "command_applied", "hot_path_stage_latency", "cold_path_snapshot", "hot_cold_path_pass"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("events missing %q: %+v", want, evidence.Events)
		}
	}
	if evidence.TelemetryFields["latency_sample_count"] != 2 ||
		evidence.TelemetryFields["receive_to_output_latency_us"] != int64(8000) ||
		evidence.TelemetryFields["latency_max_ms"] != 8.0 ||
		evidence.TelemetryFields["latency_budget_ms"] != 10.0 ||
		evidence.TelemetryFields["latency_budget_pass"] != true ||
		evidence.TelemetryFields["latency_warning_pass"] != true ||
		evidence.TelemetryFields["latency_target_pass"] != true {
		t.Fatalf("unexpected hot-path telemetry fields: %+v", evidence.TelemetryFields)
	}
	for _, want := range []string{"hot-path-stage-latency", "latency-budget", "latency-warning", "latency-target", "cold-path-snapshot", "hot-output-before-cold-path", "hot_cold_path_result=pass"} {
		if !containsString(evidence.LogMarkers, want) {
			t.Fatalf("log markers missing %q: %+v", want, evidence.LogMarkers)
		}
	}
}

func TestValidateHotColdPathFailsColdSnapshotBeforeHotOutput(t *testing.T) {
	evidence := ValidateHotColdPath([]HotColdPathSample{
		{
			CommandID:        "cmd-1",
			CommandReceiveNS: 10_000,
			HotOutputNS:      30_000,
			ColdSnapshotNS:   20_000,
		},
		{
			CommandID:        "cmd-2",
			CommandReceiveNS: 40_000,
			HotOutputNS:      50_000,
			ColdSnapshotNS:   80_000,
		},
	}, HotColdPathOptions{MaxHotPathLatencyUS: 100})

	if evidence.Passed {
		t.Fatalf("hot/cold evidence passed despite cold snapshot before hot output: %+v", evidence)
	}
	if evidence.TelemetryFields["cold_before_hot_count"] != 1 {
		t.Fatalf("cold-before-hot count missing: %+v", evidence.TelemetryFields)
	}
	if !containsString(evidence.Events, "hot_cold_path_fail") ||
		!containsString(evidence.LogMarkers, "hot_cold_path_result=fail") {
		t.Fatalf("failure evidence missing: events=%+v markers=%+v", evidence.Events, evidence.LogMarkers)
	}
}

func TestValidateHotColdPathFailsSlowOrSparseHotPath(t *testing.T) {
	evidence := ValidateHotColdPath([]HotColdPathSample{
		{
			CommandID:        "cmd-1",
			CommandReceiveNS: 1_000_000,
			HotOutputNS:      80_000_000,
			ColdSnapshotNS:   90_000_000,
		},
	}, HotColdPathOptions{MaxHotPathLatencyUS: 40_000, MinSamples: 2})

	if evidence.Passed {
		t.Fatalf("hot/cold evidence passed despite sparse slow sample: %+v", evidence)
	}
	if evidence.TelemetryFields["latency_sample_count"] != 1 ||
		evidence.TelemetryFields["latency_max_ms"] != 79.0 ||
		evidence.TelemetryFields["latency_budget_pass"] != false ||
		evidence.TelemetryFields["latency_warning_pass"] != false ||
		evidence.TelemetryFields["latency_target_pass"] != false {
		t.Fatalf("slow/sparse telemetry fields missing: %+v", evidence.TelemetryFields)
	}
}
