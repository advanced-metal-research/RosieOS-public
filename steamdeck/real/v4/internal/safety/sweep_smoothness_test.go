package safety

import "testing"

func TestValidateSweepSmoothnessPassesStableSweep(t *testing.T) {
	evidence := ValidateSweepSmoothness(SweepSmoothnessMetrics{
		MaxAmpFluctuationMA:        250,
		VelocityFluctuationP95:     0.25,
		VelocityFluctuationMax:     0.45,
		MiniStopCount:              0,
		MaxPositionStepCounts:      50000,
		MaxTargetStepCounts:        52000,
		MaxTargetFeedbackGapCounts: 2000,
		AmpTelemetrySamples:        20,
		VelocityTelemetrySamples:   20,
		PositionStepSamples:        20,
	}, SweepSmoothnessOptions{
		MinScore:                   0.90,
		RequireAmpTelemetry:        true,
		RequireVelocityTelemetry:   true,
		RequirePositionStepMetrics: true,
	})

	if !evidence.Passed {
		t.Fatalf("smooth sweep evidence failed unexpectedly: %+v", evidence)
	}
	if evidence.TelemetryFields["sweep_smoothness_score"].(float64) < 0.90 ||
		evidence.TelemetryFields["max_amp_fluctuation_ma"] != 250.0 ||
		evidence.TelemetryFields["velocity_fluctuation_p95"] != 0.25 {
		t.Fatalf("smoothness telemetry fields missing: %+v", evidence.TelemetryFields)
	}
	for _, want := range []string{"sweep_smoothness_score", "amp_fluctuation", "velocity_fluctuation", "sweep_smoothness_pass"} {
		if !containsString(evidence.Events, want) {
			t.Fatalf("smoothness events missing %q: %+v", want, evidence.Events)
		}
	}
}

func TestValidateSweepSmoothnessFailsRoughSweep(t *testing.T) {
	evidence := ValidateSweepSmoothness(SweepSmoothnessMetrics{
		MaxAmpFluctuationMA:        1200,
		VelocityFluctuationP95:     2.0,
		VelocityFluctuationMax:     3.0,
		MiniStopCount:              4,
		MaxPositionStepCounts:      400000,
		MaxTargetStepCounts:        350000,
		MaxTargetFeedbackGapCounts: 50000,
		AmpTelemetrySamples:        30,
		VelocityTelemetrySamples:   30,
		PositionStepSamples:        30,
	}, SweepSmoothnessOptions{MinScore: 0.80})

	if evidence.Passed {
		t.Fatalf("rough sweep passed despite score regression: %+v", evidence)
	}
	if !containsString(evidence.Events, "sweep_smoothness_fail") ||
		!containsString(evidence.LogMarkers, "sweep_smoothness_result=fail") {
		t.Fatalf("smoothness failure evidence missing: events=%+v markers=%+v", evidence.Events, evidence.LogMarkers)
	}
}

func TestValidateSweepSmoothnessFailsMissingRequiredTelemetry(t *testing.T) {
	evidence := ValidateSweepSmoothness(SweepSmoothnessMetrics{}, SweepSmoothnessOptions{
		RequireAmpTelemetry:        true,
		RequireVelocityTelemetry:   true,
		RequirePositionStepMetrics: true,
	})

	if evidence.Passed {
		t.Fatalf("smoothness evidence passed despite missing required telemetry: %+v", evidence)
	}
	if len(evidence.FailureReasons) < 3 {
		t.Fatalf("missing telemetry failures not captured: %+v", evidence.FailureReasons)
	}
}
