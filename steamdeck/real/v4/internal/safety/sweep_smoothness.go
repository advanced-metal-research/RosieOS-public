package safety

import "fmt"

type SweepSmoothnessOptions struct {
	MinScore                   float64
	MaxAmpFluctuationMA        float64
	MaxVelocityFluctuation     float64
	MaxMiniStopCount           int
	MaxPositionStepCounts      float64
	MaxTargetStepCounts        float64
	MaxTargetFeedbackGapCounts float64
	RequireAmpTelemetry        bool
	RequireVelocityTelemetry   bool
	RequirePositionStepMetrics bool
}

type SweepSmoothnessMetrics struct {
	MaxAmpFluctuationMA        float64
	VelocityFluctuationP95     float64
	VelocityFluctuationMax     float64
	MiniStopCount              int
	MaxPositionStepCounts      float64
	MaxTargetStepCounts        float64
	MaxTargetFeedbackGapCounts float64
	AmpTelemetrySamples        int
	VelocityTelemetrySamples   int
	PositionStepSamples        int
}

type SweepSmoothnessEvidence struct {
	Passed          bool
	Events          []string
	TelemetryFields map[string]any
	LogMarkers      []string
	FailureReasons  []string
	Score           float64
}

func ValidateSweepSmoothness(metrics SweepSmoothnessMetrics, options SweepSmoothnessOptions) SweepSmoothnessEvidence {
	options = normalizeSweepSmoothnessOptions(options)
	evidence := SweepSmoothnessEvidence{
		Passed:     true,
		Events:     []string{"sweep_smoothness_score", "amp_fluctuation", "velocity_fluctuation"},
		LogMarkers: []string{"sweep-smoothness-score", "amp-fluctuation", "velocity-fluctuation"},
	}
	if options.RequireAmpTelemetry && metrics.AmpTelemetrySamples <= 0 {
		evidence.fail("amp telemetry samples missing")
	}
	if options.RequireVelocityTelemetry && metrics.VelocityTelemetrySamples <= 0 {
		evidence.fail("velocity telemetry samples missing")
	}
	if options.RequirePositionStepMetrics && metrics.PositionStepSamples <= 0 {
		evidence.fail("position step samples missing")
	}
	score := 1.0
	score -= smoothnessPenalty(metrics.MaxAmpFluctuationMA, options.MaxAmpFluctuationMA) * 0.25
	score -= smoothnessPenalty(metrics.VelocityFluctuationP95, options.MaxVelocityFluctuation) * 0.25
	score -= smoothnessPenalty(float64(metrics.MiniStopCount), float64(options.MaxMiniStopCount)) * 0.15
	score -= smoothnessPenalty(metrics.MaxPositionStepCounts, options.MaxPositionStepCounts) * 0.15
	score -= smoothnessPenalty(metrics.MaxTargetStepCounts, options.MaxTargetStepCounts) * 0.10
	score -= smoothnessPenalty(metrics.MaxTargetFeedbackGapCounts, options.MaxTargetFeedbackGapCounts) * 0.10
	if score < 0 {
		score = 0
	}
	evidence.Score = score
	if evidence.Score < options.MinScore {
		evidence.fail("sweep_smoothness_score=%.3f below required %.3f", evidence.Score, options.MinScore)
	}
	evidence.TelemetryFields = map[string]any{
		"sweep_smoothness_score":          evidence.Score,
		"sweep_smoothness_score_min":      options.MinScore,
		"max_amp_fluctuation_ma":          metrics.MaxAmpFluctuationMA,
		"velocity_fluctuation_p95":        metrics.VelocityFluctuationP95,
		"velocity_fluctuation_max":        metrics.VelocityFluctuationMax,
		"mini_stop_count":                 metrics.MiniStopCount,
		"max_position_step_counts":        metrics.MaxPositionStepCounts,
		"max_target_step_counts":          metrics.MaxTargetStepCounts,
		"max_target_feedback_gap_counts":  metrics.MaxTargetFeedbackGapCounts,
		"amp_telemetry_samples":           metrics.AmpTelemetrySamples,
		"velocity_telemetry_samples":      metrics.VelocityTelemetrySamples,
		"position_step_telemetry_samples": metrics.PositionStepSamples,
		"failure_reasons":                 evidence.FailureReasons,
	}
	return evidence.finish()
}

func (e *SweepSmoothnessEvidence) fail(format string, args ...any) {
	e.Passed = false
	e.FailureReasons = append(e.FailureReasons, fmt.Sprintf(format, args...))
}

func (e SweepSmoothnessEvidence) finish() SweepSmoothnessEvidence {
	if e.Passed {
		e.Events = append(e.Events, "sweep_smoothness_pass")
		e.LogMarkers = append(e.LogMarkers, "sweep_smoothness_result=pass")
	} else {
		e.Events = append(e.Events, "sweep_smoothness_fail")
		e.LogMarkers = append(e.LogMarkers, "sweep_smoothness_result=fail")
	}
	return e
}

func normalizeSweepSmoothnessOptions(options SweepSmoothnessOptions) SweepSmoothnessOptions {
	if options.MinScore <= 0 {
		options.MinScore = 0.80
	}
	if options.MaxAmpFluctuationMA <= 0 {
		options.MaxAmpFluctuationMA = 600
	}
	if options.MaxVelocityFluctuation <= 0 {
		options.MaxVelocityFluctuation = 1
	}
	if options.MaxMiniStopCount <= 0 {
		options.MaxMiniStopCount = 1
	}
	if options.MaxPositionStepCounts <= 0 {
		options.MaxPositionStepCounts = 150000
	}
	if options.MaxTargetStepCounts <= 0 {
		options.MaxTargetStepCounts = 150000
	}
	if options.MaxTargetFeedbackGapCounts <= 0 {
		options.MaxTargetFeedbackGapCounts = 10000
	}
	return options
}

func smoothnessPenalty(value, limit float64) float64 {
	if value <= limit || limit <= 0 {
		return 0
	}
	penalty := (value - limit) / limit
	if penalty > 1 {
		return 1
	}
	return penalty
}
