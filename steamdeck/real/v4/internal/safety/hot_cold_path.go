package safety

import (
	"fmt"
	"math"
	"sort"
)

const (
	DefaultMaxHotPathLatencyUS     = 40000
	DefaultWarningHotPathLatencyUS = 40000
	DefaultTargetHotPathLatencyUS  = 30000
)

type HotColdPathOptions struct {
	MaxHotPathLatencyUS     int64
	WarningHotPathLatencyUS int64
	TargetHotPathLatencyUS  int64
	MinSamples              int
}

type HotColdPathSample struct {
	CommandID        string
	CommandSentNS    int64
	CommandReceiveNS int64
	HotOutputNS      int64
	ColdSnapshotNS   int64
}

type HotColdPathEvidence struct {
	Passed              bool
	Events              []string
	TelemetryFields     map[string]any
	LogMarkers          []string
	FailureReasons      []string
	LatencyP50US        int64
	LatencyP95US        int64
	LatencyP99US        int64
	LatencyMaxUS        int64
	ReceiveToOutputUS   int64
	SampleCount         int
	ColdBeforeHotCount  int
	MissingHotPathCount int
}

func ValidateHotColdPath(samples []HotColdPathSample, options HotColdPathOptions) HotColdPathEvidence {
	options = normalizeHotColdPathOptions(options)
	evidence := HotColdPathEvidence{
		Passed:     true,
		Events:     []string{"command_sent", "command_received", "command_applied", "hot_path_stage_latency", "cold_path_snapshot"},
		LogMarkers: []string{"hot-path-stage-latency", "latency-budget", "latency-warning", "latency-target", "cold-path-snapshot", "hot-output-before-cold-path"},
		TelemetryFields: map[string]any{
			"latency_budget_us":    options.MaxHotPathLatencyUS,
			"latency_budget_ms":    float64(options.MaxHotPathLatencyUS) / 1000,
			"latency_warning_us":   options.WarningHotPathLatencyUS,
			"latency_warning_ms":   float64(options.WarningHotPathLatencyUS) / 1000,
			"latency_target_us":    options.TargetHotPathLatencyUS,
			"latency_target_ms":    float64(options.TargetHotPathLatencyUS) / 1000,
			"latency_sample_count": len(samples),
		},
		SampleCount: len(samples),
	}
	if len(samples) < options.MinSamples {
		evidence.fail("latency_sample_count=%d below required %d", len(samples), options.MinSamples)
	}

	latencies := make([]int64, 0, len(samples))
	for index, sample := range samples {
		if sample.CommandReceiveNS <= 0 || sample.HotOutputNS <= 0 {
			evidence.MissingHotPathCount++
			evidence.fail("sample %d missing receive/output timestamp", index)
			continue
		}
		if sample.CommandSentNS > 0 && sample.CommandReceiveNS < sample.CommandSentNS {
			evidence.fail("sample %d receive timestamp precedes command send", index)
		}
		if sample.HotOutputNS < sample.CommandReceiveNS {
			evidence.fail("sample %d hot output precedes command receive", index)
			continue
		}
		if sample.ColdSnapshotNS > 0 && sample.ColdSnapshotNS < sample.HotOutputNS {
			evidence.ColdBeforeHotCount++
			evidence.fail("sample %d cold snapshot precedes hot output", index)
		}
		latencyUS := nsToUS(sample.HotOutputNS - sample.CommandReceiveNS)
		latencies = append(latencies, latencyUS)
		if latencyUS > evidence.LatencyMaxUS {
			evidence.LatencyMaxUS = latencyUS
			evidence.ReceiveToOutputUS = latencyUS
		}
	}

	if len(latencies) > 0 {
		sort.Slice(latencies, func(left, right int) bool {
			return latencies[left] < latencies[right]
		})
		evidence.LatencyP50US = percentileNearestRank(latencies, 50)
		evidence.LatencyP95US = percentileNearestRank(latencies, 95)
		evidence.LatencyP99US = percentileNearestRank(latencies, 99)
		if evidence.LatencyMaxUS > options.MaxHotPathLatencyUS {
			evidence.fail("latency_max_us=%d exceeds budget %d", evidence.LatencyMaxUS, options.MaxHotPathLatencyUS)
		}
	}
	return evidence.finish()
}

func (e *HotColdPathEvidence) fail(format string, args ...any) {
	e.Passed = false
	e.FailureReasons = append(e.FailureReasons, fmt.Sprintf(format, args...))
}

func (e HotColdPathEvidence) finish() HotColdPathEvidence {
	if e.Passed {
		e.Events = append(e.Events, "hot_cold_path_pass")
		e.LogMarkers = append(e.LogMarkers, "hot_cold_path_result=pass")
	} else {
		e.Events = append(e.Events, "hot_cold_path_fail")
		e.LogMarkers = append(e.LogMarkers, "hot_cold_path_result=fail")
	}
	e.TelemetryFields["latency_p50_ms"] = float64(e.LatencyP50US) / 1000
	e.TelemetryFields["latency_p95_ms"] = float64(e.LatencyP95US) / 1000
	e.TelemetryFields["latency_p99_ms"] = float64(e.LatencyP99US) / 1000
	e.TelemetryFields["latency_max_ms"] = float64(e.LatencyMaxUS) / 1000
	e.TelemetryFields["latency_budget_pass"] = e.LatencyMaxUS <= int64Number(e.TelemetryFields["latency_budget_us"])
	e.TelemetryFields["latency_warning_pass"] = e.LatencyP95US <= int64Number(e.TelemetryFields["latency_warning_us"])
	e.TelemetryFields["latency_target_pass"] = e.LatencyP95US <= int64Number(e.TelemetryFields["latency_target_us"])
	e.TelemetryFields["receive_to_output_latency_us"] = e.ReceiveToOutputUS
	e.TelemetryFields["cold_before_hot_count"] = e.ColdBeforeHotCount
	e.TelemetryFields["missing_hot_path_count"] = e.MissingHotPathCount
	e.TelemetryFields["failure_reasons"] = e.FailureReasons
	return e
}

func normalizeHotColdPathOptions(options HotColdPathOptions) HotColdPathOptions {
	if options.MaxHotPathLatencyUS <= 0 {
		options.MaxHotPathLatencyUS = DefaultMaxHotPathLatencyUS
	}
	if options.WarningHotPathLatencyUS <= 0 {
		options.WarningHotPathLatencyUS = DefaultWarningHotPathLatencyUS
	}
	if options.TargetHotPathLatencyUS <= 0 {
		options.TargetHotPathLatencyUS = DefaultTargetHotPathLatencyUS
	}
	if options.WarningHotPathLatencyUS > options.MaxHotPathLatencyUS {
		options.WarningHotPathLatencyUS = options.MaxHotPathLatencyUS
	}
	if options.TargetHotPathLatencyUS > options.WarningHotPathLatencyUS {
		options.TargetHotPathLatencyUS = options.WarningHotPathLatencyUS
	}
	if options.MinSamples <= 0 {
		options.MinSamples = 2
	}
	return options
}

func int64Number(value any) int64 {
	switch typed := value.(type) {
	case int64:
		return typed
	case int:
		return int64(typed)
	case float64:
		return int64(typed)
	default:
		return 0
	}
}

func nsToUS(value int64) int64 {
	if value <= 0 {
		return 0
	}
	return int64(math.Ceil(float64(value) / 1000))
}

func percentileNearestRank(sortedValues []int64, percentile int) int64 {
	if len(sortedValues) == 0 {
		return 0
	}
	rank := int(math.Ceil(float64(percentile) / 100 * float64(len(sortedValues))))
	if rank < 1 {
		rank = 1
	}
	if rank > len(sortedValues) {
		rank = len(sortedValues)
	}
	return sortedValues[rank-1]
}
