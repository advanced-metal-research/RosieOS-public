package safety

import (
	"fmt"
	"math"
	"strings"
)

const (
	DefaultJoystickDeadband       = 0.02
	DefaultMaxTargetDeltaCounts   = 0
	DefaultMaxPositionDeltaCounts = 64
)

type NeutralWindowOptions struct {
	JoystickDeadband       float64
	MaxTargetDeltaCounts   int64
	MaxPositionDeltaCounts int64
	MinSamples             int
}

type NeutralSample struct {
	Source                   string
	JoystickAxes             []float64
	ActiveJog                bool
	AxisTargetCounts         []int64
	AxisPositionCounts       []int64
	AxisErrorMask            int64
	AxisFaultMask            int64
	UnauthorizedCommandCount int
	StuckOpenMotionCommand   bool
}

type NeutralWindowEvidence struct {
	Passed                   bool
	Events                   []string
	TelemetryFields          map[string]any
	LogMarkers               []string
	FailureReasons           []string
	MaxJoystickAbsAxis       float64
	MaxTargetDeltaCounts     int64
	MaxPositionDeltaCounts   int64
	UnauthorizedCommandCount int
	StuckOpenMotionCommand   bool
}

func ValidateNeutralWindow(samples []NeutralSample, options NeutralWindowOptions) NeutralWindowEvidence {
	options = normalizeNeutralWindowOptions(options)
	evidence := NeutralWindowEvidence{
		Passed:     true,
		Events:     []string{"neutral_window_start"},
		LogMarkers: []string{"neutral_window_start", "stuck-open-check", "no-target-walk"},
		TelemetryFields: map[string]any{
			"neutral_sample_count":        len(samples),
			"joystick_deadband":           options.JoystickDeadband,
			"target_delta_limit_counts":   options.MaxTargetDeltaCounts,
			"position_delta_limit_counts": options.MaxPositionDeltaCounts,
		},
	}
	if len(samples) < options.MinSamples {
		evidence.fail("neutral_sample_count=%d below required %d", len(samples), options.MinSamples)
	}
	if len(samples) == 0 {
		return evidence.finish()
	}

	first := samples[0]
	last := samples[len(samples)-1]
	for index, sample := range samples {
		maxAxis := maxAbsFloat64(sample.JoystickAxes)
		if maxAxis > evidence.MaxJoystickAbsAxis {
			evidence.MaxJoystickAbsAxis = maxAxis
		}
		if maxAxis > options.JoystickDeadband {
			evidence.fail("sample %d joystick_max_abs_axis %.5f exceeds deadband %.5f", index, maxAxis, options.JoystickDeadband)
		}
		if sample.ActiveJog {
			evidence.fail("sample %d active_jog=true", index)
		}
		if sample.AxisErrorMask != 0 || sample.AxisFaultMask != 0 {
			evidence.fail("sample %d axis masks error=0x%x fault=0x%x", index, sample.AxisErrorMask, sample.AxisFaultMask)
		}
		if sample.UnauthorizedCommandCount > 0 {
			evidence.UnauthorizedCommandCount += sample.UnauthorizedCommandCount
			evidence.fail("sample %d unauthorized_command_count=%d", index, sample.UnauthorizedCommandCount)
		}
		if sample.StuckOpenMotionCommand {
			evidence.StuckOpenMotionCommand = true
			evidence.fail("sample %d stuck_open_motion_command=true", index)
		}
	}

	evidence.MaxTargetDeltaCounts = maxAbsDeltaCounts(first.AxisTargetCounts, last.AxisTargetCounts)
	evidence.MaxPositionDeltaCounts = maxAbsDeltaCounts(first.AxisPositionCounts, last.AxisPositionCounts)
	if evidence.MaxTargetDeltaCounts > options.MaxTargetDeltaCounts {
		evidence.fail("axis_target_delta_counts=%d exceeds limit %d", evidence.MaxTargetDeltaCounts, options.MaxTargetDeltaCounts)
	}
	if evidence.MaxPositionDeltaCounts > options.MaxPositionDeltaCounts {
		evidence.fail("axis_position_delta_counts=%d exceeds limit %d", evidence.MaxPositionDeltaCounts, options.MaxPositionDeltaCounts)
	}
	return evidence.finish()
}

func (e *NeutralWindowEvidence) fail(format string, args ...any) {
	e.Passed = false
	e.FailureReasons = append(e.FailureReasons, fmt.Sprintf(format, args...))
}

func (e NeutralWindowEvidence) finish() NeutralWindowEvidence {
	if e.Passed {
		e.Events = append(e.Events, "neutral_window_pass")
		e.LogMarkers = append(e.LogMarkers, "neutral_window_result=pass")
	} else {
		e.Events = append(e.Events, "neutral_window_fail", "unexpected_motion", "unexpected_target_walk")
		e.LogMarkers = append(e.LogMarkers, "neutral_window_result=fail")
	}
	e.TelemetryFields["joystick_max_abs_axis"] = e.MaxJoystickAbsAxis
	e.TelemetryFields["active_jog"] = !e.Passed && strings.Contains(strings.Join(e.FailureReasons, "|"), "active_jog=true")
	e.TelemetryFields["axis_target_delta_counts"] = e.MaxTargetDeltaCounts
	e.TelemetryFields["axis_position_delta_counts"] = e.MaxPositionDeltaCounts
	e.TelemetryFields["unauthorized_command_count"] = e.UnauthorizedCommandCount
	e.TelemetryFields["stuck_open_motion_command"] = e.StuckOpenMotionCommand
	e.TelemetryFields["failure_reasons"] = e.FailureReasons
	return e
}

func normalizeNeutralWindowOptions(options NeutralWindowOptions) NeutralWindowOptions {
	if options.JoystickDeadband <= 0 {
		options.JoystickDeadband = DefaultJoystickDeadband
	}
	if options.MaxPositionDeltaCounts <= 0 {
		options.MaxPositionDeltaCounts = DefaultMaxPositionDeltaCounts
	}
	if options.MinSamples <= 0 {
		options.MinSamples = 2
	}
	return options
}

func maxAbsFloat64(values []float64) float64 {
	var maxValue float64
	for _, value := range values {
		if abs := math.Abs(value); abs > maxValue {
			maxValue = abs
		}
	}
	return maxValue
}

func maxAbsDeltaCounts(first, last []int64) int64 {
	limit := len(first)
	if len(last) < limit {
		limit = len(last)
	}
	var maxValue int64
	for index := 0; index < limit; index++ {
		delta := last[index] - first[index]
		if delta < 0 {
			delta = -delta
		}
		if delta > maxValue {
			maxValue = delta
		}
	}
	if len(first) != len(last) && maxValue == 0 {
		return math.MaxInt64
	}
	return maxValue
}
