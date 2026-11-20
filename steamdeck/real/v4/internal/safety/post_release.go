package safety

import "fmt"

const (
	DefaultPostReleaseMaxMotionAbs = 0
)

type PostReleaseOptions struct {
	MaxMotionCommandAbs     float64
	MaxTargetDeltaCounts    int64
	MaxPositionDeltaCounts  int64
	MinPostReleaseSamples   int
	RequireActiveBeforeStop bool
}

type PostReleaseSample struct {
	Source             string
	Phase              string
	MotionCommandAbs   float64
	AxisTargetCounts   []int64
	AxisPositionCounts []int64
	ReleaseCommand     bool
	ActiveBeforeStop   bool
}

type PostReleaseEvidence struct {
	Passed                 bool
	Events                 []string
	TelemetryFields        map[string]any
	LogMarkers             []string
	FailureReasons         []string
	MaxPostReleaseMotion   float64
	MaxTargetDeltaCounts   int64
	MaxPositionDeltaCounts int64
	ReleaseCommandSeen     bool
	ActiveBeforeStopSeen   bool
	StuckOpenMotionCommand bool
	PostReleaseSampleCount int
}

func ValidatePostReleaseStop(samples []PostReleaseSample, options PostReleaseOptions) PostReleaseEvidence {
	options = normalizePostReleaseOptions(options)
	evidence := PostReleaseEvidence{
		Passed:     true,
		Events:     []string{"post_release_start"},
		LogMarkers: []string{"post-release-start", "release-command-check", "stuck-open-check", "post-release-no-drift"},
		TelemetryFields: map[string]any{
			"post_release_motion_limit_abs":      options.MaxMotionCommandAbs,
			"post_release_target_delta_limit":    options.MaxTargetDeltaCounts,
			"post_release_position_delta_limit":  options.MaxPositionDeltaCounts,
			"post_release_required_sample_count": options.MinPostReleaseSamples,
		},
	}

	var firstPost *PostReleaseSample
	var lastPost *PostReleaseSample
	for index := range samples {
		sample := samples[index]
		if sample.ActiveBeforeStop {
			evidence.ActiveBeforeStopSeen = true
		}
		if sample.ReleaseCommand {
			evidence.ReleaseCommandSeen = true
		}
		if sample.Phase != "post-release" {
			continue
		}
		evidence.PostReleaseSampleCount++
		if firstPost == nil {
			firstPost = &samples[index]
		}
		lastPost = &samples[index]
		if sample.MotionCommandAbs > evidence.MaxPostReleaseMotion {
			evidence.MaxPostReleaseMotion = sample.MotionCommandAbs
		}
		if sample.MotionCommandAbs > options.MaxMotionCommandAbs {
			evidence.StuckOpenMotionCommand = true
			evidence.fail("sample %d post-release motion %.6f exceeds limit %.6f", index, sample.MotionCommandAbs, options.MaxMotionCommandAbs)
		}
	}

	if options.RequireActiveBeforeStop && !evidence.ActiveBeforeStopSeen {
		evidence.fail("active_before_stop=false")
	}
	if !evidence.ReleaseCommandSeen {
		evidence.fail("release_command_seen=false")
	}
	if evidence.PostReleaseSampleCount < options.MinPostReleaseSamples {
		evidence.fail("post_release_sample_count=%d below required %d", evidence.PostReleaseSampleCount, options.MinPostReleaseSamples)
	}
	if firstPost != nil && lastPost != nil {
		evidence.MaxTargetDeltaCounts = maxAbsDeltaCounts(firstPost.AxisTargetCounts, lastPost.AxisTargetCounts)
		evidence.MaxPositionDeltaCounts = maxAbsDeltaCounts(firstPost.AxisPositionCounts, lastPost.AxisPositionCounts)
		if evidence.MaxTargetDeltaCounts > options.MaxTargetDeltaCounts {
			evidence.fail("post_release_target_delta_counts=%d exceeds limit %d", evidence.MaxTargetDeltaCounts, options.MaxTargetDeltaCounts)
		}
		if evidence.MaxPositionDeltaCounts > options.MaxPositionDeltaCounts {
			evidence.fail("post_release_position_delta_counts=%d exceeds limit %d", evidence.MaxPositionDeltaCounts, options.MaxPositionDeltaCounts)
		}
	}
	return evidence.finish()
}

func (e *PostReleaseEvidence) fail(format string, args ...any) {
	e.Passed = false
	e.FailureReasons = append(e.FailureReasons, fmt.Sprintf(format, args...))
}

func (e PostReleaseEvidence) finish() PostReleaseEvidence {
	if e.Passed {
		e.Events = append(e.Events, "release_command_sent", "release_zero_hold_applied", "post_release_no_drift", "post_release_pass")
		e.LogMarkers = append(e.LogMarkers, "post_release_result=pass")
	} else {
		e.Events = append(e.Events, "post_release_fail", "stuck_open_motion_command")
		e.LogMarkers = append(e.LogMarkers, "post_release_result=fail")
	}
	e.TelemetryFields["release_command_seen"] = e.ReleaseCommandSeen
	e.TelemetryFields["active_before_stop"] = e.ActiveBeforeStopSeen
	e.TelemetryFields["post_release_sample_count"] = e.PostReleaseSampleCount
	e.TelemetryFields["post_release_max_motion_command_abs"] = e.MaxPostReleaseMotion
	e.TelemetryFields["post_release_target_delta_counts"] = e.MaxTargetDeltaCounts
	e.TelemetryFields["post_release_position_delta_counts"] = e.MaxPositionDeltaCounts
	e.TelemetryFields["stuck_open_motion_command"] = e.StuckOpenMotionCommand
	e.TelemetryFields["failure_reasons"] = e.FailureReasons
	return e
}

func normalizePostReleaseOptions(options PostReleaseOptions) PostReleaseOptions {
	if options.MaxMotionCommandAbs < 0 {
		options.MaxMotionCommandAbs = DefaultPostReleaseMaxMotionAbs
	}
	if options.MaxPositionDeltaCounts <= 0 {
		options.MaxPositionDeltaCounts = DefaultMaxPositionDeltaCounts
	}
	if options.MinPostReleaseSamples <= 0 {
		options.MinPostReleaseSamples = 1
	}
	return options
}
