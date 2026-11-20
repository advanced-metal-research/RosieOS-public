package safety

import "fmt"

type NoMotionOutputOptions struct {
	MaxAcceptedNonzeroCommands int
	AllowBoundedZeroHold       bool
	RequireStaleInputBlocked   bool
}

type MotionOutputObservation struct {
	Source                 string
	CommandKind            string
	AcceptedNonzeroCommand bool
	UnexpectedNonzeroAxes  int
	ActiveMotionCommand    bool
	BoundedZeroHold        bool
	StaleInputGenerated    bool
	MotionOutputBlocked    bool
}

type NoMotionOutputEvidence struct {
	Passed                      bool
	Events                      []string
	TelemetryFields             map[string]any
	LogMarkers                  []string
	FailureReasons              []string
	NoMotionOutputPass          bool
	AcceptedNonzeroCommandCount int
	UnexpectedNonzeroAxesCount  int
	ActiveMotionCommandCount    int
	BoundedZeroHoldOnly         bool
	StaleInputNoNewMotion       bool
	MotionOutputBlocked         bool
}

func ValidateNoMotionOutput(observations []MotionOutputObservation, options NoMotionOutputOptions) NoMotionOutputEvidence {
	evidence := NoMotionOutputEvidence{
		Passed:                true,
		Events:                []string{"no_motion_output_start"},
		LogMarkers:            []string{"no-motion-output", "accepted-nonzero-command-count", "bounded-zero-hold-only", "stale-input-no-new-motion"},
		BoundedZeroHoldOnly:   true,
		StaleInputNoNewMotion: true,
		MotionOutputBlocked:   true,
		TelemetryFields: map[string]any{
			"motion_output_observation_count": len(observations),
			"accepted_nonzero_limit":          options.MaxAcceptedNonzeroCommands,
		},
	}
	for index, observation := range observations {
		if observation.AcceptedNonzeroCommand {
			evidence.AcceptedNonzeroCommandCount++
		}
		if observation.UnexpectedNonzeroAxes > 0 {
			evidence.UnexpectedNonzeroAxesCount += observation.UnexpectedNonzeroAxes
			evidence.fail("sample %d unexpected_nonzero_axes=%d", index, observation.UnexpectedNonzeroAxes)
		}
		if observation.ActiveMotionCommand {
			evidence.ActiveMotionCommandCount++
			evidence.fail("sample %d active_motion_command=true", index)
		}
		if observation.BoundedZeroHold {
			continue
		}
		if observation.MotionOutputBlocked {
			continue
		}
		evidence.BoundedZeroHoldOnly = false
		evidence.MotionOutputBlocked = false
		evidence.fail("sample %d emitted unbounded non-motion output command=%s", index, observation.CommandKind)
	}
	if evidence.AcceptedNonzeroCommandCount > options.MaxAcceptedNonzeroCommands {
		evidence.fail("accepted_nonzero_command_count=%d exceeds limit %d", evidence.AcceptedNonzeroCommandCount, options.MaxAcceptedNonzeroCommands)
	}
	for index, observation := range observations {
		if observation.StaleInputGenerated {
			evidence.StaleInputNoNewMotion = false
			if options.RequireStaleInputBlocked {
				evidence.fail("sample %d stale_input_generated=true", index)
			}
		}
	}
	return evidence.finish()
}

func (e *NoMotionOutputEvidence) fail(format string, args ...any) {
	e.Passed = false
	e.FailureReasons = append(e.FailureReasons, fmt.Sprintf(format, args...))
}

func (e NoMotionOutputEvidence) finish() NoMotionOutputEvidence {
	e.NoMotionOutputPass = e.Passed
	if e.Passed {
		e.Events = append(e.Events, "no_motion_output_pass")
		e.LogMarkers = append(e.LogMarkers, "no_motion_output_result=pass")
	} else {
		e.Events = append(e.Events, "no_motion_output_fail")
		if e.ActiveMotionCommandCount > 0 || e.UnexpectedNonzeroAxesCount > 0 {
			e.Events = append(e.Events, "motion_output_blocked")
		}
		e.LogMarkers = append(e.LogMarkers, "no_motion_output_result=fail")
	}
	e.TelemetryFields["no_motion_output_pass"] = e.NoMotionOutputPass
	e.TelemetryFields["accepted_nonzero_command_count"] = e.AcceptedNonzeroCommandCount
	e.TelemetryFields["unexpected_nonzero_axes_count"] = e.UnexpectedNonzeroAxesCount
	e.TelemetryFields["active_motion_command_count"] = e.ActiveMotionCommandCount
	e.TelemetryFields["bounded_zero_hold_only"] = e.BoundedZeroHoldOnly
	e.TelemetryFields["stale_input_no_new_motion"] = e.StaleInputNoNewMotion
	e.TelemetryFields["motion_output_blocked"] = e.MotionOutputBlocked
	e.TelemetryFields["failure_reasons"] = e.FailureReasons
	return e
}
