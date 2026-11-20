package safety

import (
	"fmt"
	"strings"
)

type CommandAuthorityOptions struct {
	AllowedCommands         []string
	ForbiddenCommands       []string
	LiveMotionUnlockPresent bool
	RequireLiveMotionUnlock bool
}

type CommandObservation struct {
	Source  string
	Command string
}

type CommandAuthorityEvidence struct {
	Passed                   bool
	Events                   []string
	TelemetryFields          map[string]any
	LogMarkers               []string
	FailureReasons           []string
	AuthorizedCommandNames   []string
	ForbiddenCommandNames    []string
	ForbiddenCommandCount    int
	UnexpectedCommandCount   int
	ForbiddenCommandDetected bool
	CommandAuthorityPass     bool
}

func ValidateCommandAuthority(observations []CommandObservation, options CommandAuthorityOptions) CommandAuthorityEvidence {
	options = normalizeCommandAuthorityOptions(options)
	evidence := CommandAuthorityEvidence{
		Passed:                 true,
		Events:                 []string{"command_authority_start"},
		LogMarkers:             []string{"command-authority-start", "forbidden-command-check", "live-motion-unlock-check"},
		AuthorizedCommandNames: options.AllowedCommands,
		TelemetryFields: map[string]any{
			"authorized_command_names":   options.AllowedCommands,
			"live_motion_unlock_present": options.LiveMotionUnlockPresent,
			"observed_command_count":     len(observations),
		},
	}
	if options.RequireLiveMotionUnlock && !options.LiveMotionUnlockPresent {
		evidence.fail("live_motion_unlock_present=false")
	}
	allowed := commandSet(options.AllowedCommands)
	forbidden := commandSet(options.ForbiddenCommands)
	for index, observation := range observations {
		command := normalizeCommandName(observation.Command)
		if command == "" {
			continue
		}
		if forbidden[command] {
			evidence.ForbiddenCommandDetected = true
			evidence.ForbiddenCommandCount++
			evidence.ForbiddenCommandNames = appendUnique(evidence.ForbiddenCommandNames, command)
			evidence.fail("sample %d forbidden_command=%s", index, command)
			continue
		}
		if !allowed[command] {
			evidence.UnexpectedCommandCount++
			evidence.fail("sample %d unexpected_command=%s", index, command)
		}
	}
	return evidence.finish()
}

func (e *CommandAuthorityEvidence) fail(format string, args ...any) {
	e.Passed = false
	e.FailureReasons = append(e.FailureReasons, fmt.Sprintf(format, args...))
}

func (e CommandAuthorityEvidence) finish() CommandAuthorityEvidence {
	e.CommandAuthorityPass = e.Passed
	if e.Passed {
		e.Events = append(e.Events, "command_authority_pass")
		e.LogMarkers = append(e.LogMarkers, "command_authority_result=pass")
	} else {
		e.Events = append(e.Events, "command_authority_fail")
		if e.ForbiddenCommandDetected {
			e.Events = append(e.Events, "forbidden_command_detected")
		}
		e.LogMarkers = append(e.LogMarkers, "command_authority_result=fail")
	}
	e.TelemetryFields["command_authority_pass"] = e.CommandAuthorityPass
	e.TelemetryFields["forbidden_command_detected"] = e.ForbiddenCommandDetected
	e.TelemetryFields["forbidden_command_count"] = e.ForbiddenCommandCount
	e.TelemetryFields["forbidden_command_names"] = e.ForbiddenCommandNames
	e.TelemetryFields["unexpected_command_count"] = e.UnexpectedCommandCount
	e.TelemetryFields["failure_reasons"] = e.FailureReasons
	return e
}

func normalizeCommandAuthorityOptions(options CommandAuthorityOptions) CommandAuthorityOptions {
	if len(options.ForbiddenCommands) == 0 {
		options.ForbiddenCommands = []string{"arm", "disarm", "home", "hm35", "go_home", "position", "quick_stop", "disable", "brake", "deactivation"}
	}
	normalizedAllowed := make([]string, 0, len(options.AllowedCommands))
	for _, command := range options.AllowedCommands {
		if normalized := normalizeCommandName(command); normalized != "" {
			normalizedAllowed = appendUnique(normalizedAllowed, normalized)
		}
	}
	options.AllowedCommands = normalizedAllowed
	normalizedForbidden := make([]string, 0, len(options.ForbiddenCommands))
	for _, command := range options.ForbiddenCommands {
		if normalized := normalizeCommandName(command); normalized != "" {
			normalizedForbidden = appendUnique(normalizedForbidden, normalized)
		}
	}
	options.ForbiddenCommands = normalizedForbidden
	return options
}

func commandSet(commands []string) map[string]bool {
	result := make(map[string]bool, len(commands))
	for _, command := range commands {
		result[normalizeCommandName(command)] = true
	}
	return result
}

func normalizeCommandName(command string) string {
	command = strings.TrimSpace(strings.ToLower(command))
	command = strings.ReplaceAll(command, "-", "_")
	command = strings.ReplaceAll(command, " ", "_")
	return command
}

func appendUnique(items []string, item string) []string {
	for _, existing := range items {
		if existing == item {
			return items
		}
	}
	return append(items, item)
}
