package main

import (
	"bytes"
	"strings"
	"testing"
)

// TestCellCommandHelpListsFlags pins that `<command> --help` on every
// cell-driven command prints that command's flags and exits 0. Before, the
// retired-flag filter took --help for a retired v4 selection and answered
// reason=backend_retired, so the commands the docs name had no usable help.
func TestCellCommandHelpListsFlags(t *testing.T) {
	commands := [][]string{
		{"olp", "install"}, {"olp", "rollback"}, {"olp", "status"},
		{"olp", "ui", "start"}, {"olp", "ui", "stop"}, {"olp", "ui", "restart"}, {"olp", "ui", "status"}, {"olp", "ui", "logs"},
		{"pendant", "build"}, {"pendant", "install"}, {"pendant", "rollback"}, {"pendant", "status"},
		{"pendant", "tunnel", "install"}, {"pendant", "tunnel", "status"}, {"pendant", "tunnel", "logs"},
		{"pendant", "tunnel", "restart"}, {"pendant", "tunnel", "stop"}, {"pendant", "tunnel", "rollback"},
		{"realtime-core", "deploy"}, {"realtime-core", "start"}, {"realtime-core", "config"},
	}
	for _, c := range cellDeployRealtimeCoreCommands {
		commands = append(commands, []string{"realtime-core", c})
	}
	for _, args := range commands {
		for _, help := range []string{"--help", "-h"} {
			var stdout, stderr bytes.Buffer
			code := Run(append(append([]string{}, args...), help), &stdout, &stderr)
			out := stdout.String() + stderr.String()
			name := strings.Join(args, " ") + " " + help
			if code != 0 {
				t.Errorf("%s: exit %d, want 0:\n%s", name, code, out)
			}
			if strings.Contains(out, "backend_retired") || strings.Contains(out, "failed") {
				t.Errorf("%s: help reported a failure:\n%s", name, out)
			}
			if !strings.Contains(out, "-manifest") && !strings.Contains(out, "--manifest") {
				t.Errorf("%s: help lacks the --manifest flag:\n%s", name, out)
			}
		}
	}
}
