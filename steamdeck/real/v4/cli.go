package main

import (
	"errors"
	"flag"
	"fmt"
	"io"
)

const usage = `usage: go run . <command> [flags]   (from steamdeck/real/v4)

Commands:
  realtime-core
         Build, release, deploy, configure, verify, restart and roll back a
         cell's native rt-core (deployment tiers config, olp and full)
  olp    Install, roll back or inspect a cell's relay OLP server (deployment tier 2)
  camera Install or inspect a cell's camera service on its relay (cell-camera/v1)
  pendant
         Build, install, roll back or inspect the Steam Deck v5 pendant (tier 4)
  help   Show this help

Every command takes --manifest rt-core/config/cells/<cell>.json and has
--dry-run; <command> --help lists its subcommands and flags.
docs/DEPLOY_AND_TEST_WORKFLOW.md is the guide.

Examples:
  go run . realtime-core release --manifest rt-core/config/cells/<cell>.json --commit <sha> --plan
  go run . realtime-core config --manifest rt-core/config/cells/<cell>.json --dry-run
  go run . olp status --manifest rt-core/config/cells/<cell>.json
  go run . pendant status --manifest rt-core/config/cells/<cell>.json
`

const (
	robotV4MotionServerNode = "motion-server"
	robotV4RealtimeCoreNode = "realtime-core"
)

// Run is the testable Robot v4 command entrypoint.
func Run(args []string, stdout, stderr io.Writer) int {
	if len(args) > 0 && args[0] == "v4" {
		args = args[1:]
	}
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, usage)
		return 0
	}

	command := args[0]
	rest := args[1:]
	var err error

	switch command {
	case "realtime-core":
		err = runRealtimeCore(rest, stdout, stderr)
	case "olp":
		err = runOLP(rest, stdout, stderr)
	case "camera":
		err = runCamera(rest, stdout, stderr)
	case "pendant":
		err = runPendant(rest, stdout, stderr)
	default:
		fmt.Fprintf(stderr, "unknown command %q\n\n%s", command, usage)
		return 2
	}

	if errors.Is(err, flag.ErrHelp) {
		return 0
	}
	if err != nil {
		var loggedError *robotV4DeployLoggedError
		if !errors.As(err, &loggedError) {
			fmt.Fprintf(stderr, "robot-v4 %s failed: %v\n", command, err)
		}
		var coded interface{ RealtimeCoreExitCode() int }
		if errors.As(err, &coded) {
			return coded.RealtimeCoreExitCode()
		}
		return 1
	}
	return 0
}

type robotV4ProcessIntentProcess struct {
	Name             string                        `json:"name"`
	Path             string                        `json:"path"`
	SHA256           string                        `json:"sha256"`
	DesiredState     string                        `json:"desired_state"`
	Args             []string                      `json:"args"`
	Env              []string                      `json:"env"`
	CWD              string                        `json:"cwd"`
	StartupGraceSecs int                           `json:"startup_grace_secs"`
	Supervisor       *robotV4ProcessSupervisorSpec `json:"supervisor,omitempty"`
}

type robotV4ProcessSupervisorSpec struct {
	Kind               string   `json:"kind"`
	Unit               string   `json:"unit"`
	Scope              string   `json:"scope"`
	Sudo               bool     `json:"sudo"`
	ExpectedProperties []string `json:"expected_properties,omitempty"`
}

func isRobotV4DaemonV1RefChar(ch byte) bool {
	return ch == '-' || ch == '_' || ch == '.' || ch == '/' ||
		ch >= '0' && ch <= '9' ||
		ch >= 'A' && ch <= 'Z' ||
		ch >= 'a' && ch <= 'z'
}
