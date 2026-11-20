package rtctl

import (
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
)

// RunCLI dispatches compilation, deployment and explicit runtime launch. Validate
// always appends --validate-config so validation cannot start a runtime session.
func RunCLI(args []string, root string, stdout, stderr io.Writer) int {
	if len(args) > 0 && args[0] == "recover-encoder" {
		return RunRecoverEncoderCLI(args[1:], stdout, stderr)
	}
	if len(args) > 0 && args[0] == "drive-mode-transition" {
		return RunDriveModeTransitionCLI(args[1:], stdout, stderr)
	}
	if len(args) > 0 && args[0] == "run" {
		return runLauncher(args[1:], root, stderr)
	}
	if len(args) > 0 && args[0] == "control-env" {
		return runControlEnv(args[1:], root, stdout, stderr)
	}
	if len(args) == 0 || args[0] != "compile" && args[0] != "validate" {
		fmt.Fprintln(stderr, "usage: rtctl compile --config FILE [--profiles DIR] --out DIR | validate --config FILE --backend simulation|ipc-only|live [--binary FILE]")
		return 2
	}
	flags := flag.NewFlagSet(args[0], flag.ContinueOnError)
	flags.SetOutput(stderr)
	config := flags.String("config", "", "machine JSON file")
	profiles := flags.String("profiles", filepath.Join(root, "config/drives"), "drive config directory")
	out := flags.String("out", "", "compiled output directory")
	var backend, binary string
	if args[0] == "validate" {
		flags.StringVar(&backend, "backend", "", "simulation, ipc-only or live")
		flags.StringVar(&binary, "binary", "", "daemon executable")
	}
	if err := flags.Parse(args[1:]); err != nil {
		return 2
	}
	if flags.NArg() != 0 || *config == "" || args[0] == "compile" && *out == "" {
		fmt.Fprintln(stderr, "--config and compile --out are required; positional arguments are not accepted")
		return 2
	}
	if args[0] == "validate" {
		var suffix string
		switch backend {
		case "simulation":
			suffix = "-sim"
		case "ipc-only":
			suffix = "-ipc"
		case "live":
		default:
			fmt.Fprintln(stderr, "--backend must be simulation, ipc-only or live")
			return 2
		}
		if binary == "" {
			binary = filepath.Join(root, "build", "rosie-rt-core"+suffix)
		}
		if *out == "" {
			*out = filepath.Join(root, "build", "config")
		}
	}
	data, err := os.ReadFile(*config)
	var compiled Compiled
	if err == nil {
		compiled, err = Compile(data, *profiles)
	}
	var directory string
	var argv []string
	if err == nil {
		for _, warning := range compiled.Warnings {
			fmt.Fprintln(stderr, "warning:", warning)
		}
		if asObject(decode(data)).optional("max_motor_rpm", nil) != nil {
			fmt.Fprintln(stderr, "warning: max_motor_rpm is accepted for one release with legacy wire-frame semantics; migrate to drive_speed_limit_motor_rpm in motor rpm (default 3000)")
		}
		if compiled.Robot == nil {
			fmt.Fprintln(stderr, "warning: legacy flat axes are accepted for one release; migrate to robot.definition (rosie.robot-definition.v1) and robot_joint")
		}
		directory, argv, err = writeCompiled(compiled, *out)
	}
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 1
	}
	if args[0] == "compile" {
		fmt.Fprintln(stdout, directory)
		return 0
	}
	cmd := exec.Command(binary, append(argv, "--validate-config")...)
	cmd.Stdout, cmd.Stderr = stdout, stderr
	if err := cmd.Run(); err != nil {
		var exited *exec.ExitError
		if errors.As(err, &exited) && exited.ExitCode() >= 0 {
			fmt.Fprintf(stderr, "daemon exit status: %d\n", exited.ExitCode())
			return exited.ExitCode()
		}
		fmt.Fprintln(stderr, err)
		return 1
	}
	fmt.Fprintln(stdout, "daemon exit status: 0")
	return 0
}

// DefaultRoot finds config/drives beside the build directory or above the
// working directory, preventing unrelated launch directories from selecting a
// different drive config tree.
func DefaultRoot() (string, error) {
	exe, err := os.Executable()
	if err == nil {
		if resolved, e := filepath.EvalSymlinks(exe); e == nil {
			exe = resolved
		}
		root := filepath.Dir(filepath.Dir(exe))
		if info, e := os.Stat(filepath.Join(root, "config/drives")); e == nil && info.IsDir() {
			return root, nil
		}
	}
	dir, err := os.Getwd()
	if err != nil {
		return "", err
	}
	for {
		if info, e := os.Stat(filepath.Join(dir, "config/drives")); e == nil && info.IsDir() {
			if _, e := os.Stat(filepath.Join(dir, "go.mod")); e == nil {
				return dir, nil
			}
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			break
		}
		dir = parent
	}
	return "", fmt.Errorf("cannot locate rt-core root for default drive configs and binaries")
}
