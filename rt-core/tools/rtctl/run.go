package rtctl

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"syscall"
)

// runLauncher compiles and replaces this process with the selected daemon.
// Explicit live admission prevents a simulation configuration from reaching hardware.
func runLauncher(args []string, root string, stderr io.Writer) int {
	flags := flag.NewFlagSet("run", flag.ContinueOnError)
	flags.SetOutput(stderr)
	config := flags.String("config", "", "machine JSON file")
	out := flags.String("out", filepath.Join(root, "build", "config"), "compiled output directory")
	backend := flags.String("backend", "", "simulation, ipc-only or live")
	socket := flags.String("socket", "/run/rosie-rt-core/ipc.sock", "native socket path")
	if err := flags.Parse(args); err != nil {
		return 2
	}
	if flags.NArg() != 0 || *config == "" || *socket == "" {
		fmt.Fprintln(stderr, "run requires --config and a nonempty --socket; positional arguments are not accepted")
		return 2
	}
	var suffix string
	switch *backend {
	case "simulation":
		suffix = "-sim"
	case "ipc-only":
		suffix = "-ipc"
	case "live":
	default:
		fmt.Fprintln(stderr, "--backend must be simulation, ipc-only or live")
		return 2
	}
	binary := filepath.Join(root, "build", "rosie-rt-core"+suffix)
	// Installed host launchers require explicit live admission. Never resolve
	// a simulation request to a package's unsuffixed hardware executable.
	if _, statErr := os.Stat(filepath.Join(root, "bin", "rtctl")); statErr == nil {
		if *backend != "live" {
			fmt.Fprintln(stderr, "packaged run requires the explicitly live host backend")
			return 2
		}
		binary = filepath.Join(root, "bin", "rosie-rt-core")
	}
	data, err := os.ReadFile(*config)
	if err == nil && *backend == "live" {
		var machine map[string]any
		err = json.Unmarshal(data, &machine)
		if err == nil && machine["backend"] != "live" {
			err = fmt.Errorf("live execution requires an explicitly live machine configuration")
		}
	}
	var compiled Compiled
	if err == nil {
		compiled, err = Compile(data, filepath.Join(root, "config/drives"))
	}
	var argv []string
	if err == nil {
		for _, warning := range compiled.Warnings {
			fmt.Fprintln(stderr, "warning:", warning)
		}
		_, argv, err = writeCompiled(compiled, *out)
	}
	if err == nil {
		argv = append([]string{binary}, argv...)
		argv = append(argv, "--socket-path", *socket)
		err = syscall.Exec(binary, argv, os.Environ())
	}
	fmt.Fprintln(stderr, err)
	return 1
}
