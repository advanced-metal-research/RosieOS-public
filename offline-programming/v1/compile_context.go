package main

// compile-context: the planning context for one model, written to a
// directory.
//
// The one compiler is `internal/robotconfig.Compile`
// (docs/PLANNING-CONTEXT.md). This makes it reachable from outside the
// server so the weld planner's fixture tools pack the same context the OLP
// packs, rather than carrying a second implementation of the merge in Python
// that would drift from the first -- the failure the context exists to end.

import (
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"sort"

	"rosieos/offline-programming/v1/internal/robotconfig"
)

func runCompileContext(args []string, stdout, stderr io.Writer) error {
	flags := flag.NewFlagSet("compile-context", flag.ContinueOnError)
	flags.SetOutput(stderr)
	model := flags.String("model", "", "the robot description to compile: a directory name under the root")
	machinePlanningCalibration := flags.String("machine-planning-calibration", "", "the cell's machine_planning_calibration.json; absent compiles the model's empty document")
	rootFlag := flags.String("root", "", "where the descriptions live (default <repo>/robot_description/robots)")
	out := flags.String("out", "", "directory to write the context's members into")
	if err := flags.Parse(args); err != nil {
		return err
	}
	if *model == "" || *out == "" {
		return errors.New("--model and --out are required")
	}
	root := *rootFlag
	if root == "" {
		project, err := projectRoot()
		if err != nil {
			return err
		}
		repo, err := repositoryRoot(project)
		if err != nil {
			return err
		}
		root = filepath.Join(repo, "robot_description", "robots")
	}
	description, err := robotconfig.At(root).Load(*model)
	if err != nil {
		return err
	}
	var cell []byte
	source := robotconfig.SourceEmpty
	if *machinePlanningCalibration != "" {
		// An uploaded or copied machine_planning_calibration.json is labelled as the target's: the
		// gate is the hash, not the label (docs/PLANNING-CONTEXT.md, Offline).
		cell, err = os.ReadFile(*machinePlanningCalibration)
		if err != nil {
			return err
		}
		source = robotconfig.SourceTarget
	}
	settings, err := robotconfig.Resolve(description, cell, source)
	if err != nil {
		return err
	}
	compiled, err := robotconfig.Compile(description, settings)
	if err != nil {
		return err
	}
	paths := make([]string, 0, len(compiled.Files))
	for path := range compiled.Files {
		paths = append(paths, path)
	}
	sort.Strings(paths)
	for _, path := range paths {
		target := filepath.Join(*out, filepath.FromSlash(path))
		if err := os.MkdirAll(filepath.Dir(target), 0o755); err != nil {
			return err
		}
		if err := os.WriteFile(target, compiled.Files[path], 0o644); err != nil {
			return err
		}
	}
	fmt.Fprintf(stdout, "compile-context: %s %s, cell %s (%s) -> %s, %d members\n",
		compiled.Identity.ModelID, compiled.Identity.RobotDescriptionSHA256,
		compiled.Identity.MachinePlanningCalibrationSHA256, compiled.Identity.MachinePlanningCalibrationSource, *out, len(paths))
	return nil
}
