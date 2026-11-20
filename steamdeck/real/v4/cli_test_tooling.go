package main

import (
	"fmt"

	"os"

	"path/filepath"

	"runtime"

	"strings"
)

func shellQuote(value string) string {
	if runtime.GOOS == "windows" {
		return "'" + strings.ReplaceAll(value, "'", "''") + "'"
	}
	return "'" + strings.ReplaceAll(value, "'", "'\"'\"'") + "'"
}

func projectRoot() (string, error) {
	wd, err := os.Getwd()
	if err != nil {
		return "", err
	}
	return findProjectRoot(wd)
}

func repoRoot() (string, error) {
	robotRoot, err := projectRoot()
	if err != nil {
		return "", err
	}
	wd := robotRoot
	for {
		// A sparse worktree is a repository boundary even when other components
		// are absent; never resolve builds or test artifacts into its parent checkout.
		if pathExists(filepath.Join(wd, ".git")) && pathExists(filepath.Join(wd, "steamdeck", "real", "v4", "go.mod")) {
			return wd, nil
		}
		if pathExists(filepath.Join(wd, "rosie.sh")) ||
			(pathExists(filepath.Join(wd, "steamdeck", "real", "v4", "go.mod")) &&
				pathExists(filepath.Join(wd, "rt-core", "go.mod"))) {
			return wd, nil
		}
		parent := filepath.Dir(wd)
		if parent == wd {
			return "", fmt.Errorf("could not find repo root from %s", robotRoot)
		}
		wd = parent
	}
}

func findProjectRoot(start string) (string, error) {
	wd := start
	isRobotV4Root := func(candidate string) bool {
		if !pathExists(filepath.Join(candidate, "go.mod")) {
			return false
		}
		return pathExists(filepath.Join(candidate, "orchestrator", "artifact_manifest.schema.json"))
	}
	for {
		if isRobotV4Root(wd) {
			return wd, nil
		}
		repoRobotRoot := filepath.Join(wd, "steamdeck", "real", "v4")
		if isRobotV4Root(repoRobotRoot) {
			return repoRobotRoot, nil
		}
		parent := filepath.Dir(wd)
		if parent == wd {
			return "", fmt.Errorf("could not find steamdeck/real/v4 root from %s", start)
		}
		wd = parent
	}
}

func pathExists(path string) bool {
	_, err := os.Stat(path)
	return err == nil
}

func isHelp(arg string) bool {
	return arg == "help" || arg == "-h" || arg == "--help"
}

func main() {
	os.Exit(Run(os.Args[1:], os.Stdout, os.Stderr))
}
