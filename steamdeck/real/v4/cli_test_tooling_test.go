package main

import (
	"os"
	"path/filepath"

	"testing"
)

func TestFindProjectRootDoesNotMistakeRepositoryRootForRobotV4(t *testing.T) {
	repositoryRoot, err := repoRoot()
	if err != nil {
		t.Fatalf("resolve repository root: %v", err)
	}
	got, err := findProjectRoot(repositoryRoot)
	if err != nil {
		t.Fatalf("find Robot v4 project root from repository root: %v", err)
	}
	want := filepath.Join(repositoryRoot, "steamdeck", "real", "v4")
	if got != want {
		t.Fatalf("Robot v4 project root = %q, want %q", got, want)
	}
}

func TestRepoRootRecognizesExtractedRosieOSMarkers(t *testing.T) {
	root, err := repoRoot()
	if err != nil {
		t.Fatalf("resolve extracted RosieOS repository root: %v", err)
	}
	for _, marker := range []string{
		filepath.Join("steamdeck", "real", "v4", "go.mod"),
		filepath.Join("rt-core", "go.mod"),
	} {
		if _, err := os.Stat(filepath.Join(root, marker)); err != nil {
			t.Fatalf("extracted RosieOS root %s is missing marker %s: %v", root, marker, err)
		}
	}
}
