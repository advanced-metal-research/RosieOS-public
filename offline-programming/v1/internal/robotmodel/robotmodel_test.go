package robotmodel

import (
	"os"
	"path/filepath"
	"testing"
)

// TestClassReadsTheDescriptionDirectory keeps a model's class in its
// description: an arm has a robot.urdf, a bench only a manifest, and an id
// with no description, or not shaped like one, is unknown.
func TestClassReadsTheDescriptionDirectory(t *testing.T) {
	repo := t.TempDir()
	write := func(rel string) {
		t.Helper()
		path := filepath.Join(repo, "robot_description", "robots", rel)
		if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, []byte("{}"), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	write("example_arm/robot_description_manifest.json")
	write("example_arm/robot.urdf")
	write("example_bench/robot_description_manifest.json")
	write("no_manifest/robot.urdf")
	if err := os.WriteFile(filepath.Join(repo, "robot_description", "outside_manifest.json"), []byte("{}"), 0o644); err != nil {
		t.Fatal(err)
	}
	SetRoot(repo)
	t.Cleanup(func() { SetRoot(discover()) })

	for id, want := range map[string]string{
		"example_arm":   Arm,
		"example_bench": Bench,
		"no_manifest":   "",
		"missing":       "",
		"":              "",
		"../robots":     "",
		"Example_Arm":   "",
		"example-arm":   "",
		"example_arm_":  "",
	} {
		if got := Class(id); got != want {
			t.Errorf("Class(%q) = %q, want %q", id, got, want)
		}
	}
	if !IsArm("example_arm") || IsArm("example_bench") {
		t.Fatal("IsArm must hold for arms only")
	}
	SetRoot("")
	if Class("example_arm") != "" {
		t.Fatal("a tree with no root must know no model")
	}
}

// TestClassKnowsThePublicModels pins the public tree's own descriptions, found
// by walking up from this package.
func TestClassKnowsThePublicModels(t *testing.T) {
	SetRoot(discover())
	for id, want := range map[string]string{
		"rosie_1400_v3":          Arm,
		"rosie_1420_v1":          Arm,
		"bench_one_motor_1to1":   Bench,
		"bench_nine_motors_1to1": Bench,
		"linear_rail_1axis":      Bench,
	} {
		if got := Class(id); got != want {
			t.Errorf("Class(%q) = %q, want %q", id, got, want)
		}
	}
}
