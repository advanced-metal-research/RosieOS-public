package main

import (
	"bytes"
	"os"
	"path/filepath"
	"testing"
)

// Schema regeneration replaces conflicts in every generated client; check mode
// must leave conflicts intact so a partial resolution cannot appear canonical.
func TestRegenerationReplacesConflictsInAllClients(t *testing.T) {
	outputs, err := generate(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	build := filepath.Join("..", "..", "build")
	if err := os.MkdirAll(build, 0755); err != nil {
		t.Fatal(err)
	}
	root, err := os.MkdirTemp(build, "api-regen-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(root) })
	paths := []string{
		"adapters/rosie/control/api_generated.go",
		"clients/cpp/include/rosie/rt_control_api_generated.hpp",
		"clients/ts/rt_control_api_generated.ts",
	}
	if len(outputs) != 3 {
		t.Fatalf("generated %d artifacts, want 3", len(outputs))
	}
	conflict := []byte("<<<<<<< HEAD\nintegration client\n=======\nlimit hold client\n>>>>>>> agent/limithold\n")
	for _, name := range paths {
		if len(outputs[name]) == 0 {
			t.Fatalf("schema did not generate %s", name)
		}
		path := filepath.Join(root, name)
		if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, conflict, 0644); err != nil {
			t.Fatal(err)
		}
	}
	if err := writeOutputs(root, outputs, true); err == nil {
		t.Fatal("conflicted clients passed check mode")
	}
	for _, name := range paths {
		got, err := os.ReadFile(filepath.Join(root, name))
		if err != nil || !bytes.Equal(got, conflict) {
			t.Fatalf("check changed conflict in %s: %v", name, err)
		}
	}
	if err := writeOutputs(root, outputs, false); err != nil {
		t.Fatal(err)
	}
	for _, name := range paths {
		got, err := os.ReadFile(filepath.Join(root, name))
		if err != nil || !bytes.Equal(got, outputs[name]) {
			t.Fatalf("regeneration did not resolve %s: %v", name, err)
		}
	}
	if err := writeOutputs(root, outputs, true); err != nil {
		t.Fatal(err)
	}
}
