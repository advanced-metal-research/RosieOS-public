package main

import (
	"bytes"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestAbsenceScanRejectsEveryRetiredReference(t *testing.T) {
	for _, token := range []string{"python", "python3", "tool.py", "PYTHONPATH", "PYTHONOPTIMIZE=2", "PYTHON=tool"} {
		t.Run(token, func(t *testing.T) {
			root := t.TempDir()
			if err := os.WriteFile(filepath.Join(root, "source.go"), []byte("safe\n"+token+"\n"), 0o644); err != nil {
				t.Fatal(err)
			}
			var output bytes.Buffer
			if err := scanRetiredRuntime(root, []string{"source.go"}, &output); err == nil ||
				output.String() != "source.go:2:"+token+"\n" {
				t.Fatalf("missed forbidden token: %v %s", err, output.String())
			}
		})
	}
}

func TestAbsenceAllowlistBoundariesAndTrackedFiles(t *testing.T) {
	root := t.TempDir()
	files := []string{"docs/history/check.md", "tests/parity/proof.json", "tools/rtctl/testdata/parity/golden.json"}
	for _, file := range append(append([]string(nil), files...), "docs-other/check.md", "tests/parity/nested/proof.json", "tests/parity/probe.go", "untracked.txt") {
		name := filepath.Join(root, file)
		if err := os.MkdirAll(filepath.Dir(name), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(name, []byte("python3\n"), 0o644); err != nil {
			t.Fatal(err)
		}
	}
	var output bytes.Buffer
	if err := scanRetiredRuntime(root, files, &output); err != nil || output.String() != "retired-runtime absence: passed\n" {
		t.Fatalf("allowlisted or untracked content affected scan: %v %s", err, output.String())
	}
	for _, file := range []string{"docs-other/check.md", "tests/parity/nested/proof.json", "tests/parity/probe.go", "missing.go", "../escape"} {
		if err := scanRetiredRuntime(root, []string{file}, &output); err == nil {
			t.Fatalf("accepted %s", file)
		}
	}
	file := "empty.py"
	if err := os.WriteFile(filepath.Join(root, file), nil, 0o644); err != nil {
		t.Fatal(err)
	}
	if err := scanRetiredRuntime(root, []string{file}, &output); err == nil || !strings.Contains(output.String(), "retired source file") {
		t.Fatalf("accepted empty retired source: %v", err)
	}
}
