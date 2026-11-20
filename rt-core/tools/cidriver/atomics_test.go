package main

import (
	"bytes"
	"context"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/ipcclient"
	"strings"
	"testing"
)

// TestAtomicInteropStatusSnapshotQualification checks the actual native processes and
// injected failures so CI cannot silently drop the status-sized publication check.
func TestAtomicInteropStatusSnapshotQualification(t *testing.T) {
	root, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll(filepath.Join(root, "build"), 0755); err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "atomicqual-test-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	binary := filepath.Join(dir, "shared-words-interop")
	build := exec.Command("g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic",
		"-pthread", "-I"+filepath.Join(root, "include"), filepath.Join(root, "tests/shared_words_interop.cpp"), "-o", binary)
	if output, err := build.CombinedOutput(); err != nil {
		t.Fatalf("compile qualification: %v\n%s", err, output)
	}
	var log bytes.Buffer
	if err := atomicInterop(context.Background(), binary, &log); err != nil {
		t.Fatalf("cross-process qualification: %v\n%s", err, log.String())
	}
	for _, field := range []string{"atomic_snapshot result=PASS", fmt.Sprintf("payload_bytes=%d", ipcclient.StatusSnapshotV1Bytes), "requested=50000",
		"publications=50000", "observed_publications=50000", "last_publication=50000", "torn_reads=0",
		"sequence_gaps=0", "regressions=0", "max_staleness_ns="} {
		if !strings.Contains(log.String(), field) {
			t.Fatalf("missing qualification field %q:\n%s", field, log.String())
		}
	}
	selfTest := exec.Command(binary, "qualify_snapshot_self_test")
	output, err := selfTest.CombinedOutput()
	if err != nil || !strings.Contains(string(output), "PASS: atomic_snapshot self-test") {
		t.Fatalf("qualification refusal checks: %v\n%s", err, output)
	}
}
