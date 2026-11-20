//go:build linux && soak && !race

package main

import (
	"context"
	"path/filepath"
	"testing"
)

// TestSoakShortRunHasNoAccumulation exercises the real binaries without race
// instrumentation. Missing measurements fail instead of becoming skipped proof.
func TestSoakShortRunHasNoAccumulation(t *testing.T) {
	root, err := absoluteRoot()
	if err != nil {
		t.Fatal(err)
	}
	opts := options{Iterations: 10, Root: root, Out: filepath.Join(root, "build/soak-check.json")}
	r, runErr := runSoak(context.Background(), opts)
	if err := writeRecord(opts.Out, r); err != nil {
		t.Fatal(err)
	}
	t.Logf("iterations=%d host_loaded=%t record=%s", r.IterationsCompleted, r.HostLoaded, opts.Out)
	for _, a := range r.Assertions {
		t.Logf("%s: %s: %s", a.Name, a.Result, a.Detail)
	}
	if runErr != nil {
		t.Fatal(runErr)
	}
}
