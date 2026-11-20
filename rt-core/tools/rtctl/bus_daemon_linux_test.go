//go:build linux

package rtctl

import (
	"os/exec"
	"strings"
	"testing"
)

// TestBusDiagnosticCoherence injects a change between IgH observations, so an
// unused disappearance cannot publish a count mismatch to cyclic readiness.
func TestBusDiagnosticCoherence(t *testing.T) {
	t.Parallel()
	binary, out, err := nativeTestBinary(t, "bus-diagnostics-tests", func(binary string) []string {
		return []string{"-std=c++17", "-O0", "-UNDEBUG", "-Iinclude", "-I.", "tests/bus_diagnostics_tests.cpp", "-o", binary, "-pthread"}
	})
	if err != nil {
		t.Fatalf("compile diagnostics fixture: %v %s", err, out)
	}
	if out, err := exec.Command(binary).CombinedOutput(); err != nil || !strings.Contains(string(out), "bus_diagnostics_tests: PASS") {
		t.Fatalf("diagnostics fixture: %v %s", err, out)
	}
}

// TestBusDaemonIdentity requires native recompilation of the exact topology
// identity, preventing changed argv from weakening a compiled refuse policy.
func TestBusDaemonIdentity(t *testing.T) {
	f := fresh()
	f.put("machine.bus", `{"unknown_slaves":"hold","unknown_slaves_note":"unverified bench subset","unused_slaves":[{"slave_position":6,"vendor_id":4194304,"product_code":1813,"revision":12024,"note":"spare"}]}`)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	validatePublishedBringup(t, compiled)
	_, args, err := writeCompiled(compiled, buildTemp(t))
	if err != nil {
		t.Fatal(err)
	}
	for _, change := range [][]string{{"--bus-unknown-slaves", "refuse"}, {"--bus-unused-slaves", "7|4194304|1813|12024"}} {
		tampered := append([]string{}, args...)
		for i := 0; i+1 < len(tampered); i++ {
			if tampered[i] == change[0] {
				tampered[i+1] = change[1]
			}
		}
		out, err := runBringupDaemon(t, append(tampered, "--validate-config")...)
		if err == nil || !strings.Contains(out, "machine identity mismatch: bus_") {
			t.Fatalf("tampered bus accepted: %v %s", err, out)
		}
	}
	for _, value := range []string{"0|4194304|1813|-", "6|4194304|1813|-;6|4194304|1813|-", "65536|1|2|-", "6|0|2|-", "6|1|2|4294967296", "6|1|2|-;"} {
		tampered := append([]string{}, args...)
		for i := 0; i+1 < len(tampered); i++ {
			if tampered[i] == "--bus-unused-slaves" {
				tampered[i+1] = value
			}
		}
		if out, err := runBringupDaemon(t, append(tampered, "--validate-config")...); err == nil {
			t.Fatalf("accepted %s: %s", value, out)
		}
	}
}
