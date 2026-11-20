//go:build linux

package buildcontract

import (
	"errors"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// The existing probe uses ordinary control flow to detect erased assert setup.
// Exercise the unprotected build as a negative control; compiling every test
// only with -UNDEBUG would never prove that this guard rejects vacuous tests.
func TestNativeAssertionProbeRejectsUnprotectedNDEBUG(t *testing.T) {
	root, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	source := filepath.Join(root, "tests", "assertions_enabled_tests.cpp")
	for _, test := range []struct {
		name      string
		protected bool
		output    string
	}{
		{"unprotected", false, "FAIL: assert erased its setup expression\n"},
		{"protected", true, "PASS: assertion setup executes and a false assertion aborts\n"},
	} {
		t.Run(test.name, func(t *testing.T) {
			binary := filepath.Join(t.TempDir(), "assertion-probe")
			args := []string{"-std=c++17", "-O2", "-DNDEBUG", source, "-o", binary}
			if test.protected {
				args = append(args, "-UNDEBUG")
			}
			compile := exec.Command("g++", args...)
			t.Logf("compile argv=%q", compile.Args)
			if output, err := compile.CombinedOutput(); err != nil {
				t.Fatalf("assertion probe setup failed: %v\n%s", err, output)
			}
			output, err := exec.Command(binary).CombinedOutput()
			exitCode := 0
			if err != nil {
				var exited *exec.ExitError
				if !errors.As(err, &exited) {
					t.Fatalf("could not execute assertion probe: %v", err)
				}
				exitCode = exited.ExitCode()
			}
			wantExit := 1
			if test.protected {
				wantExit = 0
			}
			if exitCode != wantExit || string(output) != test.output {
				t.Fatalf("probe did not establish %s behavior: exit=%d want=%d output=%q", test.name, exitCode, wantExit, output)
			}
			t.Logf("run argv=%q exit=%d result=%s", []string{binary}, exitCode, strings.TrimSpace(string(output)))
		})
	}
}
