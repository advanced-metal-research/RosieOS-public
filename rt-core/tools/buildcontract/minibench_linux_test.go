//go:build linux

package buildcontract

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

// TestMiniBenchLiveBuildPin requires a matching supported package, preventing
// the new build parameter from admitting an arbitrary or mismatched IgH release.
func TestMiniBenchLiveBuildPin(t *testing.T) {
	root, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	// Measured source pins: RevPi 1.6.9 and Pi 1.6.12; other releases are unqualified.
	for _, tc := range []struct {
		selected, installed string
		pass                bool
	}{
		{"1.6.12", "1.6.12", true}, {"1.6.12", "1.6.9", false},
		{"1.6.9", "1.6.12", false}, {"1.6.11", "1.6.11", true}, {"1.6.10", "1.6.10", true}, {"1.6.13", "1.6.13", false},
	} {
		t.Run(tc.selected+"/"+tc.installed, func(t *testing.T) {
			dir := t.TempDir()
			pc := fmt.Sprintf("Name: libethercat\nDescription: unverified version fixture\nVersion: %s\nLibs: -lethercat\nCflags:\n", tc.installed)
			// Unverified fixture permissions: metadata is private to the test.
			if err := os.WriteFile(filepath.Join(dir, "libethercat.pc"), []byte(pc), 0600); err != nil {
				t.Fatal(err)
			}
			t.Setenv("PKG_CONFIG_PATH", dir)
			t.Setenv("PKG_CONFIG_LIBDIR", dir)
			t.Setenv("ROSIE_RT_FOUNDATION_PROFILE", "")
			cmd := exec.Command("make", "--no-print-directory", "-s", "check-ecrt", "ROSIE_RT_IGH_VERSION="+tc.selected)
			cmd.Dir = root
			out, err := cmd.CombinedOutput()
			if (err == nil) != tc.pass {
				t.Fatalf("pass=%t error=%v output=%s", tc.pass, err, out)
			}
		})
	}
}
