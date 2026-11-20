//go:build linux

package motionserver_test

import (
	"context"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// Rejected startup leaves no ingest listener receipt or plan store, preventing
// an obsolete selection from partially starting a second execution owner.
func TestRetiredBackendBeforeResources(t *testing.T) {
	for _, selection := range []string{"legacy_v4", "missing_binding"} {
		t.Run(selection, func(t *testing.T) {
			dir := t.TempDir()
			store, ready := filepath.Join(dir, "plans"), filepath.Join(dir, "ready")
			args := []string{"--plan-store-dir", store, "--ready-file", ready}
			reason := "reason=native_binding_required"
			if selection == "legacy_v4" {
				args = append(args, "--backend", selection)
				reason = "reason=backend_retired"
			}
			ctx, cancel := context.WithTimeout(context.Background(), observation)
			defer cancel()
			cmd := exec.CommandContext(ctx, filepath.Join(composedBuild, "bin/joint_trajectory_daemon"), args...)
			out, err := cmd.CombinedOutput()
			if err == nil || cmd.ProcessState.ExitCode() != 2 || !strings.Contains(string(out), reason) {
				t.Fatalf("startup: %v, %s", err, out)
			}
			for _, path := range []string{store, ready} {
				if _, err := os.Stat(path); !os.IsNotExist(err) {
					t.Fatalf("resource created before refusal: %s (%v)", path, err)
				}
			}
		})
	}
}
