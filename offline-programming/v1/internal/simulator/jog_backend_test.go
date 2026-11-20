package simulator

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// Invalid selection must win over missing files before any child or socket exists.
func TestRetiredJogBackendRefusedBeforeResources(t *testing.T) {
	for _, backend := range []string{"legacy_v4", "unknown"} {
		t.Run(backend, func(t *testing.T) {
			t.Setenv("OFFLINE_PROGRAMMING_JOG_BACKEND", backend)
			root := t.TempDir()
			session, err := startJogSession(root, filepath.Join(root, "absent"), nil)
			var refusal *JogBackendError
			if session != nil || !errors.As(err, &refusal) || refusal.Backend != backend {
				t.Fatalf("selection: session=%v error=%v", session, err)
			}
			entries, err := os.ReadDir(root)
			if err != nil || len(entries) != 0 {
				t.Fatalf("resources created: %v %v", entries, err)
			}
		})
	}
}
