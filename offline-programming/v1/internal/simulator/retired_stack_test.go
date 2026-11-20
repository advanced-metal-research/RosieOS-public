package simulator

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// Refusal must precede even cache creation; absent dependencies cannot mask it.
func TestRetiredStackSelectionBeforeResources(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	for _, selection := range []struct{ backend, member, reason string }{
		{"legacy_v4", "rt-sim", "backend_retired"},
		{"rt_core", "sim", "service_retired"},
		{"rt_core", "controller", "service_retired"},
		{"rt_core", "cell", "service_retired"},
	} {
		t.Run(selection.backend+"/"+selection.member, func(t *testing.T) {
			cache := filepath.Join(t.TempDir(), "uncreated")
			cmd := exec.Command("bash", filepath.Join(root, "dev-stack.sh"), "start", selection.member)
			cmd.Env = append(os.Environ(), "DEV_STACK_BACKEND="+selection.backend, "ROSIE_DEV_STACK_DIR="+cache)
			output, err := cmd.CombinedOutput()
			if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 || !strings.Contains(string(output), selection.reason) {
				t.Fatalf("selection: %v %s", err, output)
			}
			if _, err := os.Stat(cache); !os.IsNotExist(err) {
				t.Fatalf("cache acquired: %v", err)
			}
		})
	}
}

func TestRetiredMuJoCoControllerTransport(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	cmd := exec.Command("python3", filepath.Join(root, "mujoco-sim/v1/python/cellsim/main.py"), "--udp-listen", "127.0.0.1:0")
	output, err := cmd.CombinedOutput()
	if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 || !strings.Contains(string(output), "option_retired") {
		t.Fatalf("transport: %v %s", err, output)
	}
}
