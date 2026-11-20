//go:build linux

package simulator

import (
	"context"
	"fmt"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// The launcher may borrow a reachable explicit adapter, but must never replace
// an unavailable explicit binding or start native children for the legacy path.
func TestLocalLauncherAdapterSelection(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	// Socket paths use TMPDIR; Go's test directories may use a build filesystem.
	runtime, err := os.MkdirTemp("", "ll-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(runtime) })
	socket := filepath.Join(runtime, "control.sock")
	listener, err := net.Listen("unix", socket)
	if err != nil {
		t.Fatal(err)
	}
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/v1/describe" {
			http.NotFound(w, r)
			return
		}
		fmt.Fprint(w, `{"data":{"backend":"simulation"}}`)
	})}
	go server.Serve(listener)
	t.Cleanup(func() { server.Close() })
	build, err := os.MkdirTemp(filepath.Join(root, "rt-core/build"), "launcher-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(build) })
	for _, test := range []struct {
		name, backend, socket string
		success               bool
	}{
		{"reuse", "rt_core", socket, true},
		{"legacy", "legacy_v4", socket + "-missing", false},
		{"unavailable", "rt_core", socket + "-missing", false},
	} {
		t.Run(test.name, func(t *testing.T) {
			ctx, cancel := context.WithTimeout(context.Background(), nativeJogStartup)
			defer cancel()
			command := exec.CommandContext(ctx, "bash", "-c", `source "$1"
trap cleanup EXIT
ensure_local_rt_core
[[ ${#RT_CHILDREN[@]} == 0 ]]
printf '%s|%s\n' "$OFFLINE_PROGRAMMING_EXECUTION_BACKEND" "$OFFLINE_PROGRAMMING_JOG_BACKEND"
`, "launcher-test", filepath.Join(root, "offline-programming/v1/start-offline-programming.sh"))
			command.Env = append(os.Environ(), "ROSIEOS_OLP_CACHE="+build,
				"ROSIE_LOCAL_RT_BUILD="+build, "ROSIE_RT_CONTROL_SOCKET="+test.socket,
				"OFFLINE_PROGRAMMING_EXECUTION_BACKEND=", "OFFLINE_PROGRAMMING_JOG_BACKEND=",
				"DEV_STACK_BACKEND="+test.backend)
			output, err := command.CombinedOutput()
			if (err == nil) != test.success {
				t.Fatalf("launcher: %v\n%s", err, output)
			}
			if test.success && !strings.Contains(string(output), test.backend+"|"+test.backend) {
				t.Fatalf("launcher selection: %s", output)
			}
			reason := "configured simulation adapter is unavailable"
			if test.backend == "legacy_v4" {
				reason = "backend_retired"
			}
			if !test.success && !strings.Contains(string(output), reason) {
				t.Fatalf("missing adapter was not refused: %s", output)
			}
		})
	}
	if _, err := os.Stat(socket); err != nil {
		t.Fatalf("launcher removed borrowed adapter: %v", err)
	}
}

// Status must identify the saved stack binding even when a later shell supplies
// different defaults, preventing diagnostics from pointing at another adapter.
func TestDevStackStatusUsesSavedBinding(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	build, err := os.MkdirTemp(filepath.Join(root, "rt-core/build"), "stack-status-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(build) })
	runtime := filepath.Join(os.TempDir(), "saved-stack")
	for name, value := range map[string]string{"backend": "rt_core", "rt-dir": runtime, "rt-build": build} {
		if err := os.WriteFile(filepath.Join(build, name), []byte(value+"\n"), 0600); err != nil {
			t.Fatal(err)
		}
	}
	ctx, cancel := context.WithTimeout(context.Background(), nativeJogStartup)
	defer cancel()
	command := exec.CommandContext(ctx, "bash", "-c", `tailscale() { return 1; }
export -f tailscale
exec bash "$1" status rt-sim rt-control
`, "status-test", filepath.Join(root, "dev-stack.sh"))
	command.Env = append(os.Environ(), "ROSIE_DEV_STACK_DIR="+build, "DEV_STACK_BACKEND=rt_core",
		"ROSIE_LOCAL_RT_DIR=/wrong", "ROSIE_LOCAL_RT_BUILD=/wrong", "DEV_STACK_UI_HOST=127.0.0.1")
	output, err := command.CombinedOutput()
	if err != nil {
		t.Fatalf("status: %v\n%s", err, output)
	}
	for _, want := range []string{"backend: rt_core", "native=" + runtime + "/ipc.sock", "control=" + runtime + "/control.sock", "jog=" + runtime + "/jog.sock"} {
		if !strings.Contains(string(output), want) {
			t.Fatalf("status omitted %q: %s", want, output)
		}
	}
}
