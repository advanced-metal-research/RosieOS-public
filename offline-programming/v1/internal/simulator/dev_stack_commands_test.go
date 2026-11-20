//go:build linux

package simulator

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// Compare exact generated commands with recorded selections, preventing native launcher drift.
func TestDevStackCommandBaselines(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "dev-stack.sh"))
	if err != nil {
		t.Fatal(err)
	}
	function := func(name string) string {
		start := strings.Index(string(raw), name+"() {")
		if start < 0 {
			t.Fatalf("missing %s", name)
		}
		body := string(raw)[start:]
		end := strings.Index(body, "\n}\n")
		if end < 0 {
			t.Fatalf("unterminated %s", name)
		}
		return body[:end+3]
	}
	for _, backend := range []string{"rt_core"} {
		script := stackCommandInputs + "\nBACKEND=" + backend + "\n" + function("ready_file") + function("command_for") +
			"\nmembers=\"catalog nats rt-sim rt-control daemon motion olp ui\"; for member in $members; do command_for \"$member\"; done\n"
		cmd := exec.Command("bash", "-c", script)
		output, err := cmd.CombinedOutput()
		if err != nil {
			t.Fatalf("commands: %v\n%s", err, output)
		}
		want, err := os.ReadFile(filepath.Join("testdata", "dev-stack-"+backend+".commands"))
		if err != nil {
			t.Fatal(err)
		}
		if string(output) != string(want) {
			t.Fatalf("%s command diff\n got:\n%s\nwant:\n%s", backend, output, want)
		}
	}
}

// The launcher must consume the Makefile's default so a normal build is found.
func TestDevStackDenseBinaryPath(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "dev-stack.sh"))
	if err != nil {
		t.Fatal(err)
	}
	var assignment string
	for _, line := range strings.Split(string(raw), "\n") {
		if strings.HasPrefix(line, "MS_BIN=") {
			assignment = line
		}
	}
	if assignment == "" || !strings.Contains(assignment, "print-bin-dir") {
		t.Fatal("dev-stack must query the dense Makefile output directory")
	}
	for _, override := range []string{"", filepath.Join(t.TempDir(), "custom")} {
		cmd := exec.Command("bash", "-c", assignment+"\nprintf '%s\\n' \"$MS_BIN/joint_trajectory_daemon\"")
		cmd.Env = append(os.Environ(), "ROOT="+root, "DEV_STACK_DENSE_BIN_DIR="+override)
		output, err := cmd.CombinedOutput()
		if err != nil {
			t.Fatalf("resolve dense binary: %v\n%s", err, output)
		}
		want := filepath.Join(root, "rt-core", "build", "dense", "joint_trajectory_daemon")
		if override != "" {
			want = filepath.Join(override, "joint_trajectory_daemon")
		}
		if strings.TrimSpace(string(output)) != want {
			t.Fatalf("dense binary = %q, want %q", output, want)
		}
	}
}

// Unverified fixture paths pin command bytes independently of each checkout and user.
const stackCommandInputs = `ROOT=/repo
STACK_DIR=/stack
CATALOG_SCRIPT=/catalog/start.sh
NATS_BIN=/bin/nats-server
NATS_PORT=14222
NATS_MONITOR=18222
SIM_PY=/pixi/python
SIM_ADDR=127.0.0.1:49004
CONTROLLER_ADDR=127.0.0.1:49014
MS_BIN=/bin/dense
DENSE_PLANS=/plans
DAEMON_INGEST=127.0.0.1:8797
NATS_URL=nats://127.0.0.1:14222
ROBOT_CELL=dev-cell
RTCORE_HOST=olpsimrt1
BRIDGE_CELL=
CATALOG_ORIGIN=http://127.0.0.1:8787
MOTION_ORIGIN=http://127.0.0.1:8796
UI_MODE=dev
UI_HOST=127.0.0.1
ROSIE_LOCAL_RT_BUILD=/build
ROSIE_LOCAL_RT_DIR=/runtime
RT_HELPER=/repo/motion-server/v1/local-rt-core.sh
`

// Private defaults must use the selected runtime base and restrict directory access.
func TestDevStackPrivateDirectories(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	base, err := os.MkdirTemp("", "private-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(base) })
	for _, xdg := range []string{"", base} {
		cmd := exec.Command("bash", "-c", `set -euo pipefail
unset ROSIE_DEV_STACK_DIR ROSIE_LOCAL_RT_DIR
base="${XDG_RUNTIME_DIR:-$TMPDIR}"
status="$(bash "$1/dev-stack.sh" status rt-sim)"
grep -F "native=$base/rosie-dev-rt-$UID/ipc.sock" <<< "$status"
[[ $(stat -c %a "$base/rosie-stack-$UID") == 700 ]]
# The helper creates socket directories when launching, after saved binding resolution.
ROSIE_LOCAL_RT_DIR="$base/rosie-dev-rt-$UID" bash "$1/motion-server/v1/local-rt-core.sh" invalid 2>/dev/null || [[ $? == 2 ]]
[[ $(stat -c %a "$base/rosie-dev-rt-$UID") == 700 ]]
# An unsupported helper action still creates and validates its private directories.
bash "$1/motion-server/v1/local-rt-core.sh" invalid 2>/dev/null || [[ $? == 2 ]]
[[ $(stat -c %a "$base/rosie-local-rt-$UID") == 700 ]]
`, "private-test", root)
		cmd.Env = append(os.Environ(), "XDG_RUNTIME_DIR="+xdg, "TMPDIR="+base, "ROSIE_LOCAL_RT_BUILD="+filepath.Join(root, "rt-core/build/private-helper"), "DEV_STACK_BACKEND=rt_core", "DEV_STACK_UI_HOST=127.0.0.1")
		if output, err := cmd.CombinedOutput(); err != nil {
			t.Fatalf("directories: %v\n%s", err, output)
		}
	}
}
