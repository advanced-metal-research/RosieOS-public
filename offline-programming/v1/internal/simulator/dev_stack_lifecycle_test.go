//go:build linux

package simulator

import (
	"context"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
	"time"
)

// npm changes comm to a name containing spaces after launch. Ownership must
// survive startup and that rename, reject a mismatched start time, and stop the child.
func TestDevStackOwnsAndStopsProcessWithSpacesInName(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	stack := t.TempDir()
	fixture := filepath.Join(stack, "named.py")
	if err := os.WriteFile(fixture, []byte(`import ctypes, pathlib, signal, sys
assert ctypes.CDLL(None).prctl(15, b'ui ) with space', 0, 0, 0) == 0
pathlib.Path(sys.argv[1]).write_text('ready')
signal.pause()
`), 0600); err != nil {
		t.Fatal(err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()
	cmd := exec.CommandContext(ctx, "bash", "-c", `set -euo pipefail
source "$1"
healthy() { pid_of "$1" >/dev/null && [[ -f "$STACK_DIR/ready" ]]; }
command_for() { printf 'exec python3 %q %q\n' "$STACK_DIR/named.py" "$STACK_DIR/ready"; }
# Hold the child before exec installs its owner environment. Readiness must
# reach its pending sleep, not mistake this live child for an exited process.
mkfifo "$STACK_DIR/exec-gate"
setsid() { read -r _ < "$STACK_DIR/exec-gate"; exec /usr/bin/setsid "$@"; }
sleep() {
  if [[ -p "$STACK_DIR/exec-gate" ]]; then
    printf 'exec\n' > "$STACK_DIR/exec-gate"
    rm "$STACK_DIR/exec-gate"
  fi
  command sleep "$@"
}
cleanup() {
  if [[ -n "${pid:-}" ]]; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
start_one ui
pid="$(cat "$STACK_DIR/ui.pid")"
kill -0 "$pid"
if pid_of ui; then echo 'accepted child before owner environment'; exit 1; fi
STACK_WAIT_SECS=5 wait_healthy ui
unset -f sleep setsid
[[ -f "$STACK_DIR/ready" ]]
[[ "$(cat "/proc/$pid/comm")" == 'ui ) with space' ]]
[[ "$(pid_of ui)" == "$pid" ]]
ticks="$(process_start_ticks "$pid")"
[[ "$(cat "$STACK_DIR/ui.pid.start")" == "$ticks" ]]
printf '%s\n' "$((ticks + 1))" > "$STACK_DIR/ui.pid.start"
if pid_of ui; then echo 'accepted wrong start time'; exit 1; fi
kill -0 "$pid"
printf '%s\n' "$ticks" > "$STACK_DIR/ui.pid.start"
stop_one ui
if kill -0 "$pid" 2>/dev/null; then echo 'owned named process survived stop'; exit 1; fi
[[ ! -f "$STACK_DIR/ui.pid" && ! -f "$STACK_DIR/ui.pid.start" ]]
# A genuinely exited child must be classified immediately, without a retry.
printf '%s\n' "$pid" > "$STACK_DIR/ui.pid"
printf '%s\n' "$ticks" > "$STACK_DIR/ui.pid.start"
sleep() { echo 'waited for exited child'; exit 1; }
wait_healthy ui
`, "named-process-probe", filepath.Join(root, "dev-stack.sh"))
	cmd.Env = append(os.Environ(), "DEV_STACK_BACKEND=rt_core", "DEV_STACK_UI_HOST=127.0.0.1", "DEV_STACK_FIREWALL=0", "ROSIE_DEV_STACK_DIR="+stack)
	if output, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("named process lifecycle: %v\n%s", err, output)
	}
}

// Exercise the stack script against real native members; process existence and
// foreign HTTP success must not conceal a stopped or disconnected simulator.
func TestDevStackLifecycleReadiness(t *testing.T) {
	root, err := filepath.Abs("../../../..")
	if err != nil {
		t.Fatal(err)
	}
	build, err := os.MkdirTemp(filepath.Join(root, "rt-core/build"), "lifecycle-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(build) })
	runtime, err := os.MkdirTemp("", "dl-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(runtime) })
	// Unverified startup failure bound; readiness assertions require observed state.
	env := append(os.Environ(), "DEV_STACK_BACKEND=rt_core", "DEV_STACK_FIREWALL=0", "DEV_STACK_UI_HOST=127.0.0.1", "STACK_WAIT_SECS=10",
		"XDG_RUNTIME_DIR="+runtime, "ROSIE_LOCAL_RT_DIR="+runtime, "ROSIE_LOCAL_RT_BUILD="+filepath.Join(build, "runtime"), "ROSIE_DEV_STACK_DIR="+filepath.Join(build, "stack"))
	run := func(script string) ([]byte, error) {
		cmd := exec.Command("bash", "-c", script, "lifecycle-test", filepath.Join(root, "dev-stack.sh"))
		cmd.Env = env
		return cmd.CombinedOutput()
	}
	t.Cleanup(func() {
		output, err := run(`set -euo pipefail
stack="$1"
if [[ -f "$ROSIE_DEV_STACK_DIR/rt-sim.pid" ]]; then kill -CONT "$(cat "$ROSIE_DEV_STACK_DIR/rt-sim.pid")" 2>/dev/null || true; fi
bash "$stack" stop rt-sim rt-control`)
		if err != nil {
			t.Errorf("cleanup: %v\n%s", err, output)
		}
	})
	output, err := run(`set -euo pipefail
stack="$1"
assert_state() {
  local output
  output="$(bash "$stack" status "$1")"
  printf '%s\n' "$output"
  grep -Eq "^  $1 +pid +[^ ]+ +$2 +" <<< "$output"
}
assert_state rt-control down
# A foreign process's live pid does not prove stack ownership.
mkdir -p "$ROSIE_DEV_STACK_DIR"
echo "$PPID" > "$ROSIE_DEV_STACK_DIR/rt-control.pid"
awk '{print $22}' "/proc/$PPID/stat" > "$ROSIE_DEV_STACK_DIR/rt-control.pid.start"
assert_state rt-control down
bash "$stack" start rt-sim rt-control
assert_state rt-sim healthy
assert_state rt-control healthy
pid="$(cat "$ROSIE_DEV_STACK_DIR/rt-sim.pid")"
kill -STOP "$pid"
assert_state rt-sim down
assert_state rt-control down
kill -CONT "$pid"
assert_state rt-sim healthy
assert_state rt-control healthy
kill -KILL "$pid"
assert_state rt-sim down
assert_state rt-control down
`)
	if err != nil {
		t.Fatalf("lifecycle script: %v\n%s", err, output)
	}
}
