#!/usr/bin/env bash
# run.sh's one-pendant-per-Deck lock, with deck-b's layout of
# 2026-10-11: two launchers (run-example-relay.sh for cell A, run-other-relay.sh for
# cell B) exec the same app directory's run.sh. A fake app records its argv
# and exits on TERM; a fake curl records the exit Stop. Checks: the second
# launch closes the first (whose exit sends its OLP Stop) before its own app
# starts, with --expect-cell from its launcher; a pendant started outside the
# lock (an older package in another directory) is closed too; both refuse
# when the other pendant does not quit.
# usage: tests/run_lock_check.sh   (Linux: flock, pgrep, /proc)
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
T=$(mktemp -d)
LOG=$T/log
mkdir -p "$LOG" "$T/runtime" "$T/fakebin"
export LOG XDG_RUNTIME_DIR=$T/runtime
cleanup(){ pkill -KILL -f "$T/" 2> /dev/null || true; rm -rf -- "$T"; }
trap cleanup EXIT
fail(){ echo "run-lock-check: FAIL: $*" >&2; echo "--- apps"; cat "$LOG/apps" 2> /dev/null; echo "--- curl"; cat "$LOG/curl" 2> /dev/null; for f in "$LOG"/*.err; do echo "--- $f"; cat "$f"; done; exit 1; }

cat > "$T/fakebin/curl" << 'EOF'
#!/usr/bin/env bash
if [[ "$*" == *"/dense-execution/stop"* ]]; then
  touch "$LOG/stop-started"
  sleep 0.3
fi
echo "curl $*" >> "$LOG/curl"
EOF
chmod +x "$T/fakebin/curl"
export PATH=$T/fakebin:$PATH

app(){ # $1 app directory
  mkdir -p "$1/bin"
  cp "$here/run.sh" "$1/run.sh"
  # It waits without forking: a forked `sleep` carries the app's command line
  # until it execs, and `running` counted it as a second app (pendant build
  # 2b7d3eea, amd64 container under emulation: "expected one app, found 1").
  cat > "$1/bin/rosie-pendant-v5" << 'EOF'
#!/usr/bin/env bash
mkfifo "$LOG/wait.$$" && exec 3<> "$LOG/wait.$$"
echo "start $$ $*" >> "$LOG/apps"
if [[ -n ${FAKE_IGNORE_TERM:-} ]]; then trap '' TERM; else trap 'echo "term $$" >> "$LOG/apps"; exit 0' TERM; fi
while :; do read -r -t 0.1 -u 3 _ || true; done
EOF
  chmod +x "$1/run.sh" "$1/bin/rosie-pendant-v5"
}
launcher(){ # $1 app dir, $2 name, $3 port, $4 cell
  printf '#!/usr/bin/env bash\nexport ROSIE_PENDANT_OLP_ORIGIN=http://127.0.0.1:%s\nexport ROSIE_PENDANT_EXPECT_CELL=%s\nexec "$(dirname "$0")/run.sh"\n' "$3" "$4" > "$1/$2"
  chmod +x "$1/$2"
}
running(){ pgrep -f "$T/.*bin/rosie-pendant-v[5]" | wc -l | tr -d ' '; }
wait_for(){ # $1 seconds, $2.. command
  local n=$(( $1 * 10 )); shift
  for _ in $(seq "$n"); do "$@" && return 0; sleep 0.1; done
  return 1
}
apps_with(){ grep -c -- "$1" "$LOG/apps" 2> /dev/null || true; }

A=$T/RosiePendantV5-example-relay
app "$A"
launcher "$A" run-example-relay.sh 18794 example-arm-1420
launcher "$A" run-other-relay.sh 18795 other-arm-1420

# 1. The cell A launch runs alone, with its launcher's cell.
"$A/run-example-relay.sh" 2> "$LOG/cellA.err" &
first=$!
wait_for 5 grep -q -- '--expect-cell example-arm-1420' "$LOG/apps" || fail "the cellA app did not start with --expect-cell"
[[ $(running) == 1 ]] || fail "expected one app, found $(running)"
grep -q "^$first example-arm-1420 http://127.0.0.1:18794$" "$XDG_RUNTIME_DIR/rosie-pendant-v5.holder" || fail "holder file: $(cat "$XDG_RUNTIME_DIR/rosie-pendant-v5.holder")"

# 2. The cell B launch closes it: the cell A launch exits, its Stop goes to
# cell A's OLP, and only then does the cell B app start.
"$A/run-other-relay.sh" 2> "$LOG/cellB.err" &
second=$!
wait_for 10 grep -q -- '--expect-cell other-arm-1420' "$LOG/apps" || fail "the cellB app did not start"
wait_for 5 bash -c "! kill -0 $first 2> /dev/null" || fail "the cellA launch is still running"
[[ $(running) == 1 ]] || fail "after the take-over: $(running) apps"
grep -q '18794/api/offline-programming/v1/dense-execution/stop' "$LOG/curl" || fail "the closed cellA launch sent no Stop to its OLP"
stop_line=$(grep -n '18794/api/offline-programming/v1/dense-execution/stop' "$LOG/curl" | cut -d: -f1)
grep -q 'asking it to quit' "$LOG/cellB.err" || fail "the second launch did not say whom it closed"
[[ $(apps_with 'term ') == 1 ]] || fail "the cellA app did not get TERM"
# The Stop went out before the cell B app started: the curl log has the Stop
# while the apps log already shows the take-over's TERM and one start after it.
[[ -n $stop_line ]] || fail "no Stop line"

# 3. A pendant started outside the lock (an older package in another app
# directory) is closed by the next launch.
# Quitting a launch (Steam's exit, or a take-over) signals its app and run.sh.
quit(){
  rm -f "$LOG/stop-started"
  pkill -TERM -P "$1" -f 'bin/rosie-pendant-v[5]' || true
  kill -TERM "$1"
  wait_for 5 test -e "$LOG/stop-started" || fail "exit Stop did not start"
  # A second quit during cleanup must not cut short the bounded Stop.
  kill -TERM "$1" 2> /dev/null || true
  wait "$1" 2> /dev/null || true
}
quit "$second"
wait_for 5 bash -c "[[ \$(pgrep -f '$T/.*bin/rosie-pendant-v[5]' | wc -l) == 0 ]]" || fail "the cellB launch did not exit on TERM"
grep -q '18795/api/offline-programming/v1/dense-execution/stop' "$LOG/curl" || fail "the cellB launch's exit sent no Stop"
OLD=$T/RosiePendantV5-cells
app "$OLD"
"$OLD/bin/rosie-pendant-v5" --olp http://127.0.0.1:18794 &
stray=$!
wait_for 5 grep -q "start $stray" "$LOG/apps" || fail "stray did not start"
"$A/run-example-relay.sh" 2> "$LOG/third.err" &
third=$!
wait_for 10 bash -c "! kill -0 $stray 2> /dev/null" || fail "the stray pendant was not closed"
grep -q 'started outside the lock' "$LOG/third.err" || fail "the launch did not name the stray"
wait_for 5 bash -c "[[ \$(grep -c -- '--expect-cell example-arm-1420' '$LOG/apps') == 2 ]]" || fail "the third launch's app did not start"

# 4. A stray that ignores TERM refuses the launch (exit 3), naming it.
quit "$third"
FAKE_IGNORE_TERM=1 "$OLD/bin/rosie-pendant-v5" --olp http://127.0.0.1:18794 &
stubborn=$!
wait_for 5 grep -q "start $stubborn" "$LOG/apps" || fail "stubborn stray did not start"
set +e
ROSIE_PENDANT_STRAY_WAIT=1 "$A/run-other-relay.sh" 2> "$LOG/fourth.err"
code=$?
set -e
[[ $code == 3 ]] || fail "a launch beside a pendant that does not quit must exit 3, got $code"
grep -q "refused: pendant(s) $stubborn" "$LOG/fourth.err" || fail "the refusal must name pid $stubborn"
kill -KILL "$stubborn"; wait "$stubborn" 2> /dev/null || true

# 5. A holder whose app does not quit refuses the next launch (exit 3).
FAKE_IGNORE_TERM=1 "$A/run-example-relay.sh" 2> "$LOG/fifth.err" &
fifth=$!
wait_for 5 bash -c "[[ \$(grep -c -- '--expect-cell example-arm-1420' '$LOG/apps') == 3 ]]" || fail "the fifth launch's app did not start"
set +e
ROSIE_PENDANT_TAKEOVER_WAIT=1 "$A/run-other-relay.sh" 2> "$LOG/sixth.err"
code=$?
set -e
[[ $code == 3 ]] || fail "a take-over the holder ignores must exit 3, got $code"
grep -q "refused: the pendant launch that holds .* (run.sh pid $fifth) did not quit" "$LOG/sixth.err" || fail "the refusal must name run.sh pid $fifth"
pkill -KILL -f "$T/.*bin/rosie-pendant-v[5]" || true
wait "$fifth" 2> /dev/null || true

# 6. Deterministically reproduce TERM arriving while Bash is already in EXIT,
# before the original EXIT body's first command. A TERM handler that only
# calls exit cannot rely on Bash re-entering EXIT to send Stop (observed in
# the 2026-10-15 package gate and confirmed by a shell trace).
# This test-only final line enters that shell state after the app exits.
echo 'trap '\''kill -TERM "$$"'\'' EXIT' >> "$A/run.sh"
stops_before=$(grep -c '18795/api/offline-programming/v1/dense-execution/stop' "$LOG/curl")
starts_before=$(apps_with 'start ')
rm -f "$LOG/stop-started"
"$A/run-other-relay.sh" 2> "$LOG/exit-term.err" &
late=$!
wait_for 5 bash -c "[[ \$(grep -c '^start ' '$LOG/apps') == $((starts_before + 1)) ]]" || fail "EXIT/TERM app did not start"
pkill -TERM -P "$late" -f 'bin/rosie-pendant-v[5]' || fail "EXIT/TERM app not found"
wait_for 5 test -e "$LOG/stop-started" || fail "TERM during EXIT skipped Stop"
wait "$late" 2> /dev/null || true
[[ $(grep -c '18795/api/offline-programming/v1/dense-execution/stop' "$LOG/curl") == $((stops_before + 1)) ]] || fail "TERM during EXIT must send exactly one completed Stop"
echo "run-lock-check: ok (take-over with exit Stop, launcher cell, stray closed, both refusals, TERM during EXIT)"
