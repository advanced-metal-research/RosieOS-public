package main

import (
	"fmt"
	"os"
	"path"
	"path/filepath"
	"sort"
	"strconv"
	"strings"
)

// Test levels for the build host's gate (`realtime-core build --test`,
// `realtime-core release --test`):
//
//   - full: every suite below, the deploy gate;
//   - fast: each suite whose inputs changed since its last recorded pass on
//     the cell (release_gate_history.go); a suite with no usable record runs.
//     With --regate the records are ignored and fast takes the suites whose
//     inputs changed between the cell's installed commits and the commit
//     under test (or between the previous tree and it). A path no suite
//     claims runs nothing; documentation never runs anything.
//
// Suites run in the build host's persistent tree after the build, longest
// chain first, several at once (lanes), each logged on its own; a suite that needs
// another's build output waits for it (After). Failures are classified on the
// Mac against rt-core/tools/cidriver/known_failures.json (the one list, which
// rt-core's CI driver validates): its suite names are these.

type gateSuite struct {
	Name string
	Dir  string // under the tree
	Cmd  string // bash, run in Dir; $JOBS, $S, $D and $TREE are set; make
	// runs its recipes through $S/tsh so each recipe line is timed
	// Needs: suites that run whenever this one does, first (npm ci before
	// npm test). After: suites that finish first when both run (they rebuild
	// output this one reads); they are not added for it.
	Needs, After []string
	// Alone: the suite asserts real-time behaviour (clocks, deadlines,
	// daemons it starts) and runs with no other suite running: it starts once
	// every lane is free, and nothing starts until it ends. Under lanes on
	// 2026-10-07 sdk_path_jog failed jog_clock_uncertainty_exceeded on a
	// commit whose gate passed alone three times (DEVLOG 20:17, run 5).
	Alone bool
	// Seconds orders the suites when no journal timing exists. Measured on
	// a relay by the hand-made fd9af899 gate (2026-10-07 13:05-13:35, one
	// suite at a time, nice 15, a fresh clone): see each suite. 0: unmeasured.
	Seconds float64
	// Inputs: a changed path under any of these (documentation and Exclude
	// excepted) selects the suite in fast mode.
	Inputs, Exclude []string
}

// rtCoreNative is everything `make test` compiles or generates from.
var gateRTCoreNative = []string{"rt-core/src/", "rt-core/include/", "rt-core/engine/", "rt-core/host/", "rt-core/protocol/",
	"rt-core/clients/", "rt-core/tests/", "rt-core/Makefile", "rt-core/adapters/", "rt-core/cmd/", "rt-core/ipcclient/",
	"rt-core/internal/", "rt-core/tools/protocolgen/", "rt-core/tools/apigen/", "rt-core/config/drives/", "rt-core/go.mod", "rt-core/go.sum"}

var gateSuites = []gateSuite{
	// make test in two parts (rt-core Makefile): test-runs compiles every
	// binary and runs the C++ contract binaries on the lanes; test-serial is
	// the real-time tail (apigen C++ interop, the Go -race oracle, the host
	// scripts) and runs alone. A commit whose Makefile predates the split runs
	// plain make test as make_test. Seconds: the fd9af899 make_test's 686 s
	// split by the 2026-10-07 17:50 relay micro-benchmark (-j4): lifecycle
	// 206 s + oracle 111 s + package-test 17 s + apigen 18 s = 352 s serial.
	{Name: "make_test", Dir: "rt-core", Cmd: `if make -n test-serial > /dev/null 2>&1; then make -j"$JOBS" test-runs SHELL="$S/tsh"; else make -j"$JOBS" test SHELL="$S/tsh"; fi`,
		Seconds: 334, Inputs: gateRTCoreNative},
	{Name: "make_test_serial", Dir: "rt-core", Cmd: `if make -n test-serial > /dev/null 2>&1; then make test-serial SHELL="$S/tsh"; else echo "this commit's Makefile has no test-serial: make_test ran make test"; fi`,
		After: []string{"make_test"}, Alone: true, Seconds: 352, Inputs: gateRTCoreNative},
	// rtctl's package compiles and runs the native daemon and bring-up
	// fixtures (a daemon test missing its cycle deadline under a loaded host,
	// FaultCycleDeadline, 2026-10-07): it runs alone, and its native sources
	// select it.
	{Name: "rtctl", Dir: "rt-core", Cmd: "go test -count=1 ./tools/rtctl/...", Alone: true, Seconds: 347,
		Inputs: []string{"rt-core/tools/rtctl/", "rt-core/config/", "robot_description/", "rt-core/go.mod",
			"rt-core/tests/", "rt-core/src/", "rt-core/include/", "rt-core/engine/"}},
	{Name: "motion_test", Dir: "motion-server/v1", Cmd: `make test BIN_DIR="$S/motion-test-bin" SHELL="$S/tsh"`, Seconds: 345,
		Inputs: []string{"motion-server/v1/", "robot_description/"}},
	{Name: "ui_test", Dir: "offline-programming/v1/ui", Cmd: "npm test", Needs: []string{"ui_npm_ci"}, Seconds: 308,
		Inputs: []string{"offline-programming/v1/ui/", "rt-core/clients/ts/"}},
	{Name: "olp", Dir: "offline-programming/v1", Cmd: "go test -count=1 ./internal/...", Seconds: 39,
		Inputs:  []string{"offline-programming/v1/", "rt-core/sdk/", "rt-core/adapters/", "rt-core/ipcclient/", "rt-core/config/", "robot_description/"},
		Exclude: []string{"offline-programming/v1/ui/"}},
	{Name: "faultmatrix_shutdown", Dir: "rt-core", Cmd: "go test -race -count=1 -run TestShutdown ./ipcclient/faultmatrix", After: []string{"make_test"}, Alone: true, Seconds: 22,
		Inputs: []string{"rt-core/ipcclient/", "rt-core/src/", "rt-core/include/", "rt-core/engine/"}},
	{Name: "sdk_path_jog", Dir: "rt-core", Cmd: "go test -count=1 -p=1 -run PathJog -v ./sdk/control", After: []string{"make_test"}, Alone: true, Seconds: 10,
		Inputs: []string{"rt-core/sdk/", "rt-core/adapters/rosie/control/", "rt-core/engine/", "rt-core/include/"}},
	{Name: "olp_composed", Dir: "offline-programming/v1",
		Cmd:   `env RTCORE_COMPOSED=1 OLP_CARTESIAN_RESOLVER="$D/motion-bin/robot-v4-cartesiand" go test -count=1 -p=1 -run TestCartesianJogComposed -v ./internal/denseexec`,
		After: []string{"make_test", "olp"}, Alone: true, Seconds: 6,
		Inputs: []string{"offline-programming/v1/internal/denseexec/", "rt-core/sdk/", "rt-core/engine/", "rt-core/include/", "rt-core/adapters/rosie/control/", "motion-server/v1/"}},
	// npm ci only when package-lock.json changed since the tree's last install
	// (node_modules persists in the build host's tree).
	{Name: "ui_npm_ci", Dir: "offline-programming/v1/ui",
		Cmd:     `lock=$(sha256sum package-lock.json | cut -d' ' -f1); if [ -d node_modules ] && [ "$(cat node_modules/.rosie-lock-sha256 2> /dev/null)" = "$lock" ]; then echo "node_modules current for package-lock $lock"; else npm ci --no-audit --no-fund && printf '%s\n' "$lock" > node_modules/.rosie-lock-sha256; fi`,
		Seconds: 6, Inputs: []string{"offline-programming/v1/ui/", "steamdeck/real/v5/olp-core/"}},
	// The pendant's bundled OLP program logic under node (steamdeck/real/v5
	// make check-core's node half); its Qt checks run in `pendant build`.
	// Unmeasured.
	{Name: "pendant_olp_core", Dir: "offline-programming/v1/ui",
		Cmd:   `node --experimental-strip-types --no-warnings --test "$TREE"/steamdeck/real/v5/olp-core/*.test.ts`,
		Needs: []string{"ui_npm_ci"}, Inputs: []string{"steamdeck/real/v5/olp-core/", "offline-programming/v1/ui/src/"}},
	{Name: "cells", Dir: "rt-core", Cmd: "go test -count=1 ./config/cells/...", Seconds: 1,
		Inputs: []string{"rt-core/config/", "rt-core/tools/rtctl/", "robot_description/"}},
}

func gateSuiteByName(name string) (gateSuite, bool) {
	for _, s := range gateSuites {
		if s.Name == name {
			return s, true
		}
	}
	return gateSuite{}, false
}

func (s gateSuite) selects(p string) bool {
	if releaseDocPath(p) {
		return false
	}
	for _, ex := range s.Exclude {
		if strings.HasPrefix(p, ex) {
			return false
		}
	}
	for _, in := range s.Inputs {
		if strings.HasSuffix(in, "/") && strings.HasPrefix(p, in) || p == in {
			return true
		}
	}
	return false
}

// gateSelection is what a gate run will do and why.
type gateSelection struct {
	Level   string
	Suites  []gateSuite // in start order: slowest first
	Why     map[string]string
	Skipped []string // fast mode: suites no changed path selects
	// SkipWhy: why a skipped suite did not run and which recorded pass
	// covers it (fast mode against the gate records).
	SkipWhy map[string]string
}

// gateSelect picks and orders the suites. changed is nil with known false
// when the base commit is unknown: fast then runs everything. seconds holds
// journal timings (mean per suite) that override the catalogue's.
func gateSelect(level string, changed []string, known bool, seconds map[string]float64) (gateSelection, error) {
	sel := gateSelection{Level: level, Why: map[string]string{}}
	switch level {
	case "full":
	case "fast":
	default:
		return sel, fmt.Errorf("--test must be fast or full")
	}
	for _, s := range gateSuites {
		switch {
		case level == "full":
			sel.Why[s.Name] = "full gate"
		case !known:
			sel.Why[s.Name] = "base commit unknown: every suite"
		default:
			var hits []string
			for _, p := range changed {
				if s.selects(p) {
					hits = append(hits, p)
				}
			}
			if len(hits) == 0 {
				sel.Skipped = append(sel.Skipped, s.Name)
				continue
			}
			sel.Why[s.Name] = fmt.Sprintf("%d changed path(s): %s", len(hits), releaseSummarizePaths(hits, 2))
		}
		sel.Suites = append(sel.Suites, s)
	}
	gateOrder(&sel, seconds)
	return sel, nil
}

// gateOrder completes a selection: journal timings, the suites the chosen
// ones need, then the start order.
func gateOrder(sel *gateSelection, seconds map[string]float64) {
	for i := 0; i < len(sel.Suites); i++ {
		for _, dep := range sel.Suites[i].Needs {
			if _, ok := sel.Why[dep]; ok {
				continue
			}
			if s, ok := gateSuiteByName(dep); ok {
				sel.Why[dep] = "needed by " + sel.Suites[i].Name
				for j, name := range sel.Skipped {
					if name == dep {
						sel.Skipped = append(sel.Skipped[:j], sel.Skipped[j+1:]...)
						break
					}
				}
				delete(sel.SkipWhy, dep)
				sel.Suites = append(sel.Suites, s)
			}
		}
	}
	for i := range sel.Suites {
		if v, ok := seconds[sel.Suites[i].Name]; ok && v > 0 {
			sel.Suites[i].Seconds = v
		}
	}
	// Start order: longest chain first, a chain being a suite plus the suites
	// that wait for it (npm ci starts early because npm test is long).
	chain := map[string]float64{}
	var length func(s gateSuite) float64
	length = func(s gateSuite) float64 {
		if v, ok := chain[s.Name]; ok {
			return v
		}
		longest := 0.0
		for _, t := range sel.Suites {
			for _, dep := range append(append([]string(nil), t.Needs...), t.After...) {
				if dep == s.Name {
					longest = max(longest, length(t))
				}
			}
		}
		chain[s.Name] = s.Seconds + longest
		return chain[s.Name]
	}
	for _, s := range sel.Suites {
		length(s)
	}
	sort.SliceStable(sel.Suites, func(i, j int) bool { return chain[sel.Suites[i].Name] > chain[sel.Suites[j].Name] })
}

// gateLanePlan estimates the wall time of a selection on n lanes: the suites
// start in order on the first free lane once their After suites finished; an
// Alone suite waits for every lane and holds them all.
// It assumes suites do not slow each other down, so it is a lower bound.
func gateLanePlan(suites []gateSuite, lanes int) float64 {
	if lanes < 1 {
		lanes = 1
	}
	free := make([]float64, lanes)
	finish := map[string]float64{}
	selected := map[string]bool{}
	for _, s := range suites {
		selected[s.Name] = true
	}
	total := 0.0
	for _, s := range suites {
		ready := 0.0
		for _, dep := range append(append([]string(nil), s.Needs...), s.After...) {
			if selected[dep] {
				ready = max(ready, finish[dep])
			}
		}
		if s.Alone {
			start := ready
			for _, f := range free {
				start = max(start, f)
			}
			for i := range free {
				free[i] = start + s.Seconds
			}
			finish[s.Name] = start + s.Seconds
			total = max(total, finish[s.Name])
			continue
		}
		best := 0
		for i := range free {
			if free[i] < free[best] {
				best = i
			}
		}
		start := max(free[best], ready)
		free[best] = start + s.Seconds
		finish[s.Name] = free[best]
		total = max(total, free[best])
	}
	return total
}

func (sel gateSelection) Describe() []string {
	var lines []string
	for _, s := range sel.Suites {
		est := "unmeasured"
		if s.Seconds > 0 {
			est = formatGateSeconds(s.Seconds)
		}
		alone := ""
		if s.Alone {
			alone = ", alone"
		}
		lines = append(lines, fmt.Sprintf("%s (%s, %s%s): %s", s.Name, s.Dir, est, alone, sel.Why[s.Name]))
	}
	var unaffected []string
	for _, name := range sel.Skipped {
		if why := sel.SkipWhy[name]; why != "" {
			lines = append(lines, fmt.Sprintf("skipped %s: %s", name, why))
		} else {
			unaffected = append(unaffected, name)
		}
	}
	if len(unaffected) > 0 {
		lines = append(lines, "not affected: "+strings.Join(unaffected, ", "))
	}
	return lines
}

// Summary is one line: how many suites run and their summed measured time
// (the lanes overlap the ones that do not run alone).
func (sel gateSelection) Summary() string {
	total := 0.0
	for _, s := range sel.Suites {
		total += s.Seconds
	}
	return fmt.Sprintf("%s gate: %d of %d suite(s) run, %s of suite time one after another (journal means, else the catalogue), %d skipped",
		sel.Level, len(sel.Suites), len(gateSuites), formatGateSeconds(total), len(sel.Skipped))
}

func formatGateSeconds(s float64) string {
	if s < 60 {
		return strconv.FormatFloat(s, 'f', 0, 64) + "s"
	}
	return fmt.Sprintf("%dm%02ds", int(s)/60, int(s)%60)
}

// gateSchedulerShell runs the selected suites from a table on stdin (name,
// directory, after, alone, command, separated by 0x1f: not whitespace, so an
// empty after field is kept) on $LANES lanes in table order,
// each through gate() (defined by the caller), and waits for all. An alone
// suite (alone=1) starts only when nothing else runs, and once one is ready
// nothing else starts until it has run, so it is neither starved nor
// overlapped. It uses only bash 3 features and one-second polling: no
// `wait -n`.
const gateSchedulerShell = `gate_run_all(){
  local names=() dirs=() afters=() alones=() cmds=() started=() n d a o c i ok dep running all solo
  while IFS=$'\x1f' read -r n d a o c; do [ -n "$n" ] || continue; names+=("$n"); dirs+=("$d"); afters+=("$a"); alones+=("$o"); cmds+=("$c"); started+=(0); done
  while :; do
    all=1; running=0; solo=0
    for i in "${!names[@]}"; do
      [ -e "$L/${names[$i]}.done" ] && continue
      all=0
      if [ "${started[$i]}" = 1 ]; then running=$((running + 1)); [ "${alones[$i]}" = 1 ] && solo=1; fi
    done
    [ "$all" = 1 ] && break
    for i in "${!names[@]}"; do
      [ "$solo" = 0 ] || break
      [ "${started[$i]}" = 1 ] && continue
      [ "$running" -lt "$LANES" ] || break
      ok=1
      for dep in ${afters[$i]//,/ }; do [ -e "$L/$dep.done" ] || ok=0; done
      [ "$ok" = 1 ] || continue
      if [ "${alones[$i]}" = 1 ]; then
        [ "$running" = 0 ] || break
        solo=1
        ! command -v step > /dev/null 2>&1 || step "gate ${names[$i]} runs alone (real-time assertions)"
      fi
      started[$i]=1; running=$((running + 1))
      ( gate "${names[$i]}" "${dirs[$i]}" "${cmds[$i]}"; : > "$L/${names[$i]}.done" ) &
    done
    sleep 1
  done
  wait
}
`

// gateSuiteTable renders the selection for gate_run_all: After lists only
// suites that are selected (an unselected prerequisite cannot block).
func gateSuiteTable(sel gateSelection) string {
	selected := map[string]bool{}
	for _, s := range sel.Suites {
		selected[s.Name] = true
	}
	var b strings.Builder
	for _, s := range sel.Suites {
		var after []string
		for _, dep := range append(append([]string(nil), s.Needs...), s.After...) {
			if selected[dep] {
				after = append(after, dep)
			}
		}
		fmt.Fprintf(&b, "%s\x1f%s\x1f%s\x1f%s\x1f%s\n", s.Name, s.Dir, strings.Join(after, ","), flagShell(s.Alone), s.Cmd)
	}
	return b.String()
}

// gateSuiteLogDir is where one commit's gate logs live on the build host.
func gateSuiteLogDir(ctx cellDeployContext, commit string) string {
	return path.Join(ctx.Deploy.ScratchDir, "build-"+cellDeployShort(commit), "gate-logs")
}

// gateMeasuredSeconds is each suite's mean duration over the test gates this
// machine journalled (realtime-core timings: spans "test gate/<suite>"); a
// suite that never ran keeps the catalogue's measurement.
func gateMeasuredSeconds() map[string]float64 {
	home, err := os.UserHomeDir()
	if err != nil {
		return nil
	}
	recs, err := loadDeployTimings(filepath.Join(home, ".rosie", "deploy-journal"))
	if err != nil {
		return nil
	}
	return gateMeanSeconds(recs)
}

func gateMeanSeconds(recs []deployTimingRecord) map[string]float64 {
	sum, n := map[string]float64{}, map[string]int{}
	for _, rec := range recs {
		for _, s := range rec.Spans {
			name, ok := strings.CutPrefix(s.Name, "test gate/")
			if !ok || strings.Contains(name, "/") || s.Seconds <= 0 {
				continue
			}
			if _, known := gateSuiteByName(name); known {
				sum[name] += s.Seconds
				n[name]++
			}
		}
	}
	out := map[string]float64{}
	for name, total := range sum {
		out[name] = total / float64(n[name])
	}
	return out
}
