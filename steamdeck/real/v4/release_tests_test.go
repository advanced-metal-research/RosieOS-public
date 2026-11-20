package main

import (
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"sort"
	"strconv"
	"strings"
	"testing"
)

func gateNames(sel gateSelection) []string {
	var out []string
	for _, s := range sel.Suites {
		out = append(out, s.Name)
	}
	return out
}

func TestGateFullRunsEverySuiteLongestChainFirst(t *testing.T) {
	sel, err := gateSelect("full", nil, true, nil)
	if err != nil {
		t.Fatal(err)
	}
	names := gateNames(sel)
	if len(names) != len(gateSuites) || names[0] != "make_test" {
		t.Fatalf("full order %v", names)
	}
	index := map[string]int{}
	for i, n := range names {
		index[n] = i
	}
	// npm ci starts before the shorter suites: npm test (308 s) waits for it.
	if index["ui_npm_ci"] > index["olp"] || index["ui_npm_ci"] > index["ui_test"] {
		t.Fatalf("ui_npm_ci must start early: %v", names)
	}
	// The suites' own measurements (example-relay, 2026-10-07, one at a time) sum
	// to about 29.5 min. The suites that run alone (make_test_serial 352 s,
	// faultmatrix_shutdown 22 s, sdk_path_jog 10 s, olp_composed 6 s) hold
	// every lane; the rest share three lanes around them.
	sequential := 0.0
	for _, s := range sel.Suites {
		sequential += s.Seconds
	}
	if sequential < 1700 || sequential > 1800 {
		t.Fatalf("catalogue total %.0f s, the fd9af899 gate took 1770 s", sequential)
	}
	lane, alone := 0.0, 0.0
	for _, s := range sel.Suites {
		if s.Alone {
			alone += s.Seconds
		} else {
			lane += s.Seconds
		}
	}
	got := gateLanePlan(sel.Suites, 3)
	if got < alone+lane/3 || got > alone+lane {
		t.Fatalf("three lanes: %.0f s, outside [%.0f, %.0f] (alone %.0f s exclusive, lane work %.0f s on 3 lanes)", got, alone+lane/3, alone+lane, alone, lane)
	}
	t.Logf("full gate on three lanes: %.0f s estimated (alone %.0f s, lane work %.0f s)", got, alone, lane)
	if got := gateLanePlan(sel.Suites, 1); got != sequential {
		t.Fatalf("one lane %.0f s != sequential %.0f s", got, sequential)
	}
}

func TestGateFastSelectsOnlyAffectedSuites(t *testing.T) {
	cases := []struct {
		name    string
		changed []string
		want    []string
	}{
		{"documentation only", []string{"README.md", "rt-core/docs/CONTRACTS.md", ".cursor/memory/DEVLOG.md", "steamdeck/real/v4/README.md"}, nil},
		{"machine config value and re-pin", []string{"rt-core/config/machines/example-arm-1430.json", "rt-core/config/cells/example-arm-1420.json"},
			[]string{"cells", "olp", "rtctl"}},
		{"OLP server only", []string{"offline-programming/v1/internal/execution/maintenance.go"}, []string{"olp"}},
		{"denseexec reaches the composed test", []string{"offline-programming/v1/internal/denseexec/jog.go"}, []string{"olp", "olp_composed"}},
		{"UI brings npm ci and the pendant bundle check", []string{"offline-programming/v1/ui/src/machine/jog.ts"}, []string{"pendant_olp_core", "ui_npm_ci", "ui_test"}},
		{"resolver", []string{"motion-server/v1/src/cartesian.cpp"}, []string{"motion_test", "olp_composed"}},
		{"native core", []string{"rt-core/engine/cycle_machine.hpp"}, []string{"faultmatrix_shutdown", "make_test", "make_test_serial", "olp_composed", "rtctl", "sdk_path_jog"}},
		{"deploy tool runs on the Mac", []string{"steamdeck/real/v4/release.go"}, nil},
	}
	for _, c := range cases {
		sel, err := gateSelect("fast", c.changed, true, nil)
		if err != nil {
			t.Fatal(err)
		}
		got := gateNames(sel)
		sort.Strings(got)
		if strings.Join(got, ",") != strings.Join(c.want, ",") {
			t.Errorf("%s: %v, want %v", c.name, got, c.want)
		}
		if len(got)+len(sel.Skipped) != len(gateSuites) {
			t.Errorf("%s: %d selected + %d skipped != %d suites", c.name, len(got), len(sel.Skipped), len(gateSuites))
		}
	}
	// The real 9a48c367 OLP-only range: rt-control's maintenance route came
	// with it, so the native suites run too.
	sel, _ := gateSelect("fast", releaseGolden(t, "d7a81080..9a48c367"), true, nil)
	got := strings.Join(gateNames(sel), ",")
	for _, want := range []string{"make_test", "olp", "ui_test", "sdk_path_jog"} {
		if !strings.Contains(got, want) {
			t.Errorf("d7a81080..9a48c367 fast lacks %s: %s", want, got)
		}
	}
	if strings.Contains(got, "motion_test") || strings.Contains(got, "rtctl") {
		t.Errorf("d7a81080..9a48c367 touched neither the resolver nor rtctl: %s", got)
	}
	// Unknown base: everything.
	sel, _ = gateSelect("fast", nil, false, nil)
	if len(sel.Suites) != len(gateSuites) {
		t.Fatalf("unknown base ran %d suites", len(sel.Suites))
	}
	if _, err := gateSelect("quick", nil, true, nil); err == nil {
		t.Fatal("an unknown level must refuse")
	}
}

func TestGateJournalTimingsReorder(t *testing.T) {
	sel, _ := gateSelect("full", nil, true, map[string]float64{"cells": 5000})
	if sel.Suites[0].Name != "cells" {
		t.Fatalf("a measured 5000 s suite starts first: %v", gateNames(sel))
	}
}

// The scheduler under bash with a fake gate(): never more than $LANES suites
// at once, a suite starts only after its table prerequisites, and every suite
// runs exactly once.
func TestGateSchedulerRespectsLanesAndOrder(t *testing.T) {
	if runtime.GOOS == "windows" {
		t.Skip("bash")
	}
	sel, _ := gateSelect("full", nil, true, nil)
	dir := t.TempDir()
	script := "set -u\nL=" + shellQuote(dir) + "\nLANES=3\n" + gateSchedulerShell + `gate(){
  local n=$1
  echo "start $n $(ls "$L"/*.running 2> /dev/null | wc -l | tr -d ' ')" >> "$L/events"
  : > "$L/$n.running"; sleep 0.3; rm -f "$L/$n.running"
  echo "end $n" >> "$L/events"
}
gate_run_all <<'ROSIE_SUITES'
` + gateSuiteTable(sel) + "ROSIE_SUITES\n"
	out, err := exec.Command("bash", "-c", script).CombinedOutput()
	if err != nil {
		t.Fatalf("scheduler: %v\n%s", err, out)
	}
	raw, _ := os.ReadFile(filepath.Join(dir, "events"))
	ended := map[string]bool{}
	starts, alone := 0, ""
	for _, line := range strings.Split(strings.TrimSpace(string(raw)), "\n") {
		f := strings.Fields(line)
		switch f[0] {
		case "start":
			starts++
			n, _ := strconv.Atoi(f[2])
			if n >= 3 {
				t.Errorf("%s started with %d already running on 3 lanes", f[1], n)
			}
			if alone != "" {
				t.Errorf("%s started while %s ran alone", f[1], alone)
			}
			s, _ := gateSuiteByName(f[1])
			if s.Alone {
				if n != 0 {
					t.Errorf("%s runs alone but started with %d other suite(s) running", f[1], n)
				}
				alone = f[1]
			}
			for _, dep := range append(append([]string(nil), s.Needs...), s.After...) {
				if !ended[dep] {
					t.Errorf("%s started before %s ended", f[1], dep)
				}
			}
		case "end":
			ended[f[1]] = true
			if alone == f[1] {
				alone = ""
			}
		}
	}
	var solo []string
	for _, s := range gateSuites {
		if s.Alone {
			solo = append(solo, s.Name)
		}
	}
	sort.Strings(solo)
	if strings.Join(solo, ",") != "faultmatrix_shutdown,make_test_serial,olp_composed,rtctl,sdk_path_jog" {
		t.Fatalf("suites with real-time assertions run alone: %v", solo)
	}
	if starts != len(gateSuites) || len(ended) != len(gateSuites) {
		t.Fatalf("%d starts, %d ends:\n%s", starts, len(ended), raw)
	}
}
