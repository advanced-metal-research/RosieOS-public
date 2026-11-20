package main

import (
	"strings"
	"testing"
)

// The tier follows the build: with the shipped binaries' import graph (read
// from this checkout), a change to an off-cell rt-core tool needs no package,
// while anything rt-control, rtctl or the native core is built from does.
func TestReleaseShippedInputsFollowTheBuild(t *testing.T) {
	repo, err := repoRoot()
	if err != nil {
		t.Fatal(err)
	}
	servo, err := releaseGoDeps(repo, "rt-core", releaseShippedMains)
	if err != nil {
		t.Skipf("go list unavailable: %v", err)
	}
	olp, err := releaseGoDeps(repo, "offline-programming/v1", []string{"."})
	if err != nil {
		t.Skipf("go list unavailable: %v", err)
	}
	s := &releaseShipped{ServoGo: servo, OLPGo: olp}
	target := releaseTarget{Shipped: s}
	for p, want := range map[string][2]bool{ // servo package, OLP
		"rt-core/cmd/rt-control/main.go":           {true, false},
		"rt-core/adapters/rosie/control/server.go": {true, true},
		"rt-core/sdk/control/path_jog.go":          {true, true},
		"rt-core/tools/rtctl/render.go":            {true, true},
		"rt-core/tools/packageinfo/main.go":        {true, false},
		"rt-core/src/main.cpp":                     {true, false},
		"rt-core/engine/cycle_machine.hpp":         {true, false},
		"rt-core/host/install.sh":                  {true, false},
		"rt-core/config/drives/a6ec.json":          {true, false},
		"robot_description/go/description.go":      {true, true},
		"rt-core/tools/tuningreport/report.go":     {false, false},
		"rt-core/tools/cidriver/main.go":           {false, false},
		"rt-core/host/lifecycle-test.sh":           {false, false},
		"rt-core/config/cells/bench-relay.json":    {false, false},
	} {
		if got := [2]bool{target.runtimePath(p), target.olpPath(p)}; got != want {
			t.Errorf("%s: servo package, OLP = %v, want %v", p, got, want)
		}
	}
	// Without the graph the prefix rule stands: every rt-core source.
	if !(releaseTarget{}).runtimePath("rt-core/tools/tuningreport/report.go") {
		t.Error("the conservative rule must keep every rt-core source")
	}

	in := releaseInstalled{RTCoreCommit: releaseFd9af899, OLPCommit: release9a48c367, Configuration: releasePin37, Capabilities: releaseCaps114}
	tools := releaseTarget{Commit: release9a48c367, Configuration: releasePin37, Capabilities: releaseCaps114, RTKnown: true, OLPKnown: true, Shipped: s,
		RTChanges: []string{"rt-core/tools/tuningreport/report.go", "rt-core/tools/cidriver/main.go"}, OLPChanges: []string{"rt-core/tools/tuningreport/report.go"}}
	plan, err := releaseChoose(in, tools, "auto")
	if err != nil {
		t.Fatal(err)
	}
	if plan.Tier != releaseTierNone || plan.OLP || !releaseHas(plan.Notes, "2 changed rt-core path(s) are in no deployed binary") {
		t.Fatalf("off-cell tools only: %+v", plan)
	}
}

func TestReleaseShippedNeedsTheCheckoutAtTheCommit(t *testing.T) {
	repo, _ := repoRoot()
	s, why := releaseShippedAt(repo, release9a48c367)
	if s != nil || !strings.Contains(why, "differ from 9a48c367") {
		t.Fatalf("an older commit than the checkout: %v %q", s, why)
	}
}
