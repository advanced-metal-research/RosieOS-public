package main

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// Golden path lists in testdata/release are `git diff --name-only A B` of real
// commit ranges, kept so the tier choice is judged on what actually changed
// between the 1430 cell releases (TestReleaseGoldenPathsMatchHistory re-derives them
// when the history is present).
const (
	releaseFd9af899 = "fd9af899a9942fa5db31aefb7e333a851e825a53" // rt-core release installed on the 1430 cell (2026-10-07)
	releaseD7a81080 = "d7a81080c4c70363679ca22110a45b24b2cb4720"
	release9a48c367 = "9a48c3676c92d6a533b2d25dbb970709df5026aa"
	releaseCd8c5b4c = "cd8c5b4c"
	release6c6f97c0 = "6c6f97c0"
	releaseCaps114  = "b1e28f06dfc586c7971e93a7524df6a1830719939d1591f0caec9a1008e599ad"
	releasePin37    = "37eb7b1f9e50ea193c19dbe424fdbdc4206a8f17e37169dbdd8f34e9a9813673"
)

func releaseGolden(t *testing.T, name string) []string {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join("testdata", "release", name+".paths"))
	if err != nil {
		t.Fatal(err)
	}
	var out []string
	for _, line := range strings.Split(string(raw), "\n") {
		if line != "" && !strings.HasPrefix(line, "#") {
			out = append(out, line)
		}
	}
	return out
}

func releaseHas(lines []string, want string) bool {
	for _, l := range lines {
		if strings.Contains(l, want) {
			return true
		}
	}
	return false
}

// the 1430 cell on 2026-10-07: rt-core fd9af899 installed, OLP 9a48c367. The commits
// since fd9af899 carry rt-control's maintenance route, so the automatic tier
// is full even though the last OLP-only install shipped 9a48c367's OLP.
func TestReleaseTierFullForRTControlChangesSinceTheInstalledRelease(t *testing.T) {
	in := releaseInstalled{RTCoreCommit: releaseFd9af899, OLPCommit: release9a48c367, Configuration: releasePin37, Capabilities: releaseCaps114}
	target := releaseTarget{Commit: release9a48c367, Configuration: releasePin37, Capabilities: releaseCaps114,
		RTChanges: releaseGolden(t, "fd9af899..9a48c367"), RTKnown: true, OLPKnown: true}
	plan, err := releaseChoose(in, target, "auto")
	if err != nil {
		t.Fatal(err)
	}
	if plan.Tier != releaseTierFull || !plan.OLP || plan.Config {
		t.Fatalf("plan %+v", plan)
	}
	for _, want := range []string{"rt-core/adapters/rosie/control/maintenance.go", "rt-core/cmd/rt-control/main.go"} {
		if !releaseHas(plan.Runtime, want) {
			t.Errorf("runtime paths lack %s: %v", want, plan.Runtime)
		}
	}
	for _, p := range plan.Runtime {
		if strings.HasSuffix(p, ".md") || strings.HasSuffix(p, "_test.go") || strings.HasPrefix(p, "offline-programming/") || strings.HasPrefix(p, "steamdeck/") {
			t.Errorf("%s does not need the servo host's package", p)
		}
	}
	if releaseDefaultTestLevel(plan.Tier) != "full" {
		t.Error("a full release defaults to the full gate")
	}
}

// The 9a48c367 OLP-only install (2026-10-07 16:50): the OLP tree changed since
// d7a81080, the pin and the capabilities digest did not. Forcing olp is
// allowed and names the rt-core changes it leaves undeployed.
func TestReleaseTierOLPForcedKeepsRuntimeChangesVisible(t *testing.T) {
	in := releaseInstalled{RTCoreCommit: releaseFd9af899, OLPCommit: releaseD7a81080, Configuration: releasePin37, Capabilities: releaseCaps114}
	target := releaseTarget{Commit: release9a48c367, Configuration: releasePin37, Capabilities: releaseCaps114,
		RTChanges: releaseGolden(t, "fd9af899..9a48c367"), OLPChanges: releaseGolden(t, "d7a81080..9a48c367"), RTKnown: true, OLPKnown: true}
	auto, err := releaseChoose(in, target, "")
	if err != nil || auto.Tier != releaseTierFull {
		t.Fatalf("auto: %v %+v", err, auto)
	}
	plan, err := releaseChoose(in, target, "olp")
	if err != nil {
		t.Fatal(err)
	}
	if plan.Tier != releaseTierOLP || !plan.OLP || !releaseHas(plan.Notes, "stay undeployed on the servo host") {
		t.Fatalf("plan %+v", plan)
	}
	if !releaseHas(plan.OLPPaths, "offline-programming/v1/internal/") || releaseHas(plan.OLPPaths, "offline-programming/v1/ui/") {
		t.Fatalf("OLP paths %v", plan.OLPPaths)
	}
	if !releaseHas(plan.Notes, "browser UI changed") {
		t.Errorf("UI changes must be reported for operators: %v", plan.Notes)
	}
	if !plan.Pendant || !releaseHas(plan.PendantReasons, "steamdeck/real/v5/") {
		t.Errorf("pendant sources changed in this range: %+v", plan.PendantReasons)
	}
	if releaseDefaultTestLevel(plan.Tier) != "fast" {
		t.Error("an OLP release defaults to fast tests")
	}
}

// 6c6f97c0 (speed feedforward) changed the machine config, the pin AND rtctl's
// render.go: the installed rtctl would not understand it, so full; config is
// refused when forced.
func TestReleaseTierMachineConfigWithCompilerChangeIsFull(t *testing.T) {
	paths := releaseGolden(t, "cd8c5b4c..6c6f97c0")
	in := releaseInstalled{RTCoreCommit: releaseCd8c5b4c, OLPCommit: releaseCd8c5b4c, Configuration: "6885ee6be8cdd755b93960a5031d82a87b05d4352cf6dbf54185b88e9c0eb88d", Capabilities: releaseCaps114}
	target := releaseTarget{Commit: release6c6f97c0, Configuration: "41fc1be3960828a2feecb89f174084027fcda8f5060540f8352a06f7ab3e230a", Capabilities: releaseCaps114,
		RTChanges: paths, OLPChanges: paths, RTKnown: true, OLPKnown: true}
	plan, err := releaseChoose(in, target, "auto")
	if err != nil {
		t.Fatal(err)
	}
	if plan.Tier != releaseTierFull || !plan.Config || !releaseHas(plan.Runtime, "rt-core/tools/rtctl/render.go") {
		t.Fatalf("plan %+v", plan)
	}
	if releaseHas(plan.Runtime, "rt-core/config/machines/") || releaseHas(plan.Runtime, "rt-core/config/templates/") {
		t.Fatalf("machine configs are judged by the pin, not by path: %v", plan.Runtime)
	}
	if _, err := releaseChoose(in, target, "config"); err == nil || !strings.Contains(err.Error(), "cannot carry") {
		t.Fatalf("--tier config: %v", err)
	}
}

// Synthetic: a value change in the cell's machine JSON with its re-pin, and
// docs. No commit in the 1430 cell's history is config-only yet (every one so far came
// with an rtctl or native change).
func TestReleaseTierConfigOnly(t *testing.T) {
	in := releaseInstalled{RTCoreCommit: releaseFd9af899, OLPCommit: releaseFd9af899, Configuration: releasePin37, Capabilities: releaseCaps114}
	target := releaseTarget{Commit: strings.Repeat("c", 40), Configuration: strings.Repeat("e", 64), Capabilities: releaseCaps114, RTKnown: true, OLPKnown: true,
		RTChanges: []string{"rt-core/config/cells/example-arm-1420.json", "rt-core/config/machines/example-arm-1430.json", "rt-core/docs/drives/a6ec/README.md", ".cursor/memory/DEVLOG.md"}}
	target.OLPChanges = target.RTChanges
	plan, err := releaseChoose(in, target, "auto")
	if err != nil {
		t.Fatal(err)
	}
	if plan.Tier != releaseTierConfig || !plan.Config || plan.OLP || plan.Pendant || len(plan.Runtime) != 0 {
		t.Fatalf("plan %+v", plan)
	}
	if _, err := releaseChoose(in, target, "olp"); err == nil || !strings.Contains(err.Error(), "Use --tier auto") {
		t.Fatalf("--tier olp with a new pin: %v", err)
	}
	// The same commit once the cell runs its pin: nothing to do.
	in.Configuration = target.Configuration
	plan, err = releaseChoose(in, target, "auto")
	if err != nil || plan.Tier != releaseTierNone || plan.OLP || !releaseHas(plan.Reasons, "nothing the cell runs changed") {
		t.Fatalf("applied: %v %+v", err, plan)
	}
	if _, err := releaseChoose(in, target, "config"); err == nil {
		t.Fatal("--tier config with nothing to apply must refuse")
	}
}

func TestReleaseTierUIOnlyFlagsThePendantAndDeploysNothing(t *testing.T) {
	in := releaseInstalled{RTCoreCommit: releaseFd9af899, OLPCommit: release9a48c367, Configuration: releasePin37, Capabilities: releaseCaps114}
	target := releaseTarget{Commit: strings.Repeat("d", 40), Configuration: releasePin37, Capabilities: releaseCaps114, RTKnown: true, OLPKnown: true,
		OLPChanges: []string{"offline-programming/v1/ui/src/machine/jog.ts", "offline-programming/v1/ui/src/machine/jog.test.ts"}}
	plan, err := releaseChoose(in, target, "auto")
	if err != nil {
		t.Fatal(err)
	}
	if plan.Tier != releaseTierNone || plan.OLP || !plan.Pendant || len(plan.PendantReasons) != 1 || !releaseHas(plan.Notes, "operators run it from their own checkout") {
		t.Fatalf("plan %+v", plan)
	}
}

func TestReleaseTierCapabilitiesChangeAndUnknownInstalledCommits(t *testing.T) {
	in := releaseInstalled{RTCoreCommit: releaseFd9af899, OLPCommit: release9a48c367, Configuration: releasePin37, Capabilities: strings.Repeat("2", 64)}
	target := releaseTarget{Commit: strings.Repeat("d", 40), Configuration: releasePin37, Capabilities: releaseCaps114, RTKnown: true, OLPKnown: true}
	plan, err := releaseChoose(in, target, "auto")
	if err != nil || plan.Tier != releaseTierFull || !plan.Pendant || !strings.Contains(plan.PendantReasons[0], "capabilities digest") {
		t.Fatalf("capabilities: %v %+v", err, plan)
	}
	if _, err := releaseChoose(in, target, "olp"); err == nil || !strings.Contains(err.Error(), "Use --tier full") {
		t.Fatalf("--tier olp across a digest change: %v", err)
	}
	in.Capabilities = releaseCaps114
	target.RTKnown, target.OLPKnown = false, false
	plan, err = releaseChoose(in, target, "auto")
	if err != nil || plan.Tier != releaseTierFull || !plan.OLP || !releaseHas(plan.Runtime, "not in this checkout") {
		t.Fatalf("unknown installed commits: %v %+v", err, plan)
	}
	if _, err := releaseChoose(in, target, "fastest"); err == nil {
		t.Fatal("an unknown --tier must refuse")
	}
}
