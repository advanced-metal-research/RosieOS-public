package main

import (
	"encoding/json"
	"fmt"
	"regexp"
	"sort"
	"strings"
)

// Release tier selection for `realtime-core release`. The tier is the
// smallest one that carries every change between what the cell runs and the
// commit being released (steamdeck/real/v4/README.md, "Deployment tiers"):
//
//   - full: anything the servo host's package carries changed since the
//     installed rt-core release (native code, protocol, rtctl, drive
//     profiles, robot descriptions, host scripts);
//   - config: only the cell's machine configuration changed, so the commit's
//     cell pin differs from the running Describe; `realtime-core config`
//     then proves that the installed rtctl understands every key (it refuses
//     deployment-identity drift) before anything stops;
//   - olp: only the relay's OLP tree changed (server, resolver, seam worker)
//     and the commit's capabilities digest equals the running one;
//   - none: nothing the cell runs changed.
//
// A full release also installs the OLP (its catalogue pins change). The Steam
// Deck pendant is never rebuilt by a release; it is flagged when its own
// sources, the bundled OLP program logic, the meshes or the capabilities
// digest change.

type releaseTier string

const (
	releaseTierNone   releaseTier = "none"
	releaseTierConfig releaseTier = "config"
	releaseTierOLP    releaseTier = "olp"
	releaseTierFull   releaseTier = "full"
)

var releaseTierRank = map[releaseTier]int{releaseTierNone: 0, releaseTierOLP: 1, releaseTierConfig: 2, releaseTierFull: 3}

// releaseInstalled is what the cell runs, read by the release probe.
type releaseInstalled struct {
	RTCoreCommit  string // basename of /opt/rosie-rt-core/current: the package's git sha
	OLPCommit     string // HEAD of the OLP unit's tree
	Configuration string // live Describe configuration_sha256
	Capabilities  string // live Describe capabilities_digest
	Protocol      string
	Axes          int // Describe axes before the release; needed to verify a paired rollback
}

// releaseTarget is what the commit being released carries.
type releaseTarget struct {
	Commit        string
	Configuration string // the cell manifest's rt_core.configuration_sha256 at the commit
	Capabilities  string // CapabilitiesDigest in rt-core's generated control API at the commit
	// RTChanges and OLPChanges are `git diff --name-only` from the installed
	// rt-core and OLP commits; nil with Known false when that commit is not in
	// this checkout.
	RTChanges, OLPChanges []string
	RTKnown, OLPKnown     bool
	// Shipped is the import graph of the deployed Go binaries (release_inputs.go);
	// nil keeps the conservative prefix rule, ShippedWhy says why.
	Shipped    *releaseShipped
	ShippedWhy string
	// Surface: quality/deployment-surface.mjs over the OLP range (advisory).
	Surface []string
}

// runtimePath: the servo host's package carries p. With the import graph, an
// rt-core source no shipped binary is built from (an off-cell tool) is not.
func (t releaseTarget) runtimePath(p string) bool {
	return releaseRuntimePath(p) && (t.Shipped == nil || t.Shipped.servo(p))
}

// olpPath: the relay's OLP tree carries p; with the import graph, an rt-core
// Go file counts exactly when the OLP server imports its package (it imports
// rt-core/tools/rtctl, which the prefix rule leaves out).
func (t releaseTarget) olpPath(p string) bool {
	if t.Shipped != nil && strings.HasPrefix(p, "rt-core/") {
		return !releaseDocPath(p) && !releaseTestPath(p) && t.Shipped.olp(p)
	}
	return releaseOLPPath(p)
}

type releasePlan struct {
	Tier releaseTier
	// Config: the machine configuration changes (applied by realtime-core
	// config in the config tier, by the package's control.env in full).
	Config bool
	// OLP: the relay's OLP tree is rebuilt and installed.
	OLP bool
	// Pendant: a Steam Deck v5 rebuild is required (never done by release).
	Pendant        bool
	Reasons        []string // why the tier (one line per class of change)
	PendantReasons []string
	Notes          []string // informational: UI, local-OLP Decks, undeployed changes
	Runtime        []string // the changed paths that need the servo host's package
	OLPPaths       []string // the changed paths that need the OLP tree
}

// releaseDocPath is documentation, memory and agent notes: never deployed.
func releaseDocPath(p string) bool {
	switch {
	case strings.HasSuffix(p, ".md"), strings.HasSuffix(p, ".txt") && strings.Contains(p, "/docs/"),
		strings.HasPrefix(p, ".cursor/"), strings.HasPrefix(p, ".claude/"), strings.HasPrefix(p, ".github/"),
		strings.Contains(p, "/docs/"), strings.HasPrefix(p, "docs/"), strings.Contains(p, "/testdata/"),
		strings.HasSuffix(p, "/LICENSE"), p == "LICENSE", strings.HasSuffix(p, ".html") && strings.Contains(p, "/docs"):
		return true
	}
	return false
}

// releaseTestPath is a test source: it selects test suites but ships nothing.
func releaseTestPath(p string) bool {
	return strings.HasSuffix(p, "_test.go") || strings.Contains(p, "/tests/") || strings.Contains(p, ".test.") ||
		strings.HasSuffix(p, "_test.py") || strings.HasPrefix(p, "rt-core/simfixture/")
}

// releaseRuntimePath: carried by the servo host's rt-core package, or
// compiled into a binary it carries. Other cells' and template configs are
// not this cell's runtime; this cell's machine config and manifest are judged
// by the pin, not by path.
func releaseRuntimePath(p string) bool {
	if releaseDocPath(p) || releaseTestPath(p) {
		return false
	}
	if strings.HasPrefix(p, "robot_description/") {
		return true
	}
	if !strings.HasPrefix(p, "rt-core/") {
		return false
	}
	for _, prefix := range []string{"rt-core/config/cells/", "rt-core/config/machines/", "rt-core/config/templates/"} {
		if strings.HasPrefix(p, prefix) {
			return false
		}
	}
	return true
}

// releaseOLPPath: the relay's OLP tree (server binary, Cartesian resolver,
// seam worker sources, the CadQuery part-import scripts and their env, the
// models it reads). The browser UI is served from the operator's own
// checkout, not the relay (the relay serves an API-only stub), so ui/ is not
// here, nor cadquery/v1's own development UI.
func releaseOLPPath(p string) bool {
	if releaseDocPath(p) || releaseTestPath(p) {
		return false
	}
	switch {
	case strings.HasPrefix(p, "offline-programming/v1/ui/"), strings.HasPrefix(p, "cadquery/v1/ui/"):
		return false
	case strings.HasPrefix(p, "offline-programming/v1/"), strings.HasPrefix(p, "motion-server/v1/"),
		strings.HasPrefix(p, "weld_planner/v1/"), strings.HasPrefix(p, "robot_description/"), strings.HasPrefix(p, "cadquery/v1/"):
		return true
	case strings.HasPrefix(p, "rt-core/") && strings.HasSuffix(p, ".go"):
		// The OLP module imports rosieos/rt-core (go.mod replace).
		return !strings.HasPrefix(p, "rt-core/tools/")
	}
	return false
}

// releasePendantPath: the Steam Deck v5 package's own inputs. The bundle
// (olp-core.js) is built from every non-test .ts under OLP's ui/src.
func releasePendantPath(p string) bool {
	if releaseDocPath(p) || releaseTestPath(p) {
		return false
	}
	switch {
	case strings.HasPrefix(p, "steamdeck/real/v5/"):
		return true
	case strings.HasPrefix(p, "offline-programming/v1/ui/src/") && strings.HasSuffix(p, ".ts"):
		return true
	case strings.HasPrefix(p, "robot_description/robots/") && (strings.Contains(p, "/meshes/") || strings.HasSuffix(p, ".urdf")):
		return true
	}
	return false
}

func releaseSummarizePaths(paths []string, limit int) string {
	if len(paths) <= limit {
		return strings.Join(paths, ", ")
	}
	return strings.Join(paths[:limit], ", ") + fmt.Sprintf(" and %d more", len(paths)-limit)
}

// releaseChoose picks the tier. forced is "" or "auto" for automatic, else a
// tier name; a forced tier smaller than the changes need is refused unless it
// cannot hurt the cell (olp with an unchanged pin and capabilities digest).
func releaseChoose(in releaseInstalled, t releaseTarget, forced string) (releasePlan, error) {
	var plan releasePlan
	plan.Tier = releaseTierNone
	if !t.RTKnown {
		plan.Runtime = []string{"(installed rt-core release " + firstNonEmpty(in.RTCoreCommit, "unknown") + " is not in this checkout)"}
	} else {
		var unshipped []string
		for _, p := range t.RTChanges {
			if t.runtimePath(p) {
				plan.Runtime = append(plan.Runtime, p)
			} else if releaseRuntimePath(p) {
				unshipped = append(unshipped, p)
			}
		}
		if len(unshipped) > 0 {
			plan.Notes = append(plan.Notes, fmt.Sprintf("%d changed rt-core path(s) are in no deployed binary or file (go list -deps of the shipped mains, package-runtime.sh): %s", len(unshipped), releaseSummarizePaths(unshipped, 4)))
		}
		if t.Shipped == nil && t.ShippedWhy != "" {
			plan.Notes = append(plan.Notes, t.ShippedWhy)
		}
	}
	if !t.OLPKnown {
		plan.OLPPaths = []string{"(installed OLP tree " + firstNonEmpty(in.OLPCommit, "unknown") + " is not in this checkout)"}
	} else {
		var ui []string
		for _, p := range t.OLPChanges {
			if t.olpPath(p) {
				plan.OLPPaths = append(plan.OLPPaths, p)
			}
			if strings.HasPrefix(p, "offline-programming/v1/ui/") && !releaseTestPath(p) && !releaseDocPath(p) {
				ui = append(ui, p)
			}
		}
		if len(ui) > 0 {
			plan.Notes = append(plan.Notes, fmt.Sprintf("browser UI changed (%s): operators run it from their own checkout (Vite :5189), so they pull %s; the relay serves an API-only stub", releaseSummarizePaths(ui, 3), cellDeployShort(t.Commit)))
		}
	}
	// The Decks' installed commit is not read: the pendant flag is advisory,
	// over everything changed since what the cell runs.
	seen := map[string]bool{}
	for _, p := range append(append([]string(nil), t.RTChanges...), t.OLPChanges...) {
		if releasePendantPath(p) && !seen[p] {
			seen[p] = true
			plan.PendantReasons = append(plan.PendantReasons, p)
		}
	}
	sort.Strings(plan.PendantReasons)
	plan.Config = in.Configuration != "" && t.Configuration != "" && in.Configuration != t.Configuration
	capsChanged := in.Capabilities != "" && t.Capabilities != "" && in.Capabilities != t.Capabilities
	if capsChanged {
		plan.PendantReasons = append([]string{fmt.Sprintf("capabilities digest %.12s -> %.12s (the bundled olp-core.js refuses any other)", in.Capabilities, t.Capabilities)}, plan.PendantReasons...)
	}
	if len(plan.PendantReasons) > 0 {
		plan.Pendant = true
	}
	if len(plan.OLPPaths) > 0 && t.OLPKnown {
		plan.Notes = append(plan.Notes, "a Deck running its own local OLP server (no pendant.env) also needs a rebuild for OLP server changes")
	}
	for _, line := range t.Surface {
		plan.Notes = append(plan.Notes, "deployment surface (advisory, quality/deployment-surface.mjs): "+line)
	}

	auto := releaseTierNone
	switch {
	case len(plan.Runtime) > 0 || capsChanged:
		auto = releaseTierFull
		if len(plan.Runtime) > 0 {
			plan.Reasons = append(plan.Reasons, fmt.Sprintf("full: %d path(s) the servo host's package carries changed since %s: %s", len(plan.Runtime), cellDeployShort(in.RTCoreCommit), releaseSummarizePaths(plan.Runtime, 6)))
		}
		if capsChanged {
			plan.Reasons = append(plan.Reasons, fmt.Sprintf("full: capabilities digest %.12s -> %.12s", in.Capabilities, t.Capabilities))
		}
	case plan.Config:
		auto = releaseTierConfig
		plan.Reasons = append(plan.Reasons, fmt.Sprintf("config: the cell pin at %s is %.12s, the cell runs %.12s; nothing the package carries changed since %s", cellDeployShort(t.Commit), t.Configuration, in.Configuration, cellDeployShort(in.RTCoreCommit)))
	case len(plan.OLPPaths) > 0:
		auto = releaseTierOLP
	}
	if len(plan.OLPPaths) > 0 {
		plan.OLP = true
		plan.Reasons = append(plan.Reasons, fmt.Sprintf("olp: %d path(s) of the relay's OLP tree changed since %s: %s", len(plan.OLPPaths), cellDeployShort(in.OLPCommit), releaseSummarizePaths(plan.OLPPaths, 6)))
	}
	if auto == releaseTierFull {
		plan.OLP = true // new catalogue pins
	}

	switch forced {
	case "", "auto":
		plan.Tier = auto
	case string(releaseTierFull):
		plan.Tier, plan.OLP = releaseTierFull, true
		if auto != releaseTierFull {
			plan.Notes = append(plan.Notes, fmt.Sprintf("--tier full forced; the changes need only %s", auto))
		}
	case string(releaseTierConfig):
		if auto == releaseTierFull {
			return plan, fmt.Errorf("--tier config cannot carry these changes:\n  - %s", strings.Join(plan.Reasons, "\n  - "))
		}
		if !plan.Config {
			return plan, fmt.Errorf("--tier config: the cell already runs the pin at %s (%.12s); nothing to apply", cellDeployShort(t.Commit), t.Configuration)
		}
		plan.Tier = releaseTierConfig
	case string(releaseTierOLP):
		if plan.Config {
			return plan, fmt.Errorf("--tier olp: the commit pins configuration %.12s but the cell runs %.12s; the OLP would carry a configuration the cell does not run. Use --tier auto (config first)", t.Configuration, in.Configuration)
		}
		if capsChanged {
			return plan, fmt.Errorf("--tier olp: capabilities digest %.12s at the commit, %.12s running: an OLP built for another protocol is refused by its own client. Use --tier full", t.Capabilities, in.Capabilities)
		}
		plan.Tier, plan.OLP = releaseTierOLP, true
		if len(plan.Runtime) > 0 {
			plan.Notes = append(plan.Notes, fmt.Sprintf("--tier olp forced: %d rt-core change(s) since %s stay undeployed on the servo host (%s)", len(plan.Runtime), cellDeployShort(in.RTCoreCommit), releaseSummarizePaths(plan.Runtime, 4)))
		}
	default:
		return plan, fmt.Errorf("--tier must be auto, config, olp or full")
	}
	if plan.Tier == releaseTierNone && !plan.OLP {
		plan.Reasons = append(plan.Reasons, fmt.Sprintf("none: nothing the cell runs changed (rt-core %s, OLP %s, pin %.12s)", cellDeployShort(in.RTCoreCommit), cellDeployShort(in.OLPCommit), in.Configuration))
	}
	return plan, nil
}

// releaseDefaultTestLevel: fast for config and OLP releases, full for a full
// package (README "Test levels").
func releaseDefaultTestLevel(tier releaseTier) string {
	if tier == releaseTierFull {
		return "full"
	}
	return "fast"
}

var releaseCapabilitiesLine = regexp.MustCompile(`CapabilitiesDigest\s*=\s*"([0-9a-f]{64})"`)

// releaseCapabilitiesAt reads the commit's capabilities digest from rt-core's
// generated control API (tools/apigen output).
func releaseCapabilitiesAt(repo, commit string) (string, error) {
	text, err := robotV4GitOutput(repo, "show", commit+":rt-core/adapters/rosie/control/api_generated.go")
	if err != nil {
		return "", fmt.Errorf("read the capabilities digest at %s: %w", cellDeployShort(commit), err)
	}
	m := releaseCapabilitiesLine.FindStringSubmatch(text)
	if m == nil {
		return "", fmt.Errorf("no CapabilitiesDigest in rt-core/adapters/rosie/control/api_generated.go at %s", cellDeployShort(commit))
	}
	return m[1], nil
}

// releasePinIn returns the configuration pin of the manifest's realtime-core
// node(s); exactly one distinct pin is required.
func releasePinIn(manifest []byte) (string, error) {
	var doc struct {
		Nodes []struct {
			RTCore *struct {
				ConfigurationSHA256 string `json:"configuration_sha256"`
			} `json:"rt_core"`
		} `json:"nodes"`
	}
	if err := json.Unmarshal(manifest, &doc); err != nil {
		return "", err
	}
	pins := map[string]bool{}
	for _, n := range doc.Nodes {
		if n.RTCore != nil && n.RTCore.ConfigurationSHA256 != "" {
			pins[n.RTCore.ConfigurationSHA256] = true
		}
	}
	if len(pins) != 1 {
		return "", fmt.Errorf("the manifest has %d distinct rt_core configuration pins, need exactly 1", len(pins))
	}
	for pin := range pins {
		return pin, nil
	}
	return "", nil
}

// releaseChangedPaths is `git diff --name-only from to`, sorted; known is
// false when from is not a commit of this checkout.
func releaseChangedPaths(repo, from, to string) ([]string, bool, error) {
	if !cellDeployHexCommit.MatchString(from) {
		return nil, false, nil
	}
	if _, err := robotV4GitOutput(repo, "cat-file", "-e", from+"^{commit}"); err != nil {
		return nil, false, nil
	}
	out, err := robotV4GitOutput(repo, "diff", "--name-only", from, to)
	if err != nil {
		return nil, false, err
	}
	paths := strings.Fields(out)
	sort.Strings(paths)
	return paths, true, nil
}

// releaseTargetAt reads everything the tier choice needs from this checkout.
func releaseTargetAt(repo, commit, manifestPath string, in releaseInstalled) (releaseTarget, error) {
	t := releaseTarget{Commit: commit}
	raw, err := robotV4GitOutput(repo, "show", commit+":"+manifestPath)
	if err != nil {
		return t, fmt.Errorf("%s is not in commit %s: %w", manifestPath, cellDeployShort(commit), err)
	}
	if t.Configuration, err = releasePinIn([]byte(raw)); err != nil {
		return t, fmt.Errorf("%s at %s: %w", manifestPath, cellDeployShort(commit), err)
	}
	if t.Capabilities, err = releaseCapabilitiesAt(repo, commit); err != nil {
		return t, err
	}
	if t.RTChanges, t.RTKnown, err = releaseChangedPaths(repo, in.RTCoreCommit, commit); err != nil {
		return t, err
	}
	if t.OLPChanges, t.OLPKnown, err = releaseChangedPaths(repo, in.OLPCommit, commit); err != nil {
		return t, err
	}
	t.Shipped, t.ShippedWhy = releaseShippedAt(repo, commit)
	t.Surface = releaseDeploymentSurface(repo, firstNonEmpty(in.RTCoreCommit, in.OLPCommit), commit)
	return t, nil
}
