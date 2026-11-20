package main

import (
	"bufio"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path"
	"sort"
	"strings"
)

// The fast gate chooses each suite against its own last pass on the cell,
// not against what the cell runs (owner-approved 2026-10-08). The OLP-tier
// release of bcd74a8e on a Rosie 1430 cell (a manifest one-liner) spent 13m19s in the
// fast gate: rt-core commits since the installed bc423a86 selected the
// rt-core and rtctl suites, although the previous gate (9dcf3158) had passed
// them and their inputs had not changed since.
//
// A suite's inputs at a commit are the (path, object) entries of the
// commit's tree that its Inputs select (gateSuite.selects). Fast runs a suite
// unless a pass recorded on this Mac for this cell, under the suite's current
// definition and with no failure today's known-failures list no longer
// excuses, saw exactly the inputs the commit has: compared through git, or
// through the recorded input hash when that commit is no longer in this
// checkout. A suite no recorded pass covers is skipped only when its inputs
// equal those at every commit the cell runs (--test-since; a release passes
// the installed rt-core and OLP commits): then nothing it covers ships. Any
// other suite runs, including one with no record or an unknown base. A suite
// whose inputs ship is therefore skipped only when a recorded pass covers
// the very inputs the release ships.
//
// A tier that leaves the servo host's package as installed (olp, config)
// ships no servo-only path (gateServoOnly: native sources, native tests,
// host scripts, protocol files). Those do not count for it: after the
// release they run on no host, and the full release that ships them runs the
// full gate. Every other changed input counts, so a changed rt-core Go
// package (the OLP server imports some), rt-core/config or the TypeScript
// client still selects its suites.

// gateTreeIndex is a commit's tree: path -> object id.
type gateTreeIndex map[string]string

// gateTreeIndexOf reads a commit's tree (git ls-tree -r).
var gateTreeIndexOf = func(repo, commit string) (gateTreeIndex, error) {
	out, err := exec.Command("git", "-C", repo, "ls-tree", "-r", "-z", "--full-tree", commit).Output()
	if err != nil {
		return nil, fmt.Errorf("git ls-tree %s: %w", cellDeployShort(commit), err)
	}
	idx := gateTreeIndex{}
	for _, entry := range strings.Split(string(out), "\x00") {
		meta, p, ok := strings.Cut(entry, "\t")
		if f := strings.Fields(meta); ok && len(f) == 3 {
			idx[p] = f[2]
		}
	}
	if len(idx) == 0 {
		return nil, fmt.Errorf("git ls-tree %s: empty tree", cellDeployShort(commit))
	}
	return idx, nil
}

// goDirs are the directories that hold Go sources: any file there may be
// embedded in a Go binary.
func (idx gateTreeIndex) goDirs() map[string]bool {
	dirs := map[string]bool{}
	for p := range idx {
		if strings.HasSuffix(p, ".go") {
			dirs[path.Dir(p)] = true
		}
	}
	return dirs
}

// gateServoOnly: p reaches only the servo host's package, so a release that
// leaves that package as installed ships nothing built from it. Any Go
// source, any file beside one, rt-core's go.mod/go.sum (the OLP module
// replaces rosieos/rt-core), rt-core/config (the OLP compiles the cell's
// configuration) and the TypeScript client are never servo-only.
func gateServoOnly(p string, goDirs map[string]bool) bool {
	if !strings.HasPrefix(p, "rt-core/") || strings.HasPrefix(p, "rt-core/config/") || strings.HasPrefix(p, "rt-core/clients/ts/") {
		return false
	}
	if strings.HasSuffix(p, ".go") || p == "rt-core/go.mod" || p == "rt-core/go.sum" || goDirs[path.Dir(p)] {
		return false
	}
	return true
}

// gateView is which inputs count: all of them, or for a release tier that
// keeps the installed servo package, all but the servo-only ones.
type gateView struct {
	Tier   string
	goDirs map[string]bool
}

func (v gateView) keepsServo() bool {
	return v.Tier == string(releaseTierOLP) || v.Tier == string(releaseTierConfig)
}

func (v gateView) counts(p string) bool {
	return !v.keepsServo() || !gateServoOnly(p, v.goDirs)
}

// gateSuiteInputs are the entries of idx the suite's inputs select and the
// view counts.
func gateSuiteInputs(idx gateTreeIndex, s gateSuite, v gateView) map[string]string {
	out := map[string]string{}
	for p, obj := range idx {
		if s.selects(p) && v.counts(p) {
			out[p] = obj
		}
	}
	return out
}

func gateInputHash(entries map[string]string) string {
	paths := make([]string, 0, len(entries))
	for p := range entries {
		paths = append(paths, p)
	}
	sort.Strings(paths)
	h := sha256.New()
	for _, p := range paths {
		fmt.Fprintf(h, "%s\x00%s\n", p, entries[p])
	}
	return hex.EncodeToString(h.Sum(nil))
}

// gateInputDiff is the paths added, removed or changed between two input sets.
func gateInputDiff(from, to map[string]string) []string {
	var out []string
	for p, obj := range to {
		if from[p] != obj {
			out = append(out, p)
		}
	}
	for p := range from {
		if _, ok := to[p]; !ok {
			out = append(out, p)
		}
	}
	sort.Strings(out)
	return out
}

// gateSuiteInputHashes is what a passed gate records: each suite's input
// hash at the commit, every input counted.
func gateSuiteInputHashes(repo, commit string, sel gateSelection) map[string]string {
	idx, err := gateTreeIndexOf(repo, commit)
	if err != nil {
		return nil
	}
	out := map[string]string{}
	for _, s := range sel.Suites {
		out[s.Name] = gateInputHash(gateSuiteInputs(idx, s, gateView{}))
	}
	return out
}

// gateLoadPasses returns the cell's recorded passes, oldest first.
func gateLoadPasses(cell string) []gatePassRecord {
	file, err := gatePassFile()
	if err != nil {
		return nil
	}
	f, err := os.Open(file)
	if err != nil {
		return nil
	}
	defer f.Close()
	var out []gatePassRecord
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 0, 1<<20), 16<<20)
	for sc.Scan() {
		var rec gatePassRecord
		if json.Unmarshal(sc.Bytes(), &rec) == nil && rec.Cell == cell {
			out = append(out, rec)
		}
	}
	return out
}

// gateRecordPassed: the record ran the suite under its current definition and
// today's known-failures list still excuses every failure it saw.
func gateRecordPassed(rec gatePassRecord, s gateSuite, known map[string]map[string]string) bool {
	if !containsString(rec.Suites, s.Name) {
		return false
	}
	if d, ok := rec.SuiteDefinitions[s.Name]; ok {
		if d != gateSuiteDefinitionDigest(s) {
			return false
		}
	} else if rec.Gate != gateDefinitionDigest() {
		return false
	}
	var steps []cellDeployGateStep
	for _, st := range rec.Steps {
		if st.Step == s.Name {
			steps = append(steps, st)
		}
	}
	if rec.Partial {
		// Partial records have stronger evidence than legacy whole-gate records:
		// no implicit success from an absent step or an excused failure.
		return rec.SuiteDefinitions[s.Name] != "" && rec.SuiteInputs[s.Name] != "" &&
			len(steps) == 1 && steps[0].RC == 0 && steps[0].Failures != nil && len(steps[0].Failures) == 0
	}
	_, _, newFails := cellDeployClassifyGate(steps, known)
	return len(newFails) == 0
}

// gateHistory is what fast selection against the records needs.
type gateHistory struct {
	Repo, Cell, Commit string
	// Tier is the release tier ("" for a plain build): olp and config leave
	// servo-only inputs out.
	Tier string
	// Since are the commits the cell runs, for the plan's account of what a
	// skipped suite's covering pass shipped; empty when unknown.
	Since []string
	Known map[string]map[string]string
}

// gateSelectSincePasses is the fast level: each suite whose counted inputs
// differ from every recorded pass of it on the cell.
func gateSelectSincePasses(h gateHistory, seconds map[string]float64) (gateSelection, error) {
	target, err := gateTreeIndexOf(h.Repo, h.Commit)
	if err != nil {
		sel, serr := gateSelect("fast", nil, false, seconds)
		for name := range sel.Why {
			sel.Why[name] = fmt.Sprintf("the commit's tree is unreadable (%v): every suite", err)
		}
		return sel, serr
	}
	sel := gateSelection{Level: "fast", Why: map[string]string{}, SkipWhy: map[string]string{}}
	v := gateView{Tier: h.Tier, goDirs: target.goDirs()}
	trees := map[string]gateTreeIndex{h.Commit: target}
	tree := func(c string) gateTreeIndex {
		if idx, ok := trees[c]; ok {
			return idx
		}
		var idx gateTreeIndex
		if cellDeployHexCommit.MatchString(c) {
			idx, _ = gateTreeIndexOf(h.Repo, c)
		}
		trees[c] = idx
		return idx
	}
	records := gateLoadPasses(h.Cell)
	for _, s := range gateSuites {
		want := gateSuiteInputs(target, s, v)
		wantHash := gateInputHash(want)
		fullHash := gateInputHash(gateSuiteInputs(target, s, gateView{}))
		var passes []gatePassRecord
		for _, r := range records {
			if gateRecordPassed(r, s, h.Known) {
				passes = append(passes, r)
			}
		}
		var cover *gatePassRecord
		var coverIdx gateTreeIndex
		for i := len(passes) - 1; i >= 0 && cover == nil; i-- {
			if idx := tree(passes[i].Commit); idx != nil {
				if gateInputHash(gateSuiteInputs(idx, s, v)) == wantHash {
					cover, coverIdx = &passes[i], idx
				}
			} else if passes[i].SuiteInputs[s.Name] == fullHash {
				cover = &passes[i]
			}
		}
		if cover != nil {
			why := fmt.Sprintf("inputs unchanged since its pass at %s (%s, %s gate)", cellDeployShort(cover.Commit), cover.At, cover.Level)
			if coverIdx != nil && v.keepsServo() {
				if ex := gateInputDiff(gateSuiteInputs(coverIdx, s, gateView{}), gateSuiteInputs(target, s, gateView{})); len(ex) > 0 {
					why += fmt.Sprintf("; %d servo-only path(s) changed since, which the %s tier does not ship: %s", len(ex), v.Tier, releaseSummarizePaths(ex, 2))
				}
			}
			if shipped, from, _ := gateDifferFromInstalled(h, tree, s, v, want); len(shipped) > 0 {
				why += fmt.Sprintf("; %d of its inputs differ from what the cell runs (%s) and that pass covered exactly them", len(shipped), strings.Join(from, "/"))
			}
			sel.Skipped = append(sel.Skipped, s.Name)
			sel.SkipWhy[s.Name] = why
			continue
		}
		if shipped, from, known := gateDifferFromInstalled(h, tree, s, v, want); known && len(shipped) == 0 {
			sel.Skipped = append(sel.Skipped, s.Name)
			sel.SkipWhy[s.Name] = fmt.Sprintf("no recorded pass covers its inputs, but they equal those at %s (what the cell runs): nothing it covers ships", strings.Join(from, "/"))
			continue
		}
		if len(passes) == 0 {
			sel.Why[s.Name] = "no recorded pass of this suite on " + h.Cell + " under its current definition, and its inputs differ from what the cell runs (or that is unknown)"
			sel.Suites = append(sel.Suites, s)
			continue
		}
		last := passes[len(passes)-1]
		if idx := tree(last.Commit); idx == nil {
			sel.Why[s.Name] = fmt.Sprintf("its last pass (%s at %s) is not a commit of this checkout and no recorded input hash matches", cellDeployShort(last.Commit), last.At)
		} else {
			changed := gateInputDiff(gateSuiteInputs(idx, s, v), want)
			sel.Why[s.Name] = fmt.Sprintf("%d input path(s) changed since its last pass at %s (%s): %s", len(changed), cellDeployShort(last.Commit), last.At, releaseSummarizePaths(changed, 2))
		}
		sel.Suites = append(sel.Suites, s)
	}
	gateOrder(&sel, seconds)
	return sel, nil
}

// gateDifferFromInstalled lists the suite's counted inputs that differ
// between what the cell runs and the commit; known is false when what the
// cell runs is not given or one of its commits is not in this checkout.
func gateDifferFromInstalled(h gateHistory, tree func(string) gateTreeIndex, s gateSuite, v gateView, want map[string]string) (paths, from []string, known bool) {
	seen := map[string]bool{}
	known = len(h.Since) > 0
	for _, c := range h.Since {
		idx := tree(c)
		if idx == nil {
			known = false
			continue
		}
		from = append(from, cellDeployShort(c))
		for _, p := range gateInputDiff(gateSuiteInputs(idx, s, v), want) {
			if !seen[p] {
				seen[p] = true
				paths = append(paths, p)
			}
		}
	}
	return paths, from, known
}

// gateFastSelection is the fast level for build and release --plan: against
// the records, or with --regate against the changes since the given commits.
func gateFastSelection(h gateHistory, regate bool, seconds map[string]float64) (gateSelection, error) {
	if !regate {
		return gateSelectSincePasses(h, seconds)
	}
	changed, known := []string(nil), len(h.Since) > 0
	for _, b := range h.Since {
		paths, ok, err := releaseChangedPaths(h.Repo, b, h.Commit)
		if err != nil {
			return gateSelection{}, err
		}
		known = known && ok
		changed = append(changed, paths...)
	}
	return gateSelect("fast", changed, known, seconds)
}
