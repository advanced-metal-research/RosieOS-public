package main

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"time"
)

// The maintenance notice's ETA for a release is what this Mac's journal says
// the release's steps take on the cell, not a fixed number per tier. The
// fixed 75 min of a full release (the owner's 45 min deploy-session estimate
// plus 30 min of build) showed about 1 h 10 min left on a Rosie 1430 cell's overlays on
// 2026-10-10, while that release's build and gate took 18m47s and the whole
// full release of 2026-10-08 took 24m44s, a 9 min lock wait included.
//
// A step's estimate is the median of the cell's newest releaseETASamples runs
// of that step that the release got past. build+test is the build (the median
// build part of the cell's builds that compiled) plus the gate the release
// will run: the selected suites' lane plan (gateLanePlan on the journal's
// suite means) times the median ratio of measured gate wall time to that plan
// over the cell's recorded gates, or nothing when a recorded pass covers it.
// The notice shows the sum plus releaseETAMargin from when it goes up (the
// lock and idle waits come before), and the release re-estimates it after
// every step from the steps still to run. The watchdogs (the gate's stall
// limit, start's ready wait) are ceilings, not estimates, and stay separate.

// releaseETASamples is how many of the newest runs a median takes.
const releaseETASamples = 5

// releaseETAMargin is added to the summed estimates: 20 %, at least a
// minute, the total rounded up to a whole minute. Chosen, unverified.
const releaseETAMargin = 0.2

// releaseStepDefaults are a step's estimate when this Mac has no run of it on
// the cell. Unverified and chosen long: one cell's measured runs (2026-10-08 and
// -11 releases) were copy-dumps 3.6-4.8 s with nothing to copy and 123.5 s
// for five stop dumps, precheck 1.6-2.0 s, deploy 35.7 s, start 22.4 s,
// verify 7.8 s, olp install 5.6-7.2 s and prune 1.3 s; config ~35 s. With no
// history at all a full release's notice shows 1h50m (TestReleaseETADefaults).
var releaseStepDefaults = map[string]time.Duration{
	"copy-dumps":  5 * time.Minute,
	"precheck":    2 * time.Minute,
	"deploy":      10 * time.Minute,
	"start":       rtCoreMaintenanceStartEstimate,
	"verify":      2 * time.Minute,
	"olp install": cellDeployOLPEstimate,
	"prune":       2 * time.Minute,
	"config":      5 * time.Minute,
	"rollback":    10 * time.Minute,
	// The 2026-10-11 hand restarts: stop 2.5 s, start 21.6-21.9 s, copy-dumps
	// 4.1 s with nothing new, verify ~8 s (DEVLOG 14:50, 20:47); the command
	// itself is untimed.
	"restart": 5 * time.Minute,
}

// releaseGateDefaultETA is a gate's estimate with no gate of the cell
// recorded: the build-and-gate default less the build's (cell_lock.go),
// unverified.
const releaseGateDefaultETA = cellDeployBuildTestDefaultETA - cellDeployBuildDefaultETA

// releaseStepNames are the steps a release runs (releaseSteps). A step whose
// span failed still counts when the release went on into a later one of
// these: deploy's health gate failing only on the owner-accepted igh_master
// is every release's deploy on a Rosie 1430 cell until its servo host's IgH
// module is fixed.
var releaseStepNames = []string{"build+test", "copy-dumps", "precheck", "deploy", "start", "verify", "olp install", "prune", "config"}

// releaseStandaloneSteps are the commands that run alone as one release step:
// their successful runs (machine time, the idle wait excluded) count for it.
var releaseStandaloneSteps = map[string]string{
	"realtime-core copy-dumps": "copy-dumps", "realtime-core precheck": "precheck", "realtime-core deploy": "deploy",
	"realtime-core start": "start", "realtime-core verify": "verify", "olp install": "olp install",
	"realtime-core prune": "prune", "realtime-core config": "config", "realtime-core restart": "restart",
}

var releaseGateLanesMark = regexp.MustCompile(` on ([0-9]+) lane\(s\)`)

// releaseGateRun is one recorded gate: its wall time and what ran.
type releaseGateRun struct {
	Wall   float64
	Suites []string
	Lanes  int
}

// releaseHistory is a cell's timings from this Mac's journal, oldest first.
type releaseHistory struct {
	Steps  map[string][]float64 // seconds per release step
	Builds []float64            // the build part of builds that compiled
	Gates  []releaseGateRun
	Means  map[string]float64 // per suite, over the whole journal (gateMeanSeconds)
}

// releaseHistoryOf reads the cell's release and build records: release
// steps from the release's top-level spans; the build part (machine time
// less the gate) of builds whose build span ran and passed; and every gate
// that ran to its end (a suite failing ends it "ok" with a gate_failed run:
// its duration is still the gate's).
func releaseHistoryOf(recs []deployTimingRecord, cell string) releaseHistory {
	h := releaseHistory{Steps: map[string][]float64{}, Means: gateMeanSeconds(recs)}
	// A step run inside a release journals itself too; only the release's
	// span counts for it, so a standalone record counts only outside every
	// release of the cell.
	var releases [][2]time.Time
	for _, rec := range recs {
		if !rec.Legacy && rec.Cell == cell && rec.Command == "realtime-core release" {
			releases = append(releases, [2]time.Time{rec.Start, rec.End})
		}
	}
	insideRelease := func(at time.Time) bool {
		for _, w := range releases {
			if !at.Before(w[0]) && !at.After(w[1]) {
				return true
			}
		}
		return false
	}
	for _, rec := range recs {
		if rec.Legacy || rec.Cell != cell {
			continue
		}
		if step, ok := releaseStandaloneSteps[rec.Command]; ok {
			if rec.Result == "ok" && rec.MachineSeconds > 0 && !insideRelease(rec.Start) {
				h.Steps[step] = append(h.Steps[step], rec.MachineSeconds)
			}
			continue
		}
		switch rec.Command {
		case "realtime-core release":
			var top []deployTimingSpan
			for _, s := range rec.Spans {
				if !strings.Contains(s.Name, "/") {
					top = append(top, s)
				}
			}
			for i, s := range top {
				if s.Seconds <= 0 || !containsString(releaseStepNames, s.Name) {
					continue
				}
				usable := s.Result == "ok"
				for _, later := range top[i+1:] {
					if later.Name == s.Name+" retry" {
						usable = s.Result == "ok"
						break
					}
					usable = usable || containsString(releaseStepNames, later.Name)
				}
				if usable {
					h.Steps[s.Name] = append(h.Steps[s.Name], s.Seconds)
				}
			}
		case "realtime-core build":
			if rec.Result != "ok" && rec.Result != "gate_failed" {
				continue
			}
			built, gate := false, releaseGateRun{Lanes: 3}
			for _, s := range rec.Spans {
				switch {
				case s.Name == "build":
					built = s.Result == "ok"
				case s.Name == "test gate" && s.Result == "ok":
					gate.Wall = s.Seconds
				case strings.HasPrefix(s.Name, "test gate/") && strings.Count(s.Name, "/") == 1:
					gate.Suites = append(gate.Suites, strings.TrimPrefix(s.Name, "test gate/"))
				}
			}
			if built {
				h.Builds = append(h.Builds, rec.MachineSeconds-gate.Wall)
			}
			if gate.Wall > 0 && len(gate.Suites) > 0 {
				for _, m := range rec.Marks {
					if g := releaseGateLanesMark.FindStringSubmatch(m.Detail); m.Phase == "gate" && g != nil {
						gate.Lanes, _ = strconv.Atoi(g[1])
						break
					}
				}
				h.Gates = append(h.Gates, gate)
			}
		}
	}
	return h
}

// releaseJournalHistory is releaseHistoryOf over ~/.rosie/deploy-journal.
func releaseJournalHistory(cell string) releaseHistory {
	var recs []deployTimingRecord
	if home, err := os.UserHomeDir(); err == nil {
		recs, _ = loadDeployTimings(filepath.Join(home, ".rosie", "deploy-journal"))
	}
	return releaseHistoryOf(recs, cell)
}

// releaseMedian is the median of the newest releaseETASamples values.
func releaseMedian(values []float64) (float64, int) {
	if len(values) > releaseETASamples {
		values = values[len(values)-releaseETASamples:]
	}
	s := append([]float64(nil), values...)
	sort.Float64s(s)
	switch n := len(s); {
	case n == 0:
		return 0, 0
	case n%2 == 1:
		return s[n/2], n
	default:
		return (s[n/2-1] + s[n/2]) / 2, n
	}
}

func releaseSeconds(s float64) time.Duration {
	return time.Duration(math.Round(s)) * time.Second
}

// releaseGatePlan is the gate the release's build will run.
type releaseGatePlan struct {
	Level  string
	Suites []string
	// Reused, when set, says why no suite runs: a recorded pass covers them.
	Reused string
}

// releaseLanePlan is gateLanePlan for the named suites in their start order
// with the journal's suite means.
func releaseLanePlan(names []string, means map[string]float64, lanes int) float64 {
	sel := gateSelection{Why: map[string]string{}}
	for _, name := range names {
		if s, ok := gateSuiteByName(name); ok {
			sel.Suites = append(sel.Suites, s)
			sel.Why[name] = "ran"
		}
	}
	gateOrder(&sel, means)
	return gateLanePlan(sel.Suites, lanes)
}

func (h releaseHistory) buildEstimate() (time.Duration, string) {
	if m, n := releaseMedian(h.Builds); n > 0 {
		return releaseSeconds(m), fmt.Sprintf("median of the last %d build(s)", n)
	}
	return cellDeployBuildDefaultETA, "default, unverified: no build of this cell recorded on this Mac"
}

func (h releaseHistory) gateEstimate(g releaseGatePlan) (time.Duration, string) {
	switch {
	case g.Reused != "":
		return 0, g.Reused
	case len(g.Suites) == 0:
		return 0, "no suite to run"
	}
	var ratios []float64
	lanes := 3
	for _, run := range h.Gates {
		if plan := releaseLanePlan(run.Suites, h.Means, run.Lanes); plan > 0 {
			ratios = append(ratios, run.Wall/plan)
			lanes = run.Lanes
		}
	}
	k, n := releaseMedian(ratios)
	if n == 0 {
		return releaseGateDefaultETA, "default, unverified: no gate of this cell recorded on this Mac"
	}
	plan := releaseLanePlan(g.Suites, h.Means, lanes)
	return releaseSeconds(k * plan), fmt.Sprintf("%d suite(s): lane plan %s on %d lanes x %.2f, the median measured/planned ratio of the last %d gate(s)",
		len(g.Suites), formatDeployDuration(plan), lanes, k, n)
}

// releaseStepETA is one step's estimate and where it came from.
type releaseStepETA struct {
	Name     string
	Expected time.Duration
	Basis    string
}

func (h releaseHistory) stepEstimate(name string, gate releaseGatePlan) releaseStepETA {
	if name == "build+test" {
		b, bwhy := h.buildEstimate()
		g, gwhy := h.gateEstimate(gate)
		return releaseStepETA{Name: name, Expected: b + g, Basis: fmt.Sprintf("build %s (%s) + %s gate %s (%s)",
			formatDeployDuration(b.Seconds()), bwhy, gate.Level, formatDeployDuration(g.Seconds()), gwhy)}
	}
	if m, n := releaseMedian(h.Steps[name]); n > 0 {
		return releaseStepETA{Name: name, Expected: releaseSeconds(m), Basis: fmt.Sprintf("median of the last %d run(s)", n)}
	}
	d, ok := releaseStepDefaults[name]
	if !ok {
		d = 10 * time.Minute
	}
	return releaseStepETA{Name: name, Expected: d, Basis: "default, unverified: no run of it on this cell recorded on this Mac"}
}

// releaseETA is a release's estimate, step by step in run order.
type releaseETA struct {
	Steps    []releaseStepETA
	Rollback releaseStepETA
	// History and Level re-estimate the gate once the build has chosen its
	// suites (notice_progress.go).
	History releaseHistory
	Level   string
}

func releaseEstimateSteps(h releaseHistory, steps []releaseStep, gate releaseGatePlan) releaseETA {
	e := releaseETA{Rollback: h.stepEstimate("rollback", gate), History: h, Level: gate.Level}
	for _, s := range steps {
		e.Steps = append(e.Steps, h.stepEstimate(s.Name, gate))
	}
	return e
}

// releaseETAWithMargin is the notice ETA for work estimated at sum.
func releaseETAWithMargin(sum time.Duration) time.Duration {
	if sum <= 0 {
		return 0
	}
	total := sum + max(time.Duration(float64(sum)*releaseETAMargin), time.Minute)
	return (total + time.Minute - 1) / time.Minute * time.Minute
}

func (e releaseETA) sum(from int) time.Duration {
	var sum time.Duration
	for _, s := range e.Steps[min(from, len(e.Steps)):] {
		sum += s.Expected
	}
	return sum
}

// remaining is the notice ETA with the steps from `from` on still to run.
func (e releaseETA) remaining(from int) time.Duration {
	return releaseETAWithMargin(e.sum(from))
}

// lines is the estimate as --plan prints it.
func (e releaseETA) lines() []string {
	out := []string{fmt.Sprintf("notice ETA %s from when the notice goes up: the steps' estimates (%s) + %.0f%% (at least 1 min), from this Mac's journal; re-estimated after every step",
		formatDeployDuration(e.remaining(0).Seconds()), formatDeployDuration(e.sum(0).Seconds()), releaseETAMargin*100)}
	for i, s := range e.Steps {
		out = append(out, fmt.Sprintf("%d. %s %s: %s", i+1, s.Name, formatDeployDuration(s.Expected.Seconds()), s.Basis))
	}
	return out
}

// releaseGateSelection is the gate the release's build will choose: fast
// from this machine's gate records (build chooses again after its probe),
// full every suite.
func releaseGateSelection(repo string, ctx cellDeployContext, commit string, tier releaseTier, test string, since []string, regate bool, means map[string]float64) (gateSelection, error) {
	if test == "full" {
		return gateSelect("full", nil, true, means)
	}
	raw, err := os.ReadFile(filepath.Join(repo, filepath.FromSlash(cellDeployKnownFailuresPath)))
	if err != nil {
		return gateSelection{}, fmt.Errorf("known failures: %w", err)
	}
	known, err := cellDeployKnownFailures(raw)
	if err != nil {
		return gateSelection{}, err
	}
	return gateFastSelection(gateHistory{Repo: repo, Cell: ctx.CellID, Commit: commit, Tier: string(tier), Since: since, Known: known}, regate, means)
}

// releaseGatePlanOf is the selection as the estimate needs it: no suite when
// a recorded pass of the commit covers it (build reuses it unless --regate).
func releaseGatePlanOf(repo string, ctx cellDeployContext, commit string, sel gateSelection, regate bool) releaseGatePlan {
	g := releaseGatePlan{Level: sel.Level}
	for _, s := range sel.Suites {
		g.Suites = append(g.Suites, s.Name)
	}
	if regate || len(g.Suites) == 0 {
		return g
	}
	if tree, err := gateTreeOf(repo, commit); err == nil {
		if rec, ok := gateFindPass(ctx.CellID, commit, tree, sel); ok {
			g.Reused = fmt.Sprintf("reused: the %s gate of %s passed at %s", rec.Level, cellDeployShort(commit), rec.At)
		}
	}
	return g
}
