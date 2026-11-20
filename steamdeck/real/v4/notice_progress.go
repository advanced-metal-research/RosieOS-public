package main

import (
	"encoding/base64"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sync"
	"time"
)

// The maintenance notice's progress: the steps a run will take, which one
// runs, and inside the test gate which suites ran, so the overlays show a
// progress bar and "Step 4/8 · Install rt-core" instead of a bare countdown
// (owner, 2026-10-10: "the timer plus a real progress bar showing what it's
// doing"). The run computes it on this Mac at every step and suite transition
// and sends it, with the re-estimated ETA, as one "progress <eta_s> <base64
// JSON>" line on its lock holder's heartbeat stdin; the relay holder writes it
// into both notices (rt_core_maintenance.go). Old clients keep reading
// "phase" (set to the step line) and "eta".
//
// The fraction is weighted by estimated time, not step count: each step
// counts its estimate, finished steps whole, the running one by the share of
// its estimate done (1 - remaining/estimate, at most 99 %). Inside the gate
// the remaining estimate falls as suites finish, each suite weighted by its
// journal mean (gateMeanSeconds). The ETA is the remaining estimates plus
// the release margin (release_eta.go).

// noticeProgressStep is one step as the notice carries it. Remaining is the
// running step's estimate left when the progress was sent; clients count
// elapsed up and remaining down from the progress's updated_at.
type noticeProgressStep struct {
	Name       string  `json:"name"`
	Label      string  `json:"label"`
	State      string  `json:"state"` // pending, running, done or failed
	EstimateS  float64 `json:"estimate_s"`
	ElapsedS   float64 `json:"elapsed_s"`
	RemainingS float64 `json:"remaining_s"`
}

// noticeProgressSuite is the gate inside the running step: the newest suite
// started (its start order of the selection's total), the suites done and
// the ones running (the gate runs several lanes at once).
type noticeProgressSuite struct {
	Name    string   `json:"name"`
	Index   int      `json:"index"`
	Total   int      `json:"total"`
	Done    []string `json:"done"`
	Running []string `json:"running"`
}

type noticeProgress struct {
	Steps    []noticeProgressStep `json:"steps"`
	Current  int                  `json:"current"` // 1-based running step, 0 before the first
	Total    int                  `json:"total"`
	Suite    *noticeProgressSuite `json:"suite,omitempty"`
	Fraction float64              `json:"fraction"`
	// Phase is the step line for clients that read only "phase"; the notice
	// writer moves it there.
	Phase string `json:"phase,omitempty"`
}

// noticeStepLabels are the operator words for each step.
var noticeStepLabels = map[string]string{
	"build+test":  "Build and test gate",
	"build":       "Build",
	"copy-dumps":  "Copy stop dumps off the servo host",
	"precheck":    "Pre-checks",
	"deploy":      "Install rt-core on the servo host",
	"start":       "Start the controller",
	"verify":      "Verify every axis",
	"olp install": "Install the OLP server",
	"prune":       "Prune old releases",
	"config":      "Apply the configuration",
	"rollback":    "Roll back to the previous release",
	"restart":     "Restart the controller",
}

func noticeStepLabel(name string) string {
	if l, ok := noticeStepLabels[name]; ok {
		return l
	}
	return name
}

type noticeStepState struct {
	name, label string
	est         time.Duration
	state       string
	start       time.Time
	elapsed     time.Duration
}

// noticeGate is the gate running inside one step, once its suites are known.
type noticeGate struct {
	step     int
	selected []gateSuite // with journal means
	est      time.Duration
	start    time.Time
	started  []string
	startAt  map[string]time.Time
	done     []string
}

func (g *noticeGate) isDone(name string) bool {
	return containsString(g.done, name)
}

// fraction is the share of the selection's suite time done: finished suites
// whole, running ones up to 90 % of their mean.
func (g *noticeGate) fraction(now time.Time) float64 {
	var total, done float64
	for _, s := range g.selected {
		w := max(s.Seconds, 1)
		total += w
		switch {
		case g.isDone(s.Name):
			done += w
		case !g.startAt[s.Name].IsZero():
			done += min(now.Sub(g.startAt[s.Name]).Seconds(), 0.9*w)
		}
	}
	if total == 0 {
		return 1
	}
	return done / total
}

// noticeTracker is one run's progress; nil-safe, so callers need no checks.
type noticeTracker struct {
	mu    sync.Mutex
	now   func() time.Time
	send  func(eta time.Duration, p noticeProgress) bool
	say   func(string)
	h     releaseHistory
	level string
	steps []noticeStepState
	gate  *noticeGate
}

func newNoticeTracker(h releaseHistory, e releaseETA, level string, send func(time.Duration, noticeProgress) bool, say func(string)) *noticeTracker {
	t := &noticeTracker{now: time.Now, send: send, say: say, h: h, level: level}
	for _, s := range e.Steps {
		t.steps = append(t.steps, noticeStepState{name: s.Name, label: noticeStepLabel(s.Name), est: s.Expected, state: "pending"})
	}
	return t
}

// stepLeft is the running step's estimate left and the time it has run.
func (t *noticeTracker) stepLeft(i int, now time.Time) (left, elapsed time.Duration) {
	s := t.steps[i]
	switch s.state {
	case "done", "failed":
		return 0, s.elapsed
	case "pending":
		return s.est, 0
	}
	elapsed = now.Sub(s.start)
	if g := t.gate; g != nil && g.step == i {
		return time.Duration(float64(g.est) * (1 - g.fraction(now))), elapsed
	}
	return max(s.est-elapsed, s.est/20), elapsed
}

func roundSeconds(d time.Duration) float64 {
	return math.Round(d.Seconds())
}

// snapshot is the progress now and the notice ETA it implies.
func (t *noticeTracker) snapshot() (time.Duration, noticeProgress) {
	now := t.now()
	p := noticeProgress{Total: len(t.steps)}
	var left time.Duration
	var weighted, total float64
	for i, s := range t.steps {
		l, elapsed := t.stepLeft(i, now)
		est := s.est
		if g := t.gate; g != nil && g.step == i && s.state == "running" {
			est = g.start.Sub(s.start) + g.est
		}
		p.Steps = append(p.Steps, noticeProgressStep{Name: s.name, Label: s.label, State: s.state, EstimateS: roundSeconds(est),
			ElapsedS: roundSeconds(elapsed), RemainingS: roundSeconds(l)})
		left += l
		frac := 0.0
		switch s.state {
		case "done", "failed":
			frac = 1
		case "running":
			p.Current = i + 1
			if est > 0 {
				frac = min(max(1-float64(l)/float64(est), 0), 0.99)
			}
		}
		weighted += est.Seconds() * frac
		total += est.Seconds()
	}
	if total > 0 {
		p.Fraction = math.Round(weighted/total*1000) / 1000
	}
	if p.Current > 0 {
		p.Phase = fmt.Sprintf("Step %d/%d · %s", p.Current, p.Total, t.steps[p.Current-1].label)
	}
	if g := t.gate; g != nil && t.steps[g.step].state == "running" {
		suite := &noticeProgressSuite{Total: len(g.selected), Done: append([]string{}, g.done...), Running: []string{}}
		for i, name := range g.started {
			if !g.isDone(name) {
				suite.Running = append(suite.Running, name)
				suite.Name, suite.Index = name, i+1
			}
		}
		if suite.Name == "" && len(g.started) > 0 {
			suite.Name, suite.Index = g.started[len(g.started)-1], len(g.started)
		}
		p.Suite = suite
		if suite.Name != "" {
			p.Phase += fmt.Sprintf(" · suite %d/%d %s", suite.Index, suite.Total, suite.Name)
		}
	}
	return releaseETAWithMargin(left), p
}

// push sends the progress; with say set it also prints the step line.
func (t *noticeTracker) push(announce bool) {
	eta, p := t.snapshot()
	if announce && t.say != nil {
		t.say(fmt.Sprintf("%s · %.0f%% · notice ETA %s", firstNonEmpty(p.Phase, "no step running"), p.Fraction*100, formatDeployDuration(eta.Seconds())))
	}
	if t.send != nil {
		t.send(eta, p)
	}
}

// Begin marks step i running (again, for a retry, with its full estimate).
func (t *noticeTracker) Begin(i int) {
	if t == nil {
		return
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	if i < 0 || i >= len(t.steps) {
		return
	}
	s := &t.steps[i]
	s.state, s.start, s.elapsed = "running", t.now(), 0
	t.push(true)
}

// End marks step i done, or failed.
func (t *noticeTracker) End(i int, ok bool) {
	if t == nil {
		return
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	if i < 0 || i >= len(t.steps) || t.steps[i].state != "running" {
		return
	}
	s := &t.steps[i]
	s.elapsed = t.now().Sub(s.start)
	s.state = map[bool]string{true: "done", false: "failed"}[ok]
	if t.gate != nil && t.gate.step == i {
		t.gate = nil
	}
	t.push(false)
}

// Add appends a step (a rollback) and returns its index.
func (t *noticeTracker) Add(name string, est time.Duration) int {
	if t == nil {
		return -1
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	t.steps = append(t.steps, noticeStepState{name: name, label: noticeStepLabel(name), est: est, state: "pending"})
	return len(t.steps) - 1
}

func (t *noticeTracker) running() int {
	for i, s := range t.steps {
		if s.state == "running" {
			return i
		}
	}
	return -1
}

// GateSelected starts the gate inside the running step with the suites the
// build chose; the step's estimate becomes its time so far plus the gate's
// estimate for exactly these suites.
func (t *noticeTracker) GateSelected(names []string) {
	if t == nil {
		return
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	i := t.running()
	if i < 0 || len(names) == 0 {
		return
	}
	est, _ := t.h.gateEstimate(releaseGatePlan{Level: t.level, Suites: names})
	sel := gateSelection{Why: map[string]string{}}
	for _, name := range names {
		if s, ok := gateSuiteByName(name); ok {
			sel.Suites = append(sel.Suites, s)
			sel.Why[name] = "selected"
		}
	}
	gateOrder(&sel, t.h.Means)
	t.gate = &noticeGate{step: i, selected: sel.Suites, est: est, start: t.now(), startAt: map[string]time.Time{}}
	t.push(true)
}

func (t *noticeTracker) suite(name string, begin bool) {
	t.mu.Lock()
	defer t.mu.Unlock()
	g := t.gate
	if g == nil || g.isDone(name) {
		return
	}
	selected := false
	for _, s := range g.selected {
		selected = selected || s.Name == name
	}
	if !selected {
		return
	}
	if begin {
		if !g.startAt[name].IsZero() {
			return
		}
		g.started = append(g.started, name)
		g.startAt[name] = t.now()
	} else {
		if g.startAt[name].IsZero() {
			g.started = append(g.started, name)
			g.startAt[name] = t.now()
		}
		g.done = append(g.done, name)
	}
	t.push(false)
}

// ObserveGate takes the build's progress lines: "gate <suite> (<dir>): ..."
// starts a suite, "gate <suite> exit <rc> in <n>s" ends it (the test gate's
// own step pairs, which the timings already parse).
func (t *noticeTracker) ObserveGate(phase, detail string) {
	if t == nil || phase != "gate" {
		return
	}
	if m := deployTimingStepEnd.FindStringSubmatch(detail); m != nil {
		t.suite(m[1], false)
	} else if m := deployTimingStepBegin.FindStringSubmatch(detail); m != nil {
		t.suite(m[1], true)
	}
}

// The tracker of each run lock held in this process, by its cellLockEnv
// value: steps inside the run (the build inside a release) report into it.
var noticeTrackers sync.Map

func noticeTrackerActive() *noticeTracker {
	if v, ok := noticeTrackers.Load(os.Getenv(cellLockEnv)); ok {
		return v.(*noticeTracker)
	}
	return nil
}

func noticeTrackerRegister(t *noticeTracker) func() {
	key := os.Getenv(cellLockEnv)
	if t == nil || key == "" {
		return func() {}
	}
	noticeTrackers.Store(key, t)
	return func() { noticeTrackers.CompareAndDelete(key, t) }
}

// noticeProgressLine is the holder's stdin line for a progress update.
func noticeProgressLine(eta time.Duration, p noticeProgress) (string, error) {
	raw, err := json.Marshal(p)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf("progress %d %s\n", int(eta.Round(time.Second)/time.Second), base64.StdEncoding.EncodeToString(raw)), nil
}

// cellLockNoticeProgress sends progress to the holder of the run lock this
// process holds (cellLockForRun); false when it holds none.
func cellLockNoticeProgress(eta time.Duration, p noticeProgress) bool {
	v, ok := cellLockRunSessions.Load(os.Getenv(cellLockEnv))
	if !ok {
		return false
	}
	return v.(*cellLockSession).sendProgress(eta, p)
}

// noticeRunProgress gives a command that took its own run lock
// (cellLockForRun: config, prune) a one-step progress. Inside another run's
// lock it returns nil: that run's progress already has the step.
func noticeRunProgress(p *rtCoreProgress, cell, step string) (*noticeTracker, func()) {
	if noticeTrackerActive() != nil {
		return nil, func() {}
	}
	if _, ok := cellLockRunSessions.Load(os.Getenv(cellLockEnv)); !ok {
		return nil, func() {}
	}
	h := releaseJournalHistory(cell)
	t := newNoticeTracker(h, releaseETA{Steps: []releaseStepETA{h.stepEstimate(step, releaseGatePlan{})}}, "", cellLockNoticeProgress,
		func(line string) { p.Step("progress", line) })
	stop := noticeTrackerRegister(t)
	t.Begin(0)
	return t, stop
}

// noticeStaticProgress is a one-step progress written once with the notice
// (olp install alone: one relay script, no holder to update it).
func noticeStaticProgress(step releaseStepETA) string {
	p := noticeProgress{Steps: []noticeProgressStep{{Name: step.Name, Label: noticeStepLabel(step.Name), State: "running",
		EstimateS: roundSeconds(step.Expected), RemainingS: roundSeconds(step.Expected)}}, Current: 1, Total: 1,
		Phase: "Step 1/1 · " + noticeStepLabel(step.Name)}
	raw, _ := json.Marshal(p)
	return base64.StdEncoding.EncodeToString(raw)
}
