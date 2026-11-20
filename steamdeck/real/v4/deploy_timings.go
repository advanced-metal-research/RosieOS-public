package main

import (
	"bufio"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"sync"
	"time"
)

// Deploy timings: every deploy command records where its wall time went, next
// to its transcript, so a slow phase is measured instead of reconstructed from
// terminal scrollback (the 2026-10-07 reconstruction on a Rosie 1430 cell had to stitch
// build logs, RevPi journals and terminal headers together by hand).
//
// Spans reach the recorder through rtCoreProgress, from three places:
//   - the Mac: each relay session and each Mac-side step (bundle, upload,
//     package fetch), on the Mac clock;
//   - the build and test-gate scripts' existing step pairs, "build <name>:
//     <cmd>" ... "build <name> ok in <n>s" and "gate <name> (<dir>): <cmd>"
//     ... "gate <name> exit <rc> in <n>s";
//   - "@@step <ms> phase <name>" markers in relay and servo scripts: a marker
//     ends the session's previous phase and starts the next; "phase end" only
//     ends it.
//
// Remote spans keep the script's own millisecond clock beside the Mac's
// receive time. The maintenance idle wait ("waiting up to ..." to "cell
// idle") is an operator wait and is kept out of machine time.

const (
	deployTimingsSchema  = "rosie-deploy-timings.v1"
	deployTimingMaxMarks = 4000
)

// deployTimingStepFunc defines `step` unless the script already has one (a
// relay script that embeds another keeps the outer clock). Unlike
// rtCoreConfigStepFunc it also runs where `date` has no %N (macOS, in the
// rendered-script tests): it then counts whole seconds.
const deployTimingStepFunc = `declare -F step > /dev/null || {
deploy_ms(){ local n; n=$(date +%s%N); case "$n" in *N) echo $(( ${n%N} * 1000 )) ;; *) echo $(( n / 1000000 )) ;; esac; }
T0=$(deploy_ms)
step(){ printf '@@step %d %s\n' $(( $(deploy_ms) - T0 )) "$*"; }
}`

var (
	deployTimingStepBegin = regexp.MustCompile(`^([A-Za-z0-9_.-]+)(?: \([^)]*\))?: `)
	deployTimingStepEnd   = regexp.MustCompile(`^([A-Za-z0-9_.-]+) (?:ok in [0-9]+s|exit ([0-9]+) in [0-9]+s)$`)
)

type deployTimingSuite struct {
	Name    string  `json:"name"`
	Seconds float64 `json:"seconds"`
}

type deployTimingSpan struct {
	Name          string              `json:"name"`
	Start         time.Time           `json:"start"`
	End           time.Time           `json:"end"`
	Seconds       float64             `json:"seconds"`
	RemoteStartMS *int64              `json:"remote_start_ms,omitempty"`
	RemoteEndMS   *int64              `json:"remote_end_ms,omitempty"`
	Result        string              `json:"result,omitempty"`
	Operator      bool                `json:"operator,omitempty"`
	Suites        []deployTimingSuite `json:"suites,omitempty"`
	open          bool
}

type deployTimingMark struct {
	At       time.Time `json:"at"`
	RemoteMS *int64    `json:"remote_ms,omitempty"`
	Phase    string    `json:"phase"`
	Detail   string    `json:"detail"`
}

// deployTimingRecord is the <run>.timings.json document.
type deployTimingRecord struct {
	Schema          string             `json:"schema"`
	RunID           string             `json:"run_id"`
	Command         string             `json:"command"`
	Cell            string             `json:"cell"`
	Start           time.Time          `json:"start"`
	End             time.Time          `json:"end"`
	Seconds         float64            `json:"seconds"`
	MachineSeconds  float64            `json:"machine_seconds"`
	OperatorSeconds float64            `json:"operator_seconds"`
	Result          string             `json:"result"`
	Exit            int                `json:"exit"`
	Log             string             `json:"log,omitempty"`
	Spans           []deployTimingSpan `json:"spans"`
	Marks           []deployTimingMark `json:"marks,omitempty"`
	// Legacy rows come from a journal line written before timings existed:
	// total only.
	Legacy bool `json:"-"`
}

// deployTimer collects one run's spans. A nil timer records nothing, so
// progress printers without a journal (olp status) need no checks.
type deployTimer struct {
	mu                   sync.Mutex
	runID, command, cell string
	start                time.Time
	spans                []*deployTimingSpan
	marks                []deployTimingMark
	session              string // innermost open top-level span: parent of remote spans
	phase                string // open "phase" marker span
}

func newDeployTimer(runID, command, cell string, start time.Time) *deployTimer {
	return &deployTimer{runID: runID, command: command, cell: cell, start: start}
}

// deployTimingResult is a span's result for a command's exit.
func deployTimingResult(code int, err error) string {
	switch {
	case err != nil:
		return "error"
	case code == 0:
		return "ok"
	}
	return "exit " + strconv.Itoa(code)
}

func deployTimingParseMS(raw string) *int64 {
	v, err := strconv.ParseInt(raw, 10, 64)
	if err != nil {
		return nil
	}
	return &v
}

func (t *deployTimer) begin(name string, at time.Time) {
	if t == nil {
		return
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	t.beginLocked(name, at, nil, false)
	if !strings.Contains(name, "/") {
		t.session, t.phase = name, ""
	}
}

func (t *deployTimer) end(name, result string, at time.Time) {
	if t == nil {
		return
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	t.endLocked(name, result, at, nil)
}

func (t *deployTimer) beginLocked(name string, at time.Time, remoteMS *int64, operator bool) {
	t.spans = append(t.spans, &deployTimingSpan{Name: name, Start: at, RemoteStartMS: remoteMS, Operator: operator, open: true})
}

func deployTimingClose(s *deployTimingSpan, result string, at time.Time, remoteMS *int64) {
	s.End, s.RemoteEndMS, s.Result, s.open = at, remoteMS, result, false
	s.Seconds = math.Round(at.Sub(s.Start).Seconds()*1000) / 1000
}

// endLocked closes the newest open span called name. Closing a top-level span
// closes what is still open under it: the running phase normally, anything
// else as "unfinished" (a step that never reported its end).
func (t *deployTimer) endLocked(name, result string, at time.Time, remoteMS *int64) {
	for i := len(t.spans) - 1; i >= 0; i-- {
		s := t.spans[i]
		if !s.open || s.Name != name {
			continue
		}
		if !strings.Contains(name, "/") {
			for _, c := range t.spans {
				if c.open && strings.HasPrefix(c.Name, name+"/") {
					r := "unfinished"
					if c.Name == t.phase {
						r = ""
					}
					deployTimingClose(c, r, at, remoteMS)
				}
			}
			if t.session == name {
				t.session, t.phase = "", ""
			}
		}
		if t.phase == name {
			t.phase = ""
		}
		deployTimingClose(s, result, at, remoteMS)
		return
	}
}

// observe takes every progress line. remoteMS is set for "@@step" lines.
func (t *deployTimer) observe(phase, detail string, remoteMS *int64, at time.Time) {
	if t == nil {
		return
	}
	if rest, ok := strings.CutPrefix(detail, "maintenance: "); ok && phase == "remote" {
		phase, detail = "maintenance", rest
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	if (remoteMS != nil || (phase != "remote" && phase != "stderr")) && len(t.marks) < deployTimingMaxMarks {
		t.marks = append(t.marks, deployTimingMark{At: at, RemoteMS: remoteMS, Phase: phase, Detail: detail})
	}
	child := func(name string) string {
		if t.session == "" {
			return name
		}
		return t.session + "/" + name
	}
	switch phase {
	case "build", "gate":
		if m := deployTimingStepEnd.FindStringSubmatch(detail); m != nil {
			result := "ok"
			if m[2] != "" && m[2] != "0" {
				result = "exit " + m[2]
			}
			t.endLocked(child(m[1]), result, at, remoteMS)
		} else if m := deployTimingStepBegin.FindStringSubmatch(detail); m != nil {
			t.beginLocked(child(m[1]), at, remoteMS, false)
		} else if strings.HasPrefix(detail, "FAILED") {
			for _, s := range t.spans {
				if s.open && t.session != "" && strings.HasPrefix(s.Name, t.session+"/") {
					deployTimingClose(s, "failed", at, remoteMS)
				}
			}
		}
	case "phase":
		if t.phase != "" {
			t.endLocked(t.phase, "", at, remoteMS)
		}
		if name, _, _ := strings.Cut(detail, " "); name != "" && name != "end" {
			t.beginLocked(child(name), at, remoteMS, false)
			t.phase = child(name)
		}
	case "maintenance":
		switch {
		case strings.HasPrefix(detail, "waiting up to"):
			t.beginLocked(child("idle wait"), at, remoteMS, true)
		case strings.HasPrefix(detail, "cell idle"):
			t.endLocked(child("idle wait"), "ok", at, remoteMS)
		}
	case "revpi":
		// realtime-core config's servo apply script (rt_core_config_scripts.go).
		switch {
		case strings.HasPrefix(detail, "started; waiting"):
			t.beginLocked(child("start-to-verified"), at, remoteMS, false)
		case strings.HasPrefix(detail, "verified"):
			t.endLocked(child("start-to-verified"), "ok", at, remoteMS)
		}
	}
}

// attachSuites records a gate step's slowest suites on its span.
func (t *deployTimer) attachSuites(name string, suites []deployTimingSuite) {
	if t == nil || len(suites) == 0 {
		return
	}
	t.mu.Lock()
	defer t.mu.Unlock()
	for i := len(t.spans) - 1; i >= 0; i-- {
		if t.spans[i].Name == name {
			t.spans[i].Suites = suites
			return
		}
	}
}

// finish closes everything still open and returns the record.
func (t *deployTimer) finish(result string, exit int, at time.Time, logPath string) deployTimingRecord {
	t.mu.Lock()
	defer t.mu.Unlock()
	for _, s := range t.spans {
		if s.open {
			r := "unfinished"
			if s.Name == t.phase {
				r = ""
			}
			deployTimingClose(s, r, at, nil)
		}
	}
	t.session, t.phase = "", ""
	rec := deployTimingRecord{Schema: deployTimingsSchema, RunID: t.runID, Command: t.command, Cell: t.cell, Start: t.start, End: at,
		Seconds: math.Round(at.Sub(t.start).Seconds()*1000) / 1000, Result: result, Exit: exit, Log: logPath,
		Spans: make([]deployTimingSpan, 0, len(t.spans)), Marks: append([]deployTimingMark(nil), t.marks...)}
	for _, s := range t.spans {
		rec.Spans = append(rec.Spans, *s)
		if s.Operator {
			rec.OperatorSeconds += s.Seconds
		}
	}
	rec.OperatorSeconds = math.Round(rec.OperatorSeconds*1000) / 1000
	rec.MachineSeconds = math.Round((rec.Seconds-rec.OperatorSeconds)*1000) / 1000
	return rec
}

// deployTimingsPath is the timings file beside a run transcript.
func deployTimingsPath(logPath string) string {
	return strings.TrimSuffix(logPath, ".log") + ".timings.json"
}

func writeDeployTimings(path string, rec deployTimingRecord) error {
	raw, err := json.MarshalIndent(rec, "", " ")
	if err != nil {
		return err
	}
	return os.WriteFile(path, append(raw, '\n'), 0o600)
}

// formatDeployDuration: 4.2s, 41.0s, 6m12s, 1h05m.
func formatDeployDuration(seconds float64) string {
	switch {
	case seconds < 60:
		return fmt.Sprintf("%.1fs", seconds)
	case seconds < 3600:
		s := int(math.Round(seconds))
		return fmt.Sprintf("%dm%02ds", s/60, s%60)
	default:
		m := int(math.Round(seconds / 60))
		return fmt.Sprintf("%dh%02dm", m/60, m%60)
	}
}

func deployTimingChildren(spans []deployTimingSpan, parent string) []deployTimingSpan {
	var out []deployTimingSpan
	for _, s := range spans {
		if rest, ok := strings.CutPrefix(s.Name, parent+"/"); ok && !strings.Contains(rest, "/") {
			out = append(out, s)
		}
	}
	return out
}

func deployTimingSpanText(s deployTimingSpan, name string, suites int) string {
	text := name + " " + formatDeployDuration(s.Seconds)
	if s.Operator {
		text += " operator"
	}
	if s.Result != "" && s.Result != "ok" {
		text += " " + s.Result
	}
	if n := min(suites, len(s.Suites)); n > 0 {
		var parts []string
		for _, su := range s.Suites[:n] {
			parts = append(parts, su.Name+" "+formatDeployDuration(su.Seconds))
		}
		text += " [" + strings.Join(parts, ", ") + "]"
	}
	return text
}

// deployTimingSummary is the end-of-run table: the total split into machine
// time and operator wait, then one row per top-level span with its phases.
func deployTimingSummary(rec deployTimingRecord) []string {
	lines := []string{fmt.Sprintf("total %s: machine %s, operator wait %s", formatDeployDuration(rec.Seconds), formatDeployDuration(rec.MachineSeconds), formatDeployDuration(rec.OperatorSeconds))}
	for _, s := range rec.Spans {
		if strings.Contains(s.Name, "/") {
			continue
		}
		line := fmt.Sprintf("%-20s %8s", s.Name, formatDeployDuration(s.Seconds))
		if s.Result != "" && s.Result != "ok" {
			line += " " + s.Result
		}
		var parts []string
		for _, c := range deployTimingChildren(rec.Spans, s.Name) {
			if c.Seconds >= 0.05 || len(c.Suites) > 0 || c.Operator {
				parts = append(parts, deployTimingSpanText(c, strings.TrimPrefix(c.Name, s.Name+"/"), 3))
			}
		}
		if len(parts) > 0 {
			line += "  " + strings.Join(parts, ", ")
		}
		lines = append(lines, line)
	}
	return lines
}

// deployTimingCompact is a history row's "where the time went": the n
// longest top-level spans, each with its longest phase.
func deployTimingCompact(rec deployTimingRecord, n int) string {
	if rec.Legacy {
		return "(total only: run recorded before phase timings)"
	}
	var top []deployTimingSpan
	for _, s := range rec.Spans {
		if !strings.Contains(s.Name, "/") {
			top = append(top, s)
		}
	}
	sort.SliceStable(top, func(i, j int) bool { return top[i].Seconds > top[j].Seconds })
	var parts []string
	for _, s := range top[:min(n, len(top))] {
		text := s.Name + " " + formatDeployDuration(s.Seconds)
		kids := deployTimingChildren(rec.Spans, s.Name)
		sort.SliceStable(kids, func(i, j int) bool { return kids[i].Seconds > kids[j].Seconds })
		if len(kids) > 0 && kids[0].Seconds >= 1 {
			text += " (" + deployTimingSpanText(kids[0], strings.TrimPrefix(kids[0].Name, s.Name+"/"), 0) + ")"
		}
		parts = append(parts, text)
	}
	return strings.Join(parts, ", ")
}

// --- history -----------------------------------------------------------------

// loadDeployTimings reads every <run>.timings.json in dir, plus the runs the
// cell-deploy.jsonl and rt-core-config.jsonl journals list without one (total
// only), oldest first.
func loadDeployTimings(dir string) ([]deployTimingRecord, error) {
	files, err := filepath.Glob(filepath.Join(dir, "*.timings.json"))
	if err != nil {
		return nil, err
	}
	var out []deployTimingRecord
	seen := map[string]bool{}
	for _, f := range files {
		raw, err := os.ReadFile(f)
		if err != nil {
			return nil, err
		}
		var rec deployTimingRecord
		if err := json.Unmarshal(raw, &rec); err != nil || rec.Schema != deployTimingsSchema {
			continue
		}
		seen[rec.RunID] = true
		out = append(out, rec)
	}
	for _, journal := range []string{"cell-deploy.jsonl", "rt-core-config.jsonl"} {
		f, err := os.Open(filepath.Join(dir, journal))
		if err != nil {
			continue
		}
		s := bufio.NewScanner(f)
		s.Buffer(make([]byte, 0, 64*1024), 4*1024*1024)
		for s.Scan() {
			var line struct {
				Time     time.Time `json:"time"`
				RunID    string    `json:"run_id"`
				Command  string    `json:"command"`
				Cell     string    `json:"cell"`
				Result   string    `json:"result"`
				Exit     int       `json:"exit"`
				Duration float64   `json:"duration_s"`
				Log      string    `json:"log"`
			}
			if json.Unmarshal(s.Bytes(), &line) != nil || line.RunID == "" || seen[line.RunID] {
				continue
			}
			seen[line.RunID] = true
			out = append(out, deployTimingRecord{RunID: line.RunID, Command: firstNonEmpty(line.Command, "realtime-core config"), Cell: line.Cell,
				Start: line.Time.Add(-time.Duration(line.Duration * float64(time.Second))), End: line.Time, Seconds: line.Duration,
				MachineSeconds: line.Duration, Result: line.Result, Exit: line.Exit, Log: line.Log, Legacy: true})
		}
		f.Close()
	}
	sort.SliceStable(out, func(i, j int) bool { return out[i].Start.Before(out[j].Start) })
	return out, nil
}

type deployTimingAggregate struct {
	Name           string
	Runs           int
	Total, Longest float64
}

// deployTimingBottlenecks sums machine time per leaf phase (a span with no
// phases of its own) over the given runs, longest total first.
func deployTimingBottlenecks(recs []deployTimingRecord) (phases []deployTimingAggregate, operator float64, suites []deployTimingAggregate) {
	byName := map[string]*deployTimingAggregate{}
	bySuite := map[string]*deployTimingAggregate{}
	add := func(m map[string]*deployTimingAggregate, name string, seconds float64) {
		a := m[name]
		if a == nil {
			a = &deployTimingAggregate{Name: name}
			m[name] = a
		}
		a.Runs++
		a.Total += seconds
		a.Longest = max(a.Longest, seconds)
	}
	for _, rec := range recs {
		parents := map[string]bool{}
		for _, s := range rec.Spans {
			if i := strings.LastIndex(s.Name, "/"); i > 0 {
				parents[s.Name[:i]] = true
			}
		}
		for _, s := range rec.Spans {
			if parents[s.Name] {
				continue
			}
			if s.Operator {
				operator += s.Seconds
				continue
			}
			add(byName, s.Name, s.Seconds)
			for _, su := range s.Suites {
				add(bySuite, s.Name+": "+su.Name, su.Seconds)
			}
		}
	}
	sorted := func(m map[string]*deployTimingAggregate) []deployTimingAggregate {
		var out []deployTimingAggregate
		for _, a := range m {
			out = append(out, *a)
		}
		sort.Slice(out, func(i, j int) bool {
			if out[i].Total != out[j].Total {
				return out[i].Total > out[j].Total
			}
			return out[i].Name < out[j].Name
		})
		return out
	}
	return sorted(byName), operator, sorted(bySuite)
}

const deployTimingsUsage = `usage: realtime-core timings [--last 20] [--cell id] [--command text] [--journal dir] [--top 12]

Prints the deploy history from ~/.rosie/deploy-journal: one row per run (total,
machine time, where the time went), then the phases and test suites that took
the most machine time over those runs. Runs journalled before phase timings
existed show their total only. Reads local files only.
`

func runRealtimeCoreTimings(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("realtime-core timings", flag.ContinueOnError)
	fs.SetOutput(stderr)
	last := fs.Int("last", 20, "show the newest N runs")
	cell := fs.String("cell", "", "only this cell id")
	command := fs.String("command", "", "only commands containing this text (for example build, deploy, olp)")
	journal := fs.String("journal", "", "journal directory (default ~/.rosie/deploy-journal)")
	top := fs.Int("top", 12, "bottleneck rows to print")
	if len(args) > 0 && isHelp(args[0]) {
		fmt.Fprint(stdout, deployTimingsUsage)
		return nil
	}
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() != 0 || *last < 1 || *top < 0 {
		return fmt.Errorf("realtime-core timings takes --last N (N >= 1) and no positional arguments")
	}
	dir := *journal
	if dir == "" {
		home, err := os.UserHomeDir()
		if err != nil {
			return err
		}
		dir = filepath.Join(home, ".rosie", "deploy-journal")
	}
	all, err := loadDeployTimings(dir)
	if err != nil {
		return err
	}
	var recs []deployTimingRecord
	for _, r := range all {
		if (*cell == "" || r.Cell == *cell) && (*command == "" || strings.Contains(r.Command, *command)) {
			recs = append(recs, r)
		}
	}
	if len(recs) > *last {
		recs = recs[len(recs)-*last:]
	}
	if len(recs) == 0 {
		fmt.Fprintf(stdout, "no deploy runs journalled in %s\n", dir)
		return nil
	}
	fmt.Fprintf(stdout, "%-16s  %-28s  %-26s  %-10s  %8s  %8s  %s\n", "START", "COMMAND", "CELL", "RESULT", "TOTAL", "MACHINE", "WHERE THE TIME WENT")
	for _, r := range recs {
		machine := formatDeployDuration(r.MachineSeconds)
		if r.Legacy {
			machine = "-"
		}
		fmt.Fprintf(stdout, "%-16s  %-28s  %-26s  %-10s  %8s  %8s  %s\n", r.Start.Local().Format("2006-01-02 15:04"), r.Command, r.Cell, r.Result,
			formatDeployDuration(r.Seconds), machine, deployTimingCompact(r, 3))
	}
	phases, operator, suites := deployTimingBottlenecks(recs)
	if len(phases) > 0 {
		fmt.Fprintf(stdout, "\nslowest phases over these %d run(s), machine time (operator wait %s excluded):\n", len(recs), formatDeployDuration(operator))
		fmt.Fprintf(stdout, "  %-44s  %4s  %8s  %8s  %8s\n", "PHASE", "RUNS", "TOTAL", "MEAN", "LONGEST")
		for _, a := range phases[:min(*top, len(phases))] {
			fmt.Fprintf(stdout, "  %-44s  %4d  %8s  %8s  %8s\n", a.Name, a.Runs, formatDeployDuration(a.Total), formatDeployDuration(a.Total/float64(a.Runs)), formatDeployDuration(a.Longest))
		}
	}
	if len(suites) > 0 {
		fmt.Fprintf(stdout, "\nslowest test suites:\n")
		for _, a := range suites[:min(*top, len(suites))] {
			fmt.Fprintf(stdout, "  %-60s  %4d  %8s  %8s\n", a.Name, a.Runs, formatDeployDuration(a.Total/float64(a.Runs)), formatDeployDuration(a.Longest))
		}
	}
	return nil
}

// --- streaming commands --------------------------------------------------------

// deployTimingLineWriter feeds a streamed command's output (deploy and start
// run through runRemoteSSH) into the progress printer one line at a time, so
// its "@@step" lines are timed like a relay session's.
type deployTimingLineWriter struct {
	p      *rtCoreProgress
	stderr bool
	buf    []byte
}

func (w *deployTimingLineWriter) Write(b []byte) (int, error) {
	w.buf = append(w.buf, b...)
	for {
		i := strings.IndexByte(string(w.buf), '\n')
		if i < 0 {
			return len(b), nil
		}
		w.line(strings.TrimRight(string(w.buf[:i]), "\r"))
		w.buf = w.buf[i+1:]
	}
}

func (w *deployTimingLineWriter) line(text string) {
	if w.stderr {
		w.p.Step("stderr", text)
	} else {
		w.p.Remote(text)
	}
}

// Flush emits a final line without a newline.
func (w *deployTimingLineWriter) Flush() {
	if len(w.buf) > 0 {
		w.line(string(w.buf))
		w.buf = nil
	}
}

// stream runs one relay command over the relay route as the timed span
// label, its output through the progress printer.
func (r *cellDeployRun) stream(label string, via rtCoreRelay, command string) error {
	r.p.Record(label+" (relay script)", command)
	out, errw := &deployTimingLineWriter{p: r.p}, &deployTimingLineWriter{p: r.p, stderr: true}
	r.begin(label)
	err := via.run(out, errw, command)
	out.Flush()
	errw.Flush()
	r.end(label, err)
	return err
}

// finishStream journals a streamed command by its error: ok, or failed with
// the remote exit code (-1 when it did not run).
func (r *cellDeployRun) finishStream(err error) {
	if err == nil {
		r.finish("ok", 0, "")
		return
	}
	code := -1
	var exit *exec.ExitError
	if errors.As(err, &exit) {
		code = exit.ExitCode()
	}
	r.finish("failed", code, err.Error())
}
