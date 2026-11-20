// Command tuningreport turns copied Stop dumps into one per-joint tuning and
// acceptance report, the same way every iteration: lag, following error and
// target lead against the configured bounds, acceleration, jerk, stops,
// ringing (frequency, damping), every drive's loop quality up to 500 Hz
// (torque, velocity and residual band energies, lines, buzz episodes),
// Cartesian path holds, Move-tos and the servo host's cycle cost, with an
// optional delta against a previous report.
//
// It decodes on the host it runs on (the Mac or the relay), never on the servo
// host: decoding there stalled EtherCAT for ~1 s on 2026-10-06.
//
//	go run ./tools/tuningreport report --dumps DIR --manifest CELL.json --out DIR [--baseline OLD/metrics.json]
//	go run ./tools/tuningreport extract --dump X.bin --from-s 10 --to-s 18 --out testdata/x.trfx.gz
//	go run ./tools/tuningreport inertia-plan --manifest CELL.json --joint J4 --accels 200,400 --start-deg ... --out DIR
//	go run ./tools/tuningreport inertia --dumps DIR --manifest CELL.json --joint J4 --motors MOTORS.json --out DIR
package main

import (
	"errors"
	"flag"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"
)

func main() {
	os.Exit(run(os.Args[1:], os.Stdout, os.Stderr))
}

func usage(w io.Writer) {
	fmt.Fprint(w, `usage:
  tuningreport report  --dumps DIR (--manifest CELL.json | --machine MACHINE.json | --compiled DIR) --out DIR
                       [--since 45m|2026-10-08T10:30] [--journal FILE ...]
                       [--baseline PREVIOUS/metrics.json] [--label TEXT]
                       [--tz America/Los_Angeles] [--clock-offset-ns N] [--repo DIR] [--no-plots]
  tuningreport extract --dump FILE.bin (--from-s S --to-s S | --from-seq N --to-seq N) --out FILE.trfx.gz [--note TEXT]
  tuningreport inertia-plan (--manifest CELL.json | --machine M.json | --compiled DIR) --joint J4 --accels 200,400
                       --start-deg q1,...,q6 --out DIR [--center-deg C] [--motors config/motors/<robot>.json]
                       [--peak-speed-frac 0.75] [--jerk R] [--repeats 2] [--dwell-s 0.5]
  tuningreport inertia --dumps DIR (--manifest CELL.json | --machine M.json | --compiled DIR) --joint J4
                       --motors config/motors/<robot>.json --out DIR [--since 30m] [--plan DIR/plan.json]
                       [--band-deg-s LO,HI] [--opportunistic] [--label TEXT] [--no-plots]

DIR holds Stop dumps (*.bin, as copied by 'realtime-core copy-dumps'; *.trfx.gz
fixtures) and optionally journal text (*.log: OLP and RevPi journalctl output).
`)
}

type multi []string

func (m *multi) String() string     { return strings.Join(*m, ",") }
func (m *multi) Set(v string) error { *m = append(*m, v); return nil }

func run(args []string, stdout, stderr io.Writer) int {
	if len(args) == 0 {
		usage(stderr)
		return 2
	}
	var err error
	switch args[0] {
	case "report":
		err = runReport(args[1:], stdout, stderr)
	case "extract":
		err = runExtract(args[1:], stdout, stderr)
	case "inertia-plan":
		err = runInertiaPlan(args[1:], stdout, stderr)
	case "inertia":
		err = runInertia(args[1:], stdout, stderr)
	case "-h", "--help", "help":
		usage(stdout)
		return 0
	default:
		usage(stderr)
		return 2
	}
	if err != nil {
		fmt.Fprintln(stderr, "tuningreport:", err)
		return 1
	}
	return 0
}

type reportOptions struct {
	dumps, manifest, machine, compiled, repo, out, baseline, label, tz, since string
	journals                                                                  []string
	clockOffset                                                               *int64
	plots                                                                     bool
	params                                                                    Params
}

func runReport(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("report", flag.ContinueOnError)
	fs.SetOutput(stderr)
	o := reportOptions{params: defaultParams()}
	var journals multi
	var offset string
	noPlots := false
	fs.StringVar(&o.dumps, "dumps", "", "directory of copied Stop dumps")
	fs.StringVar(&o.manifest, "manifest", "", "cell manifest (rt-core/config/cells/<cell>.json)")
	fs.StringVar(&o.machine, "machine", "", "machine config, instead of the manifest's (compiled here with rtctl)")
	fs.StringVar(&o.compiled, "compiled", "", "compiled configuration directory (rtctl compile output, e.g. the one deployed), instead of compiling")
	fs.StringVar(&o.repo, "repo", "", "RosieOS checkout (default: found above the manifest)")
	fs.StringVar(&o.out, "out", "", "output directory")
	fs.StringVar(&o.baseline, "baseline", "", "previous report's metrics.json")
	fs.StringVar(&o.label, "label", "", "iteration label, e.g. 'J4 C00.06=300'")
	fs.StringVar(&o.tz, "tz", "Local", "time zone for printed times")
	fs.StringVar(&o.since, "since", "", "only dumps named after this: a duration back from now (45m) or a time in --tz (2026-10-08T10:30)")
	fs.StringVar(&offset, "clock-offset-ns", "", "UTC ns minus record t_ns, overriding the derived clock")
	fs.Var(&journals, "journal", "journal text file (repeatable); *.log in --dumps are read too")
	fs.BoolVar(&noPlots, "no-plots", false, "skip PNG plots")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if o.dumps == "" || o.out == "" || (o.manifest == "" && o.machine == "" && o.compiled == "") {
		usage(stderr)
		return errors.New("--dumps, --out and --manifest (or --machine or --compiled) are required")
	}
	if offset != "" {
		var v int64
		if _, err := fmt.Sscan(offset, &v); err != nil {
			return fmt.Errorf("--clock-offset-ns: %w", err)
		}
		o.clockOffset = &v
	}
	o.journals, o.plots = journals, !noPlots
	r, err := buildReport(o, stderr)
	if err != nil {
		return err
	}
	fmt.Fprintf(stdout, "wrote %s (%d windows, %d holds, %d trajectories)\n", filepath.Join(o.out, "README.md"), len(r.Windows), len(r.Holds), len(r.Trajectories))
	if r.Baseline != nil {
		fmt.Fprintf(stdout, "baseline %s: %d better, %d worse\n", r.Baseline.File, r.Baseline.Better, r.Baseline.Worse)
	}
	return nil
}

func buildReport(o reportOptions, stderr io.Writer) (*Report, error) {
	cfg, err := loadCellConfig(o.manifest, o.machine, o.compiled, o.repo)
	if err != nil {
		return nil, err
	}
	tz, err := time.LoadLocation(o.tz)
	if err != nil {
		return nil, err
	}
	entries, err := os.ReadDir(o.dumps)
	if err != nil {
		return nil, err
	}
	// --since keeps one iteration's dumps out of a dump_dir that collects
	// every session; it reads the servo host's file-name time, so nothing is
	// decoded to decide.
	var since time.Time
	if o.since != "" {
		if d, err := time.ParseDuration(o.since); err == nil {
			since = time.Now().Add(-d)
		} else if since, err = time.ParseInLocation("2006-01-02T15:04", o.since, tz); err != nil {
			return nil, fmt.Errorf("--since %q: want a duration (45m) or 2006-01-02T15:04", o.since)
		}
	}
	var inputs []*Series
	journals := append([]string(nil), o.journals...)
	for _, e := range entries {
		name := e.Name()
		p := filepath.Join(o.dumps, name)
		switch {
		case strings.HasSuffix(name, ".bin") || strings.HasSuffix(name, ".trfx.gz"):
			if ft, ok := parseDumpFileTime(name); ok && !since.IsZero() && ft.Before(since) {
				continue
			}
			s, err := loadInput(p)
			if err != nil {
				return nil, err
			}
			if s.Axes < len(cfg.Joints) {
				return nil, fmt.Errorf("%s has %d axes, the configuration %d joints", name, s.Axes, len(cfg.Joints))
			}
			inputs = append(inputs, s)
		case strings.HasSuffix(name, ".log"):
			journals = append(journals, p)
		}
	}
	windows, err := mergeSources(inputs)
	if err != nil {
		return nil, err
	}
	year := time.Now().Year()
	for _, s := range inputs {
		if t, err := time.Parse(time.RFC3339Nano, s.Sources[0].FileTimeUTC); err == nil {
			year = t.Year()
			break
		}
	}
	var lines []JournalLine
	for _, j := range journals {
		l, err := parseJournalFile(j, year)
		if err != nil {
			return nil, err
		}
		lines = append(lines, l...)
	}
	clocks := settleClock(windows, lines, o.clockOffset)

	a := newAnalysis(cfg, o.params, tz, windows)
	r := &Report{Schema: reportSchema, Generated: time.Now().UTC().Format(time.RFC3339), TimeZone: tz.String(), Label: o.label,
		Journals: journals, Config: cfg, Params: o.params}
	for _, s := range inputs {
		r.Sources = append(r.Sources, s.Sources...)
	}
	for i, w := range a.wins {
		armed := 0
		for _, v := range w.armed {
			if v {
				armed++
			}
		}
		wi := WindowInfo{First: a.wall(w, 0).Format("2006-01-02 15:04:05.000"), Last: a.wall(w, w.n-1).Format("15:04:05.000"),
			FirstSeq: w.s.Seq[0], LastSeq: w.s.Seq[w.n-1], Records: w.n, ArmedS: float64(armed) * w.dt, Clock: clocks[i]}
		for _, s := range w.s.Sources {
			wi.Sources = append(wi.Sources, s.File)
		}
		r.Windows = append(r.Windows, wi)
	}
	nj := len(cfg.Joints)
	for j := 0; j < nj; j++ {
		r.Joints = append(r.Joints, a.jointMetrics(j))
	}
	// Residual shift: each joint's own best-shift lag when its lag is a lag
	// (the error/speed fit explains it, R2 >= LagCredibleR2); a joint that
	// barely moved (J4 under J1/J6 motion: R2 0.02 at 21:04, "lag" 31 ms
	// that is really ringing) takes the median of the credible others, as
	// every joint runs the same drive loop design.
	credible := func(m JointMetrics) bool { return !math.IsNaN(m.BestShiftMs) && m.FitR2 >= o.params.LagCredibleR2 }
	var fitted []float64
	for _, m := range r.Joints {
		if credible(m) {
			fitted = append(fitted, m.BestShiftMs)
		}
	}
	shifts := make([]int, nj)
	for j := range r.Joints {
		sh := r.Joints[j].BestShiftMs
		if !credible(r.Joints[j]) {
			sh = median(fitted)
		}
		if math.IsNaN(sh) {
			sh = 0
		}
		shifts[j] = int(sh)
		r.Joints[j].Ring = a.ring(j, sh)
	}
	olp := parseOLPHolds(lines)
	if len(a.wins) > 0 {
		r.Holds = a.holds(olp, olpShift(a.wins[0].s, lines))
		shiftH := 0.0
		for _, c := range clocks {
			if c.Method == "RevPi journal observed_ns" {
				shiftH = c.RevPiShiftH
			}
		}
		r.LeadJournal, r.Observed = revpiFacts(lines, shiftH)
	}
	r.Trajectories = a.trajectories(shifts)
	servo := a.servo(shifts, r.Holds)
	r.Servo = &servo
	r.Cycle = a.cycle()
	r.buildScalars()
	if o.baseline != "" {
		if r.Baseline, err = loadBaseline(o.baseline, r.Scalars); err != nil {
			return nil, err
		}
	}
	if err = os.MkdirAll(o.out, 0o755); err != nil {
		return nil, err
	}
	var plots []string
	if o.plots {
		if plots, err = a.plots(o.out, r); err != nil {
			return nil, err
		}
	}
	if err = writeJSON(filepath.Join(o.out, "metrics.json"), r); err != nil {
		return nil, err
	}
	if err = a.writeServoTimeline(filepath.Join(o.out, "servo-timeline.csv"), r.Servo); err != nil {
		return nil, err
	}
	if err = os.WriteFile(filepath.Join(o.out, "README.md"), []byte(r.markdown(plots)), 0o644); err != nil {
		return nil, err
	}
	return r, nil
}

func runExtract(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("extract", flag.ContinueOnError)
	fs.SetOutput(stderr)
	var dump, out, note string
	var fromS, toS float64
	var fromSeq, toSeq int64
	fs.StringVar(&dump, "dump", "", "Stop dump (.bin)")
	fs.StringVar(&out, "out", "", "fixture to write (.trfx.gz)")
	fs.StringVar(&note, "note", "", "provenance note stored in the fixture")
	fs.Float64Var(&fromS, "from-s", -1, "start, seconds after the first record")
	fs.Float64Var(&toS, "to-s", -1, "end, seconds after the first record")
	fs.Int64Var(&fromSeq, "from-seq", -1, "first sequence")
	fs.Int64Var(&toSeq, "to-seq", -1, "last sequence")
	if err := fs.Parse(args); err != nil {
		return err
	}
	if dump == "" || !strings.HasSuffix(out, ".trfx.gz") {
		usage(stderr)
		return errors.New("--dump and --out FILE.trfx.gz are required")
	}
	s, err := loadInput(dump)
	if err != nil {
		return err
	}
	a, b := 0, s.Len()
	if fromS >= 0 || toS >= 0 {
		t0 := s.TNs[0]
		a = sort.Search(s.Len(), func(k int) bool { return float64(s.TNs[k]-t0)/1e9 >= fromS })
		if toS >= 0 {
			b = sort.Search(s.Len(), func(k int) bool { return float64(s.TNs[k]-t0)/1e9 > toS })
		}
	}
	if fromSeq >= 0 {
		a = sort.Search(s.Len(), func(k int) bool { return s.Seq[k] >= fromSeq })
	}
	if toSeq >= 0 {
		b = sort.Search(s.Len(), func(k int) bool { return s.Seq[k] > toSeq })
	}
	if b-a < 2 {
		return errors.New("extract range holds fewer than 2 records")
	}
	x := s.slice(a, b)
	for i := range x.Sources {
		src := &x.Sources[i]
		src.Note = strings.TrimSpace(fmt.Sprintf("extract of %s (sha256 %.16s) records %d-%d of %d. %s", src.File, src.SHA256, a, b-1, src.Records, src.Note))
		src.FirstSeq, src.LastSeq, src.Records, src.Missing = uint64(x.Seq[0]), uint64(x.Seq[x.Len()-1]), x.Len(), 0
	}
	if err = writeFixture(out, x, note); err != nil {
		return err
	}
	st, _ := os.Stat(out)
	fmt.Fprintf(stdout, "wrote %s: %d records, seq %d-%d, %d bytes\n", out, x.Len(), x.Seq[0], x.Seq[x.Len()-1], st.Size())
	return nil
}
