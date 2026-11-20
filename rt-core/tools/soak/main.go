//go:build linux

// soak drives the public simulated control boundary and records unavailable
// instrumentation explicitly, preventing incomplete measurements from passing.
package main

import (
	"compress/gzip"
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/signal"
	"path/filepath"
	"strconv"
	"syscall"
)

type options struct {
	Iterations           int     `json:"iterations"`
	Root                 string  `json:"root"`
	Out                  string  `json:"out"`
	AdapterHeapAllowance *uint64 `json:"adapter_heap_allowance_bytes,omitempty"`
}

func main() {
	ctx, cancel := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	code := runCLI(ctx, os.Args[1:], os.Stdout, os.Stderr)
	cancel()
	os.Exit(code)
}

// runCLI always writes the collected record, including failures, so a failed
// workload or missing metric cannot disappear behind a successful summary.
func runCLI(ctx context.Context, args []string, stdout, stderr io.Writer) int {
	var opts options
	var check string
	flags := flag.NewFlagSet("soak", flag.ContinueOnError)
	flags.SetOutput(stderr)
	flags.IntVar(&opts.Iterations, "iterations", 50, "iteration count, at least 2")
	flags.StringVar(&opts.Root, "root", ".", "rt-core source/build root")
	flags.StringVar(&opts.Out, "out", "build/soak.json", "JSON record under root/build")
	flags.StringVar(&check, "check", "", "verify an existing JSON or JSON.gz record without starting processes")
	flags.Func("adapter-heap-allowance-bytes", "predeclared absolute cap for post-iteration/final HeapInuse snapshots in bytes (GC completion unobserved)", func(raw string) error {
		n, err := strconv.ParseUint(raw, 10, 64)
		if err == nil {
			opts.AdapterHeapAllowance = &n
		}
		return err
	})
	if err := flags.Parse(args); err != nil {
		if errors.Is(err, flag.ErrHelp) {
			return 0
		}
		return 2
	}
	if flags.NArg() != 0 || opts.Iterations < 2 {
		fmt.Fprintln(stderr, "soak requires flags only and --iterations >= 2")
		return 2
	}
	if check != "" {
		return checkRecord(check, stdout, stderr, opts.AdapterHeapAllowance)
	}
	var err error
	opts.Root, err = filepath.Abs(opts.Root)
	if err == nil {
		opts.Out, err = outputPath(opts.Root, opts.Out)
	}
	if err != nil {
		fmt.Fprintln(stderr, err)
		return 2
	}
	r, runErr := runSoak(ctx, opts)
	writeErr := writeRecord(opts.Out, r)
	printSummary(stdout, r)
	verdict := r.verdict()
	if runErr != nil || writeErr != nil {
		verdict = "FAIL"
	}
	code := printVerdict(stdout, "soak", verdict, r.IterationsCompleted)
	if err := errors.Join(runErr, writeErr); err != nil {
		fmt.Fprintln(stderr, "soak:", err)
		return 1
	}
	return code
}

// checkRecord recomputes acceptance from the saved raw measurements and rejects
// incomplete qualification runs; it never trusts the saved summary or status.
func checkRecord(path string, stdout, stderr io.Writer, allowance *uint64) int {
	fail := func(err error) int {
		fmt.Fprintln(stderr, err)
		return printVerdict(stdout, "soak-check", "FAIL", 0)
	}
	f, err := os.Open(path)
	if err != nil {
		return fail(err)
	}
	defer f.Close()
	var reader io.Reader = f
	if filepath.Ext(path) == ".gz" {
		z, err := gzip.NewReader(f)
		if err != nil {
			return fail(err)
		}
		defer z.Close()
		reader = z
	}
	var r record
	d := json.NewDecoder(reader)
	if err := d.Decode(&r); err != nil {
		return fail(err)
	}
	var trailing any
	if err := d.Decode(&trailing); err != io.EOF {
		return fail(fmt.Errorf("record has trailing data or invalid gzip checksum: %v", err))
	}
	if allowance != nil {
		r.Configuration.AdapterHeapAllowance = allowance
	}
	r.assess()
	printSummary(stdout, r)
	verdict := r.verdict()
	if r.Configuration.Iterations != 50 || r.IterationsCompleted != 50 {
		verdict = "FAIL"
		fmt.Fprintln(stderr, "soak-check requires 50 complete iterations")
	}
	return printVerdict(stdout, "soak-check", verdict, r.IterationsCompleted)
}

// printVerdict gives automation distinct outcomes; unqualified bounds cannot
// silently acquire the successful exit code used by a fully passing record.
func printVerdict(w io.Writer, command, verdict string, iterations int) int {
	fmt.Fprintf(w, "%s: %s (%d iterations)\n", command, verdict, iterations)
	switch verdict {
	case "PASS":
		return 0
	case "UNQUALIFIED":
		return 3
	default:
		return 1
	}
}

func outputPath(root, out string) (string, error) {
	if !filepath.IsAbs(out) {
		out = filepath.Join(root, out)
	}
	build := filepath.Join(root, "build")
	rel, err := filepath.Rel(build, out)
	if err != nil || !filepath.IsLocal(rel) || rel == "." {
		return "", errors.New("soak output must be a file inside root/build")
	}
	return out, nil
}

func writeRecord(path string, r record) error {
	if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
		return err
	}
	f, err := os.CreateTemp(filepath.Dir(path), ".soak-*.json")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	enc := json.NewEncoder(f)
	enc.SetIndent("", "  ")
	err = enc.Encode(r)
	err = errors.Join(err, f.Close())
	if err != nil {
		return err
	}
	return os.Rename(f.Name(), path)
}

func printSummary(w io.Writer, r record) {
	fmt.Fprintf(w, "metric\tvalue\niterations\t%d/%d\nhost_loaded\t%t\n", r.IterationsCompleted, r.Configuration.Iterations, r.HostLoaded)
	fmt.Fprintf(w, "daemon_peak_rss_bytes\t%d\nadapter_peak_rss_bytes\t%d\n", r.Summary.DaemonPeakRSS, r.Summary.AdapterPeakRSS)
	fmt.Fprintf(w, "daemon_peak_hwm_bytes\t%d\nadapter_peak_hwm_bytes\t%d\n", r.Summary.DaemonPeakHWM, r.Summary.AdapterPeakHWM)
	fmt.Fprintf(w, "rt_max_abs_jitter_ns\t%d\nrt_max_cycle_work_ns\t%d\nprepared_handles\t%d\n", r.Summary.MaxJitterNS, r.Summary.MaxWorkNS, r.Summary.PreparedHandles)
	for _, metric := range []struct {
		name  string
		value any
	}{
		{"daemon_rss_growth_bytes", r.Summary.DaemonRSSGrowth},
		{"rt_overruns_after_warmup", r.Summary.Overruns},
		{"rt_minor_faults_after_warmup", r.Summary.MinorFaults},
		{"rt_major_faults_after_warmup", r.Summary.MajorFaults},
		{"diagnostics_dropped_after_warmup", r.Summary.DiagnosticsDropped},
		{"daemon_io_delta", r.Summary.IO},
		{"daemon_faults_delta", r.Summary.DaemonFaults},
	} {
		value := metric.value
		if reason := r.Summary.Unavailable[metric.name]; reason != "" {
			value = "unavailable (" + reason + ")"
		}
		fmt.Fprintf(w, "%s\t%+v\n", metric.name, value)
	}
	fmt.Fprintf(w, "status_publication_gap_max_ns\t%d\njog_input_age_p50_ns\t%d\njog_input_age_p99_ns\t%d\njog_input_age_max_ns\t%d\n", r.Summary.PublicationGapNS, r.Summary.Jog.P50NS, r.Summary.Jog.P99NS, r.Summary.Jog.MaxNS)
	fmt.Fprintf(w, "adapter_heap_start_peak_end_bytes\t%v -> %v -> %v\nevents_observed\t%d\nevent_ring_peak\t%d\nnative_slots_peak\t%d\n", printable(r.Summary.AdapterHeapStart), printable(r.Summary.AdapterPeakHeap), printable(r.Summary.AdapterHeapEnd), r.Summary.EventCount, r.Summary.EventRingPeak, r.Summary.NativeSlotsPeak)
	for _, minimum := range r.Summary.AdapterHeapMinima {
		fmt.Fprintf(w, "adapter_heap_after_iteration_%d_min_bytes\t%v\n", minimum.Iteration, printable(minimum.Bytes))
	}
	for _, a := range r.Assertions {
		fmt.Fprintf(w, "%s: %s (%s)\n", a.Name, a.Result, a.Detail)
	}
	fmt.Fprintf(w, "record\t%s\n", r.Configuration.Out)
}

func printable(v *uint64) any {
	if v == nil {
		return "unavailable"
	}
	return *v
}
