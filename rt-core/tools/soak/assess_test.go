//go:build linux

package main

import (
	"bytes"
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func savedFixture(t *testing.T, r record) map[string]any {
	t.Helper()
	raw, err := json.Marshal(r)
	if err != nil {
		t.Fatal(err)
	}
	var fields map[string]any
	if err := json.Unmarshal(raw, &fields); err != nil {
		t.Fatal(err)
	}
	return fields
}

func checkFixture(t *testing.T, fields any, args ...string) (int, string) {
	t.Helper()
	raw, err := json.Marshal(fields)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(t.TempDir(), "record.json")
	if err := os.WriteFile(path, raw, 0600); err != nil {
		t.Fatal(err)
	}
	var out, errs bytes.Buffer
	code := runCLI(context.Background(), append([]string{"--check", path}, args...), &out, &errs)
	return code, out.String() + errs.String()
}

// TestSavedRecordRequiresEveryMeasurement removes every sample member, including
// nested zeros, at every stored sample location so omissions cannot qualify.
func TestSavedRecordRequiresEveryMeasurement(t *testing.T) {
	r := testRecord(50)
	if code, out := checkFixture(t, r); code != 0 {
		t.Fatalf("fixture: %d %s", code, out)
	}
	var paths [][]string
	var collect func(map[string]any, []string)
	collect = func(fields map[string]any, prefix []string) {
		for key, value := range fields {
			path := append(append([]string{}, prefix...), key)
			if strings.Join(path, ".") == "daemon.heap_inuse_bytes" {
				continue
			}
			paths = append(paths, path)
			if nested, ok := value.(map[string]any); ok {
				collect(nested, path)
			}
		}
	}
	collect(savedFixture(t, r)["samples"].([]any)[0].(map[string]any), nil)
	for _, location := range []string{"samples[0]", "after_warmup", "final", "iterations[24].end"} {
		for _, path := range paths {
			for _, null := range []bool{false, true} {
				name := location + "." + strings.Join(path, ".")
				t.Run(name+map[bool]string{false: "/missing", true: "/null"}[null], func(t *testing.T) {
					fields := savedFixture(t, r)
					var sample map[string]any
					switch location {
					case "samples[0]":
						sample = fields["samples"].([]any)[0].(map[string]any)
					case "iterations[24].end":
						sample = fields["iterations"].([]any)[24].(map[string]any)["end"].(map[string]any)
					default:
						sample = fields[location].(map[string]any)
					}
					for _, key := range path[:len(path)-1] {
						sample = sample[key].(map[string]any)
					}
					key := path[len(path)-1]
					if null {
						sample[key] = nil
					} else {
						delete(sample, key)
					}
					code, out := checkFixture(t, fields)
					if code != 1 || !strings.Contains(out, name+":required measurement missing or null") {
						t.Fatalf("exit=%d; expected named rejection %s\n%s", code, name, out)
					}
				})
			}
		}
	}
}

func TestSavedRecordRequiresWorkloadAndReceiptMeasurements(t *testing.T) {
	for _, test := range []struct{ root, field, nested string }{
		{"iterations", "overlap_upload_started_ns", ""},
		{"jog_observations", "input_age_ns", ""},
		{"jog_observations", "UpdateReason", "jog_observation"},
		{"status_publications", "gap_ns", ""},
		{"event_batches", "next_sequence", "batch"},
	} {
		for _, null := range []bool{false, true} {
			fields := savedFixture(t, testRecord(50))
			object := fields[test.root].([]any)[0].(map[string]any)
			path := test.root + "[0]."
			if test.nested != "" {
				object = object[test.nested].(map[string]any)
				path += test.nested + "."
			}
			if null {
				object[test.field] = nil
			} else {
				delete(object, test.field)
			}
			code, out := checkFixture(t, fields)
			if code != 1 || !strings.Contains(out, path+test.field+":required measurement missing or null") {
				t.Fatalf("exit=%d %s", code, out)
			}
		}
	}
	for _, field := range []string{"iterations", "points_per_plan", "page_bytes", "published_jog_input_age_bound_ns", "cycle_lateness_bound_ns"} {
		for _, null := range []bool{false, true} {
			fields := savedFixture(t, testRecord(50))
			configuration := fields["configuration"].(map[string]any)
			if null {
				configuration[field] = nil
			} else {
				delete(configuration, field)
			}
			code, out := checkFixture(t, fields)
			if code != 1 || !strings.Contains(out, "configuration."+field+":required measurement missing or null") {
				t.Fatalf("exit=%d %s", code, out)
			}
		}
	}
}

func TestAdapterHeapUsesDeclaredAbsoluteAllowance(t *testing.T) {
	for _, peak := range []uint64{5120, 5124, 1000000} {
		r := testRecord()
		r.Samples[1].Adapter.HeapInuse = &peak
		end := uint64(2049)
		r.Final.Adapter.HeapInuse = &end
		r.assess()
		if r.verdict() != "FAIL" || assertionResult(r, "adapter_heap") != "fail" {
			t.Fatalf("peak=%d %+v", peak, r.Assertions)
		}
	}
	r := testRecord()
	r.assess()
	if r.Summary.AdapterHeapStart == nil || *r.Summary.AdapterHeapStart != 1024 || *r.Summary.AdapterPeakHeap != 5120 || *r.Summary.AdapterHeapEnd != 2048 {
		t.Fatal(r.Summary)
	}
	if len(r.Summary.AdapterHeapMinima) != 10 || *r.Summary.AdapterHeapMinima[0].Bytes != 1024 {
		t.Fatal(r.Summary.AdapterHeapMinima)
	}
	quiescent := r.Iterations[0].End
	quiescent.HostNS = 105
	value := uint64(512)
	quiescent.Adapter.HeapInuse = &value
	r.Samples = append(r.Samples[:3], append([]sample{quiescent}, r.Samples[3:]...)...)
	r.assess()
	if *r.Summary.AdapterHeapMinima[0].Bytes != 512 || *r.Summary.AdapterHeapMinima[1].Bytes != 1024 {
		t.Fatal(r.Summary.AdapterHeapMinima)
	}
	r = testRecord()
	value = 2049
	r.Iterations[0].End.Adapter.HeapInuse = &value
	r.Samples[2].Adapter.HeapInuse = &value
	r.assess()
	if assertionResult(r, "adapter_heap") != "fail" {
		t.Fatal("post-iteration minimum exceeded 2048 bytes without rejection")
	}
}

func TestEndpointCountersAcceptOnlyEqualOrBracketingValues(t *testing.T) {
	b := testRecord().Samples[0]
	b.HostNS, b.DaemonFaults = 1, &faults{Minor: 10}
	f := b
	f.HostNS, f.DaemonFaults = 3, &faults{Minor: 30}
	end := b
	end.HostNS, end.DaemonFaults = 2, &faults{Minor: 20}
	if !endpointMatchesSamples(end, []sample{b, f}) {
		t.Fatal("20 faults not bracketed by 10 and 30")
	}
	end.DaemonFaults = &faults{Minor: 31}
	if endpointMatchesSamples(end, []sample{b, f}) {
		t.Fatal("31 faults accepted outside 10..30")
	}
	end.HostNS = 3
	if endpointMatchesSamples(end, []sample{b, f}) {
		t.Fatal("31 faults accepted at sample timestamp with 30")
	}
}

func assertionResult(r record, name string) string {
	for _, a := range r.Assertions {
		if a.Name == name {
			return a.Result
		}
	}
	return "absent"
}

func TestIterationCounterRegressionAndSampleReconciliation(t *testing.T) {
	for _, test := range []string{"review_regression", "endpoint_mismatch", "bracketed", "outside_bracket", "outside_stream", "timing_regression", "io_regression"} {
		t.Run(test, func(t *testing.T) {
			r := testRecord(50)
			switch test {
			case "review_regression":
				r.Iterations[24].End.DaemonFaults = &faults{Minor: 42}
				r.Iterations[25].End.DaemonFaults = &faults{Minor: 0}
			case "endpoint_mismatch":
				r.Iterations[49].End.DaemonFaults = &faults{Minor: 42}
			case "bracketed":
				r.Iterations[24].End.HostNS = 2501
			case "outside_bracket":
				r.Iterations[24].End.HostNS = 2501
				r.Iterations[24].End.DaemonFaults = &faults{Minor: 42}
			case "outside_stream":
				r.Iterations[49].End.HostNS = 10000
			case "timing_regression":
				r.Iterations[24].End.Metrics.MaxWorkNS = 42
			case "io_regression":
				r.Iterations[24].End.IO = &ioCounters{ReadChars: 42}
			}
			r.assess()
			if (r.verdict() == "PASS") != (test == "bracketed") {
				t.Fatalf("%s: %+v", r.verdict(), r.Assertions)
			}
			if test == "review_regression" && (assertionResult(r, "iteration_counter_continuity") != "fail" || assertionResult(r, "endpoint_sample_consistency") != "fail") {
				t.Fatal(r.Assertions)
			}
		})
	}
}

func TestSoakVerdictExitCodesAndAllowanceFlag(t *testing.T) {
	for _, test := range []struct {
		name, verdict string
		code          int
		args          []string
	}{
		{"declared", "PASS", 0, nil},
		{"undeclared", "UNQUALIFIED", 3, nil},
		{"cli_allowance", "PASS", 0, []string{"--adapter-heap-allowance-bytes", "2048"}},
		{"cli_zero", "FAIL", 1, []string{"--adapter-heap-allowance-bytes", "0"}},
		{"overrun", "FAIL", 1, nil},
		{"invalid_allowance", "", 2, []string{"--adapter-heap-allowance-bytes", "-1"}},
	} {
		t.Run(test.name, func(t *testing.T) {
			r := testRecord(50)
			if test.name != "declared" {
				r.Configuration.AdapterHeapAllowance = nil
			}
			if test.name == "overrun" {
				r.Final.Metrics.Overruns = 1
			}
			code, out := checkFixture(t, r, test.args...)
			if code != test.code || (test.verdict != "" && !strings.Contains(out, "soak-check: "+test.verdict+" (50 iterations)")) {
				t.Fatalf("exit=%d\n%s", code, out)
			}
			if test.name == "undeclared" && !strings.Contains(out, "adapter_heap: unqualified (no declared allowance)") {
				t.Fatal(out)
			}
		})
	}
}

func TestMissingBaselinePrintsUnavailableDeltas(t *testing.T) {
	r := testRecord()
	r.Baseline = nil
	r.assess()
	var out bytes.Buffer
	printSummary(&out, r)
	for _, field := range []string{"rt_overruns_after_warmup", "rt_minor_faults_after_warmup", "rt_major_faults_after_warmup", "diagnostics_dropped_after_warmup", "daemon_rss_growth_bytes", "daemon_io_delta", "daemon_faults_delta"} {
		if !strings.Contains(out.String(), field+"\tunavailable (warm-up baseline or final observation absent") {
			t.Fatalf("%s\n%s", field, &out)
		}
	}
}
