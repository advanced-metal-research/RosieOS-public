//go:build linux

package main

import (
	"bytes"
	"compress/gzip"
	"encoding/json"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
	control "rosieos/rt-core/sdk/control"
)

func TestRetainedJogReceiptIsNotCountedInNewGrant(t *testing.T) {
	old := ipcclient.JogObservation{ObservedSourceSequence: 200, FirstObservedHostNs: 20000, AppGrantGeneration: 1}
	next := old
	next.AppGrantGeneration, next.JogGeneration = 2, 7
	if newJogObservation(old, next) {
		t.Fatal("retained receipt counted after Stop")
	}
	next.ObservedSourceSequence, next.FirstObservedHostNs = 1, 30000
	if !newJogObservation(old, next) {
		t.Fatal("new session receipt omitted")
	}
	next.FirstObservedHostNs = 0
	if newJogObservation(old, next) {
		t.Fatal("absent timestamp counted")
	}
}

func TestProcCountersRequireNamedUnits(t *testing.T) {
	m, err := parseMemory("Name: rt-control\nVmRSS: 100 kB\nVmHWM: 200 kB\n")
	if err != nil || m.RSS != 102400 || m.HWM != 204800 || m.HeapInuse != nil {
		t.Fatalf("memory=%+v error=%v", m, err)
	}
	for _, raw := range []string{"", "VmRSS: 100 kB\n", "VmRSS: 100 bytes\nVmHWM: 200 kB\n", "VmRSS: 100 kB\nVmHWM: 90 kB\n"} {
		if _, err := parseMemory(raw); err == nil {
			t.Fatalf("accepted invalid memory: %q", raw)
		}
	}
	f, err := parseFaults("42 (rt (cycle)) S 1 2 3 4 5 6 17 8 19")
	if err != nil || f.Minor != 17 || f.Major != 19 {
		t.Fatalf("faults=%+v error=%v", f, err)
	}
	for _, raw := range []string{"", "42 (rt-cycle) S 1", "42 (rt-cycle) S 1 2 3 4 5 6 x 8 19"} {
		if _, err := parseFaults(raw); err == nil {
			t.Fatalf("accepted invalid stat: %q", raw)
		}
	}
}

func TestSoakPercentilesUseUniqueObservationRanks(t *testing.T) {
	p := distribution([]uint64{50, 10, 30, 40, 20})
	if p.Count != 5 || p.P50NS != 30 || p.P99NS != 50 || p.MaxNS != 50 {
		t.Fatal(p)
	}
}

func testRecord(counts ...int) record {
	count := 10
	if len(counts) > 0 {
		count = counts[0]
	}
	zero, start, peak, end := uint64(0), uint64(1024), uint64(5120), uint64(2048)
	allowance := uint64(2048)
	open := uint32(2)
	b := sample{HostNS: 1, Phase: "warmup", Adapter: memory{RSS: 4096, HWM: 4096, HeapInuse: &start}, Daemon: memory{RSS: 4096, HWM: 36864}, NativeSlots: &zero, PreparationSlots: &open,
		DaemonFaults: &faults{}, IO: &ioCounters{}, EventRing: &eventRingSample{First: 1, Latest: 1, Occupancy: 1},
		BufferHealth: &ipcclient.BufferHealth{TimeNS: 1, SlotsFree: 3, NativeSlotOccupancy: map[string]uint32{"free": 3, "writing": 0, "prepared": 0, "ready": 0, "active": 0, "retired": 0}}, Metrics: nativeMetrics{TimeNS: 1}}
	f := b
	f.HostNS, f.Phase = uint64(count*100+50), "run"
	f.Adapter.HeapInuse, f.Daemon.RSS = &end, 36864
	p := b
	p.HostNS, p.Phase = 2, "run"
	p.Adapter.HeapInuse = &peak
	r := record{Schema: "rosie.rt-core.soak.v2", Configuration: configuration{options: options{Iterations: count, AdapterHeapAllowance: &allowance}, Points: planPoints, PageBytes: 4096, JogAgeBoundNS: 250000000, CycleLatenessBoundNS: 20000000}, IterationsCompleted: count, Baseline: &b, Final: &f,
		Publications: []publication{{HostNS: 1, TimeNS: 1}}, Samples: []sample{b, p}, EventBatches: []eventBatchSample{{Batch: control.EventBatch{NextSequence: 1, LatestSequence: 1, AdapterIncarnation: "adapter", Events: []control.Event{{Sequence: 1, TimeNS: 1, Type: "enabled", AdapterIncarnation: "adapter"}}}}}}
	for i := 1; i <= count; i++ {
		s := b
		s.HostNS, s.Phase, s.Iteration = uint64(i*100), "run", i
		r.Samples = append(r.Samples, s)
		r.Iterations = append(r.Iterations, iteration{Phase: "run", Number: i, Handle: uint64(i), StartSequence: uint64(i), Completed: true, ExecutedDiscarded: true, OverlapDiscarded: true, OverlapHandle: 100 + uint64(i), JogSent: 200, UploadStartedNS: uint64(i*100 - 90), UploadCompletedNS: uint64(i*100 - 80), OverlapStartedNS: 10, OverlapCompletedNS: 2000, End: s})
		r.Jog = append(r.Jog, jogSample{Phase: "run", Iteration: i, DuringUpload: true, AgeNS: 1000, Observation: ipcclient.JogObservation{FirstObservedHostNs: 1500, ObservedOriginHostNs: 500, Open: 1, HasInput: 1}})
	}
	r.Samples = append(r.Samples, f)
	return r
}

func TestSoakAcceptanceAbsoluteBoundariesAndMissingCounters(t *testing.T) {
	r := testRecord()
	if !r.assess() {
		t.Fatalf("boundary values rejected: %+v", r.Assertions)
	}
	for _, name := range []string{"heap", "rss", "minor", "major", "handles", "slots", "missing_heap", "missing_slots", "incomplete"} {
		t.Run(name, func(t *testing.T) {
			r := testRecord()
			switch name {
			case "heap":
				*r.Final.Adapter.HeapInuse = 2049
			case "rss":
				r.Final.Daemon.RSS = 36865
			case "minor":
				r.Final.RT.Minor = 1
			case "major":
				r.Final.RT.Major = 1
			case "handles":
				r.Final.PreparedHandles = 1
			case "slots":
				*r.Final.NativeSlots = 1
			case "missing_heap":
				r.Final.Adapter.HeapInuse = nil
			case "missing_slots":
				r.Final.NativeSlots = nil
			case "incomplete":
				r.IterationsCompleted = 9
			}
			if r.assess() {
				t.Fatalf("accepted %s: %+v", name, r.Assertions)
			}
		})
	}
}

func TestLoadedHostCannotExemptOverruns(t *testing.T) {
	r := testRecord()
	r.Final.Metrics.Overruns = 1
	if r.assess() {
		t.Fatal("quiet-host overrun passed")
	}
	r.HostLoaded = true
	if r.assess() {
		t.Fatalf("loaded-host overrun passed: %+v", r.Assertions)
	}
	r.Final.RT.Minor = 1
	if r.assess() {
		t.Fatal("loaded host excused a page fault")
	}
}

func TestIORequiresEveryNamedCounter(t *testing.T) {
	v, err := parseIO("rchar: 17\nwchar: 19\nread_bytes: 4096\nwrite_bytes: 8192\n")
	if err != nil || v.ReadBytes != 4096 || v.WriteBytes != 8192 || v.ReadChars != 17 || v.WriteChars != 19 {
		t.Fatalf("%+v %v", v, err)
	}
	for _, raw := range []string{"", "read_bytes: 0\nwrite_bytes: 0", "rchar: 1\nwchar: 2\nread_bytes: -1\nwrite_bytes: 0", "rchar: 1\nwchar: 2\nread_bytes: 0\nwrite_bytes: 0\nwrite_bytes: 1"} {
		if _, err := parseIO(raw); err == nil {
			t.Fatalf("accepted %q", raw)
		}
	}
}

func TestRetainedRingOccupancyIncludesBeyondOneBatch(t *testing.T) {
	v, err := retainedEvents(control.EventBatch{LatestSequence: 300, Events: []control.Event{{Sequence: 44, Dropped: &control.EventsDropped{FirstLostSequence: 1, LastLostSequence: 44}}, {Sequence: 45}}})
	if err != nil || v.First != 45 || v.Occupancy != 256 {
		t.Fatalf("%+v %v", v, err)
	}
	r := testRecord()
	delete(r.Samples[0].BufferHealth.NativeSlotOccupancy, "retired")
	if r.assess() {
		t.Fatal("missing native state passed")
	}
}

func TestRecordedRunCheckRejectsLeaksAndHeapGrowth(t *testing.T) {
	for _, name := range []string{"valid", "leaked_slot", "heap_growth", "event_gap", "jog_age", "missing_io", "truncated", "short_run"} {
		t.Run(name, func(t *testing.T) {
			r := testRecord(50)
			switch name {
			case "leaked_slot":
				n := uint64(1)
				r.Iterations[24].End.NativeSlots = &n
			case "heap_growth":
				n := uint64(2049)
				r.Final.Adapter.HeapInuse = &n
			case "event_gap":
				r.EventBatches[0].Batch.Events[0].Sequence = 2
			case "jog_age":
				r.Jog[0].AgeNS = 250000001
			case "missing_io":
				r.Samples[0].IO = nil
			case "truncated":
				r.Iterations = r.Iterations[:49]
			case "short_run":
				r.Configuration.Iterations = 10
				r.IterationsCompleted = 10
				r.Iterations = r.Iterations[:10]
			}
			r.Status = "pass" // A forged summary must not bypass raw-record checks.
			path := filepath.Join(t.TempDir(), "record.json.gz")
			f, err := os.Create(path)
			if err != nil {
				t.Fatal(err)
			}
			z := gzip.NewWriter(f)
			if err := json.NewEncoder(z).Encode(r); err != nil {
				t.Fatal(err)
			}
			if err := z.Close(); err != nil {
				t.Fatal(err)
			}
			if err := f.Close(); err != nil {
				t.Fatal(err)
			}
			var out, errs bytes.Buffer
			code := checkRecord(path, &out, &errs, nil)
			if (code == 0) != (name == "valid") {
				t.Fatalf("exit=%d\n%s\n%s", code, &out, &errs)
			}
		})
	}
}

func TestOutputMustStayInBuild(t *testing.T) {
	for _, path := range []string{"docs/soak.json", "../soak.json", "build/../README.md"} {
		if _, err := outputPath("/source/rt-core", path); err == nil {
			t.Fatalf("accepted %q", path)
		}
	}
	path, err := outputPath("/source/rt-core", "build/soak.json")
	if err != nil || !strings.HasSuffix(path, "/build/soak.json") {
		t.Fatalf("%s %v", path, err)
	}
}
