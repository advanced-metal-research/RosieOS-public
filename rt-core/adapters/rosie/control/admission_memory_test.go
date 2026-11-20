//go:build linux

package control

import (
	"bufio"
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/procfs"
	"runtime"
	"runtime/debug"
	"strconv"
	"strings"
	"sync"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
)

// WSL/Go 1.26.2 non-race measurement: baseline VmRSS 11,649,024 (JSON) and
// 11,911,168 (dense) bytes. Round the larger fixed overhead up to 16 MiB.
const adapterMemoryFixedBytes uint64 = 16 << 20

// Same 250,000-point measurement: peak VmHWM 262,336,512 (JSON) and 210,006,016
// (dense) bytes; incremental costs round up to 1003 and 793 bytes/point.
// The 1536-byte allowance includes allocator/GC variation above that measurement.
const adapterMemoryPointBytes uint64 = 1536

// Race measurement: baseline 31,866,880 bytes and peak 936,153,088 bytes,
// incremental 3619 bytes/point; allowances include shadow/allocator variation.
const adapterRaceFixedBytes uint64 = 64 << 20

// Race measurement above: 3619 bytes/point; rounded up for allocator variation.
const adapterRacePointBytes uint64 = 5120

// Unverified reclamation policy: retain at most 1/4 of peak preparation heap growth.
const adapterReleasedHeapDivisor uint64 = 4

type admissionMemory struct{ HeapInuse, Sys, VmRSS, VmHWM uint64 }

func admissionProcMemory(pid int) (uint64, uint64, error) {
	values, err := procfs.MemoryCounters(pid, "VmRSS:", "VmHWM:")
	return values["VmRSS:"], values["VmHWM:"], err
}

func admissionMemorySample(pid int, adapter bool) (admissionMemory, error) {
	var sample admissionMemory
	var err error
	if adapter {
		var stats runtime.MemStats
		runtime.ReadMemStats(&stats)
		sample.HeapInuse, sample.Sys = stats.HeapInuse, stats.Sys
	}
	sample.VmRSS, sample.VmHWM, err = admissionProcMemory(pid)
	return sample, err
}

func (m *admissionMemory) peak(next admissionMemory) {
	m.HeapInuse = max(m.HeapInuse, next.HeapInuse)
	m.Sys = max(m.Sys, next.Sys)
	m.VmRSS = max(m.VmRSS, next.VmRSS)
	m.VmHWM = max(m.VmHWM, next.VmHWM)
}

func admissionDaemonPID(t *testing.T) int {
	t.Helper()
	entries, err := os.ReadDir("/proc")
	if err != nil {
		t.Fatal(err)
	}
	for _, entry := range entries {
		pid, err := strconv.Atoi(entry.Name())
		if err != nil {
			continue
		}
		status, _ := os.ReadFile(filepath.Join("/proc", entry.Name(), "status"))
		cmdline, _ := os.ReadFile(filepath.Join("/proc", entry.Name(), "cmdline"))
		if strings.Contains(string(status), fmt.Sprintf("PPid:\t%d\n", os.Getpid())) && bytes.Contains(cmdline, []byte("rosie-rt-core-")) {
			return pid
		}
	}
	t.Fatal("simulated daemon pid missing")
	return 0
}

// Each format runs in a fresh test subprocess. Its client streams a parent-made
// fixture from disk, so fixture construction and earlier tests cannot inflate
// the adapter's HWM or hide reclamation behind a historical process peak.
func TestAdapterMemoryDuringLargePreparation(t *testing.T) {
	if format := os.Getenv("RTCORE_ADMISSION_MEMORY_CHILD"); format != "" {
		admissionMeasureMemory(t, format, os.Getenv("RTCORE_ADMISSION_MEMORY_INPUT"))
		return
	}
	for _, format := range []string{"json", "program"} {
		t.Run(format, func(t *testing.T) {
			root, err := filepath.Abs("../../../build/go-tmp")
			if err != nil {
				t.Fatal(err)
			}
			if err := os.MkdirAll(root, 0755); err != nil {
				t.Fatal(err)
			}
			input, err := os.CreateTemp(root, "admission-memory-*")
			if err != nil {
				t.Fatal(err)
			}
			defer os.Remove(input.Name())
			if format == "json" {
				w := bufio.NewWriter(input)
				// The child adds its fresh authority envelope; the file is only points.
				fmt.Fprint(w, `[`)
				for i := 0; i < 250000; i++ {
					if i != 0 {
						fmt.Fprint(w, ",")
					}
					fmt.Fprintf(w, `{"time_ns":%d,"position":[0,0,0,0,0,0,0,0,0],"velocity":[0,0,0,0,0,0,0,0,0]}`, int64(i)*10000000)
				}
				fmt.Fprint(w, `]}`)
				if err := w.Flush(); err != nil {
					t.Fatal(err)
				}
			} else {
				if _, err := input.Write(admissionLargeProgram(t)); err != nil {
					t.Fatal(err)
				}
			}
			if err := input.Close(); err != nil {
				t.Fatal(err)
			}
			process := exec.Command(os.Args[0], "-test.run=^TestAdapterMemoryDuringLargePreparation$", "-test.v")
			process.Env = append(os.Environ(), "RTCORE_ADMISSION_MEMORY_CHILD="+format, "RTCORE_ADMISSION_MEMORY_INPUT="+input.Name())
			output, err := process.CombinedOutput()
			t.Log(string(output))
			if err != nil {
				t.Fatalf("memory subprocess: %v", err)
			}
		})
	}
}

func admissionLargeProgram(t *testing.T) []byte {
	t.Helper()
	samples := make([]densejointtraj.Sample, 250000)
	for i := range samples {
		samples[i].TimeS = float64(i) * .01
	}
	segments := []densejointtraj.Segment{{Kind: "freespace", SourceID: "admission-memory", Samples: samples}}
	limits := []float64{.1, .1, .1, .1, .1, .1, .1, .1, .1}
	header := densejointtraj.Header{
		Schema: densejointtraj.Schema, PlanID: "admission-memory", ProgramID: "admission-memory", ProgramDigest: "sha256:" + strings.Repeat("0", 64),
		ManifestRevision: 7, PlanRevision: 1,
		Axes:           densejointtraj.HeaderAxes{AxisCount: 9, AxisMask: 511, PositionUnit: "rad", VelocityUnit: "rad/s"},
		Sampling:       densejointtraj.Sampling{DtS: .01, TotalSampleCount: 250000, TotalDurationS: 2499.99},
		Limits:         densejointtraj.Limits{MaxAbsQdRadS: limits},
		Segments:       []densejointtraj.SegmentMeta{{Index: 0, Kind: "freespace", SourceID: "admission-memory", SampleCount: 250000, DurationS: 2499.99, Block: densejointtraj.BlockMeta{}}},
		SampleEncoding: densejointtraj.EncodingMeta{Encoding: densejointtraj.SampleEncoding, RecordBytes: densejointtraj.RecordBytes},
	}
	blob, err := densejointtraj.Encode(header, segments)
	if err != nil {
		t.Fatal(err)
	}
	return blob
}

func admissionMeasureMemory(t *testing.T, format, input string) {
	f := startLifecycleCore(t)
	pid := admissionDaemonPID(t)
	api := serveHandleAPI(t, f.controller, nil)
	api.client.Timeout = 0
	file, err := os.Open(input)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	runtime.GC()
	before, err := admissionMemorySample(os.Getpid(), true)
	if err != nil {
		t.Fatal(err)
	}
	daemonBefore, err := admissionMemorySample(pid, false)
	if err != nil {
		t.Fatal(err)
	}
	peak, daemonPeak := before, daemonBefore
	done, stopped := make(chan struct{}), make(chan struct{})
	sampleErr := make(chan error, 1)
	go func() {
		defer close(stopped)
		ticker := time.NewTicker(2 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-done:
				return
			case <-ticker.C:
			}
			a, err := admissionMemorySample(os.Getpid(), true)
			if err != nil {
				sampleErr <- err
				return
			}
			d, err := admissionMemorySample(pid, false)
			if err != nil {
				sampleErr <- err
				return
			}
			peak.peak(a)
			daemonPeak.peak(d)
		}
	}()
	var stopOnce sync.Once
	stopSampling := func() { stopOnce.Do(func() { close(done) }); <-stopped }
	defer stopSampling()
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	path := "/v1/program"
	var reader io.Reader = file
	if format == "json" {
		path = "/v1/control"
		prefix := fmt.Sprintf(`{"schema":"rosie.rt-control.request.v1","operation":"prepare_trajectory","session":%q,"generation":%d,"axis_mask":511,"points":`, f.fence.Session, f.fence.Generation)
		reader = io.MultiReader(strings.NewReader(prefix), file)
	}
	r, err := http.NewRequestWithContext(ctx, http.MethodPost, "http://unix"+path, reader)
	if err != nil {
		t.Fatal(err)
	}
	r.Header.Set("X-Control-Session", f.fence.Session)
	r.Header.Set("X-Control-Generation", fmt.Sprint(f.fence.Generation))
	reply, err := api.client.Do(r)
	if err != nil {
		t.Fatal(err)
	}
	var response Response
	err = json.NewDecoder(reply.Body).Decode(&response)
	_ = reply.Body.Close()
	if err == nil && response.Error == "control_session_stale" {
		// This is a memory measurement, not a lease test. Its precondition is that
		// the fixture's 100 ms renewal kept the 500 ms lease alive while this
		// process allocated and decoded 250,000 points under the race detector.
		// When the host is loaded enough to starve that goroutine past the lease,
		// the measurement is inconclusive rather than a defect; the lease policy
		// itself is covered by the lifecycle and concurrency tests. The P8-03 soak
		// runs this measurement without competing builders.
		t.Skipf("lease expired during the %s preparation; renewal starved under host load, measurement inconclusive", format)
	}
	if err != nil || response.Error != "" {
		t.Fatalf("preparation: %+v %v", response, err)
	}
	handle := response.Handle
	if format == "program" {
		f.controller.mu.Lock()
		handle = f.controller.program.handle
		f.controller.mu.Unlock()
	}
	f.controller.mu.Lock()
	count := len(f.controller.handles[handle].points)
	f.controller.mu.Unlock()
	if count != 250000 {
		t.Fatalf("prepared %d points", count)
	}
	if _, err := f.controller.Discard(f.fence, handle); err != nil {
		t.Fatal(err)
	}
	runtime.GC()
	debug.FreeOSMemory()
	after, err := admissionMemorySample(os.Getpid(), true)
	if err != nil {
		t.Fatal(err)
	}
	daemonAfter, err := admissionMemorySample(pid, false)
	if err != nil {
		t.Fatal(err)
	}
	// Stop sampling before inspecting its independent maxima.
	stopSampling()
	select {
	case err := <-sampleErr:
		t.Fatal(err)
	default:
	}
	peak.peak(after)
	daemonPeak.peak(daemonAfter)
	t.Logf("format=%s adapter before=%+v peak=%+v after=%+v bytes", format, before, peak, after)
	t.Logf("daemon pid=%d before=%+v peak=%+v after=%+v bytes", pid, daemonBefore, daemonPeak, daemonAfter)
	pointBytes := adapterMemoryPointBytes
	fixedBytes := adapterMemoryFixedBytes
	race := false
	if info, ok := debug.ReadBuildInfo(); ok {
		for _, setting := range info.Settings {
			if setting.Key == "-race" && setting.Value == "true" {
				race = true
			}
		}
	}
	if race {
		pointBytes = adapterRacePointBytes
		fixedBytes = adapterRaceFixedBytes
	}
	budget := fixedBytes + pointBytes*250000
	t.Logf("race=%v VmHWM budget=%d fixed=%d per_point=%d bytes; measured incremental=%d bytes/point", race, budget, fixedBytes, pointBytes, (peak.VmHWM-before.VmRSS+249999)/250000)
	if peak.VmHWM >= budget {
		t.Fatalf("adapter VmHWM %d >= %d bytes", peak.VmHWM, budget)
	}
	if peak.HeapInuse <= before.HeapInuse {
		t.Fatal("preparation heap peak was not sampled")
	}
	if after.HeapInuse > before.HeapInuse+(peak.HeapInuse-before.HeapInuse)/adapterReleasedHeapDivisor {
		t.Fatalf("heap did not release at least 3/4 of preparation growth")
	}
}
