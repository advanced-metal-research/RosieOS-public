//go:build linux

package ipcclient

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"golang.org/x/sys/unix"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strconv"
	"strings"
	"testing"
	"time"
)

// A live connection must refuse a foreign ring before disconnect detection;
// an already mapped reader must retain the original daemon's records.
func TestTelemetryConnectionRejectsReplacedRing(t *testing.T) {
	original := startCoreWithRole(t, "simulation.json", false)
	foreign := startCoreWithRole(t, "simulation.json", false)
	path := filepath.Join(filepath.Dir(original.options.SocketPath), "telemetry.ring")
	foreignPath := filepath.Join(filepath.Dir(foreign.options.SocketPath), "telemetry.ring")
	pinned, err := original.client.OpenTelemetry()
	if err != nil {
		t.Fatal(err)
	}
	defer pinned.Close()
	for _, bridge := range []bool{false, true} {
		t.Run(fmt.Sprintf("descriptor=%v", bridge), func(t *testing.T) {
			replaced, target := path, foreignPath
			if bridge {
				if err := os.Rename(path, path+".saved"); err != nil {
					t.Fatal(err)
				}
				defer os.Rename(path+".saved", path)
				replaced, target = path+".sock", foreignPath+".sock"
			}
			if err := os.Symlink(target, replaced+".replacement"); err != nil {
				t.Fatal(err)
			}
			if err := os.Rename(replaced+".replacement", replaced); err != nil {
				t.Fatal(err)
			}
			if err := original.client.Err(); err != nil {
				t.Fatalf("original connection is not live: %v", err)
			}
			reader, err := original.client.OpenTelemetry()
			if err == nil {
				reader.Close()
				t.Fatal("foreign ring admitted under the original live connection")
			}
			if !errors.Is(err, ErrTelemetryIdentity) {
				t.Fatalf("foreign ring refusal: got %v, want ErrTelemetryIdentity", err)
			}
			batch, err := pinned.ReadFrom(0, 1)
			if err != nil || len(batch.Records) != 1 {
				t.Fatalf("pinned reader lost original records: %+v %v", batch, err)
			}
		})
	}
}

// Each batch rechecks the header even after handoff, so a changed identity
// cannot return records or advance a consumer cursor.
func TestTelemetryBatchRejectsChangedIdentity(t *testing.T) {
	r := &TelemetryReader{Capacity: 1, AxisCount: 1, RecordBytes: 136,
		incarnation: [2]uint64{1, 2}, mapping: make([]byte, 4096+144)}
	binary.LittleEndian.PutUint64(r.mapping[72:], 1)
	binary.LittleEndian.PutUint64(r.mapping[80:], 2)
	if _, err := r.ReadFrom(17, 1); err != nil {
		t.Fatal(err)
	}
	for _, offset := range []int{72, 80} {
		previous := binary.LittleEndian.Uint64(r.mapping[offset:])
		binary.LittleEndian.PutUint64(r.mapping[offset:], 3)
		batch, err := r.ReadFrom(17, 1)
		if !errors.Is(err, ErrTelemetryIdentity) || batch.NextSequence != 17 || batch.Written != 0 || len(batch.Records) != 0 {
			t.Fatalf("changed identity exposed evidence: %+v %v", batch, err)
		}
		if head, err := r.Written(); !errors.Is(err, ErrTelemetryIdentity) || head != 0 {
			t.Fatalf("changed identity exposed head: %d %v", head, err)
		}
		binary.LittleEndian.PutUint64(r.mapping[offset:], previous)
	}
}

// The disk worker must lower only its own scheduling priority; accidentally
// lowering the cycle or metrics owner would destroy unrelated timing evidence.
func TestTelemetryWriterScheduling(t *testing.T) {
	f := startCore(t, "simulation.json")
	pid := storageDaemonPID(t, f.client)
	// Unverified observation deadline: 5000 ms for the named worker to start.
	if !simfixture.PollUntil(time.Now().Add(5*time.Second), time.Millisecond, func() bool {
		paths, err := filepath.Glob(fmt.Sprintf("/proc/%d/task/*/comm", pid))
		if err != nil {
			t.Fatal(err)
		}
		found := false
		for _, path := range paths {
			raw, err := os.ReadFile(path)
			if err != nil {
				t.Fatal(err)
			}
			tid, err := strconv.Atoi(filepath.Base(filepath.Dir(path)))
			if err != nil {
				t.Fatal(err)
			}
			attr, err := unix.SchedGetAttr(tid, 0)
			if err != nil {
				t.Fatal(err)
			}
			name := strings.TrimSpace(string(raw))
			if name == "telemetry-dump" {
				// Unverified worker scheduling contract: SCHED_BATCH and nice 19.
				if attr.Policy != unix.SCHED_BATCH || attr.Nice != 19 {
					return false
				}
				found = true
			} else if attr.Nice != 0 {
				t.Fatalf("%s priority changed: policy=%d nice=%d", name, attr.Policy, attr.Nice)
			}
		}
		return found
	}) {
		t.Fatal("telemetry worker did not acquire batch/nice 19 scheduling")
	}
}

// Busy headers preserve the exact cursor; committed corrupt payloads require
// an explicit caller decision, preventing silent loss or a perpetual empty poll.
func TestTelemetryTornHeaderAndCorruptSlot(t *testing.T) {
	for _, repair := range []bool{false, true} {
		loads := 0
		written, overwritten, ok := sampleTelemetryHeader(func(offset uint64) uint64 {
			loads++
			if offset == 56 {
				return 2
			}
			if repair && loads >= 6 {
				return 1
			}
			return 2
		}, 1)
		if repair {
			if !ok || written != 2 || overwritten != 1 || loads != 6 {
				t.Fatalf("retry repair: %d %d %d %v", loads, written, overwritten, ok)
			}
		} else if ok || loads != 16 || written != 0 || overwritten != 0 {
			t.Fatalf("retry bound: loads=%d ok=%v", loads, ok)
		}
	}
	r := &TelemetryReader{Capacity: 1, AxisCount: 1, RecordBytes: 152, mapping: make([]byte, 4096+160)}
	put := func(offset int, value uint64) { binary.LittleEndian.PutUint64(r.mapping[offset:], value) }
	put(56, 2)
	put(64, 2)
	b, err := r.ReadFrom(2, 1)
	if !errors.Is(err, ErrTelemetryBusy) || b.NextSequence != 2 || len(b.Records) != 0 || b.Written != 0 {
		t.Fatalf("torn pair: %+v %v", b, err)
	}
	if written, err := r.Written(); !errors.Is(err, ErrTelemetryBusy) || written != 0 {
		t.Fatalf("torn Written: %d %v", written, err)
	}
	put(64, 1)
	put(4096, 4)
	raw := EncodeCycleCaptureRecordV2(CycleCaptureRecordV2{Seq: 2, Axes: 2})
	copy(r.mapping[4104:], raw[:152])
	b, err = r.ReadFrom(2, 1)
	var corrupt *TelemetryCorruptSlotError
	if !errors.As(err, &corrupt) || corrupt.Sequence != 2 || b.NextSequence != 2 || len(b.Records) != 0 {
		t.Fatalf("corrupt slot: %+v %v", b, err)
	}
	raw = EncodeCycleCaptureRecordV2(CycleCaptureRecordV2{Seq: 2, Axes: 1})
	copy(r.mapping[4104:], raw[:152])
	b, err = r.ReadFrom(2, 1)
	if err != nil || b.NextSequence != 3 || len(b.Records) != 1 {
		t.Fatalf("repaired slot: %+v %v", b, err)
	}
}

// Poll observed sequence advancement, never elapsed sleep, to establish that
// telemetry actually traversed new pages on the cyclic thread under test.
func TestTelemetryLiveContinuityAndNoCycleFaults(t *testing.T) {
	f := startCore(t, "simulation.json")
	arm(t, f.client, 3)
	wait(t, f.client, allOperationEnabled)
	reader, err := OpenTelemetry(filepath.Join(filepath.Dir(f.options.SocketPath), "telemetry.ring"))
	if err != nil {
		t.Fatal(err)
	}
	defer reader.Close()
	// Wire measurement: 272-byte prefix plus two 320-byte diagnostic axis records.
	if reader.Capacity != 100000 || reader.RecordBytes != 912 || reader.AxisCount != 2 {
		t.Fatalf("geometry: capacity=%d record_bytes=%d axes=%d", reader.Capacity, reader.RecordBytes, reader.AxisCount)
	}
	pid := storageDaemonPID(t, f.client)
	tid := storageCycleTID(t, pid)
	before := storageReadFaults(t, pid, tid)
	initial, err := reader.ReadFrom(0, 1)
	if err != nil {
		t.Fatal(err)
	}
	sequence := initial.Written + 1
	var cycles, previousCycle uint64
	if !simfixture.PollUntil(time.Now().Add(10*time.Second), time.Millisecond, func() bool {
		batch, err := reader.ReadFrom(sequence, 256)
		if err != nil {
			t.Fatal(err)
		}
		if batch.Drops != 0 || batch.Overwritten != 0 {
			t.Fatalf("unexpected loss: %+v", batch)
		}
		for _, record := range batch.Records {
			if (sequence != 0 && record.Seq != sequence) || (previousCycle != 0 && record.Cycle != previousCycle+1) ||
				record.WorkNs == 0 || record.TNs == 0 || record.Axes != 2 {
				t.Fatalf("invalid record: %+v", record)
			}
			sequence = record.Seq + 1
			previousCycle = record.Cycle
			cycles++
		}
		return cycles >= 2000
	}) {
		t.Fatal("did not observe 2000 cycles")
	}
	after := storageReadFaults(t, pid, tid)
	if after != before {
		t.Fatalf("telemetry cycle page faults: before=%+v after=%+v", before, after)
	}
	t.Logf("records=%d cycle minor_fault_delta=0 major_fault_delta=0 ring_bytes=%d", cycles,
		TelemetryHeaderBytes+reader.Capacity*(reader.RecordBytes+8))
}

func TestTelemetryStalledReaderReportsDrops(t *testing.T) {
	f := startCoreWithArgs(t, "simulation.json", "--telemetry-retention-ms", "100")
	reader, err := OpenTelemetry(filepath.Join(filepath.Dir(f.options.SocketPath), "telemetry.ring"))
	if err != nil {
		t.Fatal(err)
	}
	defer reader.Close()
	if reader.Capacity != 100 {
		t.Fatalf("capacity=%d", reader.Capacity)
	}
	var batch TelemetryBatch
	if !simfixture.PollUntil(time.Now().Add(5*time.Second), time.Millisecond, func() bool {
		batch, err = reader.ReadFrom(1, 100)
		if errors.Is(err, ErrTelemetryBusy) {
			return false
		}
		if err != nil {
			t.Fatal(err)
		}
		return batch.Written >= 300 && len(batch.Records) != 0
	}) {
		t.Fatal("no wrap observed")
	}
	if batch.Drops != batch.Written-100 || batch.Records[0].Seq != batch.Drops+1 ||
		batch.Overwritten != batch.Drops {
		t.Fatalf("incorrect loss accounting: %+v", batch)
	}
	for i := 1; i < len(batch.Records); i++ {
		if batch.Records[i].Seq != batch.Records[i-1].Seq+1 {
			t.Fatal("gap within stable batch")
		}
	}
}

func TestTelemetryReadOnlyDescriptorBridge(t *testing.T) {
	f := startCore(t, "simulation.json")
	file, err := openTelemetryDescriptor(filepath.Join(filepath.Dir(f.options.SocketPath), "telemetry.ring.sock"))
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	writable, err := unix.Mmap(int(file.Fd()), 0, 4096, unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err == nil {
		unix.Munmap(writable)
		t.Fatal("observer descriptor allowed writable shared memory")
	}
	reader, err := mapTelemetry(file)
	if err != nil {
		t.Fatal(err)
	}
	defer reader.Close()
	batch, err := reader.ReadFrom(0, 16)
	if err != nil || len(batch.Records) != 16 {
		t.Fatalf("descriptor bridge records=%d err=%v", len(batch.Records), err)
	}
}

func telemetryAwaitDump(t *testing.T, directory, reason string) (string, TelemetryDumpIndex) {
	t.Helper()
	var path string
	var index TelemetryDumpIndex
	if !simfixture.PollUntil(time.Now().Add(5*time.Second), time.Millisecond, func() bool {
		files, _ := filepath.Glob(filepath.Join(directory, "telemetry", "*-"+reason+".bin"))
		if len(files) == 0 {
			return false
		}
		path = files[len(files)-1]
		var err error
		index, err = ReadTelemetryDump(path, nil)
		if err != nil {
			t.Fatal(err)
		}
		return true
	}) {
		t.Fatalf("no %s dump", reason)
	}
	raw, err := os.ReadFile(path[:len(path)-4] + ".json")
	if err != nil {
		t.Fatal(err)
	}
	var sibling TelemetryDumpIndex
	if err = json.Unmarshal(raw, &sibling); err != nil || sibling.FirstSequence != index.FirstSequence ||
		sibling.LastSequence != index.LastSequence || sibling.RecordCount != index.RecordCount {
		t.Fatalf("index mismatch: %s %v", raw, err)
	}
	if index.RecordCount == 0 || index.RecordCount+index.MissingRecords > 100 {
		t.Fatalf("unbounded/empty window: %+v", index)
	}
	return path, index
}

func TestTelemetryMarksStopRetentionAndFault(t *testing.T) {
	f := startCoreWithArgs(t, "simulation.json", "--telemetry-retention-ms", "100", "--telemetry-dump-count", "2")
	directory := filepath.Dir(f.options.SocketPath)
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	for _, invalid := range []string{"", string([]byte{0xff}), string(make([]byte, 129))} {
		if _, err := f.client.MarkTelemetry(ctx, invalid); err == nil {
			t.Fatal("invalid label admitted")
		}
	}
	var previous string
	for _, label := range []string{"first <>& \"quoted\"\nlabel", "../second λ", "third"} {
		if _, err := f.client.MarkTelemetry(ctx, label); err != nil {
			t.Fatal(err)
		}
		if !simfixture.PollUntil(time.Now().Add(5*time.Second), time.Millisecond, func() bool {
			files, _ := filepath.Glob(filepath.Join(directory, "telemetry", "*-mark.bin"))
			for _, path := range files {
				if path == previous {
					continue
				}
				index, err := ReadTelemetryDump(path, nil)
				if err == nil && index.Label == label {
					previous = path
					return true
				}
			}
			return false
		}) {
			t.Fatalf("missing mark %q", label)
		}
	}
	files, _ := filepath.Glob(filepath.Join(directory, "telemetry", "*.bin"))
	if len(files) != 2 {
		t.Fatalf("retained dump count=%d", len(files))
	}
	// Native disarm is the same urgent cancellation/Stop path as rt-control.
	if err := f.client.Disarm(); err != nil {
		t.Fatal(err)
	}
	telemetryAwaitDump(t, directory, "stop")
}

func TestTelemetryFaultAndExplicitExitDumps(t *testing.T) {
	t.Run("fault", func(t *testing.T) {
		f := startCoreWithArgs(t, "simulation.json", "--telemetry-retention-ms", "100",
			"--sim-send-fail-from-call", "1000", "--sim-send-fail-for-calls", "1")
		telemetryAwaitDump(t, filepath.Dir(f.options.SocketPath), "fault")
	})
	t.Run("exit", func(t *testing.T) {
		f := startCoreWithArgs(t, "simulation.json", "--telemetry-retention-ms", "100", "--telemetry-dump-on-exit")
		pid := storageDaemonPID(t, f.client)
		if err := unix.Kill(pid, unix.SIGTERM); err != nil {
			t.Fatal(err)
		}
		telemetryAwaitDump(t, filepath.Dir(f.options.SocketPath), "exit")
	})
	t.Run("separate-dump-directory", func(t *testing.T) {
		directory := t.TempDir()
		f := startCoreWithArgs(t, "simulation.json", "--telemetry-retention-ms", "100",
			"--telemetry-directory", filepath.Join(directory, "telemetry"), "--telemetry-dump-on-exit")
		pid := storageDaemonPID(t, f.client)
		if err := unix.Kill(pid, unix.SIGTERM); err != nil {
			t.Fatal(err)
		}
		telemetryAwaitDump(t, directory, "exit")
		if _, err := os.Stat(filepath.Join(filepath.Dir(f.options.SocketPath), "telemetry")); !os.IsNotExist(err) {
			t.Fatalf("dump used the socket directory: %v", err)
		}
	})
}
