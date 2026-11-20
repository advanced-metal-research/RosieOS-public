//go:build linux

package ipcclient

import (
	"context"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"rosieos/rt-core/internal/procfs"
	"strconv"
	"strings"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

// Measured LP64 layout, also pinned by trajectory_store_tests.cpp:
// three slots * (250,000 points * 320 bytes + 56-byte slot header).
const storageResidentPlanBytes = 3 * (250000*320 + 56)

// Measured generated layout: 4096 metadata bytes + 100000 slots *
// (8-byte stamp + 272-byte prefix + 2 axes * 320 bytes) = 92004096 bytes.
const storageTelemetryMappingBytes = 4096 + 100000*(8+CycleCaptureRecordV2AxOffset+2*CycleCaptureAxisV2Bytes)

// Measured WSL page size is 4096 bytes: the mapping occupies 92004352 resident
// bytes, including its final partial page, outside the overhead allowance.
const storageResidentTelemetryBytes = (storageTelemetryMappingBytes + 4095) / 4096 * 4096

// Residual allowance measured on WSL/g++11 when mlockall was refused. Keep it
// fixed for allocator/page rounding, IPC, libraries and touched stack/diagnostic
// pages. Successful mlockall also makes the components below fully resident.
const storageDaemonOverheadBytes = 16 * 1024 * 1024

// LP64 diagnostics.hpp: 10,800 records * (two pointers + 24 * 112-byte
// arguments), plus two queue counters. Confirmed by the binary's ELF symbol.
const storageResidentDiagnosticsBytes = 10800*(16+24*112) + 16

// Six native workers, each with the Linux/g++11 default 8 MiB stack: rt-cycle,
// commissioning, telemetry-dump, metrics, bulk-worker and ipc-helper. Confirmed
// by /proc task stack pointers and their resident mappings under mlockall.
const storageResidentWorkerStackBytes = 6 * 8 * 1024 * 1024

// Sanitizer detection by address-space size. ASan reserves its shadow at
// process start; measured on this WSL/g++11 host the instrumented simulation
// daemon reports VmSize=21,990,609,575,936 bytes and the uninstrumented one
// VmSize=418,570,240 bytes. No production daemon maps a terabyte. The declared resident budget is a
// contract of the shipped binary, and shadow pages touched by the sanitizer
// count toward VmHWM, so the bound is not meaningful for an instrumented run.
const storageSanitizerShadowBytes = 1 << 40

type storageMemory struct {
	rss, hwm, locked, size uint64
}

// Read the connected daemon's peer identity; never sample the Go test process
// or a different concurrently running daemon's memory/fault counters.
func storageDaemonPID(t *testing.T, c *Client) int {
	t.Helper()
	raw, err := c.conn.SyscallConn()
	if err != nil {
		t.Fatal(err)
	}
	var peer *unix.Ucred
	var peerErr error
	if err := raw.Control(func(fd uintptr) {
		peer, peerErr = unix.GetsockoptUcred(int(fd), unix.SOL_SOCKET, unix.SO_PEERCRED)
	}); err != nil {
		t.Fatal(err)
	}
	if peerErr != nil {
		t.Fatal(peerErr)
	}
	return int(peer.Pid)
}

// Require all three kernel counters in bytes; missing /proc evidence must fail
// rather than silently turn the memory bound into a zero-valued pass.
func storageReadMemory(t *testing.T, pid int, stage string) storageMemory {
	t.Helper()
	values, err := procfs.MemoryCounters(pid, "VmRSS:", "VmHWM:", "VmLck:", "VmSize:")
	if err != nil {
		t.Fatal(err)
	}
	memory := storageMemory{values["VmRSS:"], values["VmHWM:"], values["VmLck:"], values["VmSize:"]}
	t.Logf("%s: VmRSS=%d bytes VmHWM=%d bytes VmLck=%d bytes VmSize=%d bytes plans=%d bytes telemetry=%d bytes overhead=%d bytes",
		stage, memory.rss, memory.hwm, memory.locked, memory.size, storageResidentPlanBytes, storageResidentTelemetryBytes, storageDaemonOverheadBytes)
	return memory
}

// Full-capacity stationary points touch all slot pages during decoding without
// coupling the memory test to unqualified moving-axis limits.
func storageFullPlan() []Point {
	points := make([]Point, 250000)
	position := []float64{0, 0}
	for i := range points {
		points[i] = Point{Time: time.Duration(i) * 10 * time.Microsecond, Position: position}
	}
	return points
}

func TestDaemonResidentMemoryWithinDeclaredBudget(t *testing.T) {
	f := startCore(t, "simulation.json")
	c := f.client
	pid := storageDaemonPID(t, c)
	samples := []storageMemory{storageReadMemory(t, pid, "startup")}
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	points := storageFullPlan()
	for id := uint64(18001); id <= 18003; id++ {
		if handle, err := c.PrepareTrajectory(ctx, id, 3, points); err != nil || handle != id {
			t.Fatalf("prepare %d: handle=%d err=%v", id, handle, err)
		}
		samples = append(samples, storageReadMemory(t, pid, fmt.Sprintf("prepared-%d", id)))
	}
	_, err := c.PrepareTrajectory(ctx, 18004, 3, points[:2])
	var rejected *RejectedError
	if !errors.As(err, &rejected) || rejected.Result.Reason != ReasonCapacity {
		t.Fatalf("fourth preparation must reject capacity: %v", err)
	}
	arm(t, c, 3)
	wait(t, c, allOperationEnabled)
	if _, err := c.StartTrajectory(ctx, 18001); err != nil {
		t.Fatal(err)
	}
	samples = append(samples, storageReadMemory(t, pid, "started-one"))
	if _, err := c.DiscardTrajectory(ctx, 18002); err != nil {
		t.Fatal(err)
	}
	samples = append(samples, storageReadMemory(t, pid, "discarded-one"))
	for _, sample := range samples {
		if sample.rss < storageResidentPlanBytes+storageResidentTelemetryBytes {
			t.Errorf("VmRSS=%d bytes does not contain prefaulted plans=%d and telemetry=%d bytes", sample.rss, storageResidentPlanBytes, storageResidentTelemetryBytes)
		}
	}
	t.Run("declared-budget", func(t *testing.T) {
		if samples[0].size > storageSanitizerShadowBytes {
			t.Skipf("sanitizer shadow mapping present (VmSize=%d bytes); the declared resident budget binds the production binary only, store containment, capacity refusal and locked-page assertions still run", samples[0].size)
		}
		for _, sample := range samples {
			budget := uint64(storageResidentPlanBytes + storageResidentTelemetryBytes + storageDaemonOverheadBytes)
			fullyLocked := sample.locked >= storageResidentPlanBytes+storageResidentTelemetryBytes+storageResidentDiagnosticsBytes+storageResidentWorkerStackBytes
			if fullyLocked {
				budget += storageResidentDiagnosticsBytes + storageResidentWorkerStackBytes
			}
			if sample.hwm > budget {
				t.Errorf("VmHWM=%d bytes exceeds declared total=%d bytes (fully_locked=%t, VmLck=%d)", sample.hwm, budget, fullyLocked, sample.locked)
			}
		}
	})
	t.Run("locked-plan-pages", func(t *testing.T) {
		kernel, err := os.ReadFile("/proc/sys/kernel/osrelease")
		if err != nil {
			t.Fatal(err)
		}
		log, err := os.ReadFile(filepath.Join(filepath.Dir(f.options.SocketPath), "core.log"))
		if err != nil {
			t.Fatal(err)
		}
		if samples[0].size > storageSanitizerShadowBytes {
			// engine/cycle_machine.hpp compiles the simulation mlockall out under
			// __SANITIZE_ADDRESS__ and __SANITIZE_THREAD__; nothing is locked to measure.
			t.Skipf("sanitizer shadow mapping present (VmSize=%d bytes); the sanitized simulation daemon does not call mlockall, store containment and capacity refusal still run", samples[0].size)
		}
		if strings.Contains(strings.ToLower(string(kernel)), "microsoft") &&
			strings.Contains(string(log), "Simulation plan prefault: mlockall refused errno=") {
			t.Skip("WSL refused daemon mlockall; plain prefaulting, RSS/HWM and cyclic fault assertions still run")
		}
		for _, sample := range samples {
			if sample.locked < storageResidentPlanBytes+storageResidentTelemetryBytes {
				t.Errorf("VmLck=%d bytes below declared resident plans=%d and telemetry=%d bytes", sample.locked, storageResidentPlanBytes, storageResidentTelemetryBytes)
			}
		}
	})
}

type storageFaults struct {
	minor, major uint64
}

// Select the actual cyclic thread by its native comm name, excluding worker
// faults that cannot establish or refute cyclic page-fault freedom.
func storageCycleTID(t *testing.T, pid int) string {
	t.Helper()
	tid, err := procfs.FirstCycleTID(pid)
	if err != nil {
		t.Fatal(err)
	}
	return tid
}

// Linux stat fields 10 and 12 are this thread's minflt and majflt. Parsing
// starts after comm so spaces/parentheses cannot shift the numeric fields.
func storageReadFaults(t *testing.T, pid int, tid string) storageFaults {
	t.Helper()
	id, err := strconv.Atoi(tid)
	if err != nil {
		t.Fatal(err)
	}
	faults, err := procfs.ThreadFaults(pid, id)
	if err != nil {
		t.Fatal(err)
	}
	return storageFaults{faults.Minor, faults.Major}
}

func TestCycleTakesNoPageFaultsDuringPlanTransfer(t *testing.T) {
	f := startCore(t, "simulation.json")
	c := f.client
	arm(t, c, 3)
	wait(t, c, allOperationEnabled)
	pid := storageDaemonPID(t, c)
	tid := storageCycleTID(t, pid)
	before := storageReadFaults(t, pid, tid)
	ctx, cancel := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancel()
	handle, err := c.PrepareTrajectory(ctx, 19001, 3, storageFullPlan())
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("after Prepare: %+v", storageReadFaults(t, pid, tid))
	sequence, err := c.StartTrajectory(ctx, handle)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("after Start: %+v", storageReadFaults(t, pid, tid))
	for {
		motion, err := c.Motion()
		if err != nil {
			t.Fatal(err)
		}
		if motion.TrajectoryID == handle && motion.CommandSequence == sequence && motion.Done {
			if motion.PointIndex != 249999 || motion.State != 4 {
				t.Fatalf("plan did not complete at its final point: %+v", motion)
			}
			break
		}
		select {
		case <-ctx.Done():
			t.Fatalf("missing correlated completion: %+v", motion)
		case <-time.After(time.Millisecond):
		}
	}
	t.Logf("after completion: %+v", storageReadFaults(t, pid, tid))
	if _, err := c.DiscardTrajectory(ctx, handle); err != nil {
		t.Fatal(err)
	}
	after := storageReadFaults(t, pid, tid)
	t.Logf("rt-cycle tid=%s minor=%d->%d major=%d->%d", tid, before.minor, after.minor, before.major, after.major)
	if after.minor != before.minor || after.major != before.major {
		t.Errorf("cyclic page faults: minor delta=%d major delta=%d; both must be zero", after.minor-before.minor, after.major-before.major)
	}
}
