//go:build linux

package ipcclient

import (
	"bufio"
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"syscall"
	"testing"
	"time"
)

// Inspect every sampled status over a daemon-clock window. Elapsed wall time
// alone cannot satisfy this assertion or conceal stale pre-restart metrics.
func (f *restartFixture) assertInhibited(after uint64, zeroPosition bool) restartStatus {
	f.t.Helper()
	first := f.status(func(s restartStatus) bool {
		return s.Core.TimeNS >= after && s.Core.Armed == 0 && s.Core.AxisEnableMask == 0
	})
	positions := append([]AxisStatus(nil), first.Core.Axes...)
	return f.status(func(s restartStatus) bool {
		if s.Core.TimeNS < after {
			return false
		}
		if s.Core.Armed != 0 || s.Core.AxisEnableMask != 0 || s.Grant.GrantActive != 0 ||
			s.Jog.Open != 0 || s.Jog.HasInput != 0 || !s.Motion.Done || s.Motion.QueueDepth != 0 ||
			(s.Motion.Mode != 0 && !(s.Motion.Mode == 2 && s.Motion.State == 5)) {
			f.t.Fatalf("restart restored native permission or motion: %+v", s)
		}
		for i, axis := range s.Core.Axes {
			want := positions[i].PositionCounts
			if zeroPosition {
				want = 0
			}
			if axis.PositionCounts != want {
				f.t.Fatalf("restart motion: axis=%d delta=%d counts at daemon=%d ns", i, axis.PositionCounts-want, s.Core.TimeNS)
			}
		}
		return s.Core.TimeNS >= first.Core.TimeNS+300_000_000
	})
}

func (f *restartFixture) stale(operation, expected string, fields map[string]any) {
	f.t.Helper()
	reply, err := f.service.request(operation, fields)
	if err != nil || reply.Error != expected {
		f.t.Fatalf("old %s: error=%q want=%q transport=%v", operation, reply.Error, expected, err)
	}
}

func TestAdapterRecoversStaleSocketAfterCrash(t *testing.T) {
	f := newRestartFixture(t)
	f.startAdapter()
	f.acquire()
	stopRenew := f.renew()
	f.arm()
	plan := f.prepare(4*time.Second, .08, "crash-plan")
	f.service.must("start_trajectory", map[string]any{"handle": plan})
	before := f.status(func(s restartStatus) bool {
		return s.Motion.TrajectoryID == plan && !s.Motion.Done && s.Core.Axes[0].PositionCounts >= 20
	})
	stopRenew()
	killed := HostMonotonicNS()
	f.adapter.signal(t, syscall.SIGKILL)
	for _, path := range []string{f.controlSocket, filepath.Join(f.dir, "jog.sock")} {
		if _, err := os.Lstat(path); err != nil {
			t.Fatalf("missing stale adapter socket: %v", err)
		}
		restartLock(t, path, false, f.adapter.cmd.Process.Pid)
	}
	// Observe connection-loss revocation before a successor exists.
	restartEventually(t, "dead adapter grant revocation", func() bool {
		s := f.diagnostic()
		return s.TimeNS > killed && s.Armed == 0 && s.AxisEnableMask == 0 && s.LeaseValid == 0
	})
	f.startAdapter()
	after := f.assertInhibited(killed, false)
	if after.DaemonIncarnation != before.DaemonIncarnation || after.AdapterIncarnation == before.AdapterIncarnation {
		t.Fatalf("wrong process incarnation transition: before=%+v after=%+v", before, after)
	}
	f.stale("renew", "fence", nil)
	f.stale("start_trajectory", "fence", map[string]any{"handle": plan})
	f.acquire()
	f.service.must("renew", nil)
	f.adapter.signal(t, syscall.SIGTERM)
	for _, path := range []string{f.controlSocket, filepath.Join(f.dir, "jog.sock")} {
		if _, err := os.Lstat(path); !errors.Is(err, os.ErrNotExist) {
			t.Fatalf("clean adapter left socket: %v", err)
		}
		restartLock(t, path, false, f.adapter.cmd.Process.Pid)
	}
}

func TestDaemonRestartFencesSessionsAndHandles(t *testing.T) {
	f := newRestartFixture(t)
	f.startAdapter()
	f.acquire()
	stopRenew := f.renew()
	f.arm()
	plan := f.prepare(4*time.Second, .08, "plan")
	prepared := f.prepare(time.Second, .02, "prepared")
	f.service.must("start_trajectory", map[string]any{"handle": plan, "request_id": "start"})
	before := f.status(func(s restartStatus) bool {
		return s.Motion.TrajectoryID == plan && !s.Motion.Done && s.Core.Axes[0].PositionCounts >= 20
	})
	f.core.signal(t, syscall.SIGKILL)
	stopRenew()
	f.startCore()
	after := f.status(func(s restartStatus) bool {
		return s.DaemonIncarnation != before.DaemonIncarnation && s.Core.TimeNS >= f.launchNS
	})
	if len(after.DaemonIncarnation) != 32 || len(after.AdapterIncarnation) != 32 || after.AdapterIncarnation != before.AdapterIncarnation {
		t.Fatalf("wrong incarnation transition: before=%+v after=%+v", before, after)
	}
	if after.Execution.State != "released" || after.Execution.Error != "daemon_restarted" || len(after.Execution.Handles) != 2 {
		t.Fatalf("public restart transition: %+v", after.Execution)
	}
	for _, h := range after.Execution.Handles {
		if h.State != "retired" {
			t.Fatalf("handle %d survived as %s", h.Handle, h.State)
		}
	}
	if after.Motion.TrajectoryID != 0 || after.Motion.CommandSequence != 0 {
		t.Fatalf("new daemon retained old execution: %+v", after.Motion)
	}
	f.assertInhibited(f.launchNS, true)
	f.stale("renew", "daemon_restarted", nil)
	f.stale("start_trajectory", "daemon_restarted", map[string]any{"handle": plan, "request_id": "start"})
	f.stale("start_trajectory", "daemon_restarted", map[string]any{"handle": prepared})
	f.stale("release", "daemon_restarted", nil)
	f.stale("begin_jog", "daemon_restarted", map[string]any{"clock_incarnation": before.DaemonIncarnation})
	f.acquire()
	defer f.renew()()
	f.stale("start_trajectory", "handle_retired", map[string]any{"handle": plan})
	f.arm()
	// Reusing the old idempotence keys with fresh content proves the table was
	// detached; a cached Start cannot masquerade as new execution.
	fresh := f.prepare(time.Second, .02, "plan")
	if fresh <= prepared {
		t.Fatalf("public handle identity reused: %d <= %d", fresh, prepared)
	}
	f.service.must("start_trajectory", map[string]any{"handle": fresh, "request_id": "start"})
	done := f.status(func(s restartStatus) bool {
		return s.Motion.TrajectoryID == fresh && s.Motion.Done && s.Motion.State == 4 &&
			s.Core.Axes[0].PositionCounts >= 410 && s.Core.Axes[1].PositionCounts <= -410
	})
	if done.Core.Axes[0].PositionCounts < 410 || done.Core.Axes[1].PositionCounts > -410 {
		t.Fatalf("fresh execution did not move 410 counts: %+v", done.Core.Axes)
	}
}

func TestRestartNeverReplaysPlanOrRestoresPermission(t *testing.T) {
	f := newRestartFixture(t)
	f.startAdapter()
	f.acquire()
	stopRenew := f.renew()
	f.arm()
	plan := f.prepare(4*time.Second, .08, "never-replay")
	f.service.must("start_trajectory", map[string]any{"handle": plan})
	before := f.status(func(s restartStatus) bool {
		return s.Motion.TrajectoryID == plan && !s.Motion.Done && s.Core.Axes[0].PositionCounts >= 20
	})
	f.core.signal(t, syscall.SIGKILL)
	stopRenew()
	trace := filepath.Join(f.dir, "cycle-capture.jsonl")
	f.args = append(f.args, "--capture-cycles", "10000")
	f.startCore()
	f.status(func(s restartStatus) bool { return s.DaemonIncarnation != before.DaemonIncarnation })
	after := f.assertInhibited(f.launchNS, true)
	file, err := os.Open(trace)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	scanner := bufio.NewScanner(file)
	samples := 0
	for scanner.Scan() {
		var sample struct {
			TimeNS uint64 `json:"t_ns"`
			Axes   []struct {
				Position   int32  `json:"p"`
				Target     int32  `json:"tp"`
				Statusword uint16 `json:"sw"`
			} `json:"ax"`
		}
		if err := json.Unmarshal(scanner.Bytes(), &sample); err != nil {
			t.Fatal(err)
		}
		if sample.TimeNS > after.Core.TimeNS {
			break
		}
		if sample.TimeNS < f.launchNS {
			continue
		}
		for _, axis := range sample.Axes {
			if axis.Position != 0 || axis.Target != 0 || axis.Statusword&0x6f == 0x27 {
				t.Fatalf("restart trace restored motion/output at %d ns: %+v", sample.TimeNS, axis)
			}
		}
		samples++
	}
	if err := scanner.Err(); err != nil {
		t.Fatal(err)
	}
	if samples < 20 {
		t.Fatalf("only %d fresh restart trace samples; require 20", samples)
	}
	t.Logf("%d daemon trace samples: 0 position counts, 0 target counts, armed=0 through %d ns", samples, after.Core.TimeNS)
}

func TestDaemonRestartClosesJogSession(t *testing.T) {
	f := newRestartFixture(t)
	f.startAdapter()
	f.acquire()
	stopRenew := f.renew()
	f.arm()
	f.service.begin()
	before := f.status(func(s restartStatus) bool { return s.Jog.Open == 1 })
	f.core.signal(t, syscall.SIGKILL)
	stopRenew()
	f.startCore()
	f.status(func(s restartStatus) bool { return s.DaemonIncarnation != before.DaemonIncarnation && s.Jog.Open == 0 })
	f.stale("end_jog", "daemon_restarted", map[string]any{"jog_generation": before.Jog.JogGeneration})
	f.assertInhibited(f.launchNS, true)
}
