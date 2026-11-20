//go:build linux

package faultmatrix

import (
	"context"
	ipc "rosieos/rt-core/ipcclient"
	"testing"
	"time"
)

func outputLag(t *testing.T, r row) {
	f := start(t, "--sim-feedback-offset-counts", "-1000")
	f.command(f.c.Enable(context.Background(), 3))
	f.command(f.c.Arm(context.Background()))
	at, counts := f.event("sim_feedback_offset")
	if counts != -1000 {
		t.Fatal("lag injection did not apply -1000 counts")
	}
	deadline := time.Now().Add(watchdog)
	for {
		m, err := f.c.Motion()
		if err == nil && m.TimeNS >= at && m.State == 6 {
			if m.TimeNS-at < 100_000_000 || m.TimeNS-at > r.boundNS {
				t.Fatalf("following-error fault timestamp outside configured timeout/publication bound: elapsed_ns=%d", m.TimeNS-at)
			}
			break
		}
		if time.Now().After(deadline) {
			t.Fatal("missing faulted motion timestamp")
		}
		pause()
	}
	s := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.ExecutionFaultReasons&r.fault != 0 })
	inhibited(t, s, r.fault)
	if s.TimeNS < at+100_000_000 {
		t.Fatalf("following-error fired before configured 100000000 ns timeout: elapsed_ns=%d", s.TimeNS-at)
	}
	if s.ExecutionFaultReasons != followingFault {
		t.Fatalf("lag triggered unrelated fault: %+v", s)
	}
	f.hold(s, 400_000_000, 0)
}
