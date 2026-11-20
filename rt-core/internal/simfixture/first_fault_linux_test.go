//go:build linux

package simfixture

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

// A deadline-cancelled Home must be named with its reason, abort code and the
// measured cycle lateness, printed apart from the configured lateness limit; a
// precondition abort that never reaches the worker's failure record must still
// be named; a file no daemon refreshes must say "stale".
// Field names are status_publisher.hpp's metrics.json keys.
func TestFirstFaultNamesHomeDeadline(t *testing.T) {
	metrics := filepath.Join(t.TempDir(), "metrics.json")
	raw := `{"time_ns":5,"rt_max_abs_jitter_ns":69942000,"rt_max_cycle_work_ns":80000,"rt_overrun_count":3,
"max_cycle_lateness_ns":20000000,"safety_fault_mask":4096,
"recovery":{"home_valid_mask":0,"faults":[{"bit":12,"name":"cycle_deadline","axis_mask":63}]},
"axes":[
 {"native_home_state":3,"native_home_last_abort_code":4043309068,"native_home_failure":{"axis":0,"step":2,"operation":"set_mode","reason":"cycle_deadline","cancellation_reason":"cycle_deadline","abort_code":4043309068}},
 {"native_home_state":3,"native_home_last_abort_code":4043309068,"native_home_failure":{"axis":1,"step":2,"operation":"set_mode","reason":"cycle_deadline","cancellation_reason":"cycle_deadline","abort_code":4043309068}},
 {"native_home_state":3,"native_home_last_abort_code":4043309057,"native_home_failure":null}]}`
	if err := os.WriteFile(metrics, []byte(raw), 0o600); err != nil {
		t.Fatal(err)
	}
	got := FirstFault(metrics, 50*time.Millisecond)
	for _, want := range []string{
		"native_home_failure=[{axes=0,1 step=2 op=set_mode reason=cycle_deadline cancellation=cycle_deadline abort=0xf100000c}]",
		"native_home_abort=[{axes=0,1 state=3 abort=0xf100000c} {axes=2 state=3 abort=0xf1000001}]",
		"faults=[cycle_deadline(axes=0x3f)]",
		"measured: rt_max_abs_jitter_ns=69942000 rt_overrun_count=3 rt_max_cycle_work_ns=80000;",
		"configured: cycle_lateness_limit_ns=20000000;",
		"(stale)",
	} {
		if !strings.Contains(got, want) {
			t.Fatalf("FirstFault = %q, missing %q", got, want)
		}
	}
	if got := FirstFault(filepath.Join(t.TempDir(), "absent.json"), 0); !strings.Contains(got, "metrics unavailable") {
		t.Fatalf("absent metrics: %q", got)
	}
}

// A daemon that keeps publishing must be reported "fresh", with the newest
// publication's evidence. The writer replaces the file by rename, as
// status_publisher.hpp does, and keeps publishing until the reader returns,
// so the reader waits on publications, not on this test's timing.
func TestFirstFaultFreshAfterTwoPublications(t *testing.T) {
	directory := t.TempDir()
	metrics := filepath.Join(directory, "metrics.json")
	publish := func(timeNS int) {
		raw := fmt.Sprintf(`{"time_ns":%d,"rt_max_abs_jitter_ns":%d,"max_cycle_lateness_ns":20000000,"axes":[]}`, timeNS, timeNS)
		tmp := metrics + ".tmp"
		if err := os.WriteFile(tmp, []byte(raw), 0o600); err != nil {
			t.Error(err)
			return
		}
		if err := os.Rename(tmp, metrics); err != nil {
			t.Error(err)
		}
	}
	publish(1)
	done, stopped := make(chan struct{}), make(chan struct{})
	go func() {
		defer close(stopped)
		// Unverified writer cadence: 10 ms, faster than the reader's 25 ms
		// poll, like any publisher the reader can keep up with.
		ticker := time.NewTicker(10 * time.Millisecond)
		defer ticker.Stop()
		for next := 2; ; next++ {
			select {
			case <-done:
				return
			case <-ticker.C:
				publish(next)
			}
		}
	}()
	// Unverified ceiling: it bounds only a broken reader; a working one
	// returns after its second fresh publication, about 50 ms.
	got := FirstFault(metrics, 5*time.Second)
	close(done)
	<-stopped
	if !strings.HasSuffix(got, "(fresh)") || strings.Contains(got, "rt_max_abs_jitter_ns=1 ") {
		t.Fatalf("FirstFault = %q, want a fresh report newer than the first read", got)
	}
}
