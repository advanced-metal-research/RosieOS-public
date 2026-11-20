//go:build linux

package faultmatrix

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"syscall"
	"testing"
	"time"

	ipc "rosieos/rt-core/ipcclient"
)

// A simulated IgH mailbox call that needs mailboxSends further cyclic sends
// to return: 200 ms at the the 1430 cell 1 ms cycle, so a call is in flight for most
// of startup and a stop can be issued deterministically inside one.
const mailboxSends = 200

// Unverified bound in absolute time from SIGTERM to process exit: the 250 ms
// minimum disabled grace, one in-flight call (200 ms at 1 ms cycles), worker
// and thread joins, with margin for an unloaded test host.
const mailboxStopBound = 2 * time.Second

type mailboxCall struct{ enter, exit uint64 }

// mailboxCalls pairs simulator enter/exit lines in log order. The worker is the
// only runtime mailbox caller and holds one call at a time.
func mailboxCalls(t *testing.T, path string) []mailboxCall {
	t.Helper()
	raw, err := os.ReadFile(path)
	must(t, err)
	var calls []mailboxCall
	for _, line := range strings.Split(string(raw), "\n") {
		var at uint64
		if _, err := fmt.Sscanf(line, "sim_mailbox_enter time_ns=%d", &at); err == nil {
			calls = append(calls, mailboxCall{enter: at})
		} else if _, err := fmt.Sscanf(line, "sim_mailbox_exit time_ns=%d", &at); err == nil {
			if len(calls) == 0 || calls[len(calls)-1].exit != 0 {
				t.Fatalf("mailbox exit without a matching enter: %q", line)
			}
			calls[len(calls)-1].exit = at
		}
	}
	return calls
}

// example-servo, 2026-10-07 05:22 UTC: rt-control stopped (the core restarted its
// commissioning worker), the core got SIGTERM 350 ms later, and the worker was
// left in ecrt_master_sdo_upload after the cyclic exchange had stopped. IgH
// completes that request only while the application sends, so the process
// could not exit and held the EtherCAT master until a reboot. A stop issued
// while a call is in flight (with the controller connected, while the
// connection is being torn down, or after a fresh worker started) must keep
// exchanging until the call returns, then exit cleanly within the bound.
func TestShutdownKeepsExchangeUntilMailboxCallReturns(t *testing.T) {
	for _, scenario := range []string{"connected", "during_disconnect", "after_disconnect"} {
		t.Run(scenario, func(t *testing.T) {
			f := compileFixture(t, "simulation-rosie1420.json", nil)
			var err error
			f.log, err = os.Create(f.logPath)
			must(t, err)
			f.process = exec.Command(f.binary, append(append([]string{}, f.args...),
				"--sim-mailbox-cycle-sends", fmt.Sprint(mailboxSends))...)
			f.process.Stdout, f.process.Stderr = f.log, f.log
			must(t, f.process.Start())
			ctx, cancel := context.WithTimeout(context.Background(), watchdog)
			defer cancel()
			deadline := time.Now().Add(watchdog)
			for f.c == nil {
				if c, dialErr := ipc.Dial(ctx, f.options); dialErr == nil {
					f.c = c
				} else if time.Now().After(deadline) {
					t.Fatalf("dial: %v", dialErr)
				} else {
					pause()
				}
			}
			// Wait for a call that starts after `after` and has not returned.
			inFlight := func(after uint64) {
				t.Helper()
				for {
					calls := mailboxCalls(t, f.logPath)
					if n := len(calls); n > 0 && calls[n-1].exit == 0 && calls[n-1].enter >= after {
						return
					}
					if time.Now().After(deadline) {
						t.Fatal("no simulated mailbox call observed in flight")
					}
					pause()
				}
			}
			switch scenario {
			case "connected":
				inFlight(0)
			case "during_disconnect":
				inFlight(0)
				must(t, f.c.Close())
				f.c = nil
			case "after_disconnect":
				must(t, f.c.Close())
				f.c = nil
				inFlight(ipc.HostMonotonicNS())
			}
			stopped := ipc.HostMonotonicNS()
			must(t, f.process.Process.Signal(syscall.SIGTERM))
			done := make(chan error, 1)
			go func() { done <- f.process.Wait() }()
			select {
			case err = <-done:
			case <-time.After(watchdog):
				_ = f.process.Process.Kill()
				<-done
				f.process = nil
				t.Fatal("daemon did not exit: a mailbox call was left without cyclic exchange")
			}
			exited := ipc.HostMonotonicNS()
			f.process = nil
			must(t, f.log.Close())
			must(t, err)
			if elapsed := time.Duration(exited - stopped); elapsed > mailboxStopBound {
				t.Fatalf("stop took %v, bound %v", elapsed, mailboxStopBound)
			}
			calls := mailboxCalls(t, f.logPath)
			spanned := false
			for _, call := range calls {
				if call.exit == 0 {
					t.Fatalf("mailbox call entered at %d never returned", call.enter)
				}
				if call.exit > exited {
					t.Fatalf("mailbox call returned after process exit: %+v", call)
				}
				spanned = spanned || (call.enter < stopped && call.exit > stopped)
			}
			if !spanned {
				t.Fatal("stop was not issued inside a mailbox call; scenario not exercised")
			}
			log, err := os.ReadFile(f.logPath)
			must(t, err)
			for _, stage := range []string{"shutdown drain begin", "shutdown drain complete",
				"shutdown: workers joined", "shutdown: cyclic thread joined", "Stopped."} {
				if !strings.Contains(string(log), stage) {
					t.Fatalf("daemon journal lacks shutdown stage %q: %s", stage, log)
				}
			}
		})
	}
}

// This qualifies native worker exit acknowledgement and ongoing disabled cyclic
// exchange. The simulated mailbox returns after a fixed interval; it does not
// claim cancellation of an uninterruptible IgH kernel request.
func TestShutdownDrainsBlockedHomeMailbox(t *testing.T) {
	for _, blockMS := range []int{700, 5500} {
		t.Run(fmt.Sprintf("block_%dms", blockMS), func(t *testing.T) {
			f := compileFixture(t, "simulation-home.json", nil)
			f.launch(true, "--sim-sdo-block-ns", fmt.Sprint(blockMS*1000000), "--capture-cycles", "10000")
			_, err := f.c.Home(context.Background(), 1)
			must(t, err)
			blocked, _ := f.event("sim_sdo_block")
			stopped := ipc.HostMonotonicNS()
			must(t, f.process.Process.Signal(syscall.SIGTERM))
			done := make(chan error, 1)
			go func() { done <- f.process.Wait() }()
			select {
			case err = <-done:
			case <-time.After(watchdog):
				_ = f.process.Process.Kill()
				<-done
				f.process = nil
				t.Fatal("shutdown did not finish after simulated mailbox return")
			}
			f.process = nil // The fixture must not Wait on the process twice.
			must(t, f.log.Close())
			returned, _ := f.event("sim_sdo_return")
			if returned < blocked+uint64(blockMS)*1000000 {
				t.Fatal("mailbox did not exercise configured blocking interval")
			}
			if blockMS < 5000 {
				must(t, err)
			} else {
				var exit *exec.ExitError
				if !errors.As(err, &exit) || exit.ExitCode() != 2 {
					t.Fatalf("late mailbox return must preserve explicit failure exit 2: %v", err)
				}
			}
			log, err := os.ReadFile(f.logPath)
			must(t, err)
			if strings.Contains(string(log), "shutdown mailbox drain exceeded") != (blockMS > 5000) {
				t.Fatalf("unexpected drain budget diagnostic: %s", log)
			}
			raw, err := os.ReadFile(filepath.Join(f.socketDir, "cycle-capture.jsonl"))
			must(t, err)
			var last uint64
			samples := 0
			for _, line := range bytes.Split(raw, []byte{'\n'}) {
				if len(line) == 0 {
					continue
				}
				var sample struct {
					Time uint64 `json:"t_ns"`
					Axes []struct {
						Status uint16 `json:"sw"`
					} `json:"ax"`
				}
				must(t, json.Unmarshal(line, &sample))
				// Skip the disable transition itself. The pure cyclic test checks the
				// immediate zero controlword; here verify the plant stays disabled while
				// the mailbox is still blocked beyond the original 250 ms grace.
				if sample.Time < stopped+250000000 {
					continue
				}
				for _, axis := range sample.Axes {
					if axis.Status&0x6f != 0x40 {
						t.Fatalf("axis was not disabled during shutdown: %+v", sample)
					}
				}
				samples++
				last = sample.Time
			}
			if samples < 2 || last < returned {
				t.Fatalf("cyclic traffic did not cover mailbox return: samples=%d last=%d returned=%d", samples, last, returned)
			}
		})
	}
}
