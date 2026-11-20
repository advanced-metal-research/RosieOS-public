//go:build linux

package faultmatrix

import (
	"context"
	ipc "rosieos/rt-core/ipcclient"
	"strings"
	"testing"
	"time"
)

func busFailure(t *testing.T, r row) {
	op := strings.TrimPrefix(r.name, "d_")
	codes := map[string]uint32{"send": 5, "queue": 7, "receive": 2, "dc": 4, "process": 6}
	f := start(t, "--sim-"+op+"-fail-from-call", "2000", "--sim-"+op+"-fail-for-calls", "1000")
	f.arm()
	f.execute(f.prepare(401, 0, 4*time.Second))
	at, _ := f.event("sim_" + op + "_failure")
	fault := f.until(at, r.boundNS, func(s ipc.Status) bool { return s.ExecutionFaultReasons&r.fault != 0 })
	inhibited(t, fault, r.fault)
	if fault.LastBusFailureOperation != codes[op] || fault.LastBusFailureCode != -5 || fault.SafetyFaultMask != 3 {
		t.Fatalf("wrong transport diagnosis: %+v", fault)
	}
	// A finite outage makes a successful eventual ACK meaningful; while send or
	// queue still fails the Stop call must remain outstanding across new status.
	stopped := make(chan error, 1)
	stopAt := ipc.HostMonotonicNS()
	go func() { _, err := f.c.Stop(context.Background()); stopped <- err }()
	if op == "send" || op == "queue" {
		during := f.until(fault.TimeNS+1, r.boundNS, func(s ipc.Status) bool { return true })
		inhibited(t, during, r.fault)
		select {
		case err := <-stopped:
			t.Fatalf("Stop returned during failed submission: %v", err)
		default:
		}
	}
	select {
	case err := <-stopped:
		must(t, err)
	case <-time.After(watchdog):
		t.Fatal("Stop did not acknowledge restored submission")
	}
	ackAt := ipc.HostMonotonicNS()
	ackOrigin := stopAt
	if op == "send" || op == "queue" {
		recovered, _ := f.event("sim_" + op + "_recovered")
		if ackAt < recovered {
			t.Fatal("Stop ACK preceded successful submission")
		}
		ackOrigin = recovered
	}
	if ackAt-ackOrigin > r.boundNS {
		t.Fatalf("Stop ACK exceeded bound after submission became possible: elapsed_ns=%d bound_ns=%d", ackAt-ackOrigin, r.boundNS)
	}
	s := f.until(ipc.HostMonotonicNS(), r.boundNS, func(s ipc.Status) bool { return s.Armed == 0 && s.AxisEnableMask == 0 })
	inhibited(t, s, r.fault)
}
