//go:build linux

package main

import (
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control"
)

// TestRemoteListenerTimeouts preserves LAN behavior and prevents the remote
// listener from bounding an internet TLS handshake with the old LAN deadline.
func TestRemoteListenerTimeouts(t *testing.T) {
	for _, tc := range []struct {
		name       string
		lease, age uint64
		want       time.Duration
	}{
		{"legacy_lan", 0, 0, 2000000000},
		{"lan", 500000000, 250000000, 2000000000},
		{"internet", 3000000000, 750000000, 10000000000},
		{"custom_short_lease", 100000000, 120000000, 10000000000},
		{"custom_lease", 10000000000, 250000000, 10000000000},
		{"custom_age", 500000000, 2000000000, 10000000000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			description := control.Description{}
			description.ControlIdleTimeoutNS = control.CompiledControlIdleTimeoutNS
			description.MaxGrantLeaseNS, description.MaxJogInputAgeNS = tc.lease, tc.age
			server := newRemoteServer(nil, description)
			if server.ReadHeaderTimeout != tc.want || server.ReadTimeout != 30000000000 || server.WriteTimeout != 65000000000 || server.IdleTimeout != 90000000000 {
				t.Fatalf("header=%d read=%d write=%d", server.ReadHeaderTimeout, server.ReadTimeout, server.WriteTimeout)
			}
		})
	}
}

// TestKeepAliveListenerUsesDescribe rejects a separately hard-coded idle bound:
// the listener must consume the exact timing published to reservation clients.
func TestKeepAliveListenerUsesDescribe(t *testing.T) {
	for _, idle := range []uint64{30000000000, 90000000000} {
		d := control.Description{ControlIdleTimeoutNS: idle}
		if got := newRemoteServer(nil, d).IdleTimeout; got != time.Duration(idle) {
			t.Fatalf("idle_timeout_ns=%d want=%d", got, idle)
		}
	}
}
