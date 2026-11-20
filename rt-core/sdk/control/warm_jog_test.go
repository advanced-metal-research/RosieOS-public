//go:build linux

package control

import (
	"context"
	"crypto/tls"
	"net/http"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
)

// TestReviewRemoteJogUsesReservedTLS exercises the actual verified WSS upgrade;
// the active upgrade must consume the reserved socket. Its replacement may
// handshake in the background for the next hold (the repeated-hold contract).
func TestReviewRemoteJogUsesReservedTLS(t *testing.T) {
	s, _, _, c := remoteJogService(t, true)
	var handshakes atomic.Int32
	for _, lane := range append([]*http.Client{c.control, c.urgent, c.bulk, c.renewal}, c.warmClients...) {
		lane.Transport.(*http.Transport).TLSClientConfig.VerifyConnection = func(tls.ConnectionState) error { handshakes.Add(1); return nil }
	}
	// Unverified test watchdog: 10000 ms; accepted upgrade and counts pass.
	ctx, cancel := context.WithTimeout(context.Background(), 10000*time.Millisecond)
	defer cancel()
	if _, err := c.Describe(ctx); err != nil {
		t.Fatal(err)
	}
	// Unverified remote classification stimulus: 21 ms, within the LAN lease.
	c.mu.Lock()
	c.roundTrip = 21 * time.Millisecond
	c.mu.Unlock()
	remoteJogReady(t, s, c, ctx)
	c.StopRenewal()
	before := handshakes.Load()
	if before != 8 {
		t.Fatalf("reserved TLS connections=%d", before)
	}
	must(t, c.jogSpareMu.LockContext(ctx))
	reserved := c.jogSpare
	c.jogSpareMu.Unlock()
	if reserved == nil {
		t.Fatal("missing reserved jog connection")
	}
	jog, err := c.NewRemoteJogSession(ctx, 511, 100000000, ipcclient.HostMonotonicNS)
	if err != nil {
		t.Fatal(err)
	}
	if jog.conn != reserved {
		t.Fatal("active WSS upgrade did not consume the reserved connection")
	}
	if err := jog.End(ctx); err != nil {
		t.Fatal(err)
	}
	poll(t, time.Second, func() bool {
		if err := c.jogSpareMu.LockContext(ctx); err != nil {
			return false
		}
		defer c.jogSpareMu.Unlock()
		return c.jogSpare != nil && c.jogSpare != reserved
	})
	if handshakes.Load() != before+1 {
		t.Fatalf("replacement TLS handshakes=%d, want one", handshakes.Load()-before)
	}
}
