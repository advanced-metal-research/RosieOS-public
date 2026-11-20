//go:build linux

package control

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"net/http/httptest"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// TestReviewStopKeepsMaintainedUrgentSocket cancels renewal during an independent
// maintenance probe; Stop must inherit its warm socket without replacing it.
func TestReviewStopKeepsMaintainedUrgentSocket(t *testing.T) {
	var opened atomic.Int32
	server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		payload, _ := json.Marshal(Grant{Session: "warm", Generation: 1, LeaseMS: 3000})
		json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: payload})
	}))
	server.Config.ConnContext = func(ctx context.Context, _ net.Conn) context.Context { opened.Add(1); return ctx }
	server.Start()
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	// Measured composed link: 250 ms RTT, 3000 ms lease ceiling.
	c.roundTrip, c.leaseCeiling = 250*time.Millisecond, 3000*time.Millisecond
	// Unverified test watchdog: 10000 ms; maintenance and Stop receipts pass.
	ctx, cancel := context.WithTimeout(context.Background(), 10000*time.Millisecond)
	defer cancel()
	if _, err := c.Acquire(ctx, "test", Binding{}); err != nil {
		t.Fatal(err)
	}
	entered := make(chan context.Context, 1)
	release := make(chan struct{})
	var releaseOnce sync.Once
	unblock := func() { releaseOnce.Do(func() { close(release) }) }
	defer unblock()
	transport := c.urgent.Transport
	c.urgent.Transport = roundTripFunc(func(r *http.Request) (*http.Response, error) {
		if r.URL.Path == "/v1/jog/ingress" {
			entered <- r.Context()
			select {
			case <-release:
			case <-r.Context().Done():
				return nil, r.Context().Err()
			}
		}
		return transport.RoundTrip(r)
	})
	defer transport.(*http.Transport).CloseIdleConnections()
	if _, err := c.StartRenewal(ctx, 0); err != nil {
		t.Fatal(err)
	}
	// Discovery normally starts maintenance; this fixture seeds its measurement.
	c.startKeepAlive()
	c.wakeKeepAlive()
	var maintenance context.Context
	select {
	case maintenance = <-entered:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	c.mu.Lock()
	stop := c.renewCancel
	c.mu.Unlock()
	stop()
	if maintenance.Err() != nil {
		t.Fatal("keeper cancellation destroyed the urgent maintenance request")
	}
	unblock()
	c.StopRenewal()
	if _, err := c.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	c.stopKeepAlive()
	if n := opened.Load(); n != 7 {
		t.Fatalf("connections=%d; Stop replaced its maintained reservation", n)
	}
}
