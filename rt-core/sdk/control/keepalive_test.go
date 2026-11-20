//go:build linux

package control

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
	"time"
)

// TestKeepAliveConnectedWithoutGrant uses real idle-close policy and request
// receipts to prove every HTTP reservation survives without acquisition.
func TestKeepAliveConnectedWithoutGrant(t *testing.T) {
	var mu sync.Mutex
	counts := map[string]int{}
	opened := 0
	progress := make(chan struct{}, 1)
	server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Method != "GET" {
			t.Errorf("maintenance mutated authority: %s", r.Method)
		}
		if r.URL.Path == "/v1/jog/ingress" {
			mu.Lock()
			counts[r.RemoteAddr]++
			mu.Unlock()
			select {
			case progress <- struct{}{}:
			default:
			}
		}
		json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: json.RawMessage(`{"control_idle_timeout_ns":900000000}`)})
	}))
	// Unverified test policy: 900 ms idle limit and 300 ms maintenance cadence.
	server.Config.IdleTimeout = 900 * time.Millisecond
	server.Config.ConnContext = func(ctx context.Context, _ net.Conn) context.Context { mu.Lock(); opened++; mu.Unlock(); return ctx }
	server.Start()
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	_, err := c.Describe(context.Background())
	must(t, err)
	// Unverified watchdog: 6000 ms; seven peers each receiving six probes prove survival.
	ctx, cancel := context.WithTimeout(context.Background(), 6*time.Second)
	defer cancel()
	for {
		select {
		case <-progress:
		case <-ctx.Done():
			t.Fatal(ctx.Err())
		}
		mu.Lock()
		complete := len(counts) == 7
		for _, n := range counts {
			complete = complete && n >= 6
		}
		n := opened
		mu.Unlock()
		if complete {
			if n != 7 || c.Grant().Session != "" {
				t.Fatalf("connections=%d grant=%+v", n, c.Grant())
			}
			break
		}
	}
	server.CloseClientConnections()
	// The next maintenance sweep repairs all seven closed reservations.
	for {
		select {
		case <-progress:
		case <-ctx.Done():
			t.Fatal(ctx.Err())
		}
		mu.Lock()
		complete := len(counts) == 14
		mu.Unlock()
		if complete {
			break
		}
	}
}

// TestKeepAliveCompiledLinkTiming requires Describe to supply the shared bound;
// legacy adapters retain the conservative read-timeout fallback.
func TestKeepAliveCompiledLinkTiming(t *testing.T) {
	for _, tc := range []struct {
		name string
		idle uint64
		want time.Duration
	}{
		{"lan", 90000000000, 30000000000},
		{"internet", 90000000000, 30000000000},
		{"legacy", 0, 10000000000},
		{"custom", 900000000, 300000000},
	} {
		t.Run(tc.name, func(t *testing.T) {
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				d := Description{ControlIdleTimeoutNS: tc.idle}
				body, _ := json.Marshal(d)
				json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: body})
			}))
			defer server.Close()
			c := newClient(server.URL, &http.Transport{})
			defer c.Close()
			_, err := c.Describe(context.Background())
			must(t, err)
			c.mu.Lock()
			interval := c.keepAliveIntervalLocked()
			c.mu.Unlock()
			if interval != tc.want {
				t.Fatalf("keepalive_ns=%d", interval)
			}
		})
	}
}
