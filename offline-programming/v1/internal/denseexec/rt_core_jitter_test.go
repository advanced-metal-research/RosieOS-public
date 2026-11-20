//go:build linux

package denseexec

import (
	"bytes"
	"context"
	"encoding/json"
	"io"
	"net"
	"net/http"
	"net/http/httputil"
	"os/exec"
	"path/filepath"
	"sync"
	"sync/atomic"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

type jitterConnections struct {
	opened            atomic.Int32
	mu                sync.Mutex
	acquire           int32
	renewals          int
	coldRenewals      int
	wrongLane         int
	enable            chan struct{}
	once              sync.Once
	acquired          bool
	coldRequests      []string
	active            int
	peak              int
	peakHeld          int
	delayStatus       bool
	slowFinished      chan struct{}
	statusCalls       int
	requests          []string
	armEntered        chan struct{}
	armRelease        chan struct{}
	halfOpenPath      string
	halfOpenRemaining int
	halfOpenEntered   chan struct{}
}

type jitterConnection struct {
	id       int32
	requests int
}
type jitterContextKey struct{}

// delayedJointFixture adds transport latency before adapter admission. The
// proxy retains the native jog socket directory, so local input stays local.
func delayedJointFixture(t *testing.T, jitter bool) (*control.Client, control.Description, string, string, *jitterConnections) {
	t.Helper()
	_, d, root, socket := rtComposedFixture(t, func(cfg map[string]any) {
		cfg["control"] = map[string]any{"link_profile": "internet"}
	})
	transport := control.NewUnixHTTPClient(socket).Transport
	proxy := &httputil.ReverseProxy{Director: func(r *http.Request) { r.URL.Scheme = "http"; r.URL.Host = "localhost" }, Transport: transport}
	counts := &jitterConnections{enable: make(chan struct{})}
	server := &http.Server{ConnContext: func(ctx context.Context, _ net.Conn) context.Context {
		return context.WithValue(ctx, jitterContextKey{}, &jitterConnection{id: counts.opened.Add(1)})
	}, Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		conn := r.Context().Value(jitterContextKey{}).(*jitterConnection)
		conn.requests++
		counts.mu.Lock()
		counts.requests = append(counts.requests, r.URL.Path)
		halfOpen := counts.halfOpenRemaining != 0 && counts.halfOpenPath == r.URL.Path && conn.requests > 1
		if halfOpen && counts.halfOpenRemaining > 0 {
			counts.halfOpenRemaining--
		}
		halfOpenEntered := counts.halfOpenEntered
		delayed := r.URL.Path == "/v1/status" && counts.delayStatus
		if r.URL.Path == "/v1/status" {
			counts.statusCalls++
		}
		if delayed {
			counts.delayStatus = false
		}
		counts.active++
		if counts.active > counts.peak {
			counts.peak = counts.active
		}
		if counts.acquired {
			if counts.active > counts.peakHeld {
				counts.peakHeld = counts.active
			}
			if conn.requests == 1 {
				counts.coldRequests = append(counts.coldRequests, r.URL.Path)
			}
		}
		counts.mu.Unlock()
		defer func() { counts.mu.Lock(); counts.active--; counts.mu.Unlock() }()
		if halfOpen {
			halfOpenEntered <- struct{}{}
			<-r.Context().Done()
			return
		}
		if delayed {
			<-r.Context().Done()
			close(counts.slowFinished)
			return
		}
		if r.URL.Path == "/v1/control" {
			raw, err := io.ReadAll(r.Body)
			if err != nil {
				http.Error(w, err.Error(), 502)
				return
			}
			r.Body = io.NopCloser(bytes.NewReader(raw))
			var command control.Request
			if err := json.Unmarshal(raw, &command); err != nil {
				http.Error(w, err.Error(), 502)
				return
			}
			counts.mu.Lock()
			counts.requests = append(counts.requests, command.Operation)
			switch command.Operation {
			case "acquire":
				counts.acquire = conn.id
				defer func() { counts.mu.Lock(); counts.acquired = true; counts.mu.Unlock() }()
			case "renew":
				counts.renewals++
				if conn.requests == 1 {
					counts.coldRenewals++
				}
				if conn.id != counts.acquire {
					counts.wrongLane++
				}
			case "enable":
				counts.once.Do(func() { close(counts.enable) })
			}
			entered, release := counts.armEntered, counts.armRelease
			counts.mu.Unlock()
			if command.Operation == "arm" && entered != nil {
				close(entered)
				select {
				case <-release:
				case <-r.Context().Done():
					return
				}
			}
		}
		// Unverified transport stimulus: 250 ms per request plus 900 ms on
		// every connection's first request, standing in for a TLS handshake.
		delay := 250 * time.Millisecond
		if jitter && conn.requests == 1 {
			delay += 900 * time.Millisecond
		}
		timer := time.NewTimer(delay)
		defer timer.Stop()
		select {
		case <-timer.C:
			proxy.ServeHTTP(w, r)
		case <-r.Context().Done():
		}
	})}
	path := filepath.Join(filepath.Dir(socket), "delay.sock")
	listener, err := net.Listen("unix", path)
	if err != nil {
		t.Fatal(err)
	}
	go server.Serve(listener)
	t.Cleanup(func() { server.Close(); transport.(*http.Transport).CloseIdleConnections() })
	c, err := control.Dial(control.UnixPath(path))
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { c.Close() })
	return c, d, root, path, counts
}

func assertWarmRenewals(t *testing.T, counts *jitterConnections) {
	t.Helper()
	counts.mu.Lock()
	defer counts.mu.Unlock()
	if counts.renewals < 3 || counts.coldRenewals != 0 || counts.wrongLane != 0 {
		t.Fatalf("renewals=%d cold=%d outside_acquire_lane=%d", counts.renewals, counts.coldRenewals, counts.wrongLane)
	}
	if len(counts.coldRequests) != 0 {
		t.Fatalf("cold requests after Acquire: %v", counts.coldRequests)
	}
	t.Logf("ARM plus three status polls: opened=%d renewal_connections=1 cold_renewals=0 renewals=%d", counts.opened.Load(), counts.renewals)
}

// TestReviewHandshakeJitterComposed requires the native grant to survive ARM,
// three concurrent browser status polls and three renewals on Acquire's socket.
func TestReviewHandshakeJitterComposed(t *testing.T) {
	c, d, _, socket, counts := delayedJointFixture(t, true)
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:jitter"}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	// Unverified test watchdog: 30000 ms; receipts and native state pass the test.
	ctx, cancel := context.WithTimeout(context.Background(), 30000*time.Millisecond)
	defer cancel()
	done := make(chan struct{})
	go func() {
		defer close(done)
		ticker := time.NewTicker(50 * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				if s := r.statusSnapshot().Session; s != nil {
					_, _ = r.Heartbeat(s.ID)
				}
			}
		}
	}()
	defer func() { cancel(); <-done }()
	armed := make(chan error, 1)
	go func() { _, err := r.Arm(ctx, true); armed <- err }()
	select {
	case <-counts.enable:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	polls := make(chan *StatusBody, 3)
	for i := 0; i < 3; i++ {
		go func() { polls <- r.Status(ctx) }()
	}
	if err := <-armed; err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 3; i++ {
		if s := <-polls; !s.Available {
			t.Fatalf("poll unavailable: %+v", s)
		}
	}
	rtAwait(t, func() bool { counts.mu.Lock(); defer counts.mu.Unlock(); return counts.renewals >= 3 })
	s, err := c.Status(ctx)
	if err != nil || s.Grant == nil || s.Grant.GrantActive != 1 || s.Core.Armed != 1 || c.Grant().LeaseMS != 3000 {
		t.Fatalf("grant expired during ARM: native=%+v core=%+v lease=%d err=%v", s.Grant, s.Core, c.Grant().LeaseMS, err)
	}
	assertWarmRenewals(t, counts)
}

// TestReviewDeckHandshakeJitterComposed runs the Deck's C++ SDK through the same
// delayed native adapter, preventing a Go-only fix from hiding a cold C++ keeper.
func TestReviewDeckHandshakeJitterComposed(t *testing.T) {
	reviewDeckKeeper(t, false)
}

// TestReviewDeckCustomKeeperComposed preserves the shipped Deck's separate
// observer and custom renewal loop, including prewarmed status and Stop lanes.
func TestReviewDeckCustomKeeperComposed(t *testing.T) {
	reviewDeckKeeper(t, true)
}

func reviewDeckKeeper(t *testing.T, custom bool) {
	_, d, root, socket, counts := delayedJointFixture(t, true)
	binary := filepath.Join(root, "build", "remote-lease-probe")
	build := exec.Command("g++", "-std=c++17", "-pthread", "-I"+filepath.Join(root, "clients/cpp/include"), filepath.Join(root, "tests/remote_lease_probe.cpp"), "-o", binary)
	if output, err := build.CombinedOutput(); err != nil {
		t.Fatalf("build: %s %v", output, err)
	}
	args := []string{socket, d.MachineSHA256}
	if custom {
		args = append(args, "custom")
	}
	if output, err := exec.Command(binary, args...).CombinedOutput(); err != nil {
		t.Fatalf("Deck SDK: %s %v", output, err)
	}
	assertWarmRenewals(t, counts)
}
