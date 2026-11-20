//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net"
	"net/http"
	"net/http/httptest"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

// TestLeaseSizing uses absolute measured samples and ceilings; internet policy
// must not force a local client to hold authority for the full cell ceiling.
func TestLeaseSizing(t *testing.T) {
	for _, tc := range []struct{ rtt, ceiling, want time.Duration }{
		{time.Millisecond, 3 * time.Second, 500 * time.Millisecond},
		{20 * time.Millisecond, 3 * time.Second, 500 * time.Millisecond},
		{20*time.Millisecond + time.Nanosecond, 3 * time.Second, 3 * time.Second},
		{240 * time.Millisecond, 3 * time.Second, 3 * time.Second},
		{300 * time.Millisecond, 3 * time.Second, 3 * time.Second},
		{300 * time.Millisecond, 500 * time.Millisecond, 500 * time.Millisecond},
	} {
		if got := RequestedLease(tc.rtt, tc.ceiling); got != tc.want {
			t.Fatalf("lease=%v want=%v", got, tc.want)
		}
	}
	timing := TimingForLease(3*time.Second, 300*time.Millisecond)
	if timing.RenewInterval != time.Second || timing.CallBudget != 3900*time.Millisecond || timing.StatusAge != 3600*time.Millisecond {
		t.Fatalf("timing=%+v", timing)
	}
}

// TestDelayedAdapterLeaseRenewal delays every receipt by 300 ms. Sixty correlated
// renewals and reservation maintenance must progress while Status is held, then
// Status must complete before renewal stops. A LAN ceiling fails typed before motion.
func TestDelayedAdapterLeaseRenewal(t *testing.T) {
	for _, ceiling := range []int{3000, 500} {
		t.Run(fmt.Sprint(ceiling), func(t *testing.T) {
			var mu sync.Mutex
			var deadline time.Time
			var grant Grant
			var renews int
			var statusPeer, renewPeer string
			var opened atomic.Int32
			probePeers := make(map[string]int)
			probeProgress := make(chan struct{}, 1)
			statusEntered, releaseStatus := make(chan struct{}), make(chan struct{})
			var releaseOnce, statusOnce sync.Once
			release := func() { releaseOnce.Do(func() { close(releaseStatus) }) }
			server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				response := Response{Schema: "rosie.rt-control.response.v1"}
				var payload any
				if r.URL.Path == "/v1/describe" {
					response.Operation = "describe"
					payload = map[string]any{"max_grant_lease_ns": uint64(ceiling) * 1000000, "max_jog_input_age_ns": 750000000}
				} else if r.URL.Path == "/v1/status" {
					mu.Lock()
					statusPeer = r.RemoteAddr
					mu.Unlock()
					statusOnce.Do(func() { close(statusEntered) })
					select {
					case <-releaseStatus:
					case <-r.Context().Done():
						return
					}
					response.Operation = "status"
					payload = Status{}
				} else if r.Method == http.MethodGet && r.URL.Path == "/v1/jog/ingress" {
					response.Operation = "jog_ingress"
					payload = JogIngressObservation{}
				} else if r.Method == http.MethodPost && r.URL.Path == "/v1/control" {
					var request Request
					if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
						t.Error(err)
						return
					}
					response.Operation = request.Operation
					mu.Lock()
					switch request.Operation {
					case "acquire":
						lease := request.RequestedLeaseMS
						if lease <= 0 || lease > ceiling {
							t.Errorf("requested=%d ceiling=%d", lease, ceiling)
							lease = ceiling
						}
						grant = Grant{Session: "delayed", Generation: 1, LeaseMS: lease}
						deadline = time.Now().Add(time.Duration(lease) * time.Millisecond)
					case "renew":
						renewPeer = r.RemoteAddr
						if !time.Now().Before(deadline) {
							response.Error = "control_session_stale"
						} else {
							renews++
							deadline = time.Now().Add(time.Duration(grant.LeaseMS) * time.Millisecond)
						}
					case "release":
						if ceiling == 500 {
							response.Error = "control_session_stale"
						} else {
							grant = Grant{}
						}
					default:
						t.Errorf("unexpected operation %s", request.Operation)
					}
					payload = grant
					mu.Unlock()
				} else {
					t.Errorf("unexpected request %s %s", r.Method, r.URL.Path)
					http.NotFound(w, r)
					return
				}
				// Injected transport delay, not a success condition.
				timer := time.NewTimer(300 * time.Millisecond)
				defer timer.Stop()
				select {
				case <-timer.C:
				case <-r.Context().Done():
					return
				}
				response.Data, _ = json.Marshal(payload)
				if response.Error != "" {
					w.WriteHeader(http.StatusConflict)
				}
				_ = json.NewEncoder(w).Encode(response)
				if r.URL.Path == "/v1/jog/ingress" {
					mu.Lock()
					probePeers[r.RemoteAddr]++
					mu.Unlock()
					select {
					case probeProgress <- struct{}{}:
					default:
					}
				}
			}))
			server.Config.ConnContext = func(ctx context.Context, _ net.Conn) context.Context { opened.Add(1); return ctx }
			server.Start()
			defer server.Close()
			client := newClient(server.URL, &http.Transport{})
			defer client.Close()
			// Unverified test watchdog; only receipt counts and typed refusals pass.
			ctx, cancel := context.WithTimeout(context.Background(), 90*time.Second)
			defer cancel()
			g, err := client.AcquireLease(ctx, "remote", Binding{}, 3*time.Second)
			if ceiling == 500 {
				var rejected *Rejected
				if !errors.As(err, &rejected) || rejected.Reason != "control_session_stale" {
					t.Fatalf("want typed stale refusal, got %v", err)
				}
				if client.Grant().Session != "" {
					t.Fatal("failed acquisition retained a usable local fence")
				}
				t.Log("300 ms delayed receipts: 500 ms lease refused typed control_session_stale")
				return
			}
			must(t, err)
			if g.LeaseMS != 3000 {
				t.Fatalf("lease=%d", g.LeaseMS)
			}
			statusDone := make(chan error, 1)
			go func() { _, err := client.Status(ctx); statusDone <- err }()
			defer release()
			select {
			case <-statusEntered:
			case <-ctx.Done():
				t.Fatal(ctx.Err())
			}
			updates, err := client.StartRenewal(ctx, 100*time.Millisecond)
			must(t, err)
			// Force maintenance during the held Status instead of relying on cadence.
			client.wakeKeepAlive()
			for i := 0; i < 60; i++ {
				select {
				case update, ok := <-updates:
					if !ok || update.Err != nil || update.Grant.LeaseMS != 3000 {
						t.Fatalf("renewal %d: %+v", i+1, update)
					}
				case <-ctx.Done():
					t.Fatal(ctx.Err())
				}
				// Injected 300 ms receipts: three renewals overlap Status within
				// its bounded request window; all sixty receipts remain required.
				if i == 2 {
					for {
						mu.Lock()
						probes, heldProbes := len(probePeers), probePeers[statusPeer]
						mu.Unlock()
						if heldProbes != 0 {
							t.Fatal("maintenance borrowed the held Status reservation")
						}
						// Absolute reservation count: three observation, urgent and bulk.
						if probes >= 5 {
							break
						}
						select {
						case <-probeProgress:
						case <-ctx.Done():
							t.Fatal(ctx.Err())
						}
					}
					select {
					case err := <-statusDone:
						t.Fatalf("Status completed before release: %v", err)
					default:
					}
					release()
					select {
					case err := <-statusDone:
						must(t, err)
					case <-ctx.Done():
						t.Fatal(ctx.Err())
					}
				}
			}
			if n := opened.Load(); n != 7 {
				t.Fatalf("connections=%d; delayed live reservations were replaced", n)
			}
			client.StopRenewal()
			mu.Lock()
			count, independent := renews, renewPeer != "" && renewPeer != statusPeer
			mu.Unlock()
			if count < 60 || !independent {
				t.Fatalf("count=%d independent=%v", count, independent)
			}
			t.Log("300 ms delayed receipts: 60 successful renewals at 3000 ms lease; Status completed during renewal, maintenance progressed, seven connections retained")
		})
	}
}
