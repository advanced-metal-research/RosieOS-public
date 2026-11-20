//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"net/http/httptest"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

// TestReviewColdDescribeWarmLease preserves the Describe-only delay probe:
// cold setup must not deny authority, while an excessive warm RTT must do so.
func TestReviewColdDescribeWarmLease(t *testing.T) {
	// Unverified delay stimuli in ms; assertions use grants and absolute ns.
	for _, coldMS := range []int{300, 1500, 3000} {
		for _, warmMS := range []int{250, 700} {
			t.Run(fmt.Sprintf("cold_%d_warm_%d", coldMS, warmMS), func(t *testing.T) {
				var describes, acquires atomic.Int32
				var firstPeer string
				server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
					response := Response{Schema: "rosie.rt-control.response.v1"}
					var payload any
					if r.URL.Path == "/v1/describe" {
						delay := warmMS
						n := describes.Add(1)
						if n == 1 {
							delay, firstPeer = coldMS, r.RemoteAddr
						} else if n == 2 && r.RemoteAddr != firstPeer {
							t.Error("warm Describe did not reuse the connection")
						}
						timer := time.NewTimer(time.Duration(delay) * time.Millisecond)
						defer timer.Stop()
						select {
						case <-timer.C:
						case <-r.Context().Done():
							return
						}
						response.Operation = "describe"
						payload = map[string]any{"max_grant_lease_ns": 3000000000, "max_jog_input_age_ns": 750000000}
					} else {
						var request Request
						if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
							t.Error(err)
							return
						}
						response.Operation = request.Operation
						if request.Operation != "acquire" {
							t.Errorf("unexpected operation: %s", request.Operation)
						}
						acquires.Add(1)
						payload = Grant{Session: "warm", Generation: 1, LeaseMS: request.RequestedLeaseMS}
					}
					response.Data, _ = json.Marshal(payload)
					_ = json.NewEncoder(w).Encode(response)
				}))
				defer server.Close()
				client := newClient(server.URL, &http.Transport{})
				defer client.Close()
				// Unverified operation watchdog: 10000 ms; no timeout is a positive assertion.
				ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
				defer cancel()
				grant, err := client.Acquire(ctx, "remote", Binding{})
				if warmMS == 250 {
					if err != nil || grant.LeaseMS < 1125 || grant.LeaseMS > 3000 || acquires.Load() != 1 {
						t.Fatalf("grant=%+v acquire_count=%d err=%v", grant, acquires.Load(), err)
					}
					if measured := client.Timing().RoundTrip; measured < 250000000 || measured >= 666666667 {
						t.Fatalf("warm_rtt_ns=%d", measured)
					}
					t.Logf("acquire=accepted lease_ms=%d warm_rtt_ns=%d", grant.LeaseMS, client.Timing().RoundTrip)
				} else {
					var rejected *LeaseTimingRejected
					var reason *Rejected
					if !errors.As(err, &rejected) || !errors.As(err, &reason) || reason.Reason != "control_session_stale" {
						t.Fatalf("refusal=%v", err)
					}
					if rejected.Ceiling != 3000000000 || rejected.RoundTrip < 700000000 || acquires.Load() != 0 {
						t.Fatalf("refusal=%+v acquire_count=%d", rejected, acquires.Load())
					}
					if !strings.Contains(err.Error(), "lease_ceiling_ns=3000000000") || !strings.Contains(err.Error(), fmt.Sprintf("measured_warm_rtt_ns=%d", rejected.RoundTrip)) {
						t.Fatal(err)
					}
					t.Log(err)
				}
				wantDescribes := int32(2)
				if warmMS == 250 {
					wantDescribes = 9
				}
				if describes.Load() != wantDescribes {
					t.Fatalf("describes=%d", describes.Load())
				}
			})
		}
	}
}
