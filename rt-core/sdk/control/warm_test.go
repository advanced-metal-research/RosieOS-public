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

// TestReviewWarmConnectionAccounting compares the previous lazy lanes with
// reserved sockets; a cold connection cannot hide behind TLS resumption.
func TestReviewWarmConnectionAccounting(t *testing.T) {
	for _, legacy := range []bool{true, false} {
		name := "reserved"
		if legacy {
			name = "previous_lazy_policy"
		}
		t.Run(name, func(t *testing.T) {
			var opened, cold atomic.Int32
			var acquired atomic.Bool
			type key struct{}
			server := httptest.NewUnstartedServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				first := r.Context().Value(key{}).(*atomic.Bool).CompareAndSwap(false, true)
				if first && acquired.Load() {
					cold.Add(1)
				}
				// Review transport stimulus: 250 ms RTT and 900 ms per new socket.
				delay := 250 * time.Millisecond
				if first {
					delay += 900 * time.Millisecond
				}
				timer := time.NewTimer(delay)
				defer timer.Stop()
				select {
				case <-timer.C:
				case <-r.Context().Done():
					return
				}
				response := Response{Schema: "rosie.rt-control.response.v1"}
				payload := any(map[string]any{})
				if r.URL.Path == "/v1/control" {
					var request Request
					if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
						t.Error(err)
						return
					}
					response.Operation = request.Operation
					payload = Grant{Session: "warm", Generation: 1, LeaseMS: 3000}
				}
				response.Data, _ = json.Marshal(payload)
				json.NewEncoder(w).Encode(response)
			}))
			server.Config.ConnContext = func(ctx context.Context, _ net.Conn) context.Context {
				opened.Add(1)
				return context.WithValue(ctx, key{}, &atomic.Bool{})
			}
			server.Start()
			defer server.Close()
			c := newClient(server.URL, &http.Transport{})
			defer c.Close()
			// Measured composed link: 250 ms RTT, 3000 ms cell ceiling.
			c.roundTrip, c.leaseCeiling = 250*time.Millisecond, 3000*time.Millisecond
			// Unverified test watchdog: 15000 ms; request counts establish success.
			ctx, cancel := context.WithTimeout(context.Background(), 15000*time.Millisecond)
			defer cancel()
			if legacy {
				c.warmLanes = nil
				for _, lane := range []*http.Client{c.control, c.urgent, c.bulk} {
					tr := lane.Transport.(*http.Transport)
					tr.MaxConnsPerHost = 0
					tr.MaxIdleConnsPerHost = 2
				}
				if _, err := c.Status(ctx); err != nil {
					t.Fatal(err)
				}
				if _, err := c.Command(ctx, &Request{Operation: "acquire"}); err != nil {
					t.Fatal(err)
				}
			} else if _, err := c.Acquire(ctx, "test", Binding{}); err != nil {
				t.Fatal(err)
			}
			acquired.Store(true)
			var wg sync.WaitGroup
			for i := 0; i < 6; i++ {
				wg.Add(1)
				go func(i int) {
					defer wg.Done()
					var err error
					switch i {
					case 4:
						_, err = c.Stop(ctx)
					case 5:
						_, err = c.PrepareProgram(ctx, []byte("program"))
					default:
						_, err = c.Status(ctx)
					}
					if err != nil {
						t.Error(err)
					}
				}(i)
			}
			wg.Wait()
			want := int32(0)
			if legacy {
				want = 5
			}
			if cold.Load() != want || opened.Load() != 7 {
				t.Fatalf("opened=%d cold_after_acquire=%d", opened.Load(), cold.Load())
			}
			t.Logf("connections=%d cold_after_acquire=%d", opened.Load(), cold.Load())
		})
	}
}
