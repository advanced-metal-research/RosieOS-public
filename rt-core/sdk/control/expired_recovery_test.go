//go:build linux

package control

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
)

// TestReviewExpiredStopRelease keeps expired credentials only for recovery;
// delayed renewals cannot revive motion between acknowledged Stop and Release.
func TestReviewExpiredStopRelease(t *testing.T) {
	old := Fence{Session: "expired", Generation: 7}
	var operations []string
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		var request Request
		if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
			t.Error(err)
			return
		}
		operations = append(operations, request.Operation)
		if request.Fence != old {
			t.Errorf("recovery fence=%+v", request.Fence)
		}
		raw, _ := json.Marshal(Grant{Generation: 8})
		_ = json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Operation: request.Operation, Data: raw})
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	c.SetFence(old)
	stopped, err := c.Stop(context.Background())
	if err != nil || stopped.Session != "" || c.Grant().Session != "" || c.Grant().Generation != 8 {
		t.Fatalf("stop=%+v local=%+v err=%v", stopped, c.Grant(), err)
	}
	c.remember("renew", old, Grant{Session: old.Session, Generation: 7, DeadlineHostNS: 9000000000})
	if c.Grant().Session != "" {
		t.Fatal("late renewal restored authority")
	}
	if _, err := c.Release(context.Background()); err != nil {
		t.Fatal(err)
	}
	if c.recoveryFence != (Fence{}) || c.Grant().Session != "" {
		t.Fatal("Release retained credentials")
	}
	if len(operations) != 2 || operations[0] != "stop" || operations[1] != "release" {
		t.Fatalf("operations=%v", operations)
	}
}

func TestResetReceiptRetiresOnlySubmittedFence(t *testing.T) {
	for _, outcome := range []string{"accepted", "native_refused", "not_submitted"} {
		t.Run(outcome, func(t *testing.T) {
			old := Fence{Session: "old", Generation: 7}
			server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				var request Request
				if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
					t.Fatal(err)
				}
				response := Response{Schema: "rosie.rt-control.response.v1", Operation: request.Operation, Sequence: 19}
				if outcome != "accepted" {
					response.Error = "native_rejected"
					if outcome == "not_submitted" {
						response.Sequence = 0
					}
					w.WriteHeader(http.StatusConflict)
				}
				_ = json.NewEncoder(w).Encode(response)
			}))
			defer server.Close()
			c := newClient(server.URL, &http.Transport{})
			defer c.Close()
			c.SetFence(old)
			_, err := c.Command(context.Background(), &Request{Operation: "reset_fault", AxisMask: 1})
			if (err == nil) != (outcome == "accepted") {
				t.Fatalf("reset result: %v", err)
			}
			if outcome == "not_submitted" {
				if c.Fence() != old {
					t.Fatal("unsubmitted reset discarded recovery authority")
				}
				return
			}
			if c.Fence() != (Fence{}) {
				t.Fatal("reset retained retired grant")
			}
			fresh := Grant{Session: "fresh", Generation: 8, LeaseMS: 3000}
			c.remember("acquire", Fence{}, fresh)
			c.remember("reset_fault", old, Grant{})
			c.remember("renew", old, Grant{Session: old.Session, Generation: 7})
			if c.Grant() != fresh {
				t.Fatal("old reset or renewal replaced new authority")
			}
		})
	}
}
