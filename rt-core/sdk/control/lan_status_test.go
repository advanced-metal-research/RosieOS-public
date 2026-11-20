//go:build linux

package control

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"
)

// TestReviewLANStatusDuringHome keeps local status independent too; reserving
// remote connections must not introduce a single-connection LAN bottleneck.
func TestReviewLANStatusDuringHome(t *testing.T) {
	entered, release := make(chan struct{}), make(chan struct{})
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path == "/v1/control" {
			close(entered)
			select {
			case <-release:
			case <-r.Context().Done():
				return
			}
		}
		json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: json.RawMessage(`{}`)})
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	// Unverified test watchdog: 1000 ms; Status must finish before Home releases.
	ctx, cancel := context.WithTimeout(context.Background(), 1000*time.Millisecond)
	defer cancel()
	done := make(chan error, 1)
	go func() { _, err := c.Home(ctx, 1); done <- err }()
	select {
	case <-entered:
	case <-ctx.Done():
		t.Fatal(ctx.Err())
	}
	_, err := c.Status(ctx)
	close(release)
	if err != nil {
		t.Fatal(err)
	}
	if err := <-done; err != nil {
		t.Fatal(err)
	}
}
