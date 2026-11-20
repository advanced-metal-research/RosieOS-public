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

// TestReviewStreamsLeaveStatusAvailable holds SSE and telemetry responses open
// while status completes, preventing shared-socket starvation on either link.
func TestReviewStreamsLeaveStatusAvailable(t *testing.T) {
	entered := make(chan struct{}, 2)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		switch r.URL.Path {
		case "/v1/events/stream", "/v1/telemetry/stream":
			kind := "text/event-stream"
			if r.URL.Path == "/v1/telemetry/stream" {
				kind = "application/octet-stream"
			}
			w.Header().Set("Content-Type", kind)
			w.(http.Flusher).Flush()
			entered <- struct{}{}
			<-r.Context().Done()
		default:
			json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: json.RawMessage(`{}`)})
		}
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	// Unverified watchdog: 1000 ms; both open streams precede the status receipt.
	ctx, cancel := context.WithTimeout(context.Background(), 1000*time.Millisecond)
	defer cancel()
	if err := c.warmConnections(ctx, true); err != nil {
		t.Fatal(err)
	}
	c.SetFence(Fence{Session: "held", Generation: 1})
	done := make(chan error, 2)
	go func() { done <- c.StreamEvents(ctx, 0, func(Event) error { return nil }) }()
	go func() { done <- c.TelemetryStream(ctx, 0, func(TelemetryBatch) error { return nil }) }()
	for i := 0; i < 2; i++ {
		select {
		case <-entered:
		case <-ctx.Done():
			t.Fatal(ctx.Err())
		}
	}
	_, err := c.Status(ctx)
	cancel()
	<-done
	<-done
	if err != nil {
		t.Fatal(err)
	}
}
