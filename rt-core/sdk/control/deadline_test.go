//go:build linux

package control

import (
	"context"
	"errors"
	"io"
	"net/http"
	"net/http/httptest"
	"testing"
	"time"
)

// TestReviewDeadlineDistinguishesRenewal preserves the keeper's transport-loss
// contract without allowing an unrelated status deadline to revoke authority.
func TestReviewDeadlineDistinguishesRenewal(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		io.Copy(io.Discard, r.Body)
		<-r.Context().Done()
	}))
	defer server.Close()
	c := newClient(server.URL, &http.Transport{})
	defer c.Close()
	for _, operation := range []string{"status", "renew"} {
		t.Run(operation, func(t *testing.T) {
			// Unverified request budget: 100 ms, with the reply withheld until cancellation.
			ctx, cancel := context.WithTimeout(context.Background(), 100*time.Millisecond)
			defer cancel()
			var err error
			if operation == "renew" {
				_, err = c.Renew(ctx)
			} else {
				_, err = c.Status(ctx)
			}
			if !errors.Is(err, context.DeadlineExceeded) || errors.Is(err, ErrTransport) != (operation == "renew") {
				t.Fatalf("%s deadline typing: %v", operation, err)
			}
		})
	}
}
