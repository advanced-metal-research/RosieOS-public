//go:build linux

package control

import (
	"context"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"

	adapter "rosieos/rt-core/adapters/rosie/control"
)

// Decode the wire label through the SDK rejection path, preventing a generated
// principal refusal from being hidden as a transport or protocol failure.
func TestSDKSessionPrincipalMismatchDecode(t *testing.T) {
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.WriteHeader(http.StatusConflict)
		_, _ = w.Write([]byte(`{"schema":"rosie.rt-control.response.v1","operation":"renew","error":"session_principal_mismatch"}`))
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	client.SetFence(Fence{Session: "other-owner", Generation: 7})
	_, err := client.Renew(context.Background())
	var rejected *Rejected
	if !errors.As(err, &rejected) || rejected.Reason != adapter.ReasonSessionPrincipalMismatch {
		t.Fatalf("principal refusal: %v", err)
	}
}
