//go:build linux

package control

import (
	"context"
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync/atomic"
	"testing"
)

// TestTLSResumptionAcrossLanes requires a resumed verified handshake after the
// first connection closes, preventing a configured but unused ticket cache.
func TestTLSResumptionAcrossLanes(t *testing.T) {
	var resumed atomic.Bool
	server := httptest.NewTLSServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.TLS.DidResume {
			resumed.Store(true)
		}
		_ = json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Data: json.RawMessage(`{"max_grant_lease_ns":3000000000}`)})
	}))
	defer server.Close()
	roots := x509.NewCertPool()
	roots.AddCert(server.Certificate())
	client, err := DialTLS(server.URL, &tls.Config{RootCAs: roots, Certificates: server.TLS.Certificates})
	must(t, err)
	defer client.Close()
	_, err = client.Describe(context.Background())
	must(t, err)
	_, err = client.call(context.Background(), client.renewal, "GET", "/v1/describe", nil, nil, false)
	must(t, err)
	if !resumed.Load() {
		t.Fatal("renewal lane did not resume the verified TLS session")
	}
}
