//go:build linux

package control

import (
	"context"
	"crypto/sha256"
	"fmt"
	"io"
	"net/http"
	"net/http/httptest"
	"testing"
)

// TestResourceVerifiesBeforeReading prevents a matching ETag from concealing
// corrupt bytes and prevents oversized responses from allocating unbounded data.
func TestResourceVerifiesBeforeReading(t *testing.T) {
	body := []byte("verified model")
	sha := fmt.Sprintf("%x", sha256.Sum256(body))
	for _, kind := range []string{"valid", "corrupt", "etag", "oversize", "unknown"} {
		t.Run(kind, func(t *testing.T) {
			s := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
				if r.Method != "GET" || r.URL.Path != "/v1/resources/"+sha || r.Header.Get("X-Control-Session") != "" {
					t.Error("resource must be an observation")
				}
				if kind == "unknown" {
					w.WriteHeader(404)
					io.WriteString(w, `{"schema":"rosie.rt-control.response.v1","operation":"resource","error":"resource_unknown"}`)
					return
				}
				w.Header().Set("ETag", `"`+sha+`"`)
				w.Header().Set("Content-Type", "application/xml")
				if kind == "oversize" {
					w.Header().Set("Content-Length", "33554433")
					w.WriteHeader(200)
					return
				}
				if kind == "etag" {
					w.Header().Set("ETag", `"wrong"`)
				}
				data := append([]byte{}, body...)
				if kind == "corrupt" {
					data[0] = 'X'
				}
				w.Write(data)
			}))
			defer s.Close()
			c := newClient(s.URL, http.DefaultTransport.(*http.Transport))
			defer c.Close()
			reader, err := c.Resource(context.Background(), sha)
			if kind != "valid" {
				if err == nil {
					reader.Close()
					t.Fatal("invalid resource returned a reader")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			defer reader.Close()
			data, err := io.ReadAll(reader)
			if err != nil || string(data) != "verified model" {
				t.Fatalf("reader: %q %v", data, err)
			}
		})
	}
}
