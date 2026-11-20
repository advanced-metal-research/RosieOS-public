// Package unixhttp provides the transport shared by the SDK and internal tests.
package unixhttp

import (
	"context"
	"net"
	"net/http"
)

// NewClient retains standard HTTP behaviour over the specified Unix socket.
// Each call owns a transport so unrelated test lanes cannot share a pool.
func NewClient(path string) *http.Client {
	return &http.Client{Transport: &http.Transport{DialContext: func(ctx context.Context, _, _ string) (net.Conn, error) {
		return (&net.Dialer{}).DialContext(ctx, "unix", path)
	}}}
}
