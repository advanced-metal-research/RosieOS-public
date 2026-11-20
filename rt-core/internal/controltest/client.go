// Package controltest sends raw public envelopes without mutating fences or
// retrying commands, preserving malformed-input and uncertain-outcome tests.
package controltest

import (
	"bytes"
	"context"
	"encoding/json"
	"io"
	"net/http"
)

// Client retains the caller's HTTP timeout and connection pool.
type Client struct {
	HTTP *http.Client
	Base string
}

// Raw sends the exact body and headers; it never repairs a malformed envelope.
func (c Client) Raw(ctx context.Context, method, path string, body []byte, headers http.Header, result any) ([]byte, int, error) {
	r, err := http.NewRequestWithContext(ctx, method, c.Base+path, bytes.NewReader(body))
	if err != nil {
		return nil, 0, err
	}
	r.Header = headers
	response, err := c.HTTP.Do(r)
	if err != nil {
		return nil, 0, err
	}
	defer response.Body.Close()
	raw, err := io.ReadAll(response.Body)
	if err == nil && result != nil {
		err = json.Unmarshal(raw, result)
	}
	return raw, response.StatusCode, err
}

// JSON encodes only the provided value; schema, IDs and fences remain explicit.
func (c Client) JSON(ctx context.Context, method, path string, value, result any) ([]byte, int, error) {
	body, err := json.Marshal(value)
	if err != nil {
		return nil, 0, err
	}
	return c.Raw(ctx, method, path, body, nil, result)
}

// Decode retains streaming JSON envelope semantics for fixtures that accept one
// response object; Raw remains available for exact-byte response assertions.
func (c Client) Decode(ctx context.Context, method, path string, body []byte, headers http.Header, result any) (int, error) {
	raw, code, err := c.Raw(ctx, method, path, body, headers, nil)
	if err == nil {
		err = json.NewDecoder(bytes.NewReader(raw)).Decode(result)
	}
	return code, err
}
