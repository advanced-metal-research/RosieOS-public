//go:build linux

package control

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"io"
	"net/http"

	contract "rosieos/rt-core/adapters/rosie/control"
)

type ResourceInfo = contract.ResourceInfo
type RobotDescription = contract.RobotDescription
type DriveDescription = contract.DriveDescription
type DriveIdentity = contract.DriveIdentity

// Resource returns a reader only after the bounded body and ETag match sha.
// Corrupt or truncated downloads cannot become model input, even when a caller
// reads only a prefix. The reader holds no transport reservation or lease.
func (c *Client) Resource(ctx context.Context, sha string) (io.ReadCloser, error) {
	if !contract.ValidResourceDigest(sha) {
		return nil, &Rejected{Reason: contract.ReasonResourceUnknown}
	}
	c.mu.Lock()
	closed := c.closed
	c.mu.Unlock()
	if closed {
		return nil, ErrClosed
	}
	r, err := http.NewRequestWithContext(ctx, http.MethodGet, c.base+"/v1/resources/"+sha, nil)
	if err != nil {
		return nil, err
	}
	response, err := c.observationHTTP(r)
	if err != nil {
		return nil, err
	}
	defer response.Body.Close()
	if response.StatusCode != http.StatusOK {
		var refusal Response
		// Unverified error envelope bound: 65536 bytes.
		if response.StatusCode == http.StatusNotFound && json.NewDecoder(io.LimitReader(response.Body, 65536)).Decode(&refusal) == nil && refusal.Schema == contract.ResponseSchema && refusal.Error == contract.ReasonResourceUnknown {
			return nil, &Rejected{Reason: refusal.Error}
		}
		return nil, ErrProtocol
	}
	if response.ContentLength < 0 || response.ContentLength > contract.MaxResourceBytes || response.Header.Get("ETag") != `"`+sha+`"` {
		return nil, ErrProtocol
	}
	body, err := io.ReadAll(io.LimitReader(response.Body, response.ContentLength+1))
	if err != nil {
		return nil, err
	}
	if int64(len(body)) != response.ContentLength || fmt.Sprintf("%x", sha256.Sum256(body)) != sha {
		return nil, ErrProtocol
	}
	return io.NopCloser(bytes.NewReader(body)), nil
}
