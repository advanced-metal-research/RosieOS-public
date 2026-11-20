//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
)

// StatusObservation returns the current Status fields without Execution.Handles.
// Use Status when prepared/retired handle history is needed. The optional
// control view avoids constructing and transferring history on supporting
// servers. Older servers ignore the query; the same one-pass decoder still
// discards their historical handles without a second request or lost evidence.
func (c *Client) StatusObservation(ctx context.Context) (Status, error) {
	c.mu.Lock()
	closed := c.closed
	c.mu.Unlock()
	if closed {
		return Status{}, ErrClosed
	}
	if err := ctx.Err(); err != nil {
		return Status{}, err
	}
	r, err := http.NewRequestWithContext(ctx, http.MethodGet, c.base+"/v1/status?view=control", nil)
	if err != nil {
		return Status{}, err
	}
	wire, err := c.observationHTTP(r)
	if err != nil {
		return Status{}, err
	}
	// The existing observation lane bounds connection/body reads and returns its
	// reservation only when the response has been consumed or quarantined.
	const responseLimit = 8 << 20 // Same response cap as Client.call.
	raw, readErr := io.ReadAll(io.LimitReader(wire.Body, responseLimit+1))
	wire.Body.Close()
	if err := ctx.Err(); err != nil {
		return Status{}, err
	}
	if readErr != nil {
		return Status{}, errors.Join(ErrTransport, readErr)
	}
	if len(raw) > responseLimit {
		return Status{}, ErrProtocol
	}
	if wire.StatusCode != http.StatusOK {
		// A refusal's data is operation-specific, not necessarily a Status.
		// Preserve it exactly through the existing envelope on this slow path.
		var response Response
		if json.Unmarshal(raw, &response) != nil || response.Schema != "rosie.rt-control.response.v1" {
			return Status{}, ErrProtocol
		}
		if wire.StatusCode == http.StatusConflict {
			return Status{}, &Rejected{Reason: response.Error, NativeResult: response.NativeResult,
				NativeJogResult: response.NativeJogResult, ResponseData: response.Data}
		}
		return Status{}, ErrProtocol
	}
	var response statusObservationResponse
	if json.Unmarshal(raw, &response) != nil || response.Schema != "rosie.rt-control.response.v1" || response.Error != "" {
		return Status{}, ErrProtocol
	}
	if err := ctx.Err(); err != nil {
		return Status{}, err
	}
	response.Data.Status.Execution = response.Data.Execution.Execution
	return response.Data.Status, nil
}

// Embed generated contract types so new current fields remain visible. Only the
// explicitly shadowed historical handle array is discarded; no map[string]any
// round-trip may round native timestamps, epochs or sequences through float64.
type statusObservationResponse struct {
	Response
	Data statusObservation `json:"data"`
}

type statusObservation struct {
	Status
	Execution executionObservation `json:"execution"`
}

type executionObservation struct {
	Execution
	Handles discardedStatusHandles `json:"handles"`
}

type discardedStatusHandles struct{}

func (*discardedStatusHandles) UnmarshalJSON([]byte) error { return nil }
