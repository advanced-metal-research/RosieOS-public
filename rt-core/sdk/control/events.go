//go:build linux

package control

import (
	"bufio"
	"context"
	"encoding/json"
	"errors"
	"io"
	"net/http"
	"strconv"
	"strings"
	"time"
)

// Events returns one bounded batch after a cursor without acquiring authority.
// A caller must consume an explicit drop record before advancing past lost history.
func (c *Client) Events(after uint64) (EventBatch, error) {
	// Unverified polling request deadline: 5 seconds; streaming owns its context.
	const pollingDeadline = 5 * time.Second
	ctx, cancel := context.WithTimeout(context.Background(), pollingDeadline)
	defer cancel()
	return c.EventsContext(ctx, after)
}

// EventsContext allows callers to cancel polling and preserves the exact uint64
// cursor, preventing JSON floating-point rounding from skipping an event.
func (c *Client) EventsContext(ctx context.Context, after uint64) (EventBatch, error) {
	b, err := get[EventBatch](c, ctx, "/v1/events?after="+strconv.FormatUint(after, 10))
	if err != nil {
		return b, err
	}
	cursor := after
	for _, event := range b.Events {
		if err = validateEvent(event, cursor); err != nil {
			return EventBatch{}, err
		}
		cursor = event.Sequence
	}
	if b.NextSequence != cursor || b.LatestSequence < cursor {
		return EventBatch{}, ErrProtocol
	}
	return b, nil
}

func validateEvent(event Event, after uint64) error {
	if event.Sequence <= after || event.TimeNS == 0 || event.Type == "" {
		return ErrProtocol
	}
	if event.Type == "events_dropped" {
		if event.Dropped == nil || event.Dropped.FirstLostSequence != after+1 || event.Dropped.LastLostSequence != event.Sequence {
			return ErrProtocol
		}
	} else if event.Sequence != after+1 || event.Dropped != nil {
		return ErrProtocol
	}
	return nil
}

// StreamEvents delivers ordered SSE records until context cancellation, handler
// failure, a negotiated read idle timeout or disconnect. Resume explicitly with
// the last handled sequence;
// reconnection never silently discards an overflow notification.
func (c *Client) StreamEvents(ctx context.Context, after uint64, handler func(Event) error) error {
	if handler == nil {
		return errors.New("nil event handler")
	}
	c.mu.Lock()
	closed := c.closed
	c.mu.Unlock()
	if closed {
		return ErrClosed
	}
	r, err := http.NewRequestWithContext(ctx, http.MethodGet, c.base+"/v1/events/stream?after="+strconv.FormatUint(after, 10), nil)
	if err != nil {
		return err
	}
	r.Header.Set("Accept", "text/event-stream")
	response, err := c.observationHTTP(r)
	if err != nil {
		return err
	}
	defer response.Body.Close()
	if response.StatusCode != http.StatusOK || !strings.HasPrefix(response.Header.Get("Content-Type"), "text/event-stream") {
		return ErrProtocol
	}
	scanner := bufio.NewScanner(response.Body)
	// Unverified wire record bound: 65536 bytes; payloads contain no plan points.
	const eventRecordLimit = 65536
	scanner.Buffer(make([]byte, 4096), eventRecordLimit)
	var id, kind, body string
	for scanner.Scan() {
		line := scanner.Text()
		if line == "" {
			if body != "" {
				var event Event
				if json.Unmarshal([]byte(body), &event) != nil || id != strconv.FormatUint(event.Sequence, 10) || kind != event.Type {
					return ErrProtocol
				}
				if err = validateEvent(event, after); err != nil {
					return err
				}
				if err = handler(event); err != nil {
					return err
				}
				after = event.Sequence
			}
			id, kind, body = "", "", ""
			continue
		}
		if strings.HasPrefix(line, ":") {
			continue
		}
		key, value, ok := strings.Cut(line, ":")
		if !ok {
			continue
		}
		value = strings.TrimPrefix(value, " ")
		switch key {
		case "id":
			id = value
		case "event":
			kind = value
		case "data":
			if len(body)+len(value)+1 > eventRecordLimit {
				return ErrProtocol
			}
			if body != "" {
				body += "\n"
			}
			body += value
		}
	}
	if ctx.Err() != nil {
		return ctx.Err()
	}
	if err = scanner.Err(); err != nil {
		return errors.Join(ErrTransport, err)
	}
	return errors.Join(ErrTransport, io.EOF)
}
