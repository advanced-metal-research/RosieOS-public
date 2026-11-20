//go:build linux

package control

import (
	"crypto/sha256"
	"encoding/json"
	"errors"
	"time"
)

// Unverified resource policy: retain at most 256 request outcomes per session.
const requestCapacity = 256

type requestOutcome struct {
	status int
	body   []byte
}
type requestEntry struct {
	id        string
	principal sessionPrincipal
	digest    [32]byte
	done      chan struct{}
	outcome   requestOutcome
}
type requestTable struct {
	entries map[string]*requestEntry
	order   []*requestEntry
}

// idempotent binds an ID to its principal, payload and immutable final wire reply.
// Concurrent duplicates join the original, independent of HTTP disconnection.
// An executing oldest entry must finish before eviction: evicting it earlier
// would allow a retry to execute a second time. Release/expiry detach the table;
// existing waiters still receive their original result without repopulating it.
func (c *Controller) idempotent(request Request, execute func() requestOutcome, principals ...sessionPrincipal) (requestOutcome, error) {
	var principal sessionPrincipal
	if len(principals) != 0 {
		principal = principals[0]
	}
	if request.RequestID == nil {
		return execute(), nil
	}
	id := *request.RequestID
	if len(id) < 1 || len(id) > 64 {
		return requestOutcome{}, errors.New(ReasonInvalidRequestId)
	}
	for _, b := range []byte(id) {
		if b < 0x20 || b > 0x7e {
			return requestOutcome{}, errors.New(ReasonInvalidRequestId)
		}
	}
	request.RequestID = nil
	payload, err := json.Marshal(request)
	if err != nil {
		return requestOutcome{}, err
	}
	digest := sha256.Sum256(payload)
	for {
		c.mu.Lock()
		if c.grant.Session != "" && !time.Now().Before(c.expires) {
			c.requests = requestTable{}
		}
		// A foreign/stale session must use normal authorization, never read or
		// evict the current owner's outcomes. Acquire has no session yet.
		eligible := request.Operation == OperationAcquire && request.Session == "" ||
			(request.Session != "" && request.Session == c.grant.Session && time.Now().Before(c.expires))
		if !eligible {
			c.mu.Unlock()
			return execute(), nil
		}
		if entry := c.requests.entries[id]; entry != nil {
			// Acquire has no session yet, including while its first request is
			// running. Its cached capability must never cross transport owners.
			if entry.principal != principal {
				c.mu.Unlock()
				return requestOutcome{}, errors.New(ReasonSessionPrincipalMismatch)
			}
			if entry.digest != digest {
				c.mu.Unlock()
				return requestOutcome{}, errors.New(ReasonRequestIdConflict)
			}
			c.mu.Unlock()
			<-entry.done
			return entry.outcome, nil
		}
		if len(c.requests.order) == requestCapacity {
			oldest := c.requests.order[0]
			select {
			case <-oldest.done:
				delete(c.requests.entries, oldest.id)
				c.requests.order[0] = nil
				c.requests.order = c.requests.order[1:]
			default:
				c.mu.Unlock()
				<-oldest.done
				continue
			}
		}
		if c.requests.entries == nil {
			c.requests.entries = make(map[string]*requestEntry)
		}
		entry := &requestEntry{id: id, principal: principal, digest: digest, done: make(chan struct{})}
		c.requests.entries[id] = entry
		c.requests.order = append(c.requests.order, entry)
		c.mu.Unlock()
		entry.outcome = execute()
		close(entry.done)
		return entry.outcome, nil
	}
}
