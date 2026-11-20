//go:build linux

package publisher

import "fmt"

type diagnostics struct {
	State      string `json:"publisher_state"`
	LastError  string `json:"last_error"`
	Reconnects uint64 `json:"reconnects"`
	Drops      uint64 `json:"drops"`
}

func (p *Publisher) diagnosticsLocked() diagnostics {
	state := p.transport
	if state == "publishing" {
		if p.sourceError != "" {
			state = "source_unavailable"
		} else if p.dropping {
			state = "dropping"
		}
	}
	return diagnostics{state, p.lastError, p.reconnects, p.Dropped()}
}

func (p *Publisher) diagnostics() diagnostics {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.diagnosticsLocked()
}

// logLocked emits one escaped journal record per transition. Counts do not
// trigger records, preventing repeated failures from flooding stderr.
func (p *Publisher) logLocked(event, transition string) {
	d := p.diagnosticsLocked()
	fmt.Fprintf(p.log, "component=rt-natspublisher event=%s transition=%s publisher_state=%s last_error=%q reconnects=%d drops=%d\n", event, transition, d.State, d.LastError, d.Reconnects, d.Drops)
}

func (p *Publisher) setTransport(state string, err error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if err != nil {
		// The most recent incident is recorded even when the state is unchanged, so
		// a repeated refusal with a new reason is not hidden behind the old one.
		p.lastError = err.Error()
	}
	if p.transport == state {
		return
	}
	p.transport = state
	p.logLocked("transport", state)
}

func (p *Publisher) setSource(err error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	reason := ""
	if err != nil {
		reason = err.Error()
	}
	if p.sourceError == reason {
		return
	}
	p.sourceError = reason
	transition := "source_ready"
	if reason != "" {
		p.lastError = reason
		transition = "source_unavailable"
	}
	p.logLocked("source", transition)
}

func (p *Publisher) drop(reason string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.dropped.Add(1)
	if p.dropping {
		return
	}
	p.dropping = true
	p.lastError = reason
	p.logLocked("delivery", "dropping")
}

// recovered clears a drop episode only after a complete fresh pair with no
// concurrent sample loss, preventing a late ack from hiding continuing drops.
func (p *Publisher) recovered(before uint64) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if !p.dropping || p.Dropped() != before {
		return
	}
	p.dropping = false
	p.logLocked("delivery", "recovered")
}
