//go:build linux

package main

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"sync"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type record struct {
	Kind            string                   `json:"kind"`
	Class           string                   `json:"class,omitempty"`
	ExitCode        int                      `json:"exit_code"`
	Operation       string                   `json:"operation,omitempty"`
	HostNS          uint64                   `json:"host_monotonic_ns"`
	StartedNS       uint64                   `json:"started_host_monotonic_ns,omitempty"`
	Request         any                      `json:"request,omitempty"`
	Response        any                      `json:"response,omitempty"`
	Error           string                   `json:"error,omitempty"`
	Reason          string                   `json:"reason,omitempty"`
	NativeResult    *ipcclient.CommandResult `json:"native_result,omitempty"`
	NativeJogResult *control.JogObservation  `json:"native_jog_result,omitempty"`
	NativeReason    *uint32                  `json:"native_reason,omitempty"`
	SourceSequence  uint64                   `json:"source_sequence,omitempty"`
	Met             []string                 `json:"expectations_met,omitempty"`
	NotMet          []string                 `json:"expectations_not_met,omitempty"`
}

type evidence struct {
	mu      sync.Mutex
	file    *os.File
	sink    io.Writer
	err     error
	reasons []string
}

// openEvidence exclusively creates a private record, preventing an invocation
// from overwriting an earlier qualification or following an existing symlink.
func openEvidence(path string) (*evidence, error) {
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0600)
	if err != nil {
		return nil, err
	}
	return &evidence{file: f, sink: f}, nil
}

// write serialises concurrent renewal, Home and jog observations. The first
// storage error remains fatal, preventing an incomplete record claiming success.
// Session fields are truncated in the encoded copy, so recording a grant cannot
// disclose usable authority or mutate the live request's fence.
// Only summaries carry exit_code, preventing intermediate records implying success.
func (e *evidence) write(r record) error {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.err != nil {
		return e.err
	}
	r.HostNS = ipcclient.HostMonotonicNS()
	if r.Reason != "" {
		e.reasons = append(e.reasons, r.Reason)
	}
	if r.NativeResult != nil {
		e.reasons = append(e.reasons, fmt.Sprintf("native:%d", r.NativeResult.Reason))
	}
	raw, err := json.Marshal(r)
	if err != nil {
		e.err = err
		return err
	}
	var safe map[string]any
	decoder := json.NewDecoder(bytes.NewReader(raw))
	decoder.UseNumber()
	if e.err = decoder.Decode(&safe); e.err != nil {
		return e.err
	}
	if r.Kind != "summary" {
		delete(safe, "exit_code")
	}
	redactSessions(safe)
	e.err = json.NewEncoder(e.sink).Encode(safe)
	return e.err
}

func (e *evidence) reasonCount() int {
	e.mu.Lock()
	defer e.mu.Unlock()
	return len(e.reasons)
}

func (e *evidence) reasonsAfter(index int) []string {
	e.mu.Lock()
	defer e.mu.Unlock()
	return sortedReasons(e.reasons[index:])
}

// redactSessions visits nested grants, requests and raw response data without
// converting numeric fields to float64 and losing exact generations or times.
// Unverified disclosure policy: retain only the first 8 hex characters.
func redactSessions(value any) {
	switch value := value.(type) {
	case map[string]any:
		for key, child := range value {
			if key == "session" {
				if session, ok := child.(string); ok && len(session) > 8 {
					value[key] = session[:8]
				}
			} else {
				redactSessions(child)
			}
		}
	case []any:
		for _, child := range value {
			redactSessions(child)
		}
	}
}

func (e *evidence) failure() error {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.err
}

func (e *evidence) close() error {
	return errors.Join(e.failure(), e.file.Sync(), e.file.Close())
}

// receipt retains typed native details separately from transport diagnostics;
// neither a missing response nor a transport failure can satisfy a refusal.
func (e *evidence) receipt(op string, start uint64, req, response any, err error) error {
	r := record{Kind: "request", Operation: op, StartedNS: start, Request: req, Response: response}
	if err != nil {
		r.Error = err.Error()
		var rejected *control.Rejected
		if errors.As(err, &rejected) {
			r.Reason, r.NativeResult, r.NativeJogResult = rejected.Reason, rejected.NativeResult, rejected.NativeJogResult
		}
		var jogRejected *control.JogRejection
		if errors.As(err, &jogRejected) {
			r.Reason, r.NativeReason, r.SourceSequence = jogRejected.Reason, &jogRejected.NativeReason, jogRejected.Sequence
		}
	}
	return errors.Join(err, e.write(r))
}
