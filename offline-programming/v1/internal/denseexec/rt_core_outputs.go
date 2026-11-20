//go:build linux

package denseexec

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"sort"
	"strings"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

// The OLP's output execution step (docs/IO_SCHEDULE.md): a production plan
// whose header carries process_markers loads only when its schedule is the
// one the operator's plan record names and every marker names an output the
// cell describes; Play arms the cell's outputs (io_arm, fenced by the grant)
// before StartProgram; the end of the session disarms them (io_disarm).
// rt-core executes each marker at its sample, expires every ON on its own
// clock and writes every output OFF in the cycle that applies a Stop, a fault,
// permit loss or a grant change, so nothing here is a safety function: these
// steps only decide when outputs may be switched at all.

// describedOutput is one output of rt-core's Describe `io` block.
type describedOutput struct {
	Name     string `json:"name"`
	Class    string `json:"class"`
	ExpiryNS int64  `json:"expiry_ns"`
	Readback string `json:"readback,omitempty"`
}

type describedIO struct {
	Inputs []struct {
		Name string `json:"name"`
	} `json:"inputs"`
	TorchQualified bool              `json:"torch_qualified"`
	Outputs        []describedOutput `json:"outputs"`
}

func describedOutputs(d control.Description) (*describedIO, error) {
	if len(d.IO) == 0 || string(d.IO) == "null" {
		return nil, nil
	}
	var io describedIO
	if err := json.Unmarshal(d.IO, &io); err != nil {
		return nil, fmt.Errorf("the cell's io description is unreadable: %w", err)
	}
	return &io, nil
}

// markerOutputs is the sorted set of outputs a schedule switches.
func markerOutputs(markers []densejointtraj.ProcessMarker) []string {
	seen := map[string]bool{}
	var names []string
	for _, m := range markers {
		if m.TimeoutMS != 0 || m.Monitor != "" {
			continue
		}
		if !seen[m.Action] {
			seen[m.Action] = true
			names = append(names, m.Action)
		}
	}
	sort.Strings(names)
	return names
}

// processOutputsGate is Load's robot-free output check, run before any
// authority is taken. It answers the outputs a production Load will switch
// (nil for a motion-only plan and for a dry run, which strips the markers).
//
//   - A dry run strips the markers, so it is refused only when the client's
//     record names a different schedule.
//   - The header's markers must hash to the header's own
//     process_markers_sha256 (the planner wrote both).
//   - A production Load must carry the plan record's digest and it must be
//     that one: the schedule is outside the trajectory digest, so without it
//     an old client (one that cannot show what the plan switches) or a
//     substituted schedule would run under the trajectory's identity.
//   - Every marker names an output in the cell's Describe, and a torch-class
//     output only on a cell whose torch outputs are qualified. rt-core refuses
//     both again at preparation; this says it first, by name.
func processOutputsGate(h densejointtraj.Header, request LoadRequest, d control.Description) ([]string, error) {
	if len(h.ProcessMarkers) == 0 {
		if request.ProcessMarkersSHA256 != "" && !request.DryRun {
			return nil, httpError(409, "process_markers_mismatch",
				"the plan record says this plan switches outputs (schedule %s), but the loaded plan has no output schedule; plan the program again",
				shortDigest(request.ProcessMarkersSHA256))
		}
		return nil, nil
	}
	outputs := markerOutputs(h.ProcessMarkers)
	computed := densejointtraj.ProcessMarkersDigest(h.ProcessMarkers)
	if request.DryRun {
		// The dry run strips the schedule; only a record that names a
		// different one says this is not the plan the operator chose.
		if request.ProcessMarkersSHA256 != "" && request.ProcessMarkersSHA256 != computed {
			return nil, httpError(409, "process_markers_mismatch",
				"this plan's output schedule (%s, %s) is not the one the plan record names (%s); plan the program again",
				strings.Join(outputs, ", "), shortDigest(computed), shortDigest(request.ProcessMarkersSHA256))
		}
		return nil, nil
	}
	if h.ProcessMarkersSHA256 != computed {
		recorded := shortDigest(h.ProcessMarkersSHA256)
		if recorded == "" {
			recorded = "none"
		}
		return nil, httpError(409, "process_markers_mismatch",
			"this plan's output schedule (%s) does not hash to the digest its header records (%s); plan the program again",
			strings.Join(outputs, ", "), recorded)
	}
	if request.ProcessMarkersSHA256 != "" && request.ProcessMarkersSHA256 != computed {
		return nil, httpError(409, "process_markers_mismatch",
			"this plan's output schedule (%s, %s) is not the one the plan record names (%s); plan the program again",
			strings.Join(outputs, ", "), shortDigest(computed), shortDigest(request.ProcessMarkersSHA256))
	}
	if request.ProcessMarkersSHA256 == "" {
		return nil, httpError(409, "process_markers_unbound",
			"this plan switches outputs (%s), and this client did not send the plan's output schedule digest, so the OLP cannot check that the schedule is the one you planned. Update this browser or pendant and press again, or Load it as a dry run to check the motion",
			strings.Join(outputs, ", "))
	}
	io, err := describedOutputs(d)
	if err != nil {
		return nil, httpError(409, "io_not_configured", "%v", err)
	}
	if io == nil || (len(io.Outputs) == 0 && len(io.Inputs) == 0) {
		return nil, httpError(409, "io_not_configured",
			"this plan switches outputs (%s), and this cell has no outputs configured (its rt-core configuration has no io block). Load it as a dry run, or configure and qualify the cell's I/O terminal",
			strings.Join(outputs, ", "))
	}
	byName := map[string]describedOutput{}
	described := make([]string, 0, len(io.Outputs))
	for _, o := range io.Outputs {
		byName[o.Name] = o
		described = append(described, o.Name)
	}
	for _, m := range h.ProcessMarkers {
		if m.TimeoutMS == 0 && m.Monitor == "" {
			if m.StableMS != 0 {
				return nil, httpError(409, "io_marker_invalid", "input stability needs a timeout")
			}
			continue
		}
		found := false
		if io != nil {
			for _, in := range io.Inputs {
				found = found || in.Name == m.Action
			}
		}
		if !found || m.StableMS > m.TimeoutMS || (m.Monitor != "" && (m.Monitor != "start" && m.Monitor != "end" || m.TimeoutMS != 0 || m.StableMS != 0)) {
			return nil, httpError(409, "io_wait_invalid", "Wait for %s needs a configured input and valid timeout", m.Action)
		}
	}
	for _, name := range outputs {
		o, ok := byName[name]
		if !ok {
			return nil, httpError(409, "io_marker_invalid",
				"this plan switches output %q, which this cell does not have (its outputs: %s); plan the program again against this cell",
				name, strings.Join(described, ", "))
		}
		if o.Class == "torch" && !io.TorchQualified {
			return nil, httpError(409, "io_torch_unqualified",
				"this plan switches torch output %q, and this cell's torch outputs are not qualified (rt-core refuses them until the terminal checks in CELL_IO.md are recorded)",
				name)
		}
		if o.Class == "indicator" {
			return nil, httpError(409, "io_marker_invalid", "output %q is a controller-owned indicator; it cannot be programmed", name)
		}
	}
	return outputs, nil
}

// ioArmCodes are rt-core's io_arm refusals that leave the session loaded:
// nothing moved and nothing was switched, and the operator can fix the cause
// (close the interlock, wait for the terminal) and press Play again. Any
// other failure takes Play's usual path (Stop and release).
var ioArmCodes = map[string]bool{
	"io_not_configured": true, "io_fast_input_unsatisfied": true, "io_readback_disagreement": true,
	"io_exchange_lost": true, "io_torch_unqualified": true, "inhibited": true,
}

// ioArmRefusal is an io_arm refusal in operator words, naming the inputs or
// outputs the observation shows, or nil when err is not one of ioArmCodes.
func ioArmRefusal(err error, s *control.Status, outputs []string) *Error {
	var rejected *control.Rejected
	if !errors.As(err, &rejected) {
		return nil
	}
	code := strings.SplitN(rejected.Reason, ":", 2)[0]
	if !ioArmCodes[code] {
		return nil
	}
	switched := strings.Join(outputs, ", ")
	var detail string
	switch code {
	case "io_fast_input_unsatisfied":
		var open []string
		if s != nil {
			for _, in := range s.Core.IO.Inputs {
				if in.Fast && !(in.Valid && in.Value) {
					open = append(open, in.Name)
				}
			}
		}
		which := "a fast input"
		if len(open) > 0 {
			which = "fast input " + strings.Join(open, ", ")
		}
		detail = fmt.Sprintf("the cell did not arm its outputs: %s is not satisfied, so %s cannot switch on. Close it (consent or interlock) and press Play again; the program is still loaded and nothing moved", which, switched)
	case "io_readback_disagreement":
		var wrong []string
		if s != nil {
			for _, o := range s.Core.IO.Outputs {
				if o.Valid && o.Readback {
					wrong = append(wrong, o.Name)
				}
			}
		}
		which := "an output's wired readback has not been observed OFF since the last Stop"
		if len(wrong) > 0 {
			which = "the wired readback of " + strings.Join(wrong, ", ") + " reads ON while commanded OFF"
		}
		detail = fmt.Sprintf("the cell did not arm its outputs: %s. Check that output's wiring and the I/O terminal, then press Play again; the program is still loaded and nothing moved", which)
	case "io_exchange_lost":
		detail = "the cell did not arm its outputs: the I/O terminal has not exchanged data since the last cycle (bus or terminal fault). Check the terminal, then press Play again; the program is still loaded and nothing moved"
	case "io_not_configured":
		detail = fmt.Sprintf("the cell did not arm its outputs: rt-core has no outputs configured, so %s cannot switch. Load the plan as a dry run", switched)
	case "io_torch_unqualified":
		detail = "the cell did not arm its outputs: its torch outputs are not qualified (CELL_IO.md terminal checks)"
	case "inhibited":
		detail = "the cell did not arm its outputs: the bus is not ready or a safety fault is latched. Clear it (Arm resets resettable faults), then press Play again"
	}
	return httpError(409, code, "%s", detail)
}

// armOutputs sends io_arm on the held grant before StartProgram. Its receipt
// arms permission only: rt-core switches nothing until a marker's sample.
func (r *RTCoreController) armOutputs(ctx context.Context) error {
	r.mu.Lock()
	fence := r.fence
	r.mu.Unlock()
	request := &control.Request{Operation: "io_arm", Fence: fence}
	response, err := rtRequest(r, ctx, "rt_core_request_slow", func(call context.Context) (control.Response, error) { return r.c.Command(call, request) })
	if err == nil && (response.Operation != "io_arm" || response.Sequence == 0) {
		err = control.ErrProtocol
	}
	if err != nil {
		return err
	}
	r.mu.Lock()
	r.ioArmed = fence
	r.mu.Unlock()
	return nil
}

// settleOutputs disarms the outputs a finished session armed: once the
// session is no longer playing (done, or loaded again for a loop's next
// cycle), io_disarm goes out on the grant io_arm used. It runs off the
// observation lane under the operation lock, so it can never disarm the
// outputs a concurrent Play has just armed: that Play holds the lock across
// io_arm and StartProgram, and afterwards the session reads playing. A Stop,
// a fault and a grant change need nothing from here: rt-core fences the
// outputs OFF and clears permission in that cycle, and stopObserved forgets
// the armed grant.
func (r *RTCoreController) settleOutputs() {
	r.mu.Lock()
	armed := r.ioArmed
	due := armed.Session != "" && !r.ioDisarming && (r.session == nil || r.session.State != "playing")
	if due {
		r.ioDisarming = true
	}
	r.mu.Unlock()
	if !due {
		return
	}
	go func() {
		defer func() { r.mu.Lock(); r.ioDisarming = false; r.mu.Unlock() }()
		ctx, cancel := context.WithTimeout(context.Background(), r.callBudget()+rtOperationBudget)
		defer cancel()
		if err := r.opMu.LockContext(ctx); err != nil {
			return
		}
		defer r.opMu.Unlock()
		r.mu.Lock()
		current := r.ioArmed == armed && r.held && r.fence == armed && (r.session == nil || r.session.State != "playing")
		if r.ioArmed == armed && (!r.held || r.fence != armed) {
			r.ioArmed = control.Fence{}
		}
		state := ""
		if r.session != nil {
			state = r.session.State
		}
		r.mu.Unlock()
		if !current {
			return
		}
		request := &control.Request{Operation: "io_disarm", Fence: armed}
		response, err := rtRequest(r, ctx, "rt_core_request_slow", func(call context.Context) (control.Response, error) { return r.c.Command(call, request) })
		if err == nil && (response.Operation != "io_disarm" || response.Sequence == 0) {
			err = control.ErrProtocol
		}
		r.mu.Lock()
		if r.ioArmed == armed && (err == nil || r.fence != armed) {
			r.ioArmed = control.Fence{}
		}
		r.mu.Unlock()
		if err != nil {
			log.Printf("dense-execution: io_disarm after session %s: %v (outputs expire on their own clocks; Stop fences them OFF)", state, err)
		}
	}()
}
