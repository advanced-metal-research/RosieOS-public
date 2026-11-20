//go:build linux

package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"net/http"
	"reflect"
	"sort"
	"strings"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

type phaseResult struct {
	Phase   string   `json:"phase"`
	Met     bool     `json:"expectations_met"`
	Reasons []string `json:"typed_reasons"`
}

// phase records the actual result, including expected refusals, so a composed
// run cannot hide a failed step behind successful terminal cleanup.
func (r *runner) phase(name string, work func() error) error {
	previous := r.activePhase
	r.activePhase = name
	defer func() { r.activePhase = previous }()
	before := r.e.reasonCount()
	err := work()
	row := phaseResult{Phase: name, Met: err == nil, Reasons: r.e.reasonsAfter(before)}
	if fault := sessionFault(r.last); err != nil && fault != nil {
		row.Reasons = append([]string{fault.Error()}, row.Reasons...)
	}
	r.rows = append(r.rows, row)
	return errors.Join(err, r.e.write(record{Kind: "phase", Operation: name, Response: row}))
}

func (r *runner) printPhases() {
	fmt.Fprintln(r.out, "phase\texpectations met\ttyped reasons seen")
	for _, row := range r.rows {
		fmt.Fprintf(r.out, "%s\t%t\t%s\n", row.Phase, row.Met, strings.Join(row.Reasons, ","))
	}
}

func (r *runner) stopRenewal() {
	r.c.StopRenewal()
	if r.renewDone != nil {
		<-r.renewDone
		r.renewDone = nil
	}
}

// release accepts a stale empty fence only when this runner confirmed its
// retirement through Stop. Preserve the typed receipt and require fresh native
// inhibition, preventing other refusals or missing evidence from becoming success.
func (r *runner) release(ctx context.Context) error {
	if _, err := r.command(ctx, "release", 0); err != nil {
		var refused *control.Rejected
		if !errors.As(err, &refused) || refused.Reason != "control_session_stale" ||
			r.retiredStop.Generation == 0 || r.c.Fence() != r.retiredStop || r.e.failure() != nil {
			return err
		}
	}
	completed := ipcclient.HostMonotonicNS()
	_, err := r.observe(ctx, func(s control.Status) (bool, error) {
		return s.Core.TimeNS >= completed && r.inhibited(s) && s.Core.LeaseValid == 0, nil
	})
	if err != nil {
		return err
	}
	return r.check("release", r.c.Fence().Session == "")
}

// sequence reuses the standalone phase operations under one renewed grant;
// run owns the final Stop and Release even when an intermediate step fails.
func (r *runner) sequence(ctx context.Context) error {
	if err := r.phase("inhibited", func() error { return r.inhibitedPhase(ctx) }); err != nil {
		return err
	}
	if err := r.phase("home", func() error { return r.home(ctx) }); err != nil {
		return err
	}
	if err := r.preEnable(r.last); err != nil {
		return err
	}
	if err := r.enable(ctx); err != nil {
		return err
	}
	return r.phase("hold", func() error { return r.hold(ctx) })
}

func (r *runner) expectRefusal(name, reason string, err error) error {
	var rejected *control.Rejected
	if e := r.check(name, errors.As(err, &rejected) && rejected.Reason == reason); e != nil {
		return errors.Join(e, err)
	}
	return r.e.failure()
}

// recovery compares the observed fault table and correlates a submitted reset
// before replacing its retired session. It never Homes or energises outputs.
func (r *runner) recovery(ctx context.Context, cancel context.CancelCauseFunc) error {
	s, err := r.status(ctx)
	if err != nil {
		return err
	}
	response, err := r.command(ctx, "recovery_status", 0)
	if err != nil {
		return err
	}
	var current control.RecoveryStatus
	if err := json.Unmarshal(response.Data, &current); err != nil {
		return errors.Join(control.ErrProtocol, err)
	}
	if err := r.check("recovery_fault_table", s.Core.Recovery != nil && current.HomeValidMask == s.Core.HomeValidMask && reflect.DeepEqual(current.Faults, s.Core.Recovery.Faults)); err != nil {
		return err
	}
	r.stopRenewal()
	old := r.c.Fence()
	response, resetErr := r.command(ctx, "reset_fault", uint32(r.o.mask))
	var reset control.RecoveryStatus
	if err := json.Unmarshal(response.Data, &reset); err != nil {
		return errors.Join(resetErr, control.ErrProtocol, err)
	}
	if response.Sequence == 0 || response.NativeResult == nil || response.NativeResult.Sequence != response.Sequence || reset.ResetSequence != response.Sequence {
		return errors.Join(resetErr, control.ErrProtocol)
	}
	if resetErr != nil {
		if err := r.expectRefusal("reset_typed_outcome", "fault_persists", resetErr); err != nil {
			return err
		}
		if reset.Reason != "fault_persists" || response.NativeResult.Result != ipcclient.ResultRejected {
			return control.ErrProtocol
		}
	} else if reset.Reason != "" || response.NativeResult.Result != ipcclient.ResultAccepted {
		return control.ErrProtocol
	}
	if err := r.check("reset_correlated", true); err != nil {
		return err
	}
	_, err = r.observe(ctx, func(s control.Status) (bool, error) { return r.inhibited(s) && s.Core.LeaseValid == 0, nil })
	if err != nil {
		return err
	}
	// Reset retires native authority but the SDK retains its old local fence.
	// Clear it only after observed lease retirement so Acquire remembers its reply.
	r.c.SetFence(control.Fence{})
	if _, err := r.command(ctx, "acquire", 0); err != nil {
		return err
	}
	if err := r.check("reset_reacquired", r.c.Fence().Session != "" && r.c.Fence().Session != old.Session && r.c.Fence().Generation > old.Generation); err != nil {
		return err
	}
	if err := r.startRenewal(ctx, cancel); err != nil {
		return err
	}
	response, err = r.command(ctx, "restore_anchor", uint32(r.o.mask))
	if current.HomeValidMask&uint32(r.o.mask) == 0 {
		return r.expectRefusal("anchor_missing_before_home", "anchor_missing", err)
	}
	if err != nil {
		return err
	}
	var restored control.RecoveryStatus
	if err := json.Unmarshal(response.Data, &restored); err != nil {
		return errors.Join(control.ErrProtocol, err)
	}
	return r.check("anchor_restored", response.Sequence != 0 && restored.HomeValidMask&uint32(r.o.mask) == uint32(r.o.mask) && restored.Reason == "")
}

// telemetry requires decoded native records and a visible diagnostic mark;
// a successful mark receipt alone cannot establish publication in /v1/events.
func (r *runner) telemetry(ctx context.Context) error {
	ctx, cancel := timeout(ctx, r.o.deadlineNS)
	defer cancel()
	origin := r.last
	before, err := r.c.EventsContext(ctx, 0)
	if err != nil {
		return err
	}
	markStart := ipcclient.HostMonotonicNS()
	response, err := r.request(ctx, control.Request{Operation: "mark_telemetry", Label: r.o.label})
	if err != nil {
		return err
	}
	if err := r.check("telemetry_mark_submitted", response.Sequence != 0); err != nil {
		return err
	}
	start := ipcclient.HostMonotonicNS()
	batch, err := r.c.TelemetryBatches(ctx, 0)
	if err := r.e.receipt("telemetry", start, map[string]any{"after": 0}, map[string]any{"header": batch.Header, "decoded_records": len(batch.Records)}, err); err != nil {
		return err
	}
	if err := r.check("telemetry_described_axes", batch.Header.AxisCount == uint32(len(r.description.Axes)) && len(batch.Records) >= 2); err != nil {
		return err
	}
	var previous uint64
	for _, v := range batch.Records {
		if v.Axes != uint32(len(r.description.Axes)) || v.Seq <= previous {
			return r.check("telemetry_increasing_sequence", false)
		}
		previous = v.Seq
	}
	r.expectations["telemetry_increasing_sequence"] = true
	visible, err := r.markEvent(ctx, before.LatestSequence, markStart, response.Sequence)
	if err != nil {
		return err
	}
	if err := r.noMotion(ctx, origin, "telemetry_no_motion"); err != nil {
		return err
	}
	return r.check("telemetry_mark_visible_in_events", visible)
}

// markEvent records the published receipt and requires its typed label and
// native sequence, preventing another mark or unrelated event claiming success.
func (r *runner) markEvent(ctx context.Context, after, markStart, nativeSequence uint64) (bool, error) {
	client := control.NewUnixHTTPClient(r.o.socket)
	defer client.CloseIdleConnections()
	request, err := http.NewRequestWithContext(ctx, http.MethodGet, fmt.Sprintf("http://localhost/v1/events?after=%d", after), nil)
	if err != nil {
		return false, err
	}
	start := ipcclient.HostMonotonicNS()
	response, err := client.Do(request)
	if err != nil {
		return false, r.e.receipt("events", start, map[string]any{"after": after}, nil, errors.Join(control.ErrTransport, err))
	}
	defer response.Body.Close()
	// Unverified bounded JSON response allocation: 8 MiB, matching the SDK cap.
	const eventBytes = 8 << 20
	raw, err := io.ReadAll(io.LimitReader(response.Body, eventBytes+1))
	if err != nil {
		return false, r.e.receipt("events", start, map[string]any{"after": after}, nil, errors.Join(control.ErrTransport, err))
	}
	var envelope control.Response
	if response.StatusCode != http.StatusOK || len(raw) > eventBytes || json.Unmarshal(raw, &envelope) != nil || envelope.Schema != "rosie.rt-control.response.v1" || envelope.Error != "" {
		return false, r.e.receipt("events", start, map[string]any{"after": after}, nil, control.ErrProtocol)
	}
	var batch struct {
		Events []json.RawMessage `json:"events"`
		Next   uint64            `json:"next_sequence"`
		Latest uint64            `json:"latest_sequence"`
	}
	if err := json.Unmarshal(envelope.Data, &batch); err != nil {
		return false, errors.Join(control.ErrProtocol, err)
	}
	if err := r.e.receipt("events", start, map[string]any{"after": after}, envelope, nil); err != nil {
		return false, err
	}
	visible, cursor := false, after
	for _, raw := range batch.Events {
		var event control.Event
		if json.Unmarshal(raw, &event) != nil || event.Sequence <= cursor || event.TimeNS == 0 {
			return false, control.ErrProtocol
		}
		if event.Dropped != nil {
			return false, errors.New("events dropped across telemetry mark")
		}
		if event.Sequence != cursor+1 {
			return false, control.ErrProtocol
		}
		cursor = event.Sequence
		if event.Type == "telemetry_mark" && event.TimeNS >= markStart && event.Mark != nil &&
			event.Mark.Label == r.o.label && event.Mark.NativeSequence == nativeSequence && nativeSequence != 0 {
			visible = true
		}
	}
	if batch.Next != cursor || batch.Latest < cursor {
		return false, control.ErrProtocol
	}
	return visible, nil
}

func (r *runner) noMotion(ctx context.Context, origin control.Status, name string) error {
	_, err := r.observe(ctx, func(s control.Status) (bool, error) {
		if err := r.stationary(s, origin); err != nil {
			return false, err
		}
		return s.Core.TimeNS > origin.Core.TimeNS, nil
	})
	if err != nil {
		return err
	}
	return r.check(name, true)
}

func (r *runner) stationary(s, origin control.Status) error {
	if !r.inhibited(s) {
		return errors.New("non-motion phase energised outputs")
	}
	for i, a := range s.Core.Axes {
		// Nine-motor bench registers-1.log 2026-09-13: the A6-EC encoders rest with a
		// 1..3 count spread, so an exact comparison reports motion on an idle
		// bench (bd-discard-1, bd-telemetry-1 on 2026-09-14). The bound is the
		// described completion tolerance in counts, never below 4 counts.
		bound := 4.0
		if i < len(r.description.Axes) {
			d := r.description.Axes[i]
			if finite(d.CompletionTolerance) && finite(d.CountsPerUnit) && d.CompletionTolerance*d.CountsPerUnit > bound {
				bound = d.CompletionTolerance * d.CountsPerUnit
			}
		}
		if math.Abs(float64(a.PositionCounts)-float64(origin.Core.Axes[i].PositionCounts)) > bound {
			return fmt.Errorf("axis %d moved during non-motion phase", i)
		}
	}
	return nil
}

func sortedReasons(values []string) []string {
	unique := map[string]bool{}
	for _, value := range values {
		unique[value] = true
	}
	out := []string{}
	for value := range unique {
		out = append(out, value)
	}
	sort.Strings(out)
	return out
}
