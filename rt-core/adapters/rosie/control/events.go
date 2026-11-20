//go:build linux

package control

import (
	"encoding/json"
	"errors"
	"fmt"
	"net/http"
	"runtime"
	"strconv"
	"sync"
	"time"

	"rosieos/rt-core/ipcclient"
)

// Unverified retention policy: 256 events per adapter incarnation.
const eventCapacity = 256

// Unverified delivery policy: at most 64 records in one polling/SSE write batch.
const eventBatchLimit = 64

// Unverified slow-reader write deadline: 5 seconds; no writer owns controller locks.
const eventWriteTimeout = 5 * time.Second

// eventRing never retains subscriber queues. Overwritten sequences remain
// identifiable, so a slow/disconnected reader cannot silently skip history.
type eventRing struct {
	mu       sync.Mutex
	entries  [eventCapacity]Event
	sequence uint64
	changed  chan struct{}
}

func (r *eventRing) append(event Event) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.sequence == ^uint64(0) {
		return
	} // Sequence exhaustion fails closed; never wrap.
	r.sequence++
	event.Sequence = r.sequence
	r.entries[(r.sequence-1)%eventCapacity] = event
	if r.changed != nil {
		close(r.changed)
	}
	r.changed = make(chan struct{})
}

// batch pins both history and its wakeup channel. A publication between the
// empty read and select therefore cannot strand a streaming reader.
func (r *eventRing) batch(after uint64) (EventBatch, <-chan struct{}, error) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.changed == nil {
		r.changed = make(chan struct{})
	}
	b := EventBatch{Events: make([]Event, 0, eventBatchLimit), NextSequence: after, LatestSequence: r.sequence}
	if r.sequence != 0 {
		b.AdapterIncarnation = r.entries[(r.sequence-1)%eventCapacity].AdapterIncarnation
	}
	if after > r.sequence {
		return b, r.changed, errors.New(ReasonEventCursorAhead)
	}
	first := uint64(1)
	if r.sequence > eventCapacity {
		first = r.sequence - eventCapacity + 1
	}
	if after < first-1 {
		oldest := r.entries[(first-1)%eventCapacity]
		b.Events = append(b.Events, Event{Sequence: first - 1, TimeNS: oldest.TimeNS, Type: "events_dropped", DaemonIncarnation: oldest.DaemonIncarnation, AdapterIncarnation: oldest.AdapterIncarnation, Dropped: &EventsDropped{FirstLostSequence: after + 1, LastLostSequence: first - 1}})
		after = first - 1
	}
	for seq := after + 1; seq != 0 && seq <= r.sequence && len(b.Events) < eventBatchLimit; seq++ {
		b.Events = append(b.Events, r.entries[(seq-1)%eventCapacity])
	}
	if len(b.Events) > 0 {
		b.NextSequence = b.Events[len(b.Events)-1].Sequence
	}
	return b, r.changed, nil
}

type eventObservation struct {
	initialized  bool
	core         ipcclient.Status
	jog          ipcclient.JogObservation
	incarnation  string
	pending      []Event
	pendingDrops uint64
}

// emitLocked labels observations with a daemon clock only. If no daemon
// timestamp exists, no motion observation is invented from the adapter clock.
func (c *Controller) emitLocked(event Event) {
	if event.TimeNS == 0 {
		if c.native != nil {
			if observed, err := c.native.FastObservation(); err == nil {
				event.TimeNS = observed.NowHostNs
			}
		}
		if event.TimeNS == 0 && c.native != nil {
			if observed, err := c.native.Status(); err == nil {
				event.TimeNS = observed.TimeNS
			}
		}
	}
	if event.TimeNS == 0 {
		event.TimeNS = c.eventObservation.core.TimeNS
	}
	if event.TimeNS == 0 {
		return
	}
	if event.DaemonIncarnation == "" {
		event.DaemonIncarnation = c.daemonIncarnation
	}
	if event.AdapterIncarnation == "" {
		event.AdapterIncarnation = c.adapterIncarnation
	}
	c.events.append(event)
}

func (c *Controller) grantEventLocked(kind string, stopping bool) {
	c.logGrantLocked(kind)
	c.emitLocked(Event{Type: kind, Grant: &GrantEvent{Generation: c.grant.Generation, Stopping: stopping}})
}

// handleEventLocked runs at the sole handle state writer, before another
// request can supersede a transition. It never publishes session capabilities.
func (c *Controller) handleEventLocked(h *preparation, from HandleState) {
	// Only a controller constructed with native observations can report motion.
	// Command-only implementations do not acquire a new Status dependency.
	if c.native == nil {
		return
	}
	c.emitLocked(Event{Type: "handle_transition", Handle: &HandleTransitionEvent{Handle: h.Handle, From: string(from), To: string(h.State), Identity: h.Identity, Generation: h.Generation, ExecutionGeneration: h.ExecutionGeneration, NativeSequence: h.NativeSequence}})
	kind := ""
	switch h.State {
	case HandleStarted:
		kind = "execution_started"
	case HandleConsumed:
		kind = "execution_completed"
	case HandleRetired:
		if from == HandleStarted {
			kind = "execution_aborted"
		}
	}
	if kind == "" {
		return
	}
	e := &ExecutionEvent{Handle: h.Handle, Identity: h.Identity, Generation: h.ExecutionGeneration, NativeSequence: h.NativeSequence, Recovery: []FaultRecovery{}}
	var stamp uint64
	if state, err := c.core.Status(); err == nil {
		e.FaultBits = state.ExecutionFaultReasons
		if state.Recovery != nil {
			e.Recovery = append(e.Recovery, state.Recovery.Faults...)
		}
		if kind == "execution_aborted" && e.FaultBits != 0 {
			kind = "execution_faulted"
		}
	}
	if motion, err := c.core.Motion(); err == nil && motion.CommandSequence == h.NativeSequence {
		stamp = motion.TimeNS
	}
	if kind == "execution_aborted" || kind == "execution_faulted" {
		// A ring motion update can precede the metrics fault publication. Wait
		// for a daemon sample beyond that update before choosing the terminal
		// fault classification; an older clear mask must not label it aborted.
		if stamp == 0 {
			stamp = c.eventObservation.core.TimeNS
		}
		if len(c.eventObservation.pending) == eventCapacity {
			c.eventObservation.pendingDrops++
			c.emitLocked(Event{Type: "publisher_overflow", TimeNS: stamp, Overflow: &PublisherOverflowEvent{PreviousDrops: c.eventObservation.pendingDrops - 1, TotalDrops: c.eventObservation.pendingDrops}})
		} else {
			c.eventObservation.pending = append(c.eventObservation.pending, Event{Type: "execution_aborted", TimeNS: stamp, Execution: e, DaemonIncarnation: c.daemonIncarnation, AdapterIncarnation: c.adapterIncarnation})
		}
		return
	}
	c.emitLocked(Event{Type: kind, TimeNS: stamp, Execution: e})
}

// observeEventsLocked compares daemon publications, not request intentions.
// Native ring loss is itself observable; no missing state is interpolated.
func (c *Controller) observeEventsLocked(state ipcclient.Status) {
	previous := c.eventObservation
	if !previous.initialized {
		c.emitLocked(Event{Type: "adapter_incarnation_changed", TimeNS: state.TimeNS, Incarnation: &IncarnationEvent{Current: c.adapterIncarnation}})
	}
	if !previous.initialized || previous.incarnation != c.daemonIncarnation {
		c.emitLocked(Event{Type: "daemon_incarnation_changed", TimeNS: state.TimeNS, Incarnation: &IncarnationEvent{Previous: previous.incarnation, Current: c.daemonIncarnation}})
		previous.core = ipcclient.Status{}
		previous.jog = ipcclient.JogObservation{}
	}
	old := previous.core
	retained := previous.pending[:0]
	for _, event := range previous.pending {
		if event.DaemonIncarnation != c.daemonIncarnation {
			c.emitLocked(event)
			continue
		}
		if state.TimeNS <= event.TimeNS {
			retained = append(retained, event)
			continue
		}
		event.Execution.FaultBits = state.ExecutionFaultReasons
		event.Execution.Recovery = []FaultRecovery{}
		if state.Recovery != nil {
			event.Execution.Recovery = append(event.Execution.Recovery, state.Recovery.Faults...)
		}
		if event.Execution.FaultBits != 0 {
			event.Type = "execution_faulted"
		}
		c.emitLocked(event)
	}
	previous.pending = retained
	if state.TimeNS != old.TimeNS {
		if state.AxisEnableMask != old.AxisEnableMask {
			c.emitLocked(Event{Type: "enable_changed", TimeNS: state.TimeNS, Enable: &EnableEvent{EnabledMask: state.AxisEnableMask, Armed: state.Armed != 0}})
		}
		if state.Armed != old.Armed {
			c.emitLocked(Event{Type: "arm_changed", TimeNS: state.TimeNS, Enable: &EnableEvent{EnabledMask: state.AxisEnableMask, Armed: state.Armed != 0}})
		}
		if state.HomeEpoch != old.HomeEpoch {
			c.emitLocked(Event{Type: "home_epoch_changed", TimeNS: state.TimeNS, Epoch: &EpochEvent{Previous: old.HomeEpoch, Current: state.HomeEpoch}})
		}
		changed := state.ExecutionFaultReasons ^ old.ExecutionFaultReasons
		for bit := uint32(1); bit != 0; bit <<= 1 {
			if changed&bit == 0 {
				continue
			}
			kind := "fault_cleared"
			source := old
			if state.ExecutionFaultReasons&bit != 0 {
				kind = "fault_latched"
				source = state
			}
			f := &FaultEvent{Bit: bit, AxisMask: source.SafetyFaultMask, RecoveryClass: "unknown"}
			if source.Recovery != nil {
				for _, r := range source.Recovery.Faults {
					if r.Bit < 32 && uint32(1)<<r.Bit == bit {
						f.AxisMask = r.AxisMask
						f.RecoveryClass = r.Recovery
					}
				}
			}
			c.emitLocked(Event{Type: kind, TimeNS: state.TimeNS, Fault: f})
		}
		// A brake warning is never a fault: it carries no recovery class and must
		// not be folded into fault_latched or execution failure.
		for i, axis := range state.Axes {
			previousSequence := uint64(0)
			if i < len(old.Axes) {
				previousSequence = old.Axes[i].BrakeHoldDiagnostic.WarningSequence
			}
			if d := axis.BrakeHoldDiagnostic; d.WarningSequence > previousSequence {
				c.emitLocked(Event{Type: "brake_warning", TimeNS: state.TimeNS, BrakeWarning: &BrakeWarningEvent{Axis: uint32(i), Diagnostic: d}})
			}
		}
		// A supervisor event ends an Arm attempt or warns; it is never a fault
		// and carries no recovery class.
		for i, axis := range state.Axes {
			previousSequence := uint64(0)
			if i < len(old.Axes) {
				previousSequence = old.Axes[i].DriveSupervisor.EventSequence
			}
			if d := axis.DriveSupervisor; d.EventSequence > previousSequence {
				c.emitLocked(Event{Type: "drive_supervisor", TimeNS: state.TimeNS, DriveSupervisor: &DriveSupervisorEvent{Axis: uint32(i), Diagnostic: d}})
			}
		}
		if state.BufferHealth.RingDrops > old.BufferHealth.RingDrops {
			c.emitLocked(Event{Type: "publisher_overflow", TimeNS: state.BufferHealth.TimeNS, Overflow: &PublisherOverflowEvent{PreviousDrops: old.BufferHealth.RingDrops, TotalDrops: state.BufferHealth.RingDrops}})
		}
	}
	if c.native != nil {
		if jog, err := c.native.JogObservation(); err == nil {
			before := previous.jog
			emit := func(kind string) {
				c.emitLocked(Event{Type: kind, TimeNS: jog.NowHostNs, Jog: &JogEvent{Generation: jog.JogGeneration, Reason: jog.StateReason, AxisMask: jog.AxisMask, VelocityScalePPM: jog.VelocityScalePpm}})
			}
			if jog.Open != 0 && (before.Open == 0 || before.JogGeneration != jog.JogGeneration) {
				emit("jog_begin")
			}
			if jog.Open == 0 && before.Open != 0 {
				if jog.StateReason == ipcclient.JogReasonExpired {
					emit("jog_expired")
				} else {
					emit("jog_end")
				}
			}
			if jog.StateReason != before.StateReason {
				switch jog.StateReason {
				case ipcclient.JogReasonRamping:
					emit("jog_ramping")
				case ipcclient.JogReasonLimited:
					emit("jog_limited")
				case ipcclient.JogReasonExpired:
					if before.Open == 0 || jog.Open != 0 {
						emit("jog_expired")
					}
				}
			}
			previous.jog = jog
		}
	}
	previous.core = state
	previous.initialized = true
	previous.incarnation = c.daemonIncarnation
	c.eventObservation = previous
}

// logicalStatus forwards axis/plan/store values from the same metrics document.
// Runtime sampling happens after releasing authority locks, so heap accounting
// cannot hold up a concurrent Stop or renewal.
func (c *Controller) logicalStatus() (ProcessStatus, error) {
	return c.logicalStatusWithHistory(true)
}

func (c *Controller) logicalStatusWithHistory(includeHistory bool) (ProcessStatus, error) {
	status, err := c.processSnapshot(includeHistory)
	if err != nil {
		return status, err
	}
	s := status.Core
	status.TimeNS = s.TimeNS
	status.Axes = make([]LogicalAxisStatus, len(s.Axes))
	for i, a := range s.Axes {
		status.Axes[i] = LogicalAxisStatus{AnchorDeltaRestore: a.AnchorDeltaRestore, TorqueRaw: a.TorqueRaw, CollisionWatchdog: a.CollisionWatchdog, BrakeHoldDiagnostic: a.BrakeHoldDiagnostic, DriveSupervisor: a.DriveSupervisor, TimeNS: a.TimeNS, LogicalPosition: a.LogicalPosition, LogicalValid: a.LogicalValid, LogicalTarget: a.LogicalTarget, LogicalVelocity: a.LogicalVelocity, Readiness: a.Readiness, PositionCounts: a.PositionCounts, Statusword: a.Statusword, BrakeState: a.BrakeState, HomeValid: a.HomeValid, VelocityActualCountsPerS: a.VelocityActualCountsPerS, FollowingErrorCounts: a.FollowingErrorCounts, VelocityActualValid: a.VelocityActualValid, FollowingErrorValid: a.FollowingErrorValid, MaxAbsVelocityActualCountsPerS: a.MaxAbsVelocityActualCountsPerS, MaxAbsFollowingErrorCounts: a.MaxAbsFollowingErrorCounts}
		status.Axes[i].DIBits, status.Axes[i].DIValid = a.DIBits, a.DIValid
		status.Axes[i].CalibrationValid, status.Axes[i].PositionState = a.CalibrationValid, a.PositionState
		status.Axes[i].PositionReason, status.Axes[i].AuditReason = a.PositionReason, a.AuditReason
		status.Axes[i].AuditLastVerifiedNS = a.AuditLastVerifiedNS
		status.Axes[i].AuditStatus = a.AuditStatus
		status.Axes[i].DriveAlarm, status.Axes[i].DriveAlarmText = a.DriveAlarm, a.DriveAlarmText
		status.Axes[i].DriveAlarmAuxCode, status.Axes[i].DriveAlarmAuxValid = a.DriveAlarmAuxCode, a.DriveAlarmAuxValid
		status.Axes[i].ExternalEnableActive, status.Axes[i].ExternalEnableValid = a.ExternalEnableActive, a.ExternalEnableValid
	}
	status.Generations = StatusGenerations{TimeNS: s.TimeNS, Native: s.ControlGeneration, ConfigurationEpoch: s.ConfigurationEpoch, HomeEpoch: s.HomeEpoch, Execution: status.Execution.Generation}
	if status.Grant != nil {
		status.Generations.Grant = status.Grant.AppGrantGeneration
		status.Generations.GrantTimeNS = status.Grant.NowHostNs
	}
	if status.Jog != nil {
		status.Generations.Jog = status.Jog.JogGeneration
		status.Generations.JogTimeNS = status.Jog.NowHostNs
	}
	status.PlanCursor = s.PlanCursor
	status.BufferHealth = s.BufferHealth
	var memory runtime.MemStats
	runtime.ReadMemStats(&memory)
	status.Adapter = AdapterStatus{HeapInuse: memory.HeapInuse, Sys: memory.Sys, Goroutines: uint32(runtime.NumGoroutine()), OpenPreparationSlots: 2}
	if c.preparations.json.Load() {
		status.Adapter.OpenPreparationSlots--
	}
	if c.preparations.program.Load() {
		status.Adapter.OpenPreparationSlots--
	}
	return status, nil
}

func eventCursor(r *http.Request) (uint64, error) {
	values, ok := r.URL.Query()["after"]
	if !ok {
		return 0, nil
	}
	if len(values) != 1 || values[0] == "" {
		return 0, errors.New(ReasonInvalidEventCursor)
	}
	for _, ch := range values[0] {
		if ch < '0' || ch > '9' {
			return 0, errors.New(ReasonInvalidEventCursor)
		}
	}
	n, err := strconv.ParseUint(values[0], 10, 64)
	if err != nil {
		return 0, errors.New(ReasonInvalidEventCursor)
	}
	return n, nil
}

// serveEvents uses a bounded ring for both listeners. Slow writes are bounded
// and never retain authority/observation locks or block native publishers.
func (h *Handler) serveEvents(w http.ResponseWriter, r *http.Request) {
	if !h.checkEventPrincipal(w, r) {
		return
	}
	after, err := eventCursor(r)
	var batch EventBatch
	var wake <-chan struct{}
	if err == nil {
		batch, wake, err = h.Controller.events.batch(after)
	}
	if err != nil {
		outcome := encodeResponse(Response{Schema: ResponseSchema, Operation: OperationSubscribeEvents}, err)
		w.WriteHeader(outcome.status)
		_, _ = w.Write(outcome.body)
		return
	}
	if r.URL.Path == "/v1/events" {
		_ = json.NewEncoder(w).Encode(Response{Schema: ResponseSchema, Operation: OperationSubscribeEvents, Data: batch})
		return
	}
	rc := http.NewResponseController(w)
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	if err = rc.SetWriteDeadline(time.Now().Add(eventWriteTimeout)); err != nil {
		return
	}
	if _, err = fmt.Fprint(w, ": rt-core events\n\n"); err != nil {
		return
	}
	if err = rc.Flush(); err != nil {
		return
	}
	for {
		if len(batch.Events) > 0 {
			if err = rc.SetWriteDeadline(time.Now().Add(eventWriteTimeout)); err != nil {
				return
			}
			for _, event := range batch.Events {
				body, e := json.Marshal(event)
				if e != nil {
					return
				}
				if _, e = fmt.Fprintf(w, "id: %d\nevent: %s\ndata: %s\n\n", event.Sequence, event.Type, body); e != nil {
					return
				}
			}
			if err = rc.Flush(); err != nil {
				return
			}
			after = batch.NextSequence
		}
		if after == batch.LatestSequence {
			select {
			case <-r.Context().Done():
				return
			case <-h.Controller.done:
				return
			case <-wake:
			}
		}
		batch, wake, err = h.Controller.events.batch(after)
		if err != nil {
			return
		}
	}
}
