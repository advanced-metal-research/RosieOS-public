//go:build linux

package denseexec

import (
	"context"
	"errors"
	"fmt"
	"math"
	"strings"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Unverified browser policy: 50 ms cadence, 100 ms input lifetime (two cadences).
const (
	jointJogCadence  = 50 * time.Millisecond
	jointJogLifetime = 100 * time.Millisecond
)

// jogLifetimeLocked uses the cell's published input policy for both jog modes.
// Missing or overflowing legacy policy stays at the LAN allowance, never widens it.
func (r *RTCoreController) jogLifetimeLocked() time.Duration {
	age := r.description.MaxJogInputAgeNS
	if age == 0 || age > math.MaxInt64 {
		return jointJogLifetime
	}
	return time.Duration(age)
}

type jointJogLane interface {
	UpdateAt([]float64, uint64, uint64) error
	End(context.Context) error
	Generation() uint64
}

type jointJogHold struct {
	context  context.Context
	lane     jointJogLane
	intent   JointJogIntent
	cancel   context.CancelFunc
	timer    *time.Timer
	last     time.Time
	sequence uint64
	origin   uint64
	lifetime time.Duration
	// A first joint press can still be opening/priming while the pendant sends
	// newer intents. Preparation accepts their receipt times, never motion.
	preparing *jointJogPreparation
	// Prepared lanes require input captured after readiness. An in-flight
	// request captured earlier cannot become the first motion sample later.
	readyOrigin uint64
	// sentOrigin is the origin of the last input sent to the lane. Clients
	// keep several intents in flight; two that reach OLP together can take
	// the lock out of order, and native refuses an origin that goes back
	// (jog_invalid_timing ended slow-link composed holds, 2026-10-07). An
	// intent overtaken by a newer one is dropped: latest wins.
	sentOrigin    uint64
	inputSequence uint64
	// Journal evidence only: the begin time lets an end line identify a hold
	// without inferring its axis from a stop dump.
	begun time.Time
	// path is set for a Cartesian path hold; its lane occupies this slot.
	path  *cartesianPathHold
	trace *traceHold
	// End evidence (jog_end.go): the client whose press began the hold, the
	// jogged axis's measured position at the press and its peak measured
	// speed (joint jogs; 10 Hz status).
	client       Client
	startQ       float64
	startValid   bool
	peakVelocity float64
}

type jointJogPreparation struct {
	origin uint64 // Latest received intent, not a transmitted native sample.
}

// newHoldLocked opens the hold record for a press. Hold r.mu.
func (r *RTCoreController) newHoldLocked(ctx context.Context, in JointJogIntent, stamp time.Time, origin uint64) *jointJogHold {
	holdContext, cancel := context.WithCancel(context.Background())
	h := &jointJogHold{lifetime: r.jogLifetimeLocked(), context: holdContext, intent: in, inputSequence: in.InputSequence, cancel: cancel, last: stamp, origin: origin, begun: stamp, client: ClientOf(ctx)}
	// The kept readout described the robot before this press moves it.
	r.lastPathLive = nil
	if s := r.observation; s != nil && in.cartesian == nil && in.Axis >= 0 && in.Axis < len(s.Axes) {
		h.startQ, h.startValid = s.Axes[in.Axis].LogicalPosition, true
	}
	return h
}

type remoteJointJog struct {
	*control.RemoteJogSession
	generation uint64
}

func (s *remoteJointJog) Generation() uint64 { return s.generation }
func (s *remoteJointJog) UpdateAt(v []float64, origin, _ uint64) error {
	return s.Update(v, origin)
}

// openJointJog uses only SDK producers, preserving their immutable fences,
// clock calibration and per-generation wire sequence on both supported transports.
func (r *RTCoreController) openJointJog(ctx context.Context, mask uint32, origin uint64) (jointJogLane, error) {
	if r.jogOpen != nil {
		return r.jogOpen(ctx, mask, origin)
	}
	c, ok := r.c.(*control.Client)
	if !ok {
		return nil, rtRefused("capability_unimplemented")
	}
	r.mu.Lock()
	lifetime := r.jogLifetimeLocked()
	r.mu.Unlock()
	if r.cfg.Address == "" {
		var s *control.JogSession
		var err error
		// Unverified LAN RTT bound: 20 ms, matching SDK lease selection.
		if r.cfg.CartesianModel != "" || c.Timing().RoundTrip > 20*time.Millisecond {
			s, err = c.PrepareJogSession(ctx, mask)
		} else {
			s, err = c.NewJogSessionAt(ctx, mask, origin, origin+uint64(lifetime))
		}
		if err != nil {
			return nil, err
		}
		return s, nil
	}
	s, err := c.NewRemoteJogSession(ctx, mask, uint64(lifetime), ipcclient.HostMonotonicNS)
	if err != nil {
		return nil, err
	}
	o, err := s.Observe(ctx)
	if err != nil {
		_ = r.endJointJog(s)
		return nil, err
	}
	return &remoteJointJog{RemoteJogSession: s, generation: o.JogGeneration}, nil
}

func (r *RTCoreController) endJointJog(s interface{ End(context.Context) error }) error {
	defer r.noteJogStopEnded(s)
	ctx, cancel := context.WithTimeout(context.Background(), r.callBudget())
	defer cancel()
	r.mu.Lock()
	fence, adapter, daemon := r.fence, r.adapter, r.daemon
	lifetime := r.jogLifetimeLocked()
	r.mu.Unlock()
	_, path := s.(pathJointLane)
	if lane, ok := s.(jointJogLane); ok && !path {
		_, _ = r.stationaryJointJog(ctx, lane, fence, adapter, daemon, lifetime)
	}
	err := s.End(ctx)
	if path {
		// A path End stops along the path and returns after native closure; a
		// lane native already closed (expiry, path end) is already retired.
		var rejection *control.PathJogRejection
		if errors.As(err, &rejection) && (rejection.NativeReason == ipcclient.JogReasonClosed || rejection.Reason == "jog_closed") {
			return nil
		}
	}
	var rejected *control.Rejected
	closedReceipt := errors.As(err, &rejected) && rejected.NativeJogResult != nil && rejected.NativeJogResult.ControlReason == ipcclient.JogReasonClosed
	// A native closed receipt establishes retirement; other uncertain cleanup
	// failures still require Stop.
	if !path && closedReceipt {
		return nil
	}
	// Adapter/WSS retirement can win the End race and clear its local session.
	// Only native proof for the original generation makes that stale receipt
	// successful cleanup; mapping it directly to fail would Stop and disarm.
	// The WSS handler ends a path session whenever its stream closes, which a
	// refused frame does (a Rosie 1430 arm, 2026-10-07: a refused first speed disarmed the
	// cell through this Stop). A path session stays open while it stops along
	// the path, so its proof is awaited within the call budget.
	// A path End refused Closed (native refuses control while the lane stops
	// along the path, e.g. after its own input expiry) takes the same proof:
	// the SDK returns that refusal as *control.Rejected, never as the
	// PathJogRejection matched above, and on 2026-10-07 15:33 a Rosie 1430 arm
	// disarmed at ~390 mm/s through this Stop.
	if errors.As(err, &rejected) && (rejected.Reason == "jog_session_stale" || path && closedReceipt) {
		if lane, ok := s.(jointJogLane); ok && fence.Generation != 0 {
			for {
				status, observed := r.c.Status(ctx)
				current := observed == nil && status.AdapterIncarnation == adapter && status.DaemonIncarnation == daemon &&
					status.Jog != nil && status.Jog.JogGeneration == lane.Generation() &&
					status.Jog.AppGrantGeneration == fence.Generation && status.Grant != nil &&
					status.Grant.GrantActive == 1 && status.Grant.AppGrantGeneration == fence.Generation
				if current && status.Jog.Open == 0 {
					return nil
				}
				if !path || !current {
					return err
				}
				select {
				case <-ctx.Done():
					return err
				case <-time.After(rtPollInterval):
				}
			}
		}
	}
	return err
}

// Native End/expiry inhibits a lane that has never accepted input. An
// acknowledged zero vector qualifies its halt without replaying held motion or
// acquiring authority. Backends without native jog observations remain legacy.
func (r *RTCoreController) stationaryJointJog(ctx context.Context, lane jointJogLane, fence control.Fence, adapter, daemon string, lifetime time.Duration) (bool, error) {
	// Priming/closure uses current native state and no historical handles. The
	// SDK projection preserves those fields without decoding retired handles.
	read := r.c.Status
	if observer, ok := r.c.(interface {
		StatusObservation(context.Context) (control.Status, error)
	}); ok {
		read = observer.StatusObservation
	}
	status, err := read(ctx)
	if err != nil {
		return false, err
	}
	if status.Jog == nil || fence.Generation == 0 {
		return false, nil
	}
	if status.AdapterIncarnation != adapter || status.DaemonIncarnation != daemon || status.Jog.JogGeneration != lane.Generation() ||
		status.Jog.AppGrantGeneration != fence.Generation || status.Core.Armed != 1 || status.Grant == nil ||
		status.Grant.GrantActive != 1 || status.Grant.AppGrantGeneration != fence.Generation {
		return false, rtRefused("jog_session_stale")
	}
	if status.Jog.Open == 0 || status.Jog.HasInput == 1 {
		return false, nil
	}
	origin := ipcclient.HostMonotonicNS()
	if err := lane.UpdateAt(make([]float64, len(status.Axes)), origin, origin+uint64(lifetime)); err != nil {
		return false, err
	}
	ticker := time.NewTicker(rtPollInterval)
	defer ticker.Stop()
	for {
		status, err = read(ctx)
		if err != nil {
			return false, err
		}
		if status.AdapterIncarnation != adapter || status.DaemonIncarnation != daemon || status.Jog == nil ||
			status.Jog.JogGeneration != lane.Generation() || status.Jog.Open == 0 {
			return false, rtRefused("jog_session_stale")
		}
		if status.Jog.HasInput == 1 {
			return true, nil
		}
		select {
		case <-ctx.Done():
			return false, ctx.Err()
		case <-ticker.C:
		}
	}
}

// retireJointJogLocked closes input before cleanup. All callers hold r.mu;
// pending Begin must clean up its own late receipt without publishing a sample.
func (r *RTCoreController) retireJointJogLocked(reason string) jointJogLane {
	h := r.jog
	r.jog = nil
	r.jogRevision++
	r.jogReason = reason
	// A press queued behind a stop belongs to the retired revision.
	r.jogQueued = nil
	if h == nil {
		r.jogEndBy, r.jogEndCause = Client{}, ""
		return nil
	}
	r.recordJogEndLocked(h, r.jogRevision-1, reason)
	if live := r.pathLiveLocked(h, false); live != nil {
		r.lastPathLive, r.lastPathLiveAt = live, time.Now()
	}
	logPathHoldEnd(h, r.jogRevision-1, reason)
	h.cancel()
	if h.timer != nil {
		h.timer.Stop()
	}
	r.noteJogStopRetiredLocked(h)
	return h.lane
}

func (r *RTCoreController) finishJointJog(h *jointJogHold, cause error) error {
	r.mu.Lock()
	if r.jog != h {
		r.mu.Unlock()
		return cause
	}
	reason := ""
	if cause != nil {
		reason = jointJogFailure(cause).Code
	}
	lane := r.retireJointJogLocked(reason)
	seq := r.jogEndSeq
	r.mu.Unlock()
	if lane != nil {
		if err := r.endJointJog(lane); err != nil {
			// An uncertain End requires the controller's existing Stop/Release fence.
			failure := r.fail(errors.Join(cause, err))
			r.mu.Lock()
			r.noteJogEndDisarmedLocked(seq)
			r.mu.Unlock()
			return failure
		}
	}
	return cause
}

func (r *RTCoreController) expireJointJog(h *jointJogHold, now time.Time) {
	r.mu.Lock()
	if r.jog != h {
		r.mu.Unlock()
		return
	}
	remaining := h.lifetime - now.Sub(h.last)
	if remaining > 0 {
		h.timer.Reset(remaining)
		r.mu.Unlock()
		return
	}
	lane := r.retireJointJogLocked("jog_input_deadline_expired")
	seq := r.jogEndSeq
	r.mu.Unlock()
	if lane != nil {
		if err := r.endJointJog(lane); err != nil {
			_ = r.fail(err)
			r.mu.Lock()
			r.noteJogEndDisarmedLocked(seq)
			r.mu.Unlock()
		}
	}
}

func (r *RTCoreController) JointJogState() JointJogState {
	r.mu.Lock()
	defer r.mu.Unlock()
	s := JointJogState{Revision: r.jogRevision, Reason: r.jogReason, CadenceMS: jointJogCadence.Milliseconds(), Move: r.moveState,
		FractionUpdates: r.description.PathJog != nil, ProportionalInput: true, LastEnd: r.lastJogEndLocked(), PathLive: r.jogLiveLocked()}
	if h := r.jog; h != nil {
		s.Active, s.Axis, s.Sequence = true, h.intent.Axis, h.sequence
		if h.lane != nil {
			s.Generation = h.lane.Generation()
		}
		if p := h.path; p != nil {
			o := p.observation
			s.Stopping, s.PathSpeed = o.Stopping, o.PathSpeed*p.scale()
			s.GoverningLimit = pathName(pathGoverningNames, o.Governing)
			s.StopReason = pathName(pathStopNames, o.StopReason)
		}
	}
	s.Queued = r.jogQueued != nil && !r.jogQueued.starting
	s.PipelinedIntents = true
	return s
}

// JointJog admits one held axis on the already armed dense grant. No request
// acquires or arms implicitly; stale End/Update cannot affect a replacement hold.
func (r *RTCoreController) JointJog(ctx context.Context, action string, in JointJogIntent) (JointJogState, error) {
	stamp, origin := time.Now(), ipcclient.HostMonotonicNS()
	err := r.jointJog(ctx, action, in, stamp, origin)
	if err != nil {
		return r.JointJogState(), jointJogFailure(err)
	}
	return r.JointJogState(), nil
}

func (r *RTCoreController) jointJog(ctx context.Context, action string, in JointJogIntent, stamp time.Time, origin uint64) error {
	r.mu.Lock()
	if in.TargetID == "" || in.TargetID != r.cfg.Binding.PairID {
		r.mu.Unlock()
		return rtRefused("dense_target_unavailable")
	}
	if r.session == nil || in.SessionID != r.session.ID || in.Revision != r.jogRevision {
		r.mu.Unlock()
		return rtRefused("jog_session_stale")
	}
	if action == "end" {
		// Ending a hold passes from any client (control.go); one that is not
		// the owner is named on the end.
		r.jogEndBy, r.jogEndCause = ClientOf(ctx), sanitizeEndCause(in.EndCause)
		lane := r.retireJointJogLocked("")
		seq := r.jogEndSeq
		r.mu.Unlock()
		if lane != nil {
			if err := r.endJointJog(lane); err != nil {
				failure := r.fail(err)
				r.mu.Lock()
				r.noteJogEndDisarmedLocked(seq)
				r.mu.Unlock()
				return failure
			}
		}
		return nil
	}
	if action != "begin" && action != "update" {
		r.mu.Unlock()
		return rtRefused("unknown_operation")
	}
	h := r.jog
	queued := queuedBeginOf(ctx)
	if action == "begin" && (h != nil || r.jogQueued != nil && r.jogQueued != queued) {
		r.mu.Unlock()
		return rtRefused("jog_mode_conflict")
	}
	// A press waiting behind a stop, or the begin it released still setting
	// up, takes the intents of its revision (jog_queue.go).
	if action == "update" && r.jogQueued != nil && (h == nil || h.timer == nil) {
		return r.refreshQueuedJogLocked(in, stamp, origin)
	}
	if action == "update" && h != nil &&
		(h.timer != nil && origin <= h.sentOrigin || h.preparing != nil && origin <= h.preparing.origin || h.readyOrigin != 0 && origin <= h.readyOrigin) {
		r.mu.Unlock()
		return nil
	}
	// A fresh joint Begin has an explicit preparation state. Matching newer
	// intents may refresh it below without touching the lane or a nil timer.
	// Path/Trace setup retain their separate admission and preparation contracts.
	if action == "update" && (h == nil || h.preparing == nil && (h.lane == nil || h.timer == nil)) {
		r.mu.Unlock()
		return rtRefused("jog_session_stale")
	}
	var refusal error
	switch {
	// A running loop is between cycles for moments (its reload, then Play),
	// not "playing": a jog there would move the arm the next cycle starts from.
	// A loaded plan that is not playing does not refuse a jog (Run program
	// moves back to the start and loads it again).
	case r.opCancel != nil || r.move != nil || r.session.State == "playing" || r.loop != nil:
		refusal = rtRefused("jog_mode_conflict")
	case !r.held || r.closed || r.inhibited:
		refusal = rtRefused("no_grant")
	case r.observation == nil || r.observation.Core.Armed != 1 || !r.ready(*r.observation) || time.Since(r.nativeSeen) > r.timingLocked().StatusAge:
		refusal = rtRefused("jog_not_ready")
	case in.trace == nil && in.cartesian == nil && (in.Axis < 0 || in.Axis >= len(r.description.Axes) || r.cfg.AxisMask&(1<<in.Axis) == 0):
		refusal = rtRefused("invalid_axis_mask")
	case in.Direction != -1 && in.Direction != 1 || math.IsNaN(in.Fraction) || math.IsInf(in.Fraction, 0) || in.Fraction <= 0 || in.Fraction > 1:
		refusal = rtRefused("jog_invalid_vector")
	case !validProportionalInput(in):
		refusal = rtRefused("jog_invalid_vector")
	case in.cartesian != nil && !validCartesian(in.cartesian):
		refusal = rtRefused("cartesian_input_invalid")
	case h != nil && (in.trace == nil) != (h.intent.trace == nil):
		r.mu.Unlock()
		return rtRefused("jog_mode_conflict")
	case h != nil && in.trace != nil && (in.trace.Digest != h.intent.trace.Digest || in.trace.Reverse != h.intent.trace.Reverse || in.trace.FirstSegment != h.intent.trace.FirstSegment || in.trace.LastSegment != h.intent.trace.LastSegment || in.trace.TimeScale != h.intent.trace.TimeScale || in.trace.AllowEntry != h.intent.trace.AllowEntry || in.trace.EndpointOnly != h.intent.trace.EndpointOnly || in.trace.LinearSpeed != h.intent.trace.LinearSpeed || in.trace.AngularSpeed != h.intent.trace.AngularSpeed):
		r.mu.Unlock()
		return rtRefused("jog_mode_conflict")
	case h != nil && (in.cartesian == nil) != (h.intent.cartesian == nil):
		r.mu.Unlock()
		return rtRefused("jog_mode_conflict")
	case h != nil && in.cartesian != nil && (in.cartesian.Frame != h.intent.cartesian.Frame || in.cartesian.Twist != h.intent.cartesian.Twist):
		// A path hold keeps its line: a new axis, frame or direction is a new
		// hold (release, then press). Its fraction may change (joystick).
		r.mu.Unlock()
		return rtRefused("jog_mode_conflict")
	case h != nil && (in.MaxFraction != h.intent.MaxFraction || in.cartesian == nil && (in.Axis != h.intent.Axis || in.Direction != h.intent.Direction || in.MaxFraction == 0 && in.Fraction != h.intent.Fraction)):
		// Direction and the proportional ceiling remain pinned. Legacy joint
		// consumers retain their fixed-speed contract.
		r.mu.Unlock()
		return rtRefused("jog_mode_conflict")
	case h != nil && in.MaxFraction > 0 && in.InputSequence <= h.inputSequence:
		// Drop an overtaken request without renewing the input deadline.
		r.mu.Unlock()
		return nil
	case h != nil && stamp.Sub(h.last) >= h.lifetime:
		refusal = rtRefused("jog_input_deadline_expired")
	}
	if refusal != nil {
		r.mu.Unlock()
		if h != nil {
			return r.finishJointJog(h, refusal)
		}
		return refusal
	}
	if action == "begin" && queued == nil {
		if stop := r.pendingJogStopLocked(in.SessionID); stop != nil {
			return r.queueJogBeginLocked(ctx, stop, in, stamp, origin)
		}
	}
	if h != nil {
		h.inputSequence = in.InputSequence
	}
	if action == "update" && h.preparing != nil {
		h.intent.Fraction = in.Fraction
		h.last, h.preparing.origin = stamp, origin
		r.lastHeartbeat = stamp
		r.mu.Unlock()
		return nil
	}
	if in.trace != nil {
		return r.traceJogLocked(ctx, action, h, in, stamp, origin)
	}
	if in.cartesian != nil {
		return r.cartesianPathJogLocked(ctx, action, h, in, stamp, origin)
	}
	mask := uint32(1 << in.Axis)
	velocities := make([]float64, len(r.description.Axes))
	{
		maxVelocity := r.description.Axes[in.Axis].MaxVelocity
		if math.IsNaN(maxVelocity) || math.IsInf(maxVelocity, 0) || maxVelocity <= 0 {
			r.mu.Unlock()
			return rtRefused("jog_not_ready")
		}
		// The hold carries the same measured reserve as a move: at 100 % the
		// operator asks for the described maximum, and the drive overshoot
		// jointMoveVelocityReserve records would otherwise put the limit itself
		// on the wire (owner decision 2026-09-21: the fraction runs to 100 %).
		velocities[in.Axis] = float64(in.Direction) * in.Fraction * maxVelocity * jointMoveVelocityReserve
	}
	if action == "begin" {
		// h.origin scopes the generation-less ingress refusals to this hold: a
		// press queued behind a stop predates refusals of the stopping session.
		h = r.newHoldLocked(ctx, in, stamp, max(origin, ipcclient.HostMonotonicNS()))
		h.preparing = &jointJogPreparation{origin: origin}
		holdContext := h.context
		r.jog, r.jogReason = h, ""
		r.logJogHoldBeginLocked(h)
		call, endCall := context.WithTimeout(ctx, r.timingLocked().CallBudget)
		defer endCall()
		stopCall := context.AfterFunc(holdContext, endCall)
		defer stopCall()
		r.mu.Unlock()
		lane, err := r.openJointJog(call, mask, origin)
		r.mu.Lock()
		if r.jog != h || call.Err() != nil {
			r.mu.Unlock()
			if lane != nil {
				_ = r.endJointJog(lane)
			}
			_ = r.finishJointJog(h, rtRefused("jog_input_deadline_expired"))
			return rtRefused("jog_session_stale")
		}
		if err != nil {
			r.mu.Unlock()
			return r.finishJointJog(h, err)
		}
		h.lane = lane
		if r.cfg.CartesianModel != "" {
			fence, adapter, daemon := r.fence, r.adapter, r.daemon
			r.mu.Unlock()
			primed, err := r.stationaryJointJog(call, lane, fence, adapter, daemon, h.lifetime)
			r.mu.Lock()
			if r.jog != h {
				r.mu.Unlock()
				return rtRefused("jog_session_stale")
			}
			if err != nil {
				r.mu.Unlock()
				return r.finishJointJog(h, err)
			}
			if primed {
				h.sequence++
			}
		}
		// Preserve the newest receipt's original lifetime across preparation;
		// readiness cannot refresh an expired press.
		if h.preparing.origin > origin {
			stamp, origin, in.Fraction = h.last, h.preparing.origin, h.intent.Fraction
		}
		stamp, origin, in.Fraction = r.latestQueuedInputLocked(ctx, stamp, origin, in.Fraction)
		remaining := h.lifetime - time.Since(stamp)
		if remaining <= 0 {
			r.mu.Unlock()
			return r.finishJointJog(h, rtRefused("jog_input_deadline_expired"))
		}
		velocities[in.Axis] = float64(in.Direction) * in.Fraction * r.description.Axes[in.Axis].MaxVelocity * jointMoveVelocityReserve
		h.last, h.intent.Fraction, h.preparing = stamp, in.Fraction, nil
		h.timer = time.AfterFunc(remaining, func() { r.expireJointJog(h, time.Now()) })
		// Stream termination retires this hold even without another browser input;
		// the optional observer also lets transport EOF exercise the cleanup path.
		if stream, ok := lane.(interface{ Rejection(context.Context) error }); ok {
			go func() { _ = r.finishJointJog(h, stream.Rejection(context.Background())) }()
		}
		// Unverified LAN RTT bound: 20 ms. Prepared lanes require a post-setup
		// capture even when setup happens to finish within the browser lifetime.
		if r.cfg.CartesianModel != "" || r.cfg.Address != "" || r.roundTrip > 20*time.Millisecond || !pressOutlivesNextIntent(stamp, h.lifetime) {
			h.readyOrigin = ipcclient.HostMonotonicNS()
			r.mu.Unlock()
			return nil
		}
	}
	// SDK updates are bounded/nonblocking. Stop can never wait behind Begin or
	// status I/O; the short critical section orders samples before retirement.
	err := h.lane.UpdateAt(velocities, origin, origin+uint64(h.lifetime))
	if errors.Is(err, control.ErrJogWouldBlock) {
		// The SDK drops a contended sample instead of queueing or replaying it.
		// A clock-refresh write can briefly own the lane. Wait for the next
		// fresh browser input; do not claim admission or renew the native deadline.
		r.mu.Unlock()
		return nil
	}
	if err == nil {
		h.intent.Fraction = in.Fraction
		h.sequence++
		h.last = stamp
		h.sentOrigin = origin
		r.lastHeartbeat = stamp
	}
	r.mu.Unlock()
	if err != nil {
		return r.finishJointJog(h, err)
	}
	return nil
}

func validProportionalInput(in JointJogIntent) bool {
	return finiteJoint(in.MaxFraction) && in.MaxFraction >= 0 && in.MaxFraction <= 1 &&
		(in.MaxFraction == 0 && in.InputSequence == 0 ||
			in.MaxFraction > 0 && in.InputSequence > 0 && in.Fraction <= in.MaxFraction)
}

// observeJointJog retires refused native frames; a successful datagram write is
// never treated as adapter admission. Only this generation's evidence applies.
func (r *RTCoreController) observeJointJog() {
	r.mu.Lock()
	h, s := r.jog, r.observation
	var cause error
	if h != nil && h.lane != nil && s != nil {
		if s.Core.Armed != 1 {
			cause = rtRefused("jog_not_ready")
		}
		if o := s.Jog; h.path == nil && o != nil && o.JogGeneration == h.lane.Generation() {
			code := o.UpdateReason
			if code == 0 {
				code = o.StateReason
			}
			if code == ipcclient.JogReasonLimited {
				// Native velocity/position governing is an accepted hold, not a
				// rejected session. Keep fresh input flowing so the axis can
				// finish decelerating and accept a later inward jog.
				r.jogReason = "jog_limited"
			} else if code != 0 {
				cause = rtRefused("%s", nativeJointJogReason(code))
			} else if r.jogReason == "jog_limited" {
				r.jogReason = ""
			}
		}
		if r.cfg.Address == "" && h.path == nil {
			o := s.JogIngress
			// Ingress has no generation; only refusals captured after this hold count.
			if o.LatestRefusal.Reason != 0 && o.LatestRefusal.NowHostNS >= h.origin {
				cause = rtRefused("%s", nativeJointJogReason(o.LatestRefusal.Reason))
			}
		}
	}
	r.mu.Unlock()
	if cause != nil {
		_ = r.finishJointJog(h, cause)
	}
}

func nativeJointJogReason(code uint32) string {
	// Generated native reason values; unknown codes remain typed refusals.
	names := map[uint32]string{
		ipcclient.JogReasonClosed: "jog_closed", ipcclient.JogReasonModeConflict: "jog_mode_conflict",
		ipcclient.JogReasonNotReady: "jog_not_ready", ipcclient.JogReasonWrongIdentity: "jog_wrong_identity",
		ipcclient.JogReasonInvalidGeneration: "jog_invalid_generation", ipcclient.JogReasonStaleSequence: "jog_stale_sequence",
		ipcclient.JogReasonInvalidVector: "jog_invalid_vector", ipcclient.JogReasonInvalidTiming: "jog_invalid_timing",
		ipcclient.JogReasonExpired: "jog_expired", ipcclient.JogReasonAuthorityLost: "jog_authority_lost",
		ipcclient.JogReasonClockReset: "jog_clock_reset", ipcclient.JogReasonGenerationExhausted: "jog_generation_exhausted",
		ipcclient.JogReasonRamping: "jog_ramping", ipcclient.JogReasonLimited: "jog_limited",
		ipcclient.JogReasonOutsideLimitsOutward: "outside_limits_outward",
		ipcclient.JogReasonPathStartMismatch:    "path_start_mismatch", ipcclient.JogReasonPathCapacity: "path_capacity",
		ipcclient.JogReasonPathOutOfOrder: "path_out_of_order", ipcclient.JogReasonPathInvalidSample: "path_invalid_sample",
		ipcclient.JogReasonPathEnded: "path_ended", ipcclient.JogReasonPathFault: "path_fault",
		ipcclient.JogReasonPathUnavailable: "cartesian_hold_unavailable", ipcclient.JogReasonPathInvalidConfig: "path_invalid_config",
	}
	if name, ok := names[code]; ok {
		return name
	}
	return fmt.Sprintf("unknown_jog_reason_%d", code)
}

func jointJogFailure(err error) *Error {
	var remote *control.JogRejection
	var rejected *control.Rejected
	var path *control.PathJogRejection
	if errors.As(err, &path) {
		code := path.Reason
		if path.NativeReason != 0 && code == "" {
			code = nativeJointJogReason(path.NativeReason)
		}
		return httpError(409, code, "%v", err)
	}
	if errors.As(err, &remote) {
		code := remote.Reason
		if remote.NativeReason != 0 {
			code = nativeJointJogReason(remote.NativeReason)
		}
		return httpError(409, code, "%v", err)
	}
	if errors.As(err, &rejected) && rejected.NativeJogResult != nil && rejected.NativeJogResult.ControlReason != 0 {
		return httpError(409, nativeJointJogReason(rejected.NativeJogResult.ControlReason), "%v", err)
	}
	if errors.Is(err, control.ErrJogWouldBlock) {
		return httpError(409, "jog_transport_congested", "%v", err)
	}
	failure := rtFailure(err)
	failure.Code = strings.SplitN(failure.Code, ":", 2)[0]
	return failure
}
