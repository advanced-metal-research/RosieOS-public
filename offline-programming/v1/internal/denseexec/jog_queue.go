//go:build linux

package denseexec

import (
	"context"
	"fmt"
	"log"
	"math"
	"reflect"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// A press that arrives while the previous hold of the same session is still
// stopping waits for that stop and then begins (owner request 2026-10-07).
// On a Rosie 1430 arm (OLP d7a81080, jog_jerk 150 rad/s^3) 7 of 51 presses came
// 123-200 ms after release while the previous Halt ramp (168 ms window + v/a)
// was still running: native refused the Begin (reason 1, Closed), OLP refused
// the press and the pendant's next update hit jog_session_stale. Design D
// 7.1/7.5 had refused such presses with jog_mode_conflict.
//
// The wait never touches the previous stop: nothing shortens, preempts or
// replaces it, and a queued press never stops, disarms or moves the robot.
// The deadman holds: the queued press keeps its own deadline (receipt + input
// lifetime); intents of its revision refresh it (latest wins); a release
// (end/halt) or Stop retires its revision and cancels it; a press whose input
// expires before the stop finishes is dropped, never started late. The wait
// is bounded by the previous stop's own bound (jogStopBoundLocked); past it
// the press is refused jog_mode_conflict as before.

// jogStopRecord is a retired hold's stop as a queued press waits for it.
type jogStopRecord struct {
	session string
	lane    any // the retired lane, matched when its End returns
	at      time.Time
	bound   time.Duration
	source  string
	// ended: OLP's End for the lane returned, at endedAt. Native began the
	// stop before End returned, so the bound runs from then (End itself took
	// ~20 ms locally in the composed fixture). A path End that met a stale
	// receipt proves closure by observing its own generation closed; a new
	// session opened before that would leave it without proof, and the
	// unproven End takes the Stop fence.
	ended   bool
	endedAt time.Time
}

// deadline is when the stop must be complete: its bound after End returned,
// or, while End is in flight, after End's own call budget. An End that fails
// takes the Stop fence, which cancels the press.
func (s *jogStopRecord) deadline(callBudget time.Duration) time.Time {
	if s.ended {
		return s.endedAt.Add(s.bound)
	}
	return s.at.Add(callBudget + s.bound)
}

// queuedJogBegin is the latest intent of a press waiting for a stop. It stays
// queued through the begin that follows (starting), so intents sent while
// that begin opens the lane refresh it too instead of meeting a hold without
// input permission (jog_session_stale).
type queuedJogBegin struct {
	intent    JointJogIntent
	stamp     time.Time
	origin    uint64
	refreshes int
	starting  bool
}

// queuedBeginKey marks the begin that releases a queued press.
type queuedBeginKey struct{}

func queuedBeginOf(ctx context.Context) *queuedJogBegin {
	q, _ := ctx.Value(queuedBeginKey{}).(*queuedJogBegin)
	return q
}

// latestQueuedInputLocked returns the newer of the begin's input and an
// intent absorbed while it set up (latest wins).
func (r *RTCoreController) latestQueuedInputLocked(ctx context.Context, stamp time.Time, origin uint64, fraction float64) (time.Time, uint64, float64) {
	if q := queuedBeginOf(ctx); q != nil && r.jogQueued == q && q.stamp.After(stamp) {
		if r.jog != nil {
			r.jog.inputSequence = q.intent.InputSequence
		}
		return q.stamp, q.origin, q.intent.Fraction
	}
	return stamp, origin, fraction
}

// jogStopBoundLocked returns how long a hold's stop may take after its End
// returned (native began it by then), derived from configured limits, and
// the bound's source.
//
//   - Joint lane ("halt_ramp"): the native HaltRamp bound
//     (rt-core/include/execution_policy.hpp), ceil(|v|/a) + window + one
//     cycle, with v the hold's commanded velocity, a its axis jog_acceleration
//     and the window the rtctl jog_jerk sizing ceil(2 A / jerk / cycle), A the
//     largest jog or maximum acceleration of any axis
//     (rt-core/tools/rtctl/jog_jerk.go). Native faults a ramp that overruns it.
//   - Path lane ("input_lifetime"): native bounds a path stop by the jerk-
//     limited brake under the path's own segment limits and publishes no
//     bound; OLP cannot reproduce those limits, so the bound is the cell's
//     input lifetime (Describe max_jog_input_age_ns: 250 ms on a Rosie 1430 arm, whose
//     400 mm/s path stops take ~100-170 ms). The same fallback covers a joint
//     lane whose limits are missing, and a cell advertising path_jog (so
//     configuring jog_jerk) when OLP has no compiled jerk.
//
// The margin covers observing completion: two measured round trips (a status
// request sent just before it), one wait poll and one native cycle.
func (r *RTCoreController) jogStopBoundLocked(h *jointJogHold) (time.Duration, string) {
	d := r.description
	cycle := time.Duration(d.CycleNS)
	margin := 2*r.roundTrip + pathPollInterval + cycle
	if bound, ok := r.haltRampBoundLocked(h); ok {
		return bound + margin, "halt_ramp"
	}
	return r.jogLifetimeLocked() + margin, "input_lifetime"
}

func (r *RTCoreController) haltRampBoundLocked(h *jointJogHold) (time.Duration, bool) {
	d := r.description
	in := h.intent
	if h.path != nil || in.cartesian != nil || in.Axis < 0 || in.Axis >= len(d.Axes) || d.CycleNS == 0 {
		return 0, false
	}
	axis := d.Axes[in.Axis]
	a := axis.JogAcceleration
	// A decelerating proportional hold may still be moving faster than its
	// latest request: bound its stop using the pinned ceiling.
	v := math.Abs(math.Max(in.Fraction, in.MaxFraction) * axis.MaxVelocity * jointMoveVelocityReserve)
	if !finiteJoint(a) || a <= 0 || !finiteJoint(v) {
		return 0, false
	}
	cycle := float64(d.CycleNS)
	ns := math.Ceil(v * 1e9 / a)
	if jerk := r.cfg.MoveJerk; jerk > 0 {
		var largest float64
		for _, x := range d.Axes {
			largest = math.Max(largest, x.JogAcceleration)
			if x.MaxAcceleration != nil {
				largest = math.Max(largest, *x.MaxAcceleration)
			}
		}
		window := math.Ceil(2 * largest / jerk * 1e9 / cycle)
		if !finiteJoint(window) || window < 2 {
			return 0, false
		}
		ns += window * cycle
	} else if d.PathJog != nil {
		return 0, false
	}
	ns += cycle
	if ns > float64(math.MaxInt64/2) {
		return 0, false
	}
	return time.Duration(ns), true
}

// noteJogStopRetiredLocked records the stop a retired lane is about to take.
func (r *RTCoreController) noteJogStopRetiredLocked(h *jointJogHold) {
	if h.lane == nil {
		return
	}
	bound, source := r.jogStopBoundLocked(h)
	r.jogStop = &jogStopRecord{session: h.intent.SessionID, lane: h.lane, at: time.Now(), bound: bound, source: source}
}

// noteJogStopEnded marks the retired lane's End as returned, whatever its result.
func (r *RTCoreController) noteJogStopEnded(lane any) {
	t := reflect.TypeOf(lane)
	if t == nil || !t.Comparable() {
		return
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if s := r.jogStop; s != nil && s.lane == lane && !s.ended {
		s.ended, s.endedAt = true, time.Now()
	}
}

// pendingJogStopLocked returns the previous hold's stop a press of this
// session must wait for: one not yet past its deadline. A stop still running
// past it is left to the ordinary admission, which refuses.
func (r *RTCoreController) pendingJogStopLocked(session string) *jogStopRecord {
	s := r.jogStop
	if s == nil || s.session != session || !time.Now().Before(s.deadline(r.timingLocked().CallBudget)) {
		return nil
	}
	return s
}

// jogStopComplete: native closed the stopping session (no input, no ramp or
// path stop), from the live jog observation. Motion done and the per-axis
// targets follow at the 10 Hz status publication; a path hold's start rule
// waits for those itself (beginCartesianPath, design R4).
func jogStopComplete(s *control.Status) bool {
	if s == nil {
		return false
	}
	j := s.Jog
	return j == nil || j.Open == 0 && j.StateReason != ipcclient.JogReasonRamping
}

// queueJogBeginLocked holds a press until the previous hold's stop is
// complete, then begins it with its latest intent. Entered with r.mu held;
// returns with it released.
func (r *RTCoreController) queueJogBeginLocked(ctx context.Context, stop *jogStopRecord, in JointJogIntent, stamp time.Time, origin uint64) error {
	q := &queuedJogBegin{intent: in, stamp: stamp, origin: origin}
	r.jogQueued = q
	lifetime := r.jogLifetimeLocked()
	budget := r.timingLocked().CallBudget
	deadline := stop.deadline(budget)
	queued := time.Now()
	log.Printf("jog hold: begin queued rev=%d kind=%s since_release_ms=%d end_returned=%v stop_bound_ms=%d bound_source=%s lifetime_ms=%d",
		in.Revision, jogKind(in), queued.Sub(stop.at).Milliseconds(), stop.ended, stop.bound.Milliseconds(), stop.source, lifetime.Milliseconds())
	r.mu.Unlock()
	for {
		// Only an observation requested after the press can prove completion;
		// it also refreshes the held command the path start rule reads. One
		// requested before the deadline may prove it even if it returns after;
		// only one requested after the deadline refuses.
		issued := time.Now()
		call, cancel := context.WithTimeout(ctx, budget)
		fresh := r.observe(call) == nil
		cancel()
		r.mu.Lock()
		now := time.Now()
		deadline = stop.deadline(budget)
		var cause error
		switch {
		case r.jogQueued != q:
			// Release, Stop or a conflicting intent retired this revision.
			cause = rtRefused("jog_session_stale")
		case ctx.Err() != nil:
			cause = ctx.Err()
		case !now.Before(q.stamp.Add(lifetime)):
			cause = rtRefused("jog_input_deadline_expired")
		case fresh && stop.ended && jogStopComplete(r.observation):
			// The stop record stays until its deadline: a released press that
			// expires waiting for the rest publication leaves the next press
			// to wait for that publication too, not to be refused.
			q.starting = true
			in, stamp, origin = q.intent, q.stamp, q.origin
			log.Printf("jog hold: queued begin released rev=%d kind=%s waited_ms=%d stop_ms=%d end_ms=%d input_age_ms=%d refreshes=%d",
				in.Revision, jogKind(in), now.Sub(queued).Milliseconds(), now.Sub(stop.at).Milliseconds(), stop.endedAt.Sub(stop.at).Milliseconds(), now.Sub(stamp).Milliseconds(), q.refreshes)
			r.mu.Unlock()
			err := r.jointJog(context.WithValue(ctx, queuedBeginKey{}, q), "begin", in, stamp, origin)
			r.mu.Lock()
			if r.jogQueued == q {
				r.jogQueued = nil
			}
			r.mu.Unlock()
			return err
		case !issued.Before(deadline):
			cause = rtRefused("jog_mode_conflict")
		}
		if cause != nil {
			if r.jogQueued == q {
				r.jogQueued = nil
			}
			native := "none"
			if s := r.observation; s != nil {
				native = fmt.Sprintf("motion_done=%v", s.Motion.Done)
				if j := s.Jog; j != nil {
					native += fmt.Sprintf(" jog_generation=%d jog_open=%d jog_state_reason=%d", j.JogGeneration, j.Open, j.StateReason)
				}
			}
			log.Printf("jog hold: queued begin dropped rev=%d kind=%s reason=%s waited_ms=%d stop_ms=%d stop_ended=%v observed_fresh=%v %s",
				q.intent.Revision, jogKind(q.intent), jointJogFailure(cause).Code, now.Sub(queued).Milliseconds(), now.Sub(stop.at).Milliseconds(), stop.ended, fresh, native)
			r.mu.Unlock()
			return cause
		}
		r.mu.Unlock()
		select {
		case <-ctx.Done():
		case <-time.After(pathPollInterval):
		}
	}
}

// refreshQueuedJogLocked applies an intent to the queued press: latest wins.
// It must name the same hold (a live hold's update rules); anything else
// cancels the press. Entered with r.mu held; returns with it released.
func (r *RTCoreController) refreshQueuedJogLocked(in JointJogIntent, stamp time.Time, origin uint64) error {
	q := r.jogQueued
	held := q.intent
	same := (in.cartesian == nil) == (held.cartesian == nil) && in.MaxFraction == held.MaxFraction &&
		validProportionalInput(in) && finiteJoint(in.Fraction) && in.Fraction > 0 && in.Fraction <= 1
	if same && in.cartesian != nil {
		same = in.cartesian.Frame == held.cartesian.Frame && in.cartesian.Twist == held.cartesian.Twist &&
			!math.IsNaN(in.Fraction) && in.Fraction > 0 && in.Fraction <= 1
	} else if same {
		same = in.Axis == held.Axis && in.Direction == held.Direction && (in.MaxFraction > 0 || in.Fraction == held.Fraction)
	}
	if !same {
		r.jogQueued = nil
		r.mu.Unlock()
		return rtRefused("jog_mode_conflict")
	}
	if in.MaxFraction > 0 && in.InputSequence <= held.InputSequence {
		r.mu.Unlock()
		return nil
	}
	if origin > q.origin {
		q.intent, q.stamp, q.origin = in, stamp, origin
	}
	q.refreshes++
	r.lastHeartbeat = stamp
	r.mu.Unlock()
	return nil
}

func jogKind(in JointJogIntent) string {
	if in.cartesian != nil {
		return "cartesian"
	}
	return "joint"
}

// pressOutlivesNextIntent: the press is sent as the hold's first input only
// if it is still live when the client's next intent is due, one cadence
// (jointJogCadence, published as cadence_ms) after Begin's reply. A press
// with less left would start the robot only for the deadman to stop it
// before that intent arrives; the next intent starts the hold instead. This
// sends less input than allowed, never more.
func pressOutlivesNextIntent(stamp time.Time, lifetime time.Duration) bool {
	return lifetime-time.Since(stamp) > jointJogCadence
}
