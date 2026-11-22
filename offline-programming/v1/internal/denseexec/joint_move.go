//go:build linux

package denseexec

import (
	"context"
	"encoding/json"
	"errors"
	"log"
	"math"
	"strings"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Benchdrive's measured velocity reserve: A6-EC feedback peaked 12.7% above
// its planned velocity on 2026-09-14; retain its 0.75 profile multiplier.
const jointMoveVelocityReserve = 0.75

type jointMoveRun struct {
	cartesian           *CartesianMoveIntent
	axisMask            uint32
	endpoints           map[int]float64
	tolerances          map[int]float64
	preflightRefusal    bool
	intent              JointMoveIntent
	fence               control.Fence
	epoch               uint64
	handle, sequence    uint64
	endpoint, tolerance float64
	completionBudget    time.Duration
	// corners are profile samples knotPoints must keep (jointMovesProfile).
	corners []int
}

func finiteJoint(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

// moveAxes lists the joints of a move: the `axes` list when given, else the
// single axis and delta.
func moveAxes(in JointMoveIntent) []JointMoveAxis {
	if len(in.Axes) > 0 {
		return in.Axes
	}
	return []JointMoveAxis{{Axis: in.Axis, Degrees: in.Degrees}}
}

// moveJoint is one joint of a Move-to: its held origin, signed travel and the
// speed and acceleration it may use alone at the selected fraction.
type moveJoint struct {
	axis                   int
	origin, delta          float64
	velocity, acceleration float64
	described              ipcclient.AxisDescription
}

func newMoveJoint(d control.Description, axis int, degrees, fraction, origin float64) (moveJoint, error) {
	if axis < 0 || axis >= len(d.Axes) {
		return moveJoint{}, rtRefused("invalid_axis_mask")
	}
	a := d.Axes[axis]
	if a.PositionUnit != "rad" {
		return moveJoint{}, rtRefused("joint_move_units")
	}
	if !finiteJoint(degrees) || !finiteJoint(fraction) || fraction <= 0 || fraction > 1 {
		return moveJoint{}, rtRefused("joint_move_invalid")
	}
	delta := degrees * math.Pi / 180
	if !finiteJoint(origin) || !finiteJoint(a.MinPosition) || !finiteJoint(a.MaxPosition) || a.MinPosition >= a.MaxPosition ||
		!finiteJoint(delta) || math.Abs(delta) > a.MaxPosition-a.MinPosition || origin < a.MinPosition || origin > a.MaxPosition ||
		origin+delta < a.MinPosition || origin+delta > a.MaxPosition {
		return moveJoint{}, rtRefused("joint_move_limits")
	}
	if !finiteJoint(a.MaxVelocity) || a.MaxVelocity <= 0 || !finiteJoint(a.JogAcceleration) || a.JogAcceleration <= 0 ||
		!finiteJoint(a.CompletionTolerance) || a.CompletionTolerance <= 0 || d.CycleNS == 0 || d.CycleNS > math.MaxInt64 || d.MaxTrajectoryPoints < 3 {
		return moveJoint{}, rtRefused("joint_move_description_invalid")
	}
	if a.CompletionTimeoutNS == 0 || a.CompletionTimeoutNS > uint64(math.MaxInt64) {
		return moveJoint{}, rtRefused("joint_move_description_invalid")
	}
	acceleration := a.JogAcceleration
	if a.MaxAcceleration != nil {
		if !finiteJoint(*a.MaxAcceleration) || *a.MaxAcceleration <= 0 {
			return moveJoint{}, rtRefused("joint_move_description_invalid")
		}
		acceleration = math.Min(acceleration, *a.MaxAcceleration)
	}
	return moveJoint{axis: axis, origin: origin, delta: delta, velocity: a.MaxVelocity * fraction * jointMoveVelocityReserve,
		acceleration: acceleration * fraction, described: a}, nil
}

// progressProfile is a rest-to-rest profile over a distance. With a jerk it
// is the symmetric double-S: each acceleration phase (ramp) rises at +jerk for
// jerkTime, holds the peak acceleration, and falls at -jerk for jerkTime, so
// acceleration is continuous and never steps. Without one (jerk 0, jerkTime 0)
// it is the trapezoid or triangle whose one-cycle acceleration steps rang the
// wrist of a Rosie 1430 arm: J4, commanded still, reached 1.60 degrees of following error
// (above its bound for 33 ms of the 100 ms fault timeout) at the start of a
// J1/J6 Move-to on 2026-10-06 21:04.
type progressProfile struct {
	distance float64
	// jerk in distance/s^3; acceleration is the peak reached and peak the
	// cruise (or, without cruise, the highest) speed.
	jerk, jerkTime, acceleration, peak, ramp, cruise, total float64
}

func newProgressProfile(distance, velocity, acceleration, jerk float64) progressProfile {
	if distance == 0 {
		return progressProfile{}
	}
	// Reach the cruise speed: the peak acceleration needs jerkTime = a/j,
	// unless the speed is reached first (v*j < a^2).
	var jerkTime, ramp float64
	switch {
	case jerk > 0 && velocity*jerk < acceleration*acceleration:
		jerkTime = math.Sqrt(velocity / jerk)
		ramp = 2 * jerkTime
	case jerk > 0:
		jerkTime = acceleration / jerk
		ramp = jerkTime + velocity/acceleration
	default:
		ramp = velocity / acceleration
	}
	// A symmetric acceleration phase to v covers v*ramp/2; when the two
	// phases need more than the distance there is no cruise, and the phases
	// shrink until they cover exactly the distance.
	if velocity*ramp > distance {
		if jerk > 0 {
			jerkTime = acceleration / jerk
			ramp = (jerkTime + math.Sqrt(jerkTime*jerkTime+4*distance/acceleration)) / 2
			if ramp < 2*jerkTime {
				jerkTime = math.Cbrt(distance / (2 * jerk))
				ramp = 2 * jerkTime
			}
		} else {
			ramp = math.Sqrt(distance / acceleration)
		}
	}
	if jerk > 0 {
		acceleration = jerk * jerkTime
	}
	peak := acceleration * (ramp - jerkTime)
	cruise := math.Max(0, (distance-peak*ramp)/peak)
	return progressProfile{distance: distance, jerk: jerk, jerkTime: jerkTime, acceleration: acceleration,
		peak: peak, ramp: ramp, cruise: cruise, total: 2*ramp + cruise}
}

// accelerating is the distance covered t into an acceleration phase.
func (p *progressProfile) accelerating(t float64) float64 {
	switch {
	case t < p.jerkTime:
		return p.jerk * t * t * t / 6
	case t <= p.ramp-p.jerkTime:
		return p.acceleration / 6 * (3*t*t - 3*p.jerkTime*t + p.jerkTime*p.jerkTime)
	default:
		r := p.ramp - t
		return p.peak*p.ramp/2 - p.peak*r + p.jerk*r*r*r/6
	}
}

// acceleratingSpeed is the speed t into an acceleration phase.
func (p *progressProfile) acceleratingSpeed(t float64) float64 {
	switch {
	case t < p.jerkTime:
		return p.jerk * t * t / 2
	case t <= p.ramp-p.jerkTime:
		return p.acceleration * (t - p.jerkTime/2)
	default:
		r := p.ramp - t
		return p.peak - p.jerk*r*r/2
	}
}

// travelled is the distance covered at time t; deceleration mirrors
// acceleration.
func (p *progressProfile) travelled(t float64) float64 {
	switch {
	case p.distance == 0 || t <= 0:
		return 0
	case t >= p.total:
		return p.distance
	case t < p.ramp:
		return p.accelerating(t)
	case t < p.ramp+p.cruise:
		return p.accelerating(p.ramp) + p.peak*(t-p.ramp)
	default:
		return p.distance - p.accelerating(p.total-t)
	}
}

// speed is the rate of travel at time t.
func (p *progressProfile) speed(t float64) float64 {
	switch {
	case p.distance == 0 || t <= 0 || t >= p.total:
		return 0
	case t < p.ramp:
		return p.acceleratingSpeed(t)
	case t < p.ramp+p.cruise:
		return p.peak
	default:
		return p.acceleratingSpeed(p.total - t)
	}
}

// corners are the profile times where jerk (with a jerk) or acceleration
// (without) changes: the only places a cubic segment cannot follow the
// profile exactly.
func (p *progressProfile) corners() []float64 {
	if p.distance == 0 {
		return nil
	}
	decelerating := p.ramp + p.cruise
	return []float64{0, p.jerkTime, p.ramp - p.jerkTime, p.ramp,
		decelerating, decelerating + p.jerkTime, p.total - p.jerkTime, p.total}
}

// jointMoveProfile mirrors tools/benchdrive's cycle-sampled trapezoid (that
// package is main and cannot be imported). Starts use the held logical target;
// rounding stretches time and preserves an exact endpoint without bridging it.
func jointMoveProfile(d control.Description, in JointMoveIntent, origin float64) ([]control.Point, error) {
	points, _, err := jointMovesProfile(d, []JointMoveAxis{{Axis: in.Axis, Degrees: in.Degrees}}, in.Fraction, []float64{origin}, 0)
	return points, err
}

// jointMovesProfile plans several joints as one trajectory along the straight
// segment in joint space: every joint follows the same progress s(t) from 0
// to 1, q_j(t) = q0_j + dq_j*s(t), so all joints start, keep their ratios and
// finish together (owner rule, 2026-10-06: the path never deviates; when one
// joint is limited, all keep their ratios). s is one double-S whose speed,
// acceleration and jerk are the minima over moving joints of v_j/|dq_j|,
// a_j/|dq_j| and jerk/|dq_j|, so no joint exceeds its own speed or
// acceleration at the fraction, nor the one jerk (rad/s^3, the same for every
// joint; 0 = acceleration-limited only). b2e38b46 instead gave each joint its
// own profile stretched to the longest; their shapes differed, and the 21:04
// move's commanded joint vector left the segment by 0.72 degrees. All joints
// to 0° sends six such joints; the sequential version stopped where a
// transient between moves cancelled the next one (LA bench, 2026-09-20).
//
// Samples carry the profile's exact velocity, and corners lists the samples
// either side of every jerk or acceleration corner of s, which knotPoints
// keeps as knots so the native cubic segments reproduce the plan. Every
// joint's knot data is affine in s, so each cubic segment stays on the line.
func jointMovesProfile(d control.Description, axes []JointMoveAxis, fraction float64, origins []float64, jerk float64) ([]control.Point, []int, error) {
	return jointMovesProfileAtRate(d, axes, fraction, origins, jerk, math.Inf(1))
}

// jointMovesClock validates and constructs timing without allocating samples.
// Recovery uses it before Arm so a slow preset exceeding trajectory capacity
// cannot energize the robot and then discover that no move can be prepared.
func jointMovesClock(d control.Description, axes []JointMoveAxis, fraction float64, origins []float64, jerk, progressVelocity float64) ([]moveJoint, progressProfile, error) {
	if len(axes) == 0 || len(origins) != len(axes) || !finiteJoint(jerk) || jerk < 0 {
		return nil, progressProfile{}, rtRefused("joint_move_invalid")
	}
	if !(progressVelocity > 0) {
		return nil, progressProfile{}, rtRefused("joint_move_invalid")
	}
	joints := make([]moveJoint, len(axes))
	seen := map[int]bool{}
	velocity, acceleration, longest := progressVelocity, math.Inf(1), 0.0
	for i, axis := range axes {
		if seen[axis.Axis] {
			return nil, progressProfile{}, rtRefused("joint_move_invalid")
		}
		seen[axis.Axis] = true
		degrees := axis.Degrees
		if axis.PositionRad != nil {
			if degrees != 0 || !finiteJoint(*axis.PositionRad) {
				return nil, progressProfile{}, rtRefused("joint_move_invalid")
			}
			degrees = (*axis.PositionRad - origins[i]) * 180 / math.Pi
		} else if degrees == 0 {
			return nil, progressProfile{}, rtRefused("joint_move_invalid")
		}
		joint, err := newMoveJoint(d, axis.Axis, degrees, fraction, origins[i])
		if err != nil {
			return nil, progressProfile{}, err
		}
		joints[i] = joint
		if distance := math.Abs(joint.delta); distance > 0 {
			velocity = math.Min(velocity, joint.velocity/distance)
			acceleration = math.Min(acceleration, joint.acceleration/distance)
			longest = math.Max(longest, distance)
		}
	}
	var progress progressProfile
	if longest > 0 {
		progress = newProgressProfile(1, velocity, acceleration, jerk/longest)
	}
	cycle := float64(d.CycleNS) / float64(time.Second)
	steps := math.Ceil(math.Max(progress.total, cycle) / cycle)
	if !finiteJoint(steps) || steps < 1 || steps+1 >= float64(d.MaxTrajectoryPoints) || steps > float64(math.MaxInt64)/float64(d.CycleNS) {
		return nil, progressProfile{}, rtRefused("joint_move_too_many_points")
	}
	return joints, progress, nil
}

// progressVelocity is an additional ceiling on the shared normalized clock.
// It changes timing only: each joint remains q0 + delta*s, including Hermite
// segments around profile corners. Ordinary joint moves keep their old ceiling.
func jointMovesProfileAtRate(d control.Description, axes []JointMoveAxis, fraction float64, origins []float64, jerk, progressVelocity float64) ([]control.Point, []int, error) {
	joints, progress, err := jointMovesClock(d, axes, fraction, origins, jerk, progressVelocity)
	if err != nil {
		return nil, nil, err
	}
	cycle := float64(d.CycleNS) / float64(time.Second)
	total := math.Max(progress.total, cycle)
	steps := math.Ceil(total / cycle)
	if !finiteJoint(steps) || steps < 1 || steps+1 >= float64(d.MaxTrajectoryPoints) || steps > float64(math.MaxInt64)/float64(d.CycleNS) {
		return nil, nil, rtRefused("joint_move_too_many_points")
	}
	scale := total / (steps * cycle)
	points := make([]control.Point, int(steps)+1)
	for i := range points {
		// Rounding up to whole cycles runs the progress clock slower by scale.
		t := float64(i) * cycle * scale
		s, rate := progress.travelled(t), progress.speed(t)*scale
		position := make([]float64, len(d.Axes))
		velocity := make([]float64, len(d.Axes))
		for _, j := range joints {
			position[j.axis], velocity[j.axis] = j.origin+j.delta*s, j.delta*rate
			if i == len(points)-1 {
				position[j.axis], velocity[j.axis] = j.origin+j.delta, 0
			}
		}
		points[i] = control.Point{Time: time.Duration(uint64(i) * d.CycleNS), Position: position, Velocity: velocity}
		for _, j := range joints {
			a := j.described
			if !finiteJoint(position[j.axis]) || position[j.axis] < a.MinPosition || position[j.axis] > a.MaxPosition {
				return nil, nil, rtRefused("joint_move_limits")
			}
			if i > 0 && math.Abs(position[j.axis]-points[i-1].Position[j.axis])/cycle > a.MaxVelocity*fraction {
				return nil, nil, rtRefused("joint_move_limits")
			}
		}
	}
	var corners []int
	if progress.distance > 0 {
		// Profile time c is reached at sample c / (cycle * scale).
		for _, c := range progress.corners() {
			k := c / (cycle * scale)
			corners = append(corners, int(math.Floor(k)), int(math.Ceil(k)))
		}
	}
	return points, corners, nil
}

// JointMove reserves the existing armed grant without queueing another motion.
// Stop cancels it independently of preparation/observation; late handles are
// discarded with their original fence and can never adopt replacement authority.
//
// Stop first, then say done (owner decision 2026-11-20): when a Stop, Disarm
// or grant loss on another goroutine ends the move, JointMove returns only
// after that Stop and its Release have been sent and their outcome is
// recorded, and the late-handle discard goes out after them. The returned
// state is read after that outcome; the error is rt_core_inhibited when the
// Stop left the controller inhibited or its outcome unconfirmed, otherwise
// the Stop's reason. Before this, the cancelled move could take the Stop lane
// first and return (discard sent, Stop not yet sent) while the Stop owner
// still waited for the lane: 1 of 500 -race runs of the grant-loss test
// (linux/arm64 container, 2026-11-20), 8 times in CI. The wait holds no lock
// and lasts until every Stop in flight has settled, not only the one that
// ended the move. It is finite: each overlapping Stop takes at most its call
// budget plus the 500 ms observation of stopConfirmedByObservation, and a
// later Stop finds nothing held and returns quickly.
func (r *RTCoreController) JointMove(parent context.Context, in JointMoveIntent) (JointMoveState, error) {
	return r.boundedMove(parent, in, nil)
}

// awaitStopOutcome is the wait JointMove's contract names: holding no lock, it
// waits until every Stop in flight (one of them ended this move) has recorded
// its outcome, then returns the error the move reports for it.
func (r *RTCoreController) awaitStopOutcome() error {
	r.mu.Lock()
	settled, awaiting := r.stopSettledLocked(), r.testMoveAwaitsStop
	r.mu.Unlock()
	if awaiting != nil {
		awaiting()
	}
	<-settled
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.inhibited || r.stopUncertain {
		return httpError(409, "rt_core_inhibited", "%s Cause: %s", r.recoveryActionLocked(), r.reason)
	}
	return rtRefused("%s", r.moveState.Reason)
}

func (r *RTCoreController) boundedMove(parent context.Context, in JointMoveIntent, cartesian *CartesianMoveIntent) (JointMoveState, error) {
	axes := moveAxes(in)
	if cartesian == nil {
		// The first listed joint stands for the move in its state; a
		// several-axis intent carries the list as well.
		in.Axis, in.Degrees = axes[0].Axis, axes[0].Degrees
	}
	var listed []int
	if len(in.Axes) > 0 {
		for _, axis := range in.Axes {
			listed = append(listed, axis.Axis)
		}
	}
	refuse := func(code string) (JointMoveState, error) {
		return JointMoveState{Axis: in.Axis, State: "refused", Reason: code, Axes: listed}, jointJogFailure(rtRefused("%s", code))
	}
	if !r.opMu.TryLock() {
		return refuse("jog_mode_conflict")
	}
	defer r.opMu.Unlock()
	ctx, cancel := context.WithCancel(parent)
	defer cancel()
	r.stopMu.Lock()
	r.mu.Lock()
	code := ""
	switch {
	case in.TargetID == "" || in.TargetID != r.cfg.Binding.PairID:
		code = "dense_target_unavailable"
	case r.session == nil || in.SessionID != r.session.ID || in.Revision != r.jogRevision:
		code = "jog_session_stale"
	case !r.held || r.closed || r.inhibited || r.stopping != 0:
		code = "no_grant"
	case r.jog != nil || r.move != nil || r.session.State == "playing" || r.loop != nil:
		code = "jog_mode_conflict"
	case cartesian == nil && !r.moveAxesAdmittedLocked(axes):
		code = "invalid_axis_mask"
	case r.observation == nil || r.observation.Core.Armed != 1 || !r.ready(*r.observation) || time.Since(r.nativeSeen) > r.timingLocked().StatusAge:
		code = "jog_not_ready"
	}
	if code != "" {
		r.mu.Unlock()
		r.stopMu.Unlock()
		return refuse(code)
	}
	m := &jointMoveRun{intent: in, fence: r.fence, epoch: r.epoch, cartesian: cartesian}
	for _, axis := range axes {
		m.axisMask |= 1 << uint(axis.Axis)
	}
	r.move, r.opCancel = m, cancel
	// A move takes the robot away from the pose the kept hold readout described.
	r.lastPathLive = nil
	r.jogRevision++
	r.jogReason = ""
	r.moveState = JointMoveState{Axis: in.Axis, State: "preparing", Revision: r.jogRevision, Axes: listed}
	r.mu.Unlock()
	r.stopMu.Unlock()
	err := r.executeJointMove(ctx, m)
	cancel()
	r.mu.Lock()
	current := r.epoch == m.epoch && r.move == m
	r.opCancel = nil
	if current && errors.Is(err, context.Canceled) {
		err = rtRefused("client_disconnected")
	}
	r.mu.Unlock()
	if !current {
		// Only Stop moves the epoch or clears r.move, so a Stop is in flight or
		// done; its Stop and Release go out before this move's discard.
		err = r.awaitStopOutcome()
	}
	if err != nil {
		requestSlow := slowRequest(err)
		if current && !m.preflightRefusal {
			if requestSlow {
				err = r.fail(err)
			} else {
				_, _ = r.stop(jointJogFailure(err).Code)
			}
		}
		if m.handle != 0 && !requestSlow {
			r.stopMu.Lock()
			cleanup, done := context.WithTimeout(context.Background(), r.callBudget())
			// A Stop that ended this move, or the move's own Stop above, has
			// already sent Stop and Release, which retire live handles
			// (awaitStopOutcome). The explicit discard also covers a late
			// Prepare receipt; a stale fence cannot name a new grant.
			_, _ = r.c.Command(cleanup, &control.Request{Operation: "discard_trajectory", Fence: m.fence, Handle: m.handle})
			done()
			r.stopMu.Unlock()
		}
	}
	r.mu.Lock()
	if current && r.epoch != m.epoch {
		// A Stop arrived after the move finished, or the move's own Stop above
		// ran: report only that Stop's recorded outcome.
		current = false
		r.mu.Unlock()
		if stopErr := r.awaitStopOutcome(); err == nil {
			err = stopErr
		}
		r.mu.Lock()
	}
	if r.move == m {
		r.move = nil
		if m.cartesian != nil {
			// Resume idle browser supervision from completion, so a disconnected
			// request cannot cause an immediate Stop after a successful step.
			r.lastHeartbeat = time.Now()
		}
	}
	if current {
		r.moveState.State = "done"
		if err != nil {
			r.moveState.State, r.moveState.Reason = "refused", jointJogFailure(err).Code
		}
	}
	if !current && err == nil {
		err = rtRefused("%s", r.moveState.Reason)
	}
	state := r.moveState
	r.mu.Unlock()
	if err != nil {
		return state, jointJogFailure(err)
	}
	return state, nil
}

// moveAxesAdmittedLocked: every listed joint is described, in the held mask,
// and listed once. Caller holds r.mu.
func (r *RTCoreController) moveAxesAdmittedLocked(axes []JointMoveAxis) bool {
	seen := map[int]bool{}
	for _, axis := range axes {
		if axis.Axis < 0 || axis.Axis >= len(r.description.Axes) || r.cfg.AxisMask&(1<<uint(axis.Axis)) == 0 || seen[axis.Axis] {
			return false
		}
		seen[axis.Axis] = true
	}
	return len(axes) > 0
}

func moveHandle(s control.Status, handle uint64) (string, uint64) {
	for _, h := range s.Execution.Handles {
		if h.Handle == handle {
			return string(h.State), h.NativeSequence
		}
	}
	return "", 0
}

// observeJointMove requires fresh ready feedback and correlated handle evidence.
// A receipt, an unrelated Done, or an endpoint without completion cannot pass.
func (r *RTCoreController) observeJointMove(ctx context.Context, m *jointMoveRun, predicate func(control.Status) (bool, error)) error {
	ticker := time.NewTicker(rtPollInterval)
	defer ticker.Stop()
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		err := r.observe(ctx)
		if err != nil {
			return err
		}
		r.mu.Lock()
		current := r.move == m && r.epoch == m.epoch && r.held
		s := *r.observation
		r.mu.Unlock()
		if !current {
			return rtRefused("control_session_stale")
		}
		if s.Core.Armed != 1 || !r.ready(s) {
			return rtRefused("jog_not_ready")
		}
		done, err := predicate(s)
		if err != nil || done {
			return err
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
		}
	}
}

func (r *RTCoreController) executeJointMove(parent context.Context, m *jointMoveRun) error {
	if m.cartesian != nil {
		return r.executeCartesianMove(parent, m)
	}
	ctx, cancel := context.WithTimeout(parent, rtOperationBudget)
	defer cancel()
	var points []control.Point
	var heldAfter uint64
	err := r.observeJointMove(ctx, m, func(s control.Status) (bool, error) {
		if !s.Motion.Done {
			heldAfter = 0
			return false, nil
		}
		if heldAfter == 0 {
			heldAfter = s.Motion.TimeNS
		}
		// Motion and axis packets publish independently. Wait for a held-axis
		// publication at or after Done on every joint of the move, preventing a
		// pre-completion origin.
		axes := moveAxes(m.intent)
		origins := make([]float64, len(axes))
		for i, axis := range axes {
			if heldAfter == 0 || axis.Axis < 0 || axis.Axis >= len(s.Axes) || s.Axes[axis.Axis].TimeNS < heldAfter {
				return false, nil
			}
			origins[i] = s.Axes[axis.Axis].LogicalTarget
		}
		r.mu.Lock()
		d := r.description
		var geometry RobotGeometry
		if m.intent.returnSpeeds != nil {
			geometry = r.robotGeometryLocked()
		}
		r.mu.Unlock()
		var err error
		progressVelocity := math.Inf(1)
		if m.intent.returnSpeeds != nil {
			var ready bool
			progressVelocity, ready, err = returnProgressVelocity(geometry.Kinematics, d, s, r.cfg.AxisMask, heldAfter, axes, m.intent.Fraction, *m.intent.returnSpeeds)
			if err != nil {
				m.preflightRefusal = true
				return false, err
			}
			if !ready {
				return false, nil
			}
		}
		points, m.corners, err = jointMovesProfileAtRate(d, axes, m.intent.Fraction, origins, r.cfg.MoveJerk, progressVelocity)
		if err != nil && m.intent.returnSpeeds != nil {
			// A local profile refusal sent no trajectory and is not a robot
			// fault. In particular, a very slow return can exceed point capacity.
			m.preflightRefusal = true
		}
		if err == nil {
			m.endpoint = points[len(points)-1].Position[m.intent.Axis]
			m.tolerance = d.Axes[m.intent.Axis].CompletionTolerance
			last := points[len(points)-1].Time
			settling := d.Axes[m.intent.Axis].CompletionTimeoutNS
			if len(axes) > 1 {
				// Completion is judged on every joint against its own tolerance,
				// within the longest described settling.
				m.endpoints, m.tolerances = map[int]float64{}, map[int]float64{}
				for _, axis := range axes {
					m.endpoints[axis.Axis] = points[len(points)-1].Position[axis.Axis]
					m.tolerances[axis.Axis] = d.Axes[axis.Axis].CompletionTolerance
					settling = max(settling, d.Axes[axis.Axis].CompletionTimeoutNS)
				}
			}
			budget := r.callBudget()
			if settling > uint64(math.MaxInt64)-uint64(budget) || uint64(last) > uint64(math.MaxInt64)-settling-uint64(budget) {
				return false, rtRefused("joint_move_too_many_points")
			}
			// The actual prepared sample clock plus described settling and the
			// negotiated call budget bounds completion without truncating slow moves.
			m.completionBudget = last + time.Duration(settling) + budget
		}
		return true, err
	})
	if err != nil {
		return err
	}
	return r.executeMovePoints(parent, m, points)
}

// executeMovePoints owns preparation, start and correlated completion for both
// move types; neither can bypass native admission or adopt an unrelated Done.
func (r *RTCoreController) executeMovePoints(parent context.Context, m *jointMoveRun, points []control.Point) error {
	ctx, cancel := context.WithTimeout(parent, rtOperationBudget)
	defer cancel()
	// The native path interpolates Hermite segments between knots that carry a
	// velocity, so the upload is the profile's knots, not its per-cycle samples.
	knots := knotPoints(points, time.Duration(r.description.CycleNS), m.corners...)
	prepare := &control.Request{Operation: "prepare_trajectory", Fence: m.fence, AxisMask: m.axisMask, Points: knots}
	// One attempt only: a prepare is not idempotent (a second upload meets the
	// adapter's busy refusal while it still decodes the first), and its budget
	// grows with the body so a slow link lengthens the wait instead of expiring
	// it. Nothing has moved yet, so a timeout or busy here is a refusal that
	// keeps the machine armed and the operator retries.
	body, _ := json.Marshal(prepare)
	prepareCtx, prepareCancel := context.WithTimeout(ctx, r.callBudget()+control.UploadAllowance(len(body)))
	prepared, err := r.c.Command(prepareCtx, prepare)
	prepareCancel()
	m.handle = prepared.Handle
	if err != nil {
		var rejected *control.Rejected
		switch {
		case errors.Is(err, context.DeadlineExceeded) && ctx.Err() == nil:
			r.mu.Lock()
			r.statusNote = "rt_core_request_slow"
			r.mu.Unlock()
			m.preflightRefusal = true
			return rtRefused("joint_move_prepare_timeout")
		case errors.As(err, &rejected) && strings.SplitN(rejected.Reason, ":", 2)[0] == "busy":
			m.preflightRefusal = true
			return rtRefused("joint_move_prepare_busy")
		case errors.As(err, &rejected):
			// The adapter refused the upload itself (native admission, limits,
			// identity): no handle exists and nothing moved, so the typed native
			// reason is a refusal of this step and the machine stays armed.
			log.Printf("dense-execution: prepare refused: %s native=%+v", rejected.Reason, rejected.NativeResult)
			m.preflightRefusal = true
			return err
		}
		return err
	}
	if prepared.Operation != "prepare_trajectory" || m.handle == 0 {
		return control.ErrProtocol
	}
	err = r.observeJointMove(ctx, m, func(s control.Status) (bool, error) {
		state, _ := moveHandle(s, m.handle)
		if state != "" && state != "prepared" {
			return false, rtRefused("joint_move_interrupted")
		}
		return state == "prepared", nil
	})
	if err != nil {
		return err
	}
	cancel()
	ctx, completeCancel := context.WithTimeoutCause(parent, m.completionBudget, rtRefused("joint_move_completion_timeout"))
	defer completeCancel()
	start := &control.Request{Operation: "start_trajectory", Fence: m.fence, Handle: m.handle}
	started, err := rtRequest(r, ctx, "rt_core_request_slow", func(call context.Context) (control.Response, error) { return r.c.Command(call, start) })
	if err != nil {
		return err
	}
	if started.Operation != "start_trajectory" || started.Sequence == 0 {
		return control.ErrProtocol
	}
	m.sequence = started.Sequence
	r.mu.Lock()
	if r.move == m {
		r.moveState.State = "moving"
	}
	r.mu.Unlock()
	var completedAfter uint64
	err = r.observeJointMove(ctx, m, func(s control.Status) (bool, error) {
		state, seq := moveHandle(s, m.handle)
		a := s.Axes[m.intent.Axis]
		if state == "discarded" || state == "invalidated" || state == "" {
			return false, rtRefused("joint_move_interrupted")
		}
		if !finiteJoint(a.LogicalPosition) {
			return false, rtRefused("joint_move_feedback_invalid")
		}
		complete := s.Motion.TrajectoryID == m.handle && s.Motion.CommandSequence == m.sequence && s.Motion.Done && s.Motion.State == 4
		if complete && completedAfter == 0 {
			completedAfter = s.Motion.TimeNS
		}
		endpointsOK := a.TimeNS >= completedAfter && math.Abs(a.LogicalPosition-m.endpoint) <= m.tolerance
		if len(m.endpoints) > 0 {
			endpointsOK = true
			for axis, endpoint := range m.endpoints {
				feedback := s.Axes[axis]
				if !finiteJoint(feedback.LogicalPosition) {
					return false, rtRefused("joint_move_feedback_invalid")
				}
				endpointsOK = endpointsOK && feedback.TimeNS >= completedAfter && math.Abs(feedback.LogicalPosition-endpoint) <= m.tolerances[axis]
			}
		}
		return complete && completedAfter != 0 && endpointsOK &&
			(state == "consumed" || state == "retired") && seq == m.sequence, nil
	})
	if ctx.Err() != nil {
		return context.Cause(ctx)
	}
	return err
}
