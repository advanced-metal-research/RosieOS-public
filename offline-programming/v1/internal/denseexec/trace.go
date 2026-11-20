//go:build linux

package denseexec

import (
	"context"
	"errors"
	"fmt"
	"log"
	"math"
	"strings"
	"time"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

// Trace is finite geometry in the existing native path-jog lane. It never
// loads or plays a production program and cannot publish process outputs.
func (r *RTCoreController) Trace(ctx context.Context, action string, in TraceIntent) (JointJogState, error) {
	if action == "prepare" {
		if err := r.prepareTrace(ctx, in); err != nil {
			// Preparation shares the jog gates. Preserve their public refusal
			// codes so an idle-time client can retry a transient busy lane.
			return r.JointJogState(), jointJogFailure(err)
		}
		return r.JointJogState(), nil
	}
	if action == "update" {
		r.mu.Lock()
		valid := in.TargetID == r.cfg.Binding.PairID && r.session != nil && in.SessionID == r.session.ID
		r.mu.Unlock()
		state := r.JointJogState()
		if valid && state.LastEnd != nil && state.LastEnd.Revision == in.Revision && state.LastEnd.Cause == "trace_complete" {
			return state, nil
		}
	}
	joint := JointJogIntent{TargetID: in.TargetID, SessionID: in.SessionID, Revision: in.Revision,
		Axis: 0, Direction: 1, Fraction: in.Fraction, MaxFraction: 1, InputSequence: in.InputSequence, EndCause: in.EndCause, trace: &in}
	state, err := r.JointJog(ctx, action, joint)
	// Completion can retire the hold between the check above and dispatch.
	// A late heartbeat acknowledges that exact completed revision; it must
	// not turn successful arrival into a client-side refusal.
	if action == "update" && err != nil && jointJogFailure(err).Code == "jog_session_stale" {
		r.mu.Lock()
		valid := in.TargetID == r.cfg.Binding.PairID && r.session != nil && in.SessionID == r.session.ID
		r.mu.Unlock()
		state = r.JointJogState()
		if valid && state.LastEnd != nil && state.LastEnd.Revision == in.Revision && state.LastEnd.Cause == "trace_complete" {
			return state, nil
		}
	}
	return state, err
}

// The prepared bytes are immutable, bound to the live coordinate system, and
// carry no authority. Preparation cannot arm, move or reserve a native lane.
type preparedTrace struct {
	digest  string
	binding string
	plan    *densejointtraj.Plan
}

func (r *RTCoreController) traceBindingLocked() string {
	if r.observation == nil {
		return ""
	}
	s := r.observation
	return fmt.Sprintf("%s:%d:%d:%s:%s", s.Core.ConfigurationSHA256,
		s.Core.ConfigurationEpoch, s.Core.HomeEpoch, s.AdapterIncarnation, s.DaemonIncarnation)
}
func (r *RTCoreController) readTracePlan(ctx context.Context, in TraceIntent) (*densejointtraj.Plan, error) {
	if in.Plan == nil || in.Digest == "" || in.Plan.TrajectoryDigest != in.Digest || in.Plan.Source != "" {
		return nil, rtRefused("trace_invalid")
	}
	request := *in.Plan
	request.DryRun = true
	fetched, err := r.fetchForLoad(ctx, request)
	if err != nil {
		return nil, err
	}
	if _, err = r.runPreflight(ctx, request, fetched); err != nil {
		return nil, err
	}
	// Dry-run derivation validates the complete original artifact and preserves
	// every position/time byte, removing only process outputs and their hashes.
	_, plan, err := densejointtraj.WithoutProcessOutputs(fetched.blob)
	if err == nil {
		plan.Header.ProcessMarkers = nil
		plan.Header.ProcessMarkersSHA256 = ""
	}
	return plan, err
}
func (r *RTCoreController) prepareTrace(ctx context.Context, in TraceIntent) error {
	if in.TargetID != r.cfg.Binding.PairID {
		return rtRefused("dense_target_mismatch")
	}
	if err := r.observe(ctx); err != nil {
		return err
	}
	r.mu.Lock()
	binding := r.traceBindingLocked()
	busy := r.closed || r.jog != nil || r.move != nil || r.opCancel != nil
	r.mu.Unlock()
	if busy || binding == "" {
		return rtRefused("jog_mode_conflict")
	}
	plan, err := r.readTracePlan(ctx, in)
	if err != nil {
		return err
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed || binding != r.traceBindingLocked() {
		return rtRefused("planning_start_stale")
	}
	if err = validatePlanningCapture(plan, *r.observation, r.description); err != nil {
		return err
	}
	r.tracePrepared = &preparedTrace{digest: in.Digest, binding: binding, plan: plan}
	log.Printf("trace: prepared program %s digest=%s segments=%d (no motion)", plan.Header.ProgramID, in.Digest, len(plan.Segments))
	return nil
}

type traceHold struct {
	lane       pathJogLane
	rows       [][]float64
	spacing    float64
	speedScale float64
	sent       int
}

func (r *RTCoreController) traceJogLocked(ctx context.Context, action string, h *jointJogHold, in JointJogIntent, stamp time.Time, origin uint64) error {
	if action == "begin" {
		h = r.newHoldLocked(ctx, in, stamp, origin)
		r.jog, r.jogReason = h, ""
		r.logJogHoldBeginLocked(h)
		err := r.beginTraceLocked(ctx, h)
		if err != nil {
			r.mu.Unlock()
			if errors.Is(err, errTraceAtEndpoint) {
				_ = r.finishJointJog(h, rtRefused("trace_complete"))
				return nil
			}
			return r.finishJointJog(h, err)
		}
		// Geometry preparation is never permission to move. Require a fresh
		// post-setup intent, as for a remote joint hold.
		h.last = time.Now()
		h.timer = time.AfterFunc(h.lifetime, func() { r.expireJointJog(h, time.Now()) })
		go r.produceTrace(h)
		r.mu.Unlock()
		return nil
	}
	err := h.trace.lane.SetSpeed(in.Fraction*h.trace.speedScale, origin)
	if errors.Is(err, control.ErrJogWouldBlock) {
		r.mu.Unlock()
		return nil
	}
	if err == nil {
		h.sequence++
		h.last = stamp
		h.sentOrigin = origin
		h.intent.Fraction = in.Fraction
		r.lastHeartbeat = stamp
		h.timer.Reset(h.lifetime)
	}
	r.mu.Unlock()
	if err != nil {
		return r.finishJointJog(h, err)
	}
	return nil
}

var errTraceAtEndpoint = errors.New("trace already at endpoint")

// Called and returned with r.mu held. All slow work is cancellable and
// fenced by the original hold, then all live state is rechecked.
func (r *RTCoreController) beginTraceLocked(ctx context.Context, h *jointJogHold) error {
	in := h.intent.trace
	if in.Digest == "" || r.description.PathJog == nil {
		return rtRefused("trace_unavailable")
	}
	call, cancel := context.WithTimeout(ctx, 10*time.Second)
	defer cancel()
	stop := context.AfterFunc(h.context, cancel)
	defer stop()
	var plan *densejointtraj.Plan
	if cached := r.tracePrepared; cached != nil && cached.digest == in.Digest && cached.binding == r.traceBindingLocked() {
		plan = cached.plan
	} else if in.Plan != nil {
		// Compatibility for callers that send a complete plan with Begin.
		r.mu.Unlock()
		var err error
		plan, err = r.readTracePlan(call, *in)
		r.mu.Lock()
		if r.jog != h || call.Err() != nil {
			return rtRefused("jog_session_stale")
		}
		if err != nil {
			return err
		}
	} else {
		return rtRefused("trace_not_prepared: prepare the current program before tracing")
	}
	if in.FirstSegment < 0 || in.LastSegment < in.FirstSegment || in.LastSegment >= len(plan.Segments) {
		return rtRefused("trace_invalid: invalid program segment range")
	}
	if r.observation == nil || !r.held || r.inhibited || r.observation.Core.Armed != 1 || !r.ready(*r.observation) || time.Since(r.nativeSeen) > r.timingLocked().StatusAge {
		return rtRefused("jog_not_ready")
	}
	if !r.observation.Motion.Done {
		return rtRefused("jog_mode_conflict")
	}
	if err := validatePlanningCapture(plan, *r.observation, r.description); err != nil {
		return err
	}
	axes, mask, err := r.cartesianAxesLocked()
	if err != nil {
		return err
	}
	count := len(r.description.Axes)
	if count != 6 {
		return rtRefused("trace_unavailable: trace requires the six-joint arm")
	}
	start, tolerance := make([]float64, count), make([]float64, count)
	for _, i := range axes {
		a := r.description.Axes[i]
		if !r.cartesianPoseFreshLocked(i) || r.observation.Axes[i].TimeNS < r.restSinceNS || !finiteJoint(a.CountsPerUnit) || a.CountsPerUnit <= 0 {
			return rtRefused("cartesian_pose_stale")
		}
		// A disarm/rearm reanchors the held command to measured feedback. Use
		// the machine's existing completion envelope, not a new noise threshold.
		// We still require terminal motion, fresh trusted axes and bounded error.
		if !finiteJoint(a.CompletionTolerance) || a.CompletionTolerance <= 0 {
			return rtRefused("trace_invalid: missing completion tolerance")
		}
		start[i], tolerance[i] = r.observation.Axes[i].LogicalTarget, a.CompletionTolerance
		if math.Abs(r.observation.Axes[i].LogicalPosition-start[i]) > tolerance[i] {
			return rtRefused("trace_settling: wait for the robot to settle")
		}
	}
	var raw [][]float64
	for si := in.FirstSegment; si <= in.LastSegment; si++ {
		segment := plan.Segments[si]
		// These are production process timing, not taught path geometry.
		// Dry-run playback retains them; D-pad Trace must not pause on them.
		if traceWeldDelay(segment) {
			continue
		}
		for i, sample := range segment.Samples {
			if sample.TorchOn {
				return rtRefused("trace_invalid: process outputs must be off")
			}
			if i == 0 && len(raw) != 0 {
				continue
			} // shared waypoint, not a dwell
			raw = append(raw, append([]float64(nil), sample.QRad[:count]...))
		}
	}
	if len(raw) < 2 {
		return rtRefused("trace_invalid: range contains no motion samples")
	}
	rows, err := traceRemainder(raw, start, tolerance, in.Reverse)
	spacing, entryScale := plan.Header.Sampling.DtS, 1.0
	entry := in.EndpointOnly || (err != nil && jointJogFailure(err).Code == "trace_off_path")
	if entry && in.AllowEntry {
		target := raw[len(raw)-1]
		if in.Reverse {
			target = raw[0]
		}
		rows, spacing, entryScale, err = r.traceEntryLocked(call, h, start, target)
	} else if in.EndpointOnly {
		return rtRefused("trace_invalid: endpoint entry was not enabled")
	}
	if err != nil {
		return err
	}
	if !finiteJoint(spacing) || spacing <= 0 || spacing > r.description.PathJog.MaxSampleStepS {
		return rtRefused("trace_invalid: unsupported sample interval")
	}
	for i, row := range rows {
		for j, q := range row {
			a := r.description.Axes[j]
			if !finiteJoint(q) || q < a.MinPosition || q > a.MaxPosition {
				return rtRefused("trace_invalid: joint limit")
			}
			if i > 0 && math.Abs(q-rows[i-1][j]) > a.MaxVelocity*r.description.PathJog.MaxSampleStepS {
				return rtRefused("trace_invalid: sample step")
			}
		}
	}
	rate := in.TimeScale
	if entry {
		rate = entryScale
	} // entry already uses the requested mm/s and rad/s
	if rate == 0 {
		rate = 1
	} // older single-leg clients
	if !finiteJoint(rate) || rate <= 0 {
		return rtRefused("trace_invalid: invalid time scale")
	}
	spacing /= max(1, rate)
	if spacing < 1e-9 {
		return rtRefused("trace_invalid: unsupported time scale")
	}
	// A completed/repeated press or two near-coincident taught points need
	// no native path lane. Check the ENTIRE remainder, not just its endpoint
	// (a closed path may return to its origin), and measured arrival too.
	// Use the machine's existing completion envelope, never a looser one.
	atEndpoint := true
	for _, row := range rows {
		for j, q := range row {
			atEndpoint = atEndpoint && math.Abs(q-start[j]) <= tolerance[j]
		}
	}
	for j, q := range rows[len(rows)-1] {
		atEndpoint = atEndpoint && math.Abs(q-r.observation.Axes[j].LogicalPosition) <= tolerance[j]
	}
	if atEndpoint {
		return errTraceAtEndpoint
	}
	p := &traceHold{rows: rows, spacing: spacing, speedScale: min(1, rate)}
	h.trace = p
	r.mu.Unlock()
	lane, err := r.openPathJog(call, mask, spacing, 1, h.lifetime)
	if err == nil {
		p.lane = lane
		err = p.append(call, pathAheadSamples)
	}
	r.mu.Lock()
	// Publish the lane even after cancellation so finish/cleanup cannot leak it.
	if lane != nil {
		h.lane = pathJointLane{lane}
	}
	if r.jog != h || call.Err() != nil {
		if lane != nil {
			r.mu.Unlock()
			_ = r.endJointJog(pathJointLane{lane})
			r.mu.Lock()
		}
		return rtRefused("jog_session_stale")
	}
	return err
}

// The live command and immutable taught endpoint are the only pose inputs.
// The resolver has no controller access. Recheck the hold, coordinate identity,
// feedback freshness and exact origin after its cancellable calculation.
func (r *RTCoreController) traceEntryLocked(ctx context.Context, h *jointJogHold, start, target []float64) ([][]float64, float64, float64, error) {
	in := h.intent.trace
	bad := func(reason string) ([][]float64, float64, float64, error) { return nil, 0, 0, rtRefused("%s", reason) }
	if !finiteJoint(in.LinearSpeed) || in.LinearSpeed <= 0 || in.LinearSpeed > 1 ||
		!finiteJoint(in.AngularSpeed) || in.AngularSpeed <= 0 || in.AngularSpeed > math.Pi || r.cartesianResolve == nil {
		return bad("trace_invalid: invalid entry speed or resolver")
	}
	req := CartesianResolveRequest{Operation: "trace_entry", Model: r.cfg.CartesianModel, Frame: "base", Fraction: 1, InputAgeNS: 1,
		LinearSpeed: in.LinearSpeed, AngularSpeed: in.AngularSpeed}
	var end [6]float64
	copy(req.Pose[:], start)
	copy(end[:], target)
	req.Target = &end
	for i, a := range r.description.Axes {
		req.Lower[i], req.Upper[i] = a.MinPosition, a.MaxPosition
	}
	binding := r.traceBindingLocked()
	resolve := r.cartesianResolve
	r.mu.Unlock()
	result, err := resolve(ctx, req)
	r.mu.Lock()
	if r.jog != h || ctx.Err() != nil {
		return bad("jog_session_stale")
	}
	if binding != r.traceBindingLocked() || r.observation == nil || !r.held || r.inhibited ||
		r.observation.Core.Armed != 1 || !r.observation.Motion.Done || !r.ready(*r.observation) ||
		time.Since(r.nativeSeen) > r.timingLocked().StatusAge {
		return bad("planning_start_stale")
	}
	for i, q := range start {
		if !r.cartesianPoseFreshLocked(i) || r.observation.Axes[i].LogicalTarget != q ||
			math.Abs(r.observation.Axes[i].LogicalPosition-q) > r.description.Axes[i].CompletionTolerance {
			return bad("planning_start_stale")
		}
	}
	if err != nil {
		return bad("cartesian_resolver_unavailable")
	}
	if !result.Accepted {
		switch result.Reason {
		case "ik_no_solution", "joint_limit", "cartesian_reach", "trace_entry_branch":
			return bad(result.Reason)
		default:
			return bad("cartesian_resolver_invalid")
		}
	}
	if len(result.Waypoints) < 2 || len(result.Waypoints) > 4097 || result.JointLimitScale != 1 ||
		!finiteJoint(result.SingularityScale) || result.SingularityScale <= 0 || result.SingularityScale > 1 {
		return bad("cartesian_resolver_invalid")
	}
	rows := make([][]float64, len(result.Waypoints))
	for k, q := range result.Waypoints {
		rows[k] = append([]float64(nil), q[:]...)
	}
	for i := range start {
		if rows[0][i] != start[i] || math.Abs(rows[len(rows)-1][i]-target[i]) > 1e-10 {
			return bad("cartesian_resolver_invalid")
		}
	}
	log.Printf("trace: entry to taught endpoint reverse=%t samples=%d (process outputs off)", in.Reverse, len(rows))
	return rows, result.Spacing, result.SingularityScale, nil
}

func (p *traceHold) append(ctx context.Context, count int) error {
	end := min(p.sent+count, len(p.rows))
	if end == p.sent {
		return nil
	}
	// Native owns velocity/acceleration/jerk limiting and endpoint braking.
	if err := p.lane.Append(ctx, uint64(p.sent), p.rows[p.sent:end], nil, end == len(p.rows), 0); err != nil {
		return err
	}
	p.sent = end
	return nil
}

func (r *RTCoreController) produceTrace(h *jointJogHold) {
	p := h.trace
	ticker := time.NewTicker(pathPollInterval)
	defer ticker.Stop()
	for {
		select {
		case <-h.context.Done():
			return
		case <-ticker.C:
		}
		call, cancel := context.WithTimeout(h.context, time.Second)
		o, err := p.lane.Observe(call)
		if err == nil && (o.Fault || (o.Stopping && o.StopReason != 5) || (!o.Open && o.StopReason != 5)) {
			err = rtRefused("%s", pathName(pathStopNames, o.StopReason))
			if o.StopReason == 0 {
				err = rtRefused("jog_closed")
			}
		}
		if err == nil && o.Ended && o.PathSpeed == 0 && o.StopReason == 5 {
			// End-of-geometry is a commanded endpoint. Wait for fresh measured
			// feedback inside each joint's published completion tolerance too.
			// Native jerk-limited braking can stop just before the final knot;
			// exact floating-point path-coordinate equality is not completion.
			r.mu.Lock()
			reached := r.jog == h && r.observation != nil && time.Since(r.nativeSeen) < r.timingLocked().StatusAge
			if reached {
				for i, q := range p.rows[len(p.rows)-1] {
					a := r.description.Axes[i]
					reached = reached && r.cartesianPoseFreshLocked(i) && a.CompletionTolerance > 0 &&
						math.Abs(r.observation.Axes[i].LogicalTarget-q) <= a.CompletionTolerance &&
						math.Abs(r.observation.Axes[i].LogicalPosition-q) <= a.CompletionTolerance
				}
			}
			r.mu.Unlock()
			if reached {
				err = rtRefused("trace_complete")
			}
		}
		if err == nil && p.sent < len(p.rows) && uint64(p.sent) < o.Consumed+pathAheadSamples+pathChunkSamples {
			err = p.append(call, pathChunkSamples)
		}
		cancel()
		if err != nil {
			if h.context.Err() == nil {
				_ = r.finishJointJog(h, err)
			}
			return
		}
	}
}

// Project the held command onto this immutable segment, then keep exactly its
// remaining geometry. This permits resume/reverse after an early release and
// refuses a jogged-off-path pose; it never invents a return move. Projection
// tolerance is the existing machine completion envelope. Only the initial
// held point can differ inside that envelope; the remaining geometry is kept.
func traceRemainder(raw [][]float64, start, tolerance []float64, reverse bool) ([][]float64, error) {
	if len(raw) < 2 || len(start) == 0 || len(start) != len(tolerance) {
		return nil, rtRefused("trace_invalid")
	}
	for _, row := range raw {
		if len(row) != len(start) {
			return nil, rtRefused("trace_invalid")
		}
	}
	best, at := math.Inf(1), 0.0
	for i := 0; i < len(raw)-1; i++ {
		numerator, denominator := 0.0, 0.0
		for j, q := range start {
			if !finiteJoint(q) || !finiteJoint(tolerance[j]) || tolerance[j] <= 0 {
				return nil, rtRefused("trace_invalid")
			}
			d := (raw[i+1][j] - raw[i][j]) / tolerance[j]
			numerator += (q - raw[i][j]) / tolerance[j] * d
			denominator += d * d
		}
		u := 0.0
		if denominator > 0 {
			u = max(0, min(1, numerator/denominator))
		}
		// Native uses quintic Hermite, not the straight chord between knots.
		// Refine on that exact interpolant so curved-path stops can resume.
		for iteration := 0; iteration < 12; iteration++ {
			numerator, denominator = 0, 0
			for j, q := range start {
				v, slope := traceCurve(raw, i, j, u)
				d := slope / tolerance[j]
				numerator += (q - v) / tolerance[j] * d
				denominator += d * d
			}
			if denominator < 1e-20 {
				break
			}
			next := max(0, min(1, u+numerator/denominator))
			if math.Abs(next-u) < 1e-12 {
				break
			}
			u = next
		}
		residual := 0.0
		for j, q := range start {
			v, _ := traceCurve(raw, i, j, u)
			residual = max(residual, math.Abs(q-v)/tolerance[j])
		}
		if residual < best {
			best, at = residual, float64(i)+u
		}
	}
	if best > 1 {
		return nil, rtRefused("trace_off_path: jog back onto this program segment before tracing")
	}
	direction, distance := 1.0, float64(len(raw)-1)-at
	if reverse {
		direction, distance = -1, at
	}
	n := max(2, int(math.Ceil(distance))+1)
	// Keep uniform parameter spacing, including the final interval. A shorter
	// final interval changes the native endpoint derivatives and can overshoot.
	rows := make([][]float64, n)
	for k := range rows {
		t := max(0, min(float64(len(raw)-1), at+direction*distance*float64(k)/float64(n-1)))
		i := min(int(t), len(raw)-2)
		u := t - float64(i)
		row := make([]float64, len(start))
		for j := range row {
			row[j], _ = traceCurve(raw, i, j, u)
		}
		rows[k] = row
	}
	rows[0] = append([]float64(nil), start...)
	return rows, nil
}

// Twin of rt-core/engine/path_jog.hpp knot() and Quintic. Values here are
// only for locating a stopped command and resampling its remaining geometry;
// native still owns interpolation validation, limits and actual execution.
func traceCurve(raw [][]float64, i, axis int, u float64) (float64, float64) {
	knot := func(k int) (float64, float64) {
		last := len(raw) - 1
		q := func(n int) float64 { return raw[n][axis] }
		if last == 1 {
			return q(1) - q(0), 0
		}
		if k == 0 {
			return (-3*q(0) + 4*q(1) - q(2)) * .5, q(0) - 2*q(1) + q(2)
		}
		if k == last {
			return (3*q(k) - 4*q(k-1) + q(k-2)) * .5, q(k) - 2*q(k-1) + q(k-2)
		}
		if k == 1 || k+1 == last {
			return (q(k+1) - q(k-1)) * .5, q(k+1) - 2*q(k) + q(k-1)
		}
		return (q(k-2) - q(k+2) + 8*(q(k+1)-q(k-1))) / 12, (-q(k+2) + 16*q(k+1) - 30*q(k) + 16*q(k-1) - q(k-2)) / 12
	}
	m0, k0 := knot(i)
	m1, k1 := knot(i + 1)
	d := raw[i+1][axis] - raw[i][axis]
	c1, c2 := m0, k0*.5
	c3 := 10*d - 6*m0 - 4*m1 - 1.5*k0 + .5*k1
	c4 := -15*d + 8*m0 + 7*m1 + 1.5*k0 - k1
	c5 := 6*d - 3*m0 - 3*m1 - .5*k0 + .5*k1
	return raw[i][axis] + u*(c1+u*(c2+u*(c3+u*(c4+u*c5)))), c1 + u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5)))
}

func traceWeldDelay(segment densejointtraj.Segment) bool {
	return segment.Kind == "dwell" && (strings.HasSuffix(segment.SourceID, ":weld-start-delay") || strings.HasSuffix(segment.SourceID, ":weld-stop-delay"))
}
