//go:build linux

package denseexec

import (
	"context"
	"errors"
	"fmt"
	"log"
	"math"
	"sort"
	"sync"
	"time"

	"rosieos/offline-programming/v1/internal/cartesianpath"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Cartesian path hold (design cartesian-jog-closed-loop.md, option D; owner
// decisions 2026-10-06). It replaces the velocity hold, whose joint
// velocities resolved from a 35-100 ms old measured pose left the commanded
// line by 0.9-75 mm on a Rosie 1430 arm. Here OLP streams exact IK joint positions of a
// line frozen at the held command, and rt-core plays them along one path
// parameter with one jerk-limited speed profile, so every joint starts, slows
// and stops together and every stop stays on the line. The measured pose only
// monitors. Cells without the native path_jog capability refuse holds; there
// is no silent fallback to the velocity hold.

// Unverified OLP policy (design 7.1): 100 samples of look-ahead (100 mm at
// 1 mm spacing; stops from ~400 mm/s took 21-44 mm in the harness) streamed
// in chunks of 25.
const (
	pathAheadSamples = 100
	pathChunkSamples = 25
	// Producer and monitor cadence: 10 ms keeps the 100-sample look-ahead
	// full at 400 samples/s with a 25-sample chunk per 62 ms of travel.
	pathPollInterval = 10 * time.Millisecond
	// Measured-versus-commanded warning: 2 mm off the commanded line
	// (owner decision 2026-10-06, unverified). rt-core following error stays
	// the hard fault.
	pathMonitorWarnMM = 2.0
	pathMonitorPeriod = 100 * time.Millisecond
)

// Resolver path-end reasons passed through the native end-of-path code.
var pathEndCodes = map[string]uint32{"ik_no_solution": 1, "joint_limit": 2, "ik_discontinuity": 3, "jacobian_gate": 4, "cartesian_reach": 5}

func pathEndName(code uint32) string {
	for name, value := range pathEndCodes {
		if value == code {
			return name
		}
	}
	return ""
}

// Native governing and stop reasons (rt-core/engine/path_jog.hpp enums).
var pathGoverningNames = []string{"none", "command", "velocity", "curvature", "jerk", "cap", "buffer_end", "path_end", "stopping"}
var pathStopNames = []string{"", "release", "expired", "halt", "path_buffer_underrun", "path_end", "joint_limit", "ik_discontinuity", "path_fault", "inhibited"}

func pathName(names []string, code uint32) string {
	if int(code) < len(names) {
		return names[code]
	}
	return fmt.Sprintf("unknown_%d", code)
}

// pathObservation is the native path lane's state as OLP uses it.
type pathObservation struct {
	Open, Stopping, Ended, Fault     bool
	PathS, PathSpeed                 float64
	Consumed, PlayableEnd, Received  uint64
	Underruns                        uint64
	Governing, StopReason, EndReason uint32
}

// pathJogLane is the SDK path session as the hold uses it; tests substitute it.
type pathJogLane interface {
	Generation() uint64
	Append(context.Context, uint64, [][]float64, []float64, bool, uint32) error
	SetSpeed(float64, uint64) error
	Observe(context.Context) (pathObservation, error)
	End(context.Context) error
}

// pathJointLane lets the path lane occupy the hold slot, so Stop, retirement,
// revision fences and mode conflicts stay one code path with joint jog.
// Velocity input never reaches a path lane.
type pathJointLane struct{ pathJogLane }

func (pathJointLane) UpdateAt([]float64, uint64, uint64) error {
	return rtRefused("jog_mode_conflict")
}

type sdkPathLane struct{ *control.PathJogSession }

func (s sdkPathLane) Observe(ctx context.Context) (pathObservation, error) {
	o, err := s.PathJogSession.Observe(ctx)
	return pathObservation{Open: o.Open, Stopping: o.Stopping, Ended: o.Ended, Fault: o.Fault,
		PathS: o.PathS, PathSpeed: o.PathSpeed, Consumed: o.ConsumedIndex, PlayableEnd: o.PlayableEndIndex,
		Received: o.Received, Underruns: o.Underruns, Governing: o.Governing, StopReason: o.StopReason,
		EndReason: o.EndReason}, err
}

// openPathJog opens the native path session through the SDK (local shared
// region or remote WSS, chosen by the client), with the cell's input lifetime.
func (r *RTCoreController) openPathJog(ctx context.Context, mask uint32, spacing, maxSpeed float64, lifetime time.Duration) (pathJogLane, error) {
	if r.pathOpen != nil {
		return r.pathOpen(ctx, mask, spacing, maxSpeed, lifetime)
	}
	c, ok := r.c.(*control.Client)
	if !ok {
		return nil, rtRefused("cartesian_hold_unavailable")
	}
	s, err := c.NewPathJogSession(ctx, mask, spacing, maxSpeed, uint64(lifetime), ipcclient.HostMonotonicNS)
	if err != nil {
		return nil, err
	}
	return sdkPathLane{s}, nil
}

// cartesianPathHold is one hold's frozen line, its geometry producer and its
// evidence. Its lane lives in the enclosing jointJogHold.
type cartesianPathHold struct {
	line      CartesianPathLine
	axes      [6]int
	axisCount int
	spacing   float64 // m or rad per sample
	box       float64 // path units/s at fraction 1 (the twist component)
	reserve   float64
	velocity  [6]float64
	resolve   CartesianResolver
	lane      pathJogLane
	unit      string // "mm" or "deg" for logs

	// The producer writes these fields; readers on the controller goroutine
	// hold geomMu while reading them.
	last       [6]float64
	sent       uint64
	ended      bool
	endReason  string
	lastScales []float64 // the last extend's per-sample singularity scales

	// Every streamed sample with its resolver singularity scale and its
	// path speed cap, by sample index, for the live readout (path_live.go).
	geomMu  sync.Mutex
	samples [][6]float64
	scales  []float64
	caps    []float64
	// The robot's wrist geometry (robot_geometry.go), fixed for the hold.
	geometry RobotGeometry

	// Evidence, guarded by the controller mutex.
	observation    pathObservation
	maxSpeed       float64
	minAhead       uint64
	resolverMS     []float64
	monitorMaxMM   float64
	monitorMaxDeg  float64
	monitorWarned  bool
	lastSpeed      float64
	lastSpeedLog   time.Time
	governing      map[string]time.Duration
	lastGoverning  string
	governingSince time.Time
	// stallSince: since when native has held the path at zero speed under a
	// geometric governor (stallBy) while the operator asked for speed; zero
	// otherwise. A release after jogStallReportAfter is reported as a stall.
	stallSince time.Time
	stallBy    string
}

// singleAxis returns the one nonzero twist component: no diagonals (owner
// decision 2026-10-06).
func singleAxis(twist [6]float64) (axis, direction int, box float64, ok bool) {
	axis = -1
	for i, v := range twist {
		if v == 0 {
			continue
		}
		if axis >= 0 {
			return 0, 0, 0, false
		}
		axis, box = i, math.Abs(v)
		direction = 1
		if v < 0 {
			direction = -1
		}
	}
	return axis, direction, box, axis >= 0
}

// extend asks the resolver for count samples after the last one sent.
func (p *cartesianPathHold) extend(ctx context.Context, count int) ([][6]float64, []float64, string, time.Duration, error) {
	var seed *[6]float64
	if p.sent > 0 {
		last := p.last
		seed = &last
	}
	started := time.Now()
	result, err := p.resolve(ctx, p.line.ExtendRequest(p.sent, count, seed))
	elapsed := time.Since(started)
	if err != nil {
		return nil, nil, "", elapsed, rtRefused("cartesian_resolver_unavailable")
	}
	if !result.Accepted {
		switch result.Reason {
		case "ik_no_solution", "joint_limit", "jacobian_gate", "cartesian_reach", "ik_discontinuity", "cartesian_input_invalid":
			return nil, nil, "", elapsed, rtRefused("%s", result.Reason)
		}
		return nil, nil, "", elapsed, rtRefused("cartesian_resolver_invalid")
	}
	if len(result.Waypoints) > count || len(result.SampleScales) != len(result.Waypoints) ||
		math.Abs(result.Spacing-p.spacing) > 1e-12*p.spacing || (p.sent == 0 && len(result.Waypoints) > 0 && result.Waypoints[0] != p.line.Origin) {
		return nil, nil, "", elapsed, rtRefused("cartesian_resolver_invalid")
	}
	for _, q := range result.Waypoints {
		for _, v := range q {
			if !finiteJoint(v) {
				return nil, nil, "", elapsed, rtRefused("cartesian_resolver_invalid")
			}
		}
	}
	var previous *[6]float64
	if p.sent > 0 {
		previous = &p.last
	}
	caps := cartesianpath.SampleCaps(previous, result.Waypoints, result.SampleScales, p.spacing, p.box, p.velocity, p.reserve)
	p.lastScales = result.SampleScales
	if _, known := pathEndCodes[result.EndReason]; result.EndReason != "" && !known {
		return nil, nil, "", elapsed, rtRefused("cartesian_resolver_invalid")
	}
	return result.Waypoints, caps, result.EndReason, elapsed, nil
}

// send appends samples to the native lane as full Describe axis vectors.
func (p *cartesianPathHold) send(ctx context.Context, samples [][6]float64, caps []float64, end string) error {
	rows := make([][]float64, len(samples))
	for i, q := range samples {
		row := make([]float64, p.axisCount)
		for j, axis := range p.axes {
			row[axis] = q[j]
		}
		rows[i] = row
	}
	if err := p.lane.Append(ctx, p.sent, rows, caps, end != "", pathEndCodes[end]); err != nil {
		return err
	}
	p.geomMu.Lock()
	if len(samples) > 0 {
		p.last = samples[len(samples)-1]
	}
	for i := range samples {
		scale, limit := 1.0, math.Inf(1)
		if i < len(p.lastScales) {
			scale = p.lastScales[i]
		}
		if i < len(caps) {
			limit = caps[i]
		}
		p.samples, p.scales, p.caps = append(p.samples, samples[i]), append(p.scales, scale), append(p.caps, limit)
	}
	p.sent += uint64(len(samples))
	if end != "" {
		p.ended, p.endReason = true, end
	}
	p.geomMu.Unlock()
	return nil
}

// cartesianPathJogLocked runs a path hold's begin or update after the shared
// admission checks in jointJog. It is entered with r.mu held and releases it.
func (r *RTCoreController) cartesianPathJogLocked(ctx context.Context, action string, h *jointJogHold, in JointJogIntent, stamp time.Time, origin uint64) error {
	if action == "begin" {
		h = r.newHoldLocked(ctx, in, stamp, origin)
		r.jog, r.jogReason = h, ""
		r.logJogHoldBeginLocked(h)
		if err := r.beginCartesianPath(ctx, h, stamp, origin); err != nil {
			r.mu.Unlock()
			return r.finishJointJog(h, err)
		}
		// The browser's next input gets a full lifetime from readiness.
		h.last = time.Now()
		h.timer = time.AfterFunc(h.lifetime, func() { r.expireJointJog(h, time.Now()) })
		// A remote lane's stream can end without a native observation; locally
		// the producer's observations already see closure. (The local SDK
		// Rejection polls adapter status and reported a transient generation
		// mismatch at session start as jog_session_stale, 1 run in 6.)
		if stream, ok := h.path.lane.(interface{ Rejection(context.Context) error }); ok && r.cfg.Address != "" {
			go func() { _ = r.finishJointJog(h, stream.Rejection(h.context)) }()
		}
		r.mu.Unlock()
		return nil
	}
	err := r.updateCartesianPath(h, in.cartesian, stamp, origin)
	if err == nil {
		h.timer.Reset(h.lifetime)
	}
	r.mu.Unlock()
	if err != nil {
		return r.finishJointJog(h, err)
	}
	return nil
}

// beginCartesianPath admits a path hold. It runs with r.mu held and returns
// with it held. Begin captures the held command at rest (never the measured
// pose), freezes the line, resolves the first look-ahead, opens the native
// lane and streams it; the first speed command is the press itself.
func (r *RTCoreController) beginCartesianPath(ctx context.Context, h *jointJogHold, stamp time.Time, origin uint64) error {
	in := h.intent.cartesian
	capability := r.description.PathJog
	if capability == nil || capability.MaxSampleStepS <= 0 {
		return rtRefused("cartesian_hold_unavailable")
	}
	axis, direction, box, ok := singleAxis(in.Twist)
	if !ok {
		return rtRefused("cartesian_input_invalid")
	}
	axes, mask, err := r.cartesianAxesLocked()
	if err != nil {
		return err
	}
	// The line starts at the held command at rest. Every axis publication must
	// postdate the motion-done sample (the Step rule, cartesian_move.go): the
	// per-axis targets come from the 10 Hz status file and lagged a just
	// finished stop, which native refused as path_start_mismatch (composed
	// test, 1 run in 15). Wait up to one status age for that, never guess.
	settled := func() bool {
		s := r.observation
		if s == nil || !s.Motion.Done {
			return false
		}
		for _, i := range axes {
			if s.Axes[i].TimeNS < r.restSinceNS {
				return false
			}
		}
		return true
	}
	// A press released from behind a stop (jog_queue.go) follows the live
	// native closure, which the 10 Hz motion publication reports up to 100 ms
	// later: it waits for motion done too, observing for itself, and is
	// dropped if its input expires first.
	q := queuedBeginOf(ctx)
	released := q != nil && r.jogQueued == q
	if r.observation != nil && (r.observation.Motion.Done || released) && !settled() {
		wait := time.Now().Add(r.timingLocked().StatusAge)
		for !settled() && time.Now().Before(wait) && r.jog == h {
			if released && !time.Now().Before(q.stamp.Add(h.lifetime)) {
				return rtRefused("jog_input_deadline_expired")
			}
			r.mu.Unlock()
			time.Sleep(pathPollInterval)
			if released {
				call, cancel := context.WithTimeout(ctx, r.callBudget())
				_ = r.observe(call)
				cancel()
			}
			r.mu.Lock()
		}
		if r.jog != h {
			return rtRefused("jog_session_stale")
		}
	}
	s := r.observation
	if !s.Motion.Done {
		// A previous stop is still settling; a new line starts from rest.
		return rtRefused("jog_mode_conflict")
	}
	if !settled() {
		return rtRefused("cartesian_pose_stale")
	}
	p := &cartesianPathHold{axes: axes, axisCount: len(r.description.Axes), box: box, reserve: jointMoveVelocityReserve,
		resolve: r.cartesianResolve, minAhead: math.MaxUint64, governing: map[string]time.Duration{}, geometry: r.robotGeometryLocked()}
	p.line = CartesianPathLine{Model: r.cfg.CartesianModel, Frame: in.Frame, Axis: axis, Direction: direction}
	p.spacing, p.unit = 0.001, "mm"
	if axis >= 3 {
		p.spacing, p.unit = math.Pi/720, "deg"
	}
	var measured [6]float64
	for j, i := range axes {
		a := r.description.Axes[i]
		if !r.cartesianPoseFreshLocked(i) {
			return rtRefused("cartesian_pose_stale")
		}
		// The native start rule admits a first sample within one count of the
		// held command (cartesian_move.go): the line starts at LogicalTarget.
		p.line.Origin[j] = s.Axes[i].LogicalTarget
		measured[j] = s.Axes[i].LogicalPosition
		p.line.Lower[j], p.line.Upper[j] = a.MinPosition, a.MaxPosition
		p.velocity[j] = a.MaxVelocity
		p.line.MaxJointStep[j] = a.MaxVelocity * capability.MaxSampleStepS
		if !finiteJoint(p.line.Origin[j]) || !finiteJoint(a.MinPosition) || !finiteJoint(a.MaxPosition) ||
			a.MinPosition >= a.MaxPosition || !finiteJoint(a.MaxVelocity) || a.MaxVelocity <= 0 {
			return rtRefused("cartesian_axes_invalid")
		}
	}
	h.path = p
	lifetime := h.lifetime
	call, cancel := context.WithTimeout(ctx, r.timingLocked().CallBudget)
	defer cancel()
	stop := context.AfterFunc(h.context, cancel)
	defer stop()
	r.mu.Unlock()
	err = func() error {
		samples, caps, end, elapsed, err := p.extend(call, pathAheadSamples)
		if err != nil {
			return err
		}
		if len(samples) < 3 {
			// Nothing playable: the line ends at its start.
			if end == "" {
				end = "cartesian_reach"
			}
			return rtRefused("%s", end)
		}
		// Resolver gate (design section 8): the producer must sustain twice the
		// peak consumption, and one chunk must arrive within half the look-ahead.
		rate := float64(len(samples)) / math.Max(elapsed.Seconds(), 1e-9)
		peak := p.box / p.spacing
		aheadTime := pathAheadSamples / peak
		chunkTime := elapsed.Seconds() * pathChunkSamples / float64(len(samples))
		if rate < 2*peak || chunkTime > aheadTime/2 {
			log.Printf("cartesian path hold: resolver too slow rev=%d samples=%d elapsed_ms=%.1f rate=%.0f/s need=%.0f/s",
				in.Revision, len(samples), float64(elapsed.Microseconds())/1e3, rate, 2*peak)
			return rtRefused("cartesian_resolver_too_slow")
		}
		p.resolverMS = append(p.resolverMS, float64(elapsed.Microseconds())/1e3)
		lane, err := r.openPathJog(call, mask, p.spacing, p.box, lifetime)
		if err != nil {
			return err
		}
		p.lane = lane
		if err := p.send(call, samples, caps, end); err != nil {
			return err
		}
		return nil
	}()
	r.mu.Lock()
	if err == nil && (r.jog != h || call.Err() != nil) {
		err = rtRefused("jog_session_stale")
	}
	if err != nil {
		if p.lane != nil {
			h.lane = pathJointLane{p.lane}
		}
		return err
	}
	h.lane = pathJointLane{p.lane}
	fx := func(q [6]float64) string { return formatRad(q[:]) }
	log.Printf("cartesian path hold: begin rev=%d session=%s target=%s frame=%s axis=%s%s box=%.1f %s/s fraction=%.3f start_q_rad=%s start_measured_q_rad=%s ds=%.4g ahead=%d chunk=%d lifetime_ms=%d resolver_ms=%.1f v_joint=%s max_step_rad=%s",
		in.Revision, in.SessionID, in.TargetID, in.Frame, map[int]string{1: "+", -1: "-"}[direction], axisNames[axis],
		p.box*map[bool]float64{true: 1000, false: 180 / math.Pi}[axis < 3], p.unit, in.Fraction, fx(p.line.Origin), fx(measured),
		p.spacing, pathAheadSamples, pathChunkSamples, lifetime.Milliseconds(), p.resolverMS[0], fx(p.velocity), fx(p.line.MaxJointStep))
	go r.producePath(h, p)
	// A press released from behind a stop starts with its latest intent.
	stamp, origin, fraction := r.latestQueuedInputLocked(ctx, stamp, origin, in.Fraction)
	if pressOutlivesNextIntent(stamp, lifetime) {
		// The press is the first speed command; setup (or a wait behind the
		// previous stop) that left it too little lifetime waits for the next
		// intent instead of ageing this one.
		r.sendPathSpeedLocked(h, fraction, origin, stamp)
	}
	return nil
}

var axisNames = []string{"X", "Y", "Z", "Rx", "Ry", "Rz"}

// updateCartesianPath applies one operator intent: the speed command. Frame,
// axis and direction are fixed for the hold; the fraction may change.
func (r *RTCoreController) updateCartesianPath(h *jointJogHold, in *CartesianJogIntent, stamp time.Time, origin uint64) error {
	if in.Frame != h.intent.cartesian.Frame {
		return rtRefused("jog_mode_conflict")
	}
	axis, direction, box, ok := singleAxis(in.Twist)
	p := h.path
	if !ok || axis != p.line.Axis || direction != p.line.Direction || box != p.box {
		return rtRefused("jog_mode_conflict")
	}
	return r.sendPathSpeedLocked(h, in.Fraction, origin, stamp)
}

// sendPathSpeedLocked sends v* = fraction x box with origin at the intent's
// receipt; the SDK sets deadline = origin + lifetime. The press that began
// the hold predates its session, so the SDK raises that origin to the
// session's opening and keeps its deadline. No timer ever sends a speed
// command, so only operator input renews the deadman.
func (r *RTCoreController) sendPathSpeedLocked(h *jointJogHold, fraction float64, origin uint64, stamp time.Time) error {
	p := h.path
	speed := fraction * p.box
	err := p.lane.SetSpeed(speed, origin)
	if errors.Is(err, control.ErrJogWouldBlock) {
		return nil
	}
	if err != nil {
		log.Printf("cartesian path hold: speed refused rev=%d: %v", h.intent.Revision, err)
		return err
	}
	h.sequence++
	h.last = stamp
	h.sentOrigin = origin
	h.intent.Fraction = fraction
	r.lastHeartbeat = stamp
	if p.lastSpeed == 0 || math.Abs(speed-p.lastSpeed) > 0.1*p.lastSpeed && time.Since(p.lastSpeedLog) > 200*time.Millisecond {
		log.Printf("cartesian path hold: speed rev=%d fraction=%.3f speed=%.1f %s/s", h.intent.Revision, fraction, speed*p.scale(), p.unit)
		p.lastSpeedLog = time.Now()
	}
	p.lastSpeed = speed
	return nil
}

func (p *cartesianPathHold) scale() float64 {
	if p.line.Axis < 3 {
		return 1000
	}
	return 180 / math.Pi
}

// producePath keeps pathAheadSamples of geometry beyond the native
// consumption point, folds native observations into the hold's state and
// monitors the measured pose. It never sends a speed command or renews a
// deadline. It exits when the hold is retired (its context is cancelled).
func (r *RTCoreController) producePath(h *jointJogHold, p *cartesianPathHold) {
	ticker := time.NewTicker(pathPollInterval)
	defer ticker.Stop()
	var lastMonitor time.Time
	var origin []float64
	for {
		select {
		case <-h.context.Done():
			return
		case <-ticker.C:
		}
		call, cancel := context.WithTimeout(h.context, time.Second)
		o, err := p.lane.Observe(call)
		cancel()
		if err != nil {
			if h.context.Err() == nil {
				log.Printf("cartesian path hold: observe failed rev=%d: %v", h.intent.Revision, err)
				_ = r.finishJointJog(h, err)
			}
			return
		}
		if cause := r.notePathObservation(h, p, o); cause != nil {
			_ = r.finishJointJog(h, cause)
			return
		}
		if !p.ended && p.sent < o.Consumed+pathAheadSamples+pathChunkSamples {
			// Retirement does not cancel a resolver exchange: a cancelled one
			// kills the resolver process, which then refuses every request for
			// its 1 s restart backoff (cartesian_process.go), and so the next
			// press (composed re-press test, 2026-10-07). It ends within its own
			// 1 s bound; a retired hold drops the result.
			resolve, cancel := context.WithTimeout(context.WithoutCancel(h.context), time.Second)
			samples, caps, end, elapsed, err := p.extend(resolve, pathChunkSamples)
			cancel()
			if h.context.Err() != nil {
				return
			}
			if err == nil {
				call, cancel := context.WithTimeout(h.context, time.Second)
				err = p.send(call, samples, caps, end)
				cancel()
			}
			if err != nil {
				if h.context.Err() == nil {
					log.Printf("cartesian path hold: geometry failed rev=%d at sample %d: %v", h.intent.Revision, p.sent, err)
					_ = r.finishJointJog(h, err)
				}
				return
			}
			r.mu.Lock()
			p.resolverMS = append(p.resolverMS, float64(elapsed.Microseconds())/1e3)
			r.mu.Unlock()
		}
		if time.Since(lastMonitor) >= pathMonitorPeriod {
			lastMonitor = time.Now()
			if origin == nil {
				origin = r.pathFK(h.context, p, p.line.Origin)
			}
			r.monitorPath(h, p, origin)
		}
	}
}

// notePathObservation records native state and returns a retirement cause
// when the native session ended on its own (expiry, path end, fault).
func (r *RTCoreController) notePathObservation(h *jointJogHold, p *cartesianPathHold, o pathObservation) error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.jog != h {
		return nil
	}
	p.observation = o
	p.maxSpeed = math.Max(p.maxSpeed, o.PathSpeed)
	if o.Received > o.Consumed {
		p.minAhead = min(p.minAhead, o.Received-o.Consumed)
	}
	name := pathName(pathGoverningNames, o.Governing)
	now := time.Now()
	if p.lastGoverning != "" {
		p.governing[p.lastGoverning] += now.Sub(p.governingSince)
	}
	p.lastGoverning, p.governingSince = name, now
	if _, geometric := governorWords[name]; geometric && o.Open && !o.Stopping && o.PathSpeed == 0 && p.lastSpeed > 0 {
		if p.stallSince.IsZero() || p.stallBy != name {
			p.stallSince, p.stallBy = now, name
		}
	} else {
		p.stallSince, p.stallBy = time.Time{}, ""
	}
	switch {
	case o.Fault:
		return rtRefused("path_fault")
	case !o.Open:
		// The native lane closed by itself: expiry or a stop it finished.
		reason := pathName(pathStopNames, o.StopReason)
		if reason == "expired" {
			return rtRefused("jog_input_deadline_expired")
		}
		if reason == "inhibited" {
			// Authority loss, inhibit or Home closed the native session.
			return rtRefused("jog_not_ready")
		}
		if reason == "" || reason == "release" || reason == "halt" {
			return rtRefused("jog_closed")
		}
		return rtRefused("%s", reason)
	case o.Ended && o.PathSpeed == 0 && float64(o.PlayableEnd)-o.PathS/p.spacing < 1:
		// Stopped at the end of the line: retire with the resolver's reason.
		if end := pathEndName(o.EndReason); end != "" {
			return rtRefused("%s", end)
		}
		return rtRefused("%s", pathName(pathStopNames, o.StopReason))
	case !o.Ended && o.PathSpeed == 0 && o.Underruns > 0 && p.lastSpeed > 0 && name == "buffer_end":
		return rtRefused("path_buffer_underrun")
	}
	return nil
}

// pathFK is evidence only. Like extend, its exchange outlives the hold: a
// cancelled exchange restarts the resolver (producePath).
func (r *RTCoreController) pathFK(ctx context.Context, p *cartesianPathHold, q [6]float64) []float64 {
	call, cancel := context.WithTimeout(context.WithoutCancel(ctx), 200*time.Millisecond)
	defer cancel()
	result, err := p.resolve(call, FKRequest(p.line.Model, q))
	if err != nil || !result.Accepted || len(result.Transform) != 12 {
		return nil
	}
	return result.Transform
}

// monitorPath compares the measured pose (10 Hz status, FK) with the frozen
// line. It warns; it never steers. rt-core following error is the hard fault.
func (r *RTCoreController) monitorPath(h *jointJogHold, p *cartesianPathHold, origin []float64) {
	if origin == nil {
		return
	}
	r.mu.Lock()
	var q [6]float64
	fresh := r.jog == h && r.observation != nil
	if fresh {
		for j, i := range p.axes {
			fresh = fresh && r.cartesianPoseFreshLocked(i)
			q[j] = r.observation.Axes[i].LogicalPosition
		}
	}
	r.mu.Unlock()
	if !fresh {
		return
	}
	actual := r.pathFK(h.context, p, q)
	off, ori, _, ok := cartesianpath.LineDeviation(origin, actual, p.line.Frame, p.line.Axis, p.line.Direction)
	if !ok {
		return
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	p.monitorMaxMM, p.monitorMaxDeg = math.Max(p.monitorMaxMM, off), math.Max(p.monitorMaxDeg, ori)
	if off > pathMonitorWarnMM && !p.monitorWarned {
		p.monitorWarned = true
		log.Printf("cartesian path hold: WARNING measured TCP %.2f mm off the commanded line (warn %.1f mm) rev=%d measured_q_rad=%s",
			off, pathMonitorWarnMM, h.intent.Revision, formatRad(q[:]))
	}
}

// logPathHoldEnd writes the hold's end evidence (design 7.10).
func logPathHoldEnd(h *jointJogHold, revision uint64, reason string) {
	p := h.path
	if p == nil {
		return
	}
	if reason == "" {
		reason = "release"
	}
	o := p.observation
	sorted := append([]float64(nil), p.resolverMS...)
	sort.Float64s(sorted)
	pct := func(f float64) float64 {
		if len(sorted) == 0 {
			return 0
		}
		return sorted[min(len(sorted)-1, int(f*float64(len(sorted))))]
	}
	if p.lastGoverning != "" {
		p.governing[p.lastGoverning] += time.Since(p.governingSince)
	}
	governing := ""
	for _, name := range pathGoverningNames {
		if d := p.governing[name]; d > 0 {
			governing += fmt.Sprintf("%s:%d,", name, d.Milliseconds())
		}
	}
	minAhead := p.minAhead
	if minAhead == math.MaxUint64 {
		minAhead = 0
	}
	p.geomMu.Lock()
	sent, endReason := p.sent, p.endReason
	p.geomMu.Unlock()
	log.Printf("cartesian path hold: end rev=%d reason=%s duration_ms=%d travel=%.2f %s max_speed=%.1f %s/s governing_ms={%s} samples_sent=%d samples_consumed=%d min_ahead=%d underruns=%d path_end=%q resolver_p50_ms=%.1f resolver_p99_ms=%.1f monitor_max_offpath_mm=%.3f monitor_max_orient_deg=%.4f native_stop=%s",
		revision, reason, time.Since(h.begun).Milliseconds(), o.PathS*p.scale(), p.unit, p.maxSpeed*p.scale(), p.unit, governing,
		sent, o.Consumed, minAhead, o.Underruns, endReason, pct(.5), pct(.99), p.monitorMaxMM, p.monitorMaxDeg,
		pathName(pathStopNames, o.StopReason))
}
