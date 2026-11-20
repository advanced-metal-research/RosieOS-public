//go:build linux

package denseexec

import (
	"context"
	"log"
	"math"
	"time"

	"rosieos/rt-core/sdk/control"
)

// CartesianMove owns accepted steps independently of the HTTP connection.
// Preparation and completion remain bounded; Stop still cancels the controller's
// operation context, preventing a disconnected browser from becoming a deadman.
func (r *RTCoreController) CartesianMove(ctx context.Context, in CartesianMoveIntent) (JointMoveState, error) {
	if (in.Frame != "base" && in.Frame != "tool") || in.Axis < 0 || in.Axis >= 6 ||
		!finiteJoint(in.Fraction) || in.Fraction <= 0 || in.Fraction > 1 ||
		!finiteJoint(in.DistanceMM) || !finiteJoint(in.AngleDegrees) ||
		(in.Axis < 3 && (in.DistanceMM == 0 || in.AngleDegrees != 0)) ||
		(in.Axis >= 3 && (in.AngleDegrees == 0 || in.DistanceMM != 0)) {
		return JointMoveState{Axis: in.Axis, State: "refused", Reason: "cartesian_input_invalid"}, jointJogFailure(rtRefused("cartesian_input_invalid"))
	}
	if ctx.Err() != nil {
		return JointMoveState{Axis: in.Axis, State: "refused", Reason: "client_disconnected"}, jointJogFailure(rtRefused("client_disconnected"))
	}
	return r.boundedMove(context.WithoutCancel(ctx), JointMoveIntent{TargetID: in.TargetID, SessionID: in.SessionID, Revision: in.Revision, Axis: in.Axis, Fraction: in.Fraction}, &in)
}

// Cartesian preparation waits for held, fresh feedback and rechecks the origin
// after computation, preventing a stale IK result from bridging a changed pose.
func (r *RTCoreController) executeCartesianMove(parent context.Context, m *jointMoveRun) error {
	ctx, cancel := context.WithTimeout(parent, rtOperationBudget)
	defer cancel()
	in := m.cartesian
	r.mu.Lock()
	axes, mask, err := r.cartesianAxesLocked()
	d, resolve := r.description, r.cartesianResolve
	r.mu.Unlock()
	if err != nil {
		m.preflightRefusal = true
		return err
	}
	m.axisMask = mask
	req := CartesianResolveRequest{Operation: "move", Model: r.cfg.CartesianModel, Frame: in.Frame, Fraction: in.Fraction}
	if in.Axis < 3 {
		req.Delta[in.Axis] = in.DistanceMM / 1000
	} else {
		req.Delta[in.Axis] = in.AngleDegrees * math.Pi / 180
	}
	var heldAfter uint64
	err = r.observeJointMove(ctx, m, func(s control.Status) (bool, error) {
		if !s.Motion.Done {
			heldAfter = 0
			return false, nil
		}
		if heldAfter == 0 {
			heldAfter = s.Motion.TimeNS
		}
		for j, i := range axes {
			if heldAfter == 0 || s.Axes[i].TimeNS < heldAfter {
				return false, nil
			}
			r.mu.Lock()
			fresh := r.cartesianPoseFreshLocked(i)
			r.mu.Unlock()
			if !fresh {
				return false, rtRefused("cartesian_pose_stale")
			}
			// The native Start rule admits a first target within one count of the
			// held command, not of the measured position: on a 13,107,200 count
			// axis the drive rests a few counts from its target, which is a
			// start_discontinuity fault (seen on J5, 2026-09-16). Start from the
			// held command; the measured position only proves the axis is held.
			req.Pose[j] = s.Axes[i].LogicalTarget
			req.Lower[j], req.Upper[j] = d.Axes[i].MinPosition, d.Axes[i].MaxPosition
		}
		return true, nil
	})
	if err != nil {
		m.preflightRefusal = jointJogFailure(err).Code == "cartesian_pose_stale"
		return err
	}
	// A bounded move has no deadman. The existing preparation budget includes
	// full-segment IK, while hold calls retain their negotiated input lifetime.
	result, err := resolve(ctx, req)
	if err != nil || ctx.Err() != nil {
		log.Printf("cartesian resolver: step resolution failed: err=%v ctx=%v", err, ctx.Err())
		m.preflightRefusal = true
		return rtRefused("cartesian_resolver_unavailable")
	}
	if !result.Accepted {
		m.preflightRefusal = true
		switch result.Reason {
		case "ik_no_solution", "joint_limit", "cartesian_reach", "cartesian_input_invalid":
			return rtRefused("%s", result.Reason)
		default:
			return rtRefused("cartesian_resolver_invalid")
		}
	}
	points, err := cartesianMoveProfile(d, axes, req, result, r.cfg.MoveJerk)
	if err != nil {
		m.preflightRefusal = true
		return err
	}
	err = r.observeJointMove(ctx, m, func(s control.Status) (bool, error) {
		if !s.Motion.Done {
			return false, rtRefused("cartesian_pose_stale")
		}
		for j, i := range axes {
			r.mu.Lock()
			fresh := r.cartesianPoseFreshLocked(i)
			r.mu.Unlock()
			if !fresh {
				return false, rtRefused("cartesian_pose_stale")
			}
			// The origin is the held command: it must not have moved since the
			// resolver read it (a jog, a Home or a Stop in between would change it).
			if !finiteJoint(s.Axes[i].LogicalTarget) || s.Axes[i].LogicalTarget != req.Pose[j] {
				return false, rtRefused("cartesian_pose_stale")
			}
		}
		return true, nil
	})
	if err != nil {
		m.preflightRefusal = true
		return err
	}
	m.endpoints, m.tolerances = map[int]float64{}, map[int]float64{}
	var settling uint64
	for _, i := range axes {
		m.endpoints[i] = points[len(points)-1].Position[i]
		m.tolerances[i] = d.Axes[i].CompletionTolerance
		settling = max(settling, d.Axes[i].CompletionTimeoutNS)
	}
	last, budget := points[len(points)-1].Time, r.callBudget()
	if settling > uint64(math.MaxInt64)-uint64(budget) || uint64(last) > uint64(math.MaxInt64)-settling-uint64(budget) {
		m.preflightRefusal = true
		return rtRefused("joint_move_too_many_points")
	}
	m.completionBudget = last + time.Duration(settling) + budget
	return r.executeMovePoints(parent, m, points)
}

// Sample a quintic progress clock along monotone cubic joint waypoints. Every
// emitted finite difference is checked against described velocity/acceleration;
// stretching the shared clock preserves the path and exact endpoint. With a
// jerk (rad/s^3, one value for every joint; 0 = none) the duration also keeps
// the progress clock's own peak jerk, 60 x slope / T^3, within it on every
// joint. Jerk added where the waypoint spline's curvature changes is not
// bounded: that spline is continuous in slope only.
func cartesianMoveProfile(d control.Description, axes [6]int, req CartesianResolveRequest, result CartesianResolution, jerk float64) ([]control.Point, error) {
	bad := func() ([]control.Point, error) { return nil, rtRefused("cartesian_resolver_invalid") }
	// Unverified resolver bound: at most 1001 poses per bounded segment.
	if result.Reason != "" || len(result.Waypoints) < 2 || len(result.Waypoints) > 1001 ||
		!finiteJoint(result.SingularityScale) || result.SingularityScale <= 0 || result.SingularityScale > 1 ||
		result.JointLimitScale != 1 || d.CycleNS == 0 || d.CycleNS > math.MaxInt64 || d.MaxTrajectoryPoints < 3 ||
		!finiteJoint(jerk) || jerk < 0 {
		return bad()
	}
	var velocities, accelerations [6]float64
	total := 0.0
	for j, i := range axes {
		a := d.Axes[i]
		if !finiteJoint(a.MaxVelocity) || a.MaxVelocity <= 0 || !finiteJoint(a.JogAcceleration) || a.JogAcceleration <= 0 ||
			!finiteJoint(a.CompletionTolerance) || a.CompletionTolerance <= 0 || a.CompletionTimeoutNS == 0 || a.CompletionTimeoutNS > math.MaxInt64 {
			return bad()
		}
		acceleration := a.JogAcceleration
		if a.MaxAcceleration != nil {
			if !finiteJoint(*a.MaxAcceleration) || *a.MaxAcceleration <= 0 {
				return bad()
			}
			acceleration = math.Min(acceleration, *a.MaxAcceleration)
		}
		velocities[j] = a.MaxVelocity * req.Fraction * jointMoveVelocityReserve * result.SingularityScale
		accelerations[j] = acceleration * req.Fraction
		for k, q := range result.Waypoints {
			if !finiteJoint(q[j]) || q[j] < req.Lower[j] || q[j] > req.Upper[j] {
				return nil, rtRefused("joint_limit")
			}
			if k == 0 {
				if q[j] != req.Pose[j] {
					return bad()
				}
			} else {
				slope := math.Abs(q[j]-result.Waypoints[k-1][j]) * float64(len(result.Waypoints)-1)
				// Exact maxima of quintic progress: 1.875 velocity, <6 acceleration, 60 jerk.
				total = math.Max(total, math.Max(1.875*slope/velocities[j], math.Sqrt(6*slope/accelerations[j])))
				if jerk > 0 {
					total = math.Max(total, math.Cbrt(60*slope/jerk))
				}
			}
		}
	}
	if total <= 0 || !finiteJoint(total) {
		return bad()
	}
	cycle := float64(d.CycleNS) / float64(time.Second)
	for {
		steps := math.Ceil(total / cycle)
		if !finiteJoint(steps) || steps+1 >= float64(d.MaxTrajectoryPoints) || steps > float64(math.MaxInt64)/float64(d.CycleNS) {
			return nil, rtRefused("joint_move_too_many_points")
		}
		points := make([]control.Point, int(steps)+1)
		ratio := 1.0
		var previousVelocity [6]float64
		for k := range points {
			t := float64(k) / steps
			s := (10*t*t*t - 15*t*t*t*t + 6*t*t*t*t*t) * float64(len(result.Waypoints)-1)
			s = math.Max(0, math.Min(float64(len(result.Waypoints)-1), s))
			segment := min(int(s), len(result.Waypoints)-2)
			blend := s - float64(segment)
			q := make([]float64, len(d.Axes))
			for j, i := range axes {
				left, right := result.Waypoints[segment][j], result.Waypoints[segment+1][j]
				slope := func(k int) float64 {
					if k == 0 {
						return result.Waypoints[1][j] - result.Waypoints[0][j]
					}
					if k == len(result.Waypoints)-1 {
						return result.Waypoints[k][j] - result.Waypoints[k-1][j]
					}
					a, b := result.Waypoints[k][j]-result.Waypoints[k-1][j], result.Waypoints[k+1][j]-result.Waypoints[k][j]
					if a*b <= 0 {
						return 0
					}
					return 2 * a * b / (a + b)
				}
				b2, b3 := blend*blend, blend*blend*blend
				q[i] = (2*b3-3*b2+1)*left + (b3-2*b2+blend)*slope(segment) + (-2*b3+3*b2)*right + (b3-b2)*slope(segment+1)
				if k == len(points)-1 {
					q[i] = result.Waypoints[len(result.Waypoints)-1][j]
				}
				if !finiteJoint(q[i]) || q[i] < req.Lower[j] || q[i] > req.Upper[j] {
					return nil, rtRefused("joint_limit")
				}
				if k > 0 {
					v := (q[i] - points[k-1].Position[i]) / cycle
					ratio = math.Max(ratio, math.Abs(v)/velocities[j])
					ratio = math.Max(ratio, math.Sqrt(math.Abs(v-previousVelocity[j])/cycle/accelerations[j]))
					previousVelocity[j] = v
				}
			}
			points[k] = control.Point{Time: time.Duration(uint64(k) * d.CycleNS), Position: q}
		}
		for j := range axes {
			ratio = math.Max(ratio, math.Sqrt(math.Abs(previousVelocity[j])/cycle/accelerations[j]))
		}
		if ratio <= 1 {
			return points, nil
		}
		// Unverified numerical reserve: 1.05 avoids rounding at the velocity cap.
		total = steps * cycle * ratio * 1.05
	}
}
