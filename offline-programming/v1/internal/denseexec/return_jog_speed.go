//go:build linux

package denseexec

import (
	"math"

	"rosieos/rt-core/cartesian"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// Uses the fresh recovery observation for feasibility only. The actual move
// recalculates from correlated held command targets under its motion fence.
func returnSpeedPreflight(k *cartesian.Kinematics, d control.Description, allowed uint32, joints []RecoverJoint, fraction float64, speeds returnJogSpeeds, jerk float64) error {
	s := control.Status{Axes: make([]ipcclient.LogicalAxisStatus, len(d.Axes))}
	var moves []JointMoveAxis
	var origins []float64
	for _, joint := range joints {
		if joint.Axis < 0 || joint.Axis >= len(s.Axes) {
			return returnSpeedRefusal("A connected return joint is missing")
		}
		s.Axes[joint.Axis].LogicalTarget, s.Axes[joint.Axis].TimeNS = joint.FromDeg*math.Pi/180, 1
		if joint.Moves {
			zero := 0.
			moves = append(moves, JointMoveAxis{Axis: joint.Axis, PositionRad: &zero})
			origins = append(origins, s.Axes[joint.Axis].LogicalTarget)
		}
	}
	rate, ready, err := returnProgressVelocity(k, d, s, allowed, 1, moves, fraction, speeds)
	if err != nil {
		return err
	}
	if !ready {
		return returnSpeedRefusal("A held arm coordinate is missing; no return move was sent")
	}
	_, _, err = jointMovesClock(d, moves, fraction, origins, jerk, rate)
	if err != nil {
		code := jointJogFailure(err).Code
		if code == "joint_move_too_many_points" {
			return returnSpeedRefusal("This return is too long at the selected Jog speed; choose a faster preset or return in shorter moves")
		}
	}
	return err
}

func returnSpeedRefusal(detail string) error {
	return httpError(409, "recover_jog_speed_unavailable", "%s", detail)
}

func returnJointAxes(k *cartesian.Kinematics, d control.Description, allowed uint32, moves []JointMoveAxis) ([]int, error) {
	if k == nil || len(k.Joints) != 6 {
		return nil, returnSpeedRefusal("The selected robot model cannot bound return-to-zero TCP speed; no return move was sent")
	}
	axes := make([]int, 6)
	byAxis := map[int]bool{}
	for joint, name := range k.Joints {
		found := -1
		for i, axis := range d.Axes {
			if axis.ID != name {
				continue
			}
			if found >= 0 || axis.Index != uint32(i) || axis.PositionUnit != "rad" || allowed&(1<<i) == 0 {
				return nil, returnSpeedRefusal("The robot model and connected joint order do not match; no return move was sent")
			}
			found = i
		}
		if found < 0 {
			return nil, returnSpeedRefusal("The robot model is missing a connected arm joint; no return move was sent")
		}
		axes[joint], byAxis[found] = found, true
	}
	for _, move := range moves {
		if !byAxis[move.Axis] {
			return nil, returnSpeedRefusal("TCP-limited return currently supports the robot arm; return external axes separately")
		}
	}
	return axes, nil
}

// returnProgressVelocity caps the existing joint-space path using the same
// Cartesian tool rates as the selected Jog preset. All six held coordinates
// are needed, including stationary joints. No resolver/network calls or IK
// are involved; the native prepared trajectory still owns motion admission.
func returnProgressVelocity(k *cartesian.Kinematics, d control.Description, s control.Status, allowed uint32, heldAfter uint64, moves []JointMoveAxis, fraction float64, speeds returnJogSpeeds) (float64, bool, error) {
	refuse := func(detail string) (float64, bool, error) {
		return 0, false, returnSpeedRefusal(detail)
	}
	axes, err := returnJointAxes(k, d, allowed, moves)
	if err != nil {
		return 0, false, err
	}
	from, to := make([]float64, 6), make([]float64, 6)
	byAxis := map[int]int{}
	for joint, found := range axes {
		if found >= len(s.Axes) {
			return refuse("A held arm coordinate is missing; no return move was sent")
		}
		if heldAfter == 0 || s.Axes[found].TimeNS < heldAfter {
			return 0, false, nil
		}
		from[joint] = s.Axes[found].LogicalTarget
		to[joint] = from[joint]
		byAxis[found] = joint
	}
	for _, move := range moves {
		joint := byAxis[move.Axis]
		if move.PositionRad != nil {
			to[joint] = *move.PositionRad
		} else {
			to[joint] += move.Degrees * math.Pi / 180
		}
	}
	linear, angular, err := k.JointPathRateBounds(from, to)
	if err != nil {
		return refuse("Could not bound return-to-zero TCP speed: " + err.Error())
	}
	rate := math.Inf(1)
	if linear > 0 {
		rate = math.Min(rate, speeds.linearMMs/1000*fraction/linear)
	}
	if angular > 0 {
		rate = math.Min(rate, speeds.angularDegs*math.Pi/180*fraction/angular)
	}
	return rate, true, nil
}
