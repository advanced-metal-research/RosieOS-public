package cartesian

import (
	"fmt"
	"math"
)

// JointPathRateBounds bounds TCP translation (metres) and rotation (radians)
// per unit progress along q(s) = from + s*(to-from), 0 <= s <= 1. Multiplying
// these bounds by a shared progress speed therefore bounds both TCP speeds
// without changing the joint-space path. This is geometry only: it neither
// chooses a motion profile nor changes native joint limits.
//
// Rates are evaluated analytically from the serial-chain Jacobian. The gaps
// between samples are covered by global derivative bounds, not an empirical
// safety factor: a vector whose derivative norm is at most B differs from its
// nearest sample by at most B/(2*N). For each link of length l, its position's
// second derivative is bounded by l*(sum of upstream |dq|)^2. Each joint's
// angular-velocity contribution has derivative bounded by
// |dq_i|*(sum of preceding |dq|). These bounds remain valid at singularities.
//
// The six-joint arm is expressed in the same J1 frame and fixed tool transform
// as FK. This does not include moving external axes or a runtime TCP override.
// Malformed geometry, nonfinite values and mismatched joint counts are refused.
func (k *Kinematics) JointPathRateBounds(from, to []float64) (linear, angular float64, err error) {
	bad := func() (float64, float64, error) {
		return 0, 0, fmt.Errorf("joint path rates require finite six-joint arm geometry and endpoints")
	}
	if k == nil || len(k.axis) != 6 || len(k.origP) != 6 || len(k.origR) != 6 ||
		len(from) != 6 || len(to) != 6 || !pathRotationValid(k.toolR) || !pathVectorFinite(k.toolP) {
		return bad()
	}
	var delta [6]float64
	var accumulated, linearDerivative, angularDerivative, linearAbsolute float64
	for i := range delta {
		if !pathFinite(from[i]) || !pathFinite(to[i]) || !pathVectorFinite(k.origP[i]) ||
			!pathVectorFinite(k.axis[i]) || math.Abs(k.axis[i].norm()-1) > 1e-12 || !pathRotationValid(k.origR[i]) {
			return bad()
		}
		delta[i] = to[i] - from[i]
		length := k.origP[i].norm()
		linearAbsolute += length * accumulated
		linearDerivative += length * accumulated * accumulated
		angularDerivative += math.Abs(delta[i]) * accumulated
		accumulated += math.Abs(delta[i])
	}
	linearAbsolute += k.toolP.norm() * accumulated
	linearDerivative += k.toolP.norm() * accumulated * accumulated
	if !pathFinite(accumulated) || !pathFinite(linearAbsolute) || !pathFinite(linearDerivative) || !pathFinite(angularDerivative) {
		return bad()
	}
	if accumulated == 0 {
		return 0, 0, nil
	}
	// Refinement improves tightness, not validity. A bounded maximum retains a
	// conservative result even for cancellation or a very long joint-space arc.
	// Reuse previous samples; only new midpoints are evaluated after doubling.
	const initialIntervals, maxIntervals = 64, 4096
	const relativeSlack = 0.01
	maxLinear, maxAngular := 0.0, 0.0
	for intervals := initialIntervals; ; intervals *= 2 {
		start, stride := 0, 1
		if intervals != initialIntervals {
			start, stride = 1, 2
		}
		for n := start; n <= intervals; n += stride {
			s := float64(n) / float64(intervals)
			var q [6]float64
			for i := range q {
				q[i] = from[i] + delta[i]*s
			}
			v, w := k.jointPathRate(q, delta)
			if !pathFinite(v) || !pathFinite(w) {
				return bad()
			}
			maxLinear, maxAngular = math.Max(maxLinear, v), math.Max(maxAngular, w)
		}
		linearGap := linearDerivative / (2 * float64(intervals))
		angularGap := angularDerivative / (2 * float64(intervals))
		if intervals == maxIntervals || (linearGap <= relativeSlack*maxLinear && angularGap <= relativeSlack*maxAngular) {
			// Cover floating-point matrix/normalization roundoff. Scale the
			// allowance by absolute (uncancelled) rates; stationary paths stay 0.
			linear = maxLinear + linearGap + 1e-10*linearAbsolute
			angular = maxAngular + angularGap + 1e-10*accumulated
			if !pathFinite(linear) || !pathFinite(angular) {
				return bad()
			}
			return linear, angular, nil
		}
	}
}

// jointPathRate uses world-space joint axes and origins. It is the derivative
// of FK, not a finite difference, so rotations crossing +/-pi need no unwrap.
func (k *Kinematics) jointPathRate(q, delta [6]float64) (linear, angular float64) {
	R, P := ident(), vec3{}
	var axes, origins [6]vec3
	for i := range q {
		P = P.add(R.apply(k.origP[i]))
		R = R.mul(k.origR[i])
		origins[i], axes[i] = P, R.apply(k.axis[i])
		R = R.mul(axisRot(k.axis[i], q[i]))
	}
	P = P.add(R.apply(k.toolP))
	v, w := vec3{}, vec3{}
	for i := range q {
		a := axes[i].scale(delta[i])
		r := P.add(origins[i].scale(-1))
		v = v.add(vec3{a[1]*r[2] - a[2]*r[1], a[2]*r[0] - a[0]*r[2], a[0]*r[1] - a[1]*r[0]})
		w = w.add(a)
	}
	return v.norm(), w.norm()
}

func pathFinite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

func pathVectorFinite(v vec3) bool {
	return pathFinite(v[0]) && pathFinite(v[1]) && pathFinite(v[2])
}

func pathRotationValid(r mat3) bool {
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			if !pathFinite(r[i][j]) {
				return false
			}
			dot := r[i][0]*r[j][0] + r[i][1]*r[j][1] + r[i][2]*r[j][2]
			want := 0.0
			if i == j {
				want = 1
			}
			if math.Abs(dot-want) > 1e-12 {
				return false
			}
		}
	}
	det := r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1]) - r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0]) + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0])
	return math.Abs(det-1) <= 1e-12
}
