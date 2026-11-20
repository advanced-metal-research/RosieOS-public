package cartesian

import (
	"math"
	"testing"
)

func planarPathKinematics() *Kinematics {
	k := &Kinematics{toolR: ident(), toolP: vec3{1, 0, 0}}
	for i := 0; i < 6; i++ {
		k.origR = append(k.origR, ident())
		k.origP = append(k.origP, vec3{})
		k.axis = append(k.axis, vec3{0, 0, 1})
	}
	k.origP[1] = vec3{1, 0, 0}
	return k
}

// These rates use independent finite differences of the public FK transform,
// including SO(3) relative rotation, rather than the helper's Jacobian.
func denseFKPathRates(k *Kinematics, from, to []float64, intervals int) (linear, angular float64) {
	previous := k.FK(from)
	var q [6]float64
	for n := 1; n <= intervals; n++ {
		for j := range q {
			q[j] = from[j] + (to[j]-from[j])*float64(n)/float64(intervals)
		}
		pose := k.FK(q[:])
		var distance float64
		var relative [3][3]float64
		for row := 0; row < 3; row++ {
			distance += math.Pow(pose[row*4+3]-previous[row*4+3], 2)
			for col := 0; col < 3; col++ {
				for inner := 0; inner < 3; inner++ {
					relative[row][col] += pose[row*4+inner] * previous[col*4+inner]
				}
			}
		}
		skew := vec3{relative[2][1] - relative[1][2], relative[0][2] - relative[2][0], relative[1][0] - relative[0][1]}
		angle := math.Atan2(skew.norm()/2, (relative[0][0]+relative[1][1]+relative[2][2]-1)/2)
		linear = math.Max(linear, math.Sqrt(distance)*float64(intervals))
		angular = math.Max(angular, angle*float64(intervals))
		previous = pose
	}
	return linear, angular
}

func TestJointPathRateBoundsAnalyticCases(t *testing.T) {
	zero := make([]float64, 6)
	t.Run("stationary", func(t *testing.T) {
		v, w, err := planarPathKinematics().JointPathRateBounds(zero, zero)
		if err != nil || v != 0 || w != 0 {
			t.Fatalf("stationary: %g %g %v", v, w, err)
		}
	})
	t.Run("pure TCP rotation across pi", func(t *testing.T) {
		k := planarPathKinematics()
		k.toolP = vec3{}
		from, to := []float64{0, 0, 0, 0, 0, 3}, []float64{0, 0, 0, 0, 0, 4}
		v, w, err := k.JointPathRateBounds(from, to)
		if err != nil || v != 0 || w < 1 || w > 1+1e-8 {
			t.Fatalf("rotation: %g %g %v", v, w, err)
		}
	})
	t.Run("TCP translation with fixed orientation", func(t *testing.T) {
		k := planarPathKinematics()
		to := []float64{1, -1, 0, 0, 0, 0}
		v, w, err := k.JointPathRateBounds(zero, to)
		if err != nil || v < 1 || v > 1.001 || w > 0.001 {
			t.Fatalf("translation: %g %g %v", v, w, err)
		}
		dv, dw := denseFKPathRates(k, zero, to, 10000)
		if dv > v || dw > w || dw > 1e-9 {
			t.Fatalf("independent translation: %g/%g %g/%g", dv, v, dw, w)
		}
	})
	t.Run("maximum inside path", func(t *testing.T) {
		k := planarPathKinematics()
		from, to := []float64{0, math.Pi, 0, 0, 0, 0}, []float64{math.Pi, 3 * math.Pi, 0, 0, 0, 0}
		v, w, err := k.JointPathRateBounds(from, to)
		// Two unit links: endpoints have speed 2*pi; at s=1/2 the links
		// align and TCP speed reaches 4*pi. Endpoint-only logic misses it.
		if err != nil || v < 4*math.Pi || v > 4*math.Pi*1.011 || w < 3*math.Pi || w > 3*math.Pi*1.011 {
			t.Fatalf("interior maximum: %g %g %v", v, w, err)
		}
		dv, dw := denseFKPathRates(k, from, to, 20000)
		if dv > v || dw > w || dv < 4*math.Pi*.99999 {
			t.Fatalf("independent interior maximum: %g/%g %g/%g", dv, v, dw, w)
		}
	})
}

func TestJointPathRateBoundsInvalid(t *testing.T) {
	for _, name := range []string{"nil", "short from", "long to", "NaN", "infinity", "overflow", "bad axis", "bad offset", "bad rotation", "not six axes"} {
		t.Run(name, func(t *testing.T) {
			k := planarPathKinematics()
			from, to := make([]float64, 6), make([]float64, 6)
			switch name {
			case "nil":
				k = nil
			case "short from":
				from = from[:5]
			case "long to":
				to = append(to, 0)
			case "NaN":
				from[0] = math.NaN()
			case "infinity":
				to[1] = math.Inf(1)
			case "overflow":
				from[0], to[0] = -math.MaxFloat64, math.MaxFloat64
			case "bad axis":
				k.axis[1] = vec3{}
			case "bad offset":
				k.toolP[0] = math.NaN()
			case "bad rotation":
				k.origR[1][0][0] = -1 // reflection, not a rotation
			case "not six axes":
				k.axis = k.axis[:5]
			}
			v, w, err := k.JointPathRateBounds(from, to)
			if err == nil || v != 0 || w != 0 {
				t.Fatalf("invalid input accepted: %g %g %v", v, w, err)
			}
		})
	}
}
