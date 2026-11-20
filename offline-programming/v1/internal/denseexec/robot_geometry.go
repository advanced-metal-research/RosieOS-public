package denseexec

import (
	"errors"
	"math"
	"regexp"
	"strconv"
	"strings"

	"rosieos/rt-core/cartesian"
)

// RobotGeometry is what the live hold readout derives from the robot's own
// description, so no robot-specific angle is written here: WristSingularRad
// lists the J5 angles at which the J4 and J6 axes are collinear (the wrist
// singularity). A Rosie 1430 arm's URDF gives +-90 deg (J4 about x, J5 about y, J6 about
// -z); a wrist built with J4 and J6 collinear at J5 = 0 gives 0 and 180.
type RobotGeometry struct {
	WristSingularRad []float64
	// The selected description's model-derived chain, for bounded return speed.
	// Nil means no TCP speed claim can be made for a joint-space return.
	Kinematics *cartesian.Kinematics
}

var (
	urdfJointBlock = regexp.MustCompile(`(?s)<joint\s+[^>]*name="(J[456])"[^>]*>(.*?)</joint>`)
	urdfAxis       = regexp.MustCompile(`<axis\s+[^>]*xyz="([^"]+)"`)
	urdfOriginRPY  = regexp.MustCompile(`<origin\s+[^>]*rpy="([^"]+)"`)
)

func urdfTriple(text string) ([3]float64, error) {
	var out [3]float64
	fields := strings.Fields(text)
	if len(fields) != 3 {
		return out, errors.New("expected three numbers")
	}
	for i, f := range fields {
		v, err := strconv.ParseFloat(f, 64)
		if err != nil || math.IsNaN(v) || math.IsInf(v, 0) {
			return out, errors.New("invalid number")
		}
		out[i] = v
	}
	return out, nil
}

func rpyMatrix(rpy [3]float64) [3][3]float64 {
	sr, cr := math.Sincos(rpy[0])
	sp, cp := math.Sincos(rpy[1])
	sy, cy := math.Sincos(rpy[2])
	return [3][3]float64{
		{cy * cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr},
		{sy * cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr},
		{-sp, cp * sr, cp * cr},
	}
}

func axisAngle(axis [3]float64, angle float64) [3][3]float64 {
	n := math.Sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2])
	x, y, z := axis[0]/n, axis[1]/n, axis[2]/n
	s, c := math.Sincos(angle)
	t := 1 - c
	return [3][3]float64{
		{t*x*x + c, t*x*y - s*z, t*x*z + s*y},
		{t*x*y + s*z, t*y*y + c, t*y*z - s*x},
		{t*x*z - s*y, t*y*z + s*x, t*z*z + c},
	}
}

func mulVec(m [3][3]float64, v [3]float64) [3]float64 {
	return [3]float64{m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2], m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2], m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]}
}

// RobotGeometryFromURDF finds the wrist-singular J5 angles of a URDF whose
// arm joints are named J4, J5, J6: J6's axis carried into J4's link (J5's
// origin rotation, J5 at angle q, J6's origin rotation) parallel to J4's
// axis. A URDF without those joints has no wrist measure (nil, no error).
func RobotGeometryFromURDF(urdf string) (RobotGeometry, error) {
	type joint struct {
		axis, rpy [3]float64
	}
	joints := map[string]joint{}
	for _, m := range urdfJointBlock.FindAllStringSubmatch(urdf, -1) {
		// The kinematic joint is the first block with an axis; a Rosie
		// 1430 arm's URDF names J4-J6 again later (its control block) without one.
		a := urdfAxis.FindStringSubmatch(m[2])
		if _, seen := joints[m[1]]; seen || a == nil {
			continue
		}
		var j joint
		v, err := urdfTriple(a[1])
		if err != nil {
			return RobotGeometry{}, err
		}
		j.axis = v
		if o := urdfOriginRPY.FindStringSubmatch(m[2]); o != nil {
			v, err := urdfTriple(o[1])
			if err != nil {
				return RobotGeometry{}, err
			}
			j.rpy = v
		}
		joints[m[1]] = j
	}
	j4, ok4 := joints["J4"]
	j5, ok5 := joints["J5"]
	j6, ok6 := joints["J6"]
	if !ok4 || !ok5 || !ok6 {
		return RobotGeometry{}, nil
	}
	r5, r6 := rpyMatrix(j5.rpy), rpyMatrix(j6.rpy)
	a6 := mulVec(r6, j6.axis)
	n4 := math.Sqrt(j4.axis[0]*j4.axis[0] + j4.axis[1]*j4.axis[1] + j4.axis[2]*j4.axis[2])
	misalign := func(q float64) float64 {
		v := mulVec(r5, mulVec(axisAngle(j5.axis, q), a6))
		c := [3]float64{j4.axis[1]*v[2] - j4.axis[2]*v[1], j4.axis[2]*v[0] - j4.axis[0]*v[2], j4.axis[0]*v[1] - j4.axis[1]*v[0]}
		return math.Sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2]) / n4
	}
	// Scan one turn at 0.1 deg, refine each local minimum that reaches
	// collinearity by golden section; collinear means |a4 x a6| ~ 0.
	const steps = 3600
	var out []float64
	for i := 0; i < steps; i++ {
		q0, q1, q2 := -math.Pi+2*math.Pi*float64(i-1)/steps, -math.Pi+2*math.Pi*float64(i)/steps, -math.Pi+2*math.Pi*float64(i+1)/steps
		if !(misalign(q1) <= misalign(q0) && misalign(q1) < misalign(q2)) {
			continue
		}
		lo, hi := q0, q2
		for k := 0; k < 80; k++ {
			a, b := hi-(hi-lo)/math.Phi, lo+(hi-lo)/math.Phi
			if misalign(a) < misalign(b) {
				hi = b
			} else {
				lo = a
			}
		}
		q := (lo + hi) / 2
		if misalign(q) < 1e-6 {
			out = append(out, math.Remainder(q, 2*math.Pi))
		}
	}
	return RobotGeometry{WristSingularRad: out}, nil
}

// nearestWristSingular returns the singular angle nearest q (wrapped) and
// the signed distance q - angle.
func (g RobotGeometry) nearestWristSingular(q float64) (angle, distance float64, ok bool) {
	best := math.Inf(1)
	for _, s := range g.WristSingularRad {
		d := math.Remainder(q-s, 2*math.Pi)
		if math.Abs(d) < math.Abs(best) {
			best, angle = d, s
		}
	}
	return angle, best, !math.IsInf(best, 0)
}
