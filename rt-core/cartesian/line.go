package cartesian

import "math"

// transformParts splits a row-major 3x4 transform into rotation and position.
func transformParts(t []float64) (r mat3, p vec3, ok bool) {
	if len(t) != 12 {
		return r, p, false
	}
	for row := 0; row < 3; row++ {
		for col := 0; col < 3; col++ {
			r[row][col] = t[row*4+col]
		}
		p[row] = t[row*4+3]
	}
	return r, p, true
}

// relative is R R0^T: the rotation from the origin pose to the pose, in the
// base frame.
func relative(r, r0 mat3) mat3 {
	var rel mat3
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			rel[i][j] = r[i][0]*r0[j][0] + r[i][1]*r0[j][1] + r[i][2]*r0[j][2]
		}
	}
	return rel
}

func rotationAngle(m mat3) float64 {
	c := (m[0][0] + m[1][1] + m[2][2] - 1) / 2
	return math.Acos(math.Max(-1, math.Min(1, c)))
}

// rotationVector is the rotation vector (rad) of a rotation matrix.
func rotationVector(m mat3) vec3 {
	angle := rotationAngle(m)
	w := vec3{m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]}
	if angle < 1e-9 {
		return w.scale(0.5)
	}
	return w.scale(angle / (2 * math.Sin(angle)))
}

// Motion is how far a pose moved from an origin pose, in the base frame:
// TCP displacement (m) and the rotation vector of R R0^T (rad). A hold with no
// logged axis takes its direction from it (the start-to-stop chord).
func Motion(origin, pose []float64) (displacement, rotation [3]float64, ok bool) {
	r0, p0, ok0 := transformParts(origin)
	r, p, ok1 := transformParts(pose)
	if !ok0 || !ok1 {
		return displacement, rotation, false
	}
	for i := range displacement {
		displacement[i] = p[i] - p0[i]
	}
	return displacement, rotationVector(relative(r, r0)), true
}

// DeviationAlong measures a pose against the line through the origin pose
// along the unit base-frame direction dir. Linear: offPath is the TCP distance
// from the line in mm, orientation the angle of R R0^T in degrees, along the
// signed travel along dir (mm). Rotational: offPath is the TCP distance from
// its start in mm, orientation the part of the rotation not about dir in
// degrees, along the signed rotation about dir (degrees). sign is +1 or -1.
// The definitions are the acceptance contract of the Cartesian path hold (TCP
// never leaves its commanded path); every tool reports them from here so a
// recorded and a replayed hold cannot be judged differently.
func DeviationAlong(origin, pose []float64, dir [3]float64, linear bool, sign float64) (offPathMM, orientationDeg, along float64, ok bool) {
	r0, p0, ok0 := transformParts(origin)
	r, p, ok1 := transformParts(pose)
	if !ok0 || !ok1 {
		return 0, 0, 0, false
	}
	e := vec3(dir)
	var d vec3
	for i := range d {
		d[i] = p[i] - p0[i]
	}
	rel := relative(r, r0)
	if linear {
		dot := d.dot(e)
		var perp float64
		for i := range d {
			x := d[i] - dot*e[i]
			perp += x * x
		}
		return math.Sqrt(perp) * 1000, rotationAngle(rel) * 180 / math.Pi, sign * dot * 1000, true
	}
	w := rotationVector(rel)
	about := w.dot(e)
	off := math.Sqrt(math.Max(0, w.dot(w)-about*about))
	return d.norm() * 1000, off * 180 / math.Pi, sign * about * 180 / math.Pi, true
}

// LineDeviation is DeviationAlong for a jog axis: axis 0-2 is a linear X/Y/Z,
// 3-5 a rotation about X/Y/Z, of the base frame or of the tool frame frozen at
// the origin pose; direction is +1 or -1.
func LineDeviation(origin, actual []float64, frame string, axis, direction int) (offPathMM, orientationDeg, along float64, ok bool) {
	r0, _, ok0 := transformParts(origin)
	if !ok0 || axis < 0 || axis > 5 {
		return 0, 0, 0, false
	}
	var e [3]float64
	k := axis % 3
	if frame == "tool" {
		for row := 0; row < 3; row++ {
			e[row] = r0[row][k]
		}
	} else {
		e[k] = 1
	}
	return DeviationAlong(origin, actual, e, axis < 3, float64(direction))
}
