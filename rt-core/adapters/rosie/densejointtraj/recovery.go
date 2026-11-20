package densejointtraj

import "math"

// PositionLimitError retains count-frame diagnostics and the recovery refusal
// class, preventing the adapter from reporting an exterior reversal as malformed.
type PositionLimitError struct {
	Recovery        bool
	Position, Limit float64
}

func (e *PositionLimitError) Error() string {
	if e.Recovery {
		return "outside_limits_outward"
	}
	return "native_limit_exceeded"
}

func finite(x float64) bool { return !math.IsNaN(x) && !math.IsInf(x, 0) }

type recoveryLimits struct{ min, max float64 }

func (l recoveryLimits) contains(q float64) bool {
	return q >= l.min && q <= l.max
}

func (l recoveryLimits) step(previous, target float64) bool {
	return target >= math.Min(previous, l.min) && target <= math.Max(previous, l.max)
}

func (l recoveryLimits) velocity(q, v float64) bool {
	return !(q <= l.min && v < 0) && !(q >= l.max && v > 0)
}

// Mirror limit_recovery_feedforward as well as step and velocity admission;
// legal endpoints must not hide a feedforward sign change before re-entry.
func (l recoveryLimits) feedforward(q0, q1, v0, v1 float64) bool {
	if !l.step(q0, q1) || !l.velocity(q0, v0) || !l.velocity(q1, v1) {
		return false
	}
	if l.contains(q0) || !l.contains(q1) || q0 == q1 {
		return true
	}
	boundary := l.max
	if q0 < l.min {
		boundary = l.min
	}
	fraction := (boundary - q0) / (q1 - q0)
	velocity := v0 + float64(fraction*(v1-v0))
	return finite(velocity) && l.velocity(boundary, velocity)
}

// AdmitHermiteKnot checks the initial command before any interval is visited,
// preventing a later axis or sample from hiding an unrepresentable start.
// An exterior stationary or inward knot remains eligible for recovery.
func AdmitHermiteKnot(frame CommandFrame, knot HermiteKnot, minimum, maximum float64) error {
	if !(frame.CountsPerUnit > 0) || !finite(frame.CountsPerUnit) ||
		(frame.Sign != -1 && frame.Sign != 1) || !finite(minimum) || !finite(maximum) || minimum > maximum {
		return errInvalidHermite
	}
	scale, home := frame.CountsPerUnit*float64(frame.Sign), float64(frame.HomeOffset)
	q, v := float64(knot.Position*scale)-home, knot.Velocity*scale
	if !finite(q) || !finite(v) {
		return errInvalidHermite
	}
	// Measured CSP position and velocity fields are signed 32-bit counts.
	if q < math.MinInt32 {
		return &PositionLimitError{Position: q, Limit: math.MinInt32}
	}
	if q > math.MaxInt32 {
		return &PositionLimitError{Position: q, Limit: math.MaxInt32}
	}
	if v < math.MinInt32 || v > math.MaxInt32 {
		return errInvalidHermite
	}
	limits := recoveryLimits{minimum, maximum}
	position, velocity := knot.Position, knot.Velocity
	if !limits.velocity(position, velocity) {
		return &PositionLimitError{Recovery: true, Position: q}
	}
	return nil
}

// AdmitHermiteSegment composes native point admission with continuous position
// bounds. Interior segments keep the projected extrema check; recovery checks
// every monotone piece in one command-circle lift and preserves reason 8.
// Logical limits are scaled with sign and Home exactly as native Commit does.
func AdmitHermiteSegment(frame CommandFrame, first, second HermiteKnot, minimum, maximum float64) (HermiteBounds, error) {
	bounds, err := BoundHermiteSegment(frame, first, second)
	if err != nil || !finite(minimum) || !finite(maximum) || minimum > maximum {
		return HermiteBounds{}, errInvalidHermite
	}
	scale, home := frame.CountsPerUnit*float64(frame.Sign), float64(frame.HomeOffset)
	lower, upper := float64(minimum*scale)-home, float64(maximum*scale)-home
	if !finite(lower) || !finite(upper) {
		return HermiteBounds{}, errInvalidHermite
	}
	limits := recoveryLimits{math.Min(lower, upper), math.Max(lower, upper)}
	q0, q1 := float64(first.Position*scale)-home, float64(second.Position*scale)-home
	v0, v1 := first.Velocity*scale, second.Velocity*scale
	// Measured CSP wire range is signed 32-bit counts. Exterior permission must
	// not bypass the upload's representability fence when logical limits widen.
	for _, q := range [2]float64{q0, q1} {
		if q < math.MinInt32 {
			return bounds, &PositionLimitError{Position: q, Limit: math.MinInt32}
		}
		if q > math.MaxInt32 {
			return bounds, &PositionLimitError{Position: q, Limit: math.MaxInt32}
		}
	}
	if v0 < math.MinInt32 || v0 > math.MaxInt32 || v1 < math.MinInt32 || v1 > math.MaxInt32 {
		return bounds, errInvalidHermite
	}
	// Native upload retains unrounded logical inputs on continuous axes.
	pointLimits := recoveryLimits{minimum, maximum}
	p0, p1, w0, w1 := first.Position, second.Position, first.Velocity, second.Velocity
	if pointLimits.contains(p0) && !pointLimits.contains(p1) {
		limit := limits.max
		if q1 < limits.min {
			limit = limits.min
		}
		return bounds, &PositionLimitError{Position: q1, Limit: limit}
	}
	if !pointLimits.feedforward(p0, p1, w0, w1) {
		return bounds, &PositionLimitError{Recovery: true, Position: q1}
	}
	if limits.contains(q0) && limits.contains(q1) {
		if bounds.MinPosition < limits.min {
			return bounds, &PositionLimitError{Position: bounds.MinPosition, Limit: limits.min}
		}
		if bounds.MaxPosition > limits.max {
			return bounds, &PositionLimitError{Position: bounds.MaxPosition, Limit: limits.max}
		}
		return bounds, nil
	}
	refused := func(q float64) error {
		limit := math.Max(q0, limits.max)
		if q < limits.min {
			limit = math.Min(q0, limits.min)
		}
		return &PositionLimitError{Recovery: true, Position: q, Limit: limit}
	}
	if !limits.feedforward(q0, q1, v0, v1) {
		return bounds, refused(q1)
	}
	delta, dt := q1-q0, float64(second.TimeNS-first.TimeNS)/1e9
	c2 := float64(3*delta) - float64(dt*(float64(2*v0)+v1))
	c3 := float64(-2*delta) + float64(dt*(v0+v1))
	times := hermitePositionTimes(dt, v0, c2, c3)
	if !finite(times[0]) {
		return bounds, errInvalidHermite
	}
	previous := q0
	for _, u := range times {
		q := q0 + float64(u*(float64(dt*v0)+float64(u*(c2+float64(u*c3)))))
		if u == 0 {
			q = q0
		} else if u == 1 {
			q = q0 + delta
		}
		if !finite(q) {
			return bounds, errInvalidHermite
		}
		if !limits.step(previous, q) {
			return bounds, refused(q)
		}
		previous = q
	}
	endpoint := q1
	if !limits.step(previous, endpoint) {
		return bounds, refused(endpoint)
	}
	return bounds, nil
}
