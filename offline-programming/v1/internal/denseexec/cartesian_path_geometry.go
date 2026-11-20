package denseexec

// Resolver requests of the Cartesian path hold. No build tag: pure data, also
// usable by tests on any platform. The cap and deviation math lives in
// internal/cartesianpath, shared with the offline harness.

// CartesianPathLine is one hold's frozen line. Origin is the held command at
// the press (never the measured pose); the resolver builds the line at
// FK(Origin) and keeps the tool frame captured there (owner decision 3).
type CartesianPathLine struct {
	Model, Frame    string
	Axis, Direction int // Axis 0-2 translation (m), 3-5 rotation (rad); Direction +-1.
	Origin          [6]float64
	Lower, Upper    [6]float64
	MaxJointStep    [6]float64 // rad; native refuses a larger per-sample step.
}

// ExtendRequest asks for count exact-IK samples starting at begin; seed is
// sample begin-1 and is required for begin > 0.
func (l CartesianPathLine) ExtendRequest(begin uint64, count int, seed *[6]float64) CartesianResolveRequest {
	axis, direction, first := l.Axis, l.Direction, begin
	step := l.MaxJointStep
	req := CartesianResolveRequest{Operation: "path_extend", Model: l.Model, Frame: l.Frame, Pose: l.Origin,
		Lower: l.Lower, Upper: l.Upper, Fraction: 1, InputAgeNS: 1,
		Axis: &axis, Direction: &direction, Begin: &first, Count: count, MaxJointStep: &step}
	if begin > 0 {
		req.Seed = seed
	}
	return req
}

// FKRequest asks the resolver for the TCP transform of pose (monitoring only).
func FKRequest(model string, pose [6]float64) CartesianResolveRequest {
	return CartesianResolveRequest{Operation: "fk", Model: model, Frame: "base", Pose: pose,
		Lower: [6]float64{-10, -10, -10, -10, -10, -10}, Upper: [6]float64{10, 10, 10, 10, 10, 10},
		Fraction: 1, InputAgeNS: 1}
}
