package denseexec

import "context"

// The operator bounds on commanded tool speed, for Cartesian jog twists and
// for the selected Jog preset rates a speed-bounded All joints to 0° takes.
// Unverified operator bounds, not a measurement or a machine limit: the
// machine's joint speed, acceleration and jerk limits still apply below them.
// They are one value for every cell; no cell configuration carries a TCP
// speed, so a cell-specific value would first need a field there. The Deck's
// preset editor (steamdeck/real/v5/src/jog_page.cpp) offers the same range.
const (
	cartesianJogMaxLinearMMs   = 1000.0 // mm/s: 1 m/s
	cartesianJogMaxAngularDegS = 180.0  // deg/s: pi rad/s
)

// CartesianJogIntent pins the same hold revision as joint jog. The model and
// measured pose are server inputs, preventing browser pose/target substitution.
// On a path hold the fraction may change between intents of one hold (the
// joystick sets the requested speed); frame and twist may not: a different
// axis, frame or direction is a new hold with a fresh line.
type CartesianJogIntent struct {
	TargetID      string     `json:"target_id"`
	SessionID     string     `json:"session_id"`
	Revision      uint64     `json:"revision"`
	Frame         string     `json:"frame"`
	Twist         [6]float64 `json:"twist"`
	Fraction      float64    `json:"fraction"`
	MaxFraction   float64    `json:"max_fraction,omitempty"`
	InputSequence uint64     `json:"input_sequence,omitempty"`
	// EndCause (halt only): as JointJogIntent.EndCause.
	EndCause string `json:"end_cause,omitempty"`
}

type CartesianResolveRequest struct {
	Target       *[6]float64 `json:"target,omitempty"`
	LinearSpeed  float64     `json:"linear_speed,omitempty"`
	AngularSpeed float64     `json:"angular_speed,omitempty"`
	Operation    string      `json:"operation,omitempty"`
	Delta        [6]float64  `json:"delta,omitempty"`
	InputAgeNS   uint64      `json:"input_age_ns"`
	Model        string      `json:"model"`
	Frame        string      `json:"frame"`
	Twist        [6]float64  `json:"twist"`
	Pose         [6]float64  `json:"pose"`
	Lower        [6]float64  `json:"lower"`
	Upper        [6]float64  `json:"upper"`
	Fraction     float64     `json:"fraction"`
	// path_extend only (pointers keep older operations' requests unchanged):
	// sample i lies at Direction*i*spacing along Axis of the line frozen at
	// FK(Pose); Seed is sample Begin-1 and is required when Begin > 0.
	Axis         *int        `json:"axis,omitempty"`
	Direction    *int        `json:"direction,omitempty"`
	Begin        *uint64     `json:"begin,omitempty"`
	Count        int         `json:"count,omitempty"`
	Seed         *[6]float64 `json:"seed,omitempty"`
	MaxJointStep *[6]float64 `json:"max_joint_step,omitempty"`
}

type CartesianResolution struct {
	Waypoints        [][6]float64 `json:"waypoints"`
	Accepted         bool         `json:"accepted"`
	Reason           string       `json:"reason"`
	Velocities       []float64    `json:"velocities"`
	JointLimitScale  float64      `json:"joint_limit_scale"`
	SingularityScale float64      `json:"singularity_scale"`
	// path_extend: per-sample singularity scale, the gate that ended the path
	// at sample Begin+len(Waypoints) ("" when all Count were produced), and the
	// spacing (m or rad). fk: the row-major 3x4 TCP transform.
	SampleScales []float64 `json:"sample_scales,omitempty"`
	EndReason    string    `json:"end_reason,omitempty"`
	Spacing      float64   `json:"spacing,omitempty"`
	Transform    []float64 `json:"transform,omitempty"`
}

// CartesianMoveIntent names one bounded displacement in the chosen frame.
// Server feedback supplies its origin, preventing browser pose substitution.
type CartesianMoveIntent struct {
	TargetID     string  `json:"target_id"`
	SessionID    string  `json:"session_id"`
	Revision     uint64  `json:"revision"`
	Frame        string  `json:"frame"`
	Axis         int     `json:"axis"`
	DistanceMM   float64 `json:"distance_mm"`
	AngleDegrees float64 `json:"angle_degrees"`
	Fraction     float64 `json:"fraction"`
}

type CartesianMoveBackend interface {
	CartesianMove(context.Context, CartesianMoveIntent) (JointMoveState, error)
}

type CartesianResolver func(context.Context, CartesianResolveRequest) (CartesianResolution, error)

type CartesianJogBackend interface {
	CartesianJog(context.Context, string, CartesianJogIntent) (JointJogState, error)
	JointJogState() JointJogState
}
