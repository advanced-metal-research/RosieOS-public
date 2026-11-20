package denseexec

import "context"

// JointJogIntent pins target, authority and browser hold revision. End advances
// the revision even before Begin arrives, preventing delayed input resurrection.
type JointJogIntent struct {
	trace     *TraceIntent
	cartesian *CartesianJogIntent
	TargetID  string  `json:"target_id"`
	SessionID string  `json:"session_id"`
	Revision  uint64  `json:"revision"`
	Axis      int     `json:"axis"`
	Direction int     `json:"direction"`
	Fraction  float64 `json:"fraction"`
	// Opt-in proportional input: the ceiling is immutable for this hold.
	// A strictly increasing client sequence prevents delayed speed replay.
	MaxFraction   float64 `json:"max_fraction,omitempty"`
	InputSequence uint64  `json:"input_sequence,omitempty"`
	// EndCause (End only): why the client ended the hold, journalled and
	// classified (jog_end.go). An older client sends none: a release.
	EndCause string `json:"end_cause,omitempty"`
}

type JointJogState struct {
	Active     bool           `json:"active"`
	Revision   uint64         `json:"revision"`
	Generation uint64         `json:"generation"`
	Sequence   uint64         `json:"sequence"`
	Axis       int            `json:"axis"`
	Reason     string         `json:"reason"`
	CadenceMS  int64          `json:"cadence_ms"`
	Move       JointMoveState `json:"move"`
	// FractionUpdates: a Cartesian hold accepts fraction changes within the
	// hold (path holds; the cell advertises path_jog). Proportional joint
	// clients use the separate ProportionalInput opt-in below.
	FractionUpdates bool `json:"fraction_updates"`
	// ProportionalInput supports max_fraction + input_sequence in both modes.
	ProportionalInput bool `json:"proportional_input"`
	// Path hold state: stopping along the path, what limits the path speed
	// (command, velocity, curvature, jerk, cap, buffer_end, path_end,
	// stopping), the native stop reason, and the path speed in mm/s or deg/s.
	Stopping       bool    `json:"stopping,omitempty"`
	GoverningLimit string  `json:"governing_limit,omitempty"`
	StopReason     string  `json:"stop_reason,omitempty"`
	PathSpeed      float64 `json:"path_speed,omitempty"`
	// Queued: a press is waiting for the previous hold's stop to finish
	// (not yet a hold; its intents refresh it, a release cancels it).
	Queued bool `json:"queued,omitempty"`
	// PipelinedIntents: this server orders intents that arrive together, so a
	// client may keep several in flight (an overtaken one is dropped). Clients
	// keep one in flight against a server without it.
	PipelinedIntents bool `json:"pipelined_intents"`
	// LastEnd is the last hold end, as on the status body's last_jog_end.
	LastEnd *JogEnd `json:"last_end,omitempty"`
	// PathLive is the status body's jog_live.
	PathLive *PathLive `json:"path_live,omitempty"`
}

// JointJogBackend is optional: legacy backends cannot route real joint input.
type JointJogBackend interface {
	JointJog(context.Context, string, JointJogIntent) (JointJogState, error)
	JointJogState() JointJogState
}

// JointMoveIntent uses the same browser revision as jog, so delayed GO cannot
// start after Stop, release, or another accepted motion intent.
type JointMoveIntent struct {
	TargetID  string  `json:"target_id"`
	SessionID string  `json:"session_id"`
	Revision  uint64  `json:"revision"`
	Axis      int     `json:"axis"`
	Degrees   float64 `json:"degrees"`
	Fraction  float64 `json:"fraction"`
	// Axes, when present, names every joint of one synchronised move (All
	// joints to 0° sends six); Axis and Degrees are then the first entry.
	// Every axis follows one shared progress along the straight joint
	// segment, so all joints keep their ratios under one prepared trajectory.
	Axes []JointMoveAxis `json:"axes,omitempty"`
	// Set only by RecoverToZero after validating the selected preset.
	returnSpeeds *returnJogSpeeds
}

// JointMoveAxis is one joint of a several-axis move: a signed delta in degrees.
type JointMoveAxis struct {
	Axis    int     `json:"axis"`
	Degrees float64 `json:"degrees"`
	// PositionRad requests an absolute endpoint from fresh native held state.
	// It is exclusive with a nonzero relative Degrees request.
	PositionRad *float64 `json:"position_rad,omitempty"`
}

type JointMoveState struct {
	Axis     int    `json:"axis"`
	State    string `json:"state"`
	Reason   string `json:"reason"`
	Revision uint64 `json:"revision"`
	// Axes lists every joint of a several-axis move (absent for one joint).
	Axes []int `json:"axes,omitempty"`
}

type JointMoveBackend interface {
	JointMove(context.Context, JointMoveIntent) (JointMoveState, error)
}
