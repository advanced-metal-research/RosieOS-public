package denseexec

import "context"

// Trace uses the same revision, authority and input deadline as joystick jog.
// Prepare carries bytes without motion authority. Begin and subsequent intents
// pin their immutable digest, segment range, time scale and direction; a released or completed revision cannot be restarted.
type TraceIntent struct {
	AllowEntry    bool         `json:"allow_entry,omitempty"`
	EndpointOnly  bool         `json:"endpoint_only,omitempty"`
	LinearSpeed   float64      `json:"linear_speed,omitempty"`
	AngularSpeed  float64      `json:"angular_speed,omitempty"`
	TimeScale     float64      `json:"time_scale,omitempty"`
	TargetID      string       `json:"target_id"`
	SessionID     string       `json:"session_id"`
	Revision      uint64       `json:"revision"`
	Fraction      float64      `json:"fraction"`
	InputSequence uint64       `json:"input_sequence"`
	Digest        string       `json:"trajectory_digest"`
	Reverse       bool         `json:"reverse"`
	FirstSegment  int          `json:"first_segment"`
	LastSegment   int          `json:"last_segment"`
	Plan          *LoadRequest `json:"plan,omitempty"`
	EndCause      string       `json:"end_cause,omitempty"`
}

type TraceBackend interface {
	Trace(context.Context, string, TraceIntent) (JointJogState, error)
	JointJogState() JointJogState
}
