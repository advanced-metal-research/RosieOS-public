package denseexec

import (
	"context"
	"time"
)

// JogEnd is the last hold end, as the journal line and every client see it.
type JogEnd struct {
	// Seq increases by one per recorded end in this server process; a client
	// shows each end once.
	Seq    uint64 `json:"seq"`
	Code   string `json:"code"`
	Cause  string `json:"cause"`
	Text   string `json:"text"`
	Detail string `json:"detail"`
	Repull string `json:"repull"`
	// Popup: the owning client shows a warning (false for a release, and
	// for a Stop or Disarm the owner pressed itself). Disarmed: the robot
	// is no longer armed after this end.
	Popup    bool `json:"popup"`
	Disarmed bool `json:"disarmed"`
	AtStart  bool `json:"at_start,omitempty"`
	// The hold: joint or cartesian, axis label (J3, X, Rz), frame.
	Kind  string `json:"kind"`
	Axis  string `json:"axis"`
	Frame string `json:"frame,omitempty"`
	// Owner: the client whose press began the hold (its X-Rosie-Client-Id
	// when it sent one). EndedBy: a different client whose Stop, Disarm or
	// release ended it.
	ClientID    string `json:"client_id,omitempty"`
	ClientLabel string `json:"client_label,omitempty"`
	EndedBy     string `json:"ended_by,omitempty"`
	// ClientCause is the End's end_cause: why the client ended the hold
	// (trigger_released, stick_direction_change, focus_lost, ...); absent
	// when the server ended it or an older client sent none.
	ClientCause string `json:"client_cause,omitempty"`
	Revision    uint64 `json:"revision"`
	Generation  uint64 `json:"generation"`
	At          string `json:"at"`
	// AgeMS is filled when a snapshot is served (server clock), so clients
	// need not trust their own clock against the server's.
	AgeMS      int64 `json:"age_ms"`
	DurationMS int64 `json:"duration_ms"`
	// Speeds in SpeedUnit (mm/s, deg/s), distance in DistanceUnit (mm, deg).
	// Path holds: the commanded path speed and the native peak path speed,
	// path s at the end. Joint jogs: the commanded joint speed, the peak
	// measured joint speed and the measured travel (10 Hz status, up to
	// ~100 ms old).
	SpeedUnit      string  `json:"speed_unit"`
	RequestedSpeed float64 `json:"requested_speed"`
	AchievedSpeed  float64 `json:"achieved_speed"`
	DistanceUnit   string  `json:"distance_unit"`
	Distance       float64 `json:"distance"`
	// Path holds only: PathGoverning at the end and samples received but not
	// yet consumed.
	Governing    string  `json:"governing,omitempty"`
	SamplesAhead *uint64 `json:"samples_ahead,omitempty"`
	// Since the last operator intent OLP admitted; the hold's input lifetime;
	// OLP's measured round trip to rt-control.
	LastIntentAgeMS int64   `json:"last_intent_age_ms"`
	LifetimeMS      int64   `json:"lifetime_ms"`
	Updates         uint64  `json:"updates"`
	RTRoundTripMS   float64 `json:"rt_round_trip_ms"`
	at              time.Time
}

// Client names the API client a request came from. The server sets it on
// every API request's context from X-Rosie-Client-Id and what the request
// carries (control.go); the hold keeps its press's client as its owner.
type Client struct {
	ID    string
	Label string
}

type clientKey struct{}

// WithClient attaches the requesting client to ctx.
func WithClient(ctx context.Context, c Client) context.Context {
	return context.WithValue(ctx, clientKey{}, c)
}

// ClientOf returns the requesting client, or the zero Client.
func ClientOf(ctx context.Context) Client {
	if ctx == nil {
		return Client{}
	}
	c, _ := ctx.Value(clientKey{}).(Client)
	return c
}
