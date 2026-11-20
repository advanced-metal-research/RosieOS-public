//go:build linux

package control

import (
	"errors"
	"math"
)

// ProgramLimitViolation carries logical joint units from adapter admission.
// It supplements the stable refusal label; it is not a native command receipt.
// Segment and Sample are zero-based source indices, matching the error string.
type ProgramLimitViolation struct {
	Kind    string  `json:"kind"`
	Segment int     `json:"segment"`
	Sample  int     `json:"sample"`
	Axis    string  `json:"axis"`
	Value   float64 `json:"value"`
	Limit   float64 `json:"limit"`
	Unit    string  `json:"unit"`
}

// ProgramLimitData is the response data for a logical joint-limit refusal.
type ProgramLimitData struct {
	LimitViolation ProgramLimitViolation `json:"limit_violation"`
}

func programLimitDetails(err error) *ProgramLimitData {
	var position *SegmentPositionError
	var velocity *SegmentVelocityError
	var detail ProgramLimitViolation
	switch {
	case errors.As(err, &position):
		detail = ProgramLimitViolation{Kind: "position", Segment: position.Segment,
			Sample: position.Sample, Axis: position.Axis, Value: position.Position, Limit: position.Limit, Unit: "rad"}
	case errors.As(err, &velocity):
		detail = ProgramLimitViolation{Kind: "velocity", Segment: velocity.Segment,
			Sample: velocity.Sample, Axis: velocity.Axis, Value: velocity.Peak, Limit: velocity.Limit, Unit: "rad/s"}
	default:
		return nil
	}
	// A malformed numeric detail must not make json.Marshal discard the whole
	// refusal. The reason remains available even when no finite value exists.
	if math.IsNaN(detail.Value) || math.IsInf(detail.Value, 0) || math.IsNaN(detail.Limit) || math.IsInf(detail.Limit, 0) {
		return nil
	}
	return &ProgramLimitData{LimitViolation: detail}
}
