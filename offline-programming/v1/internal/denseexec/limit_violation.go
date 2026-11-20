package denseexec

import "encoding/json"

func decodeLimitViolation(data json.RawMessage) *LimitViolation {
	var envelope struct {
		LimitViolation *struct {
			Kind    string   `json:"kind"`
			Segment *int     `json:"segment"`
			Sample  *int     `json:"sample"`
			Axis    string   `json:"axis"`
			Value   *float64 `json:"value"`
			Limit   *float64 `json:"limit"`
			Unit    string   `json:"unit"`
		} `json:"limit_violation"`
	}
	if json.Unmarshal(data, &envelope) != nil || envelope.LimitViolation == nil {
		return nil
	}
	detail := envelope.LimitViolation
	if detail.Axis == "" || detail.Segment == nil || detail.Sample == nil || detail.Value == nil || detail.Limit == nil || *detail.Segment < 0 || *detail.Sample < 0 {
		return nil
	}
	switch {
	case detail.Kind == "position" && detail.Unit == "rad":
	case detail.Kind == "velocity" && detail.Unit == "rad/s":
	case detail.Kind == "acceleration" && detail.Unit == "rad/s^2":
	default:
		return nil
	}
	return &LimitViolation{Kind: detail.Kind, Segment: *detail.Segment, Sample: *detail.Sample,
		Axis: detail.Axis, Value: *detail.Value, Limit: *detail.Limit, Unit: detail.Unit}
}
