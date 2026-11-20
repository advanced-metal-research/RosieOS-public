//go:build linux

package denseexec

import (
	"errors"
	"math"
	"regexp"
	"strconv"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/sdk/control"
)

var sampleLimitReason = regexp.MustCompile(`^native_limit_exceeded: segment=([0-9]+) sample=([0-9]+) axis=([^\s]+)$`)

// Older remote adapters give the source location but omit numeric evidence.
// AFTER their refusal, recover only an unambiguous endpoint position violation
// from the verified source and the description freshly bound by robotGate.
// This neither runs admission locally nor changes the remote decision.
func enrichLimitFailure(cause error, plan *densejointtraj.Plan, description control.Description) error {
	var rejected *control.Rejected
	if !errors.As(cause, &rejected) || decodeLimitViolation(rejected.ResponseData) != nil || plan == nil {
		return cause
	}
	match := sampleLimitReason.FindStringSubmatch(rejected.Reason)
	if match == nil {
		return cause
	}
	segment, err := strconv.Atoi(match[1])
	if err != nil || segment >= len(plan.Segments) {
		return cause
	}
	sample, err := strconv.Atoi(match[2])
	if err != nil || sample < 1 || sample >= len(plan.Segments[segment].Samples) {
		return cause
	}
	column, found := 0, -1
	for index, axis := range description.Axes {
		if axis.PositionUnit != "rad" {
			continue
		}
		if axis.ID == match[3] {
			if found >= 0 {
				return cause
			}
			found = index
			if column >= densejointtraj.AxisCount {
				return cause
			}
		}
		column++
	}
	if found < 0 {
		return cause
	}
	column = 0
	for index := 0; index < found; index++ {
		if description.Axes[index].PositionUnit == "rad" {
			column++
		}
	}
	axis := description.Axes[found]
	if !finiteLimitNumber(axis.MinPosition) || !finiteLimitNumber(axis.MaxPosition) ||
		axis.MinPosition > axis.MaxPosition || !finiteLimitNumber(axis.MaxVelocity) || axis.MaxVelocity <= 0 {
		return cause
	}
	current := plan.Segments[segment].Samples[sample]
	previous := plan.Segments[segment].Samples[sample-1]
	q, before, velocity := current.QRad[column], previous.QRad[column], current.QdRadS[column]
	if !finiteLimitNumber(q) || !finiteLimitNumber(before) || !finiteLimitNumber(velocity) ||
		before < axis.MinPosition || before > axis.MaxPosition || math.Abs(velocity) > axis.MaxVelocity {
		return cause
	}
	limit := axis.MinPosition
	if q > axis.MaxPosition {
		limit = axis.MaxPosition
	} else if q >= axis.MinPosition {
		return cause
	}
	failure := rtFailure(cause)
	failure.LimitViolation = &LimitViolation{Kind: "position", Segment: segment, Sample: sample,
		Axis: axis.ID, Value: q, Limit: limit, Unit: "rad"}
	return failure
}

func finiteLimitNumber(value float64) bool { return !math.IsNaN(value) && !math.IsInf(value, 0) }
