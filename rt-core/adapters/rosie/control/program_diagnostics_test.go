//go:build linux

package control

import (
	"encoding/json"
	"fmt"
	"math"
	"net/http"
	"testing"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
)

func TestProgramPositionRefusalCarriesLogicalLimitDetails(t *testing.T) {
	for _, sign := range []int{-1, 1} {
		t.Run(fmt.Sprint(sign), func(t *testing.T) {
			// Keep the real report's sub-millidegree excess. Count-frame sign and
			// Home must not leak into the logical operator diagnostic.
			value, limit := -2.094446759543233, -2.094395102
			frame := densejointtraj.CommandFrame{CountsPerUnit: 100000, Sign: sign, HomeOffset: 23500}
			scale := frame.CountsPerUnit * float64(sign)
			err := segmentPositionError(&densejointtraj.PositionLimitError{
				Position: value*scale - float64(frame.HomeOffset),
				Limit:    limit*scale - float64(frame.HomeOffset),
			}, frame, 0, 2385, "J6")
			outcome := encodeResponse(Response{}, err)
			var response struct {
				Error        string           `json:"error"`
				Data         ProgramLimitData `json:"data"`
				NativeResult any              `json:"native_result"`
			}
			if err := json.Unmarshal(outcome.body, &response); err != nil {
				t.Fatal(err)
			}
			if outcome.status != http.StatusConflict || response.Error != "native_limit_exceeded: segment=0 sample=2385 axis=J6" {
				t.Fatalf("changed stable refusal: %d %s", outcome.status, outcome.body)
			}
			detail := response.Data.LimitViolation
			if detail.Kind != "position" || detail.Unit != "rad" || detail.Axis != "J6" || detail.Segment != 0 || detail.Sample != 2385 ||
				math.Abs(detail.Value-value) > 1e-14 || math.Abs(detail.Limit-limit) > 1e-14 || response.NativeResult != nil {
				t.Fatalf("lost logical position evidence: %s", outcome.body)
			}
		})
	}
}

func TestProgramLimitDetailsPreserveWrappedRefusalAndFiniteJSON(t *testing.T) {
	velocity := &SegmentVelocityError{Segment: 2, Sample: 3, Axis: "J2", Peak: .3, Limit: .2}
	err := fmt.Errorf("preparation refused: %w", velocity)
	outcome := encodeResponse(Response{}, err)
	var response struct {
		Error string            `json:"error"`
		Data  *ProgramLimitData `json:"data"`
	}
	if parseErr := json.Unmarshal(outcome.body, &response); parseErr != nil {
		t.Fatal(parseErr)
	}
	if response.Error != err.Error() || response.Data == nil || response.Data.LimitViolation.Kind != "velocity" ||
		response.Data.LimitViolation.Value != .3 || response.Data.LimitViolation.Unit != "rad/s" {
		t.Fatalf("lost wrapped velocity details: %s", outcome.body)
	}
	for _, bad := range []float64{math.NaN(), math.Inf(1), math.Inf(-1)} {
		velocity.Peak = bad
		outcome := encodeResponse(Response{}, velocity)
		response.Data = nil
		if parseErr := json.Unmarshal(outcome.body, &response); parseErr != nil {
			t.Fatal(parseErr)
		}
		if response.Error != ReasonNativeSegmentRateExceeded || response.Data != nil {
			t.Fatalf("nonfinite detail destroyed refusal: %s", outcome.body)
		}
	}
}
