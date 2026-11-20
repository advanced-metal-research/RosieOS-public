package control

import (
	"encoding/json"
	"fmt"
	"net/http"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
)

// Public refusals preserve the typed native receipt and the actionable reason,
// preventing clients from mistaking exterior admission failure for a fault.
func TestLimitHoldReasonResponse(t *testing.T) {
	for _, err := range []error{
		&ipcclient.RejectedError{Result: ipcclient.CommandResult{Reason: 8, Result: 1}},
		&ipcclient.JogRejectedError{Reason: 15, Observation: ipcclient.JogObservation{UpdateReason: 15}},
	} {
		outcome := encodeResponse(Response{}, err)
		var response Response
		if parseErr := json.Unmarshal(outcome.body, &response); parseErr != nil {
			t.Fatal(parseErr)
		}
		if outcome.status != http.StatusConflict || response.Error != ReasonOutsideLimitsOutward {
			t.Fatal(response, outcome.status)
		}
		if response.NativeResult == nil && response.NativeJogResult == nil {
			t.Fatal("native receipt lost")
		}
	}
	native := &ipcclient.RejectedError{Result: ipcclient.CommandResult{Reason: 8, Result: 1}}
	outcome := encodeResponse(Response{}, fmt.Errorf("cleanup failed: %w", native))
	var response Response
	if err := json.Unmarshal(outcome.body, &response); err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(response.Error, "cleanup failed") || response.NativeResult == nil || response.NativeResult.Reason != 8 {
		t.Fatal("contextual failure or typed receipt lost", response)
	}
}

// Adapter admission uses the same public reason before a native upload exists;
// it must not fabricate a native receipt for a locally rejected recovery curve.
func TestHermiteRecoveryReasonResponse(t *testing.T) {
	err := &SegmentRecoveryError{SegmentPositionError: SegmentPositionError{
		Segment: 0, Sample: 1, Axis: "J1", Position: -1, Limit: 0}}
	outcome := encodeResponse(Response{}, err)
	var response Response
	if parseErr := json.Unmarshal(outcome.body, &response); parseErr != nil {
		t.Fatal(parseErr)
	}
	if outcome.status != http.StatusConflict || response.Error != ReasonOutsideLimitsOutward || response.NativeResult != nil {
		t.Fatal(response, outcome.status)
	}
}
