//go:build linux

package denseexec

import (
	"encoding/json"
	"fmt"
	"testing"

	"rosieos/offline-programming/v1/internal/densejointtraj"
	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

func TestRTFailureCarriesLimitEvidenceThroughCleanupWrapping(t *testing.T) {
	rejected := &control.Rejected{Reason: "native_limit_exceeded: segment=0 sample=2385 axis=J6", ResponseData: json.RawMessage(`{"limit_violation":{"kind":"position","segment":0,"sample":2385,"axis":"J6","value":-2.094446759543233,"limit":-2.094395102,"unit":"rad"}}`)}
	failure := rtFailure(fmt.Errorf("prepare: %w", rejected))
	if failure.Code != "native_limit_exceeded" || failure.LimitViolation == nil || failure.LimitViolation.Value != -2.094446759543233 || failure.LimitViolation.Axis != "J6" {
		t.Fatalf("lost adapter evidence: %+v", failure)
	}
	wrapped := rtFailure(fmt.Errorf("stop cleanup: %w", failure))
	if wrapped.Code != failure.Code || wrapped.LimitViolation != failure.LimitViolation {
		t.Fatalf("lost local evidence: %+v", wrapped)
	}
}

func TestRTFailureOlderOrMalformedAdapterHasNoInventedLimitEvidence(t *testing.T) {
	for _, data := range []string{"", `{}`, `{"limit_violation":null}`, `{"limit_violation":{"kind":"position","unit":"rad","axis":"J6"}}`, `{"limit_violation":{"kind":"position","unit":"degrees","axis":"J6"}}`, `{"limit_violation":{"kind":"position","unit":"rad","axis":"J6","value":"bad"}}`} {
		failure := rtFailure(&control.Rejected{Reason: "native_limit_exceeded", ResponseData: json.RawMessage(data)})
		if failure.Code != "native_limit_exceeded" || failure.LimitViolation != nil {
			t.Fatalf("invented evidence for %q: %+v", data, failure)
		}
	}
}

func TestOldAdapterPositionRefusalUsesExactSampleAndNativeRotaryOrder(t *testing.T) {
	plan := &densejointtraj.Plan{Segments: []densejointtraj.Segment{{Samples: make([]densejointtraj.Sample, 2386)}}}
	plan.Segments[0].Samples[2384].QRad[5] = -2.0942
	plan.Segments[0].Samples[2385].QRad[5] = -2.094446759543233
	plan.Segments[0].Samples[2385].QdRadS[5] = -.024289
	description := control.Description{}
	for index := 0; index < 6; index++ {
		description.Axes = append(description.Axes, ipcclient.AxisDescription{ID: fmt.Sprintf("J%d", index+1), PositionUnit: "rad", MinPosition: -2.094395102, MaxPosition: 2.094395102, MaxVelocity: .8})
	}
	// Linear native axes do not consume a column in the dense rotary wire.
	description.Axes = append([]ipcclient.AxisDescription{{ID: "slide", PositionUnit: "m"}}, description.Axes...)
	cause := &control.Rejected{Reason: "native_limit_exceeded: segment=0 sample=2385 axis=J6"}
	failure := rtFailure(enrichLimitFailure(cause, plan, description))
	if failure.Code != "native_limit_exceeded" || failure.LimitViolation == nil || failure.LimitViolation.Value != -2.094446759543233 || failure.LimitViolation.Limit != -2.094395102 || failure.LimitViolation.Sample != 2385 {
		t.Fatalf("lost source/description evidence: %+v", failure)
	}
	for _, change := range []string{"inside", "velocity-first", "predecessor-outside", "bad-index", "wrong-axis", "wrong-reason", "duplicate-axis"} {
		t.Run(change, func(t *testing.T) {
			local := *plan
			local.Segments = []densejointtraj.Segment{{Samples: append([]densejointtraj.Sample(nil), plan.Segments[0].Samples...)}}
			d := description
			d.Axes = append([]ipcclient.AxisDescription(nil), description.Axes...)
			err := *cause
			switch change {
			case "inside":
				local.Segments[0].Samples[2385].QRad[5] = 0
			case "velocity-first":
				local.Segments[0].Samples[2385].QdRadS[5] = 2
			case "predecessor-outside":
				local.Segments[0].Samples[2384].QRad[5] = -3
			case "bad-index":
				err.Reason = "native_limit_exceeded: segment=1 sample=2385 axis=J6"
			case "wrong-axis":
				err.Reason = "native_limit_exceeded: segment=0 sample=2385 axis=J7"
			case "wrong-reason":
				err.Reason = "native_segment_rate_exceeded"
			case "duplicate-axis":
				d.Axes[1].ID = "J6"
			}
			if got := enrichLimitFailure(&err, &local, d); got != &err {
				t.Fatalf("ambiguous violation invented evidence: %v", got)
			}
		})
	}
}
