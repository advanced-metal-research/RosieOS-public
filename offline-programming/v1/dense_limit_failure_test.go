package main

import (
	"encoding/json"
	"net/http/httptest"
	"testing"

	"rosieos/offline-programming/v1/internal/denseexec"
)

func TestDenseFailureExposesOptionalLimitViolation(t *testing.T) {
	for _, detail := range []*denseexec.LimitViolation{nil, {Kind: "position", Segment: 0, Sample: 2385, Axis: "J6", Value: -2.094446759543233, Limit: -2.094395102, Unit: "rad"}} {
		recorder := httptest.NewRecorder()
		writeDenseFailure(recorder, "load", &denseexec.Error{Status: 409, Code: "native_limit_exceeded", Detail: "sample refused", LimitViolation: detail})
		var payload map[string]json.RawMessage
		if err := json.Unmarshal(recorder.Body.Bytes(), &payload); err != nil {
			t.Fatal(err)
		}
		if recorder.Code != 409 || string(payload["error"]) != `"native_limit_exceeded"` || string(payload["detail"]) != `"sample refused"` {
			t.Fatal(recorder.Body.String())
		}
		if detail == nil {
			if _, present := payload["limit_violation"]; present {
				t.Fatal("old errors acquired fabricated evidence")
			}
		} else {
			var got denseexec.LimitViolation
			if err := json.Unmarshal(payload["limit_violation"], &got); err != nil || got != *detail {
				t.Fatalf("lost evidence: %s %v", recorder.Body.String(), err)
			}
		}
	}
}
