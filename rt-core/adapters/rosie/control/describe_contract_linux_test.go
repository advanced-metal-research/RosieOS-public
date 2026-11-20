package control

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"reflect"
	"testing"
)

// The consumer profile uses implemented operations and one jog lane, preventing
// the legacy oracle operation from being advertised as a consumer alternative.
func TestDescribeConsumerOperationsAreImplemented(t *testing.T) {
	response := httptest.NewRecorder()
	NewHandler(&Controller{}).ServeHTTP(response, httptest.NewRequest(http.MethodGet, "/v1/describe", nil))
	var envelope RawResponse
	if err := json.Unmarshal(response.Body.Bytes(), &envelope); err != nil {
		t.Fatal(err)
	}
	var description Description
	if err := json.Unmarshal(envelope.Data, &description); err != nil {
		t.Fatal(err)
	}
	if response.Code != http.StatusOK || envelope.Error != "" {
		t.Fatalf("describe: HTTP %d %+v", response.Code, envelope)
	}
	states := make(map[string]string)
	for _, capability := range description.Capabilities {
		states[capability.Name] = capability.State
	}
	for _, operation := range []string{
		"describe", "status", "acquire", "renew", "release", "stop", "enable", "arm", "home",
		"restore_anchor", "recovery_status", "prepare_trajectory", "start_trajectory", "discard_trajectory",
		"prepare_program", "start_program", "begin_jog", "update_jog", "end_jog", "jog_clock",
		"jog_status", "jog_ingress", "subscribe_events", "maintenance",
	} {
		if states[operation] != "implemented" {
			t.Errorf("consumer operation %s state=%q", operation, states[operation])
		}
	}
	if states["jog"] != "test_only" || states["reset_fault"] != "interim" {
		t.Fatalf("legacy/recovery states: jog=%q reset_fault=%q", states["jog"], states["reset_fault"])
	}
}

// Prepared metadata must survive GET describe with both identities intact, so
// clients can supply the complete Start fence without reconstructing digests.
func TestDescribeIncludesDetachedProgramIdentity(t *testing.T) {
	c := &Controller{}
	handler := NewHandler(c)
	describe := func() Description {
		t.Helper()
		response := httptest.NewRecorder()
		handler.ServeHTTP(response, httptest.NewRequest(http.MethodGet, "/v1/describe", nil))
		var envelope RawResponse
		if err := json.Unmarshal(response.Body.Bytes(), &envelope); err != nil {
			t.Fatal(err)
		}
		if response.Code != http.StatusOK || envelope.Error != "" || envelope.Operation != "describe" {
			t.Fatalf("describe failed: HTTP %d %+v", response.Code, envelope)
		}
		var description Description
		if err := json.Unmarshal(envelope.Data, &description); err != nil {
			t.Fatal(err)
		}
		return description
	}
	if description := describe(); description.Program != nil {
		t.Fatalf("unprepared program advertised: %+v", description.Program)
	}
	c.program = &Program{
		Identity: Identity{PlanID: "plan", ProgramID: "program", ProgramDigest: "program-digest",
			TrajectoryDigest: "source-digest", SourceDigest: "source-digest", NormalisedDigest: "normalised-digest",
			ManifestRevision: 7, PlanRevision: 11},
		Markers:  []ProcessMarker{{TimeNS: 1000000, Segment: 1, Sample: 0, Action: "torch", Value: false}},
		Segments: 2, Samples: 6, NormalisedSamples: 5, AxisMask: 511,
	}
	prepared := describe().Program
	if prepared == nil || !reflect.DeepEqual(prepared, c.program) {
		t.Fatalf("prepared metadata changed: %+v", prepared)
	}
	if prepared.Identity.SourceDigest != "source-digest" || prepared.Identity.NormalisedDigest != "normalised-digest" || prepared.NormalisedSamples != 5 {
		t.Fatalf("missing execution identity: %+v", prepared)
	}
	prepared.Identity.NormalisedDigest = "caller-mutation"
	prepared.Markers[0].Value = true
	if next := describe().Program; next.Identity.NormalisedDigest != "normalised-digest" || next.Markers[0].Value {
		t.Fatalf("caller changed prepared metadata: %+v", next)
	}
	c.program = nil
	if description := describe(); description.Program != nil {
		t.Fatalf("retired program advertised: %+v", description.Program)
	}
}
