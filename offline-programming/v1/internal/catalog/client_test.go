package catalog

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
)

func TestHTTPGatewayUsesFixedRoutesAndForwardsMutationIdentity(t *testing.T) {
	var calls []string
	upstream := httptest.NewServer(http.HandlerFunc(func(writer http.ResponseWriter, request *http.Request) {
		calls = append(calls, request.Method+" "+request.URL.RequestURI())
		writer.Header().Set("Content-Type", "application/json")
		switch {
		case request.Method == http.MethodGet && request.URL.Path == UpstreamPath:
			_, _ = writer.Write([]byte(`{"schema":"robot.v4.program_catalog.v1","owner":"motion-server","catalog_revision":0,"programs":[]}`))
		case request.Method == http.MethodGet && strings.HasSuffix(request.URL.Path, "/revisions"):
			_, _ = writer.Write([]byte(`{"schema":"robot.v4.program-revisions.v1","owner":"motion-server","program_id":"program-1","catalog_revision":1,"revisions":[]}`))
		case request.Method == http.MethodGet:
			_, _ = writer.Write([]byte(`{"schema":"robot.v4.program-entry.v1","owner":"motion-server","catalog_revision":1,"program":{"program_id":"program-1"}}`))
		case request.Method == http.MethodPost:
			for _, header := range []string{"X-Request-ID", "X-Correlation-ID", "Idempotency-Key"} {
				if request.Header.Get(header) == "" {
					t.Errorf("missing %s", header)
				}
			}
			_, _ = writer.Write([]byte(`{"ok":true,"catalog":{"schema":"robot.v4.program_catalog.v1","owner":"motion-server","catalog_revision":1,"programs":[]}}`))
		default:
			t.Fatalf("unexpected request %s %s", request.Method, request.URL)
		}
	}))
	defer upstream.Close()

	gateway, err := NewHTTPGateway(upstream.URL, upstream.Client())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := gateway.List(context.Background()); err != nil {
		t.Fatal(err)
	}
	revision := uint64(2)
	if _, err := gateway.Get(context.Background(), "program-1", &revision); err != nil {
		t.Fatal(err)
	}
	if _, err := gateway.Revisions(context.Background(), "program-1"); err != nil {
		t.Fatal(err)
	}
	if _, err := gateway.Save(
		context.Background(),
		[]byte(`{"program_id":"program-1"}`),
		MutationContext{
			RequestID:      "request-1234",
			CorrelationID:  "correlation-1234",
			IdempotencyKey: "idempotency-1234",
		},
	); err != nil {
		t.Fatal(err)
	}
	want := []string{
		"GET /api/program-catalog",
		"GET /api/program-catalog/program-1?revision=2",
		"GET /api/program-catalog/program-1/revisions",
		"POST /api/program-catalog",
	}
	if strings.Join(calls, "\n") != strings.Join(want, "\n") {
		t.Fatalf("calls=%v want=%v", calls, want)
	}
}

func TestHTTPGatewayRejectsUnsafeOriginsAndInvalidResponses(t *testing.T) {
	for _, origin := range []string{
		"",
		"file:///tmp/catalog",
		"http://user:pass@example.test",
		"http://example.test/private/path",
	} {
		if _, err := NewHTTPGateway(origin, nil); err == nil {
			t.Fatalf("origin %q was accepted", origin)
		}
	}

	upstream := httptest.NewServer(http.HandlerFunc(func(writer http.ResponseWriter, _ *http.Request) {
		_, _ = writer.Write([]byte("not-json"))
	}))
	defer upstream.Close()
	gateway, err := NewHTTPGateway(upstream.URL, upstream.Client())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := gateway.List(context.Background()); !errorsIs(err, ErrInvalidResponse) {
		t.Fatalf("error=%v", err)
	}
}

func TestMutationContextAndProgramIdentityAreBounded(t *testing.T) {
	if ValidProgramID("../escape") || ValidProgramID("program/child") || !ValidProgramID("program-1") {
		t.Fatal("program identity policy is unsafe")
	}
	if err := (MutationContext{}).Validate(); err == nil {
		t.Fatal("empty mutation identity was accepted")
	}
	if err := (MutationContext{
		RequestID:      "request-1234",
		CorrelationID:  "correlation-1234",
		IdempotencyKey: "idempotency-1234",
	}).Validate(); err != nil {
		t.Fatal(err)
	}
}

func errorsIs(err, target error) bool {
	for err != nil {
		if err == target {
			return true
		}
		type unwrapper interface{ Unwrap() error }
		value, ok := err.(unwrapper)
		if !ok {
			return false
		}
		err = value.Unwrap()
	}
	return false
}

func TestGatewayResponsesRemainJSON(t *testing.T) {
	upstream := httptest.NewServer(http.HandlerFunc(func(writer http.ResponseWriter, _ *http.Request) {
		writer.WriteHeader(http.StatusConflict)
		_ = json.NewEncoder(writer).Encode(map[string]any{
			"ok": false, "code": "catalog_conflict", "catalog_revision": 4,
		})
	}))
	defer upstream.Close()
	gateway, err := NewHTTPGateway(upstream.URL, upstream.Client())
	if err != nil {
		t.Fatal(err)
	}
	response, err := gateway.List(context.Background())
	if err != nil || response.Status != http.StatusConflict || !json.Valid(response.Body) {
		t.Fatalf("response=%+v error=%v", response, err)
	}
}
