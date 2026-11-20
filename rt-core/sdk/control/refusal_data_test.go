//go:build linux

package control

import (
	"context"
	"encoding/json"
	"errors"
	"net/http"
	"net/http/httptest"
	"testing"
)

func TestSDKRefusalRetainsDetachedStructuredData(t *testing.T) {
	data := json.RawMessage(`{"limit_violation":{"kind":"position","segment":0,"sample":2385,"axis":"J6","value":-2.094446759543233,"limit":-2.094395102,"unit":"rad"}}`)
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, _ *http.Request) {
		w.WriteHeader(http.StatusConflict)
		_ = json.NewEncoder(w).Encode(Response{Schema: "rosie.rt-control.response.v1", Error: "native_limit_exceeded: segment=0 sample=2385 axis=J6", Data: data})
	}))
	defer server.Close()
	client := newClient(server.URL, &http.Transport{})
	defer client.Close()
	response, err := client.call(context.Background(), client.control, http.MethodGet, "/test-refusal", nil, nil, false)
	var rejected *Rejected
	if !errors.As(err, &rejected) || string(rejected.ResponseData) != string(data) || rejected.Reason != response.Error {
		t.Fatalf("refusal data lost: %+v %v", response, err)
	}
	response.Data[0] = 'x'
	if string(rejected.ResponseData) != string(data) {
		t.Fatal("response mutation changed retained error evidence")
	}
}
