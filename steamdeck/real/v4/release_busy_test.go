package main

// Check that candidate releases treat a busy notice as a point-in-time gate.

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"strings"
	"testing"
	"time"
)

func TestReleaseBusyPreflightRequiresFreshFreeView(t *testing.T) {
	view := map[string]any{
		"schema": "rosie.bench-busy.v1", "generatedAt": time.Now().UTC().Format(time.RFC3339Nano),
		"authorityCeiling": "coordination_only", "hostWriteAssurance": "cooperative",
		"cells": map[string]any{"bench-relay": map[string]string{
			"state": "free", "reservation": "none", "controller": "idle"}},
	}
	server := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		json.NewEncoder(w).Encode(view)
	}))
	defer server.Close()
	url := server.URL + "/api/busy"
	if err := releaseBusyPreflight(url, "bench-relay"); err != nil {
		t.Fatal(err)
	}
	if err := releaseBusyPreflight(url, "other-relay"); err == nil || !strings.Contains(err.Error(), "not confirmed free") {
		t.Fatalf("missing cell accepted: %v", err)
	}
	view["cells"] = map[string]any{"bench-relay": map[string]string{
		"state": "busy", "reservation": "held", "controller": "idle"}}
	if err := releaseBusyPreflight(url, "bench-relay"); err == nil || !strings.Contains(err.Error(), "not confirmed free") {
		t.Fatalf("held bench accepted: %v", err)
	}
	view["generatedAt"] = time.Now().Add(-time.Minute).UTC().Format(time.RFC3339Nano)
	if err := releaseBusyPreflight(url, "bench-relay"); err == nil || !strings.Contains(err.Error(), "stale") {
		t.Fatalf("stale view accepted: %v", err)
	}
	if err := releaseBusyPreflight("https://example.com/api/busy", "bench-relay"); err == nil || !strings.Contains(err.Error(), "local or on the tailnet") {
		t.Fatalf("non-tailnet source accepted: %v", err)
	}
}
