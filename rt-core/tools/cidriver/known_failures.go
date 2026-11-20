package main

import (
	"bytes"
	_ "embed"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"regexp"
)

// known_failures.json is the one list of pre-existing test failures that a
// test gate reports apart instead of failing on: the deploy tool's gate on a
// cell's build host (steamdeck/real/v4, realtime-core build --test) reads it
// from the commit's checkout, and this package validates it. Each entry names
// the gate suite, the exact Go test, why it fails and the evidence. Remove an
// entry when its test is fixed; any failure not listed fails the gate.
//
//go:embed known_failures.json
var knownFailuresJSON []byte

type knownFailure struct {
	Suite    string `json:"suite"`
	Test     string `json:"test"`
	Reason   string `json:"reason"`
	Evidence string `json:"evidence"`
}

var knownFailureTest = regexp.MustCompile(`^Test[A-Za-z0-9_]+(/\S+)?$`)

func loadKnownFailures(data []byte) ([]knownFailure, error) {
	var entries []knownFailure
	decoder := json.NewDecoder(bytes.NewReader(data))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&entries); err != nil {
		return nil, err
	}
	if err := decoder.Decode(new(any)); err != io.EOF {
		return nil, errors.New("known failures: trailing data")
	}
	seen := map[string]bool{}
	for _, k := range entries {
		if k.Suite == "" || k.Reason == "" || k.Evidence == "" || !knownFailureTest.MatchString(k.Test) {
			return nil, fmt.Errorf("known failure %+v needs suite, a Go test name, reason and evidence", k)
		}
		if seen[k.Suite+" "+k.Test] {
			return nil, fmt.Errorf("known failure %s %s listed twice", k.Suite, k.Test)
		}
		seen[k.Suite+" "+k.Test] = true
	}
	return entries, nil
}
