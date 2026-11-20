package main

import "testing"

func TestKnownFailuresListIsValid(t *testing.T) {
	entries, err := loadKnownFailures(knownFailuresJSON)
	if err != nil {
		t.Fatal(err)
	}
	for _, k := range entries {
		t.Logf("%s %s: %s", k.Suite, k.Test, k.Reason)
	}
	for _, bad := range []string{
		`[{"suite":"rtctl","test":"TestX","reason":"r"}]`,
		`[{"suite":"rtctl","test":"not a test","reason":"r","evidence":"e"}]`,
		`[{"suite":"rtctl","test":"TestX","reason":"r","evidence":"e","extra":1}]`,
		`[{"suite":"rtctl","test":"TestX","reason":"r","evidence":"e"},{"suite":"rtctl","test":"TestX","reason":"r","evidence":"e"}]`,
	} {
		if _, err := loadKnownFailures([]byte(bad)); err == nil {
			t.Errorf("accepted %s", bad)
		}
	}
}
