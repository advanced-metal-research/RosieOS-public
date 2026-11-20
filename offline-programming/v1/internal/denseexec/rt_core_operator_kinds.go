// Code generated from testdata/operator_feedback_kinds.json; DO NOT EDIT.
//go:build linux

package denseexec

// Unverified display policy: unknown clears on acceptance of any listed kind;
// known refusals clear only on matching acceptance. Unlisted acceptance fails closed.
var operatorKinds = map[string]bool{
	"jog":             false,
	"program_start":   false,
	"prepare_program": false,
	"arm":             false,
	"home":            false,
	"enable":          false,
	"reset":           false,
	"stop":            false,
	"unknown":         true,
}

func operatorAcceptanceClears(refused, accepted string) bool {
	any, known := operatorKinds[refused]
	_, valid := operatorKinds[accepted]
	return known && valid && (any || refused == accepted)
}
