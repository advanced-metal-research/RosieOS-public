package raceprobe

import "testing"

// TestKnownRace uses unordered writes followed by completion signals; a detector-free pass must fail the CI gate.
func TestKnownRace(t *testing.T) {
	var shared int
	finished := make(chan struct{}, 2)
	go func() { shared = 1; finished <- struct{}{} }()
	go func() { shared = 2; finished <- struct{}{} }()
	<-finished
	<-finished
	t.Log(shared)
}
