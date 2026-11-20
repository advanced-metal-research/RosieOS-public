//go:build linux

package denseexec

import (
	"context"
	"testing"
	"time"

	"rosieos/rt-core/sdk/control"
)

// TestReviewArmRoundTripsComposed counts the actual adapter requests without
// background observation traffic; one native confirmation prevents redundant
// status reads from increasing the seven-stage remote ARM critical path.
func TestReviewArmRoundTripsComposed(t *testing.T) {
	c, d, _, socket, counts := delayedJointFixture(t, false)
	r, err := NewRTCore(RTCoreConfig{Socket: socket, Binding: control.Binding{PairID: "bench", Revision: 7, MachineSHA256: d.MachineSHA256}, DeploymentSHA256: d.DeploymentSHA256, CapabilitiesDigest: d.CapabilitiesDigest, AxisMask: 511}, Config{ControllerID: "offline-programming:round-trips"}, c)
	if err != nil {
		t.Fatal(err)
	}
	r.cancel()
	<-r.done
	t.Cleanup(func() { r.Close(context.Background()) })
	started := time.Now()
	result, err := r.Arm(context.Background(), true)
	elapsed := time.Since(started)
	if err != nil || result.Status.Armed == nil || !*result.Status.Armed {
		t.Fatalf("ARM err=%v", err)
	}
	counts.mu.Lock()
	requests := make(map[string]int)
	for _, request := range counts.requests {
		requests[request]++
	}
	counts.mu.Unlock()
	for path, want := range map[string]int{"/v1/describe": 7, "/v1/status": 2, "/v1/events": 1, "acquire": 1, "enable": 1, "arm": 1} {
		if requests[path] != want {
			t.Fatalf("%s count=%d want=%d all=%v", path, requests[path], want, requests)
		}
	}
	// Unverified acceptance ceiling: 2500 ms at the injected 250 ms RTT.
	if elapsed > 2500*time.Millisecond {
		t.Fatalf("ARM elapsed_ms=%d", elapsed.Milliseconds())
	}
	t.Logf("ARM serial_round_trips=7 native_confirmations=1 elapsed_ms=%d requests=%v", elapsed.Milliseconds(), requests)
}
