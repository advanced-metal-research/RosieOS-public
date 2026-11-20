//go:build linux

package control

import (
	"context"
	"encoding/json"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
)

// A native reset that leaves a drive alarm faulted publishes one operator
// sentence; the adapter's refusal keeps fault_persists as its code and adds
// the sentence, while the public RecoveryStatus gains no field.
type driveAlarmResetNative struct {
	homeAuthorityNative
	message string
	resets  int
}

func (n *driveAlarmResetNative) FaultReset(context.Context, uint32) (uint64, error) {
	n.resets++
	return 10, nil
}
func (n *driveAlarmResetNative) Status() (ipcclient.Status, error) {
	state := n.state
	state.TimeNS = ipcclient.HostMonotonicNS()
	state.Recovery = &ipcclient.RecoveryStatus{ResetSequence: 10, Reason: ReasonFaultPersists, Message: n.message}
	return state, nil
}
func (n *driveAlarmResetNative) Motion() (ipcclient.MotionState, error) {
	return ipcclient.MotionState{TimeNS: ipcclient.HostMonotonicNS(), Done: true}, nil
}

func TestFaultResetRefusalNamesTheDriveAlarm(t *testing.T) {
	const sentence = "J5 drive alarm Er41.1 persists after reset: Motor over-temperature due to locked- rotor"
	for _, message := range []string{sentence, ""} {
		n := &driveAlarmResetNative{message: message}
		binding := Binding{PairID: "cell", Revision: 1, ConfigurationSHA256: "config"}
		c := &Controller{core: n, binding: binding, description: ipcclient.Description{MaxGrantLeaseNS: 500000000}, done: make(chan struct{})}
		grant, err := c.Acquire("operator", binding)
		if err != nil {
			t.Fatal(err)
		}
		sequence, _, recovery, err := c.FaultResetRecovery(Fence{grant.Session, grant.Generation}, 1<<4)
		c.cancel()
		want := ReasonFaultPersists
		if message != "" {
			want += ": " + message
		}
		if n.resets != 1 || sequence != 10 || err == nil || err.Error() != want {
			t.Fatalf("refusal %q, want %q (resets %d)", err, want, n.resets)
		}
		if recovery == nil || recovery.Message != message {
			t.Fatalf("recovery %+v", recovery)
		}
		outcome := encodeResponse(Response{Operation: OperationResetFault, Sequence: sequence, Data: recovery}, err)
		var public struct {
			Error string                     `json:"error"`
			Data  map[string]json.RawMessage `json:"data"`
		}
		if err := json.Unmarshal(outcome.body, &public); err != nil {
			t.Fatal(err)
		}
		if public.Error != want || public.Data["reason"] == nil {
			t.Fatalf("public reply %s", outcome.body)
		}
		if _, leaked := public.Data["message"]; leaked || strings.Contains(string(outcome.body), `"message"`) {
			t.Fatalf("native-only message entered the public RecoveryStatus: %s", outcome.body)
		}
	}
}
