//go:build linux

package control

import (
	"encoding/json"
	"testing"
	"time"
)

// Raw receipts must echo the immutable caller identity and execution generation;
// a handle from another request must never authorize a native Start.
func TestRawTrajectoryIdentityEcho(t *testing.T) {
	f := startPublicReset(t, "simulation-program.json", 511)
	f.ready(t, 511)
	identity := Identity{PlanID: "source-run", ProgramID: "immutable-command", PlanRevision: 7}
	// Unverified fixture knot interval: 300,000,000 ns; endpoints remain held.
	prepared := f.command(t, Request{Operation: "prepare_trajectory", AxisMask: 511, Points: handlePoints(300 * time.Millisecond), Identity: identity}, "")
	var receipt struct {
		Handle, Generation, ExecutionGeneration, NativeSequence uint64
		Identity                                                Identity
	}
	// Decode wire names explicitly while retaining compatibility with the old
	// schema so this regression reproduces its missing echo before regeneration.
	decode := func(raw json.RawMessage) {
		t.Helper()
		var wire struct {
			Handle              uint64   `json:"handle"`
			Generation          uint64   `json:"generation"`
			ExecutionGeneration uint64   `json:"execution_generation"`
			NativeSequence      uint64   `json:"native_sequence"`
			Identity            Identity `json:"identity"`
		}
		if err := json.Unmarshal(raw, &wire); err != nil {
			t.Fatal(err)
		}
		receipt.Handle, receipt.Generation, receipt.ExecutionGeneration, receipt.NativeSequence, receipt.Identity = wire.Handle, wire.Generation, wire.ExecutionGeneration, wire.NativeSequence, wire.Identity
	}
	decode(prepared.Data)
	if receipt.Handle != prepared.Handle || receipt.Generation == 0 || receipt.ExecutionGeneration != 0 || receipt.Identity != identity {
		t.Fatalf("prepare identity: %+v", receipt)
	}
	wrong := identity
	wrong.ProgramID = "recycled-command"
	f.command(t, Request{Operation: "start_trajectory", Handle: prepared.Handle, Identity: wrong}, "trajectory_identity_mismatch")
	f.command(t, Request{Operation: "start_trajectory", Handle: prepared.Handle}, "trajectory_identity_mismatch")
	started := f.command(t, Request{Operation: "start_trajectory", Handle: prepared.Handle, Identity: identity}, "")
	decode(started.Data)
	if receipt.Handle != prepared.Handle || receipt.ExecutionGeneration != 1 || receipt.NativeSequence != started.Sequence || receipt.Identity != identity {
		t.Fatalf("start identity: %+v", receipt)
	}
	var terminal Event
	awaitHandleCondition(t, func() bool {
		for _, event := range publicEvents(t, f.client, 0).Events {
			if event.Type == "execution_completed" && event.Execution.Handle == prepared.Handle {
				terminal = event
				return true
			}
		}
		return false
	})
	encoded, err := json.Marshal(terminal.Execution)
	if err != nil {
		t.Fatal(err)
	}
	var completion struct {
		Identity       Identity `json:"identity"`
		Generation     uint64   `json:"generation"`
		NativeSequence uint64   `json:"native_sequence"`
	}
	if err := json.Unmarshal(encoded, &completion); err != nil {
		t.Fatal(err)
	}
	if completion.Identity != identity || completion.Generation != 1 || completion.NativeSequence != started.Sequence {
		t.Fatalf("terminal identity: %+v", completion)
	}
}
