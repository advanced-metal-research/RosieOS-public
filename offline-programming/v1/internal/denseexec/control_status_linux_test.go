//go:build linux

package denseexec

import (
	"encoding/json"
	"os"
	"reflect"
	"testing"

	"rosieos/rt-core/sdk/control"
)

func recordedControlStatus(t testing.TB) *StatusBody {
	t.Helper()
	raw, err := os.ReadFile("../../../../steamdeck/real/v5/tests/fixtures/example-live/status.json")
	if err != nil {
		t.Fatal(err)
	}
	var body StatusBody
	if err := json.Unmarshal(raw, &body); err != nil {
		t.Fatal(err)
	}
	return &body
}

func statusJSON(t testing.TB, body *StatusBody) []byte {
	t.Helper()
	raw, err := json.Marshal(body)
	if err != nil {
		t.Fatal(err)
	}
	return raw
}

func TestControlStatusBoundedHistoryPreservesCurrentEvidence(t *testing.T) {
	for _, axes := range []int{6, 9} {
		body := recordedControlStatus(t)
		for len(body.RTCore.Status.Axes) < axes {
			body.RTCore.Status.Axes = append(body.RTCore.Status.Axes, body.RTCore.Status.Axes[0])
			body.RTCore.Status.Core.Axes = append(body.RTCore.Status.Core.Axes, body.RTCore.Status.Core.Axes[0])
			body.RTCore.Description.Axes = append(body.RTCore.Description.Axes, body.RTCore.Description.Axes[0])
		}
		body.Loop = &LoopStatus{Mode: "loop", Cycles: 275, State: "running", Attended: false, Owner: "deck"}
		body.RTCore.Status.Core.SafetyFaultMask = 4
		body.RTCore.Status.Core.ExecutionFaultReasons = 64
		body.RTCore.Status.Core.Axes[2].ErrorCode = 0x1234
		body.RTCore.Refusal, body.RTCore.RefusalKind, body.RTCore.RefusalAfterNS = "fault_persists", "reset", 123456
		body.RTCore.Status.Motion.TrajectoryID = 20001
		body.RTCore.Status.Motion.Done = true
		body.RTCore.Status.Execution.State = "faulted"
		body.RTCore.Status.Execution.Error = "coordinate_reference"
		if err := json.Unmarshal([]byte(`{"time_ns":123456,"reason":7,"inputs":[{"name":"sensor","value":true,"valid":false}],"outputs":[{"name":"lamp","commanded":true,"readback":false,"valid":true,"expiry_ns":234567}]}`), &body.RTCore.Status.Core.IO); err != nil {
			t.Fatal(err)
		}
		current := control.HandleRecord{Handle: 20001, State: "retired", NativeSequence: 987}
		prepared := control.HandleRecord{Handle: 20002, State: "prepared"}
		body.RTCore.Status.Execution.Handles = []control.HandleRecord{current, prepared}
		baseline := statusJSON(t, ControlStatusBody(body))
		for i := uint64(1); i <= 10000; i++ {
			body.RTCore.Status.Execution.Handles = append(body.RTCore.Status.Execution.Handles, control.HandleRecord{Handle: i, State: "consumed", NativeSequence: i})
		}
		// Old prepared slots must not allow the control response to grow either.
		body.RTCore.Status.Execution.Handles = append(body.RTCore.Status.Execution.Handles, control.HandleRecord{Handle: 19000, State: "prepared"})
		before := statusJSON(t, body)
		view := ControlStatusBody(body)
		if got := statusJSON(t, view); string(got) != string(baseline) {
			t.Fatalf("%d-axis compact response changed with history: %d -> %d bytes", axes, len(baseline), len(got))
		}
		if after := statusJSON(t, body); string(before) != string(after) {
			t.Fatal("compact view mutated the source observation")
		}
		// Restore only history, then compare everything else. This catches a
		// dropped fault, raw diagnostic, I/O bit, generation or identity field.
		view.RTCore.Status.Execution.Handles = body.RTCore.Status.Execution.Handles
		if !reflect.DeepEqual(view, body) {
			t.Fatal("control projection changed current observation fields")
		}
		t.Logf("%d axes: full=%d bytes, compact=%d bytes after 10,000 completed handles", axes, len(before), len(baseline))
	}
}

func TestControlStatusCurrentHandleAndUnavailable(t *testing.T) {
	for _, body := range []*StatusBody{nil, {Available: false}, {RTCore: &RTCoreObservation{Reason: "transport_lost"}}} {
		if got := ControlStatusBody(body); !reflect.DeepEqual(got, body) {
			t.Fatal("unavailable status changed")
		}
	}
	for _, record := range []control.HandleRecord{{Handle: 1, State: "prepared"}, {Handle: 1, State: "started"}, {Handle: 1, State: "consumed"}, {Handle: 1, State: "retired"}, {Handle: 1, State: "discarded"}, {Handle: 1, State: "superseded"}} {
		body := recordedControlStatus(t)
		body.RTCore.Status.Motion.TrajectoryID = 1
		body.RTCore.Status.Execution.Handles = []control.HandleRecord{record}
		view := ControlStatusBody(body)
		if len(view.RTCore.Status.Execution.Handles) != 1 || !reflect.DeepEqual(view.RTCore.Status.Execution.Handles, body.RTCore.Status.Execution.Handles) {
			t.Fatalf("current %s handle not preserved exactly once", record.State)
		}
	}
}

func BenchmarkControlStatusProjection(b *testing.B) {
	body := recordedControlStatus(b)
	for i := uint64(1); i <= 10000; i++ {
		body.RTCore.Status.Execution.Handles = append(body.RTCore.Status.Execution.Handles, control.HandleRecord{Handle: i, State: "consumed"})
	}
	b.ReportAllocs()
	b.ResetTimer()
	for b.Loop() {
		ControlStatusBody(body)
	}
}
