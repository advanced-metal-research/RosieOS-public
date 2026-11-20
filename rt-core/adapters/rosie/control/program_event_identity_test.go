//go:build linux

package control

import "testing"

func TestProgramHandleAndEventsRetainPreparedIdentity(t *testing.T) {
	f := startLifecycleCore(t)
	api := serveHandleAPI(t, f.controller, nil)
	program := mustPrepareProgram(t, f, "keyframes-event-identity")
	status := publicEventStatus(t, api.client)
	var handle uint64
	for _, h := range status.Execution.Handles {
		if h.State == HandlePrepared {
			handle = h.Handle
			if h.Identity != program.Identity {
				t.Errorf("prepared handle lost program identity: got=%+v want=%+v", h.Identity, program.Identity)
			}
		}
	}
	if handle == 0 {
		t.Fatal("no prepared native handle")
	}
	preparedHandle := requireHandle(t, api, handle, HandlePrepared)
	// Digest mismatch refusals already have coverage in program_test.go. These
	// metadata-only changes must also leave the exact preparation untouched and
	// emit no execution event, even though every content digest still matches.
	for _, test := range []struct {
		name   string
		change func(*Identity)
	}{
		{"plan_id_only", func(identity *Identity) { identity.PlanID += "-other" }},
		{"program_id_only", func(identity *Identity) { identity.ProgramID += "-other" }},
		{"plan_revision_only", func(identity *Identity) { identity.PlanRevision++ }},
		{"manifest_revision_only", func(identity *Identity) { identity.ManifestRevision++ }},
	} {
		t.Run(test.name, func(t *testing.T) {
			before := publicEventStatus(t, api.client)
			after := publicEvents(t, api.client, 0).LatestSequence
			request := controlRequest(f.fence, "start_program")
			request.Identity = program.Identity
			test.change(&request.Identity)
			if response := api.call(t, request, "program_identity_mismatch"); response.Sequence != 0 {
				t.Fatalf("identity refusal submitted a native command: %+v", response)
			}
			if got := requireHandle(t, api, handle, HandlePrepared); got != preparedHandle {
				t.Fatalf("identity refusal changed prepared handle: got=%+v want=%+v", got, preparedHandle)
			}
			observed := publicEventStatus(t, api.client)
			if observed.Execution.Generation != before.Execution.Generation || observed.Motion.CommandSequence != before.Motion.CommandSequence {
				t.Fatalf("identity refusal advanced execution: before=%+v after=%+v", before.Execution, observed.Execution)
			}
			for _, event := range publicEvents(t, api.client, after).Events {
				if event.Execution != nil {
					t.Fatalf("identity refusal emitted execution: %+v", event)
				}
			}
		})
	}
	request := controlRequest(f.fence, "start_program")
	request.Identity = program.Identity
	started := api.call(t, request, "")
	seen := map[string]bool{}
	var after uint64
	awaitHandleCondition(t, func() bool {
		_ = publicEventStatus(t, api.client)
		batch := publicEvents(t, api.client, after)
		after = batch.NextSequence
		for _, event := range batch.Events {
			if event.Handle != nil && event.Handle.Handle == handle && event.Handle.Identity != program.Identity {
				t.Errorf("handle transition lost program identity: %+v", event.Handle)
			}
			if event.Type != "execution_started" && event.Type != "execution_completed" {
				continue
			}
			if event.Execution == nil || event.Execution.Handle != handle || event.Execution.NativeSequence != started.Sequence || event.Execution.Identity != program.Identity {
				t.Errorf("%s does not retain prepared program identity: %+v", event.Type, event.Execution)
			}
			seen[event.Type] = true
		}
		return seen["execution_started"] && seen["execution_completed"]
	})
}
