//go:build linux

package control

import (
	"testing"
	"time"
)

func TestHandleLifecycleTable(t *testing.T) {
	t.Run("transition guard", func(t *testing.T) {
		allowed := map[HandleState][]HandleState{
			"":             {HandlePrepared},
			HandlePrepared: {HandlePrepared, HandleStarted, HandleDiscarded, HandleSuperseded, HandleRetired},
			HandleStarted:  {HandleConsumed, HandleRetired},
		}
		states := []HandleState{"", HandlePrepared, HandleStarted, HandleConsumed, HandleDiscarded, HandleSuperseded, HandleRetired}
		for _, from := range states {
			for _, to := range states {
				want := false
				for _, permitted := range allowed[from] {
					want = want || to == permitted
				}
				h := preparation{HandleRecord: HandleRecord{State: from}}
				if got := transitionHandle(&h, to); got != want || (!got && h.State != from) {
					t.Fatalf("%q -> %q: %v, want %v; state=%s", from, to, got, want, h.State)
				}
			}
		}
	})
	t.Run("native lifecycle", func(t *testing.T) {
		f := newHandleFixture(t)
		a := serveHandleAPI(t, f.controller, nil)
		keepHandleGrant(t, a, f.fence)
		handle := prepareHandle(t, a, f.fence, 300*time.Millisecond)
		request := controlRequest(f.fence, "start_trajectory")
		request.Handle = handle
		a.call(t, request, "not_ready")
		if h := requireHandle(t, a, handle, HandlePrepared); h.Generation != 1 || h.ExecutionGeneration != 0 {
			t.Fatalf("rejected Start changed generations: %+v", h)
		}
		request.Generation++
		a.call(t, request, "control_session_stale")
		requireHandle(t, a, handle, HandlePrepared)
		request.Fence = f.fence
		enableHandles(t, a, f.fence)
		started := a.call(t, request, "")
		if h := requireHandle(t, a, handle, HandleStarted); h.ExecutionGeneration != 1 || h.NativeSequence != started.Sequence {
			t.Fatalf("accepted generation: %+v", h)
		}
		a.call(t, request, "handle_started")
		request.Operation = "discard_trajectory"
		a.call(t, request, "handle_active")
		awaitHandleCondition(t, func() bool {
			_, motion, _ := a.snapshot(t)
			return motion.CommandSequence == started.Sequence && motion.Done
		})
		requireHandle(t, a, handle, HandleConsumed)
		request.Operation = "start_trajectory"
		a.call(t, request, "handle_consumed")

		discarded := prepareHandle(t, a, f.fence, time.Second)
		request.Handle, request.Operation = discarded, "discard_trajectory"
		a.call(t, request, "")
		a.call(t, request, "")
		requireHandle(t, a, discarded, HandleDiscarded)
		request.Operation = "start_trajectory"
		a.call(t, request, "handle_discarded")
		request.Handle = 999999
		a.call(t, request, "unknown_handle")
		request.Operation = "discard_trajectory"
		a.call(t, request, "unknown_handle")

		first := prepareHandleProgram(t, a, f.fence, "first")
		firstIdentity := requireHandle(t, a, first, HandlePrepared).Identity
		second := prepareHandleProgram(t, a, f.fence, "second")
		secondPrepared := requireHandle(t, a, second, HandlePrepared)
		if firstIdentity.PlanID != "first" || secondPrepared.Identity.PlanID != "second" {
			t.Fatal("program preparations must retain their distinct bound identities")
		}
		requireHandle(t, a, first, HandleSuperseded)
		assertNativeUnknown(t, f.client, first)
		beforeRefusals := publicEvents(t, a.client, 0).LatestSequence
		request.Handle, request.Operation = first, "start_trajectory"
		// Compatibility tightening: program handles no longer inherit the empty
		// identity accepted by an unlabeled raw preparation. Identity is checked
		// before lifecycle state, including for superseded handles.
		a.call(t, request, "trajectory_identity_mismatch")
		request.Identity = firstIdentity
		a.call(t, request, "handle_superseded")
		request.Handle = second
		a.call(t, request, "trajectory_identity_mismatch") // first program's identity
		request.Identity = Identity{}
		a.call(t, request, "trajectory_identity_mismatch")
		if h := requireHandle(t, a, second, HandlePrepared); h != secondPrepared {
			t.Fatalf("identity refusal changed prepared program handle: got=%+v want=%+v", h, secondPrepared)
		}
		for _, event := range publicEvents(t, a.client, beforeRefusals).Events {
			if event.Execution != nil {
				t.Fatalf("identity/lifecycle refusal emitted execution: %+v", event)
			}
		}
		request.Identity = secondPrepared.Identity
		a.call(t, request, "")
		other := prepareHandle(t, a, f.fence, time.Second)
		request.Handle, request.Identity = other, Identity{} // unlabeled raw path remains compatible
		a.call(t, request, "mode_conflict")
		requireHandle(t, a, other, HandlePrepared)
		grant := grantFrom(t, a.call(t, controlRequest(f.fence, "stop"), ""))
		for _, id := range []uint64{other, second} {
			requireHandle(t, a, id, HandleRetired)
		}
		request.Fence = Fence{grant.Session, grant.Generation}
		a.call(t, request, "handle_retired")
		awaitHandleCondition(t, func() bool { state, _, _ := a.snapshot(t); return state.Armed == 0 && state.AxisEnableMask == 0 })
	})
	t.Run("grant expiry", func(t *testing.T) {
		f := newHandleFixture(t)
		a := serveHandleAPI(t, f.controller, nil)
		enableHandles(t, a, f.fence)
		active := prepareHandle(t, a, f.fence, 2*time.Second)
		prepared := prepareHandle(t, a, f.fence, 2*time.Second)
		request := controlRequest(f.fence, "start_trajectory")
		request.Handle = active
		a.call(t, request, "")
		awaitHandleCondition(t, func() bool {
			state, _, execution := a.snapshot(t)
			retired := 0
			for _, h := range execution.Handles {
				if h.State == HandleRetired {
					retired++
				}
			}
			return retired == 2 && state.Armed == 0 && state.AxisEnableMask == 0
		})
		for _, id := range []uint64{active, prepared} {
			requireHandle(t, a, id, HandleRetired)
		}
		a.call(t, controlRequest(f.fence, "renew"), "expired")
	})
	t.Run("connection loss", func(t *testing.T) {
		f := newHandleFixture(t)
		a := serveHandleAPI(t, f.controller, nil)
		id := prepareHandle(t, a, f.fence, time.Second)
		if err := f.client.Close(); err != nil {
			t.Fatal(err)
		}
		_, _, execution, err := f.controller.Snapshot()
		if err == nil || len(execution.Handles) != 1 || execution.Handles[0].Handle != id || execution.Handles[0].State != HandleRetired {
			t.Fatalf("lost daemon retained replayable handle: %+v %v", execution, err)
		}
	})
}
