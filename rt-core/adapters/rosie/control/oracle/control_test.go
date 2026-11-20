//go:build linux

package oracle

import (
	"context"
	"errors"
	"fmt"
	adapter "rosieos/rt-core/adapters/rosie/control"
	. "rosieos/rt-core/sdk/control"
	"testing"
	"time"
)

// Origin: test_control.ControlTests.test_motion_server_sdk_executes_current_format.
func TestMotionServerSDKExecutesCurrentFormat(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	c := s.client
	ctx := context.Background()
	p, err := c.PrepareProgram(ctx, oracleProgram(t, s.root, false))
	must(t, err)
	_, err = c.Enable(ctx, p.AxisMask)
	must(t, err)
	_, err = c.Arm(ctx)
	must(t, err)
	_, err = c.StartProgram(ctx, p.Identity)
	must(t, err)
	s.waitStatus(t, func(v Status) bool { return v.Execution.State == "completed" })
	must(t, c.Close())
	observer, err := Dial(UnixPath(s.socket))
	must(t, err)
	defer observer.Close()
	poll(t, 4*time.Second, func() bool { v, err := observer.Status(ctx); return err == nil && v.Core.Armed == 0 })
}

// Origin: test_control.ControlTests.test_dense_program_native_execution_and_identity.
func TestDenseProgramNativeExecutionAndIdentity(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	c := s.client
	ctx := context.Background()
	d, err := c.Describe(ctx)
	must(t, err)
	if len(d.Axes) != 9 {
		t.Fatal("axis count")
	}
	for i, a := range d.Axes {
		if a.ID != fmt.Sprintf("J%d", i+1) {
			t.Fatal("axis identity")
		}
	}
	p, err := c.PrepareProgram(ctx, oracleProgram(t, s.root, false))
	must(t, err)
	if p.Samples != 11 || p.RequiresProcessIO {
		t.Fatalf("program: %+v", p)
	}
	state, err := c.Status(ctx)
	must(t, err)
	if state.Core.Armed != 0 {
		t.Fatal("prepare armed")
	}
	wrong := p.Identity
	wrong.PlanRevision++
	_, err = c.StartProgram(ctx, wrong)
	rejected(t, err, "program_identity_mismatch")
	s.enable(t)
	_, err = c.StartProgram(ctx, p.Identity)
	must(t, err)
	state = s.waitStatus(t, func(v Status) bool { return v.Execution.State == "completed" && v.Core.Axes[0].PositionCounts > 15 })
	if state.Core.Axes[0].PositionCounts <= 15 || state.Core.SafetyFaultMask != 0 {
		t.Fatalf("completion: %+v", state)
	}
}

// Origin: test_control.ControlTests.test_lease_expiry_fences_owner_and_inhibits_outputs.
func TestLeaseExpiryFencesOwnerAndInhibitsOutputs(t *testing.T) {
	s := startService(t)
	s.acquire(t, false)
	s.enable(t)
	s.waitStatus(t, func(v Status) bool { return v.Core.Armed == 0 })
	_, err := s.client.Arm(context.Background())
	rejected(t, err, "control_session_stale")
	_, err = s.client.Acquire(context.Background(), "motion-server", Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: s.hash})
	must(t, err)
}

// Origin: test_control.ControlTests.test_stop_fences_stale_commands_and_prepared_handles.
// R07: the old SDK serialized renewal and Stop; this port asserts the daemon-visible
// generation fence while independent SDK renewal remains active.
func TestStopFencesStaleCommandsAndPreparedHandles(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	c := s.client
	ctx := context.Background()
	p := make([]float64, 9)
	p[0] = .02
	h, err := c.PrepareTrajectory(ctx, 511, []Point{{Position: make([]float64, 9)}, {Time: 2 * time.Second, Position: p}})
	must(t, err)
	old := c.Fence()
	_, err = c.Stop(ctx)
	must(t, err)
	_, err = c.Command(ctx, &Request{Operation: "arm", Fence: old})
	rejected(t, err, "control_session_stale")
	s.enable(t)
	_, err = c.StartTrajectory(ctx, h.Handle)
	var refusal *Rejected
	if !errors.As(err, &refusal) {
		t.Fatalf("expected server rejection of stale prepared handle: %v", err)
	}
}

// Origin: test_control.ControlTests.test_process_markers_are_preserved_and_cannot_be_silently_ignored.
func TestProcessMarkersArePreservedAndCannotBeSilentlyIgnored(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	ctx := context.Background()
	blob := oracleProgram(t, s.root, true)
	description, err := s.client.Describe(ctx)
	must(t, err)
	ids := make([]string, len(description.Axes))
	for i, axis := range description.Axes {
		ids[i] = axis.ID
	}
	p, err := adapter.DecodeProgram(blob, description.NativeDescription, ids)
	must(t, err)
	if !p.RequiresProcessIO || len(p.Markers) != 3 || p.Markers[1].TimeNS != 50000000 || !p.Markers[1].Value {
		t.Fatal("process markers lost")
	}
	// P6-05 refuses torch during admission, before publishing a prepared handle.
	_, err = s.client.PrepareProgram(ctx, blob)
	rejected(t, err, "process_io_executor_not_qualified")
	v, err := s.client.Status(ctx)
	must(t, err)
	if v.Core.Armed != 0 {
		t.Fatal("unqualified process program armed")
	}
}
