//go:build linux

package denseexec

import (
	"bytes"
	"context"
	"strings"
	"testing"
	"time"

	codec "rosieos/rt-core/adapters/rosie/densejointtraj"
)

// Isolated native simulation proves real admission and Start identity echo.
// FetchBlob serves retained bytes directly: no planner process is involved.
func TestRTCoreDryRunComposed(t *testing.T) {
	c, _, _, _, cfg := rtComposedTransportFixture(t, false)
	ctx, cancel := context.WithTimeout(context.Background(), 30*time.Second)
	defer cancel()
	raw, _ := fixtureBlob(t)
	plan, err := codec.Decode(raw)
	if err != nil {
		t.Fatal(err)
	}
	for i := range plan.Segments {
		for j := range plan.Segments[i].Samples {
			sample := &plan.Segments[i].Samples[j]
			for a := range sample.QRad {
				sample.QRad[a], sample.QdRadS[a] = 0, 0
			}
			sample.TorchOn = plan.Segments[i].Kind == "weld"
		}
	}
	plan.Header.ProcessMarkers = []codec.ProcessMarker{{Segment: 0, Sample: 0, Action: "gas", Value: false}}
	source, err := codec.Encode(plan.Header, plan.Segments)
	if err != nil {
		t.Fatal(err)
	}
	unchanged := append([]byte(nil), source...)
	decoded, err := codec.Decode(source)
	if err != nil {
		t.Fatal(err)
	}
	h := decoded.Header
	id := Identity{PlanID: h.PlanID, ProgramID: h.ProgramID, ProgramDigest: h.ProgramDigest, TrajectoryDigest: h.TrajectoryDigest, ManifestRevision: h.ManifestRevision, PlanRevision: h.PlanRevision}
	reads := 0
	r, err := NewRTCore(cfg, Config{ControllerID: "offline-programming:dry-run-composed", FetchBlob: func(_ context.Context, digest string) ([]byte, error) {
		if digest != id.TrajectoryDigest {
			t.Errorf("fetched derived artifact instead of retained plan: %s", digest)
		}
		reads++
		return source, nil
	}}, c)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { r.Close(context.Background()) })
	// rt-core's codec writes no process_markers_sha256, so the production Load
	// stops at the OLP's schedule binding before any authority is taken.
	if _, err := r.Load(ctx, LoadRequest{Identity: id}); err == nil || !strings.Contains(err.Error(), "process_markers_mismatch") {
		t.Fatalf("unbound output schedule loaded: %v", err)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
	loaded, err := r.Load(ctx, LoadRequest{Identity: id, DryRun: true})
	if err != nil {
		t.Fatalf("derived motion-only prepare: %v", err)
	}
	session := loaded.Status.Session
	if !session.DryRun || session.TrajectoryDigest != id.TrajectoryDigest || session.ExecutionTrajectoryDigest == id.TrajectoryDigest || session.ExecutionNormalisedDigest == "" {
		t.Fatalf("source/execution identity %+v", session)
	}
	before, err := c.Status(ctx)
	if err != nil || before.Core.Armed != 0 || before.Core.AxisEnableMask != 0 {
		t.Fatalf("Load caused motion authority: %+v %v", before.Core, err)
	}
	if _, err := r.Arm(ctx, true); err != nil {
		t.Fatal(err)
	}
	if _, err := r.Play(ctx, loaded.SessionID); err != nil {
		t.Fatalf("derived Start identity: %v", err)
	}
	rtAwait(t, func() bool { _, _ = r.Heartbeat(loaded.SessionID); return r.Status(ctx).Session.State == "done" })
	actual, err := c.Status(ctx)
	if err != nil || actual.Execution.Identity.TrajectoryDigest != session.ExecutionTrajectoryDigest || actual.Execution.Identity.SourceDigest != session.ExecutionTrajectoryDigest || actual.Execution.Identity.NormalisedDigest != session.ExecutionNormalisedDigest {
		t.Fatalf("native executed unexpected identity: %+v %v", actual.Execution, err)
	}
	if reads != 2 || !bytes.Equal(source, unchanged) {
		t.Fatalf("source changed or unexpected fetches: %d", reads)
	}
	if _, err := r.Stop(ctx); err != nil {
		t.Fatal(err)
	}
}
