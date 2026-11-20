//go:build linux

package oracle

import (
	"context"
	"os"
	"path/filepath"
	"testing"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
)

// The default uses the shared format corpus. ROSIE_TEST_DENSE_EXPORT_DIR can
// additionally prove the Python exporter's paired production.rdt/dry_run.rdt
// against this same real adapter/native-simulation admission boundary.
func TestDryRunPreparesWithoutProcessIOOrMotion(t *testing.T) {
	s := startService(t)
	s.acquire(t, true)
	production, dry := oracleProgram(t, s.root, true), oracleProgram(t, s.root, false)
	if dir := os.Getenv("ROSIE_TEST_DENSE_EXPORT_DIR"); dir != "" {
		var err error
		production, err = os.ReadFile(filepath.Join(dir, "production.rdt"))
		must(t, err)
		dry, err = os.ReadFile(filepath.Join(dir, "dry_run.rdt"))
		must(t, err)
	}
	weldPlan, err := densejointtraj.Decode(production)
	must(t, err)
	dryPlan, err := densejointtraj.Decode(dry)
	must(t, err)
	if weldPlan.Header.TrajectoryDigest == dryPlan.Header.TrajectoryDigest || len(weldPlan.Segments) != len(dryPlan.Segments) {
		t.Fatal("dry run must retain the route under a distinct content identity")
	}
	for i, segment := range weldPlan.Segments {
		other := dryPlan.Segments[i]
		if segment.Kind != other.Kind || segment.SourceID != other.SourceID || len(segment.Samples) != len(other.Samples) {
			t.Fatal("dry run changed segment structure")
		}
		for j, sample := range segment.Samples {
			motion := other.Samples[j]
			if motion.TorchOn || sample.TimeS != motion.TimeS || sample.QRad != motion.QRad || sample.QdRadS != motion.QdRadS {
				t.Fatalf("dry run changed motion or retained torch at segment %d sample %d", i, j)
			}
		}
	}
	ctx := context.Background()
	before, err := s.client.Status(ctx)
	must(t, err)
	_, err = s.client.PrepareProgram(ctx, production)
	rejected(t, err, "process_io_executor_not_qualified")
	prepared, err := s.client.PrepareProgram(ctx, dry)
	must(t, err)
	if prepared.RequiresProcessIO || prepared.Identity.TrajectoryDigest != dryPlan.Header.TrajectoryDigest {
		t.Fatal("prepared program lost dry-run identity or retained process I/O")
	}
	for _, marker := range prepared.Markers {
		if marker.Value || marker.Action != "torch" {
			t.Fatalf("dry run retained an output intent: %+v", marker)
		}
	}
	after, err := s.client.Status(ctx)
	must(t, err)
	if before.Core.Armed != 0 || after.Core.Armed != 0 || before.Core.AxisEnableMask != 0 || after.Core.AxisEnableMask != 0 {
		t.Fatal("preparing a dry run enabled outputs")
	}
	for i, axis := range before.Axes {
		if after.Axes[i].PositionCounts != axis.PositionCounts {
			t.Fatal("preparing a dry run moved an axis")
		}
	}
}
